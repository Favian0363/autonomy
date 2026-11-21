//! In-process simulation session: scene ownership, physics, and mission frames.
const std = @import("std");
const Auv = @import("../Auv.zig");
const math = @import("../math.zig");
const Scene = @import("Scene.zig");
const database = @import("database.zig");
const types = @import("types.zig");
const S = @import("support.zig");
const MissionConfig = @import("../MissionConfig.zig");
const MujocoBackend = @import("mujoco.zig").MujocoBackend;
const forces = @import("forces.zig");
const Sensors = @import("sensors.zig");
const abi = @import("abi.zig");
const reflection = @import("ReflectionWriter.zig");

pub const Session = @This();

const frame_ticks = 8;

io: std.Io,
catalogs_owned: bool,
robot_catalog: abi.RobotCatalog,
robot_slots: []u32,
scene_catalog: abi.SceneCatalog,
scene_file: []align(@alignOf(u64)) u8 = &.{},
scene_presets: []database.ScenePreset,
scene: Scene,
backend: MujocoBackend,
sensors: Sensors,
wrenches: []abi.Wrench,
fluid: []abi.Wrench,
previous_linear: []abi.Vec3,
previous_angular: []abi.Vec3,
scratch: []align(@alignOf(f64)) u8,
thruster_len: u8,
rotors: [types.thruster_count]abi.RotorState = @splat(abi.RotorState{ .omega = 0, .integral = 0, .torque = 0, .thrust = 0 }),
preset_robot: []const u8 = "",
preset_scene: []const u8 = "",
commands: types.ThrusterCommands = @splat(0),
tick: u64 = 0,
realtime: bool,
tam_len: usize,
failed: bool = false,
live: bool = false,
started: std.Io.Clock.Timestamp,

pub fn init(self: *Session, allocator: std.mem.Allocator, io: std.Io, config: *const MissionConfig.MissionConfig, tam_len: usize, realtime: bool) !void {
    const robot_name = try config.sim_robot.slice();
    const scene_name = try config.sim_scene.slice();
    if (tam_len == 0 or tam_len > MissionConfig.thruster_count) return error.InvalidThrusterCount;
    self.* = .{
        .io = io,
        .catalogs_owned = true,
        .robot_catalog = .{},
        .robot_slots = undefined,
        .scene_catalog = .{},
        .scene_presets = undefined,
        .scene = undefined,
        .backend = undefined,
        .sensors = undefined,
        .wrenches = undefined,
        .fluid = undefined,
        .previous_linear = undefined,
        .previous_angular = undefined,
        .scratch = undefined,
        .thruster_len = 0,
        .realtime = realtime,
        .tam_len = tam_len,
        .started = undefined,
    };
    self.robot_catalog = try database.loadRobot(allocator, database.alignedEmbed(@embedFile("robots.bin")));
    self.robot_slots = try allocator.alloc(u32, self.robot_catalog.names.len);
    for (self.robot_catalog.names, 0..) |name, i| {
        if (name.entity_count != 1) return error.InvalidDatabase;
        self.robot_slots[i] = name.entity_start;
    }
    switch (config.sim_scene_catalog) {
        .embedded_bin => {
            self.scene_catalog = try database.loadScene(allocator, database.alignedEmbed(@embedFile("scenes.bin")));
        },
        .file => {
            const path = try config.sim_scene_catalog_path.slice();
            if (path.len == 0) return error.MissingSceneCatalogPath;
            const bytes = try std.Io.Dir.cwd().readFileAllocOptions(io, path, allocator, .limited(std.math.maxInt(usize)), .of(u64), null);
            self.scene_catalog = database.loadScene(allocator, bytes) catch |err| {
                allocator.free(bytes);
                return err;
            };
            if (reflection.slicesMapped(abi.SceneCatalog, self.scene_catalog, bytes)) {
                self.scene_file = bytes;
            } else allocator.free(bytes);
        },
    }
    self.scene_presets = try database.allocScenePresets(allocator, &self.scene_catalog);
    const robot_index: usize = blk: {
        for (self.robot_slots, 0..) |slot, i| {
            if (std.mem.eql(u8, robot_name, database.rna.string(self.robot_catalog.strings, self.robot_catalog.entities[slot]))) break :blk i;
        }
        return error.UnknownSimRobot;
    };
    const robot_slot = self.robot_slots[robot_index];
    const robot = database.bodySlot(&self.robot_catalog, robot_slot);
    if (database.rna.owned(self.robot_catalog.thrusters, robot.slot) != tam_len) return error.InvalidThrusterCount;
    const selected_scene = if (std.mem.eql(u8, scene_name, "robot_only"))
        try Scene.init(allocator, &self.robot_catalog, robot, null, &.{}, .{}, &.{})
    else blk: {
        const preset: *const database.ScenePreset = find_preset: {
            for (self.scene_presets) |*p| if (std.mem.eql(u8, scene_name, p.name)) break :find_preset p;
            return error.UnknownSimScene;
        };
        const start: ?types.Pose = if (std.mem.eql(u8, scene_name, "robosub_course"))
            .{ .pos = .{ 0, 2, 2 }, .quat = .{ 0, 0, 1, 0 } }
        else
            null;
        try database.validateEntitySlots(&self.scene_catalog, preset.slots);
        const extras = try allocator.alloc(database.BodySlot, preset.slots.len);
        for (preset.slots, extras) |slot, *body| body.* = database.bodySlot(&self.scene_catalog, slot);
        break :blk try Scene.init(allocator, &self.robot_catalog, robot, start, extras, .{}, &.{});
    };
    self.scene = selected_scene;
    if (tam_len != self.scene.thruster_len) return error.InvalidThrusterCount;
    self.preset_robot = try allocator.dupe(u8, robot_name);
    self.preset_scene = try allocator.dupe(u8, scene_name);
    const scene = &self.scene;
    const physics_allocator = scene.allocator;
    self.backend = try MujocoBackend.init(physics_allocator, scene);
    errdefer self.backend.deinit();
    self.sensors = try Sensors.init(physics_allocator, scene);
    errdefer {
        self.sensors.deinit(physics_allocator);
        self.backend.deinit();
    }
    self.thruster_len = scene.thruster_len;
    const count = scene.objects.len;
    self.wrenches = try physics_allocator.alloc(abi.Wrench, count);
    self.fluid = try physics_allocator.alloc(abi.Wrench, count);
    self.previous_linear = try physics_allocator.alloc(abi.Vec3, count);
    self.previous_angular = try physics_allocator.alloc(abi.Vec3, count);
    // Hydrodynamics uses two temporary f64 extents per body.
    self.scratch = try physics_allocator.alignedAlloc(u8, .of(f64), 2 * count * @sizeOf(f64));
    try self.initializeTick();
    self.started = .now(self.io, .awake);
    self.live = true;
}

/// Releases MuJoCo handles only; Zig allocations live until the mission arena resets.
pub fn deinit(self: *Session) void {
    if (!self.live) {
        self.* = undefined;
        return;
    }
    const allocator = self.scene.allocator;
    if (self.scene_file.len != 0) allocator.free(self.scene_file);
    self.sensors.deinit(allocator);
    self.backend.deinit();
    self.live = false;
    self.* = undefined;
}

pub fn setThrusters(self: *Session, values: *const types.ThrusterCommandInput) !void {
    for (values[0..self.tam_len]) |v| if (!std.math.isFinite(v) or v < -1 or v > 1) return error.InvalidThrust;
    for (values[0..self.tam_len], 0..) |v, i| self.commands[i] = v;
}

pub fn timestamp(self: *const Session) u64 {
    return @intCast(@as(u128, self.tick) * self.scene.settings.dt_numerator * 1_000_000_000 / self.scene.settings.dt_denominator);
}

pub fn reset(self: *Session, states: ?[]const abi.BodyState) !void {
    try self.backend.reset(states);
    try self.initializeTick();
}

fn framePose(p: types.Pose) math.Pose {
    return .{ .pos = p.pos, .quat = p.quat };
}

fn initializeTick(self: *Session) !void {
    self.tick = 0;
    self.rotors = @splat(abi.RotorState{ .omega = 0, .integral = 0, .torque = 0, .thrust = 0 });
    self.commands = @splat(0);
    const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
    @memset(self.fluid, zero_wrench);
    @memset(self.wrenches, zero_wrench);
    try self.validateReadback();
    self.saveVelocities();
    self.refreshSensors();
}

fn validateReadback(self: *const Session) !void {
    for (self.backend.states, self.scene.objects) |body, object| {
        if (body.id != database.entity(object).id or !S.finite3(S.fromAbi3(body.linear_vel)) or !S.finite3(S.fromAbi3(body.angular_vel)) or
            !S.finite3(S.fromAbi3(body.com))) return error.InvalidBodyState;
        try S.validPose(S.bodyPose(body));
    }
    for (self.backend.contacts.items) |contact| {
        if (!S.finite3(S.fromAbi3(contact.position)) or !S.finite3(S.fromAbi3(contact.normal)) or
            !std.math.isFinite(contact.normal_force)) return error.InvalidContact;
    }
}

fn saveVelocities(self: *Session) void {
    for (self.backend.states, self.previous_linear, self.previous_angular) |body, *linear, *angular| {
        linear.* = body.linear_vel;
        angular.* = body.angular_vel;
    }
}

pub fn refreshSensors(self: *Session) void {
    Sensors.sample(self);
}

/// The detection slice is valid until the next step, reset, or deinit.
pub fn observation(self: *const Session) types.Observation {
    return self.sensors.observation;
}

pub fn bodyStates(self: *const Session) []const abi.BodyState {
    return self.backend.states;
}

pub fn nextFrame(self: *Session, out: *Auv.Frame) !void {
    errdefer self.failed = true;
    for (0..frame_ticks) |_| {
        if (self.failed) return error.SimulationFailed;
        if (self.tick == std.math.maxInt(u64)) return error.TickOverflow;
        const effective = self.commands[0..self.thruster_len];
        for (self.scene.faults) |fault| {
            if (!fault.active(self.tick)) continue;
            switch (fault.effect) {
                .dead => |channel| effective[channel] *= 0,
                .weak => |weak| effective[weak.thruster] *= weak.scale,
                else => {},
            }
        }
        const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
        for (self.wrenches, self.scene.objects) |*wrench, slot| {
            wrench.* = zero_wrench;
            wrench.id = database.entity(slot).id;
        }
        self.saveVelocities();
        for (self.scene.forces) |force| forces.evaluate(force, self) catch |err| {
            std.log.warn("simulator step failed: {s}", .{@errorName(err)});
            self.failed = true;
            return error.SimulationFailed;
        };
        self.backend.step(self.wrenches) catch |err| {
            std.log.warn("simulator step failed: {s}", .{@errorName(err)});
            self.failed = true;
            return error.SimulationFailed;
        };
        self.tick += 1;
        self.validateReadback() catch |err| {
            std.log.warn("simulator step failed: {s}", .{@errorName(err)});
            self.failed = true;
            return error.SimulationFailed;
        };
        const bounds = self.scene.settings.bounds;
        for (self.backend.states) |body| {
            if (S.norm(S.fromAbi3(body.pose_pos)) > bounds.position or
                S.norm(S.fromAbi3(body.linear_vel)) > bounds.speed or
                S.norm(S.fromAbi3(body.angular_vel)) > bounds.angular_speed)
            {
                std.log.warn("simulator step failed: {s}", .{@errorName(error.SimulationBoundsExceeded)});
                self.failed = true;
                return error.SimulationFailed;
            }
        }
        self.refreshSensors();
    }
    if (self.realtime) {
        const elapsed = self.started.untilNow(self.io).raw.toNanoseconds();
        const delay = @as(i96, @intCast(self.timestamp())) - elapsed;
        if (delay > 0) try std.Io.sleep(self.io, .fromNanoseconds(delay), .awake);
    }
    const obs = self.observation();
    out.* = std.mem.zeroes(Auv.Frame);
    out.timestamp = self.timestamp();
    out.camera_pose = framePose(obs.body);
    out.position_ok = .{ .x = obs.position_axes & 1 != 0, .y = obs.position_axes & 2 != 0, .z = obs.position_axes & 4 != 0 };
    out.attitude_ok = .{ .x = obs.attitude_axes & 1 != 0, .y = obs.attitude_axes & 2 != 0, .z = obs.attitude_axes & 4 != 0 };
    out.imu_quat = obs.imu;
    out.accel = obs.accel;
    out.gyro = obs.gyro;
    out.pressure_depth = @floatCast(obs.depth);
    out.pressure_depth_ok = obs.depth_ok;
    out.image_width = obs.image_width;
    out.image_height = obs.image_height;
    for (obs.objects[0..@min(obs.objects.len, Auv.Frame.max_objects)], 0..) |object, i| {
        if (object.cls > 255) return error.InvalidObjectClass;
        out.objects[i] = .{ .bbox = .{ .pose = framePose(object.pose), .size = object.size }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects2d[i] = .{ .top_left = .{ .x = object.x0, .y = object.y0 }, .bottom_right = .{ .x = object.x1, .y = object.y1 }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects_len += 1;
        out.objects2d_len += 1;
    }
}

pub fn snapshot(self: *const Session, allocator: std.mem.Allocator) ![]align(@alignOf(u64)) u8 {
    if (self.backend.contacts.items.len > abi.max_snapshot_contacts) return error.TooManyContacts;
    if (self.preset_robot.len > std.math.maxInt(u8) or self.preset_scene.len > std.math.maxInt(u8)) return error.NameTooLong;

    const robot_bytes = if (self.catalogs_owned)
        try database.save(allocator, self.robot_catalog)
    else
        try allocator.alignedAlloc(u8, .of(u64), 0);
    defer allocator.free(robot_bytes);
    const scene_bytes = if (self.catalogs_owned)
        try database.save(allocator, self.scene_catalog)
    else
        try allocator.alignedAlloc(u8, .of(u64), 0);
    defer allocator.free(scene_bytes);

    const strings_buf = try allocator.alloc(u8, self.preset_robot.len + self.preset_scene.len);
    defer allocator.free(strings_buf);
    @memcpy(strings_buf[0..self.preset_robot.len], self.preset_robot);
    @memcpy(strings_buf[self.preset_robot.len..], self.preset_scene);

    var rotors = self.rotors;
    const snap = abi.SessionStateCatalog{
        .strings = strings_buf,
        .meta = .{
            .tick = self.tick,
            .thruster_len = self.thruster_len,
            .robot_name_len = @intCast(self.preset_robot.len),
            .scene_name_len = @intCast(self.preset_scene.len),
            .pad0 = .{ 0, 0, 0, 0, 0 },
            .commands = self.commands,
        },
        .robot_catalog = robot_bytes,
        .scene_catalog = scene_bytes,
        .body_states = self.backend.states,
        .wrenches = self.wrenches,
        .fluid = self.fluid,
        .previous_linear = self.previous_linear,
        .previous_angular = self.previous_angular,
        .rotors = &rotors,
        .contacts = self.backend.contacts.items,
    };
    return reflection.write(allocator, snap);
}

pub fn restoreSnapshot(self: *Session, allocator: std.mem.Allocator, bytes: []const u8) !void {
    var snap = try reflection.read(abi.SessionStateCatalog, allocator, bytes);
    defer reflection.free(abi.SessionStateCatalog, allocator, &snap, bytes);
    const meta = snap.meta;
    if (snap.strings.len != meta.robot_name_len + meta.scene_name_len) return error.InvalidSession;
    if (!std.mem.eql(u8, snap.strings[0..meta.robot_name_len], self.preset_robot) or
        !std.mem.eql(u8, snap.strings[meta.robot_name_len..][0..meta.scene_name_len], self.preset_scene))
        return error.SnapshotMismatch;

    if (snap.body_states.len != self.backend.states.len or
        snap.wrenches.len != self.wrenches.len or
        snap.fluid.len != self.fluid.len or
        snap.previous_linear.len != self.previous_linear.len or
        snap.previous_angular.len != self.previous_angular.len)
        return error.SnapshotMismatch;

    try self.backend.reset(snap.body_states);
    @memcpy(self.wrenches, snap.wrenches);
    @memcpy(self.fluid, snap.fluid);
    @memcpy(self.previous_linear, snap.previous_linear);
    @memcpy(self.previous_angular, snap.previous_angular);
    const rotor_count = @min(snap.rotors.len, self.rotors.len);
    @memcpy(self.rotors[0..rotor_count], snap.rotors[0..rotor_count]);
    self.tick = meta.tick;
    self.commands = meta.commands;
    self.thruster_len = meta.thruster_len;
    self.backend.contacts.clearRetainingCapacity();
    try self.backend.contacts.appendSlice(self.scene.allocator, snap.contacts);
    self.refreshSensors();
}
