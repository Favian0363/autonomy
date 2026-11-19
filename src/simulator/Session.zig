//! In-process simulation session: scene ownership, physics, and mission frames.
const std = @import("std");
const Auv = @import("../Auv.zig");
const Scene = @import("Scene.zig");
const RobotCatalog = @import("robots.zig").RobotCatalog;
const SceneCatalog = @import("scenes.zig").SceneCatalog;
const database = @import("database.zig");
const types = @import("types.zig");
const S = @import("support.zig");
const MissionConfig = @import("../MissionConfig.zig");
const MujocoBackend = @import("mujoco.zig").MujocoBackend;
const forces = @import("forces.zig");
const Sensors = @import("sensors.zig");

pub const Session = @This();

const frame_ticks = 6;

io: std.Io,
catalogs_owned: bool,
catalog: RobotCatalog,
scene_catalog: SceneCatalog,
scene: Scene,
backend: MujocoBackend,
sensors: Sensors,
wrenches: []types.Wrench,
fluid: []types.Wrench,
previous_linear: []types.Vec3,
previous_angular: []types.Vec3,
scratch: []align(@alignOf(f64)) u8,
thruster_len: u8,
rotors: [types.thruster_count]types.RotorState = @splat(.{}),
commands: types.ThrusterCommands = @splat(0),
tick: u64 = 0,
realtime: bool,
tam_len: usize,
failed: bool = false,
started: std.Io.Clock.Timestamp,

pub fn init(self: *Session, allocator: std.mem.Allocator, io: std.Io, robot_name: []const u8, scene_name: []const u8, tam_len: usize, realtime: bool) !void {
    if (tam_len == 0 or tam_len > MissionConfig.thruster_count) return error.InvalidThrusterCount;
    self.* = .{
        .io = io,
        .catalogs_owned = true,
        .catalog = undefined,
        .scene_catalog = undefined,
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
    self.catalog = try RobotCatalog.init(allocator);
    errdefer self.catalog.deinit();
    self.scene_catalog = try SceneCatalog.init(allocator);
    errdefer self.scene_catalog.deinit();
    const robot_index = self.catalog.index(robot_name) orelse return error.UnknownSimRobot;
    const robot = self.catalog.body(robot_index);
    if (database.ownedCount(robot.db.thrusters.items, robot.slot) != tam_len) return error.InvalidThrusterCount;
    const robot_slots = self.catalog.slots[robot_index..][0..1];

    const robot_membership = [_]database.Membership{
        .{ .db = &self.catalog.db, .slots = robot_slots },
    };
    const selected_scene = if (std.mem.eql(u8, scene_name, "robot_only"))
        try Scene.init(allocator, robot, null, &.{}, .{}, &.{}, &robot_membership)
    else if (std.mem.eql(u8, scene_name, "robosub_course")) blk: {
        const start: types.Pose = .{ .pos = .{ 0, 2, 2 }, .quat = .{ 0, 0, 1, 0 } };
        const preset = self.scene_catalog.find("robosub_course") orelse return error.UnknownSimScene;
        const membership = [_]database.Membership{
            .{ .db = &self.catalog.db, .slots = robot_slots },
            .{ .db = &self.scene_catalog.db, .slots = preset.slots },
        };
        const extras = try allocator.alloc(database.BodySlot, preset.slots.len);
        defer allocator.free(extras);
        self.scene_catalog.presetBodySlots(preset, extras);
        break :blk try Scene.init(allocator, robot, start, extras, .{}, &.{}, &membership);
    } else return error.UnknownSimScene;
    self.scene = selected_scene;
    if (tam_len != self.scene.thruster_len) return error.InvalidThrusterCount;
    errdefer self.scene.deinit();
    try self.initPhysics();
    errdefer self.deinitPhysics();
    self.rebaseClock();
}

pub fn initWithScene(self: *Session, io: std.Io, scene: Scene, tam_len: usize, realtime: bool) !void {
    if (tam_len == 0 or tam_len > MissionConfig.thruster_count) return error.InvalidThrusterCount;
    if (tam_len != scene.thruster_len) return error.InvalidThrusterCount;
    self.* = .{
        .io = io,
        .catalogs_owned = false,
        .catalog = undefined,
        .scene_catalog = undefined,
        .scene = scene,
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
    errdefer self.scene.deinit();
    try self.initPhysics();
    errdefer self.deinitPhysics();
    self.rebaseClock();
}

fn initPhysics(self: *Session) !void {
    const scene = &self.scene;
    const allocator = scene.allocator;
    self.backend = try MujocoBackend.init(allocator, scene);
    errdefer self.backend.deinit();
    self.sensors = try Sensors.init(allocator, scene);
    errdefer self.sensors.deinit(allocator);
    self.thruster_len = scene.thruster_len;
    const count = scene.objects.len;
    self.wrenches = try allocator.alloc(types.Wrench, count);
    errdefer allocator.free(self.wrenches);
    self.fluid = try allocator.alloc(types.Wrench, count);
    errdefer allocator.free(self.fluid);
    self.previous_linear = try allocator.alloc(types.Vec3, count);
    errdefer allocator.free(self.previous_linear);
    self.previous_angular = try allocator.alloc(types.Vec3, count);
    errdefer allocator.free(self.previous_angular);
    // Hydrodynamics uses two temporary f64 extents per body.
    self.scratch = try allocator.alignedAlloc(u8, .of(f64), 2 * count * @sizeOf(f64));
    errdefer allocator.free(self.scratch);
    try self.initializeTick();
}

fn deinitPhysics(self: *Session) void {
    const allocator = self.scene.allocator;
    self.backend.deinit();
    self.sensors.deinit(allocator);
    allocator.free(self.scratch);
    allocator.free(self.previous_angular);
    allocator.free(self.previous_linear);
    allocator.free(self.fluid);
    allocator.free(self.wrenches);
}

pub fn deinit(self: *Session) void {
    self.deinitPhysics();
    self.scene.deinit();
    if (self.catalogs_owned) {
        self.scene_catalog.deinit();
        self.catalog.deinit();
    }
    self.* = undefined;
}

pub fn setThrusters(self: *Session, values: *const types.ThrusterCommandInput) !void {
    for (values[0..self.tam_len]) |v| if (!std.math.isFinite(v) or v < -1 or v > 1) return error.InvalidThrust;
    for (values[0..self.tam_len], 0..) |v, i| self.commands[i] = v;
}

pub fn rebaseClock(self: *Session) void {
    self.started = .now(self.io, .awake);
}

pub fn timestamp(self: *const Session) u64 {
    return @intCast(@as(u128, self.tick) * self.scene.settings.dt_numerator * 1_000_000_000 / self.scene.settings.dt_denominator);
}

pub fn reset(self: *Session, states: ?[]const types.BodyState) !void {
    try self.backend.reset(states);
    try self.initializeTick();
}

fn initializeTick(self: *Session) !void {
    self.tick = 0;
    self.rotors = @splat(.{});
    self.commands = @splat(0);
    @memset(self.fluid, .{});
    @memset(self.wrenches, .{});
    try self.validateReadback();
    self.saveVelocities();
    self.sample();
}

pub fn step(self: *Session, commands: *const types.ThrusterCommands) !void {
    if (self.tick == std.math.maxInt(u64)) return error.TickOverflow;
    const n = self.thruster_len;
    const effective = self.commands[0..n];
    if (@intFromPtr(commands.ptr) != @intFromPtr(effective.ptr)) {
        @memcpy(effective, commands[0..n]);
    }
    try self.advanceTick(effective);
}

fn stepPhysics(self: *Session) !void {
    if (self.tick == std.math.maxInt(u64)) return error.TickOverflow;
    try self.advanceTick(self.commands[0..self.thruster_len]);
}

fn advanceTick(self: *Session, effective: []f64) !void {
    for (self.scene.faults) |fault| {
        if (!fault.active(self.tick)) continue;
        switch (fault.effect) {
            .dead => |channel| effective[channel] *= 0,
            .weak => |weak| effective[weak.thruster] *= weak.scale,
            else => {},
        }
    }
    for (self.wrenches, self.scene.objects) |*wrench, slot| wrench.* = .{ .id = database.entity(slot).id };
    self.saveVelocities();
    for (self.scene.forces) |force| try forces.evaluate(force, self);

    try self.backend.step(self.wrenches);
    self.tick += 1;
    try self.validateReadback();
    const bounds = self.scene.settings.bounds;
    for (self.backend.states) |body| {
        if (S.norm(body.pose.pos) > bounds.position or S.norm(body.linear_velocity) > bounds.speed or
            S.norm(body.angular_velocity) > bounds.angular_speed) return error.SimulationBoundsExceeded;
    }
    self.sample();
}

fn validateReadback(self: *const Session) !void {
    for (self.backend.states, self.scene.objects) |body, object| {
        if (body.id != database.entity(object).id or !S.finite3(body.linear_velocity) or !S.finite3(body.angular_velocity) or
            !S.finite3(body.center_of_mass)) return error.InvalidBodyState;
        try S.validPose(body.pose);
    }
    for (self.backend.contacts.items) |contact| {
        if (!S.finite3(contact.position) or !S.finite3(contact.normal) or
            !std.math.isFinite(contact.normal_force)) return error.InvalidContact;
    }
}

fn saveVelocities(self: *Session) void {
    for (self.backend.states, self.previous_linear, self.previous_angular) |body, *linear, *angular| {
        linear.* = body.linear_velocity;
        angular.* = body.angular_velocity;
    }
}

fn sample(self: *Session) void {
    Sensors.sample(self);
}

/// The detection slice is valid until the next step, reset, or deinit.
pub fn observation(self: *const Session) types.Observation {
    return self.sensors.observation;
}

pub fn hash(self: *const Session) u64 {
    var h: S.Hash = .{};
    h.scalar(u64, self.tick);
    h.scalar(u64, self.backend.hash());
    for (self.backend.states) |body| h.state(body);
    for (self.rotors[0..self.scene.thruster_len], self.commands[0..self.scene.thruster_len]) |rotor, command| {
        h.scalar(f64, rotor.omega);
        h.scalar(f64, rotor.integral);
        h.scalar(f64, rotor.torque);
        h.scalar(f64, rotor.thrust);
        h.scalar(f64, command);
    }
    for (self.fluid) |wrench| {
        h.vec(wrench.force);
        h.vec(wrench.torque);
    }
    for (self.previous_linear) |value| h.vec(value);
    for (self.previous_angular) |value| h.vec(value);
    Sensors.hashSamples(self, &h);
    return h.value;
}

fn stepFrame(self: *Session) !void {
    if (self.failed) return error.SimulationFailed;
    self.stepPhysics() catch |err| {
        std.log.warn("simulator step failed: {s}", .{@errorName(err)});
        self.failed = true;
        return error.SimulationFailed;
    };
}

pub fn nextFrame(self: *Session, out: *Auv.Frame) !void {
    errdefer self.failed = true;
    for (0..frame_ticks) |_| try self.stepFrame();
    if (self.realtime) {
        const elapsed = self.started.untilNow(self.io).raw.toNanoseconds();
        const delay = @as(i96, @intCast(self.timestamp())) - elapsed;
        if (delay > 0) try std.Io.sleep(self.io, .fromNanoseconds(delay), .awake);
    }
    const obs = self.observation();
    out.* = std.mem.zeroes(Auv.Frame);
    out.timestamp = self.timestamp();
    out.camera_pose = .{ .pos = obs.body.pos, .quat = obs.body.quat };
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
        out.objects[i] = .{ .bbox = .{ .pose = .{ .pos = object.pose.pos, .quat = object.pose.quat }, .size = object.size }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects2d[i] = .{ .top_left = .{ .x = object.x0, .y = object.y0 }, .bottom_right = .{ .x = object.x1, .y = object.y1 }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects_len += 1;
        out.objects2d_len += 1;
    }
}