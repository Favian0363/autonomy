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
const abi = @import("abi.zig");
const session_snapshot = @import("session_snapshot.zig");

pub const Session = @This();

const frame_ticks = 6;

io: std.Io,
catalogs_owned: bool,
catalog: RobotCatalog,
scene_catalog: SceneCatalog,
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
started: std.Io.Clock.Timestamp,

fn loadSceneCatalog(allocator: std.mem.Allocator, io: std.Io, config: *const MissionConfig.MissionConfig) !SceneCatalog {
    const catalog_path = try config.sim_scene_catalog.slice();
    const blend_path = try config.sim_blend.slice();
    const collection = try config.sim_blend_collection.slice();
    const scene_name = try config.sim_scene.slice();
    if (catalog_path.len > 0) {
        return SceneCatalog.initResolved(allocator, io, .{ .file_path = catalog_path });
    }
    if (blend_path.len > 0) {
        const collection_name = if (collection.len > 0) collection else "robosub_course";
        const catalog_name = if (collection.len > 0) collection else scene_name;
        return SceneCatalog.initResolved(allocator, io, .{ .blend = .{
            .path = blend_path,
            .collection = collection_name,
            .catalog_name = catalog_name,
        } });
    }
    return SceneCatalog.initResolved(allocator, io, .embedded);
}

pub fn init(self: *Session, allocator: std.mem.Allocator, io: std.Io, config: *const MissionConfig.MissionConfig, tam_len: usize, realtime: bool) !void {
    const robot_name = try config.sim_robot.slice();
    const scene_name = try config.sim_scene.slice();
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
    self.scene_catalog = try loadSceneCatalog(allocator, io, config);
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
    else blk: {
        const preset = self.scene_catalog.find(scene_name) orelse return error.UnknownSimScene;
        const start: ?types.Pose = if (std.mem.eql(u8, scene_name, "robosub_course"))
            .{ .pos = .{ 0, 2, 2 }, .quat = .{ 0, 0, 1, 0 } }
        else
            null;
        const membership = [_]database.Membership{
            .{ .db = &self.catalog.db, .slots = robot_slots },
            .{ .db = &self.scene_catalog.db, .slots = preset.slots },
        };
        const extras = try allocator.alloc(database.BodySlot, preset.slots.len);
        defer allocator.free(extras);
        self.scene_catalog.presetBodySlots(preset, extras);
        break :blk try Scene.init(allocator, robot, start, extras, .{}, &.{}, &membership);
    };
    self.scene = selected_scene;
    if (tam_len != self.scene.thruster_len) return error.InvalidThrusterCount;
    errdefer self.scene.deinit();
    self.preset_robot = try allocator.dupe(u8, robot_name);
    errdefer self.scene.allocator.free(self.preset_robot);
    self.preset_scene = try allocator.dupe(u8, scene_name);
    errdefer self.scene.allocator.free(self.preset_scene);
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
    self.preset_robot = try scene.allocator.dupe(u8, "");
    errdefer scene.allocator.free(self.preset_robot);
    self.preset_scene = try scene.allocator.dupe(u8, "");
    errdefer scene.allocator.free(self.preset_scene);
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
    self.wrenches = try allocator.alloc(abi.Wrench, count);
    errdefer allocator.free(self.wrenches);
    self.fluid = try allocator.alloc(abi.Wrench, count);
    errdefer allocator.free(self.fluid);
    self.previous_linear = try allocator.alloc(abi.Vec3, count);
    errdefer allocator.free(self.previous_linear);
    self.previous_angular = try allocator.alloc(abi.Vec3, count);
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
    const allocator = self.scene.allocator;
    self.deinitPhysics();
    if (self.preset_robot.len != 0) allocator.free(self.preset_robot);
    if (self.preset_scene.len != 0) allocator.free(self.preset_scene);
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

pub fn reset(self: *Session, states: ?[]const abi.BodyState) !void {
    try self.backend.reset(states);
    try self.initializeTick();
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
    const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
    for (self.wrenches, self.scene.objects) |*wrench, slot| {
        wrench.* = zero_wrench;
        wrench.id = database.entity(slot).id;
    }
    self.saveVelocities();
    for (self.scene.forces) |force| try forces.evaluate(force, self);

    try self.backend.step(self.wrenches);
    self.tick += 1;
    try self.validateReadback();
    const bounds = self.scene.settings.bounds;
    for (self.backend.states) |body| {
        if (S.norm(body.pose_pos) > bounds.position or S.norm(body.linear_vel) > bounds.speed or
            S.norm(body.angular_vel) > bounds.angular_speed) return error.SimulationBoundsExceeded;
    }
    self.refreshSensors();
}

fn validateReadback(self: *const Session) !void {
    for (self.backend.states, self.scene.objects) |body, object| {
        if (body.id != database.entity(object).id or !S.finite3(body.linear_vel) or !S.finite3(body.angular_vel) or
            !S.finite3(body.com)) return error.InvalidBodyState;
        try S.validPose(S.bodyPose(body));
    }
    for (self.backend.contacts.items) |contact| {
        if (!S.finite3(contact.position) or !S.finite3(contact.normal) or
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

/// Hash of serialized dynamics (excludes MuJoCo integration buffer in `backend.hash()`).
pub fn dynamicsHash(self: *const Session) u64 {
    var h: S.Hash = .{};
    h.scalar(u64, self.tick);
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

pub fn hash(self: *const Session) u64 {
    var h: S.Hash = .{};
    h.scalar(u64, self.backend.hash());
    h.scalar(u64, self.dynamicsHash());
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

pub fn snapshot(self: *const Session, allocator: std.mem.Allocator) ![]u8 {
    var snap = try session_snapshot.capture(allocator, self);
    defer snap.deinit();
    return try session_snapshot.save(allocator, &snap);
}

pub fn restoreSnapshot(self: *Session, allocator: std.mem.Allocator, bytes: []const u8) !void {
    var snap = try session_snapshot.load(allocator, bytes);
    defer snap.deinit();
    try session_snapshot.restore(self, &snap);
}

test "session snapshot round trip preserves hash" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    var session: Session = undefined;
    var config = MissionConfig.startup_config;
    config.sim_robot = .fromLiteral("proteus");
    config.sim_scene = .fromLiteral("robot_only");
    try session.init(std.testing.allocator, std.testing.io, &config, 6, false);
    defer session.deinit();
    const idle: types.ThrusterCommandInput = @splat(0);
    try session.setThrusters(&idle);
  for (0..12) |_| try session.step(&session.commands);
    const before = session.dynamicsHash();
    const bytes = try session.snapshot(std.testing.allocator);
    defer std.testing.allocator.free(bytes);
    try session.restoreSnapshot(std.testing.allocator, bytes);
    try std.testing.expectEqual(before, session.dynamicsHash());
}