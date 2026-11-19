//! In-process simulation session. Sensor frames and command data share this owner.
const std = @import("std");
const Auv = @import("../Auv.zig");
const Scene = @import("Scene.zig");
const Runtime = @import("Runtime.zig");
const RobotCatalog = @import("robots.zig").RobotCatalog;
const SceneCatalog = @import("scenes.zig").SceneCatalog;
const database = @import("database.zig");
const types = @import("types.zig");
const MissionConfig = @import("../MissionConfig.zig");

pub const Session = @This();

const frame_ticks = 6;

allocator: std.mem.Allocator,
io: std.Io,
catalog: RobotCatalog,
scene_catalog: SceneCatalog,
scene: Scene,
runtime: Runtime,
commands: types.ThrusterCommands = @splat(0),
realtime: bool,
tam_len: usize,
failed: bool = false,
started: std.Io.Clock.Timestamp,

pub fn init(self: *Session, allocator: std.mem.Allocator, io: std.Io, robot_name: []const u8, scene_name: []const u8, tam_len: usize, realtime: bool) !void {
    if (tam_len == 0 or tam_len > MissionConfig.thruster_count) return error.InvalidThrusterCount;
    self.* = .{
        .allocator = allocator,
        .io = io,
        .catalog = undefined,
        .scene_catalog = undefined,
        .scene = undefined,
        .runtime = undefined,
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
    self.runtime = try Runtime.init(&self.scene);
    errdefer self.runtime.deinit();
    self.rebaseClock();
}

pub fn rebindScenePointers(self: *Session) void {
    self.runtime.rebindScene(&self.scene);
}

pub fn deinit(self: *Session) void {
    self.runtime.deinit();
    self.scene.deinit();
    self.scene_catalog.deinit();
    self.catalog.deinit();
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
    return @intCast(@as(u128, self.runtime.tick) * self.scene.settings.dt_numerator * 1_000_000_000 / self.scene.settings.dt_denominator);
}

fn step(self: *Session) !void {
    if (self.failed) return error.SimulationFailed;
    self.runtime.step(&self.commands) catch |err| {
        std.log.warn("simulator step failed: {s}", .{@errorName(err)});
        self.failed = true;
        return error.SimulationFailed;
    };
}

pub fn nextFrame(self: *Session, out: *Auv.Frame) !void {
    errdefer self.failed = true;
    for (0..frame_ticks) |_| try self.step();
    if (self.realtime) {
        const elapsed = self.started.untilNow(self.io).raw.toNanoseconds();
        const delay = @as(i96, @intCast(self.timestamp())) - elapsed;
        if (delay > 0) try std.Io.sleep(self.io, .fromNanoseconds(delay), .awake);
    }
    const observation = self.runtime.observation();
    out.* = std.mem.zeroes(Auv.Frame);
    out.timestamp = self.timestamp();
    out.camera_pose = .{ .pos = observation.body.pos, .quat = observation.body.quat };
    out.position_ok = .{ .x = observation.position_axes & 1 != 0, .y = observation.position_axes & 2 != 0, .z = observation.position_axes & 4 != 0 };
    out.attitude_ok = .{ .x = observation.attitude_axes & 1 != 0, .y = observation.attitude_axes & 2 != 0, .z = observation.attitude_axes & 4 != 0 };
    out.imu_quat = observation.imu;
    out.accel = observation.accel;
    out.gyro = observation.gyro;
    out.pressure_depth = @floatCast(observation.depth);
    out.pressure_depth_ok = observation.depth_ok;
    out.image_width = observation.image_width;
    out.image_height = observation.image_height;
    for (observation.objects[0..@min(observation.objects.len, Auv.Frame.max_objects)], 0..) |object, i| {
        if (object.cls > 255) return error.InvalidObjectClass;
        out.objects[i] = .{ .bbox = .{ .pose = .{ .pos = object.pose.pos, .quat = object.pose.quat }, .size = object.size }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects2d[i] = .{ .top_left = .{ .x = object.x0, .y = object.y0 }, .bottom_right = .{ .x = object.x1, .y = object.y1 }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects_len += 1;
        out.objects2d_len += 1;
    }
}