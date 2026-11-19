//! Native simulator selection; hardware-only binaries never import MuJoCo.
const std = @import("std");
const Auv = @import("Auv.zig");

pub const enabled = @import("build_options").simulator;
pub const Session = if (enabled) @import("simulator/Session.zig").Session else struct {
    pub fn create(_: std.mem.Allocator, _: std.Io, _: []const u8, _: []const u8, _: usize, _: bool) !*@This() {
        return error.SimulatorNotBuilt;
    }
    pub fn destroy(_: *@This()) void {}
    pub fn setThrusters(_: *@This(), _: *const [8]f32) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn checkHealth(_: *@This()) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn nextFrame(_: *@This(), _: *Auv.Frame) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn rebaseClock(_: *@This()) void {}
};

test "native presets share command state with deterministic sensor frames" {
    if (!enabled) return error.SkipZigTest;
    const a = try Session.create(std.testing.allocator, std.testing.io, "proteus", "robot_only", 6, false);
    defer a.destroy();
    const b = try Session.create(std.testing.allocator, std.testing.io, "proteus", "robot_only", 6, false);
    defer b.destroy();
    const idle = try Session.create(std.testing.allocator, std.testing.io, "proteus", "robot_only", 6, false);
    defer idle.destroy();
    const commands = [_]f32{ 0.2, 0.2, 0, 0, 0, 0, 0, 0 };
    try a.setThrusters(&commands);
    try b.setThrusters(&commands);
    var fa: Auv.Frame = undefined;
    var fb: Auv.Frame = undefined;
    var fzero: Auv.Frame = undefined;
    for (0..30) |_| {
        try a.nextFrame(&fa);
        try b.nextFrame(&fb);
        try idle.nextFrame(&fzero);
    }
    try std.testing.expectEqual(@as(u64, 500_000_000), fa.timestamp);
    try std.testing.expectEqualDeep(fa.camera_pose, fb.camera_pose);
    try std.testing.expect(@reduce(.Or, fa.camera_pose.pos != fzero.camera_pose.pos));
    try std.testing.expect(fa.position_ok.x and fa.position_ok.y and fa.position_ok.z);
    // A failed simulator keeps the submitted data; cleanup generates no command.
    a.scene.settings.bounds.position = 0;
    try std.testing.expectError(error.SimulationFailed, a.nextFrame(&fa));
    try std.testing.expectEqual(@as(f64, commands[0]), a.commands[0]);
    inline for ([_]struct { name: []const u8, count: usize }{
        .{ .name = "hydrus", .count = 8 },
        .{ .name = "bluerov", .count = 8 },
        .{ .name = "girona500", .count = 5 },
    }) |robot| {
        const session = try Session.create(std.testing.allocator, std.testing.io, robot.name, "robosub_course", robot.count, false);
        defer session.destroy();
        try session.nextFrame(&fa);
        try std.testing.expectEqual(@as(u64, 16_666_666), fa.timestamp);
    }
}

test "native real-time frames wait for wall clock" {
    if (!enabled) return error.SkipZigTest;
    const session = try Session.create(std.testing.allocator, std.testing.io, "proteus", "robot_only", 6, true);
    defer session.destroy();
    var frame: Auv.Frame = undefined;
    const before = std.Io.Clock.Timestamp.now(std.testing.io, .awake);
    try session.nextFrame(&frame);
    try session.nextFrame(&frame);
    try std.testing.expect(before.untilNow(std.testing.io).raw.toMilliseconds() >= 25);
}

test "native simulator presets replay the same trace after reset" {
    if (!enabled) return error.SkipZigTest;
    const robots = [_][]const u8{ "hydrus", "proteus", "bluerov", "girona500" };
    for (robots, [_]usize{ 8, 6, 8, 5 }) |robot, count| {
        for ([_][]const u8{ "robot_only", "robosub_course" }) |scene| {
            const session = try Session.create(std.testing.allocator, std.testing.io, robot, scene, count, false);
            defer session.destroy();
            const original = try presetTrace(session);
            try session.runtime.reset(null);
            try std.testing.expectEqual(original, try presetTrace(session));
        }
    }
}

fn presetTrace(session: *Session) !u64 {
    const Hash = @import("simulator/support.zig").Hash;
    const idle: [8]f32 = @splat(0);
    try session.setThrusters(&idle);
    var hash: Hash = .{};
    for (0..30) |i| {
        if (i == 6) try session.setThrusters(&.{ 0.2, -0.15, 0.3, 0.1, -0.2, 0.22, -0.1, 0.05 });
        var frame: Auv.Frame = undefined;
        try session.nextFrame(&frame);
        const r = &session.runtime;
        hash.scalar(u64, frame.timestamp);
        hash.scalar(u64, r.hash());
        for (r.wrenches) |wrench| {
            hash.scalar(u32, wrench.id);
            hash.vec(wrench.force);
            hash.vec(wrench.torque);
        }
        hash.scalar(usize, r.backend.contacts.items.len);
        for (r.backend.contacts.items) |contact| {
            hash.scalar(u32, contact.a);
            hash.scalar(u32, contact.b);
            hash.vec(contact.position);
            hash.vec(contact.normal);
            hash.scalar(f64, contact.normal_force);
        }
    }
    return hash.value;
}

test "native runtime validates inputs before stepping and applies faults on schedule" {
    if (!enabled) return error.SkipZigTest;
    const Scene = @import("simulator/Scene.zig");
    const Runtime = @import("simulator/Runtime.zig");
    const types = @import("simulator/types.zig");
    const math = @import("simulator/support.zig");
    const RobotCatalog = @import("simulator/robots.zig").RobotCatalog;
    const database = @import("simulator/database.zig");
    var catalog = try RobotCatalog.init(std.testing.allocator);
    defer catalog.deinit();
    const robot_index = catalog.index("proteus").?;
    const robot = &catalog.robots[robot_index];
    const membership = [_]database.Membership{
        .{ .db = &catalog.db, .slots = catalog.slots[robot_index..][0..1] },
    };
    // The sparse IDs are independent of both Zig slice positions and MuJoCo body IDs.
    const objects = [_]types.Object{
        .{ .body = .{ .id = 100, .name = "contact box", .initial_pose = .{ .pos = .{ 0, 0, 2 } } }, .colliders = &.{.{}} },
    };
    const faults = [_]types.Fault{
        .{ .start = 0, .end = 2, .effect = .{ .weak = .{ .thruster = 0, .scale = 0.5 } } },
        .{ .start = 1, .end = 6, .effect = .{ .gyro_bias = .{ .sensor = @fromBackingInt(@intCast(1)), .bias = 0.5 } } },
        .{ .start = 1, .end = 7, .effect = .{ .disable_sensor = @fromBackingInt(@intCast(2)) } },
    };
    var scene = try Scene.init(std.testing.allocator, robot, .{ .pos = .{ 0, 0, 2 } }, &objects, .{}, &faults, &membership);
    defer scene.deinit();
    var runtime = try Runtime.init(&scene);
    defer runtime.deinit();
    const initial = runtime.hash();
    try std.testing.expectError(error.InvalidThrusterCount, runtime.step(&.{0}));
    try std.testing.expectError(error.InvalidThrust, runtime.step(&.{ 0, 0, 0, 0, 0, std.math.nan(f64) }));
    try std.testing.expectError(error.InvalidReset, runtime.reset(runtime.backend.states[0..1]));
    try std.testing.expectEqual(initial, runtime.hash());
    try std.testing.expect(runtime.backend.contacts.items.len > 0);
    for (runtime.backend.contacts.items) |contact| {
        try std.testing.expect((contact.a == 1 and contact.b == 100) or (contact.a == 100 and contact.b == 1));
    }

    const commands: [6]f64 = .{ 0.2, 0, 0, 0, 0, 0 };
    try runtime.step(&commands);
    try std.testing.expectEqual(@as(f64, 0.1), runtime.commands[0]);
    try std.testing.expectEqual(@as(u64, 0), runtime.sensors.samples[2].tick);
    const imu = runtime.sensors.samples[1].value.?.imu;
    const expected_gyro = math.rotate(
        math.conjugate(imu.orientation),
        runtime.backend.states[0].angular_velocity,
    );
    try std.testing.expectApproxEqAbs(expected_gyro[0] + 0.5, imu.gyro[0], 1e-5);
    try runtime.step(&commands);
    try runtime.step(&commands);
    try std.testing.expectEqual(commands[0], runtime.commands[0]);
    for (0..3) |_| try runtime.step(&commands);
    try std.testing.expect(!runtime.observation().depth_ok);
    for (0..6) |_| try runtime.step(&commands);
    try std.testing.expect(runtime.observation().depth_ok);
    try runtime.reset(null);
    try std.testing.expectEqual(initial, runtime.hash());
}

test "native simulator cleans up partially initialized sessions" {
    if (!enabled) return error.SkipZigTest;
    try std.testing.checkAllAllocationFailures(std.testing.allocator, createTestSession, .{});
}

fn createTestSession(allocator: std.mem.Allocator) !void {
    const session = try Session.create(allocator, std.testing.io, "proteus", "robot_only", 6, false);
    defer session.destroy();
    var frame: Auv.Frame = undefined;
    try session.nextFrame(&frame);
}
