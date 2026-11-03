const std = @import("std");
const math = @import("math.zig");
const Io = std.Io;
const Auv = @import("Auv.zig");

const MissionLoader = @This();

const max_thrustors = 8;

config_path: []const u8,
use_ping: bool,
ping: std.heap.FixedBufferAllocator,
pong: std.heap.FixedBufferAllocator,
reader_buf: [4098]u8,

pub const NonOdometryPid = struct {
    center_deadband: f32,
    target_height: f32,
    /// Rows are body X, Y, Z, roll, pitch, then yaw. Row count must match [`math.interaction_output_len`].
    /// Each row length must match the runtime error vector passed to [`math.mapInteraction`].
    interaction: [][]f32,
};

pub const OdometryPid = struct {
    /// Body attitude error magnitude (rad) above which horizontal translation is zeroed.
    rotation_gate: f32 = std.math.pi / 8.0,
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
};

pub const Config = struct {
    teleop_port: u16 = 8080,
    /// Body direction the camera looks along, such as `[0, 1, 0]` for body +Y.
    camera_forward: math.Vector3f,
    /// Max |rotation| about body X, Y, and Z (rad) when building a look from a world direction. Zero holds that axis at the reference.
    rotation_limit: math.Vector3f = .{std.math.inf(f32), std.math.inf(f32), std.math.inf(f32) },
    tam: []math.Vector6f,
    odometry: OdometryPid,
    non_odometry: NonOdometryPid,
};

pub fn init(arena: std.mem.Allocator, config_path: []const u8) !MissionLoader {
    return .{
        .config_path = config_path,
        .use_ping = false,
        .ping = .init(try arena.alloc(u8, 4098)),
        .pong = .init(try arena.alloc(u8, 4098)),
        .reader_buf = undefined,
    };
}

pub fn load(loader: *MissionLoader, io: Io) !Config {
    const fba = if (loader.use_ping) &loader.ping else &loader.pong;
    const file = try Io.Dir.cwd().openFile(io, loader.config_path, .{ .mode = .read_only });
    var file_reader = file.reader(io, &loader.reader_buf);
    const file_contents = try file_reader.interface.allocRemaining(fba.allocator(), .unlimited);

    const config = std.json.parseFromSliceLeaky(Config, fba.allocator(), file_contents, .{});

    loader.use_ping = !loader.use_ping;
    fba.reset();

    return config;
}
