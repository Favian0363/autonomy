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

pub const Config = struct {
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
    tam: []math.Vector6f,
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
