const std = @import("std");
const Io = std.Io;
const Auv = @import("Auv.zig");
const math = @import("math.zig");

const Telemetry = @This();

const default_log_path: ?[]const u8 = null;
const default_addr: []const u8 = "127.0.0.1:5600";

io: Io,
file: ?Io.File = null,
socket: ?Io.net.Socket = null,
dest: Io.net.IpAddress = undefined,

pub fn init(io: Io) !Telemetry {
    var t: Telemetry = .{ .io = io };
    if (default_log_path) |path| t.file = try Io.Dir.cwd().createFile(io, path, .{});
    t.dest = try .parseLiteral(default_addr);
    const any: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
    t.socket = try any.bind(io, .{ .mode = .dgram });
    return t;
}

// Field order is the little-endian packet. A slice is a u8 count, then its items.
// Struct fields are written without the alignment padding of the C layout.
const Packet = struct {
    magic: [4]u8 = "AUV1".*,
    version: u8 = 1,
    timestamp: u64,
    camera_pose: math.Pose,
    tracking_ok: bool,
    depth_ok: bool,
    depth: f32,
    objects: []const Auv.Object,
    boxes: []const Auv.Object2d,
    thrusters: []const f32,
    step: []const u8,
};

pub fn publish_to_udp(t: *Telemetry, frame: *const Auv.Frame, thrusters: []const f32, step: []const u8) void {
    if (t.file == null and t.socket == null) return;

    const nobj = @min(frame.objects_len, Auv.Frame.max_objects);
    const nbox = @min(frame.objects2d_len, Auv.Frame.max_objects);

    var buf: [64 * 1024]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    write(&w, Packet{
        .timestamp = frame.timestamp,
        .camera_pose = frame.camera_pose,
        .tracking_ok = frame.tracking_ok,
        .depth_ok = frame.pressure_depth_ok,
        .depth = frame.pressure_depth,
        .objects = frame.objects[0..nobj],
        .boxes = frame.objects2d[0..nbox],
        .thrusters = thrusters[0..@min(thrusters.len, 255)],
        .step = step[0..@min(step.len, 255)],
    }) catch return;
    const packet = w.buffered();

    if (t.file) |file| file.writeStreamingAll(t.io, packet) catch |err| {
        std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
    };
    // udp never blocks, even with the laptop unplugged
    if (t.socket) |*socket| socket.send(t.io, &t.dest, packet) catch {};
}

fn write(w: *Io.Writer, value: anytype) !void {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .int => {
            var buf: [@sizeOf(T)]u8 = undefined;
            std.mem.writeInt(T, &buf, value, .little);
            try w.writeAll(&buf);
        },
        .float => |info| {
            const Bits = std.meta.Int(.unsigned, info.bits);
            try write(w, @as(Bits, @bitCast(value)));
        },
        .bool => try write(w, @as(u8, @intFromBool(value))),
        .@"enum" => |info| try write(w, @as(info.tag_type, @intFromEnum(value))),
        .array => |info| {
            if (info.child == u8) {
                try w.writeAll(&value);
            } else {
                for (value) |item| try write(w, item);
            }
        },
        .vector => |info| {
            const array: [info.len]info.child = value;
            try write(w, array);
        },
        .pointer => |info| {
            if (info.size != .slice) @compileError("only slices can be written");
            if (value.len > std.math.maxInt(u8)) return error.TooLong;
            try write(w, @as(u8, @intCast(value.len)));
            if (info.child == u8) {
                try w.writeAll(value);
            } else {
                for (value) |item| try write(w, item);
            }
        },
        .@"struct" => |info| inline for (info.fields) |field| {
            try write(w, @field(value, field.name));
        },
        else => @compileError("unsupported telemetry type " ++ @typeName(T)),
    }
}
