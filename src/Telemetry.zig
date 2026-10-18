const std = @import("std");
const Io = std.Io;
const Auv = @import("Auv.zig");

const Telemetry = @This();

io: Io,
file: ?Io.File = null,
socket: ?Io.net.Socket = null,
dest: Io.net.IpAddress = undefined,

pub fn init(io: Io, log_path: ?[]const u8, addr: ?[]const u8) !Telemetry {
    var t: Telemetry = .{ .io = io };
    if (log_path) |path| t.file = try Io.Dir.cwd().createFile(io, path, .{});
    if (addr) |text| {
        t.dest = try .parseLiteral(text);
        const any: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
        t.socket = try any.bind(io, .{ .mode = .dgram });
    }
    return t;
}

pub fn emit(t: *Telemetry, frame: *const Auv.Frame, thrusters: []const f32, step: []const u8) void {
    if (t.file == null and t.socket == null) return;
    var buf: [64 * 1024]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    writeLine(&w, frame, thrusters, step) catch return;
    const line = w.buffered();

    if (t.file) |file| file.writeStreamingAll(t.io, line) catch |err| {
        std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
    };
    // udp never blocks, even with the laptop unplugged
    if (t.socket) |*socket| socket.send(t.io, &t.dest, line) catch {};
}

pub fn writeLine(w: *Io.Writer, frame: *const Auv.Frame, thrusters: []const f32, step: []const u8) !void {
    const Object = struct { id: u32, cls: u8, pos: [3]f32, size: [3]f32 };
    var objects: [Auv.Frame.max_objects]Object = undefined;
    const len = @min(frame.objects_len, Auv.Frame.max_objects);
    for (frame.objects[0..len], objects[0..len]) |from, *to| to.* = .{
        .id = from.id,
        .cls = @intFromEnum(from.cls),
        .pos = from.bbox.pose.pos,
        .size = from.bbox.size,
    };

    const Box = struct { id: u32, cls: u8, box: [4]u32 };
    var boxes: [Auv.Frame.max_objects]Box = undefined;
    const boxes_len = @min(frame.objects2d_len, Auv.Frame.max_objects);
    for (frame.objects2d[0..boxes_len], boxes[0..boxes_len]) |from, *to| to.* = .{
        .id = from.id,
        .cls = @intFromEnum(from.cls),
        .box = .{ from.top_left.x, from.top_left.y, from.bottom_right.x, from.bottom_right.y },
    };

    const pose = frame.camera_pose;
    try std.json.Stringify.value(.{
        .t = frame.timestamp,
        .pose = @as([3]f32, pose.pos) ++ @as([4]f32, pose.quat),
        .tracking = frame.tracking_ok,
        .depth = if (frame.pressure_depth_ok) frame.pressure_depth else null,
        .objs = objects[0..len],
        .boxes = boxes[0..boxes_len],
        .thr = thrusters,
        .step = step,
    }, .{}, w);
    try w.writeByte('\n');
}
