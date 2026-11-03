const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");
const c = Auv.c;
const net = std.Io.net;

pub fn teleopMission(ctx: *MissionContext) !void {
    const port = ctx.config.teleop_port;
    const addr: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(port) };

    var sock = addr.bind(ctx.io, .{
        .mode = .dgram,
        .protocol = .udp,
    }) catch |err| switch (err) {
        error.AddressInUse => {
            std.log.err("teleop UDP port {d} is already in use", .{port});
            return error.AddressInUse;
        },
        else => return err,
    };
    defer sock.close(ctx.io);

    var recv_buf: [256]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{
        .clock = .awake,
        .raw = .fromNanoseconds(0),
    } };

    var translation: math.Vector3f = .{ 0, 0, 0 };
    var orientation: math.Quaternionf = .{ 0, 0, 0, 1 };

    while (true) {
        if (sock.receiveTimeout(ctx.io, &recv_buf, timeout)) |msg| {
            const packet_len = 7 * @sizeOf(f32);
            if (msg.data.len != packet_len) continue;
            var values: [7]f32 = undefined;
            for (0..values.len) |i| {
                const bits = std.mem.readInt(u32, msg.data[i * 4 ..][0..4], .little);
                values[i] = @bitCast(bits);
            }
            const next_translation: math.Vector3f = .{ values[0], values[1], values[2] };
            const next_orientation: math.Quaternionf = .{ values[3], values[4], values[5], values[6] };
            if (!std.math.isFinite(next_translation[0]) or
                !std.math.isFinite(next_translation[1]) or
                !std.math.isFinite(next_translation[2])) continue;
            const n2 = @reduce(.Add, next_orientation * next_orientation);
            if (!std.math.isFinite(n2) or n2 <= 1e-12) continue;
            translation = next_translation;
            orientation = math.normalize4f(next_orientation);
        } else |err| switch (err) {
            error.Timeout => {},
            else => return err,
        }

        if (math.length3f(translation) <= 1e-4) {
            ctx.motion = .{ .orientation = orientation };
        } else {
            ctx.motion = .{
                .translation = translation,
                .orientation = orientation,
            };
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

pub fn cocoMission(ctx: *MissionContext) !void {
    ctx.step = "find_person";
    const person = try ctx.yieldFind(@intCast(c.AUV_COCO_PERSON));
    ctx.step = "pass_person";
    try ctx.yieldPass(person);

    ctx.step = "find_bottle";
    const bottle = try ctx.yieldFind(@intCast(c.AUV_COCO_BOTTLE));
    ctx.step = "circle_bottle";
    try ctx.yieldCircle(bottle);
}

pub fn robosubMission(ctx: *MissionContext) !void {
    ctx.step = "find_gate";
    const opening = try ctx.yieldFind(@intCast(c.AUV_ROBOSUB_GATE_OPENING));
    ctx.step = "pass_gate";
    try ctx.yieldPass(opening);

    ctx.step = "find_marker";
    const marker = try ctx.yieldFind(@intCast(c.AUV_ROBOSUB_MARKER));
    ctx.step = "circle_marker";
    try ctx.yieldCircle(marker);
}
