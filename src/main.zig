const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");
const Telemetry = @import("Telemetry.zig");

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    const telemetry: Telemetry = try .init(
        init.io,
        init.environ_map.get("AUV_TELEMETRY_LOG"),
        init.environ_map.get("AUV_TELEMETRY_ADDR") orelse "127.0.0.1:5600",
    );
    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args, telemetry);

    const mission = switch (args.mission_id) {
        .prequalify => @import("missions/prequalify.zig").mission,
    };

    while (true) {
        try ctx.yieldUntilNextFrameAndUpdate();
        ctx.goal = math.poseTo6f(ctx.frame.camera_pose);

        mission(&ctx) catch |err| switch (err) {
            error.RestartMission => {
                std.log.info("restaring mission context", .{});
                ctx.reset();
                std.log.info("restaring mission", .{});
                continue;
            },
            error.Stopped => {
                std.log.info("stopped", .{});
                break;
            },
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }

    ctx.auv.deinit();
}
