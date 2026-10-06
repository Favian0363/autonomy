const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args);

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
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }
}
