const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args);

    while (true) {
        try ctx.yieldUntilNextFrameAndUpdate();
        //Make sure the goal starts  with the inital measurement of the odometry
        ctx.goal = math.poseTo6f(ctx.frame.camera_pose);

        const mission_result = switch (args.mission_id) {
            .prequalify_coco => @import("missions/prequalify.zig").cocoMission(&ctx),
            .prequalify_robosub => @import("missions/prequalify.zig").robosubMission(&ctx),
            .teleop => @import("missions/prequalify.zig").teleopMission(&ctx),
        };

        mission_result catch |err| switch (err) {
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
            else => return err,
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }

    ctx.auv.deinit();
}
