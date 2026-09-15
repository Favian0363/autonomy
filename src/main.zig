const std = @import("std");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

pub fn main(init: std.process.Init) !void {
    const args: MissionArgs = try .init(init);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = try .init(init.gpa, init.io, args);

    ctx.yieldUntilNextFrameAndUpdate();

    switch (args.mission_id) {
        .prequalify => @import("missions/prequalify.zig").mission(&ctx),
    }

    while (true) {
        ctx.yieldUntilNextFrameAndUpdate();
    }
}
