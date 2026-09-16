const std = @import("std");
const Io = std.Io;

const MissionArgs = @This();

pub const MissionId = enum {
    prequalify,
};

goal_dist_threshold: f32,
auv_dynlib_path: []const u8,
mission_id: MissionId,
live_config_path: []const u8,

pub fn init(p_init: std.process.Init) !MissionArgs {
    const argsSlice = try p_init.minimal.args.toSlice(p_init.arena.allocator());
    if (argsSlice.len < 3) {
        std.log.err("usage: {s} <auv_dynlib_path> <mission_name> <live_config_path>", .{argsSlice[0]});
        return error.MissingArgs;
    }

    const auv_dynlib_path = argsSlice[1];
    const mission_name = argsSlice[2];
    const live_config_path = argsSlice[3];
    const mission_id = std.meta.stringToEnum(MissionId, mission_name) orelse {
        std.log.err("mission_id `{s}` doesn't exist", .{mission_name});
        std.log.info("valids mission_id's are:", .{});
        for (std.enums.values(MissionId)) |mission_id| {
            const valid_mission_name = std.enums.tagName(MissionId, mission_id) orelse unreachable;
            std.log.info("\t{s}", .{valid_mission_name});
        }
        return error.InvalidMissionId;
    };

    return .{
        .mission_id = mission_id,
        .auv_dynlib_path = auv_dynlib_path,
        .live_config_path = live_config_path,
        .goal_dist_threshold = 1.0,
    };
}

pub fn format(args: MissionArgs, writer: *std.Io.Writer) !void {
    try writer.print(
        "auv_dynlib_path = {s}, mission_name = {s}, goal_dist_threshold = {}",
        .{
            args.auv_dynlib_path,
            std.enums.tagName(MissionId, args.mission_id) orelse unreachable,
            args.goal_dist_threshold,
        },
    );
}
