const std = @import("std");
const types = @import("types.zig");
const data = @import("robot_data.zig");

pub fn find(name: []const u8) ?*const types.Robot {
    for ([_]*const types.Robot{ &data.hydrus, &data.proteus, &data.bluerov, &data.girona500 }) |robot| {
        if (std.mem.eql(u8, name, robot.name)) return robot;
    }
    return null;
}
