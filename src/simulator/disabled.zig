const std = @import("std");
const Auv = @import("../Auv.zig");

pub const Session = struct {
    pub fn create(_: std.mem.Allocator, _: std.Io, _: []const u8, _: []const u8, _: usize, _: bool) !*@This() {
        return error.SimulatorNotBuilt;
    }
    pub fn destroy(_: *@This()) void {}
    pub fn setThrusters(_: *@This(), _: *const [8]f32) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn checkHealth(_: *@This()) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn nextFrame(_: *@This(), _: *Auv.Frame) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn rebaseClock(_: *@This()) void {}
};
