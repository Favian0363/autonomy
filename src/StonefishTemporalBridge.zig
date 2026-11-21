//! Optional Stonefish dynlib export (`stonefish_temporal_bridge_set_thrusters`).
pub const SetThrustersFn = fn (values: [*]const f32, count: u8) callconv(.c) void;
pub const symbol = "stonefish_temporal_bridge_set_thrusters";

pub fn apply(set: *const SetThrustersFn, values: []const f32) void {
    if (values.len == 0) return;
    set(values.ptr, @intCast(@min(values.len, 255)));
}