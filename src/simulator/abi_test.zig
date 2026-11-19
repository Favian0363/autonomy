const std = @import("std");
const abi = @import("abi.zig");

test "SIMDB record layouts match the on-disk contract" {
    const testing = std.testing;
    try testing.expectEqual(@as(usize, 8), abi.magic.len);
    try testing.expectEqual(@as(u32, 1), abi.version);
    try testing.expectEqual(@as(u32, 9), abi.table_count);
    try testing.expect(abi.strides[abi.table.entities] == @sizeOf(abi.EntityRecord));
    try testing.expect(abi.strides[abi.table.colliders] == @sizeOf(abi.ColliderRecord));
    try testing.expect(abi.strides[abi.table.thrusters] == @sizeOf(abi.ThrusterRecord));
    try testing.expect(abi.strides[abi.table.hulls] == @sizeOf(abi.HullRecord));
    try testing.expect(abi.strides[abi.table.sensors] == @sizeOf(abi.SensorRecord));
    try testing.expect(abi.strides[abi.table.names] == @sizeOf(abi.NameRecord));
    try testing.expectEqual(@as(u32, 0), @intFromEnum(abi.motion_tag.static));
    try testing.expectEqual(@as(u32, 3), @intFromEnum(abi.shape_tag.mesh));
    try testing.expectEqual(@as(u32, 3), @intFromEnum(abi.sensor_kind_tag.camera));
}