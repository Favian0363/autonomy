//! Blender `.meshbin` wire format for `export_mesh`.
const std = @import("std");
const types = @import("types.zig");

pub const mesh_magic: [4]u8 = "SMM1".*;

pub fn encode(allocator: std.mem.Allocator, positions: []const types.Vec3, triangles: []const types.Triangle) ![]u8 {
    const vert_count: u32 = @intCast(positions.len);
    const tri_count: u32 = @intCast(triangles.len);
    const vert_bytes = vert_count * 3 * @sizeOf(f32);
    const tri_bytes = tri_count * 3 * @sizeOf(u32);
    const out = try allocator.alloc(u8, 12 + vert_bytes + tri_bytes);
    @memcpy(out[0..4], &mesh_magic);
    std.mem.writeInt(u32, out[4..8], vert_count, .little);
    std.mem.writeInt(u32, out[8..12], tri_count, .little);
    @memcpy(out[12 .. 12 + vert_bytes], std.mem.sliceAsBytes(positions));
    const tri_offset = 12 + vert_bytes;
    const indices = std.mem.bytesAsSlice(u32, std.mem.sliceAsBytes(triangles));
    @memcpy(out[tri_offset .. tri_offset + tri_bytes], std.mem.sliceAsBytes(indices));
    return out;
}