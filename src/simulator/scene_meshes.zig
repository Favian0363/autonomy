//! GPU mesh cache for the embedded simulator visualizer.
const std = @import("std");
const rl = @import("raylib");
const Scene = @import("Scene.zig");
const database = @import("database.zig");
const types = @import("types.zig");
const abi = @import("abi.zig");
const S = @import("support.zig");

const max_index_vertices = std.math.maxInt(u16);

pub const SceneMeshCache = struct {
    allocator: std.mem.Allocator,
    entries: []Entry,

    pub fn init(allocator: std.mem.Allocator, scene: *const Scene) !SceneMeshCache {
        var list: std.ArrayList(Entry) = .empty;
        errdefer deinitEntries(allocator, list.items);

        for (scene.objects, 0..) |slot, body_index| {
            const entity = database.entity(slot);
            const collider_count = database.ownedCount(slot.db.colliders, slot.slot);
            for (0..collider_count) |ki| {
                const collider = database.colliderAt(slot.db, slot.slot, ki);
                const shape = database.shapeFrom(slot.db, collider);
                const entry = makeEntry(allocator, shape, collider, entity.cls, body_index) catch |err| switch (err) {
                    error.MeshTooLarge => {
                        std.log.warn("embedded visualizer skipped a collider mesh ({s})", .{@errorName(err)});
                        continue;
                    },
                    else => return err,
                };
                errdefer entry.deinit(allocator);
                try list.append(allocator, entry);
            }
        }
        return .{ .allocator = allocator, .entries = try list.toOwnedSlice(allocator) };
    }

    pub fn deinit(self: *SceneMeshCache) void {
        deinitEntries(self.allocator, self.entries);
        self.allocator.free(self.entries);
        self.* = undefined;
    }

    pub fn draw(self: *const SceneMeshCache, bodies: []const abi.BodyState) void {
        if (bodies.len == 0) return;
        for (self.entries) |entry| {
            if (entry.body_index >= bodies.len) continue;
            const body = bodies[entry.body_index];
            const collider_pose = database.poseOf(entry.collider_pos, entry.collider_quat);
            const world = S.compose(S.bodyPose(body), collider_pose);
            const transform = poseMatrix(world);
            rl.drawMesh(entry.mesh, entry.material, transform);
        }
    }
};

pub const DrawContext = struct {
    cache: *SceneMeshCache,
    bodies: []const abi.BodyState = &.{},
};

pub fn drawEmbedded(ctx: *const anyopaque) void {
    const self: *const DrawContext = @ptrCast(@alignCast(ctx));
    self.cache.draw(self.bodies);
}

const cpu_allocator = std.heap.c_allocator;

const Entry = struct {
    mesh: rl.Mesh,
    material: rl.Material,
    body_index: usize,
    collider_pos: [3]f32,
    collider_quat: [4]f32,

    fn deinit(self: *const Entry, allocator: std.mem.Allocator) void {
        _ = allocator;
        // UnloadMesh frees CPU vertex/index buffers with the C heap (genMesh* and uploadMesh).
        rl.unloadMesh(self.mesh);
        rl.unloadMaterial(self.material);
    }
};

fn deinitEntries(allocator: std.mem.Allocator, entries: []Entry) void {
    for (entries) |*entry| entry.deinit(allocator);
}

fn makeEntry(
    _: std.mem.Allocator,
    shape: types.Shape,
    collider: *const database.Collider,
    cls: u32,
    body_index: usize,
) !Entry {
    var mesh: rl.Mesh = switch (shape) {
        .mesh => |data| try uploadTriangleMesh(data.positions, data.triangles),
        .box => rl.genMeshCube(collider.size[0], collider.size[1], collider.size[2]),
        .sphere => rl.genMeshSphere(collider.size[0], 24, 24),
        .cylinder => rl.genMeshCylinder(collider.size[0], collider.size[2] * 2, 24),
    };
    rl.uploadMesh(&mesh, false);
    errdefer rl.unloadMesh(mesh);

    var material = try rl.loadMaterialDefault();
    material.maps[@intFromEnum(rl.MaterialMapIndex.albedo)].color = entityColor(body_index, @intCast(cls));

    return .{
        .mesh = mesh,
        .material = material,
        .body_index = body_index,
        .collider_pos = collider.pos,
        .collider_quat = collider.quat,
    };
}

fn uploadTriangleMesh(
    positions: []const abi.Vec3,
    triangles: []const types.Triangle,
) !rl.Mesh {
    if (positions.len > max_index_vertices) return error.MeshTooLarge;
    const verts = try cpu_allocator.alloc(f32, positions.len * 3);
    errdefer cpu_allocator.free(verts);
    for (positions, 0..) |p, i| {
        verts[i * 3] = p[0];
        verts[i * 3 + 1] = p[1];
        verts[i * 3 + 2] = p[2];
    }

    const indices = try cpu_allocator.alloc(u16, triangles.len * 3);
    errdefer cpu_allocator.free(indices);
    for (triangles, 0..) |tri, fi| {
        if (tri.a >= positions.len or tri.b >= positions.len or tri.c >= positions.len) return error.MeshTooLarge;
        indices[fi * 3] = @intCast(tri.a);
        indices[fi * 3 + 1] = @intCast(tri.b);
        indices[fi * 3 + 2] = @intCast(tri.c);
    }

    var mesh = std.mem.zeroes(rl.Mesh);
    mesh.vertexCount = @intCast(positions.len);
    mesh.triangleCount = @intCast(triangles.len);
    mesh.vertices = verts.ptr;
    mesh.indices = indices.ptr;
    return mesh;
}

fn entityColor(body_index: usize, cls: u8) rl.Color {
    if (body_index == 0) return rl.Color.sky_blue;
    return rl.colorFromHSV(@floatFromInt((@as(u16, cls) * 47) % 360), 0.75, 0.95);
}

fn toRay(v: types.Vec3) rl.Vector3 {
    return .{ .x = @floatCast(v[0]), .y = @floatCast(-v[2]), .z = @floatCast(-v[1]) };
}

fn poseMatrix(pose: types.Pose) rl.Matrix {
    const t = toRay(pose.pos);
    const x = toRay(S.rotate(pose.quat, .{ 1, 0, 0 }));
    const y = toRay(S.rotate(pose.quat, .{ 0, 1, 0 }));
    const z = toRay(S.rotate(pose.quat, .{ 0, 0, 1 }));
    return .{
        .m0 = x.x,
        .m4 = y.x,
        .m8 = z.x,
        .m12 = t.x,
        .m1 = x.y,
        .m5 = y.y,
        .m9 = z.y,
        .m13 = t.y,
        .m2 = x.z,
        .m6 = y.z,
        .m10 = z.z,
        .m14 = t.z,
        .m3 = 0,
        .m7 = 0,
        .m11 = 0,
        .m15 = 1,
    };
}

test "pose matrix maps unit Y to ray space" {
    const pose: types.Pose = .{ .pos = .{ 0, 1, 2 }, .quat = .{ 0, 0, 0, 1 } };
    const m = poseMatrix(pose);
    try std.testing.expectApproxEqAbs(@as(f32, 0), m.m12, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -2), m.m13, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -1), m.m14, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), m.m4, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), m.m5, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -1), m.m6, 1e-5);
}