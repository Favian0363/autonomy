const std = @import("std");
const types = @import("types.zig");
const database = @import("database.zig");
const builder = @import("database_builder.zig");
const blend = @import("blend_catalog.zig");

pub const BlendFile = blend.BlendFile;

pub const SourceObject = struct {
    name: []const u8,
    id: u32,
    cls: u32,
    tracked: bool,
    collisions: bool,
    fuzz: bool,
    pose: types.Pose,
    hx: f32,
    hy: f32,
    hz: f32,
    positions: []const types.Vec3,
    triangles: []const types.Triangle,
};

pub fn writeCatalog(allocator: std.mem.Allocator, scene_name: []const u8, objects: []const SourceObject) ![]u8 {
    var db = builder.open(allocator);
    defer db.deinit();
    const start: u32 = @intCast(db.entities.items.len);
    for (objects) |object| {
        const box = database.boxMesh(object.hx, object.hy, object.hz);
        const positions = if (object.positions.len == 0) &box.positions else object.positions;
        const triangles = if (object.triangles.len == 0) &box.triangles else object.triangles;
        var collider = types.ColliderInput{
            .shape = .{ .mesh = .{ .positions = positions, .triangles = triangles } },
            .collisions = object.collisions,
            .size = .{ object.hx, object.hy, object.hz },
        };
        _ = try builder.addEntity(&db, .{
            .name = object.name,
            .id = object.id,
            .cls = object.cls,
            .tracked = object.tracked,
            .fuzz = object.fuzz,
            .initial_pose = object.pose,
            .colliders = (&collider)[0..1],
        });
    }
    try builder.appendName(&db, scene_name, start, @intCast(objects.len));
    return db.save(allocator);
}

pub fn sourcesFromBlend(allocator: std.mem.Allocator, file: *blend.BlendFile, collection: []const u8) ![]SourceObject {
    const exported = try file.exportCollection(collection);
    defer file.deinitExported(exported);
    var out = try allocator.alloc(SourceObject, exported.len);
    for (exported, 0..) |object, i| {
        const hx = ext(object.positions, 0);
        const hy = ext(object.positions, 1);
        const hz = ext(object.positions, 2);
        out[i] = .{
            .name = try allocator.dupe(u8, object.name),
            .id = object.id,
            .cls = object.cls,
            .tracked = object.tracked,
            .collisions = object.collisions,
            .fuzz = false,
            .pose = object.pose,
            .hx = hx,
            .hy = hy,
            .hz = hz,
            .positions = try allocator.dupe(types.Vec3, object.positions),
            .triangles = try allocator.dupe(types.Triangle, object.triangles),
        };
    }
    return out;
}

fn ext(positions: []const types.Vec3, axis: usize) f32 {
    if (positions.len == 0) return 0.5;
    var min: f32 = std.math.inf(f32);
    var max: f32 = -std.math.inf(f32);
    for (positions) |vertex| {
        min = @min(min, vertex[axis]);
        max = @max(max, vertex[axis]);
    }
    return @max((max - min) * 0.5, 1e-4);
}

pub fn freeSources(allocator: std.mem.Allocator, sources: []SourceObject, owned_mesh: bool) void {
    if (owned_mesh) {
        for (sources) |*object| {
            allocator.free(object.positions);
            allocator.free(object.triangles);
            allocator.free(object.name);
        }
    }
    allocator.free(sources);
}