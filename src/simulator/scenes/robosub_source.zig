//! Hand-authored robosub scene source: catalog builder, mesh blobs, and `robosub_course` preset.
const std = @import("std");
const types = @import("../types.zig");
const abi = @import("../abi.zig");
const database = @import("../database.zig");

// --- Blender .meshbin import/export (tools/export_mesh) ---

pub const mesh_magic: [4]u8 = "SMM1".*;

pub fn decodeMeshBlob(bytes: []const u8) !struct { positions: []const types.Vec3, triangles: []const types.Triangle } {
    if (bytes.len < 12 or !std.mem.eql(u8, bytes[0..4], &mesh_magic)) return error.BadMeshMagic;
    const vert_count = std.mem.readInt(u32, bytes[4..8], .little);
    const tri_count = std.mem.readInt(u32, bytes[8..12], .little);
    const vert_bytes = vert_count * 3 * @sizeOf(f32);
    const tri_bytes = tri_count * 3 * @sizeOf(u32);
    if (bytes.len != 12 + vert_bytes + tri_bytes) return error.InvalidMeshBlob;
    const positions = std.mem.bytesAsSlice(types.Vec3, bytes[12 .. 12 + vert_bytes]);
    const tri_offset = 12 + vert_bytes;
    const indices = std.mem.bytesAsSlice(u32, bytes[tri_offset .. tri_offset + tri_bytes]);
    if (indices.len % 3 != 0) return error.InvalidMeshBlob;
    const triangles = std.mem.bytesAsSlice(types.Triangle, std.mem.sliceAsBytes(indices));
    return .{ .positions = positions, .triangles = triangles };
}

pub fn meshFromBlob(bytes: []const u8) types.Mesh {
    const decoded = decodeMeshBlob(bytes) catch unreachable;
    return .{ .positions = decoded.positions, .triangles = decoded.triangles };
}

pub fn encodeMeshBlob(allocator: std.mem.Allocator, positions: []const types.Vec3, triangles: []const types.Triangle) ![]u8 {
    const vert_bytes = positions.len * @sizeOf(types.Vec3);
    const tri_bytes = triangles.len * @sizeOf(types.Triangle);
    const out = try allocator.alloc(u8, 12 + vert_bytes + tri_bytes);
    @memcpy(out[0..4], &mesh_magic);
    std.mem.writeInt(u32, out[4..8], @intCast(positions.len), .little);
    std.mem.writeInt(u32, out[8..12], @intCast(triangles.len), .little);
    @memcpy(out[12 .. 12 + vert_bytes], std.mem.sliceAsBytes(positions));
    @memcpy(out[12 + vert_bytes ..], std.mem.sliceAsBytes(triangles));
    return out;
}

// --- In-memory SIMDB authoring ---

const Db = database.Database;

pub const EntityInput = struct {
    name: []const u8,
    asset_file: []const u8 = "",
    id: u32,
    cls: u32 = 0,
    motion_type: types.Motion = .static,
    tracked: bool = false,
    fuzz: bool = false,
    mass: f64 = 0,
    inertia: types.Vec3 = @splat(1),
    center_of_mass: types.Pose = .{},
    initial_pose: types.Pose = .{},
    initial_velocity: types.Vec3 = @splat(0),
    initial_angular_velocity: types.Vec3 = @splat(0),
    dry_mass: f64 = 0,
    volume: f64 = 0,
    buoyancy_center: types.Vec3 = @splat(0),
    colliders: []const types.ColliderInput = &.{},
    thrusters: []const abi.Thruster = &.{},
    hulls: []const struct { hull: abi.Hull, positions: []const types.Vec3 } = &.{},
    sensors: []const types.SensorInput = &.{},
};

pub fn meshCollider(mesh: types.Mesh, collisions: bool) types.ColliderInput {
    return .{
        .shape = .{ .mesh = mesh },
        .collisions = collisions,
        .size = meshBounds(mesh.positions),
    };
}

fn meshBounds(positions: []const types.Vec3) types.Vec3 {
    if (positions.len == 0) return @splat(0.5);
    var min_v: types.Vec3 = positions[0];
    var max_v: types.Vec3 = positions[0];
    for (positions[1..]) |v| {
        min_v[0] = @min(min_v[0], v[0]);
        min_v[1] = @min(min_v[1], v[1]);
        min_v[2] = @min(min_v[2], v[2]);
        max_v[0] = @max(max_v[0], v[0]);
        max_v[1] = @max(max_v[1], v[1]);
        max_v[2] = @max(max_v[2], v[2]);
    }
    return .{ (max_v[0] - min_v[0]) * 0.5, (max_v[1] - min_v[1]) * 0.5, (max_v[2] - min_v[2]) * 0.5 };
}

pub fn addEntity(db: *Db, input: EntityInput) !u32 {
    const id = try asU32(db.entities.items.len);
    const name = try putString(db, input.name);
    const asset = try putString(db, input.asset_file);
    for (input.colliders) |collider| {
        var row = std.mem.zeroes(database.Collider);
        row.owner = id;
        row.collisions = @intFromBool(collider.collisions);
        row.pos = collider.pose.pos;
        row.quat = collider.pose.quat;
        row.size = collider.size;
        row.drag = collider.drag;
        row.skin = collider.skin;
        row.friction = collider.friction;
        row.restitution = collider.restitution;
        switch (collider.shape) {
            .box => row.shape = 0,
            .sphere => row.shape = 1,
            .cylinder => row.shape = 2,
            .mesh => |mesh| {
                if (mesh.positions.len < 4 or mesh.triangles.len == 0) return error.InvalidDatabase;
                row.shape = 3;
                const mesh_vertices = try appendVertices(db, mesh.positions);
                const mesh_triangles = try appendTriangles(db, mesh.triangles);
                row.vertex_start = mesh_vertices.start;
                row.vertex_count = mesh_vertices.count;
                row.triangle_start = mesh_triangles.start;
                row.triangle_count = mesh_triangles.count;
            },
        }
        try db.colliders.append(db.allocator, row);
    }
    for (input.thrusters) |thruster| {
        var row = thruster;
        row.owner = id;
        try db.thrusters.append(db.allocator, row);
    }
    for (input.hulls) |patch| {
        if (patch.positions.len == 0 or patch.positions.len % 3 != 0) return error.InvalidDatabase;
        const hull_vertices = try appendVertices(db, patch.positions);
        var row = patch.hull;
        row.owner = id;
        row.vertex_start = hull_vertices.start;
        row.vertex_count = hull_vertices.count;
        try db.hulls.append(db.allocator, row);
    }
    for (input.sensors) |sensor| {
        var row = std.mem.zeroes(database.Sensor);
        row.owner = id;
        row.body = @intCast(@backingInt(sensor.body));
        row.id = sensor.id;
        row.enabled = @intFromBool(sensor.enabled);
        row.period_ticks = sensor.period_ticks;
        row.pos = sensor.pose.pos;
        row.quat = sensor.pose.quat;
        switch (sensor.settings) {
            .pose => |tracking| {
                row.kind = 0;
                row.position_axes = tracking.position_axes;
                row.attitude_axes = tracking.attitude_axes;
            },
            .imu => row.kind = 1,
            .pressure => row.kind = 2,
            .camera => |camera| {
                row.kind = 3;
                row.width = camera.width;
                row.height = camera.height;
                row.horizontal_fov = camera.horizontal_fov;
                row.near_range = camera.near_range;
                row.far_range = camera.far_range;
            },
        }
        try db.sensors.append(db.allocator, row);
    }

    var record = std.mem.zeroes(database.Entity);
    record.name_offset = name.offset;
    record.name_len = name.len;
    record.asset_offset = asset.offset;
    record.asset_len = asset.len;
    record.id = input.id;
    record.cls = input.cls;
    record.motion = switch (input.motion_type) {
        .static => 0,
        .dynamic => 1,
        .kinematic => 2,
    };
    record.tracked = @intFromBool(input.tracked);
    record.fuzz = @intFromBool(input.fuzz);
    record.mass = input.mass;
    record.inertia = input.inertia;
    record.com_pos = input.center_of_mass.pos;
    record.com_quat = input.center_of_mass.quat;
    record.initial_pos = input.initial_pose.pos;
    record.initial_quat = input.initial_pose.quat;
    record.initial_velocity = input.initial_velocity;
    record.initial_angular_velocity = input.initial_angular_velocity;
    record.dry_mass = input.dry_mass;
    record.volume = input.volume;
    record.buoyancy = input.buoyancy_center;
    try db.entities.append(db.allocator, record);
    try db.prev.append(db.allocator, database.nil);
    try db.next.append(db.allocator, database.nil);
    try db.generation.append(db.allocator, 1);
    db.active.append(db.prev.items, db.next.items, id);
    return id;
}

pub fn appendPresetName(db: *Db, name: []const u8, entity_start: u32, entity_count: u32) !void {
    const stored = try putString(db, name);
    try db.names.append(db.allocator, .{
        .name_offset = stored.offset,
        .name_len = stored.len,
        .entity_start = entity_start,
        .entity_count = entity_count,
    });
}

/// Runs after loading `scenes.bin` (embedded or file) to add or adjust entities in place.
pub fn patchLoadedCatalog(db: *Db) !void {
    _ = db;
}

/// Hand-authored preset when `sim_scene_catalog` is `.scratch`.
pub fn buildScratchPreset(db: *Db) !void {
    const preset_name = "robosub_course";
    const start: u32 = @intCast(db.entities.items.len);
    const props = [_]struct {
        label: []const u8,
        id: u32,
        cls: u32,
        pose: types.Pose,
        size: types.Vec3,
    }{
        .{ .label = "gate_a", .id = 101, .cls = 3, .pose = .{ .pos = .{ 2, 0.5, 4 } }, .size = .{ 0.4, 1.5, 0.2 } },
        .{ .label = "gate_b", .id = 102, .cls = 3, .pose = .{ .pos = .{ 2, 0.5, 8 } }, .size = .{ 0.4, 1.5, 0.2 } },
        .{ .label = "buoy_red", .id = 201, .cls = 1, .pose = .{ .pos = .{ -1.5, 0.3, 6 } }, .size = .{ 0.35, 0.35, 0.35 } },
        .{ .label = "buoy_blue", .id = 202, .cls = 2, .pose = .{ .pos = .{ 5, 0.3, 6 } }, .size = .{ 0.35, 0.35, 0.35 } },
    };
    for (props) |prop| {
        const box = database.boxMesh(prop.size[0], prop.size[1], prop.size[2]);
        var collider = meshCollider(.{ .positions = &box.positions, .triangles = &box.triangles }, true);
        collider.pose = prop.pose;
        _ = try addEntity(db, .{
            .name = prop.label,
            .id = prop.id,
            .cls = prop.cls,
            .tracked = true,
            .initial_pose = prop.pose,
            .colliders = (&collider)[0..1],
            // Blender mesh: meshFromBlob(@embedFile("meshes/gate_a.meshbin"))
        });
    }
    const count: u32 = @intCast(db.entities.items.len - start);
    try appendPresetName(db, preset_name, start, count);
}

fn appendVertices(db: *Db, points: []const types.Vec3) !struct { start: u32, count: u32 } {
    const start = try asU32(db.positions.items.len / 3);
    for (points) |vertex| try db.positions.appendSlice(db.allocator, &vertex);
    return .{ .start = start, .count = try asU32(points.len) };
}

fn appendTriangles(db: *Db, faces: []const types.Triangle) !struct { start: u32, count: u32 } {
    const start = try asU32(db.indices.items.len / 3);
    for (faces) |face| {
        const indices = [_]u32{ face.a, face.b, face.c };
        try db.indices.appendSlice(db.allocator, &indices);
    }
    return .{ .start = start, .count = try asU32(faces.len) };
}

fn putString(db: *Db, text: []const u8) !struct { offset: u32, len: u32 } {
    const offset = try asU32(db.strings.items.len);
    try db.strings.appendSlice(db.allocator, text);
    return .{ .offset = offset, .len = try asU32(text.len) };
}

fn asU32(value: usize) !u32 {
    if (value > std.math.maxInt(u32)) return error.InvalidDatabase;
    return @intCast(value);
}

// --- Runtime scene presets (owned by `Session`) ---

pub const ScenePreset = struct {
    name: []const u8,
    slots: []u32,
};

const embedded_scenes_bin = @embedFile("../scenes.bin");

pub fn allocScenePresets(allocator: std.mem.Allocator, db: *const Db) ![]ScenePreset {
    const presets = try allocator.alloc(ScenePreset, db.names.items.len);
    errdefer allocator.free(presets);
    for (db.names.items, 0..) |name, i| {
        const slots = try allocator.alloc(u32, name.entity_count);
        errdefer allocator.free(slots);
        for (0..name.entity_count) |j| slots[j] = name.entity_start + @as(u32, @intCast(j));
        presets[i] = .{
            .name = db.string(name.name_offset, name.name_len),
            .slots = slots,
        };
    }
    return presets;
}

pub fn deinitScenePresets(allocator: std.mem.Allocator, presets: []ScenePreset) void {
    for (presets) |preset| allocator.free(preset.slots);
    allocator.free(presets);
}

/// Alias for `export_mesh` build module (`encodeMeshBlob`).
pub const encode = encodeMeshBlob;

test "embedded scenes.bin catalog loads" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    var db = try database.Database.load(std.testing.allocator, embedded_scenes_bin);
    defer db.deinit();
    const presets = try allocScenePresets(std.testing.allocator, &db);
    defer deinitScenePresets(std.testing.allocator, presets);
    const p = blk: {
        for (presets) |*preset| if (std.mem.eql(u8, preset.name, "robosub_course")) break :blk preset;
        return error.TestExpectedEqual;
    };
    try std.testing.expect(p.slots.len > 0);
}

test "scratch catalog builds robosub_course" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    var db: Db = .{ .allocator = std.testing.allocator };
    defer db.deinit();
    try buildScratchPreset(&db);
    const presets = try allocScenePresets(std.testing.allocator, &db);
    defer deinitScenePresets(std.testing.allocator, presets);
    for (presets) |preset| if (std.mem.eql(u8, preset.name, "robosub_course")) return;
    return error.TestExpectedEqual;
}