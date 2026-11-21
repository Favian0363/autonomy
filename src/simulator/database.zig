//! In-memory robot and scene catalogs. Slots are entity rows. Child rows point at
//! their owner slot. Geometry is packed xyz floats and triangle indices.
//! `loadRobot` and `loadScene` read an AUTODNA1 document for that catalog struct.
const std = @import("std");
const types = @import("types.zig");
const abi = @import("abi.zig");
const reflection = @import("ReflectionWriter.zig");

pub const rna = reflection.rna;

pub const nil = abi.nil;
pub const Entity = abi.Entity;
pub const Collider = abi.Collider;
pub const Thruster = abi.Thruster;
pub const Hull = abi.Hull;
pub const Sensor = abi.Sensor;
pub const Name = abi.Name;
pub const RobotCatalog = abi.RobotCatalog;
pub const SceneCatalog = abi.SceneCatalog;

/// Geometry columns for one entity. Robot and scene catalogs both carry these slices.
pub const BodySlot = struct {
    strings: []const u8,
    entities: []const Entity,
    colliders: []const Collider,
    positions: []const f32,
    indices: []const u32,
    slot: u32,
};

pub const ScenePreset = struct {
    name: []const u8,
    slots: []u32,
};

/// `@embedFile` is byte-aligned. Catalog rows need their `extern` alignment, so the
/// process keeps one aligned image of the file and tables map into that image.
pub fn alignedEmbed(comptime file: anytype) []const u8 {
    const Storage = struct {
        var ready = false;
        var bytes: [file.len]u8 align(@alignOf(u64)) = undefined;
    };
    if (!Storage.ready) {
        @memcpy(Storage.bytes[0..], file);
        Storage.ready = true;
    }
    return &Storage.bytes;
}

pub fn bodySlot(catalog: anytype, slot: u32) BodySlot {
    return .{
        .strings = catalog.strings,
        .entities = catalog.entities,
        .colliders = catalog.colliders,
        .positions = catalog.positions,
        .indices = catalog.indices,
        .slot = slot,
    };
}

pub fn deinit(allocator: std.mem.Allocator, catalog: anytype, source: []const u8) void {
    reflection.free(@TypeOf(catalog.*), allocator, catalog, source);
}

pub fn loadRobot(allocator: std.mem.Allocator, bytes: []const u8) !RobotCatalog {
    var catalog = try reflection.read(RobotCatalog, allocator, bytes);
    errdefer deinit(allocator, &catalog, bytes);
    try validateGeometry(catalog.strings, catalog.entities, catalog.colliders, catalog.positions, catalog.indices, catalog.names);
    for (catalog.thrusters) |thruster| if (thruster.owner >= catalog.entities.len) return error.InvalidDatabase;
    for (catalog.hulls) |hull| {
        if (hull.owner >= catalog.entities.len or hull.vertex_count == 0 or hull.vertex_count % 3 != 0)
            return error.InvalidDatabase;
        if (@as(u64, hull.vertex_start) + hull.vertex_count > catalog.positions.len / 3) return error.InvalidDatabase;
    }
    for (catalog.sensors) |sensor| {
        if (sensor.owner >= catalog.entities.len or sensor.enabled > 1 or sensor.kind > 3)
            return error.InvalidDatabase;
        if (sensor.position_axes > 7 or sensor.attitude_axes > 7) return error.InvalidDatabase;
    }
    return catalog;
}

pub fn loadScene(allocator: std.mem.Allocator, bytes: []const u8) !SceneCatalog {
    var catalog = try reflection.read(SceneCatalog, allocator, bytes);
    errdefer deinit(allocator, &catalog, bytes);
    try validateGeometry(catalog.strings, catalog.entities, catalog.colliders, catalog.positions, catalog.indices, catalog.names);
    return catalog;
}

pub fn save(allocator: std.mem.Allocator, catalog: anytype) ![]align(@alignOf(u64)) u8 {
    return reflection.write(allocator, catalog);
}

pub fn allocScenePresets(allocator: std.mem.Allocator, catalog: *const SceneCatalog) ![]ScenePreset {
    const presets = try allocator.alloc(ScenePreset, catalog.names.len);
    errdefer allocator.free(presets);
    for (catalog.names, 0..) |name, i| {
        const slots = try allocator.alloc(u32, name.entity_count);
        errdefer allocator.free(slots);
        for (0..name.entity_count) |j| slots[j] = name.entity_start + @as(u32, @intCast(j));
        presets[i] = .{
            .name = catalog.strings[name.name_offset..][0..name.name_len],
            .slots = slots,
        };
    }
    return presets;
}

pub fn deinitScenePresets(allocator: std.mem.Allocator, presets: []ScenePreset) void {
    for (presets) |preset| allocator.free(preset.slots);
    allocator.free(presets);
}

pub fn entity(slot: BodySlot) *const Entity {
    return &slot.entities[slot.slot];
}

pub fn entityInitialPose(slot: BodySlot, spawn_override: ?types.Pose) types.Pose {
    if (spawn_override) |pose| return pose;
    const row = entity(slot);
    return poseOf(row.initial_pos, row.initial_quat);
}

pub fn entityCenterOfMass(slot: BodySlot) types.Pose {
    const row = entity(slot);
    return poseOf(row.com_pos, row.com_quat);
}

pub fn sensorKind(record: *const Sensor) types.SensorKind {
    return switch (record.kind) {
        0 => .pose,
        1 => .imu,
        2 => .pressure,
        3 => .camera,
        else => unreachable,
    };
}

pub fn boxMesh(hx: f64, hy: f64, hz: f64) struct { positions: [8]abi.Vec3, triangles: [12]types.Triangle } {
    const positions = [_]abi.Vec3{
        .{ @floatCast(-hx), @floatCast(-hy), @floatCast(-hz) }, .{ @floatCast(hx), @floatCast(-hy), @floatCast(-hz) }, .{ @floatCast(hx), @floatCast(hy), @floatCast(-hz) }, .{ @floatCast(-hx), @floatCast(hy), @floatCast(-hz) },
        .{ @floatCast(-hx), @floatCast(-hy), @floatCast(hz) },  .{ @floatCast(hx), @floatCast(-hy), @floatCast(hz) },  .{ @floatCast(hx), @floatCast(hy), @floatCast(hz) },  .{ @floatCast(-hx), @floatCast(hy), @floatCast(hz) },
    };
    const triangles = [_]types.Triangle{
        .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 0, .b = 2, .c = 3 },
        .{ .a = 4, .b = 6, .c = 5 }, .{ .a = 4, .b = 7, .c = 6 },
        .{ .a = 0, .b = 4, .c = 5 }, .{ .a = 0, .b = 5, .c = 1 },
        .{ .a = 2, .b = 6, .c = 7 }, .{ .a = 2, .b = 7, .c = 3 },
        .{ .a = 0, .b = 3, .c = 7 }, .{ .a = 0, .b = 7, .c = 4 },
        .{ .a = 1, .b = 5, .c = 6 }, .{ .a = 1, .b = 6, .c = 2 },
    };
    return .{ .positions = positions, .triangles = triangles };
}

pub fn validateEntitySlots(catalog: *const SceneCatalog, slots: []const u32) !void {
    for (slots, 0..) |id, i| {
        if (id >= catalog.entities.len) return error.InvalidSlot;
        for (slots[0..i]) |earlier| if (earlier == id) return error.InvalidSlot;
    }
}

pub fn shapeFrom(slot: BodySlot, record: *const Collider) types.Shape {
    return switch (record.shape) {
        0 => .box,
        1 => .sphere,
        2 => .cylinder,
        3 => .{ .mesh = .{
            .positions = rna.vec3s(slot.positions, record.vertex_start, record.vertex_count),
            .triangles = rna.triangles(slot.indices, record.triangle_start, record.triangle_count),
        } },
        else => unreachable,
    };
}

pub fn motionFrom(value: u32) types.Motion {
    return switch (value) {
        0 => .static,
        1 => .dynamic,
        else => .kinematic,
    };
}

pub fn poseOf(pos: [3]f32, quat: [4]f32) types.Pose {
    return .{
        .pos = .{ @floatCast(pos[0]), @floatCast(pos[1]), @floatCast(pos[2]) },
        .quat = .{ @floatCast(quat[0]), @floatCast(quat[1]), @floatCast(quat[2]), @floatCast(quat[3]) },
    };
}

fn validateGeometry(
    strings: []const u8,
    entities: []const Entity,
    colliders: []const Collider,
    positions: []const f32,
    indices: []const u32,
    names: []const Name,
) !void {
    if (positions.len % 3 != 0 or indices.len % 3 != 0) return error.InvalidDatabase;
    const vertex_total: u64 = positions.len / 3;
    const triangle_total: u64 = indices.len / 3;
    for (positions) |component| if (!std.math.isFinite(component)) return error.InvalidDatabase;
    for (entities) |row| {
        try bounds(strings.len, row.name_offset, row.name_len);
        try bounds(strings.len, row.asset_offset, row.asset_len);
        if (row.name_len == 0 or row.tracked > 1 or row.fuzz > 1 or row.motion > 2)
            return error.InvalidDatabase;
    }
    for (colliders) |collider| {
        if (collider.owner >= entities.len or collider.collisions > 1) return error.InvalidDatabase;
        switch (collider.shape) {
            0, 1, 2 => if (collider.vertex_count != 0 or collider.triangle_count != 0) return error.InvalidDatabase,
            3 => {
                if (collider.vertex_count < 4 or collider.triangle_count == 0) return error.InvalidDatabase;
                if (@as(u64, collider.vertex_start) + collider.vertex_count > vertex_total) return error.InvalidDatabase;
                if (@as(u64, collider.triangle_start) + collider.triangle_count > triangle_total) return error.InvalidDatabase;
                const start = @as(usize, collider.triangle_start) * 3;
                const end = start + @as(usize, collider.triangle_count) * 3;
                for (indices[start..end]) |index| if (index >= collider.vertex_count) return error.InvalidDatabase;
            },
            else => return error.InvalidDatabase,
        }
    }
    for (names) |name| {
        try bounds(strings.len, name.name_offset, name.name_len);
        if (name.name_len == 0 or name.entity_count == 0) return error.InvalidDatabase;
        if (@as(u64, name.entity_start) + name.entity_count > entities.len) return error.InvalidDatabase;
    }
}

fn bounds(len: usize, offset: u32, size: u32) !void {
    if (@as(u64, offset) + size > len) return error.InvalidDatabase;
}

