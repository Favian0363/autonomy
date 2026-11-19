//! Build-time SIMDB catalog writer (export_scene). Runtime loads frozen bytes via `database.zig`.
const std = @import("std");
const types = @import("types.zig");
const abi = @import("abi.zig");
const database = @import("database.zig");

pub const Database = database.Database;

/// `hull.owner`, `hull.vertex_start`, and `hull.vertex_count` are filled by `addEntity`.
pub const HullPatch = struct {
    hull: abi.Hull,
    positions: []const types.Vec3,
};

pub const EntityInput = struct {
    name: []const u8,
    asset_file: []const u8 = "",
    asset_hash: [32]u8 = @splat(0),
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
    hulls: []const HullPatch = &.{},
    sensors: []const types.SensorInput = &.{},
};

pub fn open(allocator: std.mem.Allocator) Database {
    return .{ .allocator = allocator };
}

pub fn addEntity(db: *Database, input: EntityInput) !u32 {
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
    record.asset_hash = input.asset_hash;
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

pub fn appendName(db: *Database, name: []const u8, entity_start: u32, entity_count: u32) !void {
    const stored = try putString(db, name);
    try db.names.append(db.allocator, .{
        .name_offset = stored.offset,
        .name_len = stored.len,
        .entity_start = entity_start,
        .entity_count = entity_count,
    });
}

fn appendVertices(db: *Database, points: []const types.Vec3) !struct { start: u32, count: u32 } {
    const start = try asU32(db.positions.items.len / 3);
    for (points) |vertex| try db.positions.appendSlice(db.allocator, &vertex);
    return .{ .start = start, .count = try asU32(points.len) };
}

fn appendTriangles(db: *Database, faces: []const types.Triangle) !struct { start: u32, count: u32 } {
    const start = try asU32(db.indices.items.len / 3);
    for (faces) |face| {
        const indices = [_]u32{ face.a, face.b, face.c };
        try db.indices.appendSlice(db.allocator, &indices);
    }
    return .{ .start = start, .count = try asU32(faces.len) };
}

fn putString(db: *Database, text: []const u8) !struct { offset: u32, len: u32 } {
    const offset = try asU32(db.strings.items.len);
    try db.strings.appendSlice(db.allocator, text);
    return .{ .offset = offset, .len = try asU32(text.len) };
}

fn asU32(value: usize) !u32 {
    if (value > std.math.maxInt(u32)) return error.InvalidDatabase;
    return @intCast(value);
}