//! In-memory catalog of simulation objects. Each entity is a slot in parallel
//! record columns and stores its own intrusive-list links. Slots are u32 ids.
//! Child rows point at their owner slot. Geometry is packed xyz floats and
//! triangle indices. Links and generation stay in memory; the file stores the
//! columns only, and load rebuilds the active list in entity order.
const std = @import("std");
const types = @import("types.zig");
const abi = @import("abi.zig");

pub const nil = abi.nil;
pub const magic = abi.magic;
pub const version = abi.version;
pub const table_count = abi.table_count;
pub const directory_at = abi.directory_at;
pub const entry_size = abi.entry_size;
pub const header_size = abi.header_size;
pub const table = abi.table;
pub const TableEntry = abi.TableEntry;
pub const Entity = abi.Entity;
pub const Collider = abi.Collider;
pub const Thruster = abi.Thruster;
pub const Hull = abi.Hull;
pub const Sensor = abi.Sensor;
pub const Name = abi.Name;
const strides = abi.strides;
const aligns = abi.aligns;

pub const Ref = struct { id: u32, generation: u32 };

pub const Membership = struct {
    db: *Database,
    slots: []const u32,
};

/// Body row in a specific loaded catalog (required when a scene joins multiple DBs).
pub const BodySlot = struct {
    db: *const Database,
    slot: u32,
};

pub fn entity(slot: BodySlot) *const Entity {
    return &slot.db.entities.items[slot.slot];
}

pub fn entityName(db: *const Database, slot: u32) []const u8 {
    const e = &db.entities.items[slot];
    return db.string(e.name_offset, e.name_len);
}

pub fn entityInitialPose(db: *const Database, slot: u32, spawn_override: ?types.Pose) types.Pose {
    if (spawn_override) |pose| return pose;
    const e = &db.entities.items[slot];
    return poseOf(e.initial_pos, e.initial_quat);
}

pub fn entityCenterOfMass(db: *const Database, slot: u32) types.Pose {
    const e = &db.entities.items[slot];
    return poseOf(e.com_pos, e.com_quat);
}

pub fn ownedNth(comptime Row: type, records: []const Row, owner: u32, nth: usize) usize {
    var seen: usize = 0;
    for (records, 0..) |record, i| {
        if (record.owner != owner) continue;
        if (seen == nth) return i;
        seen += 1;
    }
    unreachable;
}

pub fn colliderAt(db: *const Database, owner: u32, nth: usize) *const Collider {
    return &db.colliders.items[ownedNth(Collider, db.colliders.items, owner, nth)];
}

pub fn thrusterAt(db: *const Database, owner: u32, nth: usize) *const Thruster {
    return &db.thrusters.items[ownedNth(Thruster, db.thrusters.items, owner, nth)];
}

pub fn hullAt(db: *const Database, owner: u32, nth: usize) *const Hull {
    return &db.hulls.items[ownedNth(Hull, db.hulls.items, owner, nth)];
}

pub fn sensorAt(db: *const Database, owner: u32, nth: usize) *const Sensor {
    return &db.sensors.items[ownedNth(Sensor, db.sensors.items, owner, nth)];
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
    hulls: []const types.HullInput = &.{},
    sensors: []const types.SensorInput = &.{},
};

pub const IntrusiveList = struct {
    head: u32 = nil,
    tail: u32 = nil,
    count: u32 = 0,

    pub fn append(self: *IntrusiveList, prev: []u32, next: []u32, id: u32) void {
        next[id] = nil;
        prev[id] = self.tail;
        if (self.tail == nil) self.head = id else next[self.tail] = id;
        self.tail = id;
        self.count += 1;
    }

    pub fn unlink(self: *IntrusiveList, prev: []u32, next: []u32, id: u32) void {
        if (self.head != id and prev[id] == nil and next[id] == nil) return;
        if (prev[id] == nil) self.head = next[id] else next[prev[id]] = next[id];
        if (next[id] == nil) self.tail = prev[id] else prev[next[id]] = prev[id];
        prev[id] = nil;
        next[id] = nil;
        self.count -= 1;
    }

    pub fn clear(self: *IntrusiveList, prev: []u32, next: []u32) void {
        var id = self.head;
        while (id != nil) {
            const upcoming = next[id];
            prev[id] = nil;
            next[id] = nil;
            id = upcoming;
        }
        self.* = .{};
    }
};

pub const Database = struct {
    allocator: std.mem.Allocator,
    strings: std.ArrayList(u8) = .empty,
    entities: std.ArrayList(Entity) = .empty,
    prev: std.ArrayList(u32) = .empty,
    next: std.ArrayList(u32) = .empty,
    generation: std.ArrayList(u32) = .empty,
    colliders: std.ArrayList(Collider) = .empty,
    thrusters: std.ArrayList(Thruster) = .empty,
    hulls: std.ArrayList(Hull) = .empty,
    sensors: std.ArrayList(Sensor) = .empty,
    positions: std.ArrayList(f32) = .empty,
    indices: std.ArrayList(u32) = .empty,
    names: std.ArrayList(Name) = .empty,
    active: IntrusiveList = .{},

    pub fn init(allocator: std.mem.Allocator) Database {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *Database) void {
        self.names.deinit(self.allocator);
        self.indices.deinit(self.allocator);
        self.positions.deinit(self.allocator);
        self.sensors.deinit(self.allocator);
        self.hulls.deinit(self.allocator);
        self.thrusters.deinit(self.allocator);
        self.colliders.deinit(self.allocator);
        self.generation.deinit(self.allocator);
        self.next.deinit(self.allocator);
        self.prev.deinit(self.allocator);
        self.entities.deinit(self.allocator);
        self.strings.deinit(self.allocator);
        self.* = undefined;
    }

    pub fn string(self: *const Database, offset: u32, len: u32) []const u8 {
        return self.strings.items[offset..][0..len];
    }

    pub fn ref(self: *const Database, id: u32) Ref {
        return .{ .id = id, .generation = self.generation.items[id] };
    }

    pub fn live(self: *const Database, slot: Ref) ?u32 {
        if (slot.id >= self.generation.items.len) return null;
        if (self.generation.items[slot.id] != slot.generation) return null;
        return slot.id;
    }

    pub fn release(self: *Database, id: u32) void {
        self.active.unlink(self.prev.items, self.next.items, id);
        self.generation.items[id] +%= 1;
        if (self.generation.items[id] == 0) self.generation.items[id] = 1;
    }

    pub fn spliceActive(self: *Database, slots: []const u32) !void {
        for (slots, 0..) |id, i| {
            if (id >= self.entities.items.len) return error.InvalidSlot;
            for (slots[0..i]) |earlier| if (earlier == id) return error.InvalidSlot;
        }
        self.active.clear(self.prev.items, self.next.items);
        for (slots) |id| self.active.append(self.prev.items, self.next.items, id);
    }

    pub fn addEntity(self: *Database, input: EntityInput) !u32 {
        // New entity slot id equals current entity column length (append order).
        const id = try asU32(self.entities.items.len);

        // Copy display name and optional asset path into the shared string blob.
        const name = try self.putString(input.name);
        const asset = try self.putString(input.asset_file);

        // Child rows reference `owner = id` and append to their respective SIMDB columns.
        for (input.colliders) |collider| {
            var row = std.mem.zeroes(Collider);
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
                    const mesh_vertices = try self.appendVertices(mesh.positions);
                    const mesh_triangles = try self.appendTriangles(mesh.triangles);
                    row.vertex_start = mesh_vertices.start;
                    row.vertex_count = mesh_vertices.count;
                    row.triangle_start = mesh_triangles.start;
                    row.triangle_count = mesh_triangles.count;
                },
            }
            try self.colliders.append(self.allocator, row);
        }
        for (input.thrusters) |thruster| {
            var row = thruster;
            row.owner = id;
            try self.thrusters.append(self.allocator, row);
        }
        for (input.hulls) |hull| {
            if (hull.positions.len == 0 or hull.positions.len % 3 != 0) return error.InvalidDatabase;
            const hull_vertices = try self.appendVertices(hull.positions);
            try self.hulls.append(self.allocator, .{
                .owner = id,
                .body = @intCast(@backingInt(hull.body)),
                .pos = hull.pose.pos,
                .quat = hull.pose.quat,
                .drag = hull.drag,
                .skin = hull.skin,
                .centroid = hull.centroid,
                .coefficient_frame = hull.coefficient_frame,
                .volume = hull.volume,
                .vertex_start = hull_vertices.start,
                .vertex_count = hull_vertices.count,
            });
        }
        for (input.sensors) |sensor| {
            var row = std.mem.zeroes(Sensor);
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
            try self.sensors.append(self.allocator, row);
        }

        // Materialize the entity row (ABI layout) and register intrusive-list links.
        var record = std.mem.zeroes(Entity);
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
        try self.entities.append(self.allocator, record);
        try self.prev.append(self.allocator, nil);
        try self.next.append(self.allocator, nil);
        try self.generation.append(self.allocator, 1);
        self.active.append(self.prev.items, self.next.items, id);
        return id;
    }

    pub fn appendName(self: *Database, name: []const u8, entity_start: u32, entity_count: u32) !void {
        const stored = try self.putString(name);
        try self.names.append(self.allocator, .{
            .name_offset = stored.offset,
            .name_len = stored.len,
            .entity_start = entity_start,
            .entity_count = entity_count,
        });
    }

    pub fn save(self: *const Database, allocator: std.mem.Allocator) ![]u8 {
        const columns = [_][]const u8{
            self.strings.items,
            std.mem.sliceAsBytes(self.entities.items),
            std.mem.sliceAsBytes(self.colliders.items),
            std.mem.sliceAsBytes(self.thrusters.items),
            std.mem.sliceAsBytes(self.hulls.items),
            std.mem.sliceAsBytes(self.sensors.items),
            std.mem.sliceAsBytes(self.positions.items),
            std.mem.sliceAsBytes(self.indices.items),
            std.mem.sliceAsBytes(self.names.items),
        };
        var counts: [table_count]u32 = undefined;
        for (columns, 0..) |bytes, i| counts[i] = try asU32(bytes.len / strides[i]);
        const entries = try placeTables(counts);
        const end = payloadEnd(&entries);
        const out = try allocator.alloc(u8, end);
        errdefer allocator.free(out);
        @memset(out, 0);
        @memcpy(out[0..magic.len], magic);
        std.mem.writeInt(u32, out[magic.len..][0..4], version, .little);
        std.mem.writeInt(u32, out[magic.len + 4 ..][0..4], table_count, .little);
        for (entries, columns, 0..) |entry, bytes, i| {
            const at = directory_at + i * entry_size;
            std.mem.writeInt(u32, out[at..][0..4], entry.offset, .little);
            std.mem.writeInt(u32, out[at + 4 ..][0..4], entry.count, .little);
            std.mem.writeInt(u32, out[at + 8 ..][0..4], entry.stride, .little);
            if (bytes.len != 0) @memcpy(out[entry.offset..][0..bytes.len], bytes);
        }
        return out;
    }

    pub fn load(allocator: std.mem.Allocator, bytes: []const u8) !Database {
        const entries = try readHeader(bytes);
        var db = Database.init(allocator);
        errdefer db.deinit();
        try copyColumn(u8, allocator, &db.strings, bytes, entries[table.strings]);
        try copyColumn(Entity, allocator, &db.entities, bytes, entries[table.entities]);
        try copyColumn(Collider, allocator, &db.colliders, bytes, entries[table.colliders]);
        try copyColumn(Thruster, allocator, &db.thrusters, bytes, entries[table.thrusters]);
        try copyColumn(Hull, allocator, &db.hulls, bytes, entries[table.hulls]);
        try copyColumn(Sensor, allocator, &db.sensors, bytes, entries[table.sensors]);
        try copyColumn(f32, allocator, &db.positions, bytes, entries[table.positions]);
        try copyColumn(u32, allocator, &db.indices, bytes, entries[table.indices]);
        try copyColumn(Name, allocator, &db.names, bytes, entries[table.names]);
        try db.prev.resize(allocator, db.entities.items.len);
        try db.next.resize(allocator, db.entities.items.len);
        try db.generation.resize(allocator, db.entities.items.len);
        @memset(db.prev.items, nil);
        @memset(db.next.items, nil);
        @memset(db.generation.items, 1);
        for (0..db.entities.items.len) |id| db.active.append(db.prev.items, db.next.items, @intCast(id));
        try db.validate();
        return db;
    }

    fn validate(self: *const Database) !void {
        if (self.positions.items.len % 3 != 0 or self.indices.items.len % 3 != 0) return error.InvalidDatabase;
        const vertex_total: u64 = self.positions.items.len / 3;
        const triangle_total: u64 = self.indices.items.len / 3;
        for (self.positions.items) |component| if (!std.math.isFinite(component)) return error.InvalidDatabase;
        for (self.entities.items) |row| {
            try bounds(self.strings.items.len, row.name_offset, row.name_len);
            try bounds(self.strings.items.len, row.asset_offset, row.asset_len);
            if (row.name_len == 0 or row.tracked > 1 or row.fuzz > 1 or row.motion > 2)
                return error.InvalidDatabase;
        }
        for (self.colliders.items) |collider| {
            if (collider.owner >= self.entities.items.len or collider.collisions > 1) return error.InvalidDatabase;
            switch (collider.shape) {
                0, 1, 2 => if (collider.vertex_count != 0 or collider.triangle_count != 0) return error.InvalidDatabase,
                3 => {
                    if (collider.vertex_count < 4 or collider.triangle_count == 0) return error.InvalidDatabase;
                    if (@as(u64, collider.vertex_start) + collider.vertex_count > vertex_total) return error.InvalidDatabase;
                    if (@as(u64, collider.triangle_start) + collider.triangle_count > triangle_total) return error.InvalidDatabase;
                    const start = @as(usize, collider.triangle_start) * 3;
                    const end = start + @as(usize, collider.triangle_count) * 3;
                    for (self.indices.items[start..end]) |index| if (index >= collider.vertex_count) return error.InvalidDatabase;
                },
                else => return error.InvalidDatabase,
            }
        }
        for (self.thrusters.items) |thruster| if (thruster.owner >= self.entities.items.len) return error.InvalidDatabase;
        for (self.hulls.items) |hull| {
            if (hull.owner >= self.entities.items.len or hull.vertex_count == 0 or hull.vertex_count % 3 != 0)
                return error.InvalidDatabase;
            if (@as(u64, hull.vertex_start) + hull.vertex_count > vertex_total) return error.InvalidDatabase;
        }
        for (self.sensors.items) |sensor| {
            if (sensor.owner >= self.entities.items.len or sensor.enabled > 1 or sensor.kind > 3)
                return error.InvalidDatabase;
            if (sensor.position_axes > 7 or sensor.attitude_axes > 7) return error.InvalidDatabase;
        }
        for (self.names.items) |name| {
            try bounds(self.strings.items.len, name.name_offset, name.name_len);
            if (name.name_len == 0 or name.entity_count == 0) return error.InvalidDatabase;
            if (@as(u64, name.entity_start) + name.entity_count > self.entities.items.len) return error.InvalidDatabase;
        }
    }

    fn appendVertices(self: *Database, points: []const types.Vec3) !struct { start: u32, count: u32 } {
        const start = try asU32(self.positions.items.len / 3);
        for (points) |vertex| try self.positions.appendSlice(self.allocator, &vertex);
        return .{ .start = start, .count = try asU32(points.len) };
    }

    fn appendTriangles(self: *Database, faces: []const types.Triangle) !struct { start: u32, count: u32 } {
        const start = try asU32(self.indices.items.len / 3);
        for (faces) |face| {
            const indices = [_]u32{ face.a, face.b, face.c };
            try self.indices.appendSlice(self.allocator, &indices);
        }
        return .{ .start = start, .count = try asU32(faces.len) };
    }

    fn putString(self: *Database, text: []const u8) !struct { offset: u32, len: u32 } {
        const offset = try asU32(self.strings.items.len);
        try self.strings.appendSlice(self.allocator, text);
        return .{ .offset = offset, .len = try asU32(text.len) };
    }
};

pub fn readHeader(bytes: []const u8) ![table_count]TableEntry {
    if (bytes.len < header_size or !std.mem.eql(u8, bytes[0..magic.len], magic)) return error.InvalidDatabase;
    if (std.mem.readInt(u32, bytes[magic.len..][0..4], .little) != version) return error.UnsupportedDatabaseVersion;
    if (std.mem.readInt(u32, bytes[magic.len + 4 ..][0..4], .little) != table_count) return error.InvalidDatabase;
    var counts: [table_count]u32 = undefined;
    for (0..table_count) |i| {
        const at = directory_at + i * entry_size;
        const stride = std.mem.readInt(u32, bytes[at + 8 ..][0..4], .little);
        if (stride != strides[i]) return error.InvalidDatabase;
        counts[i] = std.mem.readInt(u32, bytes[at + 4 ..][0..4], .little);
    }
    const expected = try placeTables(counts);
    for (expected, 0..) |entry, i| {
        const at = directory_at + i * entry_size;
        const offset = std.mem.readInt(u32, bytes[at..][0..4], .little);
        if (offset != entry.offset) return error.InvalidDatabase;
        if (@as(u64, entry.offset) + @as(u64, entry.count) * entry.stride > bytes.len) return error.InvalidDatabase;
    }
    if (payloadEnd(&expected) != bytes.len) return error.InvalidDatabase;
    return expected;
}

pub fn boxMesh(hx: f32, hy: f32, hz: f32) struct { positions: [8]types.Vec3, triangles: [12]types.Triangle } {
    const positions = [_]types.Vec3{
        .{ -hx, -hy, -hz }, .{ hx, -hy, -hz }, .{ hx, hy, -hz }, .{ -hx, hy, -hz },
        .{ -hx, -hy, hz },  .{ hx, -hy, hz },  .{ hx, hy, hz },  .{ -hx, hy, hz },
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

fn placeTables(counts: [table_count]u32) ![table_count]TableEntry {
    var cursor: u32 = @intCast(header_size);
    var entries: [table_count]TableEntry = undefined;
    for (counts, 0..) |count, i| {
        cursor = alignForward(cursor, aligns[i]);
        entries[i] = .{ .offset = cursor, .count = count, .stride = strides[i] };
        const bytes = @as(u64, count) * strides[i];
        const end = @as(u64, cursor) + bytes;
        if (end > std.math.maxInt(u32)) return error.InvalidDatabase;
        cursor = @intCast(end);
    }
    return entries;
}

fn payloadEnd(entries: *const [table_count]TableEntry) usize {
    var end: usize = header_size;
    for (entries) |entry| {
        const table_end = @as(usize, entry.offset) + @as(usize, entry.count) * entry.stride;
        end = @max(end, table_end);
    }
    return end;
}

fn alignForward(value: u32, alignment: u32) u32 {
    const mask = alignment - 1;
    return (value + mask) & ~mask;
}

fn copyColumn(
    comptime T: type,
    allocator: std.mem.Allocator,
    list: *std.ArrayList(T),
    bytes: []const u8,
    entry: TableEntry,
) !void {
    const items = try allocator.alloc(T, entry.count);
    errdefer allocator.free(items);
    if (entry.count != 0) {
        const raw = bytes[entry.offset..][0 .. @as(usize, entry.count) * @sizeOf(T)];
        @memcpy(std.mem.sliceAsBytes(items), raw);
    }
    try list.appendSlice(allocator, items);
    allocator.free(items);
}

pub fn ownedCount(records: anytype, owner: u32) usize {
    var n: usize = 0;
    for (records) |record| {
        if (record.owner == owner) n += 1;
    }
    return n;
}

pub fn shapeFrom(db: *const Database, record: *const Collider) types.Shape {
    return switch (record.shape) {
        0 => .box,
        1 => .sphere,
        2 => .cylinder,
        3 => .{ .mesh = .{
            .positions = vertexSlice(db, record.vertex_start, record.vertex_count),
            .triangles = triangleSlice(db, record.triangle_start, record.triangle_count),
        } },
        else => unreachable,
    };
}

pub fn vertexSlice(db: *const Database, start: u32, count: u32) []const types.Vec3 {
    const floats = db.positions.items[@as(usize, start) * 3 ..][0 .. @as(usize, count) * 3];
    return std.mem.bytesAsSlice(types.Vec3, std.mem.sliceAsBytes(floats));
}

pub fn triangleSlice(db: *const Database, start: u32, count: u32) []const types.Triangle {
    const idxs = db.indices.items[@as(usize, start) * 3 ..][0 .. @as(usize, count) * 3];
    return std.mem.bytesAsSlice(types.Triangle, std.mem.sliceAsBytes(idxs));
}

fn bounds(len: usize, offset: u32, size: u32) !void {
    if (@as(u64, offset) + size > len) return error.InvalidDatabase;
}

fn asU32(value: usize) !u32 {
    if (value > std.math.maxInt(u32)) return error.InvalidDatabase;
    return @intCast(value);
}

pub fn motionFrom(value: u32) types.Motion {
    return switch (value) {
        0 => .static,
        1 => .dynamic,
        else => .kinematic,
    };
}

pub fn poseOf(pos: [3]f32, quat: [4]f32) types.Pose {
    return .{ .pos = pos, .quat = quat };
}
