//! In-memory catalog of simulation objects. Each entity is a slot in parallel
//! record columns and stores its own intrusive-list links. Slots are u32 ids.
//! Child rows point at their owner slot. Geometry is packed xyz floats and
//! triangle indices. Links and generation stay in memory; the file stores the
//! columns only. Runtime loads frozen bytes via `load` (exact-size slices; no growth).
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

pub const Membership = struct {
    db: *Database,
    slots: []const u32,
};

/// Body row in a specific loaded catalog (required when a scene joins multiple DBs).
pub const BodySlot = struct {
    db: *const Database,
    slot: u32,
};

pub const ScenePreset = struct {
    name: []const u8,
    slots: []u32,
};

pub fn allocScenePresets(allocator: std.mem.Allocator, db: *const Database) ![]ScenePreset {
    const presets = try allocator.alloc(ScenePreset, db.names.len);
    errdefer allocator.free(presets);
    for (db.names, 0..) |name, i| {
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

pub fn entity(slot: BodySlot) *const Entity {
    return &slot.db.entities[slot.slot];
}

pub fn entityName(db: *const Database, slot: u32) []const u8 {
    const e = &db.entities[slot];
    return db.string(e.name_offset, e.name_len);
}

pub fn entityInitialPose(db: *const Database, slot: u32, spawn_override: ?types.Pose) types.Pose {
    if (spawn_override) |pose| return pose;
    const e = &db.entities[slot];
    return poseOf(e.initial_pos, e.initial_quat);
}

pub fn entityCenterOfMass(db: *const Database, slot: u32) types.Pose {
    const e = &db.entities[slot];
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
    return &db.colliders[ownedNth(Collider, db.colliders, owner, nth)];
}

pub fn thrusterAt(db: *const Database, owner: u32, nth: usize) *const Thruster {
    return &db.thrusters[ownedNth(Thruster, db.thrusters, owner, nth)];
}

pub fn hullAt(db: *const Database, owner: u32, nth: usize) *const Hull {
    return &db.hulls[ownedNth(Hull, db.hulls, owner, nth)];
}

pub fn sensorAt(db: *const Database, owner: u32, nth: usize) *const Sensor {
    return &db.sensors[ownedNth(Sensor, db.sensors, owner, nth)];
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
    strings: []u8 = &.{},
    entities: []Entity = &.{},
    prev: []u32 = &.{},
    next: []u32 = &.{},
    generation: []u32 = &.{},
    colliders: []Collider = &.{},
    thrusters: []Thruster = &.{},
    hulls: []Hull = &.{},
    sensors: []Sensor = &.{},
    positions: []f32 = &.{},
    indices: []u32 = &.{},
    names: []Name = &.{},
    active: IntrusiveList = .{},

    pub fn deinit(self: *Database) void {
        const a = self.allocator;
        a.free(self.names);
        a.free(self.indices);
        a.free(self.positions);
        a.free(self.sensors);
        a.free(self.hulls);
        a.free(self.thrusters);
        a.free(self.colliders);
        a.free(self.generation);
        a.free(self.next);
        a.free(self.prev);
        a.free(self.entities);
        a.free(self.strings);
        self.* = undefined;
    }

    pub fn string(self: *const Database, offset: u32, len: u32) []const u8 {
        return self.strings[offset..][0..len];
    }

    pub fn spliceActive(self: *Database, slots: []const u32) !void {
        for (slots, 0..) |id, i| {
            if (id >= self.entities.len) return error.InvalidSlot;
            for (slots[0..i]) |earlier| if (earlier == id) return error.InvalidSlot;
        }
        self.active.clear(self.prev, self.next);
        for (slots) |id| self.active.append(self.prev, self.next, id);
    }

    pub fn save(self: *const Database, allocator: std.mem.Allocator) ![]u8 {
        const columns = [_][]const u8{
            self.strings,
            std.mem.sliceAsBytes(self.entities),
            std.mem.sliceAsBytes(self.colliders),
            std.mem.sliceAsBytes(self.thrusters),
            std.mem.sliceAsBytes(self.hulls),
            std.mem.sliceAsBytes(self.sensors),
            std.mem.sliceAsBytes(self.positions),
            std.mem.sliceAsBytes(self.indices),
            std.mem.sliceAsBytes(self.names),
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
        for (entries, columns, 0..) |entry, column, i| {
            const at = directory_at + i * entry_size;
            std.mem.writeInt(u32, out[at..][0..4], entry.offset, .little);
            std.mem.writeInt(u32, out[at + 4 ..][0..4], entry.count, .little);
            std.mem.writeInt(u32, out[at + 8 ..][0..4], entry.stride, .little);
            if (column.len != 0) @memcpy(out[entry.offset..][0..column.len], column);
        }
        return out;
    }

    pub fn load(allocator: std.mem.Allocator, bytes: []const u8) !Database {
        const entries = try readHeader(bytes);
        var db: Database = .{ .allocator = allocator };
        errdefer db.deinit();
        db.strings = try copyColumn(u8, allocator, bytes, entries[table.strings]);
        db.entities = try copyColumn(Entity, allocator, bytes, entries[table.entities]);
        db.colliders = try copyColumn(Collider, allocator, bytes, entries[table.colliders]);
        db.thrusters = try copyColumn(Thruster, allocator, bytes, entries[table.thrusters]);
        db.hulls = try copyColumn(Hull, allocator, bytes, entries[table.hulls]);
        db.sensors = try copyColumn(Sensor, allocator, bytes, entries[table.sensors]);
        db.positions = try copyColumn(f32, allocator, bytes, entries[table.positions]);
        db.indices = try copyColumn(u32, allocator, bytes, entries[table.indices]);
        db.names = try copyColumn(Name, allocator, bytes, entries[table.names]);
        db.prev = try allocator.alloc(u32, db.entities.len);
        db.next = try allocator.alloc(u32, db.entities.len);
        db.generation = try allocator.alloc(u32, db.entities.len);
        @memset(db.prev, nil);
        @memset(db.next, nil);
        @memset(db.generation, 1);
        for (0..db.entities.len) |id| db.active.append(db.prev, db.next, @intCast(id));
        try db.validate();
        return db;
    }

    fn validate(self: *const Database) !void {
        if (self.positions.len % 3 != 0 or self.indices.len % 3 != 0) return error.InvalidDatabase;
        const vertex_total: u64 = self.positions.len / 3;
        const triangle_total: u64 = self.indices.len / 3;
        for (self.positions) |component| if (!std.math.isFinite(component)) return error.InvalidDatabase;
        for (self.entities) |row| {
            try bounds(self.strings.len, row.name_offset, row.name_len);
            try bounds(self.strings.len, row.asset_offset, row.asset_len);
            if (row.name_len == 0 or row.tracked > 1 or row.fuzz > 1 or row.motion > 2)
                return error.InvalidDatabase;
        }
        for (self.colliders) |collider| {
            if (collider.owner >= self.entities.len or collider.collisions > 1) return error.InvalidDatabase;
            switch (collider.shape) {
                0, 1, 2 => if (collider.vertex_count != 0 or collider.triangle_count != 0) return error.InvalidDatabase,
                3 => {
                    if (collider.vertex_count < 4 or collider.triangle_count == 0) return error.InvalidDatabase;
                    if (@as(u64, collider.vertex_start) + collider.vertex_count > vertex_total) return error.InvalidDatabase;
                    if (@as(u64, collider.triangle_start) + collider.triangle_count > triangle_total) return error.InvalidDatabase;
                    const start = @as(usize, collider.triangle_start) * 3;
                    const end = start + @as(usize, collider.triangle_count) * 3;
                    for (self.indices[start..end]) |index| if (index >= collider.vertex_count) return error.InvalidDatabase;
                },
                else => return error.InvalidDatabase,
            }
        }
        for (self.thrusters) |thruster| if (thruster.owner >= self.entities.len) return error.InvalidDatabase;
        for (self.hulls) |hull| {
            if (hull.owner >= self.entities.len or hull.vertex_count == 0 or hull.vertex_count % 3 != 0)
                return error.InvalidDatabase;
            if (@as(u64, hull.vertex_start) + hull.vertex_count > vertex_total) return error.InvalidDatabase;
        }
        for (self.sensors) |sensor| {
            if (sensor.owner >= self.entities.len or sensor.enabled > 1 or sensor.kind > 3)
                return error.InvalidDatabase;
            if (sensor.position_axes > 7 or sensor.attitude_axes > 7) return error.InvalidDatabase;
        }
        for (self.names) |name| {
            try bounds(self.strings.len, name.name_offset, name.name_len);
            if (name.name_len == 0 or name.entity_count == 0) return error.InvalidDatabase;
            if (@as(u64, name.entity_start) + name.entity_count > self.entities.len) return error.InvalidDatabase;
        }
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
        const byte_len = @as(u64, count) * strides[i];
        const end = @as(u64, cursor) + byte_len;
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

fn copyColumn(comptime T: type, allocator: std.mem.Allocator, bytes: []const u8, entry: TableEntry) ![]T {
    const slice = try allocator.alloc(T, entry.count);
    errdefer allocator.free(slice);
    if (entry.count != 0) {
        const raw = bytes[entry.offset..][0 .. @as(usize, entry.count) * @sizeOf(T)];
        @memcpy(std.mem.sliceAsBytes(slice), raw);
    }
    return slice;
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
    const floats = db.positions[@as(usize, start) * 3 ..][0 .. @as(usize, count) * 3];
    return std.mem.bytesAsSlice(types.Vec3, std.mem.sliceAsBytes(floats));
}

pub fn triangleSlice(db: *const Database, start: u32, count: u32) []const types.Triangle {
    const idxs = db.indices[@as(usize, start) * 3 ..][0 .. @as(usize, count) * 3];
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

test "embedded scenes.bin loads presets" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    const embedded = @embedFile("scenes.bin");
    var db = try Database.load(std.testing.allocator, embedded);
    defer db.deinit();
    const presets = try allocScenePresets(std.testing.allocator, &db);
    defer deinitScenePresets(std.testing.allocator, presets);
    var found = false;
    for (presets) |preset| {
        if (std.mem.eql(u8, preset.name, "robosub_course")) found = true;
    }
    try std.testing.expect(found);
}