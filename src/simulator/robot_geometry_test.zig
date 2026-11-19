const std = @import("std");
const database = @import("database.zig");
const types = @import("types.zig");
const RobotCatalog = @import("robots.zig").RobotCatalog;
const Scene = @import("Scene.zig");

test "the embedded catalog builds a scene for each robot preset" {
    const testing = std.testing;
    var catalog = try RobotCatalog.init(testing.allocator);
    defer catalog.deinit();
    try testing.expect(catalog.find("unknown") == null);
    try testing.expectEqual(@as(u32, 4), catalog.db.active.count);
    for ([_][]const u8{ "hydrus", "proteus", "bluerov", "girona500" }, 0..) |name, i| {
        const robot = catalog.find(name) orelse return error.MissingRobot;
        const membership = [_]database.Membership{
            .{ .db = &catalog.db, .slots = catalog.slots[i..][0..1] },
        };
        var scene = try Scene.init(testing.allocator, robot, null, &.{}, .{}, &.{}, &membership);
        defer scene.deinit();
        try testing.expectEqualStrings(name, database.entityName(robot.db, robot.slot));
        const thrusters = database.ownedCount(robot.db.thrusters.items, robot.slot);
        try testing.expect(thrusters <= types.thruster_count);
        try testing.expectEqual(@as(u8, @intCast(thrusters)), scene.thruster_len);
        try testing.expectEqual(catalog.slots[i], catalog.db.active.head);
        try testing.expectEqual(@as(u32, 1), catalog.db.active.count);
    }
    const proteus = catalog.find("proteus").?;
    const pe = database.entity(proteus);
    try testing.expectEqual(@as(u64, 0x404feb7b0b640c44), @as(u64, @bitCast(pe.mass)));
    try testing.expectEqual(@as(usize, 6), database.ownedCount(proteus.db.thrusters.items, proteus.slot));
    try testing.expectEqual(@as(usize, 1), database.ownedCount(proteus.db.hulls.items, proteus.slot));
    const c0 = database.colliderAt(proteus.db, proteus.slot, 0);
    try testing.expect(database.shapeFromRecord(proteus.db, c0) == .mesh);
    try testing.expect(database.shapeFromRecord(proteus.db, c0).mesh.positions.len >= 4);
    const h0 = database.hullAt(proteus.db, proteus.slot, 0);
    try testing.expect(database.vertexSlice(proteus.db, h0.vertex_start, h0.vertex_count).len % 3 == 0);
}

test "catalog round trip keeps the embedded bytes" {
    const testing = std.testing;
    const embedded = @embedFile("robots.bin");
    var catalog = try RobotCatalog.initFromBytes(testing.allocator, embedded);
    defer catalog.deinit();
    const again = try catalog.db.save(testing.allocator);
    defer testing.allocator.free(again);
    try testing.expectEqualSlices(u8, embedded, again);
}

test "catalog rejects invalid headers, truncated tables, and triangle indices" {
    const testing = std.testing;
    const embedded = @embedFile("robots.bin");
    const bytes = try testing.allocator.dupe(u8, embedded);
    defer testing.allocator.free(bytes);

    bytes[0] ^= 1;
    try testing.expectError(error.InvalidDatabase, RobotCatalog.initFromBytes(testing.allocator, bytes));
    bytes[0] ^= 1;
    bytes[database.magic.len] ^= 1;
    try testing.expectError(error.UnsupportedDatabaseVersion, RobotCatalog.initFromBytes(testing.allocator, bytes));
    bytes[database.magic.len] ^= 1;
    try testing.expectError(error.InvalidDatabase, RobotCatalog.initFromBytes(testing.allocator, bytes[0 .. bytes.len - 1]));

    const header = try database.readHeader(bytes);
    const indices = header[database.table.indices];
    std.mem.writeInt(u32, bytes[indices.offset..][0..4], std.math.maxInt(u32), .little);
    try testing.expectError(error.InvalidDatabase, RobotCatalog.initFromBytes(testing.allocator, bytes));
}

test "slots stay addressable until release bumps the generation" {
    const testing = std.testing;
    var db = database.Database.init(testing.allocator);
    defer db.deinit();
    const id = try db.addEntity(.{
        .name = "buoy",
        .id = 4,
        .colliders = &.{.{ .shape = .box, .size = .{ 1, 2, 3 } }},
    });
    try db.appendName("course", id, 1);
    const bytes = try db.save(testing.allocator);
    defer testing.allocator.free(bytes);
    var loaded = try database.Database.load(testing.allocator, bytes);
    defer loaded.deinit();
    try testing.expectEqual(@as(u32, 1), loaded.active.count);
    try testing.expectEqual(id, loaded.active.head);
    try testing.expectEqualStrings("buoy", database.entityName(&loaded, id));
    const c = database.colliderAt(&loaded, id, 0);
    try testing.expect(database.shapeFromRecord(&loaded, c) == .box);
    try testing.expectEqual(types.Vec3{ 1, 2, 3 }, c.size);
    const held = loaded.ref(id);
    try testing.expectEqual(id, loaded.live(held).?);
    loaded.release(id);
    try testing.expect(loaded.live(held) == null);
    try testing.expectEqual(database.nil, loaded.active.head);
}