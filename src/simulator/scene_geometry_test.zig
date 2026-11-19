const std = @import("std");
const database = @import("database.zig");
const SceneCatalog = @import("scenes.zig").SceneCatalog;
const Scene = @import("Scene.zig");
const RobotCatalog = @import("robots.zig").RobotCatalog;

test "the embedded scene catalog hydrates robosub_course meshes" {
    const testing = std.testing;
    var catalog = try SceneCatalog.init(testing.allocator);
    defer catalog.deinit();
    const preset = catalog.find("robosub_course") orelse return error.MissingScene;
    try testing.expectEqual(@as(usize, 14), preset.slots.len);
    try testing.expectEqual(@as(u32, 14), catalog.db.active.count);
    const gate: database.BodySlot = .{ .db = &catalog.db, .slot = preset.slots[0] };
    try testing.expectEqualStrings("gate opening", database.entityName(&catalog.db, preset.slots[0]));
    try testing.expect(database.colliderAt(&catalog.db, preset.slots[0], 0).collisions == 0);
    const second: database.BodySlot = .{ .db = &catalog.db, .slot = preset.slots[1] };
    try testing.expect(database.entity(second).fuzz != 0);
    for (preset.slots) |slot| {
        const count = database.ownedCount(catalog.db.colliders.items, slot);
        try testing.expect(count == 1);
        const c = database.colliderAt(&catalog.db, slot, 0);
        const shape = database.shapeFromRecord(&catalog.db, c);
        try testing.expect(shape == .mesh);
        try testing.expect(shape.mesh.positions.len == 8);
        try testing.expect(shape.mesh.triangles.len == 12);
    }
    _ = gate;

    var robot_catalog = try RobotCatalog.init(testing.allocator);
    defer robot_catalog.deinit();
    const robot_index = robot_catalog.index("proteus") orelse return error.MissingRobot;
    const robot = robot_catalog.body(robot_index);
    const membership = [_]database.Membership{
        .{ .db = &robot_catalog.db, .slots = robot_catalog.slots[robot_index..][0..1] },
        .{ .db = &catalog.db, .slots = preset.slots },
    };
    const extras = try testing.allocator.alloc(database.BodySlot, preset.slots.len);
    defer testing.allocator.free(extras);
    catalog.presetBodySlots(preset, extras);
    var scene = try Scene.init(testing.allocator, robot, null, extras, .{}, &.{}, &membership);
    defer scene.deinit();
    try testing.expect(scene.objects.len == preset.slots.len + 1);
    try testing.expectEqual(robot_catalog.slots[robot_index], robot_catalog.db.active.head);
    try testing.expectEqual(@as(u32, 1), robot_catalog.db.active.count);
    try testing.expectEqual(preset.slots[0], catalog.db.active.head);
    try testing.expectEqual(@as(u32, @intCast(preset.slots.len)), catalog.db.active.count);
}

test "scene catalog round trip keeps the embedded bytes" {
    const testing = std.testing;
    const embedded = @embedFile("scenes.bin");
    var catalog = try SceneCatalog.initFromBytes(testing.allocator, embedded);
    defer catalog.deinit();
    const again = try catalog.db.save(testing.allocator);
    defer testing.allocator.free(again);
    try testing.expectEqualSlices(u8, embedded, again);
}

test "scene catalog rejects a bad header and a truncated table" {
    const testing = std.testing;
    const embedded = @embedFile("scenes.bin");
    const bytes = try testing.allocator.dupe(u8, embedded);
    defer testing.allocator.free(bytes);
    bytes[0] ^= 1;
    try testing.expectError(error.InvalidDatabase, SceneCatalog.initFromBytes(testing.allocator, bytes));
    bytes[0] ^= 1;
    try testing.expectError(error.InvalidDatabase, SceneCatalog.initFromBytes(testing.allocator, bytes[0 .. bytes.len - 1]));

    const header = try database.readHeader(bytes);
    const indices = header[database.table.indices];
    std.mem.writeInt(u32, bytes[indices.offset..][0..4], std.math.maxInt(u32), .little);
    try testing.expectError(error.InvalidDatabase, SceneCatalog.initFromBytes(testing.allocator, bytes));
}