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
    try testing.expectEqual(@as(usize, 14), preset.objects.len);
    try testing.expectEqual(@as(u32, 14), catalog.db.active.count);
    try testing.expectEqualStrings("gate opening", preset.objects[0].body.name);
    try testing.expect(!preset.objects[0].colliders[0].collisions);
    try testing.expect(preset.objects[1].body.fuzz);
    for (preset.objects) |object| {
        try testing.expect(object.colliders.len == 1);
        try testing.expect(object.colliders[0].shape == .mesh);
        try testing.expect(object.colliders[0].shape.mesh.positions.len == 8);
        try testing.expect(object.colliders[0].shape.mesh.triangles.len == 12);
    }

    var robot_catalog = try RobotCatalog.init(testing.allocator);
    defer robot_catalog.deinit();
    const robot_index = robot_catalog.index("proteus") orelse return error.MissingRobot;
    const robot = &robot_catalog.robots[robot_index];
    const membership = [_]database.Membership{
        .{ .db = &robot_catalog.db, .slots = robot_catalog.slots[robot_index..][0..1] },
        .{ .db = &catalog.db, .slots = preset.slots },
    };
    var scene = try Scene.init(testing.allocator, robot, null, preset.objects, .{}, &.{}, &membership);
    defer scene.deinit();
    try testing.expect(scene.objects.len == preset.objects.len + 1);
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
