const std = @import("std");
const database = @import("database.zig");

pub const SceneCatalog = struct {
    allocator: std.mem.Allocator,
    db: database.Database,
    scenes: []LoadedScene,

    pub const LoadedScene = struct {
        name: []const u8,
        slots: []u32,
    };

    pub fn init(allocator: std.mem.Allocator) !SceneCatalog {
        return initFromBytes(allocator, @embedFile("scenes.bin"));
    }

    pub fn initFromBytes(allocator: std.mem.Allocator, bytes: []const u8) !SceneCatalog {
        var db = try database.Database.load(allocator, bytes);
        errdefer db.deinit();
        const scenes = try allocator.alloc(LoadedScene, db.names.items.len);
        errdefer allocator.free(scenes);
        for (db.names.items, 0..) |name, i| {
            const slots = try allocator.alloc(u32, name.entity_count);
            errdefer allocator.free(slots);
            for (0..name.entity_count) |j| slots[j] = name.entity_start + @as(u32, @intCast(j));
            scenes[i] = .{
                .name = db.string(name.name_offset, name.name_len),
                .slots = slots,
            };
        }
        return .{ .allocator = allocator, .db = db, .scenes = scenes };
    }

    pub fn find(self: *const SceneCatalog, name: []const u8) ?*const LoadedScene {
        for (self.scenes) |*scene| if (std.mem.eql(u8, name, scene.name)) return scene;
        return null;
    }

    pub fn presetBodySlots(self: *const SceneCatalog, preset: *const LoadedScene, out: []database.BodySlot) void {
        for (preset.slots, out) |slot, *body| body.* = .{ .db = &self.db, .slot = slot };
    }

    pub fn deinit(self: *SceneCatalog) void {
        for (self.scenes) |scene| self.allocator.free(scene.slots);
        self.allocator.free(self.scenes);
        self.db.deinit();
        self.* = undefined;
    }
};

test "embedded scene catalog loads" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    var catalog = try SceneCatalog.init(std.testing.allocator);
    defer catalog.deinit();
    const preset = catalog.find("robosub_course") orelse return error.TestExpectedEqual;
    try std.testing.expect(preset.slots.len > 0);
}