const std = @import("std");
const types = @import("types.zig");
const database = @import("database.zig");

pub const SceneCatalog = struct {
    allocator: std.mem.Allocator,
    db: database.Database,
    scenes: []LoadedScene,

    pub const LoadedScene = struct {
        name: []const u8,
        objects: []types.Object,
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
        var built: usize = 0;
        errdefer for (scenes[0..built]) |scene| freeScene(allocator, scene);

        for (db.names.items, 0..) |name, i| {
            const objects = try allocator.alloc(types.Object, name.entity_count);
            errdefer allocator.free(objects);
            const slots = try allocator.alloc(u32, name.entity_count);
            errdefer allocator.free(slots);
            var hydrated: usize = 0;
            errdefer for (objects[0..hydrated]) |object| database.freeObject(allocator, object);
            for (0..name.entity_count) |j| {
                const slot = name.entity_start + @as(u32, @intCast(j));
                slots[j] = slot;
                objects[j] = try db.objectView(allocator, slot);
                hydrated += 1;
            }
            scenes[i] = .{
                .name = db.string(name.name_offset, name.name_len),
                .objects = objects,
                .slots = slots,
            };
            built += 1;
        }
        return .{ .allocator = allocator, .db = db, .scenes = scenes };
    }

    pub fn find(self: *const SceneCatalog, name: []const u8) ?*const LoadedScene {
        for (self.scenes) |*scene| if (std.mem.eql(u8, name, scene.name)) return scene;
        return null;
    }

    pub fn deinit(self: *SceneCatalog) void {
        var i = self.scenes.len;
        while (i > 0) {
            i -= 1;
            freeScene(self.allocator, self.scenes[i]);
        }
        self.allocator.free(self.scenes);
        self.db.deinit();
        self.* = undefined;
    }
};

fn freeScene(allocator: std.mem.Allocator, scene: SceneCatalog.LoadedScene) void {
    var i = scene.objects.len;
    while (i > 0) {
        i -= 1;
        database.freeObject(allocator, scene.objects[i]);
    }
    allocator.free(scene.objects);
    allocator.free(scene.slots);
}
