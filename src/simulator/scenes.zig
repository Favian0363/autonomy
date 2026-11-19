const std = @import("std");
const database = @import("database.zig");
const catalog_emit = @import("catalog_emit.zig");

pub const SceneCatalog = struct {
    allocator: std.mem.Allocator,
    db: database.Database,
    scenes: []LoadedScene,

    pub const LoadedScene = struct {
        name: []const u8,
        slots: []u32,
    };

    pub const SceneCatalogSource = union(enum) {
        embedded,
        file_path: []const u8,
        blend: struct {
            path: []const u8,
            collection: []const u8,
            catalog_name: []const u8,
        },
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

    pub fn initFromFile(allocator: std.mem.Allocator, io: std.Io, path: []const u8) !SceneCatalog {
        const bytes = try std.Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(std.math.maxInt(usize)));
        defer allocator.free(bytes);
        return initFromBytes(allocator, bytes);
    }

    pub fn initResolved(allocator: std.mem.Allocator, io: std.Io, source: SceneCatalogSource) !SceneCatalog {
        return switch (source) {
            .embedded => init(allocator),
            .file_path => |path| initFromFile(allocator, io, path),
            .blend => |blend| blk: {
                const bytes = try catalog_emit.sceneCatalogBytesFromBlend(
                    allocator,
                    io,
                    blend.path,
                    blend.collection,
                    blend.catalog_name,
                );
                defer allocator.free(bytes);
                break :blk try initFromBytes(allocator, bytes);
            },
        };
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

test "scene catalog preset round trip" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    const allocator = std.testing.allocator;
    const objects = try catalog_emit.metadataObjectsFromPresets(allocator);
    defer allocator.free(objects);
    const bytes = try catalog_emit.writeCatalog(allocator, "robosub_course", objects);
    defer allocator.free(bytes);
    var catalog = try SceneCatalog.initFromBytes(allocator, bytes);
    defer catalog.deinit();
    const preset = catalog.find("robosub_course") orelse return error.TestExpectedEqual;
    try std.testing.expect(preset.slots.len > 0);
}