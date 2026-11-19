const std = @import("std");
const database = @import("database.zig");

pub const RobotCatalog = struct {
    allocator: std.mem.Allocator,
    db: database.Database,
    slots: []u32,

    pub fn init(allocator: std.mem.Allocator) !RobotCatalog {
        return initFromBytes(allocator, @embedFile("robots.bin"));
    }

    pub fn initFromBytes(allocator: std.mem.Allocator, bytes: []const u8) !RobotCatalog {
        var db = try database.Database.load(allocator, bytes);
        errdefer db.deinit();
        const slots = try allocator.alloc(u32, db.names.items.len);
        errdefer allocator.free(slots);
        for (db.names.items, 0..) |name, i| {
            if (name.entity_count != 1) return error.InvalidDatabase;
            slots[i] = name.entity_start;
        }
        return .{ .allocator = allocator, .db = db, .slots = slots };
    }

    pub fn body(self: *const RobotCatalog, robot_index: usize) database.BodySlot {
        return .{ .db = &self.db, .slot = self.slots[robot_index] };
    }

    pub fn index(self: *const RobotCatalog, name: []const u8) ?usize {
        for (self.slots, 0..) |slot, i| {
            if (std.mem.eql(u8, name, database.entityName(&self.db, slot))) return i;
        }
        return null;
    }

    pub fn find(self: *const RobotCatalog, name: []const u8) ?database.BodySlot {
        const i = self.index(name) orelse return null;
        return self.body(i);
    }

    pub fn deinit(self: *RobotCatalog) void {
        self.allocator.free(self.slots);
        self.db.deinit();
        self.* = undefined;
    }
};