const std = @import("std");
const types = @import("types.zig");
const database = @import("database.zig");

pub const RobotCatalog = struct {
    allocator: std.mem.Allocator,
    db: database.Database,
    robots: []types.Robot,
    slots: []u32,

    pub fn init(allocator: std.mem.Allocator) !RobotCatalog {
        return initFromBytes(allocator, @embedFile("robots.bin"));
    }

    pub fn initFromBytes(allocator: std.mem.Allocator, bytes: []const u8) !RobotCatalog {
        var db = try database.Database.load(allocator, bytes);
        errdefer db.deinit();
        const robots = try allocator.alloc(types.Robot, db.names.items.len);
        errdefer allocator.free(robots);
        const slots = try allocator.alloc(u32, db.names.items.len);
        errdefer allocator.free(slots);
        var built: usize = 0;
        errdefer for (robots[0..built]) |robot| database.freeRobot(allocator, robot);

        for (db.names.items, 0..) |name, i| {
            if (name.entity_count != 1) return error.InvalidDatabase;
            slots[i] = name.entity_start;
            robots[i] = try db.robotView(allocator, name.entity_start);
            built += 1;
        }
        return .{ .allocator = allocator, .db = db, .robots = robots, .slots = slots };
    }

    pub fn index(self: *const RobotCatalog, name: []const u8) ?usize {
        for (self.robots, 0..) |robot, i| if (std.mem.eql(u8, name, robot.name)) return i;
        return null;
    }

    pub fn find(self: *const RobotCatalog, name: []const u8) ?*const types.Robot {
        const i = self.index(name) orelse return null;
        return &self.robots[i];
    }

    pub fn deinit(self: *RobotCatalog) void {
        var i = self.robots.len;
        while (i > 0) {
            i -= 1;
            database.freeRobot(self.allocator, self.robots[i]);
        }
        self.allocator.free(self.slots);
        self.allocator.free(self.robots);
        self.db.deinit();
        self.* = undefined;
    }
};
