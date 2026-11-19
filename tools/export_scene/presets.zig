//! Legacy robosub course boxes used until a .blend export replaces them.
pub const Entry = struct {
    id: u32,
    label: []const u8,
    cls: u32,
    tracked: bool,
    collisions: bool,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
};

pub const robosub_course = [_]Entry{
    .{ .id = 2, .label = "gate opening", .cls = 2, .tracked = true, .collisions = false, .x = 1.0465, .y = 8, .z = 2.3935, .hx = 1.4915, .hy = 0.05, .hz = 0.8865 },
    .{ .id = 3, .label = "marker", .cls = 1, .tracked = true, .collisions = true, .x = 0.9985, .y = 20.1525, .z = 1.1675, .hx = 0.2175, .hy = 0.5335, .hz = 0.1895 },
    .{ .id = 10, .label = "gate left post", .cls = 0, .tracked = true, .collisions = true, .x = 2.573, .y = 8, .z = 2.220, .hx = 0.035, .hy = 0.040, .hz = 0.783 },
    .{ .id = 11, .label = "gate right post", .cls = 0, .tracked = true, .collisions = true, .x = -0.480, .y = 8, .z = 2.220, .hx = 0.035, .hy = 0.040, .hz = 0.783 },
    .{ .id = 12, .label = "gate top bar", .cls = 0, .tracked = true, .collisions = true, .x = 1.0465, .y = 8, .z = 1.472, .hx = 1.5615, .hy = 0.040, .hz = 0.035 },
    .{ .id = 100, .label = "Pool west wall", .cls = 3, .tracked = true, .collisions = true, .x = 6.45, .y = 10, .z = 1.64, .hx = 0.15, .hy = 14.3, .hz = 1.64 },
    .{ .id = 101, .label = "Pool east wall", .cls = 3, .tracked = true, .collisions = true, .x = -6.45, .y = 10, .z = 1.64, .hx = 0.15, .hy = 14.3, .hz = 1.64 },
    .{ .id = 102, .label = "Pool south wall", .cls = 3, .tracked = true, .collisions = true, .x = 0, .y = -4.45, .z = 1.64, .hx = 6.6, .hy = 0.15, .hz = 1.64 },
    .{ .id = 103, .label = "Pool north wall", .cls = 3, .tracked = true, .collisions = true, .x = 0, .y = 24.45, .z = 1.64, .hx = 6.6, .hy = 0.15, .hz = 1.64 },
    .{ .id = 104, .label = "Pool floor", .cls = 4, .tracked = true, .collisions = true, .x = 0, .y = 10, .z = 3.38, .hx = 6.6, .hy = 14.6, .hz = 0.1 },
    .{ .id = 110, .label = "Bin", .cls = 5, .tracked = true, .collisions = true, .x = 2.677, .y = 10.0755, .z = 2.931, .hx = 0.305, .hy = 0.4575, .hz = 0.079 },
    .{ .id = 111, .label = "Bin_Copy", .cls = 5, .tracked = true, .collisions = true, .x = 1.677, .y = 10.5755, .z = 2.931, .hx = 0.305, .hy = 0.4575, .hz = 0.079 },
    .{ .id = 112, .label = "Board", .cls = 6, .tracked = true, .collisions = true, .x = 1.505, .y = 12, .z = 1, .hx = 0.005, .hy = 0.3, .hz = 0.3 },
    .{ .id = 113, .label = "GateSphere", .cls = 7, .tracked = true, .collisions = true, .x = -3.264, .y = 18.936, .z = 1.673, .hx = 0.35, .hy = 0.35, .hz = 0.35 },
};