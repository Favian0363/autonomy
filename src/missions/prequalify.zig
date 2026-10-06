const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");

pub fn mission(ctx: *MissionContext) !void {
    const gate_object = try ctx.yieldUntilFirstObjectWithCls(.gate);
    try goThrough(ctx, gate_object);

    const cube_or_rect_object = try ctx.yieldUntilFirstObjectWithAnyCls(&.{.cube, .rect});
    try goAround(ctx, cube_or_rect_object);
}

const FAR_ENOUGH: f64 = 2.0;
const OVERSHOOT: f64 = 2.0;

fn goThrough(ctx: *MissionContext, object: *const Auv.Object) !void {
    const sub_pose = ctx.frame.camera_pose;

    const object_pos = object.bbox.pose.pos;
    const object_pos_2d = math.xy(object_pos);
    const sub_pos_2d = math.xy(sub_pose.pos);

    const direction_2d = math.normalize2f(object_pos_2d - sub_pos_2d);

    const before_2d = object_pos_2d - direction_2d * @as(math.Vector2f, @splat(FAR_ENOUGH));
    const before: math.Vector3f = .{before_2d[0], before_2d[1], object_pos[2]};

    std.log.info("before object_pos {any}", .{object_pos_2d});
    std.log.info("before sub_pos {any}", .{sub_pos_2d});
    std.log.info("before {any}", .{before});

    try ctx.yieldUntilReachGoal(before);

    const overshoot_2d = object_pos_2d + direction_2d * @as(math.Vector2f, @splat(OVERSHOOT));
    const overshoot: math.Vector3f = .{overshoot_2d[0], overshoot_2d[1], object_pos[2]};

    try ctx.yieldUntilReachGoal(overshoot);
}

fn goAround(ctx: *MissionContext, object: *const Auv.Object) !void {
    const object_pos = object.bbox.pose.pos;
    const object_rot = object.bbox.pose.quat;

    // corners of a square centered in 0, with area 1
    const square_corners: [4]math.Vector2f = .{
        .{0.5, 0.5},
        .{0.5, -0.5},
        .{-0.5, -0.5},
        .{-0.5, 0.5},
    };

    // absolute distance from any corner of the object
    const DISTANCE_TO_CORNER: f32 = 2.0;
    // a big number so that any corner is always closer
    const HUGE_NUMBER: f32 = 10000000.0;

    var corner_pluss: [4]math.Vector3f = undefined;
    var starting_corner: math.Vector2f = .{HUGE_NUMBER, HUGE_NUMBER};
    var starting_i: usize = std.math.maxInt(usize);
    const initial_sub_pos = ctx.frame.camera_pose.pos;
    for (square_corners, 0..) |square_corner, i| {
        const sub_pose = ctx.frame.camera_pose;
        const rot_unit = math.normalize4f(object_rot);
        const actual_corner_2d = square_corner * math.xy(object.bbox.size);
        const actual_corner: math.Vector3f = .{actual_corner_2d[0], actual_corner_2d[1], 0};
        const rotated_corner = math.quaternionRotate(rot_unit, actual_corner);
        const rotated_corner_2d = math.xy(rotated_corner);
        const pos2d = math.xy(object_pos);
        const sub2d = math.xy(sub_pose.pos);
        const corner_plus =
            pos2d + rotated_corner_2d + math.normalize2f(rotated_corner_2d) * @as(math.Vector2f, @splat(DISTANCE_TO_CORNER));
        corner_pluss[i] = .{corner_plus[0], corner_plus[1], sub_pose.pos[2]};

        if (math.length2f(corner_plus - sub2d) < math.length2f(starting_corner)) {
            starting_i = i;
            starting_corner = corner_plus;
        }
    }

    for (0..corner_pluss.len) |i| {
        try ctx.yieldUntilReachGoal(corner_pluss[(starting_i + i) % corner_pluss.len]);
    }
    try ctx.yieldUntilReachGoal(initial_sub_pos);
}
