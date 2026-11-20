const std = @import("std");
const types = @import("types.zig");
const abi = @import("abi.zig");

pub fn v(x: f64, y: f64, z: f64) types.Vec3 {
    return .{ @floatCast(x), @floatCast(y), @floatCast(z) };
}

pub fn q(x: f64, y: f64, z: f64, w: f64) types.Quat {
    return .{ @floatCast(x), @floatCast(y), @floatCast(z), @floatCast(w) };
}

pub fn add(a: types.Vec3, b: types.Vec3) types.Vec3 {
    return @as(@Vector(3, f32), a) + @as(@Vector(3, f32), b);
}

pub fn sub(a: types.Vec3, b: types.Vec3) types.Vec3 {
    return @as(@Vector(3, f32), a) - @as(@Vector(3, f32), b);
}

pub fn scale(a: types.Vec3, s: f64) types.Vec3 {
    return v(@as(f64, a[0]) * s, @as(f64, a[1]) * s, @as(f64, a[2]) * s);
}

pub fn dot(a: types.Vec3, b: types.Vec3) f64 {
    return @as(f64, a[0]) * b[0] + @as(f64, a[1]) * b[1] + @as(f64, a[2]) * b[2];
}

pub fn norm(a: types.Vec3) f64 {
    return @sqrt(dot(a, a));
}

pub fn cross(a: types.Vec3, b: types.Vec3) types.Vec3 {
    return v(
        @as(f64, a[1]) * b[2] - @as(f64, a[2]) * b[1],
        @as(f64, a[2]) * b[0] - @as(f64, a[0]) * b[2],
        @as(f64, a[0]) * b[1] - @as(f64, a[1]) * b[0],
    );
}

pub fn conjugate(a: types.Quat) types.Quat {
    return q(-a[0], -a[1], -a[2], a[3]);
}

pub fn multiply(a: types.Quat, b: types.Quat) types.Quat {
    return q(
        @as(f64, a[3]) * b[0] + @as(f64, a[0]) * b[3] + @as(f64, a[1]) * b[2] - @as(f64, a[2]) * b[1],
        @as(f64, a[3]) * b[1] - @as(f64, a[0]) * b[2] + @as(f64, a[1]) * b[3] + @as(f64, a[2]) * b[0],
        @as(f64, a[3]) * b[2] + @as(f64, a[0]) * b[1] - @as(f64, a[1]) * b[0] + @as(f64, a[2]) * b[3],
        @as(f64, a[3]) * b[3] - @as(f64, a[0]) * b[0] - @as(f64, a[1]) * b[1] - @as(f64, a[2]) * b[2],
    );
}

pub fn rotate(a: types.Quat, b: types.Vec3) types.Vec3 {
    const u = v(a[0], a[1], a[2]);
    const twice_cross = scale(cross(u, b), 2);
    return add(b, add(scale(twice_cross, a[3]), cross(u, twice_cross)));
}

pub fn point(a: types.Pose, b: types.Vec3) types.Vec3 {
    return add(a.pos, rotate(a.quat, b));
}

pub fn compose(a: types.Pose, b: types.Pose) types.Pose {
    return .{ .pos = point(a, b.pos), .quat = multiply(a.quat, b.quat) };
}

pub fn finite3(a: types.Vec3) bool {
    return std.math.isFinite(a[0]) and std.math.isFinite(a[1]) and std.math.isFinite(a[2]);
}

pub fn finite4(a: types.Quat) bool {
    return std.math.isFinite(a[0]) and std.math.isFinite(a[1]) and std.math.isFinite(a[2]) and std.math.isFinite(a[3]);
}

pub fn validPose(a: types.Pose) error{InvalidPose}!void {
    var n: f64 = 0;
    for (a.quat) |x| n += @as(f64, x) * x;
    if (!finite3(a.pos) or !finite4(a.quat) or @abs(n - 1) >= 1e-4) return error.InvalidPose;
}

pub fn bodyPose(s: abi.BodyState) types.Pose {
    return .{ .pos = s.pose_pos, .quat = s.pose_quat };
}

pub fn addForce(w: *abi.Wrench, f: types.Vec3, p: types.Vec3, com: types.Vec3) void {
    w.force = add(w.force, f);
    w.torque = add(w.torque, cross(sub(p, com), f));
}
