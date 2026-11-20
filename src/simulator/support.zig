//! f64 geometry for the simulator. SIMDB / session ABI vectors in `abi.zig` are f32;
//! widen on read and narrow on write only via `fromAbi*` / `toAbi*`.
const std = @import("std");
const types = @import("types.zig");
const abi = @import("abi.zig");

pub const Vec3 = types.Vec3;
pub const Quat = types.Quat;
pub const Pose = types.Pose;

pub fn fromAbi3(v: abi.Vec3) Vec3 {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]) };
}

pub fn fromAbi4(v: abi.Quat) Quat {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]), @floatCast(v[3]) };
}

pub fn toAbi3(v: Vec3) abi.Vec3 {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]) };
}

pub fn toAbi4(v: Quat) abi.Quat {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]), @floatCast(v[3]) };
}

pub fn add(a: Vec3, b: Vec3) Vec3 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

pub fn sub(a: Vec3, b: Vec3) Vec3 {
    return .{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

pub fn scale(a: Vec3, s: f64) Vec3 {
    return .{ a[0] * s, a[1] * s, a[2] * s };
}

pub fn dot(a: Vec3, b: Vec3) f64 {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

pub fn norm(a: Vec3) f64 {
    return @sqrt(dot(a, a));
}

pub fn cross(a: Vec3, b: Vec3) Vec3 {
    return .{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

pub fn conjugate(a: Quat) Quat {
    return .{ -a[0], -a[1], -a[2], a[3] };
}

pub fn multiply(a: Quat, b: Quat) Quat {
    return .{
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    };
}

pub fn rotate(a: Quat, b: Vec3) Vec3 {
    const u: Vec3 = .{ a[0], a[1], a[2] };
    const twice_cross = scale(cross(u, b), 2);
    return add(b, add(scale(twice_cross, a[3]), cross(u, twice_cross)));
}

pub fn point(a: Pose, b: Vec3) Vec3 {
    return add(a.pos, rotate(a.quat, b));
}

pub fn compose(a: Pose, b: Pose) Pose {
    return .{ .pos = point(a, b.pos), .quat = multiply(a.quat, b.quat) };
}

pub fn finite3(a: Vec3) bool {
    return std.math.isFinite(a[0]) and std.math.isFinite(a[1]) and std.math.isFinite(a[2]);
}

pub fn finite4(a: Quat) bool {
    return std.math.isFinite(a[0]) and std.math.isFinite(a[1]) and std.math.isFinite(a[2]) and std.math.isFinite(a[3]);
}

pub fn validPose(a: Pose) error{InvalidPose}!void {
    var n: f64 = 0;
    for (a.quat) |x| n += x * x;
    if (!finite3(a.pos) or !finite4(a.quat) or @abs(n - 1) >= 1e-4) return error.InvalidPose;
}

pub fn bodyPose(s: abi.BodyState) Pose {
    return .{ .pos = fromAbi3(s.pose_pos), .quat = fromAbi4(s.pose_quat) };
}

pub fn addForce(w: *abi.Wrench, f: Vec3, p: Vec3, com: Vec3) void {
    w.force = toAbi3(add(fromAbi3(w.force), f));
    w.torque = toAbi3(add(fromAbi3(w.torque), cross(sub(p, com), f)));
}