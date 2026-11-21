
const std = @import("std");
const types = @import("types.zig");
const abi = @import("abi.zig");
const math = @import("../math.zig");

pub const Vec3 = types.Vec3;
pub const Quat = types.Quat;
pub const Pose = types.Pose;

const V3 = math.Vector3f;
const Q4 = math.Quaternionf;

inline fn v3(a: Vec3) V3 {
    return .{ a[0], a[1], a[2] };
}

inline fn arr3(v: V3) Vec3 {
    return .{ v[0], v[1], v[2] };
}

inline fn q4(a: Quat) Q4 {
    return .{ a[0], a[1], a[2], a[3] };
}

inline fn arr4(v: Q4) Quat {
    return .{ v[0], v[1], v[2], v[3] };
}

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
    return arr3(v3(a) + v3(b));
}

pub fn sub(a: Vec3, b: Vec3) Vec3 {
    return arr3(v3(a) - v3(b));
}

pub fn scale(a: Vec3, s: f64) Vec3 {
    return arr3(v3(a) * @as(V3, @splat(s)));
}

pub fn dot(a: Vec3, b: Vec3) f64 {
    return math.dot3f(v3(a), v3(b));
}

pub fn norm(a: Vec3) f64 {
    return math.length3f(v3(a));
}

pub fn cross(a: Vec3, b: Vec3) Vec3 {
    return arr3(math.cross3f(v3(a), v3(b)));
}

pub fn conjugate(a: Quat) Quat {
    return arr4(math.quaternionConjugate(q4(a)));
}

pub fn multiply(a: Quat, b: Quat) Quat {
    return arr4(math.quaternionMul(q4(a), q4(b)));
}

pub fn rotate(a: Quat, b: Vec3) Vec3 {
    return arr3(math.quaternionRotate(q4(a), v3(b)));
}

pub fn point(a: Pose, b: Vec3) Vec3 {
    return arr3(v3(a.pos) + math.quaternionRotate(q4(a.quat), v3(b)));
}

pub fn compose(a: Pose, b: Pose) Pose {
    const aq = q4(a.quat);
    const bq = q4(b.quat);
    return .{
        .pos = arr3(v3(a.pos) + math.quaternionRotate(aq, v3(b.pos))),
        .quat = arr4(math.quaternionMul(aq, bq)),
    };
}

pub fn finite3(a: Vec3) bool {
    const v = v3(a);
    return std.math.isFinite(v[0]) and std.math.isFinite(v[1]) and std.math.isFinite(v[2]);
}

pub fn finite4(a: Quat) bool {
    const v = q4(a);
    return std.math.isFinite(v[0]) and std.math.isFinite(v[1]) and std.math.isFinite(v[2]) and std.math.isFinite(v[3]);
}

pub fn validPose(a: Pose) error{InvalidPose}!void {
    const n = @reduce(.Add, q4(a.quat) * q4(a.quat));
    if (!finite3(a.pos) or !finite4(a.quat) or @abs(n - 1) >= 1e-4) return error.InvalidPose;
}

pub fn bodyPose(s: abi.BodyState) Pose {
    return .{ .pos = fromAbi3(s.pose_pos), .quat = fromAbi4(s.pose_quat) };
}

pub fn addForce(w: *abi.Wrench, f: Vec3, p: Vec3, com: Vec3) void {
    const arm = v3(p) - v3(com);
    w.force = toAbi3(arr3(v3(fromAbi3(w.force)) + v3(f)));
    w.torque = toAbi3(arr3(v3(fromAbi3(w.torque)) + math.cross3f(arm, v3(f))));
}