const std = @import("std");

pub const Vector2f = @Vector(2, f32);
pub const Vector3f = @Vector(3, f32);
pub const Vector6f = @Vector(6, f32);
pub const Quaternionf = @Vector(4, f32);

pub const Pose = extern struct {
    pos: Vector3f,
    quat: Quaternionf,
};

pub const BoundingBox = extern struct {
    pose: Pose,
    size: Vector3f,
};

pub inline fn xy(v: Vector3f) Vector2f {
    return .{v[0], v[1]};
}

pub inline fn dot2f(a: Vector2f, b: Vector2f) f32 {
    return @reduce(.Add, a * b);
}

pub inline fn length2f(v: Vector2f) f32 {
    return @sqrt(dot2f(v, v));
}

pub inline fn normalize2f(v: Vector2f) Vector2f {
    const len = length2f(v);
    return v / @as(Vector2f, @splat(len));
}

pub inline fn dot3f(a: Vector3f, b: Vector3f) f32 {
    return @reduce(.Add, a * b);
}

pub inline fn length3f(v: Vector3f) f32 {
    return @sqrt(dot3f(v, v));
}

pub inline fn normalize3f(v: Vector3f) Vector3f {
    const len = length3f(v);
    return v / @as(Vector3f, @splat(len));
}

pub inline fn cross3f(a: Vector3f, b: Vector3f) Vector3f {
    const a_yzx = @shuffle(f32, a, undefined, Vector3f{ 1, 2, 0 });
    const a_zxy = @shuffle(f32, a, undefined, Vector3f{ 2, 0, 1 });
    const b_yzx = @shuffle(f32, b, undefined, Vector3f{ 1, 2, 0 });
    const b_zxy = @shuffle(f32, b, undefined, Vector3f{ 2, 0, 1 });

    return a_yzx * b_zxy - a_zxy * b_yzx;
}

pub inline fn normalize4f(v: Quaternionf) Quaternionf {
    const len = @sqrt(@reduce(.Add, v * v));
    return v / @as(Quaternionf, @splat(len));
}

pub inline fn quaternionConjugate(q: Quaternionf) Quaternionf {
    return Quaternionf{ -1, -1, -1, 1 } * q;
}

pub inline fn quaternionMul(a: Quaternionf, b: Quaternionf) Quaternionf {
    const av: Vector3f = a[0..3].*;
    const bv: Vector3f = b[0..3].*;

    const xyz =
        a[3] * bv +
        b[3] * av +
        cross3f(av, bv);

    const w = a[3] * b[3] - dot3f(av, bv);

    return .{ xyz[0], xyz[1], xyz[2], w };
}

pub inline fn quaternionRotate(q: Quaternionf, v: Vector3f) Vector3f {
    const qv: Vector3f = .{ q[0], q[1], q[2] };
    const t = @as(Vector3f, @splat(2.0)) * cross3f(qv, v);

    return v + @as(Vector3f, @splat(q[3])) * t + cross3f(qv, t);
}

pub inline fn quaternionToEuler(q: Quaternionf) Vector3f {
    const x, const y, const z, const w = q;

    const sinr_cosp: f32 = 2.0 * (w * x + y * z);
    const cosr_cosp: f32 = 1.0 - 2.0 * (x * x + y * y);
    const roll: f32 = std.math.atan2(sinr_cosp, cosr_cosp);

    const sinp: f32 = 2.0 * (w * y - z * x);
    const pitch: f32 = if (@abs(sinp) >= 1.0)
        if (sinp < 0.0) -std.math.pi / 2.0 else std.math.pi / 2.0
    else
        std.math.asin(sinp);

    const siny_cosp: f32 = 2.0 * (w * z + x * y);
    const cosy_cosp: f32 = 1.0 - 2.0 * (y * y + z * z);
    const yaw: f32 = std.math.atan2(siny_cosp, cosy_cosp);

    return .{ roll, pitch, yaw };
}

pub inline fn wrapAngle(angle: f32) f32 {
    return @mod(angle + std.math.pi, 2.0 * std.math.pi) - std.math.pi;
}

pub fn withinDeadband(err: f32, deadband: f32) f32 {
    if (@abs(err) <= deadband) return 0;
    return err;
}

pub const BoxMetrics2d = struct {
    cx: f32,
    cy: f32,
    height: f32,
};

/// Normalized image errors: cx/cy in roughly [-1, 1] from image center; height in [0, 1].
/// `top_left` and `bottom_right` are pixel corners `{x, y}`.
pub fn boxMetrics(
    image_width: u32,
    image_height: u32,
    top_left: [2]u32,
    bottom_right: [2]u32,
) BoxMetrics2d {
    const w = @as(f32, @floatFromInt(image_width));
    const h = @as(f32, @floatFromInt(image_height));
    const center_x = (@as(f32, @floatFromInt(top_left[0])) + @as(f32, @floatFromInt(bottom_right[0]))) * 0.5;
    const center_y = (@as(f32, @floatFromInt(top_left[1])) + @as(f32, @floatFromInt(bottom_right[1]))) * 0.5;
    const cx = (center_x / w - 0.5) * 2.0;
    const cy = (center_y / h - 0.5) * 2.0;
    const height = @as(f32, @floatFromInt(bottom_right[1] - top_left[1])) / h;
    return .{ .cx = cx, .cy = cy, .height = height };
}

pub fn rotationBetween(forward: Vector3f, dir: Vector3f) Quaternionf {
    const dot = dot3f(forward, dir);

    if (dot < -1.0 + 1e-6) {
        // Opposite vectors: choose an arbitrary perpendicular axis.
        const axis = if (@abs(forward[0]) < @abs(forward[1]))
            normalize3f(cross3f(forward, .{ 1, 0, 0 }))
        else
            normalize3f(cross3f(forward, .{ 0, 1, 0 }));

        // 180 degree rotation: sin(pi/2) = 1, cos(pi/2) = 0.
        return .{ axis[0], axis[1], axis[2], 0.0 };
    }

    const q: Quaternionf = .{
        cross3f(forward, dir)[0],
        cross3f(forward, dir)[1],
        cross3f(forward, dir)[2],
        1.0 + dot,
    };

    return normalize4f(q);
}

// pub inline fn poseTo6f(pose: Pose) Auv.Vector6f {
//     const x, const y, const z = pose.pos;
//     const roll, const pitch, const yaw = quaternionToEuler(pose.quat);
//
//     return .{ x, y, z, roll, pitch, yaw };
// }

pub inline fn poseTo6f(pose: Pose) Vector6f {
    const x, const y, const z = pose.pos;
    const rot = normalize4f(pose.quat);
    const roll, const pitch, const yaw = quaternionToEuler(rot);

    return .{ x, y, z, roll, pitch, yaw };
}

pub fn tamMul(tam: []const Vector6f, v: Vector6f, out: []f32) []f32 {
    const result = out[0..tam.len];

    for (tam, result) |row, *r| {
        r.* = @reduce(.Add, row * v);
    }

    return result;
}


pub const max_thrusters = 8;
/// Linear map from 6-DOF command to per-thruster values: t_i = dot(tam[i], input).
/// Clamps to ±`thrustor_saturate`, then divides by 5 (backend scale in MissionContext).
/// `out` must be at least `tam.len` elements; returns `out[0..tam.len]`.
pub fn tamToThrusters(
    tam: []const Vector6f,
    input: Vector6f,
    thrustor_saturate: f32,
    output_scale: f32,
    out: []f32,
) []f32 {
    std.debug.assert(out.len >= tam.len);
    std.debug.assert(output_scale > 0);
    const thruster_values = tamMul(tam, input, out);
    for (thruster_values) |*value| {
        value.* = std.math.clamp(value.*, -thrustor_saturate, thrustor_saturate);
        value.* /= output_scale;
    }
    return thruster_values;
}


/// Per-axis PID on a 6-DOF error vector, producing a wrench command.
///
/// For each axis: `wrench = kp·err + ki·∫err + kd·d(err)/dt`.
/// Gains and error are element-wise (`Vector6f`), e.g. x/y/z position and roll/pitch/yaw.
///
/// When `dt` is non-null, the I term uses `sumError + err·dt` and D uses `(err - prevError)/dt`.
/// When `dt` is null (missing or invalid timestep), D is zero and I uses `sumError` unchanged—
/// same I/D skip as `MissionContext.ControllerStep`.
///
/// `sumError` and `prevError` are updated in place when `dt` is non-null.
/// Returns the wrench; feed through `wrenchToThrusterInput` / `tamToThrusters` for thrusters.
pub fn pidWrench(
    sumError: *Vector6f,
    prevError: *Vector6f,
    kp: Vector6f,
    ki: Vector6f,
    kd: Vector6f,
    err: Vector6f,
    dt: ?f32,
) Vector6f {
    const vel_err: Vector6f = if (dt) |dt_val| blk: {
        sumError.* += err * @as(Vector6f, @splat(dt_val));
        break :blk (err - prevError.*) / @as(Vector6f, @splat(dt_val));
    } else @as(Vector6f, @splat(0));
    const wrench = kp * err + ki * sumError.* + kd * vel_err;
    prevError.* = err;
    return wrench;
}




/// Maps a 6-DOF PID wrench into the 6-DOF command vector expected by `math.tamMul`.
///
/// Pose `orientation` (unit quaternion `rot`) rotates body vectors into world:
///   f_world = R(rot) · f_body     (3×1 = 3×3 · 3×1)
///
/// PID linear output `wrench[0..3]` is treated as force in **world** axes (from
/// world-frame position error). Thruster mixing uses **body** X/Y after policy:
///   f_body = R(rot)⁻¹ · f_world = R(conjugate(rot)) · f_world
///
/// Angular part of `wrench` is not re-rotated here; roll gets a sign fix for TAM.
/// `yaw_error` is err[5] (heading error); if |yaw_error| > gate, body X/Y force is
/// zeroed so the vehicle turns before surging.
///
/// Returns TAM input: [Fx_body, Fy_body, Fz, Mx, My, Mz].
pub fn wrenchToThrusterInput(
    wrench: Vector6f,
    orientation: Quaternionf,
    yaw_error: f32,
    close_enough_yaw_gate: f32,
) Vector6f {
    const rot = normalize4f(orientation);
    const world_force: Vector3f = .{ wrench[0], wrench[1], wrench[2] };
    const rotated = quaternionRotate(quaternionConjugate(rot), world_force);
    var body_force = rotated;
    if (@abs(yaw_error) > close_enough_yaw_gate) {
        body_force[0] = 0.0;
        body_force[1] = 0.0;
    } else {
        body_force[0] = @max(body_force[0], 0.0); // This clamping force the auv to always move forward
        body_force[1] = @max(body_force[1], 0.0);
    }
    return .{
        body_force[0],
        body_force[1],
        wrench[2],
        -wrench[3],
        wrench[4],
        wrench[5],
    };
}





pub fn debug3f(comptime s: []const u8, v: Vector3f) void {
    std.log.debug(s ++ " {{ {d:5.2} {d:5.2} {d:5.2} }}", .{
        v[0], v[1], v[2],
    });
}

pub fn debug6f(comptime s: []const u8, v: Vector6f) void {
    std.log.debug(s ++ " {{ {d:5.2} {d:5.2} {d:5.2} {d:5.2} {d:5.2} {d:5.2} }}", .{
        v[0], v[1], v[2], v[3], v[4], v[5],
    });
}
