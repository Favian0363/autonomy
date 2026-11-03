//! Body-frame control geometry: constrained looks, thrust subspace, and PID errors.
const std = @import("std");
const math = @import("math.zig");

pub const Vec3 = math.Vector3f;
pub const Quat = math.Quaternionf; // x,y,z,w
pub const identity: Quat = .{ 0, 0, 0, 1 };
pub const Error = error{ InvalidVector, InvalidQuaternion, InvalidAngle, InvalidTolerance };

const vec_eps: f32 = 1e-6;

pub fn normalizedQuat(q: Quat) Error!Quat {
    const n2 = @reduce(.Add, q * q);
    if (!std.math.isFinite(n2) or n2 <= vec_eps * vec_eps) return error.InvalidQuaternion;
    return math.normalize4f(q);
}

/// Rotation alpha around arbitrary camera-forward BODY axis.
/// This is only an axis-angle quaternion; it does not align a look direction.
pub fn forwardRotation(forward_body: Vec3, alpha: f32) Error!Quat {
    const forward_len = math.length3f(forward_body);
    if (!std.math.isFinite(forward_len) or forward_len <= vec_eps) return error.InvalidVector;
    const f = math.normalize3f(forward_body);
    if (!std.math.isFinite(alpha)) return error.InvalidAngle;
    const v = f * @as(Vec3, @splat(std.math.sin(alpha / 2)));
    return .{ v[0], v[1], v[2], std.math.cos(alpha / 2) };
}

fn shortestArc(from: Vec3, to: Vec3) Error!Quat {
    const c = std.math.clamp(math.dot3f(from, to), -1, 1);
    const v = math.cross3f(from, to);
    // atan2 remains accurate near parallel and antiparallel directions.
    const s = math.length3f(v);
    if (s > vec_eps) return forwardRotation(v * @as(Vec3, @splat(1 / s)), std.math.atan2(s, c));
    if (c >= 0) return identity;
    // Exactly opposite: deterministic perpendicular axis; no unique shortest axis exists.
    const ax = @abs(from[0]);
    const ay = @abs(from[1]);
    const az = @abs(from[2]);
    const seed: Vec3 = if (ay < ax and ay <= az)
        .{ 0, 1, 0 }
    else if (az < ax and az < ay)
        .{ 0, 0, 1 }
    else
        .{ 1, 0, 0 };
    const axis = math.cross3f(from, seed);
    const axis_len = math.length3f(axis);
    if (!std.math.isFinite(axis_len) or axis_len <= vec_eps) return error.InvalidVector;
    return forwardRotation(math.normalize3f(axis), std.math.pi);
}

/// Align camera forward with look_world, then rotate alpha around BODY forward.
/// reference_q defines alpha=0: shortest alignment from reference's forward to look.
/// q_target = q_align_world * reference_q * q_extra_body.
/// A fixed reference gives a fixed zero-twist convention. Passing current_q instead
/// makes alpha an increment relative to the newly aligned current orientation.
/// No movement constraints are selected or solved here.
pub fn targetQuaternion(forward_body: Vec3, look_world: Vec3, alpha: f32, reference_q: Quat) Error!Quat {
    const forward_len = math.length3f(forward_body);
    if (!std.math.isFinite(forward_len) or forward_len <= vec_eps) return error.InvalidVector;
    const f = math.normalize3f(forward_body);
    const look_len = math.length3f(look_world);
    if (!std.math.isFinite(look_len) or look_len <= vec_eps) return error.InvalidVector;
    const l = math.normalize3f(look_world);
    const reference = try normalizedQuat(reference_q);
    const alignment = try shortestArc(math.quaternionRotate(reference, f), l);
    return normalizedQuat(math.quaternionMul(math.quaternionMul(alignment, reference), try forwardRotation(f, alpha)));
}

/// Rotate `reference_q` only about `axis_body` so `forward_body` points as close as
/// that one axis allows to `look_world`. The axis is in the reference body frame.
/// `q_target = reference_q * q_axis`. Yaw from a level hull is axis `{0, 0, 1}` and
/// `reference_q` identity. Pitch is `{1, 0, 0}`. Roll about the camera is `forward_body`.
/// Returns `reference_q` when the look direction is parallel to the axis.
/// Returns `InvalidVector` when the forward axis is parallel to the rotation axis.
pub fn targetAboutAxis(axis_body: Vec3, forward_body: Vec3, look_world: Vec3, reference_q: Quat) Error!Quat {
    const reference = try normalizedQuat(reference_q);
    const angle = try axisAimAngle(axis_body, forward_body, look_world, reference) orelse return reference;
    const axis_len = math.length3f(axis_body);
    if (!std.math.isFinite(axis_len) or axis_len <= vec_eps) return error.InvalidVector;
    return normalizedQuat(math.quaternionMul(reference, try forwardRotation(math.normalize3f(axis_body), angle)));
}

/// Point `forward_body` at `look_world` using only body X, then Z, then Y, each clamped
/// to `±max_abs`. Heading (Z) is solved first, then nod (X), then twist (Y).
/// A zero limit leaves that axis at `reference_q`.
pub fn lookWithinLimits(forward_body: Vec3, look_world: Vec3, reference_q: Quat, max_abs: Vec3) Error!Quat {
    const axes = [3]Vec3{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } };
    const limits = [3]f32{ max_abs[0], max_abs[1], max_abs[2] };
    const order = [3]usize{ 2, 0, 1 };
    var q = try normalizedQuat(reference_q);
    for (order) |i| {
        const limit = @abs(limits[i]);
        if (limit == 0) continue;
        const angle = axisAimAngle(axes[i], forward_body, look_world, q) catch |err| switch (err) {
            error.InvalidVector => continue,
            else => |e| return e,
        } orelse continue;
        q = try normalizedQuat(math.quaternionMul(q, try forwardRotation(axes[i], std.math.clamp(angle, -limit, limit))));
    }
    return q;
}

fn axisAimAngle(axis_body: Vec3, forward_body: Vec3, look_world: Vec3, reference_q: Quat) Error!?f32 {
    const axis_len = math.length3f(axis_body);
    if (!std.math.isFinite(axis_len) or axis_len <= vec_eps) return error.InvalidVector;
    const axis = math.normalize3f(axis_body);
    const forward_len = math.length3f(forward_body);
    if (!std.math.isFinite(forward_len) or forward_len <= vec_eps) return error.InvalidVector;
    const forward = math.normalize3f(forward_body);
    const look_len = math.length3f(look_world);
    if (!std.math.isFinite(look_len) or look_len <= vec_eps) return error.InvalidVector;
    const look = math.normalize3f(look_world);
    const reference = try normalizedQuat(reference_q);
    const f = forward - axis * @as(Vec3, @splat(math.dot3f(axis, forward)));
    if (math.length3f(f) <= vec_eps) return error.InvalidVector;
    const look_body = math.quaternionRotate(math.quaternionConjugate(reference), look);
    const l = look_body - axis * @as(Vec3, @splat(math.dot3f(axis, look_body)));
    if (math.length3f(l) <= vec_eps) return null;
    return std.math.atan2(math.dot3f(axis, math.cross3f(f, l)), math.dot3f(f, l));
}

/// Body directions this vehicle can thrust along. Position commands off these
/// directions are dropped, so an omitted axis (Proteus body X) never moves.
/// Stored as an orthonormal basis of the directions you pass in.
pub const ThrustDirections = struct {
    basis: [3]Vec3 = .{ .{ 0, 0, 0 }, .{ 0, 0, 0 }, .{ 0, 0, 0 } },
    rank: usize = 0,

    /// `directions` are body-frame thrust axes, such as sway `.{0,1,0}` and heave `.{0,0,1}`.
    /// Zero directions are ignored. A direction already covered by an earlier one is ignored
    /// when the leftover fraction of its length is below `distinct_direction_tolerance`.
    /// Use 1e-5 for ordinary f32 axis directions.
    pub fn fromBodyDirections(directions: []const Vec3, distinct_direction_tolerance: f32) Error!ThrustDirections {
        if (!std.math.isFinite(distinct_direction_tolerance) or distinct_direction_tolerance <= 0 or distinct_direction_tolerance >= 1)
            return error.InvalidTolerance;
        var thrust: ThrustDirections = .{};
        for (directions) |direction| {
            const n = math.length3f(direction);
            if (!std.math.isFinite(n)) return error.InvalidVector;
            if (n <= vec_eps) continue;
            var v = direction * @as(Vec3, @splat(1 / n));
            for (0..2) |_| {
                for (thrust.basis[0..thrust.rank]) |b| v -= b * @as(Vec3, @splat(math.dot3f(b, v)));
            }
            const r = math.length3f(v);
            if (r > distinct_direction_tolerance and thrust.rank < 3) {
                thrust.basis[thrust.rank] = v * @as(Vec3, @splat(1 / r));
                thrust.rank += 1;
            }
        }
        return thrust;
    }
    /// Part of a body-frame vector the thrusters can produce. The rest becomes zero.
    pub fn alongThrust(self: ThrustDirections, body: Vec3) Vec3 {
        var result: Vec3 = .{ 0, 0, 0 };
        for (self.basis[0..self.rank]) |b| result += b * @as(Vec3, @splat(math.dot3f(b, body)));
        return result;
    }

    /// Body axes the TAM actually mixes into thruster force. Each row is one thruster,
    /// columns are Fx, Fy, Fz, Mx, My, Mz. A force column whose energy is below 1e-3 of
    /// the strongest force column is unused, so a zero surge column never commands body X.
    pub fn fromTamLinearForces(rows: []const math.Vector6f) Error!ThrustDirections {
        return fromColumns(rows, 0);
    }

    /// Body axes the TAM can torque about. Columns Mx, My, Mz, same energy test as `fromTamLinearForces`.
    pub fn fromTamMoments(rows: []const math.Vector6f) Error!ThrustDirections {
        return fromColumns(rows, 3);
    }

    fn fromColumns(rows: []const math.Vector6f, comptime first_column: usize) Error!ThrustDirections {
        const relative_tolerance: f32 = 1e-3;
        var energy: [3]f32 = .{ 0, 0, 0 };
        for (rows) |row| {
            inline for (0..3) |axis| {
                const gain = row[first_column + axis];
                if (!std.math.isFinite(gain)) return error.InvalidVector;
                energy[axis] += gain * gain;
            }
        }
        var strongest: f32 = 0;
        for (energy) |e| strongest = @max(strongest, e);
        const axis_directions = [3]Vec3{
            .{ 1, 0, 0 },
            .{ 0, 1, 0 },
            .{ 0, 0, 1 },
        };
        var directions: [3]Vec3 = undefined;
        var count: usize = 0;
        if (strongest > 0) {
            const cutoff = relative_tolerance * relative_tolerance * strongest;
            for (energy, 0..) |e, axis| {
                if (e > cutoff) {
                    directions[count] = axis_directions[axis];
                    count += 1;
                }
            }
        }
        return fromBodyDirections(directions[0..count], 1e-5);
    }
};

pub const Tilt = struct { axis: Vec3, angle: f32 };

pub const ForceCoverage = struct {
    force: Vec3,
    tilt: ?Tilt = null,
};

/// Swing one available thruster onto the part of `force` the map cannot produce.
/// `sideways = F_thruster * tan(angle)`, and `angle` stays inside `rotation_limit`.
/// A request past that is cut. A thruster force near zero cannot buy sideways force,
/// so that part is dropped and no tilt is commanded.
/// Among thrusters that can hold the tilt, the one that keeps the most sideways force wins.
pub fn coverForce(
    force_dirs: ThrustDirections,
    moment_dirs: ThrustDirections,
    force: Vec3,
    rotation_limit: Vec3,
    camera_forward: Vec3,
) ForceCoverage {
    const available = force_dirs.alongThrust(force);
    const missing = force - available;
    const missing_len = math.length3f(missing);
    if (missing_len <= vec_eps) return .{ .force = force };

    const Candidate = struct {
        axis: Vec3,
        angle: f32,
        force: Vec3,
        delivered: f32,
        alignment: f32,
    };
    var best: ?Candidate = null;
    const half_pi = std.math.pi / 2.0;
    const missing_hat = missing / @as(Vec3, @splat(missing_len));
    for (force_dirs.basis[0..force_dirs.rank]) |thrust| {
        const axis_raw = math.cross3f(thrust, missing);
        const axis_len = math.length3f(axis_raw);
        if (axis_len <= vec_eps) continue;
        const axis = axis_raw / @as(Vec3, @splat(axis_len));
        const held = moment_dirs.alongThrust(axis);
        if (math.length3f(axis - held) > 0.25) continue;
        var room: f32 = std.math.inf(f32);
        inline for (0..3) |i| {
            const component = @abs(axis[i]);
            if (component > 1e-4) room = @min(room, @abs(rotation_limit[i]) / component);
        }
        const theta_max = @min(room, half_pi - 0.05);
        if (!(theta_max > 1e-4)) continue;
        const along = math.dot3f(force, thrust);
        if (@abs(along) <= 1e-4) continue;
        const capacity = @abs(along) * @tan(theta_max);
        const clamped = std.math.clamp(missing_len, 0, capacity);
        if (clamped <= 1e-4) continue;
        var angle = std.math.atan2(clamped, along);
        if (angle > half_pi) angle -= std.math.pi;
        if (angle < -half_pi) angle += std.math.pi;
        angle = std.math.clamp(angle, -theta_max, theta_max);
        if (@abs(angle) <= 1e-4) continue;
        const reshaped = available + missing_hat * @as(Vec3, @splat(clamped));
        const alignment = @abs(math.dot3f(axis, camera_forward));
        const take = if (best) |chosen|
            clamped > chosen.delivered + 1e-3 or (@abs(clamped - chosen.delivered) <= 1e-3 and alignment > chosen.alignment)
        else
            true;
        if (take) best = .{
            .axis = axis,
            .angle = angle,
            .force = reshaped,
            .delivered = clamped,
            .alignment = alignment,
        };
    }
    const chosen = best orelse return .{ .force = available };
    return .{ .force = chosen.force, .tilt = .{ .axis = chosen.axis, .angle = chosen.angle } };
}

/// World-frame position error: `target_world - position_world`.
pub fn positionError(position_world: Vec3, target_world: Vec3) Error!Vec3 {
    const dw = target_world - position_world;
    if (!std.math.isFinite(math.length3f(dw))) return error.InvalidVector;
    return dw;
}

/// ATTITUDE path only: shortest rotation-vector error in CURRENT BODY coordinates.
/// `target_q` must already be unit length.
pub fn attitudeError(current_q: Quat, target_q: Quat) Error!Vec3 {
    var e = try normalizedQuat(math.quaternionMul(math.quaternionConjugate(try normalizedQuat(current_q)), target_q));
    if (e[3] < 0) e = -e;
    const v: Vec3 = .{ e[0], e[1], e[2] };
    const s = math.length3f(v);
    if (s < vec_eps) return v * @as(Vec3, @splat(2));
    return v * @as(Vec3, @splat(2 * std.math.atan2(s, e[3]) / s));
}

/// Your TAM convention: one row per motor, six columns per row. No output limiting.
pub fn mix(comptime motor_count: usize, rows: [motor_count]math.Vector6f, c: math.Vector6f) [motor_count]f32 {
    var out: [motor_count]f32 = undefined;
    _ = math.tamMul(&rows, c, &out);
    return out;
}