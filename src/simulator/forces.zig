const std = @import("std");
const types = @import("types.zig");
const Scene = @import("Scene.zig");
const S = @import("support.zig");
pub const Context = struct {
    scene: *const Scene,
    bodies: []const types.BodyState,
    commands: []const f64,
    tick: u64,
    dt: f64,
    rotors: []types.RotorState,
    fluid_cache: []types.Wrench,
    scratch: std.mem.Allocator,
};

pub fn evaluate(force: types.Force, context: *const Context, wrenches: []types.Wrench) !void {
    switch (force) {
        .gravity => gravity(context, wrenches),
        .hydrodynamics => try hydro(context, wrenches),
        .thrusters => thrusters(context, wrenches),
    }
}

fn coefficients(force: types.Vec3, basis: types.Quat, cd: types.Vec3, scale: f64) types.Vec3 {
    @setFloatMode(.strict);
    const d = S.rotate(S.conjugate(basis), force);
    const len = S.norm(d);
    const factor = if (len < 1.4901161193847656e-8) 0 else scale * (@abs(d[0]) * cd[0] + @abs(d[1]) * cd[1] + @abs(d[2]) * cd[2]) / len;
    return S.scale(force, factor);
}

fn gravity(context: *const Context, w: []types.Wrench) void {
    @setFloatMode(.strict);
    for (w, 0..) |*wrench, i| {
        const body = context.scene.objects[i].body;
        if (body.motion_type == .dynamic)
            wrench.force = S.add(wrench.force, S.scale(context.scene.settings.gravity, body.dry_mass - body.mass));
    }
}

fn hydro(context: *const Context, w: []types.Wrench) !void {
    @setFloatMode(.strict);
    const model = context.scene;
    if (context.tick % model.settings.hydro_period == 0) {
        @memset(context.fluid_cache, .{});
        const minz = try context.scratch.alloc(f64, w.len);
        const maxz = try context.scratch.alloc(f64, w.len);
        @memset(minz, std.math.inf(f64));
        @memset(maxz, -std.math.inf(f64));
        const Wet = struct { lo: f64 = std.math.inf(f64), hi: f64 = -std.math.inf(f64) };
        var wet: [types.max_hulls]Wet = @splat(.{});
        for (model.hulls, wet[0..model.hulls.len]) |h, *extent| {
            const index = @backingInt(h.body);
            const transform = S.compose(context.bodies[index].pose, h.pose);
            for (h.positions) |position| {
                const z = S.point(transform, position)[2];
                extent.lo = @min(extent.lo, z);
                extent.hi = @max(extent.hi, z);
            }
            minz[index] = @min(minz[index], extent.lo);
            maxz[index] = @max(maxz[index], extent.hi);
        }
        for (w, 0..) |_, index| if (minz[index] > 0 and std.math.isFinite(minz[index])) {
            const body = model.objects[index].body;
            S.addForce(&context.fluid_cache[index], S.scale(model.settings.gravity, -model.settings.water_density * body.volume), S.point(context.bodies[index].pose, body.buoyancy_center), context.bodies[index].center_of_mass);
        };
        for (model.hulls, wet[0..model.hulls.len]) |h, extent| {
            const index = @backingInt(h.body);
            const body = context.bodies[index];
            if (maxz[index] <= 0) continue;
            const transform = S.compose(body.pose, h.pose);
            const out = &context.fluid_cache[index];
            if (minz[index] <= 0 and extent.hi > 0) {
                const fraction = if (extent.lo > 0) 1 else extent.hi / (extent.hi - extent.lo);
                S.addForce(out, S.scale(model.settings.gravity, -fraction * h.volume * model.settings.water_density), S.point(transform, h.centroid), body.center_of_mass);
            }
            var pressure_linear = S.v(0, 0, 0);
            var pressure_angular = S.v(0, 0, 0);
            var skin_linear = S.v(0, 0, 0);
            var skin_angular = S.v(0, 0, 0);
            var vertex: usize = 0;
            while (vertex < h.positions.len) : (vertex += 3) {
                const a = S.point(transform, h.positions[vertex]);
                const b = S.point(transform, h.positions[vertex + 1]);
                const d = S.point(transform, h.positions[vertex + 2]);
                var normal = S.cross(S.sub(b, a), S.sub(d, a));
                const twice = S.norm(normal);
                if (twice < 1e-6) continue;
                normal = S.scale(normal, 1 / twice);
                const center = S.scale(S.add(S.add(a, b), d), 1.0 / 3.0);
                if (minz[index] <= 0 and center[2] <= 0) continue;
                const arm = S.sub(center, body.center_of_mass);
                const relative = S.sub(model.settings.current, S.add(body.linear_velocity, S.cross(body.angular_velocity, arm)));
                const vn = S.dot(relative, normal);
                if (vn < -1e-12) {
                    const force = S.scale(relative, S.norm(relative) * -vn * 0.5 * twice);
                    pressure_linear = S.add(pressure_linear, force);
                    pressure_angular = S.add(pressure_angular, S.cross(arm, force));
                }
                const tangent = S.sub(relative, S.scale(normal, vn));
                if (S.dot(tangent, tangent) > 1e-9) {
                    const force = S.scale(tangent, 0.5 * twice);
                    skin_linear = S.add(skin_linear, force);
                    skin_angular = S.add(skin_angular, S.cross(arm, force));
                }
            }
            const rho = model.settings.water_density;
            const frame = S.multiply(body.pose.quat, h.coefficient_frame);
            out.force = S.add(S.add(out.force, coefficients(pressure_linear, frame, h.drag, 0.5 * rho)), coefficients(skin_linear, frame, h.skin, rho));
            out.torque = S.add(S.add(out.torque, coefficients(pressure_angular, frame, h.drag, 0.5 * rho)), coefficients(skin_angular, frame, h.skin, rho));
        }
    }
    for (w, 0..) |*wrench, index| {
        wrench.force = S.add(wrench.force, context.fluid_cache[index].force);
        wrench.torque = S.add(wrench.torque, context.fluid_cache[index].torque);
    }
}

fn thrusters(context: *const Context, w: []types.Wrench) void {
    @setFloatMode(.strict);
    const model = context.scene;
    for (model.thrusters, context.rotors, context.commands) |thruster, *rotor, command| {
        const index = @backingInt(thruster.body);
        const body = context.bodies[index];
        const spec = thruster.spec;
        const max = spec.max_rpm / 60 * 2 * std.math.pi;
        const target = spec.command_sign * std.math.clamp(command, -1, 1) * max;
        const error_omega = target - rotor.omega;
        const damping = if (rotor.omega > 0) @abs(rotor.torque) else -@abs(rotor.torque);
        rotor.omega = std.math.clamp((spec.inertia * rotor.omega + context.dt * (spec.kp * target + spec.ki * rotor.integral - damping)) / (spec.inertia + context.dt * spec.kp), -2 * max, 2 * max);
        rotor.integral = std.math.clamp(rotor.integral + error_omega * context.dt, -5, 5);
        const p = S.point(body.pose, thruster.position);
        const axis = S.rotate(body.pose.quat, thruster.direction);
        rotor.thrust = 0;
        if (p[2] <= 0) {
            rotor.torque = 0;
            continue;
        }
        const inflow = S.dot(axis, S.sub(S.add(body.linear_velocity, S.cross(body.angular_velocity, S.sub(p, body.center_of_mass))), model.settings.current));
        const n = rotor.omega / (2 * std.math.pi);
        const d = spec.diameter;
        const rho = model.settings.water_density;
        rotor.thrust = rho * d * d * d * @abs(n) * (d * spec.thrust * n - spec.thrust * inflow);
        rotor.torque = -rho * d * d * d * d * @abs(n) * (d * spec.torque * n - spec.torque * inflow);
        const wrench = &w[index];
        S.addForce(wrench, S.scale(axis, rotor.thrust), p, body.center_of_mass);
        wrench.torque = S.add(wrench.torque, S.scale(axis, rotor.torque));
    }
}
