const std = @import("std");
const types = @import("types.zig");
const database = @import("database.zig");
const S = @import("support.zig");
const Runtime = @import("Runtime.zig");

pub fn evaluate(force: types.Force, runtime: *Runtime) !void {
    switch (force) {
        .gravity => gravity(runtime),
        .hydrodynamics => try hydro(runtime),
        .thrusters => thrusters(runtime),
    }
}

fn coefficients(force: types.Vec3, basis: types.Quat, cd: types.Vec3, scale: f64) types.Vec3 {
    @setFloatMode(.strict);
    const d = S.rotate(S.conjugate(basis), force);
    const len = S.norm(d);
    const factor = if (len < 1.4901161193847656e-8) 0 else scale * (@abs(d[0]) * cd[0] + @abs(d[1]) * cd[1] + @abs(d[2]) * cd[2]) / len;
    return S.scale(force, factor);
}

fn gravity(runtime: *Runtime) void {
    @setFloatMode(.strict);
    const scene = runtime.scene;
    for (runtime.wrenches, scene.objects) |*wrench, slot| {
        const e = database.entity(slot);
        if (database.motionFrom(e.motion) == .dynamic)
            wrench.force = S.add(wrench.force, S.scale(scene.settings.gravity, e.dry_mass - e.mass));
    }
}

fn hydro(runtime: *Runtime) !void {
    @setFloatMode(.strict);
    const model = runtime.scene;
    const bodies = runtime.backend.states;
    const w = runtime.wrenches;
    const robot = model.robot();
    if (runtime.tick % model.settings.hydro_period == 0) {
        @memset(runtime.fluid, .{});
        var scratch = std.heap.FixedBufferAllocator.init(runtime.scratch);
        const minz = try scratch.allocator().alloc(f64, w.len);
        const maxz = try scratch.allocator().alloc(f64, w.len);
        @memset(minz, std.math.inf(f64));
        @memset(maxz, -std.math.inf(f64));
        const Wet = struct { lo: f64 = std.math.inf(f64), hi: f64 = -std.math.inf(f64) };
        var wet: [types.max_hulls]Wet = @splat(.{});
        const hull_count = database.ownedCount(robot.db.hulls.items, robot.slot);
        for (0..hull_count) |hi| {
            const h = database.hullAt(robot.db, robot.slot, hi);
            const index = h.body;
            const transform = S.compose(bodies[index].pose, database.poseOf(h.pos, h.quat));
            const positions = database.vertexSlice(robot.db, h.vertex_start, h.vertex_count);
            for (positions) |position| {
                const z = S.point(transform, position)[2];
                wet[hi].lo = @min(wet[hi].lo, z);
                wet[hi].hi = @max(wet[hi].hi, z);
            }
            minz[index] = @min(minz[index], wet[hi].lo);
            maxz[index] = @max(maxz[index], wet[hi].hi);
        }
        for (w, 0..) |_, index| if (minz[index] > 0 and std.math.isFinite(minz[index])) {
            const e = database.entity(model.objects[index]);
            S.addForce(&runtime.fluid[index], S.scale(model.settings.gravity, -model.settings.water_density * e.volume), S.point(bodies[index].pose, e.buoyancy), bodies[index].center_of_mass);
        };
        for (0..hull_count) |hi| {
            const h = database.hullAt(robot.db, robot.slot, hi);
            const extent = wet[hi];
            const index = h.body;
            const body = bodies[index];
            if (maxz[index] <= 0) continue;
            const transform = S.compose(body.pose, database.poseOf(h.pos, h.quat));
            const out = &runtime.fluid[index];
            if (minz[index] <= 0 and extent.hi > 0) {
                const fraction = if (extent.lo > 0) 1 else extent.hi / (extent.hi - extent.lo);
                S.addForce(out, S.scale(model.settings.gravity, -fraction * h.volume * model.settings.water_density), S.point(transform, h.centroid), body.center_of_mass);
            }
            var pressure_linear = S.v(0, 0, 0);
            var pressure_angular = S.v(0, 0, 0);
            var skin_linear = S.v(0, 0, 0);
            var skin_angular = S.v(0, 0, 0);
            const positions = database.vertexSlice(robot.db, h.vertex_start, h.vertex_count);
            var vertex: usize = 0;
            while (vertex < positions.len) : (vertex += 3) {
                const a = S.point(transform, positions[vertex]);
                const b = S.point(transform, positions[vertex + 1]);
                const d = S.point(transform, positions[vertex + 2]);
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
        wrench.force = S.add(wrench.force, runtime.fluid[index].force);
        wrench.torque = S.add(wrench.torque, runtime.fluid[index].torque);
    }
}

fn thrusters(runtime: *Runtime) void {
    @setFloatMode(.strict);
    const model = runtime.scene;
    const bodies = runtime.backend.states;
    const dt = model.settings.timestep();
    const robot = model.robot();
    for (0..model.thruster_len) |ti| {
        const t = database.thrusterAt(robot.db, robot.slot, ti);
        const rotor = &runtime.rotors[ti];
        const command = runtime.commands[ti];
        const index = t.body;
        const body = bodies[index];
        const max = t.max_rpm / 60 * 2 * std.math.pi;
        const target = t.command_sign * std.math.clamp(command, -1, 1) * max;
        const error_omega = target - rotor.omega;
        const damping = if (rotor.omega > 0) @abs(rotor.torque) else -@abs(rotor.torque);
        rotor.omega = std.math.clamp((t.inertia * rotor.omega + dt * (t.kp * target + t.ki * rotor.integral - damping)) / (t.inertia + dt * t.kp), -2 * max, 2 * max);
        rotor.integral = std.math.clamp(rotor.integral + error_omega * dt, -5, 5);
        const p = S.point(body.pose, t.position);
        const axis = S.rotate(body.pose.quat, t.direction);
        rotor.thrust = 0;
        if (p[2] <= 0) {
            rotor.torque = 0;
            continue;
        }
        const inflow = S.dot(axis, S.sub(S.add(body.linear_velocity, S.cross(body.angular_velocity, S.sub(p, body.center_of_mass))), model.settings.current));
        const n = rotor.omega / (2 * std.math.pi);
        const d = t.diameter;
        const rho = model.settings.water_density;
        rotor.thrust = rho * d * d * d * @abs(n) * (d * t.thrust * n - t.thrust * inflow);
        rotor.torque = -rho * d * d * d * d * @abs(n) * (d * t.torque * n - t.torque * inflow);
        const wrench = &runtime.wrenches[index];
        S.addForce(wrench, S.scale(axis, rotor.thrust), p, body.center_of_mass);
        wrench.torque = S.add(wrench.torque, S.scale(axis, rotor.torque));
    }
}