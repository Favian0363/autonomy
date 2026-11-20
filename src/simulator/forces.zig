const std = @import("std");
const abi = @import("abi.zig");
const types = @import("types.zig");
const database = @import("database.zig");
const S = @import("support.zig");
const Session = @import("Session.zig");

pub fn evaluate(force: types.Force, runtime: *Session) !void {
    switch (force) {
        .gravity => gravity(runtime),
        .hydrodynamics => try hydro(runtime),
        .thrusters => thrusters(runtime),
    }
}

fn coefficients(force: types.Vec3, basis: types.Quat, cd: types.Vec3, scale: f64) types.Vec3 {
    const d = S.rotate(S.conjugate(basis), force);
    const len = S.norm(d);
    const factor = if (len < 1.4901161193847656e-8) 0 else scale * (@abs(d[0]) * cd[0] + @abs(d[1]) * cd[1] + @abs(d[2]) * cd[2]) / len;
    return S.scale(force, factor);
}

fn gravity(runtime: *Session) void {
    const scene = runtime.scene;
    for (runtime.wrenches, scene.objects) |*wrench, slot| {
        const e = database.entity(slot);
        if (database.motionFrom(e.motion) == .dynamic)
            wrench.force = S.toAbi3(S.add(S.fromAbi3(wrench.force), S.scale(scene.settings.gravity, e.dry_mass - e.mass)));
    }
}

fn hydro(runtime: *Session) !void {
    const model = runtime.scene;
    const bodies = runtime.backend.states;
    const w = runtime.wrenches;
    const robot = model.robot();
    if (runtime.tick % model.settings.hydro_period == 0) {
        const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
        @memset(runtime.fluid, zero_wrench);
        var scratch = std.heap.FixedBufferAllocator.init(runtime.scratch);
        const minz = try scratch.allocator().alloc(f64, w.len);
        const maxz = try scratch.allocator().alloc(f64, w.len);
        @memset(minz, std.math.inf(f64));
        @memset(maxz, -std.math.inf(f64));
        const Wet = struct { lo: f64 = std.math.inf(f64), hi: f64 = -std.math.inf(f64) };
        var wet: [types.max_hulls]Wet = @splat(.{});
        const hull_count = database.ownedCount(robot.db.hulls, robot.slot);
        for (0..hull_count) |hi| {
            const h = database.hullAt(robot.db, robot.slot, hi);
            const index = h.body;
            const transform = S.compose(S.bodyPose(bodies[index]), database.poseOf(h.pos, h.quat));
            const positions = database.vertexSlice(robot.db, h.vertex_start, h.vertex_count);
            for (positions) |position| {
                const z = S.point(transform, S.fromAbi3(position))[2];
                wet[hi].lo = @min(wet[hi].lo, z);
                wet[hi].hi = @max(wet[hi].hi, z);
            }
            minz[index] = @min(minz[index], wet[hi].lo);
            maxz[index] = @max(maxz[index], wet[hi].hi);
        }
        for (w, 0..) |_, index| if (minz[index] > 0 and std.math.isFinite(minz[index])) {
            const e = database.entity(model.objects[index]);
            const body = bodies[index];
            const pose = S.bodyPose(body);
            S.addForce(&runtime.fluid[index], S.scale(model.settings.gravity, -model.settings.water_density * e.volume), S.point(pose, S.fromAbi3(e.buoyancy)), S.fromAbi3(body.com));
        };
        for (0..hull_count) |hi| {
            const h = database.hullAt(robot.db, robot.slot, hi);
            const extent = wet[hi];
            const index = h.body;
            const body = bodies[index];
            if (maxz[index] <= 0) continue;
            const transform = S.compose(S.bodyPose(body), database.poseOf(h.pos, h.quat));
            const out = &runtime.fluid[index];
            if (minz[index] <= 0 and extent.hi > 0) {
                const fraction = if (extent.lo > 0) 1 else extent.hi / (extent.hi - extent.lo);
                S.addForce(out, S.scale(model.settings.gravity, -fraction * h.volume * model.settings.water_density), S.point(transform, S.fromAbi3(h.centroid)), S.fromAbi3(body.com));
            }
            var pressure_linear: types.Vec3 = @splat(0);
            var pressure_angular: types.Vec3 = @splat(0);
            var skin_linear: types.Vec3 = @splat(0);
            var skin_angular: types.Vec3 = @splat(0);
            const positions = database.vertexSlice(robot.db, h.vertex_start, h.vertex_count);
            var vertex: usize = 0;
            while (vertex < positions.len) : (vertex += 3) {
                const a = S.point(transform, S.fromAbi3(positions[vertex]));
                const b = S.point(transform, S.fromAbi3(positions[vertex + 1]));
                const d = S.point(transform, S.fromAbi3(positions[vertex + 2]));
                var normal = S.cross(S.sub(b, a), S.sub(d, a));
                const twice = S.norm(normal);
                if (twice < 1e-6) continue;
                normal = S.scale(normal, 1 / twice);
                const center = S.scale(S.add(S.add(a, b), d), 1.0 / 3.0);
                if (minz[index] <= 0 and center[2] <= 0) continue;
                const arm = S.sub(center, S.fromAbi3(body.com));
                const relative = S.sub(model.settings.current, S.add(S.fromAbi3(body.linear_vel), S.cross(S.fromAbi3(body.angular_vel), arm)));
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
            const frame = S.multiply(S.bodyPose(body).quat, S.fromAbi4(h.coefficient_frame));
            out.force = S.toAbi3(S.add(S.add(S.fromAbi3(out.force), coefficients(pressure_linear, frame, S.fromAbi3(h.drag), 0.5 * rho)), coefficients(skin_linear, frame, S.fromAbi3(h.skin), rho)));
            out.torque = S.toAbi3(S.add(S.add(S.fromAbi3(out.torque), coefficients(pressure_angular, frame, S.fromAbi3(h.drag), 0.5 * rho)), coefficients(skin_angular, frame, S.fromAbi3(h.skin), rho)));
        }
    }
    for (w, 0..) |*wrench, index| {
        wrench.force = S.toAbi3(S.add(S.fromAbi3(wrench.force), S.fromAbi3(runtime.fluid[index].force)));
        wrench.torque = S.toAbi3(S.add(S.fromAbi3(wrench.torque), S.fromAbi3(runtime.fluid[index].torque)));
    }
}

fn thrusters(runtime: *Session) void {
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
        const pose = S.bodyPose(body);
        const p = S.point(pose, S.fromAbi3(t.position));
        const axis = S.rotate(pose.quat, S.fromAbi3(t.direction));
        rotor.thrust = 0;
        if (p[2] <= 0) {
            rotor.torque = 0;
            continue;
        }
        const inflow = S.dot(axis, S.sub(S.add(S.fromAbi3(body.linear_vel), S.cross(S.fromAbi3(body.angular_vel), S.sub(p, S.fromAbi3(body.com)))), model.settings.current));
        const n = rotor.omega / (2 * std.math.pi);
        const d = t.diameter;
        const rho = model.settings.water_density;
        rotor.thrust = rho * d * d * d * @abs(n) * (d * t.thrust * n - t.thrust * inflow);
        rotor.torque = -rho * d * d * d * d * @abs(n) * (d * t.torque * n - t.torque * inflow);
        const wrench = &runtime.wrenches[index];
        S.addForce(wrench, S.scale(axis, rotor.thrust), p, S.fromAbi3(body.com));
        wrench.torque = S.toAbi3(S.add(S.fromAbi3(wrench.torque), S.scale(axis, rotor.torque)));
    }
}