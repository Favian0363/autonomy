//! MuJoCo engine adapter. World owns the engine handles and borrows its scene.
const std = @import("std");
const mj = @import("mujoco_c");
const types = @import("types.zig");
const Scene = @import("Scene.zig");
const S = @import("support.zig");

pub const World = struct {
    source: *const Scene,
    allocator: std.mem.Allocator,
    model: ?*mj.mjModel = null,
    data: ?*mj.mjData = null,
    body_ids: []c_int = &.{},
    states: []types.BodyState = &.{},
    contacts: std.ArrayList(types.Contact) = .empty,
    hash_state: []mj.mjtNum = &.{},

    pub fn deinit(w: *World) void {
        if (w.data) |data| mj.mj_deleteData(data);
        if (w.model) |model| mj.mj_deleteModel(model);
        w.allocator.free(w.hash_state);
        w.contacts.deinit(w.allocator);
        w.allocator.free(w.states);
        w.allocator.free(w.body_ids);
        w.* = undefined;
    }

    fn externalId(w: *const World, engine_id: c_int) u32 {
        for (w.body_ids, w.source.objects) |id, object| {
            if (id == engine_id) return object.body.id;
        }
        return std.math.maxInt(u32);
    }

    fn refresh(w: *World) !void {
        const m = w.model.?;
        const d = w.data.?;
        mj.mj_forward(m, d);
        w.contacts.clearRetainingCapacity();
        for (w.states, 0..) |*state, i| {
            const id: usize = @intCast(w.body_ids[i]);
            const p = d.xpos + 3 * id;
            const q = d.xquat + 4 * id;
            const com = d.xipos + 3 * id;
            var velocity: [6]f64 = undefined;
            mj.mj_objectVelocity(m, d, mj.mjOBJ_BODY, @intCast(id), &velocity, 0);
            state.* = .{
                .id = w.source.objects[i].body.id,
                .pose = .{ .pos = S.v(p[0], p[1], p[2]), .quat = S.q(q[1], q[2], q[3], q[0]) },
                .center_of_mass = S.v(com[0], com[1], com[2]),
                .linear_velocity = S.v(velocity[3], velocity[4], velocity[5]),
                .angular_velocity = S.v(velocity[0], velocity[1], velocity[2]),
            };
        }
        for (0..@intCast(d.ncon)) |i| {
            const contact = &d.contact[i];
            const a = m.geom_bodyid[@intCast(contact.geom[0])];
            const b = m.geom_bodyid[@intCast(contact.geom[1])];
            if (a == b) continue;
            var force: [6]f64 = @splat(0);
            mj.mj_contactForce(m, d, @intCast(i), &force);
            try w.contacts.append(w.allocator, .{
                .a = w.externalId(a),
                .b = w.externalId(b),
                .position = S.v(contact.pos[0], contact.pos[1], contact.pos[2]),
                .normal = S.v(contact.frame[0], contact.frame[1], contact.frame[2]),
                .normal_force = force[0],
            });
        }
    }

    pub fn reset(w: *World, input: ?[]const types.BodyState) !void {
        if (input) |states| if (states.len != w.source.objects.len) return error.InvalidReset;
        // Validate every entry before resetting or mutating engine state.
        if (input) |states| for (states, w.source.objects) |*state, *object| {
            const b = &object.body;
            if (state.id != b.id or !S.finite3(state.linear_velocity) or !S.finite3(state.angular_velocity)) return error.InvalidReset;
            try S.validPose(state.pose);
            if (b.motion_type == .static) {
                for (state.pose.pos, b.initial_pose.pos) |a, z| if (a != z) return error.InvalidReset;
                for (state.pose.quat, b.initial_pose.quat) |a, z| if (a != z) return error.InvalidReset;
            }
        };
        const m = w.model.?;
        const d = w.data.?;
        mj.mj_resetData(m, d);
        for (w.source.objects, 0..) |*object, i| {
            const b = &object.body;
            const state: types.BodyState = if (input) |states| states[i] else .{
                .id = b.id,
                .pose = b.initial_pose,
                .center_of_mass = @splat(0),
                .linear_velocity = b.initial_velocity,
                .angular_velocity = b.initial_angular_velocity,
            };
            const id: usize = @intCast(w.body_ids[i]);
            if (b.motion_type == .dynamic) {
                const joint: usize = @intCast(m.body_jntadr[id]);
                const qa: usize = @intCast(m.jnt_qposadr[joint]);
                const va: usize = @intCast(m.jnt_dofadr[joint]);
                transform(d.qpos + qa, d.qpos + qa + 3, state.pose);
                // Freejoint translation is at the body origin; simulator velocity is at COM.
                const offset = S.rotate(state.pose.quat, b.center_of_mass.pos);
                xyz(d.qvel + va, S.sub(state.linear_velocity, S.cross(state.angular_velocity, offset)));
                xyz(d.qvel + va + 3, S.rotate(S.conjugate(state.pose.quat), state.angular_velocity));
            } else if (b.motion_type == .kinematic) {
                const k: usize = @intCast(m.body_mocapid[id]);
                transform(d.mocap_pos + 3 * k, d.mocap_quat + 4 * k, state.pose);
            }
        }
        try w.refresh();
    }

    /// Input is one world-frame wrench per body in scene order.
    pub fn step(w: *World, input: []const types.Wrench) !void {
        if (input.len != w.source.objects.len) return error.InvalidWrenchCount;
        for (input, w.source.objects) |wrench, object| {
            if (wrench.id != object.body.id or !S.finite3(wrench.force) or !S.finite3(wrench.torque))
                return error.InvalidWrench;
        }
        const m = w.model.?;
        const d = w.data.?;
        @memset(d.xfrc_applied[0..@intCast(6 * m.nbody)], 0);
        for (input, w.body_ids) |wrench, id| {
            const out = d.xfrc_applied + 6 * @as(usize, @intCast(id));
            xyz(out, wrench.force);
            xyz(out + 3, wrench.torque);
        }
        mj.mj_step(m, d);
        try w.refresh();
    }

    pub fn hash(w: *const World) u64 {
        mj.mj_getState(w.model, w.data, w.hash_state.ptr, mj.mjSTATE_INTEGRATION);
        var h: S.Hash = .{};
        for (w.hash_state) |n| h.scalar(f64, n);
        return h.value;
    }

    pub fn init(allocator: std.mem.Allocator, scene: *const Scene) !World {
        var w: World = .{ .source = scene, .allocator = allocator };
        errdefer w.deinit();
        w.body_ids = try allocator.alloc(c_int, scene.objects.len);
        w.states = try allocator.alloc(types.BodyState, scene.objects.len);
        const spec = mj.mj_makeSpec();
        if (spec == null) return error.OutOfMemory;
        defer mj.mj_deleteSpec(spec);
        spec.*.compiler.degree = if (@TypeOf(spec.*.compiler.degree) == bool) false else 0;
        spec.*.option.timestep = scene.settings.timestep();
        xyz(&spec.*.option.gravity, scene.settings.gravity);
        spec.*.option.integrator = mj.mjINT_IMPLICITFAST;
        spec.*.option.enableflags &= ~@as(c_int, mj.mjENBL_SLEEP);
        const root = mj.mjs_findBody(spec, "world");
        if (root == null) return error.MuJoCoRootMissing;
        var name_buffer: [80]u8 = undefined;
        for (scene.objects, 0..) |*object, i| {
            const b = &object.body;
            const body = mj.mjs_addBody(root, null);
            if (body == null) return error.OutOfMemory;
            _ = mj.mjs_setName(body.*.element, (try namePrint(&name_buffer, "b{d}", .{b.id})).ptr);
            transform(&body.*.pos, &body.*.quat, b.initial_pose);
            body.*.mocap = if (@TypeOf(body.*.mocap) == bool) b.motion_type == .kinematic else @intFromBool(b.motion_type == .kinematic);
            if (b.motion_type == .dynamic) {
                if (mj.mjs_addFreeJoint(body) == null) return error.OutOfMemory;
                body.*.explicitinertial = if (@TypeOf(body.*.explicitinertial) == bool) true else 1;
                transform(&body.*.ipos, &body.*.iquat, b.center_of_mass);
                body.*.mass = b.mass;
                // Compile with isotropic inertia, then restore calibrated moments below.
                const placeholder = @max(b.inertia[0], b.inertia[1], b.inertia[2]);
                xyz(&body.*.inertia, S.v(placeholder, placeholder, placeholder));
            }
            for (object.colliders, 0..) |*g, k| {
                if (g.restitution != 0) return error.UnsupportedRestitution;
                const geom = mj.mjs_addGeom(body, null);
                if (geom == null) return error.OutOfMemory;
                _ = mj.mjs_setName(geom.*.element, (try namePrint(&name_buffer, "g{d}_{d}", .{ i, k })).ptr);
                transform(&geom.*.pos, &geom.*.quat, g.pose);
                geom.*.density = 0;
                geom.*.contype = @intFromBool(g.collisions);
                geom.*.conaffinity = @intFromBool(g.collisions);
                xyz(&geom.*.friction, S.v(g.friction, 0, 0));
                if (g.shape == .mesh) {
                    const geometry = g.shape.mesh;
                    const mesh = mj.mjs_addMesh(spec, null);
                    if (mesh == null) return error.OutOfMemory;
                    const name = try namePrint(&name_buffer, "m{d}_{d}", .{ i, k });
                    _ = mj.mjs_setName(mesh.*.element, name.ptr);
                    mj.mjs_setString(geom.*.meshname, name.ptr);
                    const positions = try w.allocator.alloc(f32, geometry.positions.len * 3);
                    defer w.allocator.free(positions);
                    const triangles = try w.allocator.alloc(c_int, geometry.triangles.len * 3);
                    defer w.allocator.free(triangles);
                    for (geometry.positions, 0..) |position, v|
                        @memcpy(positions[3 * v ..][0..3], &position);
                    for (geometry.triangles, 0..) |face, face_index| {
                        triangles[3 * face_index ..][0..3].* = .{
                            @intCast(face.a), @intCast(face.b), @intCast(face.c),
                        };
                    }
                    // These C setters copy into MuJoCo-owned asset buffers.
                    mj.mjs_setFloat(mesh.*.uservert, positions.ptr, @intCast(positions.len));
                    mj.mjs_setInt(mesh.*.userface, triangles.ptr, @intCast(triangles.len));
                    geom.*.type = mj.mjGEOM_MESH;
                } else {
                    geom.*.type = if (g.shape == .box) mj.mjGEOM_BOX else if (g.shape == .sphere) mj.mjGEOM_SPHERE else mj.mjGEOM_CYLINDER;
                    if (g.shape == .box) xyz(&geom.*.size, g.size) else if (g.shape == .sphere) geom.*.size[0] = g.size[0] else xyz(&geom.*.size, S.v(g.size[0], g.size[2], 0));
                }
            }
        }
        w.model = mj.mj_compile(spec, null);
        if (w.model == null) {
            std.log.warn("MuJoCo compile: {s}", .{std.mem.span(mj.mjs_getError(spec))});
            return error.MuJoCoCompileFailed;
        }
        const m = w.model.?;
        for (scene.objects, 0..) |*object, i| {
            const b = &object.body;
            const id = mj.mj_name2id(m, mj.mjOBJ_BODY, (try namePrint(&name_buffer, "b{d}", .{b.id})).ptr);
            if (id < 0) return error.MuJoCoBodyMissing;
            w.body_ids[i] = id;
            if (b.motion_type == .dynamic) xyz(m.body_inertia + 3 * @as(usize, @intCast(id)), b.inertia);
            for (0..object.colliders.len) |k| {
                const geom_id = mj.mj_name2id(m, mj.mjOBJ_GEOM, (try namePrint(&name_buffer, "g{d}_{d}", .{ i, k })).ptr);
                if (geom_id < 0) return error.MuJoCoGeomMissing;
            }
        }
        w.hash_state = try w.allocator.alloc(mj.mjtNum, @intCast(mj.mj_stateSize(m, mj.mjSTATE_INTEGRATION)));
        m.opt.integrator = switch (scene.settings.integrator) {
            .euler => mj.mjINT_EULER,
            .implicit => mj.mjINT_IMPLICIT,
            .rk4 => mj.mjINT_RK4,
            .default, .implicit_fast => mj.mjINT_IMPLICITFAST,
        };
        const constants = mj.mj_makeData(m);
        if (constants == null) return error.OutOfMemory;
        mj.mj_setConst(m, constants);
        mj.mj_deleteData(constants);
        w.data = mj.mj_makeData(m);
        if (w.data == null) return error.OutOfMemory;
        try w.reset(null);
        return w;
    }
};

fn namePrint(buffer: []u8, comptime format: []const u8, args: anytype) ![:0]u8 {
    return std.mem.printSentinel(buffer, format, args, 0);
}
fn xyz(out: [*c]f64, vector: types.Vec3) void {
    for (0..3) |i| out[i] = vector[i];
}
fn quaternion(out: [*c]f64, q: types.Quat) void {
    // Shared xyzw quaternions and MuJoCo wxyz, both in NED coordinates.
    out[0] = q[3];
    for (0..3) |i| out[i + 1] = q[i];
}
fn transform(pos: [*c]f64, quat: [*c]f64, p: types.Pose) void {
    xyz(pos, p.pos);
    quaternion(quat, p.quat);
}
