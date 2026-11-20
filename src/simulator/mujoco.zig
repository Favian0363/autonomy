//! MuJoCo backend owns the engine handles and borrows its scene.
const std = @import("std");
const mj = @import("mujoco_c");
const types = @import("types.zig");
const abi = @import("abi.zig");
const Scene = @import("Scene.zig");
const database = @import("database.zig");
const S = @import("support.zig");

pub const MujocoBackend = @This();

source: *const Scene,
allocator: std.mem.Allocator,
model: ?*mj.mjModel = null,
data: ?*mj.mjData = null,
body_ids: []c_int = &.{},
states: []abi.BodyState = &.{},
contacts: std.ArrayList(abi.Contact) = .empty,

pub fn deinit(self: *MujocoBackend) void {
    if (self.data) |data| mj.mj_deleteData(data);
    if (self.model) |model| mj.mj_deleteModel(model);
    self.contacts.deinit(self.allocator);
    self.allocator.free(self.states);
    self.allocator.free(self.body_ids);
    self.* = undefined;
}

fn externalId(self: *const MujocoBackend, engine_id: c_int) u32 {
    for (self.body_ids, self.source.objects) |id, slot| {
        if (id == engine_id) return database.entity(slot).id;
    }
    return std.math.maxInt(u32);
}

fn refresh(self: *MujocoBackend) !void {
    const m = self.model.?;
    const d = self.data.?;
    mj.mj_forward(m, d);
    self.contacts.clearRetainingCapacity();
    for (self.states, 0..) |*state, i| {
        const id: usize = @intCast(self.body_ids[i]);
        const p = d.xpos + 3 * id;
        const q = d.xquat + 4 * id;
        const com = d.xipos + 3 * id;
        var velocity: [6]f64 = undefined;
        mj.mj_objectVelocity(m, d, mj.mjOBJ_BODY, @intCast(id), &velocity, 0);
        state.* = .{
            .id = database.entity(self.source.objects[i]).id,
            .pose_pos = S.toAbi3(.{ p[0], p[1], p[2] }),
            .pose_quat = S.toAbi4(.{ q[1], q[2], q[3], q[0] }),
            .com = S.toAbi3(.{ com[0], com[1], com[2] }),
            .linear_vel = S.toAbi3(.{ velocity[3], velocity[4], velocity[5] }),
            .angular_vel = S.toAbi3(.{ velocity[0], velocity[1], velocity[2] }),
        };
    }
    for (0..@intCast(d.ncon)) |i| {
        const contact = &d.contact[i];
        const a = m.geom_bodyid[@intCast(contact.geom[0])];
        const b = m.geom_bodyid[@intCast(contact.geom[1])];
        if (a == b) continue;
        var force: [6]f64 = @splat(0);
        mj.mj_contactForce(m, d, @intCast(i), &force);
        try self.contacts.append(self.allocator, .{
            .a = self.externalId(a),
            .b = self.externalId(b),
            .position = S.toAbi3(.{ contact.pos[0], contact.pos[1], contact.pos[2] }),
            .normal = S.toAbi3(.{ contact.frame[0], contact.frame[1], contact.frame[2] }),
            .pad = 0,
            .normal_force = force[0],
        });
    }
}

pub fn reset(self: *MujocoBackend, input: ?[]const abi.BodyState) !void {
    if (input) |states| if (states.len != self.source.objects.len) return error.InvalidReset;
    if (input) |states| for (states, self.source.objects, 0..) |*state, slot, i| {
        const initial = self.source.initialPose(i);
        const e = database.entity(slot);
        const motion = database.motionFrom(e.motion);
        const pose = S.bodyPose(state.*);
        if (state.id != e.id or !S.finite3(S.fromAbi3(state.linear_vel)) or !S.finite3(S.fromAbi3(state.angular_vel))) return error.InvalidReset;
        try S.validPose(pose);
        if (motion == .static) {
            for (pose.pos, initial.pos) |a, z| if (a != z) return error.InvalidReset;
            for (pose.quat, initial.quat) |a, z| if (a != z) return error.InvalidReset;
        }
    };
    const m = self.model.?;
    const d = self.data.?;
    mj.mj_resetData(m, d);
    for (self.source.objects, 0..) |slot, i| {
        const e = database.entity(slot);
        const initial = self.source.initialPose(i);
        const state: abi.BodyState = if (input) |states| states[i] else .{
            .id = e.id,
            .pose_pos = S.toAbi3(initial.pos),
            .pose_quat = S.toAbi4(initial.quat),
            .com = @splat(0),
            .linear_vel = e.initial_velocity,
            .angular_vel = e.initial_angular_velocity,
        };
        const pose = S.bodyPose(state);
        const id: usize = @intCast(self.body_ids[i]);
        const motion = database.motionFrom(e.motion);
        if (motion == .dynamic) {
            const joint: usize = @intCast(m.body_jntadr[id]);
            const qa: usize = @intCast(m.jnt_qposadr[joint]);
            const va: usize = @intCast(m.jnt_dofadr[joint]);
            transform(d.qpos + qa, d.qpos + qa + 3, pose);
            const com = database.entityCenterOfMass(slot.db, slot.slot);
            const offset = S.rotate(pose.quat, com.pos);
            xyz(d.qvel + va, S.sub(S.fromAbi3(state.linear_vel), S.cross(S.fromAbi3(state.angular_vel), offset)));
            xyz(d.qvel + va + 3, S.rotate(S.conjugate(pose.quat), S.fromAbi3(state.angular_vel)));
        } else if (motion == .kinematic) {
            const k: usize = @intCast(m.body_mocapid[id]);
            transform(d.mocap_pos + 3 * k, d.mocap_quat + 4 * k, pose);
        }
    }
    try self.refresh();
}

pub fn step(self: *MujocoBackend, input: []const abi.Wrench) !void {
    for (input, self.source.objects) |wrench, slot| {
        if (wrench.id != database.entity(slot).id or
            !S.finite3(S.fromAbi3(wrench.force)) or !S.finite3(S.fromAbi3(wrench.torque)))
            return error.InvalidWrench;
    }
    const m = self.model.?;
    const d = self.data.?;
    @memset(d.xfrc_applied[0..@intCast(6 * m.nbody)], 0);
    for (input, self.body_ids) |wrench, id| {
        const out = d.xfrc_applied + 6 * @as(usize, @intCast(id));
        xyzAbi(out, wrench.force);
        xyzAbi(out + 3, wrench.torque);
    }
    mj.mj_step(m, d);
    try self.refresh();
}

pub fn init(allocator: std.mem.Allocator, scene: *const Scene) !MujocoBackend {
    var self: MujocoBackend = .{ .source = scene, .allocator = allocator };
    errdefer self.deinit();
    self.body_ids = try allocator.alloc(c_int, scene.objects.len);
    self.states = try allocator.alloc(abi.BodyState, scene.objects.len);
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
    for (scene.objects, 0..) |slot, i| {
        const e = database.entity(slot);
        const initial = scene.initialPose(i);
        const body = mj.mjs_addBody(root, null);
        if (body == null) return error.OutOfMemory;
        _ = mj.mjs_setName(body.*.element, (try namePrint(&name_buffer, "b{d}", .{e.id})).ptr);
        transform(&body.*.pos, &body.*.quat, initial);
        const motion = database.motionFrom(e.motion);
        body.*.mocap = if (@TypeOf(body.*.mocap) == bool) motion == .kinematic else @intFromBool(motion == .kinematic);
        if (motion == .dynamic) {
            if (mj.mjs_addFreeJoint(body) == null) return error.OutOfMemory;
            body.*.explicitinertial = if (@TypeOf(body.*.explicitinertial) == bool) true else 1;
            const com = database.entityCenterOfMass(slot.db, slot.slot);
            transform(&body.*.ipos, &body.*.iquat, com);
            body.*.mass = e.mass;
            const placeholder = @max(e.inertia[0], e.inertia[1], e.inertia[2]);
            xyz(&body.*.inertia, .{ placeholder, placeholder, placeholder });
        }
        const collider_count = database.ownedCount(slot.db.colliders, slot.slot);
        for (0..collider_count) |k| {
            const c = database.colliderAt(slot.db, slot.slot, k);
            if (c.restitution != 0) return error.UnsupportedRestitution;
            const geom = mj.mjs_addGeom(body, null);
            if (geom == null) return error.OutOfMemory;
            _ = mj.mjs_setName(geom.*.element, (try namePrint(&name_buffer, "g{d}_{d}", .{ i, k })).ptr);
            transform(&geom.*.pos, &geom.*.quat, database.poseOf(c.pos, c.quat));
            geom.*.density = 0;
            geom.*.contype = @intFromBool(c.collisions != 0);
            geom.*.conaffinity = @intFromBool(c.collisions != 0);
            xyz(&geom.*.friction, .{ c.friction, 0, 0 });
            const shape = database.shapeFrom(slot.db, c);
            if (shape == .mesh) {
                const mesh = mj.mjs_addMesh(spec, null);
                if (mesh == null) return error.OutOfMemory;
                const name = try namePrint(&name_buffer, "m{d}_{d}", .{ i, k });
                _ = mj.mjs_setName(mesh.*.element, name.ptr);
                mj.mjs_setString(geom.*.meshname, name.ptr);
                const positions = try self.allocator.alloc(f32, shape.mesh.positions.len * 3);
                defer self.allocator.free(positions);
                const triangles = try self.allocator.alloc(c_int, shape.mesh.triangles.len * 3);
                defer self.allocator.free(triangles);
                for (shape.mesh.positions, 0..) |position, v|
                    @memcpy(positions[3 * v ..][0..3], &position);
                for (shape.mesh.triangles, 0..) |face, face_index| {
                    triangles[3 * face_index ..][0..3].* = .{
                        @intCast(face.a), @intCast(face.b), @intCast(face.c),
                    };
                }
                mj.mjs_setFloat(mesh.*.uservert, positions.ptr, @intCast(positions.len));
                mj.mjs_setInt(mesh.*.userface, triangles.ptr, @intCast(triangles.len));
                geom.*.type = mj.mjGEOM_MESH;
            } else {
                geom.*.type = if (shape == .box) mj.mjGEOM_BOX else if (shape == .sphere) mj.mjGEOM_SPHERE else mj.mjGEOM_CYLINDER;
                if (shape == .box) xyzAbi(&geom.*.size, c.size) else if (shape == .sphere) geom.*.size[0] = c.size[0] else xyz(&geom.*.size, .{ c.size[0], c.size[2], 0 });
            }
        }
    }
    self.model = mj.mj_compile(spec, null);
    if (self.model == null) {
        std.log.warn("MuJoCo compile: {s}", .{std.mem.span(mj.mjs_getError(spec))});
        return error.MuJoCoCompileFailed;
    }
    const m = self.model.?;
    for (scene.objects, 0..) |slot, i| {
        const e = database.entity(slot);
        const id = mj.mj_name2id(m, mj.mjOBJ_BODY, (try namePrint(&name_buffer, "b{d}", .{e.id})).ptr);
        if (id < 0) return error.MuJoCoBodyMissing;
        self.body_ids[i] = id;
        if (database.motionFrom(e.motion) == .dynamic) xyz(m.body_inertia + 3 * @as(usize, @intCast(id)), S.fromAbi3(e.inertia));
        const collider_count = database.ownedCount(slot.db.colliders, slot.slot);
        for (0..collider_count) |k| {
            const geom_id = mj.mj_name2id(m, mj.mjOBJ_GEOM, (try namePrint(&name_buffer, "g{d}_{d}", .{ i, k })).ptr);
            if (geom_id < 0) return error.MuJoCoGeomMissing;
        }
    }
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
    self.data = mj.mj_makeData(m);
    if (self.data == null) return error.OutOfMemory;
    try self.reset(null);
    return self;
}

fn namePrint(buffer: []u8, comptime format: []const u8, args: anytype) ![:0]u8 {
    return std.mem.printSentinel(buffer, format, args, 0);
}
fn xyz(out: [*c]f64, vector: types.Vec3) void {
    for (0..3) |i| out[i] = vector[i];
}
fn xyzAbi(out: [*c]f64, vector: abi.Vec3) void {
    for (0..3) |i| out[i] = @floatCast(vector[i]);
}
fn transform(pos: [*c]f64, quat: [*c]f64, pose: types.Pose) void {
    xyz(pos, pose.pos);
    quat[0] = pose.quat[3];
    quat[1] = pose.quat[0];
    quat[2] = pose.quat[1];
    quat[3] = pose.quat[2];
}