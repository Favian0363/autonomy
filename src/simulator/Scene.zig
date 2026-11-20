//! Validated scene records. The runtime and MuJoCo backend borrow this scene.
const Scene = @This();
const std = @import("std");
const types = @import("types.zig");
const math = @import("support.zig");
const database = @import("database.zig");

allocator: std.mem.Allocator,
settings: types.Settings,
objects: []const database.BodySlot,
robot_spawn_pose: ?types.Pose = null,
thruster_len: u8,
sensor_len: u8,
routing: types.Routing,
faults: []const types.Fault,
forces: []const types.Force = &.{ .gravity, .hydrodynamics, .thrusters },

pub fn robot(self: *const Scene) database.BodySlot {
    return self.objects[0];
}

pub fn init(
    allocator: std.mem.Allocator,
    robot_slot: database.BodySlot,
    start: ?types.Pose,
    additional: []const database.BodySlot,
    settings: types.Settings,
    faults: []const types.Fault,
    membership: []const database.Membership,
) !Scene {
    if (additional.len >= 4096) return error.SceneCapacity;
    const thruster_count = database.ownedCount(robot_slot.db.thrusters, robot_slot.slot);
    const hull_count = database.ownedCount(robot_slot.db.hulls, robot_slot.slot);
    const sensor_count = database.ownedCount(robot_slot.db.sensors, robot_slot.slot);
    if (thruster_count > types.thruster_count or hull_count > types.max_hulls or sensor_count > 64)
        return error.SceneCapacity;

    const objects = try allocator.alloc(database.BodySlot, additional.len + 1);
    errdefer allocator.free(objects);
    objects[0] = robot_slot;
    @memcpy(objects[1..], additional);
    const owned_faults = try allocator.dupe(types.Fault, faults);
    errdefer allocator.free(owned_faults);
    if (thruster_count > std.math.maxInt(u8)) return error.SceneCapacity;
    var scene: Scene = .{
        .allocator = allocator,
        .settings = settings,
        .objects = objects,
        .robot_spawn_pose = start,
        .thruster_len = @intCast(thruster_count),
        .sensor_len = @intCast(sensor_count),
        .routing = .initFill(null),
        .faults = owned_faults,
    };
    try scene.validate();
    for (membership) |group| try group.db.spliceActive(group.slots);
    return scene;
}

pub fn deinit(self: *Scene) void {
    self.allocator.free(self.faults);
    self.allocator.free(self.objects);
    self.* = undefined;
}

pub fn initialPose(self: *const Scene, index: usize) types.Pose {
    const o = self.objects[index];
    const override = if (index == 0) self.robot_spawn_pose else null;
    return database.entityInitialPose(o.db, o.slot, override);
}

pub fn centerOfMass(self: *const Scene, index: usize) types.Pose {
    const o = self.objects[index];
    return database.entityCenterOfMass(o.db, o.slot);
}

fn validate(self: *Scene) !void {
    const s = self.settings;
    if (s.dt_numerator == 0 or s.dt_denominator == 0 or s.hydro_period == 0 or
        !math.finite3(s.gravity) or !math.finite3(s.current) or !positive(s.water_density) or
        !positive(s.bounds.position) or !positive(s.bounds.speed) or !positive(s.bounds.angular_speed))
        return error.InvalidSettings;

    for (self.objects, 0..) |object, i| {
        const e = database.entity(object);
        const name = database.entityName(object.db, object.slot);
        if (e.id == std.math.maxInt(u32) or name.len >= 64 or
            !math.finite3(e.initial_velocity) or !math.finite3(e.initial_angular_velocity) or
            !math.finite3(e.inertia) or !math.finite3(e.buoyancy) or
            !nonnegative(e.mass) or !nonnegative(e.dry_mass) or !nonnegative(e.volume))
            return error.InvalidBody;
        const motion = database.motionFrom(e.motion);
        if (motion == .dynamic and
            !(positive(e.mass) and positive(e.inertia[0]) and positive(e.inertia[1]) and positive(e.inertia[2])))
            return error.InvalidBody;
        try math.validPose(self.initialPose(i));
        try math.validPose(database.entityCenterOfMass(object.db, object.slot));
        for (self.objects[0..i]) |other| if (e.id == database.entity(other).id) return error.DuplicateBodyId;
        const collider_count = database.ownedCount(object.db.colliders, object.slot);
        if (collider_count == 0 or collider_count > 1024) return error.InvalidColliders;
        for (0..collider_count) |k| try validateCollider(object.db, database.colliderAt(object.db, object.slot, k));
    }
    const robot_slot = self.robot();
    for (0..self.thruster_len) |index| {
        const t = database.thrusterAt(robot_slot.db, robot_slot.slot, index);
        if (t.body >= self.objects.len or !math.finite3(t.position) or !math.finite3(t.direction) or
            @abs(math.norm(t.direction) - 1) >= 1e-4 or !positive(t.max_rpm) or !positive(t.diameter) or !positive(t.inertia) or
            !std.math.isFinite(t.thrust) or !std.math.isFinite(t.torque) or !nonnegative(t.kp) or !nonnegative(t.ki) or
            (t.command_sign != 1 and t.command_sign != -1)) return error.InvalidThruster;
    }
    const hull_count = database.ownedCount(robot_slot.db.hulls, robot_slot.slot);
    for (0..hull_count) |index| {
        const h = database.hullAt(robot_slot.db, robot_slot.slot, index);
        if (h.body >= self.objects.len or h.vertex_count == 0 or h.vertex_count % 3 != 0 or
            !math.finite3(h.drag) or !math.finite3(h.skin) or !math.finite3(h.centroid) or !nonnegative(h.volume))
            return error.InvalidHull;
        try math.validPose(database.poseOf(h.pos, h.quat));
        try math.validPose(.{ .quat = h.coefficient_frame });
        const positions = database.vertexSlice(robot_slot.db, h.vertex_start, h.vertex_count);
        for (positions) |position| if (!math.finite3(position)) return error.InvalidHull;
    }
    for (0..self.sensor_len) |index| {
        const sensor = database.sensorAt(robot_slot.db, robot_slot.slot, index);
        if (sensor.body >= self.objects.len or sensor.period_ticks == 0) return error.InvalidSensor;
        try math.validPose(database.poseOf(sensor.pos, sensor.quat));
        for (0..index) |other| if (sensor.id == database.sensorAt(robot_slot.db, robot_slot.slot, other).id) return error.DuplicateSensorId;
        if (database.sensorKind(sensor) == .camera) {
            if (sensor.width == 0 or sensor.height == 0 or sensor.width > 65536 or sensor.height > 65536 or
                !positive(sensor.horizontal_fov) or sensor.horizontal_fov >= 180 or
                !positive(sensor.near_range) or !positive(sensor.far_range) or sensor.far_range <= sensor.near_range)
                return error.InvalidCamera;
        }
        self.routing.set(database.sensorKind(sensor), @fromBackingInt(@intCast(index)));
    }
    var routed_body: ?types.BodyIndex = null;
    for (self.routing.values) |ref| if (ref) |sensor_index| {
        const body_index: types.BodyIndex = @enumFromInt(database.sensorAt(robot_slot.db, robot_slot.slot, @intCast(@backingInt(sensor_index))).body);
        if (routed_body) |previous| if (body_index != previous) return error.InvalidSensorRouting;
        routed_body = body_index;
    };
    for (self.faults) |fault| {
        if (fault.start >= fault.end) return error.InvalidFault;
        const valid = switch (fault.effect) {
            .dead => |channel| channel < self.thruster_len,
            .weak => |weak| weak.thruster < self.thruster_len and nonnegative(weak.scale) and weak.scale <= 1,
            .disable_sensor => |ref| @backingInt(ref) < self.sensor_len,
            .gyro_bias => |gyro| self.sensorIs(gyro.sensor, .imu) and std.math.isFinite(gyro.bias),
            .depth_bias => |depth| self.sensorIs(depth.sensor, .pressure) and std.math.isFinite(depth.bias),
        };
        if (!valid) return error.InvalidFault;
    }
}

fn sensorIs(self: *const Scene, ref: types.SensorIndex, kind: types.SensorKind) bool {
    const index = @backingInt(ref);
    const robot_slot = self.robot();
    return index < self.sensor_len and database.sensorKind(database.sensorAt(robot_slot.db, robot_slot.slot, index)) == kind;
}

fn validateCollider(db: *const database.Database, c: *const database.Collider) !void {
    const pose = database.poseOf(c.pos, c.quat);
    try math.validPose(pose);
    if (!math.finite3(c.size) or !math.finite3(c.drag) or !math.finite3(c.skin) or
        !nonnegative(c.friction) or !nonnegative(c.restitution) or c.restitution > 1)
        return error.InvalidCollider;
    const shape = database.shapeFrom(db, c);
    switch (shape) {
        .mesh => |mesh| {
            if (mesh.positions.len < 4 or mesh.positions.len > 1000000 or
                mesh.triangles.len == 0 or mesh.triangles.len > 2000000) return error.InvalidMesh;
            for (mesh.positions) |v| if (!math.finite3(v)) return error.InvalidMesh;
            for (mesh.triangles) |f| if (f.a >= mesh.positions.len or f.b >= mesh.positions.len or
                f.c >= mesh.positions.len) return error.InvalidMesh;
        },
        .box => if (c.size[0] <= 0 or c.size[1] <= 0 or c.size[2] <= 0) return error.InvalidCollider,
        .sphere => if (c.size[0] <= 0) return error.InvalidCollider,
        .cylinder => if (c.size[0] <= 0 or c.size[2] <= 0) return error.InvalidCollider,
    }
}

fn positive(x: f64) bool {
    return std.math.isFinite(x) and x > 0;
}

fn nonnegative(x: f64) bool {
    return std.math.isFinite(x) and x >= 0;
}