//! Validated scene records. The runtime and world borrow this scene.
const Scene = @This();
const std = @import("std");
const types = @import("types.zig");
const math = @import("support.zig");

allocator: std.mem.Allocator,
settings: types.Settings,
objects: []const types.Object,
thrusters: []const types.Thruster,
hulls: []const types.Hull,
sensors: []const types.Sensor,
routing: types.Routing,
faults: []const types.Fault,
forces: []const types.Force = &.{ .gravity, .hydrodynamics, .thrusters },

/// Robot and additional collider definitions borrow immutable preset assets.
/// The scene owns its object and fault arrays.
pub fn init(
    allocator: std.mem.Allocator,
    robot: *const types.Robot,
    start: ?types.Pose,
    additional: []const types.Object,
    settings: types.Settings,
    faults: []const types.Fault,
) !Scene {
    if (additional.len >= 4096 or robot.thrusters.len > types.max_thrusters or
        robot.hulls.len > types.max_hulls or robot.sensors.len > 64) return error.SceneCapacity;

    const objects = try allocator.alloc(types.Object, additional.len + 1);
    errdefer allocator.free(objects);
    objects[0] = .{ .body = robot.body, .colliders = robot.colliders };
    if (start) |pose| objects[0].body.initial_pose = pose;
    @memcpy(objects[1..], additional);
    const owned_faults = try allocator.dupe(types.Fault, faults);
    errdefer allocator.free(owned_faults);
    var scene: Scene = .{
        .allocator = allocator,
        .settings = settings,
        .objects = objects,
        .thrusters = robot.thrusters,
        .hulls = robot.hulls,
        .sensors = robot.sensors,
        .routing = .initFill(null),
        .faults = owned_faults,
    };
    try scene.validate();
    return scene;
}

pub fn deinit(self: *Scene) void {
    self.allocator.free(self.faults);
    self.allocator.free(self.objects);
    self.* = undefined;
}

fn validate(self: *Scene) !void {
    const s = self.settings;
    if (s.dt_numerator == 0 or s.dt_denominator == 0 or s.hydro_period == 0 or
        !math.finite3(s.gravity) or !math.finite3(s.current) or !positive(s.water_density) or
        !positive(s.bounds.position) or !positive(s.bounds.speed) or !positive(s.bounds.angular_speed))
        return error.InvalidSettings;

    for (self.objects, 0..) |object, i| {
        const b = object.body;
        if (b.id == std.math.maxInt(u32) or b.name.len >= 64 or
            !math.finite3(b.initial_velocity) or !math.finite3(b.initial_angular_velocity) or
            !math.finite3(b.inertia) or !math.finite3(b.buoyancy_center) or
            !nonnegative(b.mass) or !nonnegative(b.dry_mass) or !nonnegative(b.volume))
            return error.InvalidBody;
        if (b.motion_type == .dynamic and
            !(positive(b.mass) and positive(b.inertia[0]) and positive(b.inertia[1]) and positive(b.inertia[2])))
            return error.InvalidBody;
        try math.validPose(b.initial_pose);
        try math.validPose(b.center_of_mass);
        for (self.objects[0..i]) |other| if (b.id == other.body.id) return error.DuplicateBodyId;
        if (object.colliders.len == 0 or object.colliders.len > 1024) return error.InvalidColliders;
        for (object.colliders) |collider| try validateCollider(collider);
    }
    for (self.thrusters) |thruster| {
        const spec = thruster.spec;
        if (@backingInt(thruster.body) >= self.objects.len or !math.finite3(thruster.position) or
            !math.finite3(thruster.direction) or @abs(math.norm(thruster.direction) - 1) >= 1e-4 or
            !positive(spec.max_rpm) or !positive(spec.diameter) or !positive(spec.inertia) or
            !std.math.isFinite(spec.thrust) or !std.math.isFinite(spec.torque) or
            !nonnegative(spec.kp) or !nonnegative(spec.ki) or
            (spec.command_sign != 1 and spec.command_sign != -1)) return error.InvalidThruster;
    }
    for (self.hulls) |hull| {
        if (@backingInt(hull.body) >= self.objects.len or hull.positions.len == 0 or
            hull.positions.len % 3 != 0 or !math.finite3(hull.drag) or !math.finite3(hull.skin) or
            !math.finite3(hull.centroid) or !nonnegative(hull.volume)) return error.InvalidHull;
        try math.validPose(hull.pose);
        try math.validPose(.{ .quat = hull.coefficient_frame });
        for (hull.positions) |position| if (!math.finite3(position)) return error.InvalidHull;
    }
    for (self.sensors, 0..) |sensor, index| {
        if (@backingInt(sensor.body) >= self.objects.len or sensor.period_ticks == 0) return error.InvalidSensor;
        try math.validPose(sensor.pose);
        for (self.sensors[0..index]) |other| if (sensor.id == other.id) return error.DuplicateSensorId;
        if (sensor.settings == .camera) {
            const camera = sensor.settings.camera;
            if (camera.width == 0 or camera.height == 0 or camera.width > 65536 or camera.height > 65536 or
                !positive(camera.horizontal_fov) or camera.horizontal_fov >= 180 or
                !positive(camera.near_range) or !positive(camera.far_range) or camera.far_range <= camera.near_range)
                return error.InvalidCamera;
        }
        // Match the preset rule of routing the last sensor of each kind.
        self.routing.set(sensor.kind(), @fromBackingInt(@intCast(index)));
    }
    var routed_body: ?types.BodyIndex = null;
    for (self.routing.values) |ref| if (ref) |index| {
        const body = self.sensors[@backingInt(index)].body;
        if (routed_body) |previous| if (body != previous) return error.InvalidSensorRouting;
        routed_body = body;
    };
    for (self.faults) |fault| {
        if (fault.start >= fault.end) return error.InvalidFault;
        const valid = switch (fault.effect) {
            .dead => |index| index < self.thrusters.len,
            .weak => |weak| weak.thruster < self.thrusters.len and nonnegative(weak.scale) and weak.scale <= 1,
            .disable_sensor => |index| @backingInt(index) < self.sensors.len,
            .gyro_bias => |gyro| self.sensorIs(gyro.sensor, .imu) and std.math.isFinite(gyro.bias),
            .depth_bias => |depth| self.sensorIs(depth.sensor, .pressure) and std.math.isFinite(depth.bias),
        };
        if (!valid) return error.InvalidFault;
    }
}

fn sensorIs(self: *const Scene, ref: types.SensorIndex, kind: types.SensorKind) bool {
    const index = @backingInt(ref);
    return index < self.sensors.len and self.sensors[index].kind() == kind;
}

fn validateCollider(c: types.Collider) !void {
    try math.validPose(c.pose);
    if (!math.finite3(c.size) or !math.finite3(c.drag) or !math.finite3(c.skin) or
        !nonnegative(c.friction) or !nonnegative(c.restitution) or c.restitution > 1)
        return error.InvalidCollider;
    switch (c.shape) {
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
