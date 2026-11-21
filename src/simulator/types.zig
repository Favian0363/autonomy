
const abi = @import("abi.zig");

pub const Vec3 = [3]f64;
pub const Quat = [4]f64;
pub const Pose = struct {
    pos: Vec3 = @splat(0),
    quat: Quat = .{ 0, 0, 0, 1 },
};

pub const BodyIndex = enum(usize) { robot = 0, _ };
pub const SensorIndex = enum(usize) { _ };
pub const max_hulls = 5;

pub const thruster_count = 8;
pub const ThrusterCommands = [thruster_count]f64;
pub const ThrusterCommandInput = [thruster_count]f32;

pub const Triangle = struct { a: u32, b: u32, c: u32 };
pub const Mesh = struct {
    positions: []const abi.Vec3,
    triangles: []const Triangle,
};
pub const Shape = union(enum) { box, sphere, cylinder, mesh: Mesh };
pub const ColliderInput = struct {
    shape: Shape = .box,
    collisions: bool = true,
    pose: Pose = .{},
    size: Vec3 = @splat(1),
    drag: Vec3 = @splat(0),
    skin: Vec3 = @splat(0),
    friction: f64 = 0.5,
    restitution: f64 = 0,
};

pub const Motion = enum { static, dynamic, kinematic };

pub const SensorKind = enum(u32) { pose, imu, pressure, camera };
pub const AxisMask = u3;
pub const Tracking = struct { position_axes: AxisMask, attitude_axes: AxisMask };
pub const Camera = struct {
    width: u32,
    height: u32,
    horizontal_fov: f64,
    near_range: f64,
    far_range: f64,
};
pub const SensorInput = struct {
    id: u32,
    body: BodyIndex = .robot,
    pose: Pose,
    enabled: bool,
    period_ticks: u32,
    settings: union(SensorKind) { pose: Tracking, imu, pressure, camera: Camera },

    pub fn kind(self: SensorInput) SensorKind {
        return self.settings;
    }
};
pub const Routing = std.EnumArray(SensorKind, ?SensorIndex);

pub const Fault = struct {
    start: u64,
    end: u64,
    effect: union(enum) {
        dead: usize,
        weak: struct { thruster: usize, scale: f64 },
        disable_sensor: SensorIndex,
        gyro_bias: struct { sensor: SensorIndex, bias: f64 },
        depth_bias: struct { sensor: SensorIndex, bias: f64 },
    },

    pub fn active(self: Fault, tick: u64) bool {
        return tick >= self.start and tick < self.end;
    }
};

pub const Integrator = enum { default, euler, implicit, implicit_fast, rk4 };
pub const Bounds = struct {
    position: f64 = 10000,
    speed: f64 = 1000,
    angular_speed: f64 = 1000,
};
pub const Settings = struct {
    dt_numerator: u32 = 1,
    dt_denominator: u32 = 360,
    integrator: Integrator = .default,
    hydro_period: u32 = 7,
    gravity: Vec3 = .{ 0, 0, 9.81 },
    current: Vec3 = @splat(0),
    water_density: f64 = 1025,
    bounds: Bounds = .{},

    pub fn timestep(self: Settings) f64 {
        return @as(f64, @floatFromInt(self.dt_numerator)) / @as(f64, @floatFromInt(self.dt_denominator));
    }
};
pub const Force = enum { gravity, hydrodynamics, thrusters };

pub const Detection = struct {
    id: u32,
    cls: u32,
    pose: Pose,
    size: Vec3,
    x0: u32 = 0,
    y0: u32 = 0,
    x1: u32 = 0,
    y1: u32 = 0,
};
pub const Imu = struct { orientation: Quat, gyro: Vec3, accel: Vec3 };
pub const SensorSample = struct {
    tick: u64 = 0,
    value: ?union(SensorKind) {
        pose: struct { pose: Pose, axes: Tracking },
        imu: Imu,
        pressure: f64,
        camera: struct { pose: Pose, objects: []const Detection },
    } = null,
};

pub const Observation = struct {
    body: Pose = .{},
    position_axes: AxisMask = 0,
    attitude_axes: AxisMask = 0,
    image_width: u32 = 0,
    image_height: u32 = 0,
    camera: Pose = .{},
    imu: Quat = .{ 0, 0, 0, 1 },
    gyro: Vec3 = @splat(0),
    accel: Vec3 = @splat(0),
    depth: f64 = 0,
    depth_ok: bool = false,
    objects: []const Detection = &.{},
};

const std = @import("std");
