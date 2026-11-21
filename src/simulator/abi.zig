
pub const Vec3 = [3]f32;
pub const Quat = [4]f32;

pub const nil: u32 = @import("std").math.maxInt(u32);

pub const motion_tag = enum(u32) {
    static = 0,
    dynamic = 1,
    kinematic = 2,
};

pub const shape_tag = enum(u32) {
    box = 0,
    sphere = 1,
    cylinder = 2,
    mesh = 3,
};

pub const sensor_kind_tag = enum(u32) {
    pose = 0,
    imu = 1,
    pressure = 2,
    camera = 3,
};

pub const Entity = extern struct {
    name_offset: u32,
    name_len: u32,
    asset_offset: u32,
    asset_len: u32,
    asset_hash: [32]u8,
    id: u32,
    cls: u32,
    motion: u32,
    tracked: u8,
    fuzz: u8,
    pad0: u16,
    mass: f64,
    inertia: [3]f32,
    pad1: u32,
    com_pos: [3]f32,
    com_quat: [4]f32,
    initial_pos: [3]f32,
    initial_quat: [4]f32,
    initial_velocity: [3]f32,
    initial_angular_velocity: [3]f32,
    dry_mass: f64,
    volume: f64,
    buoyancy: [3]f32,
    pad2: u32,
};

pub const Collider = extern struct {
    owner: u32,
    shape: u32,
    collisions: u8,
    pad0: [3]u8,
    pos: [3]f32,
    quat: [4]f32,
    size: [3]f32,
    drag: [3]f32,
    skin: [3]f32,
    pad1: u32,
    friction: f64,
    restitution: f64,
    vertex_start: u32,
    vertex_count: u32,
    triangle_start: u32,
    triangle_count: u32,
};

pub const Thruster = extern struct {
    owner: u32,
    body: u32,
    position: Vec3,
    direction: Vec3,
    max_rpm: f64,
    diameter: f64,
    thrust: f64,
    torque: f64,
    inertia: f64,
    kp: f64,
    ki: f64,
    command_sign: f64,
};

pub const Hull = extern struct {
    owner: u32,
    body: u32,
    pos: [3]f32,
    quat: [4]f32,
    drag: [3]f32,
    skin: [3]f32,
    centroid: [3]f32,
    coefficient_frame: [4]f32,
    volume: f64,
    vertex_start: u32,
    vertex_count: u32,
};

pub const Sensor = extern struct {
    owner: u32,
    body: u32,
    id: u32,
    enabled: u8,
    pad0: [3]u8,
    period_ticks: u32,
    pos: [3]f32,
    quat: [4]f32,
    kind: u32,
    position_axes: u8,
    attitude_axes: u8,
    pad1: u16,
    width: u32,
    height: u32,
    horizontal_fov: f64,
    near_range: f64,
    far_range: f64,
};

pub const Name = extern struct {
    name_offset: u32,
    name_len: u32,
    entity_start: u32,
    entity_count: u32,
};

/// Robot document. Thrusters, hulls, and sensors belong to this catalog.
pub const RobotCatalog = struct {
    strings: []u8 = &.{},
    entities: []Entity = &.{},
    colliders: []Collider = &.{},
    thrusters: []Thruster = &.{},
    hulls: []Hull = &.{},
    sensors: []Sensor = &.{},
    positions: []f32 = &.{},
    indices: []u32 = &.{},
    names: []Name = &.{},
};

pub const SceneCatalog = struct {
    strings: []u8 = &.{},
    entities: []Entity = &.{},
    colliders: []Collider = &.{},
    positions: []f32 = &.{},
    indices: []u32 = &.{},
    names: []Name = &.{},
};

pub const thruster_slots: usize = 8;
pub const max_snapshot_contacts: usize = 1024;

pub const BodyState = extern struct {
    id: u32,
    pose_pos: Vec3,
    pose_quat: Quat,
    com: Vec3,
    linear_vel: Vec3,
    angular_vel: Vec3,
};

pub const Wrench = extern struct {
    id: u32,
    force: Vec3,
    torque: Vec3,
};

pub const RotorState = extern struct {
    omega: f64,
    integral: f64,
    torque: f64,
    thrust: f64,
};

pub const Contact = extern struct {
    a: u32,
    b: u32,
    position: Vec3,
    normal: Vec3,
    pad: u32,
    normal_force: f64,
};

pub const SessionMeta = extern struct {
    tick: u64,
    thruster_len: u8,
    robot_name_len: u8,
    scene_name_len: u8,
    pad0: [5]u8,
    commands: [thruster_slots]f64,
};

pub const SessionStateCatalog = struct {
    strings: []u8 = &.{},
    meta: SessionMeta = std_zero_meta,
    robot_catalog: []u8 = &.{},
    scene_catalog: []u8 = &.{},
    body_states: []BodyState = &.{},
    wrenches: []Wrench = &.{},
    fluid: []Wrench = &.{},
    previous_linear: []Vec3 = &.{},
    previous_angular: []Vec3 = &.{},
    rotors: []RotorState = &.{},
    contacts: []Contact = &.{},
};

const std_zero_meta = SessionMeta{
    .tick = 0,
    .thruster_len = 0,
    .robot_name_len = 0,
    .scene_name_len = 0,
    .pad0 = .{ 0, 0, 0, 0, 0 },
    .commands = @splat(0),
};
