//! On-disk SIMDB layout: extern rows and integer tags (SIMDB001 v1).
pub const Vec3 = [3]f32;
pub const Quat = [4]f32;

pub const nil: u32 = @import("std").math.maxInt(u32);
pub const magic = "SIMDB001";
pub const version: u32 = 1;
pub const table_count: u32 = 9;
pub const directory_at: usize = magic.len + 8;
pub const entry_size: usize = 12;
pub const header_size: usize = directory_at + table_count * entry_size;

pub const table = struct {
    pub const strings: usize = 0;
    pub const entities: usize = 1;
    pub const colliders: usize = 2;
    pub const thrusters: usize = 3;
    pub const hulls: usize = 4;
    pub const sensors: usize = 5;
    pub const positions: usize = 6;
    pub const indices: usize = 7;
    pub const names: usize = 8;
};

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

pub const TableEntry = struct { offset: u32, count: u32, stride: u32 };

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

pub const strides = [table_count]u32{
    1,
    @sizeOf(Entity),
    @sizeOf(Collider),
    @sizeOf(Thruster),
    @sizeOf(Hull),
    @sizeOf(Sensor),
    @sizeOf(f32),
    @sizeOf(u32),
    @sizeOf(Name),
};

pub const aligns = [table_count]u32{
    1,
    @alignOf(Entity),
    @alignOf(Collider),
    @alignOf(Thruster),
    @alignOf(Hull),
    @alignOf(Sensor),
    @alignOf(f32),
    @alignOf(u32),
    @alignOf(Name),
};

/// Dynamics rows and session container (SIMSESS001 v1).
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

pub const Vec3Slot = extern struct {
    v: Vec3,
};

pub const SessionMeta = extern struct {
    tick: u64,
    thruster_len: u8,
    robot_name_len: u8,
    scene_name_len: u8,
    pad0: [5]u8,
    commands: [thruster_slots]f64,
};

pub const session_magic = "SIMSESS001";
pub const session_version: u32 = 1;
pub const session_chunk_count: u32 = 11;
pub const session_directory_at: usize = session_magic.len + 8;
pub const session_entry_size: usize = 12;

pub const session_chunk = struct {
    pub const strings: usize = 0;
    pub const meta: usize = 1;
    pub const robot_catalog: usize = 2;
    pub const scene_catalog: usize = 3;
    pub const body_states: usize = 4;
    pub const wrenches: usize = 5;
    pub const fluid_wrenches: usize = 6;
    pub const prev_linear: usize = 7;
    pub const prev_angular: usize = 8;
    pub const rotors: usize = 9;
    pub const contacts: usize = 10;
};

pub const session_strides = [session_chunk_count]u32{
    1,
    @sizeOf(SessionMeta),
    1,
    1,
    @sizeOf(BodyState),
    @sizeOf(Wrench),
    @sizeOf(Wrench),
    @sizeOf(Vec3Slot),
    @sizeOf(Vec3Slot),
    @sizeOf(RotorState),
    @sizeOf(Contact),
};