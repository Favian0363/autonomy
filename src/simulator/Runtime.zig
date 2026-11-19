//! Fixed-step physics, force evaluation, and sensor sampling.
const Runtime = @This();
const std = @import("std");
const types = @import("types.zig");
const S = @import("support.zig");
const Scene = @import("Scene.zig");
const database = @import("database.zig");
const MujocoBackend = @import("mujoco.zig").MujocoBackend;
const forces = @import("forces.zig");
const Sensors = @import("sensors.zig");

scene: *const Scene,
backend: MujocoBackend,
sensors: Sensors,
wrenches: []types.Wrench,
fluid: []types.Wrench,
previous_linear: []types.Vec3,
previous_angular: []types.Vec3,
scratch: []align(@alignOf(f64)) u8,
thruster_len: u8,
rotors: [types.thruster_count]types.RotorState = @splat(.{}),
commands: types.ThrusterCommands = @splat(0),
tick: u64 = 0,

pub fn init(scene: *const Scene) !Runtime {
    const allocator = scene.allocator;
    var backend = try MujocoBackend.init(allocator, scene);
    errdefer backend.deinit();
    var sensors = try Sensors.init(allocator, scene);
    errdefer sensors.deinit(allocator);
    const count = scene.objects.len;
    const wrenches = try allocator.alloc(types.Wrench, count);
    errdefer allocator.free(wrenches);
    const fluid = try allocator.alloc(types.Wrench, count);
    errdefer allocator.free(fluid);
    const linear = try allocator.alloc(types.Vec3, count);
    errdefer allocator.free(linear);
    const angular = try allocator.alloc(types.Vec3, count);
    errdefer allocator.free(angular);
    // Hydrodynamics uses two temporary f64 extents per body.
    const scratch = try allocator.alignedAlloc(u8, .of(f64), 2 * count * @sizeOf(f64));
    errdefer allocator.free(scratch);

    var self: Runtime = .{
        .scene = scene,
        .backend = backend,
        .sensors = sensors,
        .thruster_len = scene.thruster_len,
        .wrenches = wrenches,
        .fluid = fluid,
        .previous_linear = linear,
        .previous_angular = angular,
        .scratch = scratch,
    };
    try self.initializeTick();
    return self;
}

/// After a Session is moved, scene pointers stored in the runtime must be updated.
pub fn rebindScene(self: *Runtime, scene: *const Scene) void {
    self.scene = scene;
    self.thruster_len = scene.thruster_len;
    self.backend.source = scene;
}

pub fn deinit(self: *Runtime) void {
    const allocator = self.scene.allocator;
    self.backend.deinit();
    self.sensors.deinit(allocator);
    allocator.free(self.scratch);
    allocator.free(self.previous_angular);
    allocator.free(self.previous_linear);
    allocator.free(self.fluid);
    allocator.free(self.wrenches);
    self.* = undefined;
}

pub fn reset(self: *Runtime, states: ?[]const types.BodyState) !void {
    try self.backend.reset(states);
    try self.initializeTick();
}

fn initializeTick(self: *Runtime) !void {
    self.tick = 0;
    self.rotors = @splat(.{});
    self.commands = @splat(0);
    @memset(self.fluid, .{});
    @memset(self.wrenches, .{});
    try self.validateReadback();
    self.saveVelocities();
    self.sample();
}

pub fn step(self: *Runtime, commands: *const types.ThrusterCommands) !void {
    if (self.tick == std.math.maxInt(u64)) return error.TickOverflow;
    const n = self.thruster_len;
    const effective = self.commands[0..n];
    @memcpy(effective, commands[0..n]);
    for (self.scene.faults) |fault| {
        if (!fault.active(self.tick)) continue;
        switch (fault.effect) {
            .dead => |channel| effective[channel] *= 0,
            .weak => |weak| effective[weak.thruster] *= weak.scale,
            else => {},
        }
    }
    for (self.wrenches, self.scene.objects) |*wrench, slot| wrench.* = .{ .id = database.entity(slot).id };
    self.saveVelocities();
    for (self.scene.forces) |force| try forces.evaluate(force, self);

    try self.backend.step(self.wrenches);
    self.tick += 1;
    try self.validateReadback();
    const bounds = self.scene.settings.bounds;
    for (self.backend.states) |body| {
        if (S.norm(body.pose.pos) > bounds.position or S.norm(body.linear_velocity) > bounds.speed or
            S.norm(body.angular_velocity) > bounds.angular_speed) return error.SimulationBoundsExceeded;
    }
    self.sample();
}

fn validateReadback(self: *const Runtime) !void {
    for (self.backend.states, self.scene.objects) |body, object| {
        if (body.id != database.entity(object).id or !S.finite3(body.linear_velocity) or !S.finite3(body.angular_velocity) or
            !S.finite3(body.center_of_mass)) return error.InvalidBodyState;
        try S.validPose(body.pose);
    }
    for (self.backend.contacts.items) |contact| {
        if (!S.finite3(contact.position) or !S.finite3(contact.normal) or
            !std.math.isFinite(contact.normal_force)) return error.InvalidContact;
    }
}

fn saveVelocities(self: *Runtime) void {
    for (self.backend.states, self.previous_linear, self.previous_angular) |body, *linear, *angular| {
        linear.* = body.linear_velocity;
        angular.* = body.angular_velocity;
    }
}

fn sample(self: *Runtime) void {
    Sensors.sample(self);
}

/// The detection slice is valid until the next step, reset, or deinit.
pub fn observation(self: *const Runtime) types.Observation {
    return self.sensors.observation;
}

pub fn hash(self: *const Runtime) u64 {
    var h: S.Hash = .{};
    h.scalar(u64, self.tick);
    h.scalar(u64, self.backend.hash());
    for (self.backend.states) |body| h.state(body);
    for (self.rotors[0..self.scene.thruster_len], self.commands[0..self.scene.thruster_len]) |rotor, command| {
        h.scalar(f64, rotor.omega);
        h.scalar(f64, rotor.integral);
        h.scalar(f64, rotor.torque);
        h.scalar(f64, rotor.thrust);
        h.scalar(f64, command);
    }
    for (self.fluid) |wrench| {
        h.vec(wrench.force);
        h.vec(wrench.torque);
    }
    for (self.previous_linear) |value| h.vec(value);
    for (self.previous_angular) |value| h.vec(value);
    Sensors.hashSamples(self, &h);
    return h.value;
}
