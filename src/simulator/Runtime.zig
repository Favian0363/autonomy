//! Fixed-step physics, force evaluation, and sensor sampling.
const Runtime = @This();
const std = @import("std");
const types = @import("types.zig");
const S = @import("support.zig");
const Scene = @import("Scene.zig");
const World = @import("mujoco.zig").World;
const forces = @import("forces.zig");
const Sensors = @import("sensors.zig");

scene: *const Scene,
world: World,
sensors: Sensors,
wrenches: []types.Wrench,
fluid: []types.Wrench,
previous_linear: []types.Vec3,
previous_angular: []types.Vec3,
scratch: []align(@alignOf(f64)) u8,
rotors: [types.max_thrusters]types.RotorState = @splat(.{}),
commands: [types.max_thrusters]f64 = @splat(0),
tick: u64 = 0,

pub fn init(scene: *const Scene) !Runtime {
    const allocator = scene.allocator;
    var world = try World.init(allocator, scene);
    errdefer world.deinit();
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
        .world = world,
        .sensors = sensors,
        .wrenches = wrenches,
        .fluid = fluid,
        .previous_linear = linear,
        .previous_angular = angular,
        .scratch = scratch,
    };
    try self.initializeTick();
    return self;
}

pub fn deinit(self: *Runtime) void {
    const allocator = self.scene.allocator;
    self.world.deinit();
    self.sensors.deinit(allocator);
    allocator.free(self.scratch);
    allocator.free(self.previous_angular);
    allocator.free(self.previous_linear);
    allocator.free(self.fluid);
    allocator.free(self.wrenches);
    self.* = undefined;
}

pub fn reset(self: *Runtime, states: ?[]const types.BodyState) !void {
    try self.world.reset(states);
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

pub fn step(self: *Runtime, commands: []const f64) !void {
    if (commands.len != self.scene.thrusters.len) return error.InvalidThrusterCount;
    if (self.tick == std.math.maxInt(u64)) return error.TickOverflow;
    for (commands) |command| {
        if (!std.math.isFinite(command) or command < -1 or command > 1) return error.InvalidThrust;
    }
    const effective = self.commands[0..commands.len];
    @memcpy(effective, commands);
    for (self.scene.faults) |fault| {
        if (!fault.active(self.tick)) continue;
        switch (fault.effect) {
            .dead => |channel| effective[channel] *= 0,
            .weak => |weak| effective[weak.thruster] *= weak.scale,
            else => {},
        }
    }
    for (self.wrenches, self.scene.objects) |*wrench, object| wrench.* = .{ .id = object.body.id };
    self.saveVelocities();

    var scratch = std.heap.FixedBufferAllocator.init(self.scratch);
    const context: forces.Context = .{
        .scene = self.scene,
        .bodies = self.world.states,
        .commands = effective,
        .tick = self.tick,
        .dt = self.scene.settings.timestep(),
        .rotors = self.rotors[0..commands.len],
        .fluid_cache = self.fluid,
        .scratch = scratch.allocator(),
    };
    for (self.scene.forces) |force| try forces.evaluate(force, &context, self.wrenches);

    try self.world.step(self.wrenches);
    self.tick += 1;
    try self.validateReadback();
    const bounds = self.scene.settings.bounds;
    for (self.world.states) |body| {
        if (S.norm(body.pose.pos) > bounds.position or S.norm(body.linear_velocity) > bounds.speed or
            S.norm(body.angular_velocity) > bounds.angular_speed) return error.SimulationBoundsExceeded;
    }
    self.sample();
}

fn validateReadback(self: *const Runtime) !void {
    for (self.world.states, self.scene.objects) |body, object| {
        if (body.id != object.body.id or !S.finite3(body.linear_velocity) or !S.finite3(body.angular_velocity) or
            !S.finite3(body.center_of_mass)) return error.InvalidBodyState;
        try S.validPose(body.pose);
    }
    for (self.world.contacts.items) |contact| {
        if (!S.finite3(contact.position) or !S.finite3(contact.normal) or
            !std.math.isFinite(contact.normal_force)) return error.InvalidContact;
    }
}

fn saveVelocities(self: *Runtime) void {
    for (self.world.states, self.previous_linear, self.previous_angular) |body, *linear, *angular| {
        linear.* = body.linear_velocity;
        angular.* = body.angular_velocity;
    }
}

fn sample(self: *Runtime) void {
    self.sensors.sample(self.scene, self.world.states, self.previous_linear, self.previous_angular, self.tick);
}

/// The detection slice is valid until the next step, reset, or deinit.
pub fn observation(self: *const Runtime) types.Observation {
    return self.sensors.observation;
}

pub fn hash(self: *const Runtime) u64 {
    var h: S.Hash = .{};
    h.scalar(u64, self.tick);
    h.scalar(u64, self.world.hash());
    for (self.world.states) |body| h.state(body);
    for (self.rotors[0..self.scene.thrusters.len], self.commands[0..self.scene.thrusters.len]) |rotor, command| {
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
    self.sensors.hashSamples(self.scene, &h);
    return h.value;
}
