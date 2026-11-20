//! In-process simulation session: scene ownership, physics, and mission frames.
const std = @import("std");
const Auv = @import("../Auv.zig");
const math = @import("../math.zig");
const Scene = @import("Scene.zig");
const database = @import("database.zig");
const types = @import("types.zig");
const S = @import("support.zig");
const MissionConfig = @import("../MissionConfig.zig");
const MujocoBackend = @import("mujoco.zig").MujocoBackend;
const forces = @import("forces.zig");
const Sensors = @import("sensors.zig");
const abi = @import("abi.zig");

pub const Session = @This();

const frame_ticks = 8;

io: std.Io,
catalogs_owned: bool,
robot_db: database.Database,
robot_slots: []u32,
scene_db: database.Database,
scene_presets: []database.ScenePreset,
scene: Scene,
backend: MujocoBackend,
sensors: Sensors,
wrenches: []abi.Wrench,
fluid: []abi.Wrench,
previous_linear: []abi.Vec3,
previous_angular: []abi.Vec3,
scratch: []align(@alignOf(f64)) u8,
thruster_len: u8,
rotors: [types.thruster_count]abi.RotorState = @splat(abi.RotorState{ .omega = 0, .integral = 0, .torque = 0, .thrust = 0 }),
preset_robot: []const u8 = "",
preset_scene: []const u8 = "",
commands: types.ThrusterCommands = @splat(0),
tick: u64 = 0,
realtime: bool,
tam_len: usize,
failed: bool = false,
live: bool = false,
started: std.Io.Clock.Timestamp,

pub fn init(self: *Session, allocator: std.mem.Allocator, io: std.Io, config: *const MissionConfig.MissionConfig, tam_len: usize, realtime: bool) !void {
    const robot_name = try config.sim_robot.slice();
    const scene_name = try config.sim_scene.slice();
    if (tam_len == 0 or tam_len > MissionConfig.thruster_count) return error.InvalidThrusterCount;
    self.* = .{
        .io = io,
        .catalogs_owned = true,
        .robot_db = undefined,
        .robot_slots = undefined,
        .scene_db = undefined,
        .scene_presets = undefined,
        .scene = undefined,
        .backend = undefined,
        .sensors = undefined,
        .wrenches = undefined,
        .fluid = undefined,
        .previous_linear = undefined,
        .previous_angular = undefined,
        .scratch = undefined,
        .thruster_len = 0,
        .realtime = realtime,
        .tam_len = tam_len,
        .started = undefined,
    };
    self.robot_db = try database.Database.load(allocator, @embedFile("robots.bin"));
    self.robot_slots = try allocator.alloc(u32, self.robot_db.names.len);
    for (self.robot_db.names, 0..) |name, i| {
        if (name.entity_count != 1) return error.InvalidDatabase;
        self.robot_slots[i] = name.entity_start;
    }
    switch (config.sim_scene_catalog) {
        .embedded_bin => {
            self.scene_db = try database.Database.load(allocator, @embedFile("scenes.bin"));
        },
        .file => {
            const path = try config.sim_scene_catalog_path.slice();
            if (path.len == 0) return error.MissingSceneCatalogPath;
            const bytes = try std.Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(std.math.maxInt(usize)));
            self.scene_db = try database.Database.load(allocator, bytes);
        },
    }
    self.scene_presets = try database.allocScenePresets(allocator, &self.scene_db);
    const robot_index: usize = blk: {
        for (self.robot_slots, 0..) |slot, i| {
            if (std.mem.eql(u8, robot_name, database.entityName(&self.robot_db, slot))) break :blk i;
        }
        return error.UnknownSimRobot;
    };
    const robot_slot = self.robot_slots[robot_index];
    const robot = database.BodySlot{ .db = &self.robot_db, .slot = robot_slot };
    if (database.ownedCount(robot.db.thrusters, robot.slot) != tam_len) return error.InvalidThrusterCount;
    const selected_scene = if (std.mem.eql(u8, scene_name, "robot_only"))
        try Scene.init(allocator, robot, null, &.{}, .{}, &.{})
    else blk: {
        const preset: *const database.ScenePreset = find_preset: {
            for (self.scene_presets) |*p| if (std.mem.eql(u8, scene_name, p.name)) break :find_preset p;
            return error.UnknownSimScene;
        };
        const start: ?types.Pose = if (std.mem.eql(u8, scene_name, "robosub_course"))
            .{ .pos = .{ 0, 2, 2 }, .quat = .{ 0, 0, 1, 0 } }
        else
            null;
        try database.validateEntitySlots(&self.scene_db, preset.slots);
        const extras = try allocator.alloc(database.BodySlot, preset.slots.len);
        for (preset.slots, extras) |slot, *body| body.* = .{ .db = &self.scene_db, .slot = slot };
        break :blk try Scene.init(allocator, robot, start, extras, .{}, &.{});
    };
    self.scene = selected_scene;
    if (tam_len != self.scene.thruster_len) return error.InvalidThrusterCount;
    self.preset_robot = try allocator.dupe(u8, robot_name);
    self.preset_scene = try allocator.dupe(u8, scene_name);
    const scene = &self.scene;
    const physics_allocator = scene.allocator;
    self.backend = try MujocoBackend.init(physics_allocator, scene);
    errdefer self.backend.deinit();
    self.sensors = try Sensors.init(physics_allocator, scene);
    errdefer {
        self.sensors.deinit(physics_allocator);
        self.backend.deinit();
    }
    self.thruster_len = scene.thruster_len;
    const count = scene.objects.len;
    self.wrenches = try physics_allocator.alloc(abi.Wrench, count);
    self.fluid = try physics_allocator.alloc(abi.Wrench, count);
    self.previous_linear = try physics_allocator.alloc(abi.Vec3, count);
    self.previous_angular = try physics_allocator.alloc(abi.Vec3, count);
    // Hydrodynamics uses two temporary f64 extents per body.
    self.scratch = try physics_allocator.alignedAlloc(u8, .of(f64), 2 * count * @sizeOf(f64));
    try self.initializeTick();
    self.started = .now(self.io, .awake);
    self.live = true;
}

/// Releases MuJoCo handles only; Zig allocations live until the mission arena resets.
pub fn deinit(self: *Session) void {
    if (!self.live) {
        self.* = undefined;
        return;
    }
    const allocator = self.scene.allocator;
    self.sensors.deinit(allocator);
    self.backend.deinit();
    self.live = false;
    self.* = undefined;
}

pub fn setThrusters(self: *Session, values: *const types.ThrusterCommandInput) !void {
    for (values[0..self.tam_len]) |v| if (!std.math.isFinite(v) or v < -1 or v > 1) return error.InvalidThrust;
    for (values[0..self.tam_len], 0..) |v, i| self.commands[i] = v;
}

pub fn timestamp(self: *const Session) u64 {
    return @intCast(@as(u128, self.tick) * self.scene.settings.dt_numerator * 1_000_000_000 / self.scene.settings.dt_denominator);
}

pub fn reset(self: *Session, states: ?[]const abi.BodyState) !void {
    try self.backend.reset(states);
    try self.initializeTick();
}

fn framePose(p: types.Pose) math.Pose {
    return .{ .pos = p.pos, .quat = p.quat };
}

fn initializeTick(self: *Session) !void {
    self.tick = 0;
    self.rotors = @splat(abi.RotorState{ .omega = 0, .integral = 0, .torque = 0, .thrust = 0 });
    self.commands = @splat(0);
    const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
    @memset(self.fluid, zero_wrench);
    @memset(self.wrenches, zero_wrench);
    try self.validateReadback();
    self.saveVelocities();
    self.refreshSensors();
}

fn validateReadback(self: *const Session) !void {
    for (self.backend.states, self.scene.objects) |body, object| {
        if (body.id != database.entity(object).id or !S.finite3(S.fromAbi3(body.linear_vel)) or !S.finite3(S.fromAbi3(body.angular_vel)) or
            !S.finite3(S.fromAbi3(body.com))) return error.InvalidBodyState;
        try S.validPose(S.bodyPose(body));
    }
    for (self.backend.contacts.items) |contact| {
        if (!S.finite3(S.fromAbi3(contact.position)) or !S.finite3(S.fromAbi3(contact.normal)) or
            !std.math.isFinite(contact.normal_force)) return error.InvalidContact;
    }
}

fn saveVelocities(self: *Session) void {
    for (self.backend.states, self.previous_linear, self.previous_angular) |body, *linear, *angular| {
        linear.* = body.linear_vel;
        angular.* = body.angular_vel;
    }
}

pub fn refreshSensors(self: *Session) void {
    Sensors.sample(self);
}

/// The detection slice is valid until the next step, reset, or deinit.
pub fn observation(self: *const Session) types.Observation {
    return self.sensors.observation;
}

pub fn bodyStates(self: *const Session) []const abi.BodyState {
    return self.backend.states;
}

pub fn nextFrame(self: *Session, out: *Auv.Frame) !void {
    errdefer self.failed = true;
    for (0..frame_ticks) |_| {
        if (self.failed) return error.SimulationFailed;
        if (self.tick == std.math.maxInt(u64)) return error.TickOverflow;
        const effective = self.commands[0..self.thruster_len];
        for (self.scene.faults) |fault| {
            if (!fault.active(self.tick)) continue;
            switch (fault.effect) {
                .dead => |channel| effective[channel] *= 0,
                .weak => |weak| effective[weak.thruster] *= weak.scale,
                else => {},
            }
        }
        const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
        for (self.wrenches, self.scene.objects) |*wrench, slot| {
            wrench.* = zero_wrench;
            wrench.id = database.entity(slot).id;
        }
        self.saveVelocities();
        for (self.scene.forces) |force| forces.evaluate(force, self) catch |err| {
            std.log.warn("simulator step failed: {s}", .{@errorName(err)});
            self.failed = true;
            return error.SimulationFailed;
        };
        self.backend.step(self.wrenches) catch |err| {
            std.log.warn("simulator step failed: {s}", .{@errorName(err)});
            self.failed = true;
            return error.SimulationFailed;
        };
        self.tick += 1;
        self.validateReadback() catch |err| {
            std.log.warn("simulator step failed: {s}", .{@errorName(err)});
            self.failed = true;
            return error.SimulationFailed;
        };
        const bounds = self.scene.settings.bounds;
        for (self.backend.states) |body| {
            if (S.norm(S.fromAbi3(body.pose_pos)) > bounds.position or
                S.norm(S.fromAbi3(body.linear_vel)) > bounds.speed or
                S.norm(S.fromAbi3(body.angular_vel)) > bounds.angular_speed)
            {
                std.log.warn("simulator step failed: {s}", .{@errorName(error.SimulationBoundsExceeded)});
                self.failed = true;
                return error.SimulationFailed;
            }
        }
        self.refreshSensors();
    }
    if (self.realtime) {
        const elapsed = self.started.untilNow(self.io).raw.toNanoseconds();
        const delay = @as(i96, @intCast(self.timestamp())) - elapsed;
        if (delay > 0) try std.Io.sleep(self.io, .fromNanoseconds(delay), .awake);
    }
    const obs = self.observation();
    out.* = std.mem.zeroes(Auv.Frame);
    out.timestamp = self.timestamp();
    out.camera_pose = framePose(obs.body);
    out.position_ok = .{ .x = obs.position_axes & 1 != 0, .y = obs.position_axes & 2 != 0, .z = obs.position_axes & 4 != 0 };
    out.attitude_ok = .{ .x = obs.attitude_axes & 1 != 0, .y = obs.attitude_axes & 2 != 0, .z = obs.attitude_axes & 4 != 0 };
    out.imu_quat = obs.imu;
    out.accel = obs.accel;
    out.gyro = obs.gyro;
    out.pressure_depth = @floatCast(obs.depth);
    out.pressure_depth_ok = obs.depth_ok;
    out.image_width = obs.image_width;
    out.image_height = obs.image_height;
    for (obs.objects[0..@min(obs.objects.len, Auv.Frame.max_objects)], 0..) |object, i| {
        if (object.cls > 255) return error.InvalidObjectClass;
        out.objects[i] = .{ .bbox = .{ .pose = framePose(object.pose), .size = object.size }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects2d[i] = .{ .top_left = .{ .x = object.x0, .y = object.y0 }, .bottom_right = .{ .x = object.x1, .y = object.y1 }, .id = object.id, .cls = @intCast(object.cls) };
        out.objects_len += 1;
        out.objects2d_len += 1;
    }
}

pub fn snapshot(self: *const Session, allocator: std.mem.Allocator) ![]u8 {
    if (self.backend.contacts.items.len > abi.max_snapshot_contacts) return error.TooManyContacts;
    if (self.preset_robot.len > std.math.maxInt(u8) or self.preset_scene.len > std.math.maxInt(u8)) return error.NameTooLong;

    const robot_catalog = if (self.catalogs_owned)
        try self.robot_db.save(allocator)
    else
        &[_]u8{};
    const scene_catalog = if (self.catalogs_owned)
        try self.scene_db.save(allocator)
    else
        &[_]u8{};

    const meta: abi.SessionMeta = .{
        .tick = self.tick,
        .thruster_len = self.thruster_len,
        .robot_name_len = @intCast(self.preset_robot.len),
        .scene_name_len = @intCast(self.preset_scene.len),
        .pad0 = .{0, 0, 0, 0, 0},
        .commands = self.commands,
    };

    const prev_linear_slots = try allocator.alloc(abi.Vec3Slot, self.previous_linear.len);
    const prev_angular_slots = try allocator.alloc(abi.Vec3Slot, self.previous_angular.len);
    for (self.previous_linear, prev_linear_slots) |v, *s| s.v = v;
    for (self.previous_angular, prev_angular_slots) |v, *s| s.v = v;

    const strings_len: u32 = @intCast(self.preset_robot.len + self.preset_scene.len);
    const counts = [_]u32{
        strings_len,
        1,
        @intCast(robot_catalog.len),
        @intCast(scene_catalog.len),
        @intCast(self.backend.states.len),
        @intCast(self.wrenches.len),
        @intCast(self.fluid.len),
        @intCast(self.previous_linear.len),
        @intCast(self.previous_angular.len),
        types.thruster_count,
        @intCast(self.backend.contacts.items.len),
    };
    const entries = try placeSnapshotChunks(&counts);
    const end = snapshotPayloadEnd(&entries);
    const out = try allocator.alignedAlloc(u8, .of(abi.SessionMeta), end);
    @memset(out, 0);
    @memcpy(out[0..abi.session_magic.len], abi.session_magic);
    std.mem.writeInt(u32, out[abi.session_magic.len..][0..4], abi.session_version, .little);
    std.mem.writeInt(u32, out[abi.session_magic.len + 4 ..][0..4], abi.session_chunk_count, .little);

    const strings_buf = try allocator.alloc(u8, self.preset_robot.len + self.preset_scene.len);
    @memcpy(strings_buf[0..self.preset_robot.len], self.preset_robot);
    @memcpy(strings_buf[self.preset_robot.len..], self.preset_scene);

    const columns = [_][]const u8{
        strings_buf,
        std.mem.asBytes(&meta),
        robot_catalog,
        scene_catalog,
        std.mem.sliceAsBytes(self.backend.states),
        std.mem.sliceAsBytes(self.wrenches),
        std.mem.sliceAsBytes(self.fluid),
        std.mem.sliceAsBytes(prev_linear_slots),
        std.mem.sliceAsBytes(prev_angular_slots),
        std.mem.sliceAsBytes(self.rotors[0..]),
        std.mem.sliceAsBytes(self.backend.contacts.items),
    };

    for (entries, columns, 0..) |entry, data, i| {
        const at = abi.session_directory_at + i * abi.session_entry_size;
        std.mem.writeInt(u32, out[at..][0..4], entry.offset, .little);
        std.mem.writeInt(u32, out[at + 4 ..][0..4], entry.count, .little);
        std.mem.writeInt(u32, out[at + 8 ..][0..4], entry.stride, .little);
        if (data.len != 0) @memcpy(out[entry.offset..][0..data.len], data);
    }
    return out;
}

pub fn restoreSnapshot(self: *Session, _: std.mem.Allocator, bytes: []const u8) !void {
    const entries = try readSnapshotHeader(bytes);
    const meta: *const abi.SessionMeta = @ptrCast(@alignCast(
        bytes[entries[abi.session_chunk.meta].offset..].ptr,
    ));
    const strings_entry = entries[abi.session_chunk.strings];
    if (strings_entry.count != meta.robot_name_len + meta.scene_name_len) return error.InvalidSession;
    const strings = bytes[strings_entry.offset..][0..strings_entry.count];
    if (!std.mem.eql(u8, strings[0..meta.robot_name_len], self.preset_robot) or
        !std.mem.eql(u8, strings[meta.robot_name_len..][0..meta.scene_name_len], self.preset_scene))
        return error.SnapshotMismatch;

    const body_states = try snapshotChunkRecords(abi.BodyState, bytes, entries[abi.session_chunk.body_states]);
    const wrenches = try snapshotChunkRecords(abi.Wrench, bytes, entries[abi.session_chunk.wrenches]);
    const fluid = try snapshotChunkRecords(abi.Wrench, bytes, entries[abi.session_chunk.fluid_wrenches]);
    const prev_linear = try snapshotChunkRecords(abi.Vec3Slot, bytes, entries[abi.session_chunk.prev_linear]);
    const prev_angular = try snapshotChunkRecords(abi.Vec3Slot, bytes, entries[abi.session_chunk.prev_angular]);
    const rotors = try snapshotChunkRecords(abi.RotorState, bytes, entries[abi.session_chunk.rotors]);
    const contacts = try snapshotChunkRecords(abi.Contact, bytes, entries[abi.session_chunk.contacts]);

    if (body_states.len != self.backend.states.len or
        wrenches.len != self.wrenches.len or
        fluid.len != self.fluid.len or
        prev_linear.len != self.previous_linear.len or
        prev_angular.len != self.previous_angular.len)
        return error.SnapshotMismatch;

    try self.backend.reset(body_states);
    @memcpy(self.wrenches, wrenches);
    @memcpy(self.fluid, fluid);
    for (prev_linear, self.previous_linear) |r, *v| v.* = r.v;
    for (prev_angular, self.previous_angular) |r, *v| v.* = r.v;
    const rotor_count = @min(rotors.len, self.rotors.len);
    @memcpy(self.rotors[0..rotor_count], rotors[0..rotor_count]);
    self.tick = meta.tick;
    self.commands = meta.commands;
    self.thruster_len = meta.thruster_len;
    self.backend.contacts.clearRetainingCapacity();
    try self.backend.contacts.appendSlice(self.scene.allocator, contacts);
    self.refreshSensors();
}

fn snapshotChunkRecords(comptime R: type, bytes: []const u8, entry: abi.TableEntry) ![]const R {
    if (entry.stride != @sizeOf(R)) return error.InvalidSession;
    if (entry.count == 0) return &[_]R{};
    const size = entry.count * entry.stride;
    if (entry.offset + size > bytes.len) return error.InvalidSession;
    const ptr: [*]const R = @ptrCast(@alignCast(bytes.ptr + entry.offset));
    return ptr[0..entry.count];
}

fn snapshotPayloadEnd(entries: []const abi.TableEntry) usize {
    var end: usize = abi.session_directory_at + abi.session_chunk_count * abi.session_entry_size;
    for (entries) |entry| {
        const chunk_end = entry.offset + entry.count * entry.stride;
        if (chunk_end > end) end = chunk_end;
    }
    return end;
}

fn placeSnapshotChunks(counts: []const u32) ![abi.session_chunk_count]abi.TableEntry {
    var entries: [abi.session_chunk_count]abi.TableEntry = undefined;
    var offset: u32 = abi.session_directory_at + @as(u32, @intCast(abi.session_chunk_count * abi.session_entry_size));
    for (counts, abi.session_strides[0..], &entries) |count, stride, *entry| {
        entry.* = .{ .offset = offset, .count = count, .stride = stride };
        const bytes = count * stride;
        offset = std.mem.alignForward(u32, offset + bytes, 8);
    }
    return entries;
}

fn readSnapshotHeader(bytes: []const u8) ![abi.session_chunk_count]abi.TableEntry {
    if (bytes.len < abi.session_directory_at + 4) return error.InvalidSession;
    if (!std.mem.eql(u8, bytes[0..abi.session_magic.len], abi.session_magic)) return error.InvalidSession;
    const version = std.mem.readInt(u32, bytes[abi.session_magic.len..][0..4], .little);
    if (version != abi.session_version) return error.UnsupportedSessionVersion;
    const chunk_count = std.mem.readInt(u32, bytes[abi.session_magic.len + 4 ..][0..4], .little);
    if (chunk_count != abi.session_chunk_count) return error.InvalidSession;
    var entries: [abi.session_chunk_count]abi.TableEntry = undefined;
    for (0..abi.session_chunk_count) |i| {
        const at = abi.session_directory_at + i * abi.session_entry_size;
        entries[i] = .{
            .offset = std.mem.readInt(u32, bytes[at..][0..4], .little),
            .count = std.mem.readInt(u32, bytes[at + 4 ..][0..4], .little),
            .stride = std.mem.readInt(u32, bytes[at + 8 ..][0..4], .little),
        };
        if (entries[i].stride != abi.session_strides[i]) return error.InvalidSession;
    }
    const end = snapshotPayloadEnd(&entries);
    if (end > bytes.len) return error.InvalidSession;
    return entries;
}

test "session snapshot round trip restores tick" {
    if (!@import("build_options").simulator) return error.SkipZigTest;
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    var session: Session = undefined;
    var config = MissionConfig.startup_config;
    config.sim_robot = .fromLiteral("proteus");
    config.sim_scene = .fromLiteral("robot_only");
    try session.init(allocator, std.testing.io, &config, 6, false);
    defer session.deinit();
    const idle: types.ThrusterCommandInput = @splat(0);
    try session.setThrusters(&idle);
    for (0..12) |_| {
        if (session.tick == std.math.maxInt(u64)) return error.TickOverflow;
        const effective = session.commands[0..session.thruster_len];
        for (session.scene.faults) |fault| {
            if (!fault.active(session.tick)) continue;
            switch (fault.effect) {
                .dead => |channel| effective[channel] *= 0,
                .weak => |weak| effective[weak.thruster] *= weak.scale,
                else => {},
            }
        }
        const zero_wrench = abi.Wrench{ .id = 0, .force = @splat(0), .torque = @splat(0) };
        for (session.wrenches, session.scene.objects) |*wrench, slot| {
            wrench.* = zero_wrench;
            wrench.id = database.entity(slot).id;
        }
        session.saveVelocities();
        for (session.scene.forces) |force| try forces.evaluate(force, &session);
        try session.backend.step(session.wrenches);
        session.tick += 1;
        try session.validateReadback();
        const bounds = session.scene.settings.bounds;
        for (session.backend.states) |body| {
            if (S.norm(S.fromAbi3(body.pose_pos)) > bounds.position or
                S.norm(S.fromAbi3(body.linear_vel)) > bounds.speed or
                S.norm(S.fromAbi3(body.angular_vel)) > bounds.angular_speed) return error.SimulationBoundsExceeded;
        }
        session.refreshSensors();
    }
    const tick_before = session.tick;
    const bytes = try session.snapshot(allocator);
    session.tick += 1;
    try session.restoreSnapshot(allocator, bytes);
    try std.testing.expectEqual(tick_before, session.tick);
}