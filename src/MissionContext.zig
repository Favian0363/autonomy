const std = @import("std");
const Io = std.Io;
const math = @import("math.zig");
const bc = @import("BodyControl.zig");

const Auv = @import("Auv.zig");
const Serial = @import("Serial.zig");
const MissionConfig = @import("MissionConfig.zig");
const MissionIO = @import("MissionIO.zig");

const simulator_enabled = @import("build_options").simulator;
const SimSession = if (simulator_enabled) @import("simulator/Session.zig").Session else struct {
    pub fn init(_: *@This(), _: std.mem.Allocator, _: std.Io, _: []const u8, _: []const u8, _: usize, _: bool) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn deinit(_: *@This()) void {}
    pub fn setThrusters(_: *@This(), _: *const [MissionConfig.thruster_count]f32) !void {
        return error.SimulatorNotBuilt;
    }
    pub fn nextFrame(_: *@This(), _: *Auv.Frame) !void {
        return error.SimulatorNotBuilt;
    }
};

const MissionContext = @This();
const Visualizer = if (simulator_enabled) @import("Visualizer.zig") else void;
const VisualizerMeshCache = if (simulator_enabled) @import("simulator/VisualizerMeshCache.zig").VisualizerMeshCache else void;
const SceneMeshDraw = if (simulator_enabled) @import("simulator/VisualizerMeshCache.zig").DrawContext else void;
const drawSceneMeshes = if (simulator_enabled) @import("simulator/VisualizerMeshCache.zig").drawEmbedded else void;
const EmbeddedView = if (simulator_enabled) struct {
    viewer: Visualizer,
    scene_meshes: VisualizerMeshCache = undefined,
    has_scene_meshes: bool = false,
    mesh_draw: SceneMeshDraw = .{ .cache = undefined, .bodies = &.{} },
    last_idle_draw: ?Io.Clock.Timestamp = null,
    config_reply_revision: u64 = 0,
    config_rejected: bool = false,
} else void;

const Goal = MissionIO.Goal;

io: Io,

frame: Auv.Frame,

seen_objects: [Auv.Frame.max_objects]Auv.Object,
seen_objects_len: u8,

target_position: math.Vector3f,
goal_queue_buf: [MissionConfig.queue_capacity_max]Goal = @splat(std.mem.zeroes(Goal)),
queue: std.Deque(Goal) = undefined,
target_orientation: math.Quaternionf = bc.identity,
use_position: bool = false,
use_orientation: bool = false,
position_axes: Auv.AxisOk = .{ .x = true, .y = true, .z = true },
desired: math.Vector2f = .{ 0, 0 },
desired_height: ?f64 = null,
translation: ?math.Vector3f = null,
pid_sum_err: math.Vector6f = @splat(0),
pid_prev_err: math.Vector6f = @splat(0),
tilt_correction: math.Vector3f = .{ 0, 0, 0 },
force_error: math.Vector3f = .{ 0, 0, 0 },
pid_prev_timestamp_ns: ?u64,

step: []const u8,
telemetry_frame: MissionIO.TelemetryFrame = std.mem.zeroes(MissionIO.TelemetryFrame),
telemetry: *MissionIO.Telemetry,
visualizer: ?*EmbeddedView,

seen_objects2d: [Auv.Frame.max_objects]Auv.Object2d = undefined,
seen_objects2d_len: u8 = 0,

auv: ?Auv,
mission_diyn_loader: ?MissionIO.MissionDiynLoader,
serial: ?Serial.Port,
serial_parser: Serial.Parser = .{},
simulator: if (simulator_enabled) ?SimSession else void = if (simulator_enabled) null else {},
hardware_frame_time: ?Io.Clock.Timestamp = null,

/// Live configuration, retained across mission attempts and while idle.
config: MissionConfig.MissionConfig,

pub fn initProcess(ctx: *MissionContext, allocator: std.mem.Allocator, io: Io, windowed: bool) !void {
    ctx.io = io;
    ctx.config = MissionConfig.startup_config;
    ctx.config.sensor_source = if (simulator_enabled) .simulated else .plugin;
    ctx.visualizer = null;
    if (windowed) ctx.config.telemetry_transport = .disabled;

    // Keep the control path available even if the configured export fails.
    var bootstrap = ctx.config;
    bootstrap.telemetry_transport = .disabled;
    bootstrap.telemetry_log_path = .{};
    ctx.telemetry = try allocator.create(MissionIO.Telemetry);
    ctx.telemetry.* = try .init(io, bootstrap, !windowed);
    errdefer ctx.telemetry.deinit(io);
    ctx.telemetry.reconfigure(io, ctx.config) catch |err| {
        ctx.telemetry.report(err);
        ctx.telemetry.telemetry_error = ctx.telemetry.last_error;
    };

    if (comptime simulator_enabled) {
        if (windowed) {
            const embedded = try allocator.create(EmbeddedView);
            embedded.* = .{
                .viewer = Visualizer.init(&ctx.config, 1, ctx.config.mission_id == .mission_control, true) catch |err| {
                    std.log.err("could not open the simulator visualizer ({s}); use --headless to run without a display", .{@errorName(err)});
                    return err;
                },
            };
            ctx.visualizer = embedded;
        }
    }
}

pub fn deinitProcess(ctx: *MissionContext) void {
    if (comptime simulator_enabled) {
        if (ctx.visualizer) |embedded| {
            if (embedded.has_scene_meshes) embedded.scene_meshes.deinit();
            embedded.viewer.deinit();
        }
    }
    ctx.telemetry.deinit(ctx.io);
}

pub fn serviceWithoutFrame(ctx: *MissionContext) !void {
    try ctx.present(null);
    try ctx.telemetry.pollControl(ctx.io, &ctx.config);
}

fn present(ctx: *MissionContext, frame: ?*const MissionIO.TelemetryFrame) !void {
    if (comptime !simulator_enabled) return;
    const embedded = ctx.visualizer orelse return;
    if (Visualizer.shouldClose()) return error.WindowClosed;

    // The simulation clock paces active frames. Pace drawing here only during
    // idle or serial startup, where there is no simulation clock.
    if (frame == null) {
        if (embedded.last_idle_draw) |last| {
            if (last.untilNow(ctx.io).raw.toNanoseconds() < 16_666_667) return;
        }
        embedded.last_idle_draw = .now(ctx.io, .awake);
    } else {
        embedded.last_idle_draw = null;
    }

    var scene_draw: ?Visualizer.SceneDraw = null;
    if (embedded.has_scene_meshes) {
        if (ctx.simulator) |session| {
            embedded.mesh_draw.bodies = session.bodyStates();
            scene_draw = .{ .ctx = &embedded.mesh_draw, .draw = drawSceneMeshes };
        }
    }
    const actions = try embedded.viewer.tick(.{
        .status = ctx.telemetry.status(&ctx.config, embedded.config_reply_revision, embedded.config_rejected),
        .frame = frame,
        .telemetry_status = if (frame != null) .live else if (ctx.telemetry.state == .starting) .connecting else .disabled,
        .scene_draw = scene_draw,
    });
    embedded.config_reply_revision = 0;
    embedded.config_rejected = false;
    if (actions.config) |packet| {
        embedded.config_reply_revision = packet.revision;
        ctx.telemetry.applyConfig(ctx.io, &ctx.config, packet) catch |err| {
            ctx.telemetry.report(err);
            embedded.config_rejected = true;
        };
    }
    if (actions.lifecycle) |request| ctx.telemetry.acceptRequest(request);
    if (actions.thrusters) |command| ctx.telemetry.acceptThrusters(ctx.io, &ctx.config, command);
    if (actions.teleop) |command| ctx.telemetry.acceptTeleop(command);
    if (actions.mission) |command| ctx.telemetry.acceptMission(command) catch |err| ctx.telemetry.report(err);
}

fn waitForReady(ctx: *MissionContext, port: *Serial.Port, parser: *Serial.Parser) !void {
    const max_retries = 1000;
    for (0..max_retries + 1) |attempt| {
        try ctx.serviceWithoutFrame();
        if (try port.readReplies(parser)) return;
        if (attempt < max_retries) try Io.sleep(ctx.io, .fromMilliseconds(10), .awake);
    }
    return error.SerialReadyTimeout;
}

pub fn init(ctx: *MissionContext, allocator: std.mem.Allocator) !void {
    try ctx.serviceWithoutFrame();
    const io = ctx.io;
    const telemetry = ctx.telemetry;
    const config = ctx.config;
    try config.validate();
    if (!simulator_enabled and config.serial_port.len == 0) return error.MissingOutput;
    const tam_len = config.tam_len;

    // MissionConfig compile-time preset checks bind Proteus to the six-channel
    // firmware protocol; the profile selects whether serial output is permitted.
    if (config.serial_port.len != 0) switch (config.mission_profile) {
        .proteus_hwd => {},
        .stonefish_sim => return error.InvalidThrusterCount,
    };
    var mission_diyn_loader: ?MissionIO.MissionDiynLoader = null;
    var auv: ?Auv = null;
    errdefer if (mission_diyn_loader) |*loader| loader.dynlib.close();
    errdefer if (auv) |loaded| loaded.deinit();
    if (comptime !simulator_enabled) {
        var loader = MissionIO.MissionDiynLoader.init(try config.sensor_library.slice());
        const loaded = try loader.load();
        mission_diyn_loader = loader;
        auv = loaded;
        auv.?.init();
        try telemetry.pollControl(io, &ctx.config);
        // The void initialization ABI reports failures through the first frame.
        // Check it before opening serial output or waiting for firmware READY.
        var initial_frame: Auv.Frame = undefined;
        try auv.?.yieldUntilNextFrame(&initial_frame);
        try telemetry.pollControl(io, &ctx.config);
    }
    var serial = try Serial.Port.init(io, try config.serial_port.sentinelSlice());
    errdefer if (serial) |*port| port.deinit();
    var parser: Serial.Parser = .{};
    if (serial) |*port| {
        try ctx.waitForReady(port, &parser);
        std.log.info("READY on {s} at 115200 baud", .{try config.serial_port.slice()});
    }
    ctx.* = .{
        .frame = undefined,

        .seen_objects = undefined,
        .seen_objects_len = 0,

        .target_position = @splat(0),
        .queue = undefined,

        .pid_prev_timestamp_ns = null,

        .step = "",
        .telemetry = telemetry,
        .visualizer = ctx.visualizer,

        .auv = auv,
        .serial = serial,
        .serial_parser = parser,
        .simulator = if (comptime simulator_enabled) null else {},
        .mission_diyn_loader = mission_diyn_loader,

        .config = ctx.config,

        .io = io,
    };
    ctx.queue = .initBuffer(&ctx.goal_queue_buf);

    if (comptime simulator_enabled) {
        const realtime = ctx.visualizer != null or config.sim_realtime or config.serial_port.len != 0;
        // Init in place: scene BodySlot slices must reference catalogs stored in this Session, not a stack copy.
        ctx.simulator = @as(SimSession, undefined);
        errdefer if (ctx.simulator) |*session| session.deinit();
        try ctx.simulator.?.init(allocator, io, &ctx.config, tam_len, realtime);
        if (ctx.visualizer) |embedded| {
            embedded.scene_meshes = try VisualizerMeshCache.init(allocator, &ctx.simulator.?.scene);
            embedded.has_scene_meshes = true;
            embedded.mesh_draw = .{ .cache = &embedded.scene_meshes, .bodies = &.{} };
        }
    }
}

pub fn deinit(ctx: *MissionContext) void {
    if (ctx.serial) |*port| port.deinit();
    if (ctx.auv) |auv| auv.deinit();
    if (ctx.mission_diyn_loader) |*loader| loader.dynlib.close();
    if (comptime simulator_enabled) {
        if (ctx.visualizer) |embedded| {
            if (embedded.has_scene_meshes) {
                embedded.scene_meshes.deinit();
                embedded.has_scene_meshes = false;
            }
        }
        if (ctx.simulator) |*session| session.deinit();
    }
}

pub fn reset(ctx: *MissionContext) void {
    ctx.seen_objects_len = 0;
    ctx.clearQueue();
    ctx.resetOdometryPid();
    ctx.target_orientation = bc.identity;
    ctx.use_position = false;
    ctx.use_orientation = false;
    ctx.desired = .{ 0, 0 };
    ctx.desired_height = null;
    ctx.translation = null;
    ctx.pid_prev_timestamp_ns = null;
}

/// Live cameras usually provide pacing; replay or synthetic plugins may not.
fn paceHardwareFrame(ctx: *MissionContext) !void {
    if (ctx.serial == null) return;
    if (ctx.hardware_frame_time) |previous| {
        const remaining = 16_666_667 - previous.untilNow(ctx.io).raw.toNanoseconds();
        if (remaining > 0) try Io.sleep(ctx.io, .fromNanoseconds(remaining), .awake);
    }
    ctx.hardware_frame_time = .now(ctx.io, .awake);
}

pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) !void {
    const start = Io.Clock.Timestamp.now(ctx.io, .awake);
    try ctx.telemetry.pollControl(ctx.io, &ctx.config);
    if (comptime simulator_enabled) {
        try ctx.simulator.?.nextFrame(&ctx.frame);
    } else {
        try ctx.auv.?.yieldUntilNextFrame(&ctx.frame);
        try ctx.paceHardwareFrame();
    }
    if (ctx.serial) |*port| _ = try port.readReplies(&ctx.serial_parser);
    try ctx.telemetry.pollControl(ctx.io, &ctx.config);

    for (ctx.frame.objects[0..ctx.frame.objects_len]) |frame_object| {
        const maybe_seen_object = for (ctx.seen_objects[0..ctx.seen_objects_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) {
                break seen_object;
            }
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else if (ctx.seen_objects_len < ctx.seen_objects.len) {
            ctx.seen_objects[ctx.seen_objects_len] = frame_object;
            ctx.seen_objects_len += 1;
        }
    }

    for (ctx.frame.objects2d[0..ctx.frame.objects2d_len]) |frame_object| {
        const maybe_seen_object = for (ctx.seen_objects2d[0..ctx.seen_objects2d_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) {
                break seen_object;
            }
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else if (ctx.seen_objects2d_len < ctx.seen_objects2d.len) {
            ctx.seen_objects2d[ctx.seen_objects2d_len] = frame_object;
            ctx.seen_objects2d_len += 1;
        }
    }
    var telemetry_frame = &ctx.telemetry_frame;
    telemetry_frame.magic = "AUV1".*;
    telemetry_frame.abi_version = MissionIO.telemetry_frame_abi_version;
    telemetry_frame.timestamp = ctx.frame.timestamp;
    telemetry_frame.camera_pose = MissionIO.telemetryPose(ctx.frame.camera_pose);
    telemetry_frame.position_ok = .{
        .x = ctx.frame.position_ok.x,
        .y = ctx.frame.position_ok.y,
        .z = ctx.frame.position_ok.z,
        ._pad = 0,
    };
    telemetry_frame.attitude_ok = .{
        .x = ctx.frame.attitude_ok.x,
        .y = ctx.frame.attitude_ok.y,
        .z = ctx.frame.attitude_ok.z,
        ._pad = 0,
    };
    telemetry_frame.depth_ok = ctx.frame.pressure_depth_ok;
    telemetry_frame.depth = MissionIO.telemetryFloat(ctx.frame.pressure_depth);
    const step_len = @min(ctx.step.len, telemetry_frame.step.len);
    telemetry_frame.step_len = @intCast(step_len);
    @memcpy(telemetry_frame.step[0..step_len], ctx.step[0..step_len]);

    var thruster_buffer: [MissionConfig.thruster_count]f32 = @splat(0);
    const direct = ctx.config.mission_id == .thruster_control;
    const thruster_values = if (direct) blk: {
        if (ctx.telemetry.direct_command) |command| thruster_buffer = command.values;
        break :blk thruster_buffer[0..ctx.config.tam_len];
    } else ctx.controlStep(&thruster_buffer, telemetry_frame);
    if (!direct or ctx.telemetry.direct_command != null) {
        if (comptime simulator_enabled) try ctx.simulator.?.setThrusters(&thruster_buffer);
        if (ctx.serial) |*port| {
            const packet = Serial.encode(&thruster_values[0..Serial.protocol_v2_command_channel_count].*);
            try port.writeAll(&packet, 100);
        }
    }

    const nobj = @min(@as(usize, ctx.seen_objects_len), ctx.seen_objects.len);
    const nbox = @min(@as(usize, ctx.seen_objects2d_len), ctx.seen_objects2d.len);
    var goals: [MissionConfig.queue_capacity_max]Goal = undefined;
    var goals_len: usize = 0;
    var goal_it = ctx.queue.iterator();
    while (goal_it.next()) |goal| {
        if (goals_len >= goals.len) break;
        goals[goals_len] = goal;
        goals_len += 1;
    }
    if (goals_len == 0 and ctx.use_position and ctx.frame.position_ok.x and
        ctx.frame.position_ok.y and ctx.frame.position_ok.z)
    {
        const q = bc.normalizedQuat(ctx.target_orientation) catch bc.identity;
        const ray = math.quaternionRotate(q, ctx.config.camera_forward);
        goals[0] = .{
            .position = ctx.target_position,
            .look_at = if (ctx.use_orientation) ctx.frame.camera_pose.pos + ray * @as(math.Vector3f, @splat(2.0)) else ctx.target_position,
        };
        goals_len = 1;
    }
    const elapsed = start.untilNow(ctx.io);
    telemetry_frame.cycle_us = @intCast(@max(0, elapsed.raw.toMicroseconds()));
    telemetry_frame.force_error = MissionIO.telemetryVec3(ctx.force_error);
    telemetry_frame.objects_len = @intCast(nobj);
    for (ctx.seen_objects[0..nobj], telemetry_frame.objects[0..nobj]) |source, *dest| {
        dest.* = .{
            .pose = MissionIO.telemetryPose(source.bbox.pose),
            .size = MissionIO.telemetryVec3(source.bbox.size),
            .id = source.id,
            .cls = source.cls,
            ._pad = .{ 0, 0, 0 },
        };
    }
    telemetry_frame.boxes_len = @intCast(nbox);
    for (ctx.seen_objects2d[0..nbox], telemetry_frame.boxes[0..nbox]) |source, *dest| {
        dest.* = .{
            .top_left = .{ .x = source.top_left.x, .y = source.top_left.y },
            .bottom_right = .{ .x = source.bottom_right.x, .y = source.bottom_right.y },
            .id = source.id,
            .cls = source.cls,
            ._pad = .{ 0, 0, 0 },
        };
    }
    telemetry_frame.thrusters_len = @intCast(ctx.config.tam_len);
    @memcpy(telemetry_frame.thrusters[0..ctx.config.tam_len], thruster_values[0..ctx.config.tam_len]);
    telemetry_frame.goals_len = @intCast(goals_len);
    for (goals[0..goals_len], telemetry_frame.goals[0..goals_len]) |source, *dest| {
        dest.* = .{
            .position = MissionIO.telemetryVec3(source.position),
            .look_at = MissionIO.telemetryVec3(source.look_at),
        };
    }
    ctx.telemetry.publishTelemetry(ctx.io, telemetry_frame);
    if (comptime simulator_enabled) {
        if (ctx.visualizer != null) {
            try ctx.present(telemetry_frame);
            try ctx.telemetry.pollControl(ctx.io, &ctx.config);
        }
    }
}

pub fn resetOdometryPid(ctx: *MissionContext) void {
    ctx.pid_sum_err = @splat(0);
    ctx.pid_prev_err = @splat(0);
    ctx.tilt_correction = .{ 0, 0, 0 };
}

/// World direction of body +Y at a level heading. Yaw 0 looks along +Y.
fn lookAtHeading(heading: f64) math.Vector3f {
    return .{ -@sin(heading), @cos(heading), 0 };
}

/// Point the camera at `target` from the current pose, clamped by `rotation_limit`.
/// A coincident target keeps the previous look.
fn setLookAtTarget(ctx: *MissionContext, target: math.Vector3f) !void {
    const offset = target - ctx.frame.camera_pose.pos;
    if (math.length3f(offset) <= 1e-6) return;
    const toward = math.normalize3f(offset);
    ctx.target_orientation = try bc.lookWithinLimits(
        ctx.config.camera_forward,
        toward,
        bc.identity,
        ctx.config.rotation_limit,
    );
}

fn slew3(current: math.Vector3f, desired: math.Vector3f, max_step: f64) math.Vector3f {
    if (max_step <= 0) return current;
    const delta = desired - current;
    const len = math.length3f(delta);
    if (len <= max_step or len <= 1e-6) return desired;
    return current + delta / @as(math.Vector3f, @splat(len / max_step));
}

/// World-frame translation toward `target`, with disallowed or invalid axes removed before the body rotation.
fn bodyTranslationError(ctx: *const MissionContext, target: math.Vector3f, use_axis: Auv.AxisOk) math.Vector3f {
    var world = bc.positionError(ctx.frame.camera_pose.pos, target) catch return .{ 0, 0, 0 };
    const ok = ctx.frame.position_ok;
    if (!ok.x or !use_axis.x) world[0] = 0;
    if (!ok.y or !use_axis.y) world[1] = 0;
    if (!ok.z or !use_axis.z) world[2] = 0;
    const body_q = bc.normalizedQuat(ctx.frame.camera_pose.quat) catch return .{ 0, 0, 0 };
    return math.quaternionRotate(math.quaternionConjugate(body_q), world);
}

fn attitudeToward(ctx: *const MissionContext, orientation: math.Quaternionf) ?math.Vector3f {
    const current = bc.normalizedQuat(ctx.frame.camera_pose.quat) catch return null;
    const desired = bc.normalizedQuat(orientation) catch return null;
    return bc.attitudeError(current, desired) catch null;
}

/// Pose error for the axes the context is tracking. Untracked and unavailable axes are zero here.
fn poseError(ctx: *const MissionContext) math.Vector6f {
    const translation = if (ctx.use_position)
        bodyTranslationError(ctx, ctx.target_position, ctx.position_axes)
    else
        math.Vector3f{ 0, 0, 0 };
    const angular = if (ctx.use_orientation)
        attitudeToward(ctx, ctx.target_orientation) orelse math.Vector3f{ 0, 0, 0 }
    else
        math.Vector3f{ 0, 0, 0 };
    return .{ translation[0], translation[1], translation[2], angular[0], angular[1], angular[2] };
}

fn poseAvailable(ctx: *const MissionContext) [6]bool {
    const position = ctx.frame.position_ok;
    const attitude = ctx.frame.attitude_ok;
    return .{
        ctx.use_position and ctx.position_axes.x and position.x,
        ctx.use_position and ctx.position_axes.y and position.y,
        ctx.use_position and ctx.position_axes.z and position.z,
        ctx.use_orientation and attitude.x,
        ctx.use_orientation and attitude.y,
        ctx.use_orientation and attitude.z,
    };
}

/// Visual body error from the first box on this frame. Null when the frame has no box or `interaction` does not match the image error.
/// Translation rows come from the interaction matrix. Roll and pitch are the level-look error plus those matrix rows.
fn visualBodyError(ctx: *MissionContext) ?math.Vector6f {
    if (ctx.frame.objects2d_len == 0) return null;
    const object = ctx.frame.objects2d[0];
    const metrics = math.boxMetrics(
        ctx.frame.image_width,
        ctx.frame.image_height,
        .{ object.top_left.x, object.top_left.y },
        .{ object.bottom_right.x, object.bottom_right.y },
    );
    const ibvs = ctx.config.ibvs;
    const image_error = [_]f64{
        math.withinDeadband(ctx.desired[0] - metrics.cx, ibvs.center_deadband),
        math.withinDeadband(ctx.desired[1] - metrics.cy, ibvs.center_deadband),
        (ctx.desired_height orelse ibvs.target_height) - metrics.height,
    };
    var interaction_rows: [math.interaction_output_len][]const f64 = undefined;
    const ibvs_cols = @min(ibvs.interaction_cols, MissionConfig.max_ibvs_error_cols);
    for (&interaction_rows, ctx.config.ibvs.interaction) |*row, interaction_row| {
        row.* = interaction_row[0..ibvs_cols];
    }
    const mapped = math.mapInteraction(interaction_rows[0..], &image_error) catch {
        return null;
    };
    const current = bc.normalizedQuat(ctx.frame.camera_pose.quat) catch unreachable;
    const yaw = math.quaternionToEuler(current)[2] + mapped[5];
    const look_q = bc.lookWithinLimits(
        ctx.config.camera_forward,
        lookAtHeading(yaw),
        bc.identity,
        ctx.config.rotation_limit,
    ) catch unreachable;
    var angular = bc.attitudeError(current, look_q) catch unreachable;
    angular[0] += mapped[3];
    angular[1] += mapped[4];
    return .{ mapped[0], mapped[1], mapped[2], angular[0], angular[1], angular[2] };
}

fn anyAxis(available: [6]bool) bool {
    for (available) |ok| if (ok) return true;
    return false;
}

/// Fill one body error from the pose, then ibvs. An axis with neither source is zero.
fn bodyError(ctx: *MissionContext) math.Vector6f {
    const visual = visualBodyError(ctx);
    var available = poseAvailable(ctx);
    var position_error = poseError(ctx);
    if (ctx.translation) |body| {
        position_error[0] = body[0];
        position_error[1] = body[1];
        position_error[2] = body[2];
        available[0] = true;
        available[1] = true;
        available[2] = true;
    }
    if (visual == null and !anyAxis(available)) {
        ctx.resetOdometryPid();
        ctx.pid_prev_timestamp_ns = null;
    }
    return math.greedy6(position_error, visual, available);
}

fn quaternionFromRotationVector(rot: math.Vector3f) math.Quaternionf {
    const angle = math.length3f(rot);
    if (angle <= 1e-6) return bc.identity;
    return bc.forwardRotation(rot, angle) catch bc.identity;
}

/// Track the context pose and return the thruster values. Invalid pose axes contribute no error.
fn controlStep(
    ctx: *MissionContext,
    thruster_buffer: *[MissionConfig.thruster_count]f32,
    telemetry_frame: *MissionIO.TelemetryFrame,
) []const f32 {
    const body_error = ctx.bodyError();
    const timestamp_ns = ctx.frame.timestamp;
    const dt: ?f64 = if (ctx.pid_prev_timestamp_ns) |previous| blk: {
        const elapsed_ns = timestamp_ns - previous;
        ctx.pid_prev_timestamp_ns = timestamp_ns;
        break :blk @as(f64, @floatFromInt(elapsed_ns)) * 1e-9;
    } else blk: {
        ctx.pid_prev_timestamp_ns = timestamp_ns;
        break :blk null;
    };

    const tam_len = ctx.config.tam_len;
    const tam_rows = ctx.config.tam[0..tam_len];
    const camera_forward = ctx.config.camera_forward;
    const rotation_limit = ctx.config.rotation_limit;
    const thrust_direction = bc.ThrustDirections.fromTamLinearForces(tam_rows) catch unreachable;
    const current = ctx.frame.camera_pose.quat;
    const error_body: math.Vector3f = .{ body_error[0], body_error[1], body_error[2] };
    const rot: math.Vector3f = .{ body_error[3], body_error[4], body_error[5] };
    const delta = quaternionFromRotationVector(rot);
    const current_q = bc.normalizedQuat(current) catch bc.identity;
    var target_q = bc.normalizedQuat(math.quaternionMul(current_q, delta)) catch current_q;
    const odom = ctx.config.odometry;
    var desired_tilt: math.Vector3f = .{ 0, 0, 0 };
    var force_error: math.Vector3f = .{ 0, 0, 0 };
    const moments = bc.ThrustDirections.fromTamMoments(tam_rows) catch unreachable;
    // Coverage swings a thruster onto a force component the map cannot produce.
    const translation_command: math.Vector3f = blk: {
        force_error = error_body;
        const available = thrust_direction.alongThrust(error_body);
        var force_body: math.Vector3f = .{ 0, 0, 0 };
        inline for (0..3) |i| {
            var rate: f64 = 0;
            if (dt) |t| {
                ctx.pid_sum_err[i] += available[i] * t;
                rate = (error_body[i] - ctx.pid_prev_err[i]) / t;
            }
            force_body[i] = odom.kp[i] * error_body[i] + odom.ki[i] * ctx.pid_sum_err[i] + odom.kd[i] * rate;
            ctx.pid_prev_err[i] = error_body[i];
        }
        const coverage = bc.coverForce(thrust_direction, moments, force_body, rotation_limit, camera_forward);
        if (coverage.tilt) |tilt| desired_tilt = tilt.axis * @as(math.Vector3f, @splat(tilt.angle));
        break :blk coverage.force;
    };
    // The tilt follows the reshaped force. Slew it so the attitude loop does not see a step.
    const tilt_step = if (dt) |t| 0.8 * t else 0;
    ctx.tilt_correction = slew3(ctx.tilt_correction, desired_tilt, tilt_step);
    const tilt_len = math.length3f(ctx.tilt_correction);
    if (tilt_len > 1e-4) {
        const tilt_q = bc.forwardRotation(ctx.tilt_correction, tilt_len) catch unreachable;
        target_q = bc.normalizedQuat(math.quaternionMul(target_q, tilt_q)) catch unreachable;
    }
    var attitude_error = bc.attitudeError(current, target_q) catch unreachable;
    const attitude_axis = [3]bool{ ctx.frame.attitude_ok.x, ctx.frame.attitude_ok.y, ctx.frame.attitude_ok.z };
    inline for (0..3) |i| {
        if (!attitude_axis[i]) attitude_error[i] = 0;
    }
    const translation_body: math.Vector3f = blk: {
        var force = thrust_direction.alongThrust(translation_command);
        if (math.length3f(attitude_error) > odom.rotation_gate) {
            force[0] = 0;
            force[1] = 0;
            //we doing zero out force[2] beause we always want to maintain the z axis.
        }
        break :blk force;
    };
    ctx.force_error = force_error;

    var rotation: math.Vector3f = .{ 0, 0, 0 };
    inline for (0..3) |i| {
        const lane = i + 3;
        if (attitude_axis[i]) {
            var rate: f64 = 0;
            if (dt) |t| {
                rate = (attitude_error[i] - ctx.pid_prev_err[lane]) / t;
                ctx.pid_sum_err[lane] += attitude_error[i] * t;
            }
            ctx.pid_prev_err[lane] = attitude_error[i];
            rotation[i] = odom.kp[lane] * attitude_error[i] + odom.ki[lane] * ctx.pid_sum_err[lane] + odom.kd[lane] * rate;
        } else {
            ctx.pid_sum_err[lane] = 0;
            ctx.pid_prev_err[lane] = 0;
        }
    }
    const input: math.Vector6f = .{
        translation_body[0], translation_body[1], translation_body[2],
        rotation[0],         rotation[1],         rotation[2],
    };
    var thruster_math: [MissionConfig.thruster_count]f64 = undefined;
    _ = math.tamToThrusters(
        tam_rows,
        input,
        ctx.config.thrustor_saturate,
        5.0,
        thruster_math[0..tam_len],
    );
    for (thruster_math[0..tam_len], thruster_buffer[0..tam_len]) |value, *out| out.* = @floatCast(value);
    telemetry_frame.clear_screen = ctx.config.clear_print_stuff;
    telemetry_frame.has_dt = dt != null;
    telemetry_frame.dt = MissionIO.telemetryFloat(dt orelse 0);
    telemetry_frame.kp = MissionIO.telemetryVec6(odom.kp);
    telemetry_frame.ki = MissionIO.telemetryVec6(odom.ki);
    telemetry_frame.kd = MissionIO.telemetryVec6(odom.kd);
    var telemetry_tam: [MissionConfig.thruster_count][MissionConfig.tam_cols]f32 = @splat(@splat(0));
    for (ctx.config.tam[0..tam_len], 0..) |row, index| {
        telemetry_tam[index] = MissionIO.telemetryVec6(row);
    }
    telemetry_frame.tam = telemetry_tam;
    telemetry_frame.tam_len = @intCast(tam_len);
    telemetry_frame.control_pose = MissionIO.telemetryVec6(math.poseTo6f(ctx.frame.camera_pose));
    telemetry_frame.translation_error = MissionIO.telemetryVec3(error_body);
    telemetry_frame.target_orientation = MissionIO.telemetryQuat(ctx.target_orientation);
    telemetry_frame.target_quaternion = MissionIO.telemetryQuat(target_q);
    telemetry_frame.attitude_error = MissionIO.telemetryVec3(attitude_error);
    telemetry_frame.rotation_limit = MissionIO.telemetryVec3(rotation_limit);
    telemetry_frame.input = MissionIO.telemetryVec6(input);
    return thruster_buffer[0..tam_len];
}

/// Search while holding still. Returns the first object of `cls`.
pub fn yieldFind(ctx: *MissionContext, cls: u8) !*const Auv.Object {
    ctx.use_position = true;
    ctx.use_orientation = true;
    ctx.translation = null;
    var seen: usize = 0;

    while (true) {
        for (ctx.seen_objects[seen..ctx.seen_objects_len]) |*object| {
            if (object.cls == cls) return object;
            seen += 1;
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

const object_clearance: f64 = 2;
const look_done: f64 = 0.1;

fn reached(ctx: *const MissionContext, target: math.Vector3f) bool {
    const ok = ctx.frame.position_ok;
    if (!ok.x or !ok.y or !ok.z) return false;
    return math.length3f(target - ctx.frame.camera_pose.pos) <= ctx.config.goal_dist_threshold;
}

/// Horizontal unit direction from the vehicle toward the object. World +Y when they overlap.
fn approachDirection(ctx: *const MissionContext, object: *const Auv.Object) math.Vector2f {
    const delta = math.xy(object.bbox.pose.pos) - math.xy(ctx.frame.camera_pose.pos);
    if (math.length2f(delta) <= 1e-4) return .{ 0, 1 };
    return math.normalize2f(delta);
}

/// Half the oriented box, measured along a horizontal unit direction.
fn horizontalHalfExtent(object: *const Auv.Object, direction: math.Vector2f) f64 {
    const rot = math.normalize4f(object.bbox.pose.quat);
    const world_dir: math.Vector3f = .{ direction[0], direction[1], 0 };
    const size = object.bbox.size;
    const axes = [3]math.Vector3f{
        math.quaternionRotate(rot, .{ 1, 0, 0 }),
        math.quaternionRotate(rot, .{ 0, 1, 0 }),
        math.quaternionRotate(rot, .{ 0, 0, 1 }),
    };
    var extent: f64 = 0;
    inline for (axes, 0..) |axis, i| extent += @abs(math.dot3f(world_dir, axis)) * size[i];
    return 0.5 * extent;
}

/// `sign` is -1 on the approach side and +1 on the far side. `direction` stays fixed for the maneuver.
fn standoffAlong(object: *const Auv.Object, direction: math.Vector2f, sign: f64) math.Vector3f {
    const reach = horizontalHalfExtent(object, direction) + object_clearance;
    const offset = direction * @as(math.Vector2f, @splat(sign * reach));
    const center = object.bbox.pose.pos;
    return .{ center[0] + offset[0], center[1] + offset[1], center[2] };
}

fn clearQueue(ctx: *MissionContext) void {
    ctx.queue.head = 0;
    ctx.queue.len = 0;
}

/// Face `object` and hold position. Returns when the attitude error is small.
pub fn yieldLookAt(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.target_position = ctx.frame.camera_pose.pos;
    ctx.use_position = true;
    ctx.use_orientation = true;
    ctx.translation = null;
    while (true) {
        try ctx.setLookAtTarget(object.bbox.pose.pos);
        try ctx.yieldUntilNextFrameAndUpdate();
        const err = attitudeToward(ctx, ctx.target_orientation) orelse continue;
        if (math.length3f(err) <= look_done) return;
    }
}

/// Drive to the near side of `object`, looking at the far side.
pub fn yieldMoveTo(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.clearQueue();
    const direction = approachDirection(ctx, object);
    const far = standoffAlong(object, direction, 1);
    try ctx.queue.pushBackBounded(.{
        .position = standoffAlong(object, direction, -1),
        .look_at = far,
    });
    try ctx.yieldQueue();
}

/// Drive to `position` while looking along `look_direction`.
pub fn yieldPoint(ctx: *MissionContext, position: math.Vector3f, look_direction: math.Vector3f) !void {
    ctx.clearQueue();
    try ctx.queue.pushBackBounded(.{
        .position = position,
        .look_at = position + look_direction,
    });
    try ctx.yieldQueue();
}

/// Stop short of `object`, then continue the same distance past it.
pub fn yieldPass(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.clearQueue();
    const direction = approachDirection(ctx, object);
    const far = standoffAlong(object, direction, 1);
    try ctx.queue.pushBackBounded(.{
        .position = standoffAlong(object, direction, -1),
        .look_at = far,
    });
    try ctx.queue.pushBackBounded(.{
        .position = far,
        .look_at = far,
    });
    try ctx.yieldQueue();
}

/// Drive the queued goals in order. Returns when the queue is empty.
pub fn yieldQueue(ctx: *MissionContext) !void {
    ctx.use_position = true;
    ctx.use_orientation = true;
    ctx.translation = null;
    while (ctx.queue.len != 0) {
        const goal = ctx.queue.front().?;
        ctx.target_position = goal.position;
        try ctx.setLookAtTarget(goal.look_at);
        try ctx.yieldUntilNextFrameAndUpdate();
        if (reached(ctx, goal.position)) {
            _ = ctx.queue.popFront();
        }
    }
}

const circle_corners: [4]math.Vector2f = .{
    .{ 0.5, 0.5 },
    .{ 0.5, -0.5 },
    .{ -0.5, -0.5 },
    .{ -0.5, 0.5 },
};

fn circleCorner(object: *const Auv.Object, index: usize, depth: f64) math.Vector3f {
    const local_2d = circle_corners[index] * math.xy(object.bbox.size);
    const rotated = math.xy(math.quaternionRotate(math.normalize4f(object.bbox.pose.quat), .{ local_2d[0], local_2d[1], 0 }));
    const outward = if (math.length2f(rotated) <= 1e-4) math.Vector2f{ 1, 0 } else math.normalize2f(rotated);
    const pos = math.xy(object.bbox.pose.pos) + rotated + outward * @as(math.Vector2f, @splat(object_clearance));
    return .{ pos[0], pos[1], depth };
}

fn yieldPlaceLookingAt(ctx: *MissionContext, place: math.Vector3f, object: *const Auv.Object) !void {
    ctx.clearQueue();
    try ctx.queue.pushBackBounded(.{ .position = place, .look_at = object.bbox.pose.pos });
    try ctx.yieldQueue();
}

/// Visit the four outside corners of `object`, then return to the start.
pub fn yieldCircle(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.clearQueue();
    const start = ctx.frame.camera_pose.pos;
    const look_at = object.bbox.pose.pos;
    var corners: [circle_corners.len]math.Vector3f = undefined;
    for (0..circle_corners.len) |i| corners[i] = circleCorner(object, i, start[2]);

    var first: usize = 0;
    var best = std.math.inf(f64);
    const sub = math.xy(start);
    for (corners, 0..) |corner, i| {
        const dist = math.length2f(math.xy(corner) - sub);
        if (dist < best) {
            best = dist;
            first = i;
        }
    }

    for (0..corners.len) |step| {
        const index = (first + step) % corners.len;
        try ctx.queue.pushBackBounded(.{
            .position = corners[index],
            .look_at = look_at,
        });
    }
    try ctx.queue.pushBackBounded(.{ .position = start, .look_at = look_at });
    try ctx.yieldQueue();
}
