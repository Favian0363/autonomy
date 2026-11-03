const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");
const math = @import("math.zig");
const bc = @import("BodyControl.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");
const SerialBinaryWriter = @import("SerialBinaryWriter.zig");

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;
const telemetry_log_path: ?[]const u8 = null;
const telemetry_addr: []const u8 = "127.0.0.1:5600";

var stop_requested = std.atomic.Value(bool).init(false);

fn requestStop(_: std.posix.SIG) callconv(.c) void {
    stop_requested.store(true, .monotonic);
}

io: Io,

frame: Auv.Frame,

seen_objects: [Auv.Frame.max_objects]Auv.Object,
seen_objects_len: u8,

goal_dist_threshold: f32,
thrustor_saturate: f32,

log_counter: u32,
log_freq_div: u16,

target_position: math.Vector3f,
/// Last look. Identity until the first valid direction, then kept when the camera reaches the target.
target_orientation: math.Quaternionf = bc.identity,
/// Goal chosen before the frame. The yield turns it into `command`.
motion: Track = .{},
/// Numeric command filled from the current frame. Only the yield writes this.
command: Command = .{},
/// Odometry PID memory in TAM order. Lanes 0..2 are body translation, lanes 3..5 are attitude.
pid_sum_err: math.Vector6f = @splat(0),
pid_prev_err: math.Vector6f = @splat(0),
/// Slew-limited tilt that swings a controllable force onto a missing translation.
tilt_correction: math.Vector3f = .{ 0, 0, 0 },
/// Orientation held while the command has no target orientation.
held_q: ?math.Quaternionf = null,
/// Last explicit target orientation. A mission-look change clears the attitude lanes of the PID.
explicit_q: ?math.Quaternionf = null,
/// Body-frame translation error the force is calculated from. X right, Y forward, Z down.
force_error: math.Vector3f = .{ 0, 0, 0 },
pid_prev_timestamp_ns: ?u64,

step: []const u8,
telemetry_file: ?Io.File = null,
telemetry_socket: Io.net.Socket = undefined,
telemetry_dest: Io.net.IpAddress = undefined,

seen_objects2d: [Auv.Frame.max_objects]Auv.Object2d = undefined,
seen_objects2d_len: u8 = 0,

auv: Auv,
auv_loader: AuvLoader,
auv_watcher: FileWatcher,

config: ConfigLoader.Config,
config_loader: ConfigLoader,
config_watcher: FileWatcher,

pub fn init(arena: std.mem.Allocator, io: Io, args: MissionArgs) !MissionContext {
    const stop: std.posix.Sigaction = .{ .handler = .{ .handler = requestStop }, .mask = std.posix.sigemptyset(), .flags = 0 };
    std.posix.sigaction(.INT, &stop, null);
    std.posix.sigaction(.TERM, &stop, null);

    var config_loader: ConfigLoader = try .init(arena, args.live_config_path);
    const config = config_loader.load(io) catch |err| {
        std.log.err("failed to load config: {s}", .{config_loader.config_path});
        return err;
    };
    if (print_stuff) {
        std.log.debug("succesfully loaded config: {s}", .{config_loader.config_path});
        std.log.debug("{any}", .{config});
    }

    var auv_loader: AuvLoader = try .init(arena, args.auv_dynlib_path);
    const auv = auv_loader.load(io) catch |err| {
        const dl_err = std.mem.span(std.c.dlerror()) orelse "NO_DL_ERROR";
        std.log.err("dlerror: `{s}`", .{dl_err});
        std.log.err("failed to load auv: {s}", .{auv_loader.auv_path});
        return err;
    };
    auv.init();
    if (print_stuff) {
        std.log.debug("succesfully loaded auv: {s}", .{auv_loader.auv_path});
        std.log.debug("{any}", .{auv});
    }

    const telemetry_dest = try Io.net.IpAddress.parseLiteral(telemetry_addr);
    const telemetry_bind: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
    const telemetry_socket = try telemetry_bind.bind(io, .{ .mode = .dgram });
    const telemetry_file: ?Io.File = if (telemetry_log_path) |path|
        try Io.Dir.cwd().createFile(io, path, .{})
    else
        null;

    return .{
        .frame = undefined,

        .seen_objects = undefined,
        .seen_objects_len = 0,

        .goal_dist_threshold = args.goal_dist_threshold,
        .thrustor_saturate = 5,

        .log_counter = 0,
        .log_freq_div = 120,

        .target_position = @splat(0),

        .pid_prev_timestamp_ns = null,

        .step = "",
        .telemetry_file = telemetry_file,
        .telemetry_socket = telemetry_socket,
        .telemetry_dest = telemetry_dest,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(arena, args.auv_dynlib_path),

        .config = config,
        .config_loader = config_loader,
        .config_watcher = try .init(arena, args.live_config_path),

        .io = io,
    };
}

/// Invalidates `seen_objects`
pub fn reset(ctx: *MissionContext) void {
    ctx.seen_objects_len = 0;
    ctx.resetOdometryPid();
    ctx.held_q = null;
    ctx.explicit_q = null;
    ctx.target_orientation = bc.identity;
    ctx.motion = .{};
    ctx.command = .{};
    ctx.pid_prev_timestamp_ns = null;
}

pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) !void {
    const start = Io.Clock.Timestamp.now(ctx.io, .awake);
    ctx.auv.yieldUntilNextFrame(&ctx.frame);
    if (stop_requested.load(.monotonic)) return error.Stopped;

    // std.log.debug("updating seen_objects...", .{});
    for (ctx.frame.objects[0..ctx.frame.objects_len]) |frame_object| {
        const maybe_seen_object = for (ctx.seen_objects[0..ctx.seen_objects_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) {
                break seen_object;
            }
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else {
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
        } else {
            ctx.seen_objects2d[ctx.seen_objects2d_len] = frame_object;
            ctx.seen_objects2d_len += 1;
        }
    }

    if (ctx.config_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        if (ctx.config_loader.load(ctx.io)) |config| {
            if (print_stuff) {
                std.log.debug("succesfully reloaded config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
                std.log.debug("{any}", .{config});
            }
            ctx.config = config;
        } else |err| {
            std.log.err("failed to reload config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous config", .{});
        }
    }

    if (ctx.auv_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        ctx.auv.deinit();
        if (ctx.auv_loader.load(ctx.io)) |auv| {
            if (print_stuff) {
                std.log.debug("succesfully reloaded auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
                std.log.debug("{any}", .{auv});
            }
            ctx.auv = auv;
            if (print_stuff) {
                std.log.debug("restarted auv loop", .{});
            }
            return error.RestartMission;
        } else |err| {
            std.log.err("failed to reload auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous auv", .{});
        }
        ctx.auv.init();
    }

    ctx.commandTrack();
    var thruster_buffer: [math.max_thrusters]f32 = @splat(0);
    const thruster_values = ctx.controlStep(&thruster_buffer);
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
    ctx.publishToUdp(thruster_values);

    const elapsed = start.untilNow(ctx.io);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{elapsed.raw.toMicroseconds()});
    }
    ctx.log_counter +%= 1;
}

// Field order is the little-endian packet.
const TelemetryPacket = struct {
    magic: [4]u8 = "AUV1".*,
    timestamp: u64,
    camera_pose: math.Pose,
    position_ok: Auv.AxisOk,
    attitude_ok: Auv.AxisOk,
    depth_ok: bool,
    depth: f32,
    objects: []const Auv.Object,
    boxes: []const Auv.Object2d,
    thrusters: []const f32,
    step: []const u8,
    force_error: math.Vector3f,
};

fn publishToUdp(ctx: *MissionContext, thrusters: []const f32) void {
    const nobj = @min(@as(usize, ctx.seen_objects_len), ctx.seen_objects.len);
    const nbox = @min(@as(usize, ctx.seen_objects2d_len), ctx.seen_objects2d.len);

    var buf: [64 * 1024]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    SerialBinaryWriter.write(&w, TelemetryPacket{
        .timestamp = ctx.frame.timestamp,
        .camera_pose = ctx.frame.camera_pose,
        .position_ok = ctx.frame.position_ok,
        .attitude_ok = ctx.frame.attitude_ok,
        .depth_ok = ctx.frame.pressure_depth_ok,
        .depth = ctx.frame.pressure_depth,
        .objects = ctx.seen_objects[0..nobj],
        .boxes = ctx.seen_objects2d[0..nbox],
        .thrusters = thrusters[0..@min(thrusters.len, 255)],
        .step = ctx.step[0..@min(ctx.step.len, 255)],
        .force_error = ctx.force_error,
    }) catch return;
    const packet = w.buffered();

    if (ctx.telemetry_file) |file| file.writeStreamingAll(ctx.io, packet) catch |err| {
        std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
    };
    // udp never blocks, even with the laptop unplugged
    ctx.telemetry_socket.send(ctx.io, &ctx.telemetry_dest, packet) catch {};
}

pub fn resetOdometryPid(ctx: *MissionContext) void {
    ctx.pid_sum_err = @splat(0);
    ctx.pid_prev_err = @splat(0);
    ctx.tilt_correction = .{ 0, 0, 0 };
}

fn clearOdometryAttitude(ctx: *MissionContext) void {
    ctx.pid_sum_err = .{ ctx.pid_sum_err[0], ctx.pid_sum_err[1], ctx.pid_sum_err[2], 0, 0, 0 };
    ctx.pid_prev_err = .{ ctx.pid_prev_err[0], ctx.pid_prev_err[1], ctx.pid_prev_err[2], 0, 0, 0 };
}

/// How body-forward error is chosen while a box is visible.
pub const ImageSurge = enum {
    /// Body Y comes from `target_height` minus the box height.
    scale,
    /// Body Y is a constant forward error. Yaw and heave still come from the box.
    forward,
};

/// Body-Y error used while `ImageSurge.forward` is active.
const forward_body_error: f32 = 2;

/// Image target carried by a `track` goal.
pub const LookForwardHeading = struct {
    id: u32,
    /// Desired image point. The center is `{0, 0}`.
    desired: math.Vector2f = .{ 0, 0 },
    /// Null uses `target_height` from the vehicle file.
    desired_height: ?f32 = null,
    /// Added to the scale-driven body-Y error. Ignored when `surge` is `.forward`.
    forward_bias: f32 = 0,
    /// Body X per unit of `desired_height - height`. Zero leaves sway to the matrix.
    range_sway: f32 = 0,
    surge: ImageSurge = .scale,
};

/// One goal. Position fills an axis when that measurement exists; otherwise vision. An axis with neither is zero.
pub const Track = struct {
    position: ?math.Vector3f = null,
    position_axes: Auv.AxisOk = .{ .x = true, .y = true, .z = true },
    visual: ?LookForwardHeading = null,
    orientation: ?math.Quaternionf = null,
    translation: ?math.Vector3f = null,
};

const Command = struct {
    translation: math.Vector3f = .{ 0, 0, 0 },
    target_orientation: ?math.Quaternionf = null,
    body_error: ?[6]f32 = null,
};

/// World direction of body +Y at a level heading. Yaw 0 looks along +Y.
fn lookAtHeading(heading: f32) math.Vector3f {
    return .{ -@sin(heading), @cos(heading), 0 };
}

fn object2dOnFrame(ctx: *const MissionContext, id: u32) ?Auv.Object2d {
    for (ctx.frame.objects2d[0..ctx.frame.objects2d_len]) |object| {
        if (object.id == id) return object;
    }
    return null;
}

/// Point the camera at `target` from the current pose, clamped by `rotation_limit`.
/// A coincident target keeps the previous look.
fn setLookAtTarget(ctx: *MissionContext, target: math.Vector3f) !void {
    const offset = target - ctx.frame.camera_pose.pos;
    if (math.length3f(offset) <= 1e-6) return;
    const toward = math.normalize3f(offset);
    ctx.target_orientation = try bc.lookWithinLimits(ctx.config.camera_forward, toward, bc.identity, ctx.config.rotation_limit);
}

fn slew3(current: math.Vector3f, desired: math.Vector3f, max_step: f32) math.Vector3f {
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

fn attitudeToward(ctx: *const MissionContext, orientation: ?math.Quaternionf) ?math.Vector3f {
    const target = orientation orelse return null;
    const current = bc.normalizedQuat(ctx.frame.camera_pose.quat) catch return null;
    const desired = bc.normalizedQuat(target) catch return null;
    return bc.attitudeError(current, desired) catch null;
}

/// Position and orientation errors for the axes the goal is allowed to fill. Unavailable axes are zero here.
fn positionGoalError(ctx: *const MissionContext, goal: Track) [6]f32 {
    const translation = if (goal.position) |target|
        bodyTranslationError(ctx, target, goal.position_axes)
    else
        math.Vector3f{ 0, 0, 0 };
    const angular = attitudeToward(ctx, goal.orientation) orelse math.Vector3f{ 0, 0, 0 };
    return .{ translation[0], translation[1], translation[2], angular[0], angular[1], angular[2] };
}

fn positionAvailable(ctx: *const MissionContext, goal: Track) [6]bool {
    const position = ctx.frame.position_ok;
    const attitude = ctx.frame.attitude_ok;
    const use_position = goal.position != null;
    const use_attitude = goal.orientation != null;
    return .{
        use_position and goal.position_axes.x and position.x,
        use_position and goal.position_axes.y and position.y,
        use_position and goal.position_axes.z and position.z,
        use_attitude and attitude.x,
        use_attitude and attitude.y,
        use_attitude and attitude.z,
    };
}

/// Visual body error. Null when the box is missing or `interaction` does not match the image error.
/// Translation rows keep surge, forward bias, and range sway. Roll and pitch are the level-look error plus those matrix rows.
fn visualBodyError(ctx: *MissionContext, heading: LookForwardHeading) ?[6]f32 {
    const object = object2dOnFrame(ctx, heading.id) orelse return null;
    const metrics = math.boxMetrics(
        ctx.frame.image_width,
        ctx.frame.image_height,
        .{ object.top_left.x, object.top_left.y },
        .{ object.bottom_right.x, object.bottom_right.y },
    );
    const vision = ctx.config.non_odometry;
    const image_error = [_]f32{
        math.withinDeadband(heading.desired[0] - metrics.cx, vision.center_deadband),
        math.withinDeadband(heading.desired[1] - metrics.cy, vision.center_deadband),
        (heading.desired_height orelse vision.target_height) - metrics.height,
    };
    var mapped = math.mapInteraction(vision.interaction, &image_error) catch |err| {
        std.log.err(
            "interaction map failed ({s}): expected {d} rows, each with {d} columns; config has {d} rows",
            .{ @errorName(err), math.interaction_output_len, image_error.len, vision.interaction.len },
        );
        if (vision.interaction.len > 0) {
            std.log.err("interaction row 0 has {d} columns", .{vision.interaction[0].len});
        }
        return null;
    };
    if (heading.surge == .forward) mapped[1] = forward_body_error else mapped[1] += heading.forward_bias;
    if (heading.desired_height) |height_ref| mapped[0] += heading.range_sway * (height_ref - metrics.height);
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

/// Fill one body error from position, then vision. An axis with neither source is zero.
fn commandTrack(ctx: *MissionContext) void {
    const goal = ctx.motion;
    const visual = if (goal.visual) |heading| visualBodyError(ctx, heading) else null;
    var available = positionAvailable(ctx, goal);
    var position_error = positionGoalError(ctx, goal);
    if (goal.translation) |body| {
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
    const chosen = math.greedy6(position_error, visual, available);
    ctx.explicit_q = null;
    ctx.command = .{
        .translation = .{ chosen[0], chosen[1], chosen[2] },
        .target_orientation = null,
        .body_error = chosen,
    };
}

fn quaternionFromRotationVector(rot: math.Vector3f) math.Quaternionf {
    const angle = math.length3f(rot);
    if (angle <= 1e-6) return bc.identity;
    return bc.forwardRotation(rot, angle) catch bc.identity;
}

fn resolveLook(ctx: *MissionContext, current: math.Quaternionf, look: ?math.Quaternionf) math.Quaternionf {
    if (look) |q| {
        const target = bc.normalizedQuat(q) catch unreachable;
        ctx.explicit_q = target;
        return target;
    }
    if (ctx.explicit_q != null) {
        ctx.explicit_q = null;
        ctx.held_q = null;
        ctx.clearOdometryAttitude();
    }
    if (ctx.held_q == null) ctx.held_q = current;
    return ctx.held_q.?;
}

/// Track `ctx.command` and return the thruster values. Invalid pose axes contribute no error.
fn controlStep(ctx: *MissionContext, thruster_buffer: *[math.max_thrusters]f32) []f32 {
    const timestamp_ns = ctx.frame.timestamp;
    const dt: ?f32 = if (ctx.pid_prev_timestamp_ns) |previous| blk: {
        const elapsed_ns = timestamp_ns - previous;
        ctx.pid_prev_timestamp_ns = timestamp_ns;
        break :blk @as(f32, @floatFromInt(elapsed_ns)) * 1e-9;
    } else blk: {
        ctx.pid_prev_timestamp_ns = timestamp_ns;
        break :blk null;
    };

    const thrust_direction = bc.ThrustDirections.fromTamLinearForces(ctx.config.tam) catch unreachable;
    const current = ctx.frame.camera_pose.quat;
    const command = ctx.command;
    var target_q = if (command.body_error) |err| blk: {
        const rot: math.Vector3f = .{ err[3], err[4], err[5] };
        const delta = quaternionFromRotationVector(rot);
        const current_q = bc.normalizedQuat(current) catch bc.identity;
        break :blk bc.normalizedQuat(math.quaternionMul(current_q, delta)) catch current_q;
    } else ctx.resolveLook(current, command.target_orientation);
    const odom = ctx.config.odometry;
    var desired_tilt: math.Vector3f = .{ 0, 0, 0 };
    var force_error: math.Vector3f = .{ 0, 0, 0 };
    const moments = bc.ThrustDirections.fromTamMoments(ctx.config.tam) catch unreachable;
    // Coverage swings a thruster onto a force component the map cannot produce.
    const translation_command: math.Vector3f = blk: {
        const error_body: math.Vector3f = if (command.body_error) |err|
            .{ err[0], err[1], err[2] }
        else
            command.translation;
        force_error = error_body;
        const available = thrust_direction.alongThrust(error_body);
        var force_body: math.Vector3f = .{ 0, 0, 0 };
        inline for (0..3) |i| {
            var rate: f32 = 0;
            if (dt) |t| {
                ctx.pid_sum_err[i] += available[i] * t;
                rate = (error_body[i] - ctx.pid_prev_err[i]) / t;
            }
            force_body[i] = odom.kp[i] * error_body[i] + odom.ki[i] * ctx.pid_sum_err[i] + odom.kd[i] * rate;
            ctx.pid_prev_err[i] = error_body[i];
        }
        const coverage = bc.coverForce(thrust_direction, moments, force_body, ctx.config.rotation_limit, ctx.config.camera_forward);
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
            var rate: f32 = 0;
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
    ctx.held_q = target_q;
    const input: math.Vector6f = .{
        translation_body[0], translation_body[1], translation_body[2],
        rotation[0],         rotation[1],         rotation[2],
    };
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        if (clear_print_stuff) {
            std.debug.print("\x1b[2J\x1b[H", .{});
        }
        std.log.debug("control:", .{});
        math.debug6f("\tkp", odom.kp);
        math.debug6f("\tki", odom.ki);
        math.debug6f("\tkd", odom.kd);
        for (ctx.config.tam) |row| {
            math.debug6f("\ttam", row);
        }
        std.log.debug("dt = {d:6.2} ms", .{(dt orelse 0) * 1000});
        std.log.debug("timestamp_ns = {d}", .{timestamp_ns});
        math.debug6f("pose", math.poseTo6f(ctx.frame.camera_pose));
        std.log.debug("position ok: {any} attitude ok: {any}", .{ ctx.frame.position_ok, ctx.frame.attitude_ok });
        math.debug3f("translation error", command.translation);
        if (command.target_orientation) |orient_q| {
            std.log.debug("target orientation: {any}", .{orient_q});
        }
        std.log.debug("attitude error body: {any}", .{attitude_error});
        math.debug3f("rotation limit", ctx.config.rotation_limit);
        std.log.debug("target quaternion: {any}", .{target_q});
        math.debug6f("\tinput", input);
    }
    const thruster_values = math.tamToThrusters(
        ctx.config.tam,
        input,
        ctx.thrustor_saturate,
        5.0,
        thruster_buffer,
    );
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    return thruster_values;
}

/// Search while holding still. Returns the first object of `cls`.
pub fn yieldFind(ctx: *MissionContext, cls: u8) !*const Auv.Object {
    ctx.motion = .{ .position = ctx.target_position, .orientation = ctx.target_orientation };
    var seen: usize = 0;

    while (true) {
        for (ctx.seen_objects[seen..ctx.seen_objects_len]) |*object| {
            if (object.cls == cls) return object;
            seen += 1;
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

const object_clearance: f32 = 2;
const look_done: f32 = 0.1;

fn reached(ctx: *const MissionContext, target: math.Vector3f) bool {
    const ok = ctx.frame.position_ok;
    if (!ok.x or !ok.y or !ok.z) return false;
    return math.length3f(target - ctx.frame.camera_pose.pos) <= ctx.goal_dist_threshold;
}

/// Horizontal unit direction from the vehicle toward the object. World +Y when they overlap.
fn approachDirection(ctx: *const MissionContext, object: *const Auv.Object) math.Vector2f {
    const delta = math.xy(object.bbox.pose.pos) - math.xy(ctx.frame.camera_pose.pos);
    if (math.length2f(delta) <= 1e-4) return .{ 0, 1 };
    return math.normalize2f(delta);
}

/// Half the oriented box, measured along a horizontal unit direction.
fn horizontalHalfExtent(object: *const Auv.Object, direction: math.Vector2f) f32 {
    const rot = math.normalize4f(object.bbox.pose.quat);
    const world_dir: math.Vector3f = .{ direction[0], direction[1], 0 };
    const size = object.bbox.size;
    const axes = [3]math.Vector3f{
        math.quaternionRotate(rot, .{ 1, 0, 0 }),
        math.quaternionRotate(rot, .{ 0, 1, 0 }),
        math.quaternionRotate(rot, .{ 0, 0, 1 }),
    };
    var extent: f32 = 0;
    inline for (axes, 0..) |axis, i| extent += @abs(math.dot3f(world_dir, axis)) * size[i];
    return 0.5 * extent;
}

/// `sign` is -1 on the approach side and +1 on the far side. `direction` stays fixed for the maneuver.
fn standoffAlong(object: *const Auv.Object, direction: math.Vector2f, sign: f32) math.Vector3f {
    const reach = horizontalHalfExtent(object, direction) + object_clearance;
    const offset = direction * @as(math.Vector2f, @splat(sign * reach));
    const center = object.bbox.pose.pos;
    return .{ center[0] + offset[0], center[1] + offset[1], center[2] };
}

fn yieldStandoff(ctx: *MissionContext, object: *const Auv.Object, sign: f32) !void {
    const direction = approachDirection(ctx, object);
    while (true) {
        const place = standoffAlong(object, direction, sign);
        try ctx.setLookAtTarget(standoffAlong(object, direction, 1));
        ctx.motion = .{ .position = place, .orientation = ctx.target_orientation };
        try ctx.yieldUntilNextFrameAndUpdate();
        if (reached(ctx, standoffAlong(object, direction, sign))) return;
    }
}

/// Face `object` and hold position. Returns when the attitude error is small.
pub fn yieldLookAt(ctx: *MissionContext, object: *const Auv.Object) !void {
    const hold = ctx.frame.camera_pose.pos;
    while (true) {
        try ctx.setLookAtTarget(object.bbox.pose.pos);
        ctx.motion = .{ .position = hold, .orientation = ctx.target_orientation };
        try ctx.yieldUntilNextFrameAndUpdate();
        const err = attitudeToward(ctx, ctx.target_orientation) orelse continue;
        if (math.length3f(err) <= look_done) return;
    }
}

/// Drive to the near side of `object`, looking at it.
pub fn yieldMoveTo(ctx: *MissionContext, object: *const Auv.Object) !void {
    try yieldStandoff(ctx, object, -1);
}

/// Stop short of `object`, then continue the same distance past it.
pub fn yieldPass(ctx: *MissionContext, object: *const Auv.Object) !void {
    try yieldMoveTo(ctx, object);
    try yieldStandoff(ctx, object, 1);
}

const circle_corners: [4]math.Vector2f = .{
    .{ 0.5, 0.5 },
    .{ 0.5, -0.5 },
    .{ -0.5, -0.5 },
    .{ -0.5, 0.5 },
};

fn circleCorner(object: *const Auv.Object, index: usize, depth: f32) math.Vector3f {
    const local_2d = circle_corners[index] * math.xy(object.bbox.size);
    const rotated = math.xy(math.quaternionRotate(math.normalize4f(object.bbox.pose.quat), .{ local_2d[0], local_2d[1], 0 }));
    const outward = if (math.length2f(rotated) <= 1e-4) math.Vector2f{ 1, 0 } else math.normalize2f(rotated);
    const pos = math.xy(object.bbox.pose.pos) + rotated + outward * @as(math.Vector2f, @splat(object_clearance));
    return .{ pos[0], pos[1], depth };
}

fn nearestCorner(ctx: *const MissionContext, object: *const Auv.Object, depth: f32) usize {
    var best_i: usize = 0;
    var best = std.math.inf(f32);
    const sub = math.xy(ctx.frame.camera_pose.pos);
    for (0..circle_corners.len) |i| {
        const dist = math.length2f(math.xy(circleCorner(object, i, depth)) - sub);
        if (dist < best) {
            best = dist;
            best_i = i;
        }
    }
    return best_i;
}

fn yieldPlaceLookingAt(ctx: *MissionContext, place: math.Vector3f, object: *const Auv.Object) !void {
    while (true) {
        try ctx.setLookAtTarget(object.bbox.pose.pos);
        ctx.motion = .{ .position = place, .orientation = ctx.target_orientation };
        try ctx.yieldUntilNextFrameAndUpdate();
        if (reached(ctx, place)) return;
    }
}

/// Visit the four outside corners of `object`, then return to the start.
pub fn yieldCircle(ctx: *MissionContext, object: *const Auv.Object) !void {
    const start = ctx.frame.camera_pose.pos;
    const first = nearestCorner(ctx, object, start[2]);
    for (0..circle_corners.len) |step| {
        const index = (first + step) % circle_corners.len;
        while (true) {
            const place = circleCorner(object, index, start[2]);
            try ctx.setLookAtTarget(object.bbox.pose.pos);
            ctx.motion = .{ .position = place, .orientation = ctx.target_orientation };
            try ctx.yieldUntilNextFrameAndUpdate();
            if (reached(ctx, circleCorner(object, index, start[2]))) break;
        }
    }
    try yieldPlaceLookingAt(ctx, start, object);
}
