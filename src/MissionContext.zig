const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;

io: Io,

frame: Auv.Frame,

seen_objects: [Auv.Frame.max_objects]Auv.Object,
seen_objects_len: u8,

goal_dist_threshold: f32,
close_enough: f32,
thrustor_saturate: f32,

log_counter: u32,
log_freq_div: u16,

goal: math.Vector6f,

pid_sum_err: math.Vector6f,
pid_prev_pose_err: math.Vector6f,
pid_prev_timestamp_ns: ?u64,

auv: Auv,
auv_loader: AuvLoader,
auv_watcher: FileWatcher,

config: ConfigLoader.Config,
config_loader: ConfigLoader,
config_watcher: FileWatcher,

pub fn init(gpa: std.mem.Allocator, io: Io, args: MissionArgs) !MissionContext {
    var config_loader: ConfigLoader = try .init(gpa, args.live_config_path);
    const config = config_loader.load(io) catch |err| {
        std.log.err("failed to load config: {s}", .{config_loader.config_path});
        return err;
    };
    std.log.debug("succesfully loaded config: {s}", .{config_loader.config_path});
    std.log.debug("{any}", .{config});

    var auv_loader: AuvLoader = try .init(gpa, args.auv_dynlib_path);
    const auv = auv_loader.load(io) catch |err| {
        std.log.err("failed to load auv: {s}", .{auv_loader.auv_path});
        return err;
    };
    auv.init();
    std.log.debug("succesfully loaded auv: {s}", .{auv_loader.auv_path});
    std.log.debug("{any}", .{auv});

    return .{
        .frame = undefined,

        .seen_objects = undefined,
        .seen_objects_len = 0,

        .goal_dist_threshold = args.goal_dist_threshold,
        .close_enough = 1,
        .thrustor_saturate = 5,

        .log_counter = 0,
        .log_freq_div = 120,

        .goal = @splat(0),

        .pid_sum_err = @splat(0),
        .pid_prev_pose_err = @splat(0),
        .pid_prev_timestamp_ns = null,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(gpa, args.auv_dynlib_path),

        .config = config,
        .config_loader = config_loader,
        .config_watcher = try .init(gpa, args.live_config_path),

        .io = io,
    };
}

const linux = std.os.linux;
pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) void {
    var start: linux.timespec = undefined;
    var end: linux.timespec = undefined;

    ctx.auv.yieldUntilNextFrame(&ctx.frame);
    _ = linux.clock_gettime(.MONOTONIC, &start);

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

    // std.log.debug("setting thruster_values with pid controller...", .{});
    ctx.pidStep();

    if (ctx.config_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        if (ctx.config_loader.load(ctx.io)) |config| {
            std.log.debug("succesfully reloaded config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.debug("{any}", .{config});
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
            std.log.debug("succesfully reloaded auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.debug("{any}", .{auv});
            ctx.auv = auv;
            std.log.debug("restarted auv loop", .{});
        } else |err| {
            std.log.err("failed to reload auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous auv", .{});
        }
        ctx.auv.init();
    }

    _ = linux.clock_gettime(.MONOTONIC, &end);
    const ns = (end.sec - start.sec) * 1000000000 + (end.nsec - start.nsec);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{@divTrunc(ns, 1000)});
    }
    ctx.log_counter +%= 1;
}

const math = @import("math.zig");

fn pidStep(ctx: *MissionContext) void {
    const pose = ctx.frame.camera_pose;

    const timestamp_ns = ctx.frame.timestamp;

    const dt: ?f32 = if (ctx.pid_prev_timestamp_ns) |previous| blk: {
        const elapsed_ns = timestamp_ns - previous;

        if (elapsed_ns > 0) {
            break :blk @as(f32, @floatFromInt(elapsed_ns)) * 1e-9;
        }

        std.log.warn(
            "odometry stamp not increasing (prev={} ns, now={} ns); skipping I/D",
            .{ previous, timestamp_ns },
        );

        break :blk null;
    } else null;

    ctx.pid_prev_timestamp_ns = timestamp_ns;

    const goal = ctx.goal;
    const current_pose = math.poseTo6f(pose);

    var pose_err = goal - current_pose;

    // Current orientation.
    const rot = math.normalize4f(pose.quat);

    // Vehicle forward is +Y in body frame.
    const forward = math.quaternionRotate(rot, .{ 0.0, 1.0, 0.0 });
    const current_yaw = std.math.atan2(forward[1], forward[0]);

    const dir = math.Vector3f{
        pose_err[0],
        pose_err[1],
        0.0,
    };

    const xy_distance = math.length3f(dir);

    // const yaw_error = if (distance > ctx.close_enough) blk: {
    //     const target_yaw = std.math.atan2(dir[1], dir[0]);
    //     break :blk math.wrapAngle(target_yaw - current_yaw);
    // } else blk: {
    //     break :blk math.wrapAngle(goal[5] - current_yaw);
    // };
    const target_yaw = if (xy_distance > ctx.close_enough)
        std.math.atan2(dir[1], dir[0])
    else
        goal[5];

    const yaw_error = math.wrapAngle(target_yaw - current_yaw);

    pose_err[5] = yaw_error;

    // const dir: math.Vector3f = .{ pose_err[0], pose_err[1], 0.0 };
    //
    // const yaw_error = if (math.length3f(dir) > ctx.close_enough) blk: {
    //     const dir_normalized = math.normalize3f(dir);
    //
    //     // Rotation from current forward -> target direction.
    //     const yaw_quat = math.rotationBetween(forward, dir_normalized);
    //
    //     _, _, const yaw_quat_yaw = math.quaternionToEuler(yaw_quat);
    //
    //     break :blk yaw_quat_yaw;
    // } else math.wrapAngle(goal[5] - current_yaw);
    //
    // pose_err[5] = yaw_error;

    const vel_err: math.Vector6f = if (dt) |delta_t| blk: {
        ctx.pid_sum_err += pose_err * @as(math.Vector6f, @splat(delta_t));

        break :blk (pose_err - ctx.pid_prev_pose_err) / @as(math.Vector6f, @splat(delta_t));
    } else @splat(0.0);

    const kp = ctx.config.kp;
    const ki = ctx.config.ki;
    const kd = ctx.config.kd;

    const wrench =
        kp * pose_err +
        ki * ctx.pid_sum_err +
        kd * vel_err;

    // // Rotate world-frame XYZ wrench into body frame.
    // const rotated = math.quaternionRotate(
    //     math.quaternionConjugate(rot),
    //     .{ wrench[0], wrench[1], wrench[2] },
    // );
    //
    // var body_force = rotated;
    //
    // // Only positive X/Y.
    // body_force[0] = @max(body_force[0], 0.0);
    // body_force[1] = @max(body_force[1], 0.0);
    //
    // // Only move in XY when approximately facing the goal.
    // if (@abs(yaw_error) > std.math.pi / 8.0) {
    //     body_force[0] = 0.0;
    //     body_force[1] = 0.0;
    // }

    const rotated = math.quaternionRotate(
        math.quaternionConjugate(rot),
        .{ wrench[0], wrench[1], wrench[2] },
    );

    var body_force = rotated;

    if (@abs(yaw_error) > std.math.pi / 8.0) {
        body_force[0] = 0.0;
        body_force[1] = 0.0;
    } else {
        // Hydrus TAM has no body-X (sway) column; only +Y is surge.
        body_force[0] = 0.0;
        body_force[1] = @max(body_force[1], 0.0);
    }

    const input: math.Vector6f = .{
        body_force[0],
        body_force[1],
        wrench[2],
        -wrench[3],
        wrench[4],
        wrench[5],
    };

    const max_thrusters = 8;
    var thruster_buffer: [max_thrusters]f32 = undefined;
    const thruster_values = math.tamMul(ctx.config.tam, input, &thruster_buffer);

    for (thruster_values) |*value| {
        value.* = std.math.clamp(
            value.*,
            -ctx.thrustor_saturate,
            ctx.thrustor_saturate,
        );

        value.* /= 5.0;
    }

    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        if (clear_print_stuff) {
            std.debug.print("\x1b[2J\x1b[H", .{});
        }
        std.log.debug("config:", .{});
        std.log.debug("\tclose_enough = {}", .{ctx.close_enough});
        math.debug6f("\tkp", kp);
        math.debug6f("\tki", ki);
        math.debug6f("\tkd", kd);
        for (ctx.config.tam) |row| {
            math.debug6f("\ttam", row);
        }
        std.log.debug("dt = {d:6.2} ms", .{(dt orelse 0) * 1000});
        math.debug6f("pose", current_pose);
        math.debug6f("goal", goal);
        std.log.debug("xy_distance = {}", .{xy_distance});
        std.log.debug("thruster mapping:", .{});
        math.debug6f("\twrench  ", wrench);
        math.debug3f("\tforward ", forward);
        math.debug3f("\tdir2d   ", dir);
        std.log.debug(
            "\tcurrent_yaw={d:3.2} target_yaw={d:3.2} yaw_error={d:3.2}",
            .{ current_yaw, target_yaw, yaw_error },
        );
        math.debug3f("\trotated ", rotated);
        math.debug3f("\tbody_force ", body_force);
        math.debug6f("\tinput   ", input);
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));

    ctx.pid_prev_pose_err = pose_err;
}

fn yieldUntilObjectWithCls(ctx: *MissionContext, clss: []const Auv.ObjectCls, start: usize) *const Auv.Object {
    var seen = start;

    while (true) {
        for (ctx.seen_objects[seen..ctx.seen_objects_len]) |*reacted_object| {
            for (clss) |cls| {
                if (reacted_object.cls == cls) {
                    return reacted_object;
                }
            }
            seen += 1;
        }

        ctx.yieldUntilNextFrameAndUpdate();
    }
}

/// yield until getting first object with any of the `cls` in `clss`
pub fn yieldUntilFirstObjectWithAnyCls(ctx: *MissionContext, clss: []const Auv.ObjectCls) *const Auv.Object {
    return yieldUntilObjectWithCls(ctx, clss, 0);
}

/// yield until getting first object wit `cls`
pub fn yieldUntilFirstObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return yieldUntilFirstObjectWithAnyCls(ctx, &.{cls});
}

/// yield until getting next object with any of the `cls` in `clss` (ignoring any seen before)
pub fn yieldUntilNewObjectWithAnyCls(ctx: *MissionContext, clss: []const Auv.ObjectCls) *const Auv.Object {
    return yieldUntilObjectWithCls(ctx, clss, ctx.seen_objects_len);
}

/// yield until getting next object wit `cls` (ignoring any seen before)
pub fn yieldUntilNewObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return yieldUntilNewObjectWithAnyCls(ctx, &.{cls});
}

/// yield until at `ctx.frame.camera_pose.pos` is at `goal_threshold` distance from `goal_pos`
pub fn yieldUntilReachGoal(ctx: *MissionContext, goal_pos: math.Vector3f) void {
    ctx.goal[0] = goal_pos[0];
    ctx.goal[1] = goal_pos[1];
    ctx.goal[2] = goal_pos[2];
    while (true) {
        const camera_pos = ctx.frame.camera_pose.pos;
        const goal_delta = goal_pos - camera_pos;
        const goal_dist = @sqrt(@reduce(.Add, goal_delta * goal_delta));
        if (goal_dist <= ctx.goal_dist_threshold) {
            break;
        }

        ctx.yieldUntilNextFrameAndUpdate();
    }
}
