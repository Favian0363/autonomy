//! Test mission: drive through a random course of waypoints, in order, and measure how each went.
//! The course comes from a seed, so the same seed always gives the same course (and, in the
//! simulator, the same run). Each leg goes in a random direction, so waypoints can be behind the
//! robot or need a sharp turn, and each waypoint changes depth. Waypoints are placed
//! relative to where the robot starts, inside an area that keeps clear of the scenario's gate and
//! pole.
//!
//! For each waypoint the robot first has to get within `goal_dist_threshold` (1 m) before the
//! timeout ("reached"), then keep the goal for a few more seconds while we measure whether it
//! overshoots and where it ends up; it must stay within the threshold all that time ("held").
//! All times are frame time.
//!
//! env: AUV_WAYPOINTS_SEED       course seed (default 1)
//!      AUV_WAYPOINTS_COUNT      number of waypoints (default 8)
//!      AUV_WAYPOINTS_TIMEOUT_S  time allowed to reach each waypoint (default 60)
//!      AUV_WAYPOINTS_HOLD_S     time to keep each waypoint after reaching it (default 5)
//!
//! Prints one `waypoint,...` line per waypoint and a `waypoints,...` summary, and fails (exit
//! code 1) if any waypoint was not reached or not held, or if the pose became absurd.
const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");

const max_waypoints = 256;

// Course shape, in meters from the start. Legs are 1.5-6 m long. Waypoints stay within 8 m in x,
// and from 11 m toward -y to 6 m toward +y (in hydrus_env.scn the gate is 9 m away in +y), and
// from 1 m above to 0.5 m below the start depth (the floor is 1 m below it).
const leg_m = [2]f32{ 1.5, 6.0 };
const area_x_m = [2]f32{ -8.0, 8.0 };
const area_y_m = [2]f32{ -11.0, 6.0 };
const depth_m = [2]f32{ -1.0, 0.5 };

// Farther than this from the goal means the simulation itself broke down (unstable control can
// make the simulator blow up), so the run stops instead of reporting kilometers.
const absurd_dist_m: f32 = 100.0;

const Result = struct {
    reached: bool = false,
    time_to_reach_s: f32 = 0,
    /// closest the camera got to the goal
    min_dist_m: f32 = std.math.inf(f32),
    held: bool = false,
    /// farthest it moved back away from the goal after having been closer, while holding: 0 for a
    /// smooth stop, 0.4 if it passed the goal and swung 40 cm past (or drifted 40 cm away)
    overshoot_m: f32 = 0,
    /// farthest from the goal while holding; above the threshold means it did not hold
    max_dist_holding_m: f32 = 0,
    /// distance at the end of the hold time
    final_dist_m: f32 = 0,
    /// largest tilt seen while going to and holding this waypoint, in degrees
    max_roll_deg: f32 = 0,
    max_pitch_deg: f32 = 0,
};

pub fn mission(ctx: *MissionContext) !void {
    const seed = try envInt(u64, "AUV_WAYPOINTS_SEED", 1);
    const count = try envInt(u32, "AUV_WAYPOINTS_COUNT", 8);
    var course_buf: [max_waypoints]math.Vector3f = undefined;
    const course = try makeCourse(seed, count, ctx.frame.camera_pose.pos, &course_buf);
    const timeout_s = try envInt(u32, "AUV_WAYPOINTS_TIMEOUT_S", 60);
    const hold_s = try envInt(u32, "AUV_WAYPOINTS_HOLD_S", 5);

    var reached: u32 = 0;
    var held: u32 = 0;
    for (course, 0..) |goal, i| {
        const r = try driveTo(ctx, goal, @as(u64, timeout_s) * std.time.ns_per_s, @as(u64, hold_s) * std.time.ns_per_s);
        if (r.reached) reached += 1;
        if (r.held) held += 1;
        std.log.info("waypoint,seed={d},index={d},reached={},time_s={d:.3},min_dist_m={d:.3},held={},overshoot_m={d:.3},final_dist_m={d:.3},max_roll_deg={d:.1},max_pitch_deg={d:.1},goal={d:.3},{d:.3},{d:.3}", .{
            seed,          i,              r.reached,      r.time_to_reach_s, r.min_dist_m, r.held,
            r.overshoot_m, r.final_dist_m, r.max_roll_deg, r.max_pitch_deg,   goal[0],      goal[1],
            goal[2],
        });
    }

    std.log.info("waypoints,seed={d},reached={d}/{d},held={d}/{d}", .{ seed, reached, count, held, count });
    if (reached != count or held != count) return error.WaypointsMissed;
}

/// Tells the controller where to go. The only place this mission touches the controller, so it is
/// the only thing to change when the controller's interface changes.
fn setGoal(ctx: *MissionContext, goal: math.Vector3f) void {
    ctx.goal[0] = goal[0];
    ctx.goal[1] = goal[1];
    ctx.goal[2] = goal[2];
}

/// Sets the goal and runs frames until the waypoint is reached and held, or the timeout passes.
fn driveTo(ctx: *MissionContext, goal: math.Vector3f, timeout_ns: u64, hold_ns: u64) !Result {
    setGoal(ctx, goal);
    ctx.step = "waypoint";

    var r: Result = .{};
    const start_ns = ctx.frame.timestamp;
    var reached_ns: u64 = 0;
    while (true) {
        const pose = math.poseTo6f(ctx.frame.camera_pose);
        const delta = goal - ctx.frame.camera_pose.pos;
        const dist = @sqrt(@reduce(.Add, delta * delta));
        if (!(dist < absurd_dist_m)) {
            // also catches NaN, which fails every comparison
            std.log.err("pose is {d:.1} m from the goal: the simulation diverged", .{dist});
            return error.SimulationDiverged;
        }
        r.min_dist_m = @min(r.min_dist_m, dist);
        r.max_roll_deg = @max(r.max_roll_deg, @abs(std.math.radiansToDegrees(pose[3])));
        r.max_pitch_deg = @max(r.max_pitch_deg, @abs(std.math.radiansToDegrees(pose[4])));

        const now = ctx.frame.timestamp;
        if (!r.reached) {
            if (dist <= ctx.goal_dist_threshold) {
                r.reached = true;
                r.time_to_reach_s = seconds(now -| start_ns);
                reached_ns = now;
            } else if (now -| start_ns >= timeout_ns) {
                r.time_to_reach_s = seconds(now -| start_ns);
                return r;
            }
        } else {
            r.max_dist_holding_m = @max(r.max_dist_holding_m, dist);
            r.overshoot_m = @max(r.overshoot_m, dist - r.min_dist_m);
            if (now -| reached_ns >= hold_ns) {
                r.final_dist_m = dist;
                r.held = r.max_dist_holding_m <= ctx.goal_dist_threshold;
                return r;
            }
        }

        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

fn seconds(ns: u64) f32 {
    return @as(f32, @floatFromInt(ns)) * 1e-9;
}

/// Fills `buf` with a course of `count` waypoints from `seed`, starting at `start`.
fn makeCourse(seed: u64, count: u32, start: math.Vector3f, buf: []math.Vector3f) ![]math.Vector3f {
    if (count == 0 or count > buf.len) {
        std.log.err("AUV_WAYPOINTS_COUNT must be between 1 and {d}, got {d}", .{ buf.len, count });
        return error.InvalidEnv;
    }
    var prng = std.Random.DefaultPrng.init(seed);
    const random = prng.random();
    var x: f32 = 0; // offsets from the start
    var y: f32 = 0;
    for (buf[0..count]) |*waypoint| {
        // try random legs until one ends inside the area; the area is large enough that this
        // takes a few tries at most
        var tries: u32 = 0;
        while (true) : (tries += 1) {
            if (tries == 1000) return error.CourseDoesNotFitArea;
            const direction = uniform(random, .{ -std.math.pi, std.math.pi });
            const length = uniform(random, leg_m);
            const nx = x + length * @cos(direction);
            const ny = y + length * @sin(direction);
            if (nx >= area_x_m[0] and nx <= area_x_m[1] and ny >= area_y_m[0] and ny <= area_y_m[1]) {
                x, y = .{ nx, ny };
                break;
            }
        }
        waypoint.* = start + math.Vector3f{ x, y, uniform(random, depth_m) };
    }
    return buf[0..count];
}

fn uniform(random: std.Random, range: [2]f32) f32 {
    return range[0] + random.float(f32) * (range[1] - range[0]);
}

/// Integer from the environment, or `default` if unset.
fn envInt(comptime T: type, name: [:0]const u8, default: T) !T {
    const text = std.c.getenv(name) orelse return default;
    return std.fmt.parseInt(T, std.mem.span(text), 10) catch {
        std.log.err("{s} must be a nonnegative integer, got '{s}'", .{ name, text });
        return error.InvalidEnv;
    };
}
