const std = @import("std");
const MissionContext = @import("MissionContext.zig");

const simulator_enabled = @import("build_options").simulator;

test {
    std.testing.refAllDecls(@import("BodyControl.zig"));
    std.testing.refAllDecls(@import("Serial.zig"));
    std.testing.refAllDecls(MissionContext);
}

pub fn main(init: std.process.Init) !void {
    const process_allocator = init.arena.allocator();
    const args = try init.minimal.args.toSlice(process_allocator);
    var windowed = simulator_enabled;
    for (args[1..]) |arg| {
        if (std.mem.eql(u8, arg, "--headless") and simulator_enabled) {
            windowed = false;
        } else {
            std.log.err("unknown argument: {s}", .{arg});
            return error.UnknownArgument;
        }
    }
    var context: MissionContext = undefined;
    try context.initProcess(process_allocator, init.io, windowed);
    defer context.deinitProcess();
    const telemetry = context.telemetry;

    // Reuse mission storage across restarts without resetting the process arena.
    var mission_arena = std.heap.ArenaAllocator.init(process_allocator);
    defer mission_arena.deinit();
    var run_mission = true;
    while (true) {
        if (run_mission) {
            run_mission = false;
            defer _ = mission_arena.reset(.retain_capacity);
            runMission(mission_arena.allocator(), &context) catch |err| switch (@as(anyerror, err)) {
                error.WindowClosed => return,
                error.MissionRestart => run_mission = true,
                error.MissionStopped => telemetry.result = .stopped,
                else => {
                    telemetry.result = .failed;
                    telemetry.report(err);
                },
            };
        } else {
            context.serviceWithoutFrame() catch |err| switch (@as(anyerror, err)) {
                error.WindowClosed => return,
                error.MissionRestart => run_mission = true,
                error.MissionStopped => telemetry.result = .stopped,
                else => return err,
            };
            if (!run_mission) try std.Io.sleep(init.io, .fromMilliseconds(10), .awake);
        }
    }
}

fn runMission(allocator: std.mem.Allocator, context: *MissionContext) !void {
    const telemetry = context.telemetry;
    std.Io.random(context.io, std.mem.asBytes(&telemetry.run_id));
    telemetry.run_id |= 1;
    telemetry.state = .starting;
    telemetry.result = .none;
    telemetry.last_error = .{};
    telemetry.clearCommands();
    defer {
        context.telemetry_frame = std.mem.zeroes(@TypeOf(context.telemetry_frame));
        telemetry.state = .idle;
        telemetry.clearCommands();
    }
    try context.init(allocator);
    defer context.deinit();

    try context.yieldUntilNextFrameAndUpdate();
    context.target_position = context.frame.camera_pose.pos;
    telemetry.state = .running;
    switch (context.config.mission_id) {
        .prequalify_coco => try @import("missions.zig").cocoMission(context),
        .prequalify_robosub => try @import("missions.zig").robosubMission(context),
        .teleop => try @import("missions.zig").teleopMission(context),
        .mission_control => try @import("missions.zig").missionControl(context),
        .thruster_control => try @import("missions.zig").thrusterControl(context),
    }
    telemetry.result = .completed;
}
