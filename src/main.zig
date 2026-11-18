const std = @import("std");
const MissionConfig = @import("MissionConfig.zig");
const MissionIO = @import("MissionIO.zig");
const MissionContext = @import("MissionContext.zig");

test {
    std.testing.refAllDecls(@import("BodyControl.zig"));
    std.testing.refAllDecls(@import("Serial.zig"));
    std.testing.refAllDecls(@import("Simulator.zig"));
    std.testing.refAllDecls(@import("Protocol.zig"));
    std.testing.refAllDecls(MissionContext);
}

pub fn main(init: std.process.Init) !void {
    const process_allocator = init.arena.allocator();
    var context: MissionContext = undefined;
    context.config = MissionConfig.startup_config;
    context.config.sensor_source = if (@import("Simulator.zig").enabled) .simulated else .plugin;
    // Keep the listener available even if a configured telemetry output fails.
    var bootstrap = context.config;
    bootstrap.telemetry_transport = .stdout;
    bootstrap.telemetry_log_path = .{};
    var telemetry: MissionIO.Telemetry = try .init(process_allocator, init.io, bootstrap);
    defer telemetry.deinit(init.io);
    telemetry.reconfigure(init.io, context.config) catch |err| {
        telemetry.report(err);
        telemetry.telemetry_error = telemetry.last_error;
    };

    // Reuse mission storage across restarts without resetting the process arena.
    var mission_arena = std.heap.ArenaAllocator.init(process_allocator);
    defer mission_arena.deinit();
    var run_mission = true;
    while (true) {
        if (run_mission) {
            run_mission = false;
            defer _ = mission_arena.reset(.retain_capacity);
            runMission(mission_arena.allocator(), init.io, &telemetry, &context) catch |err| switch (err) {
                error.MissionRestart => run_mission = true,
                error.MissionStopped => telemetry.result = .stopped,
                else => {
                    telemetry.result = .failed;
                    telemetry.report(err);
                },
            };
        } else {
            telemetry.pollControl(init.io, &context.config) catch |err| switch (err) {
                error.MissionRestart => run_mission = true,
                error.MissionStopped => telemetry.result = .stopped,
            };
            if (!run_mission) try std.Io.sleep(init.io, .fromMilliseconds(10), .awake);
        }
    }
}

fn runMission(allocator: std.mem.Allocator, io: std.Io, telemetry: *MissionIO.Telemetry, context: *MissionContext) !void {
    std.Io.random(io, std.mem.asBytes(&telemetry.run_id));
    telemetry.run_id |= 1;
    telemetry.state = .starting;
    telemetry.result = .none;
    telemetry.last_error = .{};
    telemetry.clearCommands();
    defer {
        telemetry.state = .idle;
        telemetry.clearCommands();
    }
    try context.init(allocator, io, telemetry);
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
