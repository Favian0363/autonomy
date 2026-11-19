const std = @import("std");
const rl = @import("raylib");
const ui = @import("visualizer/ui.zig");
const RuntimePanel = @import("visualizer/runtime_panel.zig");
pub const MissionIO = @import("MissionIO.zig");
const MissionConfig = MissionIO.CompiledConfig;
const math = MissionIO.Math;
const Self = @This();

const teleop_step: f32 = 0.05;
const teleop_period: f64 = 0.05;
const mission_target_step: f32 = 0.25;
const mission_look_step: f32 = 0.1;
const tune_step: f32 = 0.1;
const tune_period: f64 = 0.1;

const tune_names = [_][:0]const u8{
    "kp_x", "kp_y", "kp_z", "kp_roll", "kp_pitch", "kp_yaw",
    "ki_x", "ki_y", "ki_z", "ki_roll", "ki_pitch", "ki_yaw",
    "kd_x", "kd_y", "kd_z", "kd_roll", "kd_pitch", "kd_yaw",
};
const pid_slider_max = [_]f32{ 10, 2, 10 };

runtime_panel: RuntimePanel,
config_packet: MissionIO.LiveConfigPacket,
embedded: bool,
font: rl.Font,
owns_font: bool,
scene_w: i32 = 900,
scene_h: i32 = 470,
scene: rl.RenderTexture2D,
orbit_yaw: f32 = 0.8,
orbit_pitch: f32 = 0.55,
orbit_dist: f32 = 8,
translation: [3]f32 = @splat(0),
orientation: [4]f32 = .{ 0, 0, 0, 1 },
last_teleop: f64 = -1,
mission_target_initialized: bool = false,
mission_kind: MissionIO.MissionCommandKind = .point,
mission_maneuver: MissionIO.MissionManeuver = .move,
mission_position: math.Vector3f = @splat(0),
mission_look_direction: math.Vector3f = .{ 0, 1, 0 },
mission_look_yaw: f32 = 0,
mission_look_pitch: f32 = 0,
mission_object_class: u8 = 0,
mission_control: bool,
chosen_mission: MissionConfig.MissionId,
dropdown_open: bool = false,
workspace: ui.Workspace,
font_percent: u8 = ui.default_font_percent,
mission_sent: bool = false,
tune_sent: bool = false,
object_scroll: usize = 0,
goal_scroll: usize = 0,
tune_selection: usize = 0,
last_tune_adjust: f64 = -1,
pid_drag: ?usize = null,
direct_thruster_drag: ?usize = null,
last_thruster_adjust: f64 = -1,

pub const Status = ui.Status;

pub const Video = struct {
    enabled: bool = false,
    texture: rl.Texture2D = std.mem.zeroes(rl.Texture2D),
    width: c_int = 0,
    height: c_int = 0,
    status: Status = .disabled,
};

pub const Input = struct {
    status: ?MissionIO.RuntimeStatus = null,
    frame: ?*const MissionIO.TelemetryFrame = null,
    telemetry_status: Status = .connecting,
    video: Video = .{},
    send_error: bool = false,
};

pub const Actions = struct {
    config: ?MissionIO.LiveConfigPacket = null,
    lifecycle: ?MissionIO.RuntimeRequest = null,
    thrusters: ?MissionIO.ThrusterCommand = null,
    mission: ?MissionIO.MissionCommand = null,
    teleop: ?MissionIO.TeleopCommand = null,
};

pub fn init(config: *const MissionConfig.MissionConfig, client_id: u64, mission_control: bool, embedded: bool) !Self {
    rl.setConfigFlags(.{ .window_resizable = true });
    rl.initWindow(1280, 1000, if (embedded) "Autonomy Simulator" else "viewer");
    if (!rl.isWindowReady()) return error.WindowInitializationFailed;
    errdefer rl.closeWindow();

    rl.setWindowMinSize(1000, 700);
    if (!embedded) rl.setTargetFPS(60);
    const embedded_font = @embedFile("visualizer/assets/IBMPlexSans-Regular.ttf");
    const loaded_font = rl.loadFontFromMemory(".ttf", embedded_font, 32, null) catch null;
    errdefer if (loaded_font) |font| rl.unloadFont(font);
    const font = loaded_font orelse try rl.getFontDefault();
    if (loaded_font != null) rl.setTextureFilter(font.texture, .bilinear);

    return .{
        .runtime_panel = .{ .client_id = client_id },
        .config_packet = MissionIO.LiveConfigPacket.fromRuntimeConfig(config),
        .embedded = embedded,
        .font = font,
        .owns_font = loaded_font != null,
        .scene = try rl.loadRenderTexture(900, 470),
        .mission_control = mission_control,
        .chosen_mission = config.mission_id,
        .workspace = .runtime,
    };
}

pub fn deinit(self: *Self) void {
    rl.unloadRenderTexture(self.scene);
    if (self.owns_font) rl.unloadFont(self.font);
    rl.closeWindow();
}

pub fn shouldClose() bool {
    return rl.windowShouldClose();
}

pub fn acceptStatus(self: *Self, status: MissionIO.RuntimeStatus) void {
    const previous = self.runtime_panel.status;
    if (previous == null or previous.?.run_id != status.run_id or (previous.?.active and !status.active)) {
        self.translation = @splat(0);
        self.orientation = .{ 0, 0, 0, 1 };
        self.mission_target_initialized = false;
        self.mission_look_yaw = 0;
        self.mission_look_pitch = 0;
        self.mission_sent = false;
        self.last_teleop = -1;
    }
    if (previous == null or previous.?.revision != status.revision) {
        if (!self.dropdown_open) self.chosen_mission = status.desired.mission_id;
        self.mission_control = status.desired.mission_id == .mission_control;
    }
    self.runtime_panel.acceptStatus(status, &self.config_packet);
}

pub fn runtimeStatus(self: *const Self) ?MissionIO.RuntimeStatus {
    return self.runtime_panel.status;
}

pub fn neutral(self: *const Self) ?MissionIO.TeleopCommand {
    if (self.mission_control) return null;
    return .{ .translation = @splat(0), .orientation = self.orientation };
}

pub fn tick(self: *Self, input: Input) !Actions {
    if (input.status) |value| self.acceptStatus(value);
    const now = rl.getTime();
    const empty = emptyTelemetry();
    const telemetry = input.frame orelse &empty;
    const telemetry_live = input.telemetry_status == .live;
    const running = if (self.runtime_panel.status) |value|
        self.runtime_panel.synchronized() and value.state == .running
    else
        false;
    const read_video = input.video.enabled;
    var actions: Actions = .{};
    const layout = ui.Layout.init(@floatFromInt(rl.getScreenWidth()), @floatFromInt(rl.getScreenHeight()), read_video);
    if (rl.isKeyPressed(.equal) or rl.isKeyPressed(.kp_add)) self.font_percent = ui.adjustFontPercent(self.font_percent, 5);
    if (rl.isKeyPressed(.minus) or rl.isKeyPressed(.kp_subtract)) self.font_percent = ui.adjustFontPercent(self.font_percent, -5);
    if (rl.isKeyPressed(.zero)) self.font_percent = ui.default_font_percent;
    if (self.runtime_panel.editing == null and rl.isKeyPressed(.one)) self.workspace = .runtime;
    if (self.runtime_panel.editing == null and rl.isKeyPressed(.two)) self.workspace = .settings;
    const menu_was_open = self.dropdown_open;
    if (rl.isMouseButtonPressed(.left)) {
        const mouse = rl.getMousePosition();
        if (rl.checkCollisionPointRec(mouse, layout.fontButton(false))) self.font_percent = ui.adjustFontPercent(self.font_percent, -5);
        if (rl.checkCollisionPointRec(mouse, layout.fontButton(true))) self.font_percent = ui.adjustFontPercent(self.font_percent, 5);
        for (0..2) |i| if (rl.checkCollisionPointRec(rl.getMousePosition(), layout.navRect(i))) {
            self.workspace = @fromBackingInt(@intCast(i));
            self.pid_drag = null;
            break;
        };
        if (self.workspace == .runtime) {
            if (self.dropdown_open) {
                for (0..@typeInfo(MissionConfig.MissionId).@"enum".field_names.len) |i| {
                    if (rl.checkCollisionPointRec(mouse, layout.dropdownOption(i))) {
                        self.chosen_mission = @fromBackingInt(@as(u8, @intCast(i)));
                        break;
                    }
                }
                self.dropdown_open = false;
            } else if (rl.checkCollisionPointRec(mouse, layout.dropdownRect())) {
                self.dropdown_open = true;
            } else if (rl.checkCollisionPointRec(mouse, layout.activateRect()) and self.runtime_panel.synchronized()) {
                var request = self.config_packet;
                request.mission_id = self.chosen_mission;
                self.runtime_panel.queueConfig(request);
            }
        }
    }
    const view3d = layout.sceneContent(.runtime);
    const next_w: i32 = @max(1, @as(i32, @intFromFloat(view3d.width)));
    const next_h: i32 = @max(1, @as(i32, @intFromFloat(view3d.height)));
    if (next_w != self.scene_w or next_h != self.scene_h) {
        const replacement = try rl.loadRenderTexture(next_w, next_h);
        rl.unloadRenderTexture(self.scene);
        self.scene = replacement;
        self.scene_w = next_w;
        self.scene_h = next_h;
    }
    const list_capacity = ui.Layout.listCapacity(layout.objects);
    const object_max = @as(usize, telemetry.objects_len) -| list_capacity;
    const goal_max = @as(usize, telemetry.goals_len) -| list_capacity;
    self.object_scroll = @min(self.object_scroll, object_max);
    self.goal_scroll = @min(self.goal_scroll, goal_max);
    if (self.workspace == .runtime and rl.isWindowFocused()) {
        const wheel = rl.getMouseWheelMove();
        if (wheel != 0) {
            const mouse = rl.getMousePosition();
            if (rl.checkCollisionPointRec(mouse, layout.objects)) {
                self.object_scroll = if (wheel > 0) self.object_scroll -| 1 else @min(self.object_scroll + 1, object_max);
            } else if (rl.checkCollisionPointRec(mouse, layout.goals)) {
                self.goal_scroll = if (wheel > 0) self.goal_scroll -| 1 else @min(self.goal_scroll + 1, goal_max);
            }
        }
    }

    if (self.workspace == .runtime and rl.isKeyPressed(.t)) {
        // Release any held teleoperation translation before leaving teleop mode.
        if (!self.mission_control) {
            actions.teleop = .{ .translation = @splat(0), .orientation = self.orientation };
            self.translation = @splat(0);
        }
        self.mission_control = !self.mission_control;
        if (self.runtime_panel.synchronized()) {
            self.config_packet.mission_id = if (self.mission_control) .mission_control else .teleop;
            self.runtime_panel.queueConfig(self.config_packet);
        }
        if (self.mission_control) self.mission_target_initialized = false;
        std.debug.print(
            "control mode: {s}\n",
            .{if (self.mission_control) "mission-control" else "teleop"},
        );
    }
    const focused = rl.isWindowFocused() and self.workspace == .runtime and !self.dropdown_open and !menu_was_open;
    const mouse_click = focused and rl.isMouseButtonPressed(.left);
    if (mouse_click and self.mission_control) {
        if (rl.checkCollisionPointRec(rl.getMousePosition(), layout.missionButton(0))) self.mission_kind = .point;
        if (rl.checkCollisionPointRec(rl.getMousePosition(), layout.missionButton(1))) self.mission_kind = .object;
    }

    if (self.mission_control and !self.mission_target_initialized and telemetry_live) {
        self.mission_position = telemetry.camera_pose.pos + @as(math.Vector3f, .{ 0, 2, 0 });
        self.mission_target_initialized = true;
    }

    if (self.mission_control and focused) {
        if (rl.isKeyPressed(.p)) self.mission_kind = .point;
        if (rl.isKeyPressed(.o)) self.mission_kind = .object;
        if (rl.isKeyPressed(.m)) self.mission_maneuver = .move;
        if (rl.isKeyPressed(.b)) self.mission_maneuver = .pass;
        if (rl.isKeyPressed(.c)) self.mission_maneuver = .circle;

        if (self.mission_kind == .point) {
            if (rl.isKeyDown(.d)) self.mission_position[0] += mission_target_step;
            if (rl.isKeyDown(.a)) self.mission_position[0] -= mission_target_step;
            if (rl.isKeyDown(.w)) self.mission_position[1] += mission_target_step;
            if (rl.isKeyDown(.s)) self.mission_position[1] -= mission_target_step;
            if (rl.isKeyDown(.r)) self.mission_position[2] += mission_target_step;
            if (rl.isKeyDown(.f)) self.mission_position[2] -= mission_target_step;
        } else {
            if (rl.isKeyPressed(.q)) self.mission_object_class -%= 1;
            if (rl.isKeyPressed(.e)) self.mission_object_class +%= 1;
        }

        if (rl.isKeyDown(.right)) self.mission_look_yaw -= mission_look_step;
        if (rl.isKeyDown(.left)) self.mission_look_yaw += mission_look_step;
        if (rl.isKeyDown(.up)) self.mission_look_pitch = @min(1.4, self.mission_look_pitch + mission_look_step);
        if (rl.isKeyDown(.down)) self.mission_look_pitch = @max(-1.4, self.mission_look_pitch - mission_look_step);
        self.mission_look_direction = .{
            @sin(self.mission_look_yaw) * @cos(self.mission_look_pitch),
            @cos(self.mission_look_yaw) * @cos(self.mission_look_pitch),
            @sin(self.mission_look_pitch),
        };

        const clicked_send = mouse_click and rl.checkCollisionPointRec(rl.getMousePosition(), layout.sendButton());
        if ((rl.isKeyPressed(.enter) or clicked_send) and running and (self.mission_kind != .point or (self.mission_target_initialized and telemetry_live))) {
            const command: MissionIO.MissionCommand = switch (self.mission_kind) {
                .point => .{
                    .kind = .point,
                    .maneuver = .move,
                    .position = self.mission_position,
                    .look_direction = self.mission_look_direction,
                },
                .object => .{
                    .kind = .object,
                    .maneuver = self.mission_maneuver,
                    .object_class = self.mission_object_class,
                },
            };
            actions.mission = command;
            self.mission_sent = true;
        }
    } else if (!self.mission_control and running and now - self.last_teleop >= teleop_period) {
        self.last_teleop = now;
        self.translation = .{ 0, 0, 0 };
        if (focused and rl.isKeyDown(.d)) self.translation[0] += 1;
        if (focused and rl.isKeyDown(.a)) self.translation[0] -= 1;
        if (focused and rl.isKeyDown(.w)) self.translation[1] += 1;
        if (focused and rl.isKeyDown(.s)) self.translation[1] -= 1;
        if (focused and rl.isKeyDown(.r)) self.translation[2] += 1;
        if (focused and rl.isKeyDown(.f)) self.translation[2] -= 1;
        normalizeTranslation(&self.translation);

        if (focused and rl.isKeyDown(.right)) self.orientation = math.quaternionMul(axisAngle(.{ 0, 0, 1 }, -teleop_step), self.orientation);
        if (focused and rl.isKeyDown(.left)) self.orientation = math.quaternionMul(axisAngle(.{ 0, 0, 1 }, teleop_step), self.orientation);
        if (focused and rl.isKeyDown(.up)) self.orientation = math.quaternionMul(self.orientation, axisAngle(.{ 1, 0, 0 }, -teleop_step));
        if (focused and rl.isKeyDown(.down)) self.orientation = math.quaternionMul(self.orientation, axisAngle(.{ 1, 0, 0 }, teleop_step));
        if (focused and rl.isKeyDown(.i)) self.orientation = math.quaternionMul(self.orientation, axisAngle(.{ 0, 1, 0 }, teleop_step));
        if (focused and rl.isKeyDown(.k)) self.orientation = math.quaternionMul(self.orientation, axisAngle(.{ 0, 1, 0 }, -teleop_step));
        normalizeQuaternion(&self.orientation);

        actions.teleop = .{ .translation = self.translation, .orientation = self.orientation };
    }

    if (!self.mission_control and focused) {
        if (rl.isKeyPressed(.q)) {
            self.tune_selection = (self.tune_selection + tune_names.len - 1) % tune_names.len;
        }
        if (rl.isKeyPressed(.e)) {
            self.tune_selection = (self.tune_selection + 1) % tune_names.len;
        }
    }

    var tune_changed = false;
    if (focused and self.runtime_panel.synchronized() and rl.isMouseButtonPressed(.left) and rl.checkCollisionPointRec(rl.getMousePosition(), layout.resetPidRect())) {
        for (0..tune_names.len) |i| tuningValue(&self.config_packet, i).* = 0;
        tune_changed = true;
    }
    if (focused and rl.isMouseButtonPressed(.left)) {
        const mouse = rl.getMousePosition();
        for (0..tune_names.len) |index| {
            if (rl.checkCollisionPointRec(mouse, layout.pidRect(index))) {
                self.tune_selection = index;
                self.pid_drag = index;
                break;
            }
        }
    }
    if (!focused or !rl.isMouseButtonDown(.left)) self.pid_drag = null;
    if (self.pid_drag) |index| {
        if (now - self.last_tune_adjust >= tune_period) {
            const value = pidSliderValue(index, rl.getMousePosition().x, layout);
            const target = tuningValue(&self.config_packet, index);
            if (@abs(target.* - value) > 1e-5) {
                target.* = value;
                self.last_tune_adjust = now;
                tune_changed = true;
            }
        }
    }
    if (focused and now - self.last_tune_adjust >= tune_period) {
        var delta: f32 = 0;
        if (rl.isKeyDown(.z)) delta -= tune_step;
        if (rl.isKeyDown(.x)) delta += tune_step;
        if (delta != 0) {
            tuningValue(&self.config_packet, self.tune_selection).* += delta;
            self.last_tune_adjust = now;
            tune_changed = true;
        }
    }
    if (tune_changed and self.runtime_panel.synchronized()) {
        self.runtime_panel.queueConfig(self.config_packet);
        self.tune_sent = true;
    }

    const direct_thrusters = (if (self.runtime_panel.status) |status|
        if (status.active) status.active_config.mission_id else self.chosen_mission
    else
        self.chosen_mission) == .thruster_control;
    const direct_thruster_count: usize = if (self.runtime_panel.status) |status|
        if (status.active) status.active_config.tam_len else self.config_packet.tam_len
    else
        self.config_packet.tam_len;
    if (direct_thrusters and running and focused) {
        if (rl.isMouseButtonPressed(.left)) {
            const mouse = rl.getMousePosition();
            if (rl.checkCollisionPointRec(mouse, layout.directThrusterZeroRect()) and self.runtime_panel.synchronized()) {
                if (self.runtime_panel.status) |status| {
                    self.runtime_panel.zeroThrusters();
                    self.runtime_panel.publishThrusters(status);
                    self.last_thruster_adjust = now;
                }
            } else {
                for (0..direct_thruster_count) |i| {
                    if (rl.checkCollisionPointRec(mouse, layout.directThrusterSliderRect(i, direct_thruster_count))) {
                        self.direct_thruster_drag = i;
                        break;
                    }
                }
            }
        }
        if (self.direct_thruster_drag) |index| {
            if (self.runtime_panel.synchronized()) {
                if (self.runtime_panel.status) |status| {
                    const track = layout.directThrusterSliderRect(index, direct_thruster_count);
                    const value = ui.thrusterSliderValue(rl.getMousePosition().x, track);
                    if (@abs(self.runtime_panel.direct_values[index] - value) > 1e-4 and
                        (now - self.last_thruster_adjust >= tune_period or !rl.isMouseButtonDown(.left)))
                    {
                        self.runtime_panel.direct_values[index] = value;
                        self.runtime_panel.publishThrusters(status);
                        self.last_thruster_adjust = now;
                    }
                }
            }
        }
    }
    if (!focused or !rl.isMouseButtonDown(.left)) self.direct_thruster_drag = null;

    if (focused and rl.checkCollisionPointRec(rl.getMousePosition(), view3d)) {
        if (rl.isMouseButtonDown(.right)) {
            const delta = rl.getMouseDelta();
            self.orbit_yaw -= delta.x * 0.01;
            self.orbit_pitch = @min(1.35, @max(0.15, self.orbit_pitch - delta.y * 0.01));
        }
        self.orbit_dist = @max(1.5, self.orbit_dist - rl.getMouseWheelMove());
    }

    const sub = toRay(telemetry.camera_pose.pos);
    const camera = rl.Camera3D{
        .position = .{
            .x = sub.x + self.orbit_dist * @cos(self.orbit_pitch) * @sin(self.orbit_yaw),
            .y = sub.y + self.orbit_dist * @sin(self.orbit_pitch),
            .z = sub.z + self.orbit_dist * @cos(self.orbit_pitch) * @cos(self.orbit_yaw),
        },
        .target = sub,
        .up = .{ .x = 0, .y = 1, .z = 0 },
        .fovy = 50,
        .projection = .perspective,
    };

    rl.beginTextureMode(self.scene);
    rl.clearBackground(.{ .r = 12, .g = 18, .b = 28, .a = 255 });
    rl.beginMode3D(camera);
    drawFollowingGrid(sub);
    rl.drawLine3D(sub, .{ .x = sub.x, .y = 0, .z = sub.z }, rl.fade(rl.Color.sky_blue, 0.45));
    drawBox(telemetry.camera_pose.pos, telemetry.camera_pose.quat, .{ 0.35, 0.8, 0.22 }, rl.Color.sky_blue);

    const nose = math.quaternionRotate(telemetry.camera_pose.quat, [3]f32{ 0, 1, 0 });
    rl.drawLine3D(sub, toRay(add3(telemetry.camera_pose.pos, nose)), rl.Color.orange);
    const command_forward = math.quaternionRotate(self.orientation, [3]f32{ 0, 2, 0 });
    drawArrow(telemetry.camera_pose.pos, command_forward, 0.02, rl.Color.green);
    const command_move = math.quaternionRotate(self.orientation, self.translation);
    if (math.dot3f(command_move, command_move) > 1e-6)
        drawArrow(telemetry.camera_pose.pos, scaled3(command_move, 1.2), 0.045, rl.Color.magenta);
    if (self.mission_control and self.mission_target_initialized) {
        rl.drawSphere(toRay(self.mission_position), 0.18, rl.Color.yellow);
        drawArrow(self.mission_position, self.mission_look_direction, 0.035, rl.Color.yellow);
    }
    const force_error = math.quaternionRotate(telemetry.camera_pose.quat, telemetry.force_error);
    if (math.dot3f(force_error, force_error) > 1e-6)
        drawArrow(telemetry.camera_pose.pos, force_error, 0.03, rl.Color.yellow);

    for (telemetry.objects[0..telemetry.objects_len], 0..) |object, i| {
        if (i == 0) rl.drawLine3D(sub, toRay(object.pose.pos), rl.fade(rl.Color.ray_white, 0.45));
        var size = object.size;
        for (&size) |*axis| if (axis.* < 0.05) {
            axis.* = 0.3;
        };
        drawBox(object.pose.pos, object.pose.quat, size, objectColor(object.cls));
    }
    for (telemetry.goals[0..telemetry.goals_len], 0..) |goal, i| {
        const at = toRay(goal.position);
        const look = toRay(goal.look_at);
        const color = if (i == 0) rl.Color.white else rl.Color.lime;
        if (i + 1 < telemetry.goals_len)
            rl.drawLine3D(at, toRay(telemetry.goals[i + 1].position), rl.Color.lime);
        rl.drawLine3D(at, look, rl.fade(color, 0.65));
        rl.drawSphere(at, if (i == 0) 0.18 else 0.1, color);
    }
    rl.endMode3D();
    rl.endTextureMode();

    rl.beginDrawing();
    ui.draw(.{
        .font = self.font,
        .font_percent = self.font_percent,
        .workspace = self.workspace,
        .layout = layout,
        .frame = telemetry,
        .telemetry_status = input.telemetry_status,
        .video_status = input.video.status,
        .video = input.video.texture,
        .video_w = input.video.width,
        .video_h = input.video.height,
        .scene = self.scene,
        .mission_control = self.mission_control,
        .chosen_mission = self.chosen_mission,
        .active_mission = if (self.runtime_panel.status) |status| if (status.active) status.active_config.mission_id else null else null,
        .dropdown_open = self.dropdown_open,
        .runtime_switch_pending = self.runtime_panel.pending_config != null,
        .control_ready = self.runtime_panel.synchronized(),
        .send_error = input.send_error,
        .translation = self.translation,
        .orientation = self.orientation,
        .mission_kind = self.mission_kind,
        .mission_maneuver = self.mission_maneuver,
        .mission_position = self.mission_position,
        .mission_look = self.mission_look_direction,
        .mission_class = self.mission_object_class,
        .target_initialized = self.mission_target_initialized,
        .config = &self.config_packet,
        .tune_selection = self.tune_selection,
        .mission_sent = self.mission_sent,
        .tune_sent = self.tune_sent,
        .object_scroll = self.object_scroll,
        .goal_scroll = self.goal_scroll,
        .direct_thrusters = direct_thrusters,
        .direct_thruster_count = direct_thruster_count,
        .direct_thruster_values = self.runtime_panel.direct_values[0..direct_thruster_count],
        .direct_thruster_drag = self.direct_thruster_drag,
    });
    if (self.workspace == .settings) self.runtime_panel.draw(self.font, layout.content, &self.config_packet, telemetry, telemetry_live, self.embedded);
    rl.endDrawing();

    actions.config = self.runtime_panel.pending_config;
    actions.lifecycle = self.runtime_panel.pending_request;
    if (self.runtime_panel.synchronized()) actions.thrusters = self.runtime_panel.direct_command;
    return actions;
}

fn tuningValue(packet: *MissionIO.LiveConfigPacket, index: usize) *f32 {
    const group = index / 6;
    const axis = index % 6;
    return switch (group) {
        0 => &packet.odometry.kp[axis],
        1 => &packet.odometry.ki[axis],
        2 => &packet.odometry.kd[axis],
        else => unreachable,
    };
}

fn pidSliderValue(index: usize, mouse_x: f32, layout: ui.Layout) f32 {
    const rect = layout.pidRect(index);
    const ratio = std.math.clamp((mouse_x - rect.x) / rect.width, 0, 1);
    return ratio * pid_slider_max[index / 6];
}

fn emptyTelemetry() MissionIO.TelemetryFrame {
    var frame = std.mem.zeroes(MissionIO.TelemetryFrame);
    frame.magic = "AUV1".*;
    frame.abi_version = MissionIO.telemetry_frame_abi_version;
    frame.camera_pose.quat[3] = 1;
    return frame;
}

fn toRay(v: [3]f32) rl.Vector3 {
    return .{ .x = v[0], .y = -v[2], .z = -v[1] };
}

fn add3(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

fn scaled3(v: [3]f32, scale: f32) [3]f32 {
    return .{ v[0] * scale, v[1] * scale, v[2] * scale };
}

fn drawArrow(origin: [3]f32, vector: [3]f32, radius: f32, color: rl.Color) void {
    const from = toRay(origin);
    const to = toRay(add3(origin, vector));
    rl.drawCylinderEx(from, to, radius, radius, 8, color);
    rl.drawSphere(to, radius * 2.5, color);
}

fn drawBox(pos: [3]f32, quat: [4]f32, size: [3]f32, color: rl.Color) void {
    var corners: [8]rl.Vector3 = undefined;
    var n: usize = 0;
    const signs = [_]f32{ -1, 1 };
    for (signs) |sx| {
        for (signs) |sy| {
            for (signs) |sz| {
                const local = [3]f32{
                    sx * size[0] * 0.5,
                    sy * size[1] * 0.5,
                    sz * size[2] * 0.5,
                };
                corners[n] = toRay(add3(pos, math.quaternionRotate(quat, local)));
                n += 1;
            }
        }
    }
    const edges = [_][2]usize{
        .{ 0, 1 }, .{ 2, 3 }, .{ 4, 5 }, .{ 6, 7 },
        .{ 0, 2 }, .{ 1, 3 }, .{ 4, 6 }, .{ 5, 7 },
        .{ 0, 4 }, .{ 1, 5 }, .{ 2, 6 }, .{ 3, 7 },
    };
    for (edges) |edge| rl.drawLine3D(corners[edge[0]], corners[edge[1]], color);
}

fn objectColor(cls: u8) rl.Color {
    return rl.colorFromHSV(@floatFromInt((@as(u16, cls) * 47) % 360), 0.75, 0.95);
}

fn axisAngle(axis: [3]f32, angle: f32) [4]f32 {
    const half = angle * 0.5;
    const sine = @sin(half);
    return .{ axis[0] * sine, axis[1] * sine, axis[2] * sine, @cos(half) };
}

fn normalizeQuaternion(quat: *[4]f32) void {
    const value: math.Quaternionf = quat.*;
    const length = @sqrt(@reduce(.Add, value * value));
    if (length > 1e-6)
        quat.* = value / @as(math.Quaternionf, @splat(length));
}

fn normalizeTranslation(translation: *[3]f32) void {
    const length = @sqrt(
        translation[0] * translation[0] +
            translation[1] * translation[1] +
            translation[2] * translation[2],
    );
    if (length > 1e-6) {
        for (translation) |*axis| axis.* /= length;
    }
}

fn drawFollowingGrid(sub: rl.Vector3) void {
    const center_x = @floor(sub.x);
    const center_z = @floor(sub.z);
    for (0..21) |index| {
        const offset: f32 = @as(f32, @floatFromInt(index)) - 10;
        rl.drawLine3D(
            .{ .x = center_x + offset, .y = 0, .z = center_z - 10 },
            .{ .x = center_x + offset, .y = 0, .z = center_z + 10 },
            rl.Color.gray,
        );
        rl.drawLine3D(
            .{ .x = center_x - 10, .y = 0, .z = center_z + offset },
            .{ .x = center_x + 10, .y = 0, .z = center_z + offset },
            rl.Color.gray,
        );
    }
}

test {
    _ = ui;
}
