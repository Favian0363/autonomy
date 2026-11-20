const std = @import("std");
const rl = @import("raylib");
const IO = @import("../MissionIO.zig");

pub const Workspace = enum { runtime, settings };
pub const Status = enum { disabled, connecting, live, ready, stale, unavailable };
pub const Rect = rl.Rectangle;
const bg = color(3, 5, 4, 255);
const sidebar = color(8, 13, 11, 255);
const glass = color(17, 23, 21, 245);
const surface = color(11, 17, 15, 255);
const border = color(63, 101, 107, 180);
const white = color(235, 245, 242, 255);
const muted = color(157, 180, 183, 255);
const teal = color(0, 226, 200, 255);
const blue = color(0, 226, 200, 255);
const amber = color(244, 187, 113, 255);
const red = color(242, 123, 120, 255);
const axis_names = [_][:0]const u8{ "X", "Y", "Z", "ROLL", "PITCH", "YAW" };
const groups = [_][:0]const u8{ "P", "I", "D" };
const gain_max = [_]f32{ 10, 2, 10 };
pub const min_font_percent: u8 = 80;
pub const default_font_percent: u8 = 175;
pub const max_font_percent: u8 = 200;
var font_scale: f32 = @as(f32, @floatFromInt(default_font_percent)) / 100.0;

pub fn adjustFontPercent(current: u8, delta: i16) u8 {
    return @intCast(std.math.clamp(@as(i16, current) + delta, min_font_percent, max_font_percent));
}

fn color(r: u8, g: u8, b: u8, a: u8) rl.Color {
    return .{ .r = r, .g = g, .b = b, .a = a };
}
fn rect(x: f32, y: f32, w: f32, h: f32) Rect {
    return .{ .x = x, .y = y, .width = w, .height = h };
}
fn inset(r: Rect, x: f32, y: f32) Rect {
    return rect(r.x + x, r.y + y, @max(1, r.width - 2 * x), @max(1, r.height - 2 * y));
}
fn box(r: Rect, c: rl.Color) void {
    rl.drawRectangleRec(r, c);
}
fn panel(r: Rect) void {
    rl.drawRectangleRounded(r, 0.035, 8, glass);
    rl.drawRectangleRoundedLinesEx(r, 0.035, 8, 1, border);
}
fn text(font: rl.Font, s: [:0]const u8, x: f32, y: f32, size: f32, c: rl.Color) void {
    rl.drawTextEx(font, s, .{ .x = x, .y = y }, size * font_scale, 0.15 * font_scale, c);
}
fn title(font: rl.Font, s: [:0]const u8, r: Rect) void {
    text(font, s, r.x + 14, r.y + 12, 13, muted);
}
fn line(x: f32, y: f32, w: f32) void {
    box(rect(x, y, w, 1), border);
}
fn statusText(s: Status) [:0]const u8 {
    return switch (s) {
        .disabled => "OFF",
        .connecting => "CONNECTING",
        .live => "LIVE",
        .ready => "READY",
        .stale => "STALE",
        .unavailable => "UNAVAILABLE",
    };
}
fn statusColor(s: Status) rl.Color {
    return switch (s) {
        .live, .ready => teal,
        .connecting, .stale => amber,
        .disabled => muted,
        .unavailable => red,
    };
}
fn pill(font: rl.Font, r: Rect, label: [:0]const u8, s: Status) void {
    rl.drawRectangleRounded(r, 0.25, 6, surface);
    rl.drawRectangleRoundedLinesEx(r, 0.25, 6, 1, border);
    rl.drawCircleV(.{ .x = r.x + 10, .y = r.y + r.height / 2 }, 3, statusColor(s));
    const label_text = rl.textFormat("%s %s", .{ label.ptr, statusText(s).ptr });
    const wanted_size = 11 * font_scale;
    const measured = rl.measureTextEx(font, label_text, wanted_size, 0.15 * font_scale);
    const fit = @min(@as(f32, 1), (r.width - 25) / @max(1, measured.x));
    rl.drawTextEx(font, label_text, .{ .x = r.x + 19, .y = r.y + 7 }, wanted_size * fit, 0.15 * font_scale * fit, statusColor(s));
}

pub const Layout = struct {
    video_enabled: bool,
    sidebar: Rect,
    top: Rect,
    content: Rect,
    video: Rect,
    scene: Rect,
    state: Rect,
    thrusters: Rect,
    controls: Rect,
    objects: Rect,
    goals: Rect,
    editor: Rect,
    gains: Rect,
    observed: Rect,
    update: Rect,
    pub fn init(w: f32, h: f32, video_enabled: bool) Layout {
        const gap: f32 = if (w < 1200) 8 else 14;
        const margin: f32 = if (w < 1200) 12 else 20;
        const y: f32 = if (w < 1200) 60 else 70;
        const cw = w - 2 * margin;
        const ch = h - y - margin;
        const lw = @floor((cw - 2 * gap) * 0.40);
        const mw = @floor((cw - 2 * gap) * 0.18);
        const rw = cw - 2 * gap - lw - mw;
        const mx = margin + lw + gap;
        const rx = mx + mw + gap;
        const scene_h = @floor((ch - gap) * 0.49);
        const control_h = @floor((ch - gap) * 0.39);
        const empty = rect(margin, y, 1, 1);
        return .{
            .video_enabled = video_enabled,
            .sidebar = rect(0, 0, 0, h),
            .top = rect(0, 0, w, y - 8),
            .content = rect(margin, y, cw, ch),
            .scene = rect(margin, y, lw, scene_h),
            .video = rect(margin, y + scene_h + gap, lw, ch - scene_h - gap),
            .state = rect(mx, y, mw, ch),
            .thrusters = empty,
            .controls = rect(rx, y, rw, control_h),
            .editor = rect(rx, y, rw, control_h),
            .gains = rect(rx, y + control_h + gap, rw, ch - control_h - gap),
            .objects = empty,
            .goals = empty,
            .observed = empty,
            .update = empty,
        };
    }
    pub fn sceneContent(self: Layout, _: Workspace) Rect {
        const r = self.scene;
        return rect(r.x + 8, r.y + 38, r.width - 16, r.height - 46);
    }

    pub fn pidRect(self: Layout, index: usize) Rect {
        const a = self.gains;
        const left: f32 = 30;
        const gap: f32 = 5;
        const cell = (a.width - left - 12 - 5 * gap) / 6;
        const row = @min(@as(f32, 58), (a.height - 145) / 3);
        return rect(a.x + left + @as(f32, @floatFromInt(index % 6)) * (cell + gap), a.y + 82 + @as(f32, @floatFromInt(index / 6)) * row, cell, 23);
    }
    pub fn resetPidRect(self: Layout) Rect {
        return rect(self.gains.x + self.gains.width - 132, self.gains.y + 8, 120, 25);
    }
    pub fn dropdownRect(self: Layout) Rect {
        return rect(self.controls.x + 12, self.controls.y + 38, self.controls.width - 108, 30);
    }
    pub fn activateRect(self: Layout) Rect {
        return rect(self.controls.x + self.controls.width - 88, self.controls.y + 38, 76, 30);
    }
    pub fn dropdownOption(self: Layout, i: usize) Rect {
        const r = self.dropdownRect();
        return rect(r.x, r.y + r.height + @as(f32, @floatFromInt(i)) * 28, r.width, 28);
    }

    pub fn listCapacity(r: Rect) usize {
        return @intFromFloat(@max(0, @floor((r.height - 78) / 20)));
    }
    pub fn navRect(self: Layout, i: usize) Rect {
        _ = self;
        return rect(215 + @as(f32, @floatFromInt(i)) * 100, 14, 90, 28);
    }
    pub fn fontButton(self: Layout, increase: bool) Rect {
        return rect(self.top.width - (if (increase) @as(f32, 34) else 104), 15, 25, 27);
    }
    pub fn missionButton(self: Layout, i: usize) Rect {
        return rect(self.editor.x + 12 + @as(f32, @floatFromInt(i)) * 80, self.editor.y + 112, 75, 27);
    }
    pub fn sendButton(self: Layout) Rect {
        return rect(self.editor.x + 12, self.editor.y + self.editor.height - 42, 150, 28);
    }
    pub fn directThrusterSliderRect(self: Layout, index: usize, count: usize) Rect {
        const p = self.controls;
        const label_w: f32 = 30;
        const value_w: f32 = 48;
        const track_x = p.x + 12 + label_w;
        const track_w = @max(40, p.width - 24 - label_w - value_w);
        const top = p.y + 112;
        const row_h = @min(@as(f32, 30), (p.height - 150) / @as(f32, @floatFromInt(@max(count, 1))));
        return rect(track_x, top + @as(f32, @floatFromInt(index)) * row_h + 8, track_w, 14);
    }
    pub fn directThrusterZeroRect(self: Layout) Rect {
        return rect(self.controls.x + 12, self.controls.y + self.controls.height - 38, 108, 26);
    }
};

pub fn thrusterSliderValue(mouse_x: f32, track: Rect) f32 {
    const ratio = std.math.clamp((mouse_x - track.x) / track.width, 0, 1);
    return ratio * 2 - 1;
}

pub const Data = struct {
    font: rl.Font,
    font_percent: u8,
    workspace: Workspace,
    layout: Layout,
    frame: *const IO.TelemetryFrame,
    telemetry_status: Status,
    video_status: Status,
    video: rl.Texture2D,
    video_w: i32,
    video_h: i32,
    scene: rl.RenderTexture2D,
    mission_control: bool,
    chosen_mission: IO.CompiledConfig.MissionId,
    active_mission: ?IO.CompiledConfig.MissionId,
    dropdown_open: bool,
    runtime_switch_pending: bool,
    control_ready: bool,
    send_error: bool,
    translation: [3]f32,
    orientation: [4]f32,
    mission_kind: IO.MissionCommandKind,
    mission_maneuver: IO.MissionManeuver,
    mission_position: [3]f32,
    mission_look: [3]f32,
    mission_class: u8,
    target_initialized: bool,
    config: *IO.LiveConfigPacket,
    tune_selection: usize,
    mission_sent: bool,
    tune_sent: bool,
    object_scroll: usize,
    goal_scroll: usize,
    direct_thrusters: bool,
    direct_thruster_count: usize,
    direct_thruster_values: []const f32,
    direct_thruster_drag: ?usize,
};
fn gain(d: Data, i: usize) f32 {
    return switch (i / 6) {
        0 => d.config.odometry.kp[i % 6],
        1 => d.config.odometry.ki[i % 6],
        else => d.config.odometry.kd[i % 6],
    };
}
fn observedGain(d: Data, i: usize) f32 {
    return switch (i / 6) {
        0 => d.frame.kp[i % 6],
        1 => d.frame.ki[i % 6],
        else => d.frame.kd[i % 6],
    };
}
fn runtimeName(mission_id: IO.CompiledConfig.MissionId) [:0]const u8 {
    return switch (mission_id) {
        .prequalify_coco => "Coco",
        .prequalify_robosub => "RoboSub PQ",
        .teleop => "Teleop",
        .mission_control => "Command Mission",
        .thruster_control => "Direct Thrusters",
    };
}

fn button(font: rl.Font, r: Rect, label: [:0]const u8, active: bool) void {
    rl.drawRectangleRounded(r, 0.18, 6, if (active) teal else surface);
    rl.drawRectangleRoundedLinesEx(r, 0.18, 6, 1, border);
    text(font, label, r.x + 10, r.y + 9, 12, if (active) bg else white);
}
fn frameContent(d: Data, r: Rect) void {
    const src = rect(0, 0, @floatFromInt(d.scene.texture.width), -@as(f32, @floatFromInt(d.scene.texture.height)));
    rl.drawTexturePro(d.scene.texture, src, r, .{ .x = 0, .y = 0 }, 0, rl.Color.white);
}
fn scenePanel(d: Data, r: Rect) void {
    const panelrect = rect(r.x - 8, r.y - 38, r.width + 16, r.height + 46);
    panel(panelrect);
    title(d.font, "3D SCENE", panelrect);
    if (d.telemetry_status == .live) {
        frameContent(d, r);
        box(rect(r.x + 4, r.y + 4, @min(r.width - 8, 285), 55), color(9, 17, 24, 210));
        text(d.font, "Right drag: orbit  |  Wheel: zoom", r.x + 8, r.y + 8, 11, muted);
        text(d.font, "Blue: vehicle  Orange: nose  Green: command", r.x + 8, r.y + 24, 11, muted);
        text(d.font, "Pink: movement  Yellow: error  Lime: goals", r.x + 8, r.y + 40, 11, muted);
    } else {
        box(r, color(10, 25, 33, 255));
        text(d.font, "Telemetry unavailable", r.x + 16, r.y + r.height / 2 - 12, 16, amber);
    }
}
fn videoPanel(d: Data, p: Rect) void {
    panel(p);
    title(d.font, "FORWARD CAMERA", p);
    const r = rect(p.x + 8, p.y + 38, p.width - 16, p.height - 46);
    box(r, color(10, 25, 33, 255));
    if (d.video_status != .live or d.video_w <= 0 or d.video_h <= 0) {
        text(d.font, if (d.video_status == .disabled) "Video disabled" else if (d.video_status == .stale) "Video frames are stale" else if (d.video_status == .unavailable) "Video unavailable" else "Waiting for video", r.x + 16, r.y + r.height / 2 - 12, 16, amber);
        return;
    }
    const vw: f32 = @floatFromInt(d.video_w);
    const vh: f32 = @floatFromInt(d.video_h);
    const scale = @min(r.width / vw, r.height / vh);
    const dest = rect(r.x + (r.width - vw * scale) / 2, r.y + (r.height - vh * scale) / 2, vw * scale, vh * scale);
    rl.drawTexturePro(d.video, rect(0, 0, vw, vh), dest, .{ .x = 0, .y = 0 }, 0, rl.Color.white);
    if (d.telemetry_status == .live) for (d.frame.boxes[0..d.frame.boxes_len]) |b| {
        if (b.bottom_right.x < b.top_left.x or b.bottom_right.y < b.top_left.y) continue;
        const x0 = @min(@as(f32, @floatFromInt(b.top_left.x)), vw);
        const y0 = @min(@as(f32, @floatFromInt(b.top_left.y)), vh);
        const x1 = @min(@as(f32, @floatFromInt(b.bottom_right.x)), vw);
        const y1 = @min(@as(f32, @floatFromInt(b.bottom_right.y)), vh);
        rl.drawRectangleLinesEx(rect(dest.x + x0 * scale, dest.y + y0 * scale, (x1 - x0) * scale, (y1 - y0) * scale), 2, teal);
    };
}
fn drawThrusterBars(d: Data, p: Rect, top_y: f32) void {
    if (d.telemetry_status != .live) return;
    if (d.frame.thrusters_len == 0) {
        text(d.font, "No thruster data", p.x + 10, top_y, 11, muted);
        return;
    }
    const count: usize = d.frame.thrusters_len;
    const usable = @max(0, p.y + p.height - top_y - 16);
    const step = @min(@as(f32, 24), usable / @as(f32, @floatFromInt(@max(count, 1))));
    for (d.frame.thrusters[0..count], 0..) |v, i| {
        const yy = top_y + 4 + @as(f32, @floatFromInt(i)) * step;
        const start = p.x + 37;
        const end = p.x + p.width - 45;
        const mid = (start + end) / 2;
        text(d.font, rl.textFormat("T%d", .{@as(c_int, @intCast(i))}), p.x + 12, yy, 11, muted);
        box(rect(start, yy + 7, @max(1, end - start), 1), border);
        box(rect(mid, yy + 2, 1, 12), muted);
        const width = @min(1, @abs(v)) * (end - start) / 2;
        box(rect(if (v < 0) mid - width else mid, yy + 3, @max(1, width), 8), blue);
        text(d.font, rl.textFormat("%+.2f", .{@as(f64, v)}), end + 4, yy, 10, white);
    }
}
fn statePanel(d: Data, p: Rect) void {
    panel(p);
    title(d.font, "VEHICLE MEASUREMENTS", p);
    if (d.telemetry_status != .live) {
        text(d.font, "Telemetry unavailable", p.x + 10, p.y + 40, 12, amber);
        return;
    }
    var buf: [IO.telemetry_step_max + 1]u8 = undefined;
    const n = @min(@as(usize, d.frame.step_len), d.frame.step.len);
    @memcpy(buf[0..n], d.frame.step[0..n]);
    buf[n] = 0;
    text(d.font, "DEPTH", p.x + 10, p.y + 36, 10, muted);
    text(d.font, if (d.frame.depth_ok) rl.textFormat("%.2f m", .{@as(f64, d.frame.depth)}) else "--", p.x + 10, p.y + 51, 18, white);
    text(d.font, "MISSION STEP", p.x + 10, p.y + 76, 10, muted);
    text(d.font, if (n > 0) buf[0..n :0] else "--", p.x + 10, p.y + 91, 11, white);
    text(d.font, "POSITION X / Y / Z (m)", p.x + 10, p.y + 116, 10, muted);
    text(d.font, rl.textFormat("%.2f  %.2f  %.2f", .{ @as(f64, d.frame.camera_pose.pos[0]), @as(f64, d.frame.camera_pose.pos[1]), @as(f64, d.frame.camera_pose.pos[2]) }), p.x + 10, p.y + 132, 11, white);
    const angles = IO.Math.quaternionToEuler(d.frame.camera_pose.quat) * @as(@TypeOf(d.frame.camera_pose.pos), @splat(180.0 / std.math.pi));
    text(d.font, "ROLL / PITCH / YAW (deg)", p.x + 10, p.y + 157, 10, muted);
    text(d.font, rl.textFormat("%.1f  %.1f  %.1f", .{ @as(f64, angles[0]), @as(f64, angles[1]), @as(f64, angles[2]) }), p.x + 10, p.y + 173, 11, white);
    const pose_y: f32 = 198;
    if (p.height > pose_y + 14) text(d.font, if (d.frame.position_ok.x and d.frame.position_ok.y and d.frame.position_ok.z and d.frame.attitude_ok.x and d.frame.attitude_ok.y and d.frame.attitude_ok.z) "Pose valid" else "Pose validity incomplete", p.x + 10, p.y + pose_y, 10, teal);
    const thrusters_top = p.y + 222;
    if (p.height > thrusters_top - p.y + 24) {
        line(p.x + 10, thrusters_top - 10, p.width - 20);
        text(d.font, "THRUSTERS", p.x + 10, thrusters_top, 10, muted);
        drawThrusterBars(d, p, thrusters_top + 14);
    }
}
fn drawDirectThrusterSliders(d: Data, p: Rect) void {
    const count = d.direct_thruster_count;
    if (count == 0) {
        text(d.font, "No thrusters configured", p.x + 12, p.y + 120, 11, muted);
        return;
    }
    text(d.font, "Drag sliders (-1 to 1)", p.x + 12, p.y + 116, 10, muted);
    for (0..count) |i| {
        const track = d.layout.directThrusterSliderRect(i, count);
        const value = if (i < d.direct_thruster_values.len) d.direct_thruster_values[i] else 0;
        const label_y = track.y - 2;
        text(d.font, rl.textFormat("T%d", .{@as(c_int, @intCast(i + 1))}), p.x + 12, label_y, 11, muted);
        box(track, surface);
        rl.drawRectangleLinesEx(track, 1, if (d.direct_thruster_drag == i) teal else border);
        const mid = track.x + track.width / 2;
        box(rect(mid - 0.5, track.y - 1, 1, track.height + 2), muted);
        const half = track.width / 2;
        const fill = @min(half, @abs(value) * half);
        if (fill > 0.5) {
            box(rect(if (value < 0) mid - fill else mid, track.y + 2, fill, track.height - 4), teal);
        }
        const knob_x = mid + value * half - 4;
        rl.drawCircleV(.{ .x = knob_x, .y = track.y + track.height / 2 }, 5, if (d.direct_thruster_drag == i) white else teal);
        const value_x = track.x + track.width + 6;
        text(d.font, rl.textFormat("%+.2f", .{@as(f64, value)}), value_x, label_y, 10, white);
        if (d.telemetry_status == .live and i < d.frame.thrusters_len) {
            text(d.font, rl.textFormat("O%+.2f", .{@as(f64, d.frame.thrusters[i])}), value_x, label_y + 14, 9, muted);
        }
    }
    button(d.font, d.layout.directThrusterZeroRect(), "All zero", d.telemetry_status == .live);
}
fn topbar(d: Data) void {
    const l = d.layout;
    box(l.top, sidebar);
    text(d.font, "A", 16, 13, 20, teal);
    text(d.font, "AUTONOMY VIEWER", 42, 14, 15, white);
    const names = [_][:0]const u8{ "Runtime", "Settings" };
    for (names, 0..) |name, i| button(d.font, l.navRect(i), name, @backingInt(d.workspace) == i);
    const sx = @max(@as(f32, 545), l.top.width - 470);
    pill(d.font, rect(sx, 14, 108, 28), "TLM", d.telemetry_status);
    pill(d.font, rect(sx + 113, 14, 120, 28), "VIDEO", d.video_status);
    pill(d.font, rect(sx + 238, 14, 112, 28), "SEND", if (!d.control_ready or d.send_error) .unavailable else .ready);
    button(d.font, l.fontButton(false), "-", false);
    button(d.font, l.fontButton(true), "+", false);
    text(d.font, rl.textFormat("%d%%", .{@as(c_int, d.font_percent)}), l.top.width - 75, 22, 10, white);
}

fn dashboard(d: Data) void {
    const l = d.layout;
    scenePanel(d, l.sceneContent(.runtime));
    videoPanel(d, l.video);
    statePanel(d, l.state);
    const p = l.controls;
    panel(p);
    title(d.font, "CONTROL MODE", p);
    button(d.font, l.dropdownRect(), runtimeName(d.chosen_mission), false);
    text(d.font, "v", l.dropdownRect().x + l.dropdownRect().width - 18, l.dropdownRect().y + 8, 12, muted);
    button(d.font, l.activateRect(), "Activate", true);
    text(d.font, if (d.telemetry_status != .live) "Confirmed: unavailable" else rl.textFormat("Confirmed: %s", .{runtimeName((d.active_mission orelse d.config.mission_id)).ptr}), p.x + 12, p.y + 77, 11, if (d.telemetry_status == .live and !d.runtime_switch_pending) teal else amber);
    if (d.runtime_switch_pending) text(d.font, "Request sent; awaiting telemetry", p.x + 12, p.y + 94, 10, amber);
    if (d.mission_control) {
        button(d.font, l.missionButton(0), "Point P", d.mission_kind == .point);
        button(d.font, l.missionButton(1), "Object O", d.mission_kind == .object);
        text(d.font, if (d.mission_kind == .point) (if (d.target_initialized) rl.textFormat("Target: %.2f  %.2f  %.2f m", .{ @as(f64, d.mission_position[0]), @as(f64, d.mission_position[1]), @as(f64, d.mission_position[2]) }) else "Target: waiting for telemetry") else rl.textFormat("Class %d  %s", .{ @as(c_uint, d.mission_class), @tagName(d.mission_maneuver).ptr }), p.x + 12, p.y + 147, 11, white);
        text(d.font, if (d.mission_kind == .point) "WASD / RF target  |  Arrows look" else "Q / E class  |  M / B / C maneuver", p.x + 12, p.y + 166, 10, muted);
        button(d.font, l.sendButton(), "Send command", false);
        text(d.font, if (d.mission_sent) "Sent locally; receipt unconfirmed" else "Enter to send; receipt unconfirmed", p.x + 168, p.y + p.height - 34, 10, muted);
    } else if (d.direct_thrusters) {
        if (d.telemetry_status != .live) {
            text(d.font, "Direct thrusters need live telemetry", p.x + 12, p.y + 116, 11, amber);
        } else {
            drawDirectThrusterSliders(d, p);
        }
    } else if ((d.active_mission orelse d.config.mission_id) == .teleop) {
        text(d.font, "TELEOPERATION", p.x + 12, p.y + 116, 11, teal);
        text(d.font, "W / S forward / back     A / D left / right", p.x + 12, p.y + 137, 11, white);
        text(d.font, "R / F down / up           Arrows yaw / pitch", p.x + 12, p.y + 158, 11, white);
        text(d.font, "I / K roll", p.x + 12, p.y + 179, 11, white);
    } else {
        text(d.font, "Configured mission running", p.x + 12, p.y + 120, 12, white);
        text(d.font, "Mission step is shown in measurements", p.x + 12, p.y + 142, 10, muted);
    }
    const g = l.gains;
    panel(g);
    title(d.font, "ODOMETRY PID TUNING", g);
    text(d.font, "Immediate updates  |  drag or Z / X", g.x + 12, g.y + 34, 10, amber);
    button(d.font, l.resetPidRect(), "RESET ALL TO ZERO", d.telemetry_status == .live);
    for (0..6) |axis| {
        const r = l.pidRect(axis);
        text(d.font, axis_names[axis], r.x + 2, r.y - 18, 10, muted);
    }
    for (0..18) |i| {
        const r = l.pidRect(i);
        if (i % 6 == 0) text(d.font, groups[i / 6], g.x + 10, r.y + 4, 12, teal);
        box(r, surface);
        rl.drawRectangleLinesEx(r, 1, if (i == d.tune_selection) teal else border);
        text(d.font, if (d.telemetry_status == .live) rl.textFormat("%.2f", .{@as(f64, gain(d, i))}) else "--", r.x + 4, r.y + 5, 11, white);
        box(rect(r.x, r.y + r.height - 2, r.width * if (d.telemetry_status == .live) std.math.clamp(gain(d, i) / gain_max[i / 6], 0, 1) else 0, 2), teal);
    }
    if (d.telemetry_status == .live) {
        text(d.font, rl.textFormat("Position error (m): %+.2f  %+.2f  %+.2f", .{ @as(f64, d.frame.translation_error[0]), @as(f64, d.frame.translation_error[1]), @as(f64, d.frame.translation_error[2]) }), g.x + 12, g.y + g.height - 90, 10, muted);
        text(d.font, rl.textFormat("Attitude error: %+.2f  %+.2f  %+.2f", .{ @as(f64, d.frame.attitude_error[0]), @as(f64, d.frame.attitude_error[1]), @as(f64, d.frame.attitude_error[2]) }), g.x + 12, g.y + g.height - 72, 10, muted);
    }
    const footer_y = g.y + g.height - 46;
    text(d.font, if (d.telemetry_status == .live) rl.textFormat("Selected %s / %s   Requested %.2f", .{ groups[d.tune_selection / 6].ptr, axis_names[d.tune_selection % 6].ptr, @as(f64, gain(d, d.tune_selection)) }) else "Requested gains unavailable", g.x + 12, footer_y, 10, white);
    text(d.font, if (d.telemetry_status == .live) rl.textFormat("Observed %.2f  |  receipt unconfirmed  |  Runtime: direct control", .{@as(f64, observedGain(d, d.tune_selection))}) else "Telemetry unavailable; tuning disabled", g.x + 12, footer_y + 18, 10, muted);
    if (d.dropdown_open) {
        const modes = @typeInfo(IO.CompiledConfig.MissionId).@"enum".field_names;
        for (modes, 0..) |_, i| {
            const mode: IO.CompiledConfig.MissionId = @fromBackingInt(@as(u8, @intCast(i)));
            button(d.font, l.dropdownOption(i), runtimeName(mode), d.chosen_mission == mode);
        }
    }
}
pub fn draw(d: Data) void {
    font_scale = @as(f32, @floatFromInt(d.font_percent)) / 100.0;
    rl.clearBackground(bg);
    topbar(d);
    if (d.workspace == .runtime) dashboard(d);
}

test "workspace layouts stay within supported window sizes" {
    for ([_][2]f32{ .{ 1000, 700 }, .{ 1280, 1000 }, .{ 1920, 1080 } }) |size| {
        const l = Layout.init(size[0], size[1], true);
        const panels = [_]Rect{ l.video, l.scene, l.state, l.thrusters, l.controls, l.objects, l.goals, l.editor, l.gains, l.observed, l.update, l.sceneContent(.runtime) };
        for (panels) |p| {
            try std.testing.expect(p.width > 0 and p.height > 0);
            try std.testing.expect(p.x >= 0 and p.y >= 0);
            try std.testing.expect(p.x + p.width <= size[0] and p.y + p.height <= size[1]);
        }
        for (0..18) |i| {
            const r = l.pidRect(i);
            try std.testing.expect(r.x >= l.gains.x and r.y >= l.gains.y);
            try std.testing.expect(r.x + r.width <= l.gains.x + l.gains.width);
            try std.testing.expect(r.y + r.height + 16 <= l.gains.y + l.gains.height);
            if (i % 6 != 0) try std.testing.expect(l.pidRect(i - 1).x + l.pidRect(i - 1).width <= r.x);
        }
    }
}
