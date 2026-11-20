const std = @import("std");
const rl = @import("raylib");
const IO = @import("../MissionIO.zig");
const Config = IO.CompiledConfig;
const Self = @This();
const config_field_count = 6;

status: ?IO.RuntimeStatus = null,
draft: IO.LiveConfigPacket = IO.LiveConfigPacket.fromRuntimeConfig(&Config.startup_config),
dirty: bool = false,
editing: ?usize = null,
select_all: bool = false,
client_id: u64 = 0,
request_id: u64 = 0,
pending_request: ?IO.RuntimeRequest = null,
pending_config: ?IO.LiveConfigPacket = null,
applying_draft: bool = false,
last_status: f64 = -1,
direct_values: [Config.thruster_count]f32 = @splat(0),
direct_command: ?IO.ThrusterCommand = null,
direct_sequence: u64 = 0,
direct_run_id: u64 = 0,
error_text: Config.FixedText(256) = .{},

pub fn synchronized(self: *const Self) bool {
    return self.status != null and rl.getTime() - self.last_status < 2;
}

pub fn queueConfig(self: *Self, packet: IO.LiveConfigPacket) void {
    if (self.synchronized() and self.pending_config == null) self.pending_config = packet;
}

fn request(self: *Self, action: IO.RuntimeAction) void {
    if (!self.synchronized()) return;
    self.request_id += 1;
    self.pending_request = .{ .client_id = self.client_id, .request_id = self.request_id, .action = action };
}

pub fn acceptStatus(self: *Self, status: IO.RuntimeStatus, packet: *IO.LiveConfigPacket) void {
    const first = self.status == null;
    const old_revision = packet.revision;
    self.status = status;
    self.last_status = rl.getTime();
    if (status.run_id != self.direct_run_id or status.state != .running or
        !status.active or status.active_config.mission_id != .thruster_control)
    {
        self.direct_command = null;
        if (status.run_id != self.direct_run_id) {
            self.direct_run_id = status.run_id;
            self.direct_sequence = 0;
            self.direct_values = @splat(0);
        }
    }
    packet.* = status.desired;
    if (self.pending_request) |pending| {
        if (status.accepted_client_id == self.client_id and status.accepted_request_id >= pending.request_id) self.pending_request = null;
    }
    if (self.pending_config) |pending| {
        if (status.config_reply_revision == pending.revision) {
            self.pending_config = null;
            if (status.config_rejected) {
                self.error_text = status.last_error;
            } else {
                if (self.applying_draft) self.dirty = false;
                self.error_text = .{};
            }
            self.applying_draft = false;
        }
    }
    if (first or !self.dirty) self.draft = status.desired else if (old_revision != status.revision) {
        IO.setText(&self.error_text, "Configuration changed; Revert to refresh the draft before applying.");
    }
}

fn text(font: rl.Font, value: []const u8, x: f32, y: f32, tint: rl.Color) void {
    var buffer: [768:0]u8 = @splat(0);
    const len = @min(value.len, buffer.len - 1);
    @memcpy(buffer[0..len], value[0..len]);
    rl.drawTextEx(font, buffer[0..len :0], .{ .x = x, .y = y }, 16, 0, tint);
}

fn button(font: rl.Font, label: []const u8, bounds: rl.Rectangle, enabled: bool) bool {
    const hovered = rl.checkCollisionPointRec(rl.getMousePosition(), bounds);
    rl.drawRectangleRec(bounds, if (enabled and hovered) .{ .r = 40, .g = 95, .b = 89, .a = 255 } else .{ .r = 20, .g = 43, .b = 53, .a = 255 });
    text(font, label, bounds.x + 8, bounds.y + 5, if (enabled) rl.Color.ray_white else rl.Color.gray);
    return enabled and hovered and rl.isMouseButtonPressed(.left);
}

fn editText(self: *Self, font: rl.Font, label: []const u8, value: anytype, index: usize, bounds: rl.Rectangle) void {
    text(font, label, bounds.x, bounds.y + 5, rl.Color.gray);
    const field: rl.Rectangle = .{ .x = bounds.x + 170, .y = bounds.y, .width = bounds.width - 170, .height = bounds.height };
    rl.drawRectangleRec(field, if (self.editing == index) .{ .r = 40, .g = 70, .b = 78, .a = 255 } else .{ .r = 18, .g = 36, .b = 45, .a = 255 });
    if (rl.isMouseButtonPressed(.left) and rl.checkCollisionPointRec(rl.getMousePosition(), field)) {
        self.editing = index;
        self.select_all = false;
    }
    if (self.editing == index) {
        if ((rl.isKeyDown(.left_control) or rl.isKeyDown(.right_control)) and rl.isKeyPressed(.a)) self.select_all = true;
        while (true) {
            const code = rl.getCharPressed();
            if (code == 0) break;
            if (code >= 32 and code < 127 and value.len < value.bytes.len) {
                if (self.select_all) {
                    value.* = .{};
                    self.select_all = false;
                }
                value.bytes[value.len] = @intCast(code);
                value.len += 1;
                if (index < config_field_count) self.dirty = true;
            }
        }
        if (rl.isKeyPressed(.backspace) and value.len > 0) {
            if (self.select_all) {
                value.* = .{};
                self.select_all = false;
            } else {
                value.len -= 1;
                value.bytes[value.len] = 0;
            }
            if (index < config_field_count) self.dirty = true;
        }
        if (rl.isKeyPressed(.enter) or rl.isKeyPressed(.escape)) self.editing = null;
    }
    rl.beginScissorMode(@intFromFloat(field.x), @intFromFloat(field.y), @intFromFloat(field.width), @intFromFloat(field.height));
    text(font, value.slice() catch "invalid text", field.x + 6, field.y + 5, rl.Color.ray_white);
    rl.endScissorMode();
}

pub fn zeroThrusters(self: *Self) void {
    self.direct_values = @splat(0);
}

pub fn publishThrusters(self: *Self, status: IO.RuntimeStatus) void {
    if (status.state != .running or !status.active or status.active_config.mission_id != .thruster_control) return;
    const count = status.active_config.tam_len;
    var values: [Config.thruster_count]f32 = @splat(0);
    for (0..count) |i| {
        const value = self.direct_values[i];
        if (!std.math.isFinite(value) or value < -1 or value > 1) return;
        values[i] = value;
    }
    self.direct_sequence += 1;
    self.direct_command = .{
        .run_id = status.run_id,
        .client_id = self.client_id,
        .sequence = self.direct_sequence,
        .count = count,
        .values = values,
    };
    self.error_text = .{};
}

pub fn draw(self: *Self, font: rl.Font, bounds: rl.Rectangle, packet: *const IO.LiveConfigPacket, frame: *const IO.TelemetryFrame, telemetry_live: bool, embedded: bool) void {
    const x = bounds.x;
    const w = bounds.width;
    const ready = self.synchronized();
    var y = bounds.y;
    if (self.status) |status| {
        const label = rl.textFormat("State: %s  /  Result: %s  /  Revision: %llu", .{ @tagName(status.state).ptr, @tagName(status.result).ptr, status.revision });
        text(font, label, x, y, if (status.result == .failed) rl.Color.orange else if (ready) rl.Color.lime else rl.Color.orange);
        y += 26;
        text(font, rl.textFormat("Desired: %s / %s  |  Active: %s / %s", .{ @tagName(status.desired.mission_id).ptr, @tagName(status.desired.mission_profile).ptr, if (status.active) @tagName(status.active_config.mission_id).ptr else "none", if (status.active) @tagName(status.active_config.mission_profile).ptr else "none" }), x, y, rl.Color.ray_white);
        y += 26;
        text(font, status.last_error.slice() catch "", x, y, rl.Color.orange);
    } else text(font, "Connecting to runtime control...", x, y, rl.Color.orange);
    y += 36;
    for ([_][]const u8{ "Apply", "Revert", "Start", "Stop", "Restart" }, 0..) |label, i| {
        if (button(font, label, .{ .x = x + @as(f32, @floatFromInt(i)) * 112, .y = y, .width = 104, .height = 30 }, ready)) switch (i) {
            0 => {
                if (self.draft.revision != packet.revision) IO.setText(&self.error_text, "Draft is stale; Revert before applying.") else {
                    self.queueConfig(self.draft);
                    self.applying_draft = self.pending_config != null;
                }
            },
            1 => {
                self.draft = packet.*;
                self.dirty = false;
                self.editing = null;
                self.error_text = .{};
            },
            2 => self.request(.start),
            3 => self.request(.stop),
            4 => self.request(.restart),
            else => unreachable,
        };
    }
    y += 40;
    text(font, self.error_text.slice() catch "", x, y, rl.Color.orange);
    y += 26;
    if (button(font, rl.textFormat("Mission: %s", .{@tagName(self.draft.mission_id).ptr}), .{ .x = x, .y = y, .width = w * 0.48, .height = 28 }, ready)) {
        self.draft.mission_id = @fromBackingInt(@intCast((@backingInt(self.draft.mission_id) + 1) % @typeInfo(Config.MissionId).@"enum".field_names.len));
        self.dirty = true;
    }
    if (button(font, rl.textFormat("Profile: %s", .{@tagName(self.draft.mission_profile).ptr}), .{ .x = x + w * 0.5, .y = y, .width = w * 0.5, .height = 28 }, ready)) {
        const profile: Config.MissionProfileId = if (self.draft.mission_profile == .proteus_hwd) .stonefish_sim else .proteus_hwd;
        const preset = Config.compiledProfile(profile);
        self.draft.mission_profile = profile;
        const wire = IO.LiveConfigPacket.fromRuntimeConfig(preset);
        self.draft.tam = wire.tam;
        self.draft.tam_len = wire.tam_len;
        self.draft.camera_forward = wire.camera_forward;
        self.draft.rotation_limit = wire.rotation_limit;
        self.draft.log_freq_div = wire.log_freq_div;
        self.draft.queue_capacity = wire.queue_capacity;
        self.draft.print_stuff = wire.print_stuff;
        self.draft.clear_print_stuff = wire.clear_print_stuff;
        self.draft.odometry = wire.odometry;
        self.draft.ibvs = wire.ibvs;
        self.draft.thrustor_saturate = wire.thrustor_saturate;
        self.draft.goal_dist_threshold = wire.goal_dist_threshold;
        self.draft.sim_robot = wire.sim_robot;
        self.dirty = true;
    }
    y += 36;
    if (self.status) |status| {
        if (status.active and status.active_config.mission_id == .thruster_control) {
            _ = frame;
            _ = telemetry_live;
            text(font, "Stop the mission to edit sensor and transport settings.", x, y, rl.Color.gray);
            return;
        }
    }
    text(font, rl.textFormat("Sensor source: %s (compiled)", .{@tagName(self.draft.sensor_source).ptr}), x, y + 5, rl.Color.gray);
    if (embedded) {
        text(font, "Simulator pacing: wall clock", x + w * 0.5, y + 5, rl.Color.gray);
    } else if (button(font, if (self.draft.sim_realtime) "Simulator pacing: wall clock" else "Simulator pacing: accelerated", .{ .x = x + w * 0.5, .y = y, .width = w * 0.5, .height = 28 }, ready)) {
        self.draft.sim_realtime = !self.draft.sim_realtime;
        self.dirty = true;
    }
    y += 36;
    inline for (.{
        "sensor_library",
        "serial_port",
        "sim_robot",
        "sim_scene",
        "telemetry_addr",
        "telemetry_log_path",
    }, 0..) |name, i| {
        const labels = [_][]const u8{
            "Sensor library",
            "Serial port",
            "Robot",
            "Scene",
            "Telemetry address",
            "Log file",
        };
        self.editText(font, labels[i], &@field(self.draft, name), i, .{ .x = x, .y = y, .width = w, .height = 26 });
        y += 30;
    }
    if (button(font, rl.textFormat("Telemetry transport: %s", .{@tagName(self.draft.telemetry_transport).ptr}), .{ .x = x, .y = y, .width = w, .height = 28 }, ready)) {
        self.draft.telemetry_transport = switch (self.draft.telemetry_transport) {
            .disabled => .udp,
            .udp => .stdout,
            .stdout => .disabled,
        };
        self.dirty = true;
    }
}
