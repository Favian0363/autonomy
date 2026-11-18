const std = @import("std");
const rl = @import("raylib");
const IO = @import("MissionIO");
const Config = IO.CompiledConfig;
const Self = @This();

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
last_send: f64 = -1,
last_status: f64 = -1,
direct_fields: [Config.thruster_count]Config.FixedText(16) = @splat(.fromLiteral("0")),
direct_command: ?IO.ThrusterCommand = null,
direct_sequence: u64 = 0,
direct_run_id: u64 = 0,
error_text: Config.FixedText(256) = .{},

pub fn synchronized(self: *const Self) bool {
    return self.status != null and rl.getTime() - self.last_status < 2;
}

fn send(io: std.Io, socket: *std.Io.net.Socket, dest: *const std.Io.net.IpAddress, value: anytype) !void {
    var buffer: [4096]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    try IO.SerialBinary.write(&writer, value);
    try socket.send(io, dest, writer.buffered());
}

pub fn queueConfig(self: *Self, packet: IO.LiveConfigPacket) void {
    if (self.synchronized() and self.pending_config == null) self.pending_config = packet;
}

fn request(self: *Self, action: IO.RuntimeAction) void {
    if (!self.synchronized()) return;
    self.request_id += 1;
    self.pending_request = .{ .client_id = self.client_id, .request_id = self.request_id, .action = action };
}

/// Returns true when an authoritative configuration changed.
pub fn update(self: *Self, io: std.Io, socket: *std.Io.net.Socket, dest: *const std.Io.net.IpAddress, packet: *IO.LiveConfigPacket) bool {
    if (self.client_id == 0) {
        std.Io.random(io, std.mem.asBytes(&self.client_id));
        self.client_id |= 1;
    }
    var changed = false;
    var buffer: [8192]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{ .clock = .awake, .raw = .fromNanoseconds(0) } };
    for (0..32) |_| {
        const message = socket.receiveTimeout(io, &buffer, timeout) catch break;
        const parsed = IO.SerialBinary.readFromBuffer(IO.RuntimeStatus, message.data) catch continue;
        if (parsed.consumed != message.data.len or !std.mem.eql(u8, &parsed.value.magic, "ACS1")) continue;
        const status = parsed.value;
        status.desired.validate() catch continue;
        const first = self.status == null;
        changed = changed or first or status.revision != packet.revision;
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
                self.direct_fields = @splat(.fromLiteral("0"));
                if (self.editing != null and self.editing.? >= 7) self.editing = null;
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
    const now = rl.getTime();
    if (now - self.last_send >= 0.25) {
        self.last_send = now;
        if (self.pending_config) |pending| send(io, socket, dest, pending) catch {};
        if (self.synchronized()) {
            if (self.direct_command) |command| send(io, socket, dest, command) catch {};
        }
        const query: IO.RuntimeRequest = self.pending_request orelse .{ .client_id = self.client_id, .request_id = 0, .action = .query };
        send(io, socket, dest, query) catch {};
    }
    return changed;
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
                if (index < 7) self.dirty = true;
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
            if (index < 7) self.dirty = true;
        }
        if (rl.isKeyPressed(.enter) or rl.isKeyPressed(.escape)) self.editing = null;
    }
    rl.beginScissorMode(@intFromFloat(field.x), @intFromFloat(field.y), @intFromFloat(field.width), @intFromFloat(field.height));
    text(font, value.slice() catch "invalid text", field.x + 6, field.y + 5, rl.Color.ray_white);
    rl.endScissorMode();
}

fn submitThrusters(self: *Self, status: IO.RuntimeStatus, zero: bool) void {
    var values: [Config.thruster_count]f32 = @splat(0);
    if (!zero) {
        for (0..status.active_config.tam_len) |i| {
            const value = std.fmt.parseFloat(f32, self.direct_fields[i].slice() catch "") catch {
                IO.setText(&self.error_text, "Enter a number between -1 and 1 for every thruster.");
                return;
            };
            if (!std.math.isFinite(value) or value < -1 or value > 1) {
                IO.setText(&self.error_text, "Enter a number between -1 and 1 for every thruster.");
                return;
            }
            values[i] = value;
        }
    } else self.direct_fields = @splat(.fromLiteral("0"));
    self.direct_sequence += 1;
    self.direct_command = .{ .run_id = status.run_id, .client_id = self.client_id, .sequence = self.direct_sequence, .count = status.active_config.tam_len, .values = values };
    self.last_send = -1;
    self.editing = null;
    self.error_text = .{};
}

pub fn draw(self: *Self, font: rl.Font, bounds: rl.Rectangle, packet: *const IO.LiveConfigPacket, frame: *const IO.TelemetryFrame, telemetry_live: bool) void {
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
    } else text(font, "Connecting to runtime control (independent of sensors)...", x, y, rl.Color.orange);
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
        self.draft.tam = preset.tam;
        self.draft.tam_len = preset.tam_len;
        self.draft.camera_forward = preset.camera_forward;
        self.draft.rotation_limit = preset.rotation_limit;
        self.draft.log_freq_div = preset.log_freq_div;
        self.draft.queue_capacity = preset.queue_capacity;
        self.draft.print_stuff = preset.print_stuff;
        self.draft.clear_print_stuff = preset.clear_print_stuff;
        self.draft.odometry = preset.odometry;
        self.draft.ibvs = preset.ibvs;
        self.draft.thrustor_saturate = preset.thrustor_saturate;
        self.draft.goal_dist_threshold = preset.goal_dist_threshold;
        self.draft.sim_robot = preset.sim_robot;
        self.dirty = true;
    }
    y += 36;
    if (self.status) |status| {
        if (status.active and status.active_config.mission_id == .thruster_control) {
            text(font, "Direct thrusters (-1 to 1): S submitted, O output", x, y, rl.Color.ray_white);
            y += 26;
            const count: usize = status.active_config.tam_len;
            for (0..count) |i| {
                const col: f32 = @floatFromInt(i % 2);
                const row: f32 = @floatFromInt(i / 2);
                const label = if (self.direct_command) |command|
                    if (telemetry_live and i < frame.thrusters_len)
                        rl.textFormat("T%d S%.2f O%.2f", .{ i + 1, command.values[i], frame.thrusters[i] })
                    else
                        rl.textFormat("T%d sent %.2f", .{ i + 1, command.values[i] })
                else
                    rl.textFormat("Thruster %d", .{i + 1});
                self.editText(font, label, &self.direct_fields[i], i + 7, .{ .x = x + col * w * 0.5, .y = y + row * 30, .width = w * 0.48, .height = 26 });
            }
            y += @as(f32, @floatFromInt((count + 1) / 2)) * 30 + 4;
            const can_send = ready and status.state == .running;
            if (button(font, "Send", .{ .x = x, .y = y, .width = 100, .height = 30 }, can_send)) self.submitThrusters(status, false);
            if (button(font, "All zero", .{ .x = x + 110, .y = y, .width = 120, .height = 30 }, can_send)) self.submitThrusters(status, true);
            if (self.direct_command) |command| {
                text(font, rl.textFormat("Holding command %llu; Stop ends output", .{command.sequence}), x + 245, y + 5, rl.Color.lime);
            } else text(font, "Waiting for Send", x + 245, y + 5, rl.Color.gray);
            y += 36;
            text(font, self.error_text.slice() catch "", x, y, rl.Color.orange);
            y += 24;
            text(font, "Stop the mission to edit sensor and transport settings.", x, y, rl.Color.gray);
            return;
        }
    }
    text(font, rl.textFormat("Sensor source: %s (compiled)", .{@tagName(self.draft.sensor_source).ptr}), x, y + 5, rl.Color.gray);
    if (button(font, if (self.draft.sim_realtime) "Simulator pacing: wall clock" else "Simulator pacing: accelerated", .{ .x = x + w * 0.5, .y = y, .width = w * 0.5, .height = 28 }, ready)) {
        self.draft.sim_realtime = !self.draft.sim_realtime;
        self.dirty = true;
    }
    y += 36;
    inline for (.{ "sensor_library", "serial_port", "sim_robot", "sim_scene", "telemetry_addr", "telemetry_shm_name", "telemetry_log_path" }, 0..) |name, i| {
        const labels = [_][]const u8{ "Sensor library", "Serial port", "Robot", "Scene", "Telemetry address", "Shared memory name", "Log file" };
        self.editText(font, labels[i], &@field(self.draft, name), i, .{ .x = x, .y = y, .width = w, .height = 26 });
        y += 30;
    }
    if (button(font, rl.textFormat("Telemetry transport: %s", .{@tagName(self.draft.telemetry_transport).ptr}), .{ .x = x, .y = y, .width = w, .height = 28 }, ready)) {
        self.draft.telemetry_transport = @fromBackingInt(@intCast((@backingInt(self.draft.telemetry_transport) + 1) % 3));
        self.dirty = true;
    }
}
