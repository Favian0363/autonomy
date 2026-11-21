const std = @import("std");
const rl = @import("raylib");
const Visualizer = @import("Visualizer");
const MissionIO = Visualizer.MissionIO;
const MissionConfig = MissionIO.CompiledConfig;
const Io = std.Io;

extern fn video_default_source() callconv(.c) [*:0]const u8;
extern fn video_open(source: [*:0]const u8, width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_get_size(width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_frame(out: [*]u8, capacity: usize, width: c_int, height: c_int) callconv(.c) bool;
extern fn video_has_error() callconv(.c) bool;
extern fn video_close() callconv(.c) void;

const Config = struct {
    video_source: [:0]const u8,
    mission_control: bool = MissionConfig.startup_mission == .mission_control,
    teleop_host: []const u8 = "127.0.0.1",
};

fn parseArgs(args: []const [:0]const u8) !Config {
    var config = Config{ .video_source = std.mem.span(video_default_source()) };
    var i: usize = 1;
    while (i < args.len) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--video")) {
            i += 1;
            if (i >= args.len) return error.MissingVideoSource;
            config.video_source = args[i];
        } else if (std.mem.eql(u8, arg, "--teleop-host")) {
            i += 1;
            if (i >= args.len) return error.MissingTeleopHost;
            config.teleop_host = args[i];
        } else if (std.mem.eql(u8, arg, "--mission-control")) {
            config.mission_control = true;
        } else if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
            std.debug.print(
                "usage: viewer [--video source|none] [--teleop-host IP] [--mission-control]\n",
                .{},
            );
            return error.HelpRequested;
        } else {
            std.debug.print("unknown argument: {s}\n", .{arg});
            return error.UnknownArgument;
        }
        i += 1;
    }
    return config;
}

fn videoDisabled(source: []const u8) bool {
    return std.mem.eql(u8, source, "none") or
        std.mem.eql(u8, source, "off") or
        std.mem.eql(u8, source, "-");
}

const nonblocking: Io.Timeout = .{ .duration = .{ .clock = .awake, .raw = .zero } };

const TelemetryInput = struct {
    socket: ?Io.net.Socket = null,
    transport: MissionConfig.TelemetryTransport = .disabled,
    addr: MissionConfig.FixedText(MissionConfig.telemetry_addr_max) = .{},
    frame: MissionIO.TelemetryFrame = undefined,
    last_packet: f64 = -1,
    last_bind: f64 = -10,

    fn deinit(self: *TelemetryInput, io: Io) void {
        if (self.socket) |socket| socket.close(io);
        self.socket = null;
    }

    fn update(self: *TelemetryInput, io: Io, config: *const MissionIO.LiveConfigPacket, now: f64) Visualizer.Status {
        if (self.transport != config.telemetry_transport or !MissionConfig.textEqual(self.addr, config.telemetry_addr)) {
            self.deinit(io);
            self.transport = config.telemetry_transport;
            self.addr = config.telemetry_addr;
            self.last_packet = -1;
            self.last_bind = -10;
        }
        if (self.transport != .udp) return .unavailable;
        if (self.socket == null and now - self.last_bind >= 1) {
            self.last_bind = now;
            const address = Io.net.IpAddress.parseLiteral(self.addr.slice() catch "") catch return .unavailable;
            self.socket = address.bind(io, .{ .mode = .dgram, .protocol = .udp }) catch return .unavailable;
        }
        const socket = self.socket orelse return .unavailable;
        var buffer: [64 * 1024]u8 = undefined;
        for (0..64) |_| {
            const message = socket.receiveTimeout(io, &buffer, nonblocking) catch break;
            var frame: MissionIO.TelemetryFrame = undefined;
            MissionIO.decodeTelemetryFrame(message.data, &frame) catch continue;
            self.frame = frame;
            self.last_packet = now;
        }
        return if (self.last_packet < 0) .connecting else if (now - self.last_packet > 1) .stale else .live;
    }
};

const VideoCapture = struct {
    source: [:0]const u8,
    started: bool = false,
    last_try: f64 = -10,
    last_frame: f64 = -1,
    image: ?rl.Image = null,
    view: Visualizer.Video = .{},

    fn deinit(self: *VideoCapture) void {
        if (self.view.texture.id != 0) rl.unloadTexture(self.view.texture);
        if (self.image) |image| rl.unloadImage(image);
        if (self.view.enabled) video_close();
    }

    fn update(self: *VideoCapture, now: f64) Visualizer.Video {
        if (!self.view.enabled) return self.view;
        if (!self.started and now - self.last_try > 2) {
            self.last_try = now;
            var width: c_int = 0;
            var height: c_int = 0;
            self.started = video_open(self.source.ptr, &width, &height);
        }
        if (self.started) {
            var width: c_int = 0;
            var height: c_int = 0;
            if (video_get_size(&width, &height) and width > 0 and height > 0 and
                (width != self.view.width or height != self.view.height))
            {
                const image = rl.genImageColor(width, height, rl.Color.black);
                if (rl.loadTextureFromImage(image)) |texture| {
                    if (self.view.texture.id != 0) rl.unloadTexture(self.view.texture);
                    if (self.image) |old| rl.unloadImage(old);
                    self.image = image;
                    self.view.texture = texture;
                    self.view.width = width;
                    self.view.height = height;
                    self.last_frame = -1;
                } else |_| rl.unloadImage(image);
            }
        }
        if (self.image) |image| {
            const pixels: [*]u8 = @ptrCast(image.data);
            const capacity = @as(usize, @intCast(self.view.width)) * @as(usize, @intCast(self.view.height)) * 4;
            if (video_frame(pixels, capacity, self.view.width, self.view.height)) {
                rl.updateTexture(self.view.texture, pixels);
                self.last_frame = now;
            }
        }
        self.view.status = if (video_has_error()) .unavailable else if (self.last_frame < 0) .connecting else if (now - self.last_frame > 1) .stale else .live;
        return self.view;
    }
};

fn send(io: Io, socket: Io.net.Socket, destination: *const Io.net.IpAddress, value: anytype) !void {
    var buffer: [4096]u8 = undefined;
    var writer: Io.Writer = .fixed(&buffer);
    if (@TypeOf(value) == MissionIO.MissionCommand) {
        const packet = try MissionIO.encodeMissionCommand(&buffer, value);
        try socket.send(io, destination, packet);
    } else {
        try MissionIO.SerialBinary.write(&writer, value);
        try socket.send(io, destination, writer.buffered());
    }
}

fn pollStatus(io: Io, socket: Io.net.Socket, viewer: *Visualizer) void {
    var buffer: [8192]u8 = undefined;
    for (0..32) |_| {
        const message = socket.receiveTimeout(io, &buffer, nonblocking) catch break;
        const parsed = MissionIO.SerialBinary.readFromBuffer(MissionIO.RuntimeStatus, message.data) catch continue;
        if (parsed.consumed != message.data.len or !std.mem.eql(u8, &parsed.value.magic, "ACS1")) continue;
        parsed.value.desired.validate() catch continue;
        viewer.acceptStatus(parsed.value);
    }
}

pub fn main(init: std.process.Init) !void {
    const args = try init.minimal.args.toSlice(init.arena.allocator());
    const config = parseArgs(args) catch |err| switch (err) {
        error.HelpRequested => return,
        else => return err,
    };
    const startup = MissionConfig.startup_config;
    var address_buf: [128]u8 = undefined;
    const destination = try Io.net.IpAddress.parseLiteral(try std.fmt.bufPrint(&address_buf, "{s}:{d}", .{ config.teleop_host, startup.teleop_port }));
    const bind: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
    const socket = try bind.bind(init.io, .{ .mode = .dgram, .protocol = .udp });
    defer socket.close(init.io);

    var client_id: u64 = 0;
    Io.random(init.io, std.mem.asBytes(&client_id));
    client_id |= 1;
    var viewer = try Visualizer.init(&startup, client_id, config.mission_control, false);
    defer viewer.deinit();
    defer if (viewer.neutral()) |command| send(init.io, socket, &destination, command) catch {};

    var input: TelemetryInput = .{};
    defer input.deinit(init.io);
    var video: VideoCapture = .{ .source = config.video_source, .view = .{ .enabled = !videoDisabled(config.video_source) } };
    defer video.deinit();

    const initial_config = MissionIO.LiveConfigPacket.fromRuntimeConfig(&startup);
    var last_send: f64 = -1;
    var send_error = false;
    while (!Visualizer.shouldClose()) {
        const now = rl.getTime();
        pollStatus(init.io, socket, &viewer);
        const active = if (viewer.runtimeStatus()) |status| status.active_config else initial_config;
        const frame_status = input.update(init.io, &active, now);
        const actions = try viewer.tick(.{
            .frame = if (input.last_packet >= 0) &input.frame else null,
            .telemetry_status = frame_status,
            .video = video.update(now),
            .send_error = send_error,
        });

        if (now - last_send >= 0.25) {
            last_send = now;
            send_error = false;
            if (actions.config) |packet| send(init.io, socket, &destination, packet) catch {
                send_error = true;
            };
            if (actions.thrusters) |command| send(init.io, socket, &destination, command) catch {
                send_error = true;
            };
            const request: MissionIO.RuntimeRequest = actions.lifecycle orelse .{ .client_id = client_id, .request_id = 0, .action = .query };
            send(init.io, socket, &destination, request) catch {
                send_error = true;
            };
        }
        if (actions.mission) |command| send(init.io, socket, &destination, command) catch {
            send_error = true;
        };
        if (actions.teleop) |command| send(init.io, socket, &destination, command) catch {
            send_error = true;
        };
    }
}
