//! Nonblocking Linux serial transport and firmware READY/reply handling.
const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;
const Protocol = @import("Protocol.zig");
const MissionIO = @import("MissionIO.zig");
const MissionConfig = @import("MissionConfig.zig");
const Serial = @This();

const ready_poll_interval_ms: u64 = 10;
const ready_max_retries: usize = 1000;

io: Io,
file: Io.File,

/// An empty path disables serial output; other paths must open and configure.
pub fn init(io: Io, path: [:0]const u8) !?Serial {
    if (path.len == 0) return null;
    if (std.mem.indexOfScalar(u8, path, 0) != null) {
        return error.InvalidSerialPath;
    }
    // Io.Dir.openFile has no O_NONBLOCK option. Set it at open time so a
    // serial device cannot block startup waiting for a carrier signal.
    const fd: Io.File.Handle = while (true) {
        const result = linux.open(path, .{ .ACCMODE = .RDWR, .NOCTTY = true, .NONBLOCK = true, .CLOEXEC = true }, 0);
        switch (linux.errno(result)) {
            .SUCCESS => break @intCast(result),
            .INTR => continue,
            else => return error.SerialOpenFailed,
        }
    };
    const file: Io.File = .{ .handle = fd, .flags = .{ .nonblocking = true } };
    errdefer file.close(io);

    var settings = std.mem.zeroes(linux.termios);
    const get = try io.operate(.{ .device_io_control = .{ .file = file, .code = linux.T.CGETS, .arg = &settings } });
    if (get.device_io_control < 0) return error.SerialConfigureFailed;

    // Raw binary input, with software flow control disabled.
    settings.iflag.IGNBRK = false;
    settings.iflag.BRKINT = false;
    settings.iflag.PARMRK = false;
    settings.iflag.ISTRIP = false;
    settings.iflag.INLCR = false;
    settings.iflag.IGNCR = false;
    settings.iflag.ICRNL = false;
    settings.iflag.IXON = false;
    settings.iflag.IXOFF = false;
    settings.iflag.IXANY = false;
    settings.oflag.OPOST = false;
    settings.lflag.ECHO = false;
    settings.lflag.ECHONL = false;
    settings.lflag.ICANON = false;
    settings.lflag.ISIG = false;
    settings.lflag.IEXTEN = false;

    // 115200 baud, eight data bits, no parity, one stop bit, no RTS/CTS.
    // Preserve the device's hang-up-on-close policy.
    settings.cflag = .{ .CSIZE = .CS8, .CREAD = true, .CLOCAL = true, .HUPCL = settings.cflag.HUPCL };
    settings.cflag = @bitCast(@as(linux.tcflag_t, @bitCast(settings.cflag)) | @backingInt(linux.speed_t.B115200));
    settings.cc[@backingInt(linux.V.MIN)] = 0;
    settings.cc[@backingInt(linux.V.TIME)] = 0;
    const set = try io.operate(.{ .device_io_control = .{ .file = file, .code = linux.T.CSETS, .arg = &settings } });
    if (set.device_io_control < 0) return error.SerialConfigureFailed;

    return Serial{ .io = io, .file = file };
}

/// Wait for a complete READY reply while processing runtime control messages.
pub fn waitForReady(self: *Serial, parser: *Protocol.Parser, telemetry: *MissionIO.Telemetry, config: *MissionConfig.MissionConfig) !void {
    var retries: usize = 0;
    while (true) {
        try telemetry.pollControl(self.io, config);
        if (try self.readReplies(parser)) return;
        if (retries == ready_max_retries) return error.SerialReadyTimeout;
        retries += 1;
        try Io.sleep(self.io, .fromMilliseconds(ready_poll_interval_ms), .awake);
    }
}

/// Read one bounded batch so reply traffic cannot starve control messages.
pub fn readReplies(self: *Serial, parser: *Protocol.Parser) !bool {
    var bytes: [256]u8 = undefined;
    const count = try self.read(&bytes);
    var ready = false;
    for (bytes[0..count]) |byte| {
        const reply = parser.feed(byte) catch |err| {
            std.log.warn("serial reply: {s}", .{@errorName(err)});
            continue;
        };
        if (reply) |value| switch (value) {
            .ready => ready = true,
            .firmware_error => |detail| std.log.warn("firmware error {d} (detail {d})", .{ detail.code, detail.detail }),
        };
    }
    return ready;
}

/// Return immediately with zero when no bytes are available.
pub fn read(self: *Serial, buffer: []u8) !usize {
    if (buffer.len == 0) return 0;
    // A zero timeout checks readiness without waiting. This distinguishes an
    // idle VMIN=0 terminal from end-of-stream after a disconnect.
    const result = self.io.operateTimeout(.{ .file_read_streaming = .{ .file = self.file, .data = &.{buffer} } }, .{ .duration = .{ .clock = .awake, .raw = .zero } }) catch |err| switch (err) {
        error.Timeout => return 0,
        error.Canceled => return error.Canceled,
        else => return error.SerialReadFailed,
    };
    return result.file_read_streaming catch |err| switch (err) {
        error.WouldBlock => 0,
        error.EndOfStream, error.InputOutput, error.ConnectionResetByPeer, error.NotOpenForReading => error.SerialDisconnected,
        else => error.SerialReadFailed,
    };
}

/// Finish these bytes within one deadline, including all partial writes and retries.
pub fn writeAll(self: *Serial, bytes: []const u8, timeout_ms: u32) !void {
    const deadline = Io.Clock.Timestamp.fromNow(self.io, .{ .clock = .awake, .raw = .fromMilliseconds(timeout_ms) });
    var offset: usize = 0;
    while (offset < bytes.len) {
        if (deadline.durationFromNow(self.io).raw.nanoseconds <= 0) return error.SerialWriteTimeout;
        const result = self.io.operateTimeout(.{ .file_write_streaming = .{ .file = self.file, .data = &.{bytes[offset..]} } }, .{ .deadline = deadline }) catch |err| switch (err) {
            // Some backends round the wait down to whole milliseconds. Recheck
            // the same deadline rather than starting another timeout.
            error.Timeout => continue,
            error.Canceled => return error.Canceled,
            else => return error.SerialWriteFailed,
        };
        const count = result.file_write_streaming catch |err| switch (err) {
            error.WouldBlock => continue,
            error.InputOutput, error.BrokenPipe, error.NoDevice, error.NotOpenForWriting => return error.SerialDisconnected,
            else => return error.SerialWriteFailed,
        };
        if (count == 0) return error.SerialDisconnected;
        offset += count;
    }
}

pub fn deinit(self: *Serial) void {
    self.file.close(self.io);
    self.* = undefined;
}
