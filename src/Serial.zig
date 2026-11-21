//! Proteus firmware protocol v2 on a nonblocking Linux serial port.
//!
//! Frame layout matches `arduino/sketches/Proteus/Proteus.ino`. The host opens the
//! device, encodes thruster commands, and parses READY / error replies from the wire.
const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;

// Frame structure shared with arduino/sketches/Proteus/Proteus.ino.
pub const protocol_v2_sync_prefix_size: usize = 2;
pub const protocol_v2_frame_start_offset: usize = 0;
pub const protocol_v2_sync_byte_0_offset: usize = 0;
pub const protocol_v2_sync_byte_1_offset: usize = 1;
pub const protocol_v2_sync_byte_0: u8 = 0xa5;
pub const protocol_v2_sync_byte_1: u8 = 0x5a;
pub const protocol_v2_header_size: usize = 3;
pub const protocol_v2_crc_size: usize = 2;
pub const protocol_v2_packet_kind_or_count_offset: usize = protocol_v2_header_size - 1;
pub const protocol_v2_payload_offset: usize = protocol_v2_header_size;
pub const protocol_v2_crc_covered_offset: usize = protocol_v2_sync_prefix_size;
pub const protocol_v2_wire_byte_order: std.builtin.Endian = .little;
pub const protocol_v2_parser_empty_length: usize = 0;
pub const protocol_v2_parser_sync_first_byte_length: usize = protocol_v2_sync_prefix_size - 1;
pub const protocol_v2_parser_length_increment: usize = 1;

// Command packet fields and sizes.
pub const protocol_v2_command_channel_count: usize = 6;
pub const protocol_v2_float32_size_bytes: usize = 4;
pub const protocol_v2_command_payload_size: usize = protocol_v2_command_channel_count * protocol_v2_float32_size_bytes;
pub const protocol_v2_command_frame_size: usize = protocol_v2_header_size + protocol_v2_command_payload_size + protocol_v2_crc_size;

// Reply packet kinds, payload fields, and frame sizes.
pub const protocol_v2_ready_packet_kind: u8 = 0x80;
pub const protocol_v2_error_packet_kind: u8 = 0xe0;
pub const protocol_v2_ready_payload_size: usize = 0;
pub const protocol_v2_error_payload_size: usize = 2;
pub const protocol_v2_error_code_offset: usize = protocol_v2_payload_offset;
pub const protocol_v2_error_detail_offset: usize = protocol_v2_error_code_offset + 1;
pub const protocol_v2_ready_frame_size: usize = protocol_v2_header_size + protocol_v2_ready_payload_size + protocol_v2_crc_size;
pub const protocol_v2_error_frame_size: usize = protocol_v2_header_size + protocol_v2_error_payload_size + protocol_v2_crc_size;
pub const protocol_v2_max_reply_frame_size: usize = @max(protocol_v2_ready_frame_size, protocol_v2_error_frame_size);

// CRC-16/CCITT parameters used by the v2 firmware.
pub const protocol_v2_crc16_initial_value: u16 = 0xffff;
pub const protocol_v2_crc16_polynomial: u16 = 0x1021;
pub const protocol_v2_crc16_high_bit_mask: u16 = 0x8000;
pub const protocol_v2_crc16_bits_per_byte: usize = 8;
pub const protocol_v2_crc16_byte_shift: u4 = 8;
pub const protocol_v2_crc16_single_bit_shift: u4 = 1;
pub const protocol_v2_crc16_check_value: u16 = 0x29b1;

pub const Reply = union(enum) {
    ready,
    firmware_error: struct { code: u8, detail: u8 },
};

pub fn crc16(bytes: []const u8) u16 {
    var crc = protocol_v2_crc16_initial_value;
    for (bytes) |byte| {
        crc ^= @as(u16, byte) << protocol_v2_crc16_byte_shift;
        for (0..protocol_v2_crc16_bits_per_byte) |_| {
            crc = if (crc & protocol_v2_crc16_high_bit_mask != 0)
                (crc << protocol_v2_crc16_single_bit_shift) ^ protocol_v2_crc16_polynomial
            else
                crc << protocol_v2_crc16_single_bit_shift;
        }
    }
    return crc;
}

pub fn encode(values: *const [protocol_v2_command_channel_count]f32) [protocol_v2_command_frame_size]u8 {
    var packet: [protocol_v2_command_frame_size]u8 = undefined;
    packet[protocol_v2_frame_start_offset..protocol_v2_header_size].* = .{
        protocol_v2_sync_byte_0,
        protocol_v2_sync_byte_1,
        @intCast(protocol_v2_command_channel_count),
    };
    for (values, 0..) |value, channel| {
        const value_offset = protocol_v2_payload_offset + protocol_v2_float32_size_bytes * channel;
        std.mem.writeInt(u32, packet[value_offset..][0..protocol_v2_float32_size_bytes], @bitCast(value), protocol_v2_wire_byte_order);
    }
    const crc_offset = protocol_v2_command_frame_size - protocol_v2_crc_size;
    const crc_value = crc16(packet[protocol_v2_crc_covered_offset..crc_offset]);
    std.mem.writeInt(u16, packet[crc_offset..][0..protocol_v2_crc_size], crc_value, protocol_v2_wire_byte_order);
    return packet;
}

pub const Parser = struct {
    bytes: [protocol_v2_max_reply_frame_size]u8 = undefined,
    len: usize = protocol_v2_parser_empty_length,

    // Preserve a suffix that can begin the next frame after a corrupt reply.
    fn resync(self: *Parser) void {
        const length = self.len;
        self.len = 0;
        for (1..length) |i| {
            if (self.bytes[i] != protocol_v2_sync_byte_0) continue;
            if (i + 1 == length or (self.bytes[i + 1] == protocol_v2_sync_byte_1 and
                (i + 2 == length or self.bytes[i + 2] == protocol_v2_ready_packet_kind or self.bytes[i + 2] == protocol_v2_error_packet_kind)))
            {
                if (i + 2 < length) {
                    const size = if (self.bytes[i + 2] == protocol_v2_ready_packet_kind) protocol_v2_ready_frame_size else protocol_v2_error_frame_size;
                    if (length - i >= size) continue;
                }
                std.mem.copyForwards(u8, &self.bytes, self.bytes[i..length]);
                self.len = length - i;
                return;
            }
        }
    }

    pub fn feed(self: *Parser, byte: u8) !?Reply {
        if (self.len == protocol_v2_parser_empty_length) {
            if (byte == protocol_v2_sync_byte_0) {
                self.bytes[protocol_v2_sync_byte_0_offset] = byte;
                self.len = protocol_v2_parser_sync_first_byte_length;
            }
            return null;
        }
        if (self.len == protocol_v2_parser_sync_first_byte_length) {
            if (byte == protocol_v2_sync_byte_1) {
                self.bytes[protocol_v2_sync_byte_1_offset] = byte;
                self.len = protocol_v2_sync_prefix_size;
            } else if (byte != protocol_v2_sync_byte_0) {
                self.len = protocol_v2_parser_empty_length;
            }
            return null;
        }

        self.bytes[self.len] = byte;
        self.len += protocol_v2_parser_length_increment;
        if (self.len == protocol_v2_header_size) {
            const packet_kind = self.bytes[protocol_v2_packet_kind_or_count_offset];
            if (packet_kind != protocol_v2_ready_packet_kind and packet_kind != protocol_v2_error_packet_kind) {
                self.resync();
                return error.InvalidReply;
            }
        }
        if (self.len < protocol_v2_header_size) return null;

        const packet_kind = self.bytes[protocol_v2_packet_kind_or_count_offset];
        const frame_size = if (packet_kind == protocol_v2_ready_packet_kind)
            protocol_v2_ready_frame_size
        else
            protocol_v2_error_frame_size;
        if (self.len < frame_size) return null;
        self.len = protocol_v2_parser_empty_length;

        const crc_offset = frame_size - protocol_v2_crc_size;
        const received_crc = std.mem.readInt(u16, self.bytes[crc_offset..][0..protocol_v2_crc_size], protocol_v2_wire_byte_order);
        if (received_crc != crc16(self.bytes[protocol_v2_crc_covered_offset..crc_offset])) {
            self.len = frame_size;
            self.resync();
            return error.InvalidReplyCrc;
        }
        return if (packet_kind == protocol_v2_ready_packet_kind)
            .ready
        else
            .{ .firmware_error = .{
                .code = self.bytes[protocol_v2_error_code_offset],
                .detail = self.bytes[protocol_v2_error_detail_offset],
            } };
    }
};

pub const Port = struct {
    io: Io,
    file: Io.File,

    pub fn init(io: Io, path: [:0]const u8) !?Port {
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

        return Port{ .io = io, .file = file };
    }

    /// Read one bounded batch so reply traffic cannot starve control messages.
    pub fn readReplies(self: *Port, parser: *Parser) !bool {
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
    pub fn read(self: *Port, buffer: []u8) !usize {
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
    pub fn writeAll(self: *Port, bytes: []const u8, timeout_ms: u32) !void {
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

    pub fn deinit(self: *Port) void {
        self.file.close(self.io);
        self.* = undefined;
    }
};
