//! Proteus firmware protocol v2. No native layout or host-endian serialization.
const std = @import("std");

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

test "protocol has fixed little-endian wire bytes and CCITT CRC" {
    try std.testing.expectEqual(protocol_v2_crc16_check_value, crc16("123456789"));
    const bytes = encode(&.{ 0, 1, -1, 0.5, -0.5, 0 });
    try std.testing.expectEqualSlices(u8, &.{ 0xa5, 0x5a, 6, 0, 0, 0, 0, 0, 0, 0x80, 0x3f, 0, 0, 0x80, 0xbf }, bytes[0..15]);
    // Serialization preserves the supplied bits; validation belongs to firmware.
    const raw = encode(&.{ std.math.nan(f32), std.math.inf(f32), -2, 2, 0, 0 });
    try std.testing.expectEqual(@as(u32, @bitCast(std.math.nan(f32))), std.mem.readInt(u32, raw[3..7], .little));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, -2))), std.mem.readInt(u32, raw[11..15], .little));
}

test "reply parser accepts fragmented frames and rejects corrupt replies" {
    var parser: Parser = .{};
    const ready = [_]u8{ 0xa5, 0x5a, 0x80, 0x78, 0x70 };
    try std.testing.expectEqual(@as(u16, 0x7078), crc16(ready[2..3]));
    for ([_]u8{ 0, 0xa5, 0xa5 }) |byte| _ = try parser.feed(byte);
    for (ready[1..4]) |byte| try std.testing.expect(try parser.feed(byte) == null);
    try std.testing.expect((try parser.feed(ready[4])).? == .ready);
    for (ready[0..4]) |byte| _ = try parser.feed(byte);
    try std.testing.expectError(error.InvalidReplyCrc, parser.feed(0));
}

test "reply parser resynchronizes overlapping prefixes and malformed frame suffixes" {
    var parser: Parser = .{};
    const ready = [_]u8{ 0xa5, 0x5a, 0x80, 0x78, 0x70 };
    // The invalid kind byte is also the next frame's first sync byte.
    for ([_]u8{ 0xa5, 0x5a }) |byte| _ = try parser.feed(byte);
    try std.testing.expectError(error.InvalidReply, parser.feed(0xa5));
    for (ready[1..4]) |byte| try std.testing.expect(try parser.feed(byte) == null);
    try std.testing.expect((try parser.feed(ready[4])).? == .ready);
    // The corrupt CRC ends with the beginning of a valid frame.
    for ([_]u8{ 0xa5, 0x5a, 0xe0, 2, 3, 0xa5 }) |byte| _ = try parser.feed(byte);
    try std.testing.expectError(error.InvalidReplyCrc, parser.feed(0x5a));
    for (ready[2..4]) |byte| try std.testing.expect(try parser.feed(byte) == null);
    try std.testing.expect((try parser.feed(ready[4])).? == .ready);
    // Arbitrary input always keeps the parser's state inside its fixed buffer.
    var random = std.Random.DefaultPrng.init(42);
    for (0..10_000) |_| {
        _ = parser.feed(random.random().int(u8)) catch null;
        try std.testing.expect(parser.len < protocol_v2_max_reply_frame_size);
    }
}
