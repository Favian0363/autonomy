const std = @import("std");
const Io = std.Io;

// Little-endian. A slice is a u8 count, then its items.
// Struct fields are written without the alignment padding of the C layout.
pub fn write(w: *Io.Writer, value: anytype) !void {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .int => {
            var buf: [@sizeOf(T)]u8 = undefined;
            std.mem.writeInt(T, &buf, value, .little);
            try w.writeAll(&buf);
        },
        .float => |info| {
            const Bits = std.meta.Int(.unsigned, info.bits);
            try write(w, @as(Bits, @bitCast(value)));
        },
        .bool => try write(w, @as(u8, @intFromBool(value))),
        .@"enum" => |info| try write(w, @as(info.tag_type, @intFromEnum(value))),
        .array => |info| {
            if (info.child == u8) {
                try w.writeAll(&value);
            } else {
                for (value) |item| try write(w, item);
            }
        },
        .vector => |info| {
            const array: [info.len]info.child = value;
            try write(w, array);
        },
        .pointer => |info| {
            if (info.size != .slice) @compileError("only slices can be written");
            if (value.len > std.math.maxInt(u8)) return error.TooLong;
            try write(w, @as(u8, @intCast(value.len)));
            if (info.child == u8) {
                try w.writeAll(value);
            } else {
                for (value) |item| try write(w, item);
            }
        },
        .@"struct" => |info| inline for (info.fields) |field| {
            try write(w, @field(value, field.name));
        },
        else => @compileError("unsupported serial type " ++ @typeName(T)),
    }
}
