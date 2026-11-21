
const std = @import("std");
const types = @import("types.zig");

pub const magic = "AUTODNA1";
pub const container_version: u32 = 1;

pub const max_types = 48;
pub const max_fields = 384;
pub const max_names = 256;
pub const max_tables = 16;

const header_size: usize = 20;
const dir_entry_size: usize = 16;
const type_rec_size: usize = 16;
const field_rec_size: usize = 8;

const eSDNA_Type = struct {
    const uint: u8 = 1;
    const sint: u8 = 2;
    const float: u8 = 3;
    const boolean: u8 = 4;
    const array: u8 = 5;
    const structure: u8 = 6;
};

const TypeDesc = struct {
    name: []const u8,
    kind: u8,
    size: u16,
    alignment: u8,
    elem_type: u16 = 0,
    elem_count: u16 = 0,
    field_start: u16 = 0,
    field_count: u16 = 0,
};

const FieldDesc = struct {
    name: []const u8,
    type_index: u16,
    offset: u16,
    size: u16,
};

const TableDesc = struct {
    name: []const u8 = "",
    type_index: u16 = 0,
    single: bool = false,
    stride: u32 = 0,
    alignment: u32 = 1,
};

const Schema = struct {
    name_len: usize,
    names: [max_names][]const u8,
    type_len: usize,
    types: [max_types]TypeDesc,
    field_len: usize,
    fields: [max_fields]FieldDesc,
    table_len: usize,
    tables: [max_tables]TableDesc,
};

const FileType = struct {
    name: []const u8,
    kind: u8,
    size: u16,
    alignment: u8,
    elem_type: u16,
    elem_count: u16,
    field_start: u16,
    field_count: u16,
};

const FileField = struct {
    name: []const u8,
    type_index: u16,
    offset: u16,
    size: u16,
};

const FileTable = struct {
    name: []const u8,
    type_index: u16,
    count: u32,
    offset: u32,
    stride: u32,
};

const Match = enum { identical, remap };

const State = struct {
    names: [max_names][]const u8 = @splat(""),
    name_len: usize = 0,
    type_keys: [max_types]type = undefined,
    types: [max_types]TypeDesc = undefined,
    type_len: usize = 0,
    fields: [max_fields]FieldDesc = undefined,
    field_len: usize = 0,
    visiting: [max_types]bool = @splat(false),

    fn internName(self: *State, name: []const u8) u16 {
        @setEvalBranchQuota(100_000);
        for (self.names[0..self.name_len], 0..) |existing, i| {
            if (std.mem.eql(u8, existing, name)) return @intCast(i);
        }
        if (self.name_len >= max_names) @compileError("too many DNA names");
        self.names[self.name_len] = name;
        const index: u16 = @intCast(self.name_len);
        self.name_len += 1;
        return index;
    }

    fn intern(self: *State, comptime T: type) u16 {
        @setEvalBranchQuota(100_000);
        for (self.type_keys[0..self.type_len], 0..) |key, i| {
            if (key == T) {
                if (self.visiting[i]) @compileError("recursive DNA type " ++ @typeName(T));
                return @intCast(i);
            }
        }
        if (self.type_len >= max_types) @compileError("too many DNA types");
        const index: u16 = @intCast(self.type_len);
        self.type_len += 1;
        self.type_keys[index] = T;
        self.visiting[index] = true;

        const info = @typeInfo(T);
        var desc: TypeDesc = .{
            .name = @typeName(T),
            .kind = 0,
            .size = fitU16(@sizeOf(T), @typeName(T)),
            .alignment = fitU8(@alignOf(T), @typeName(T)),
        };
        switch (info) {
            .int => |int| {
                if (int.bits == 0 or int.bits % 8 != 0) @compileError("DNA integers must be byte-sized: " ++ @typeName(T));
                desc.kind = if (int.signedness == .unsigned) eSDNA_Type.uint else eSDNA_Type.sint;
            },
            .float => |float| {
                if (float.bits != 32 and float.bits != 64) @compileError("DNA floats are f32 or f64: " ++ @typeName(T));
                desc.kind = eSDNA_Type.float;
            },
            .bool => desc.kind = eSDNA_Type.boolean,
            .array => |arr| {
                if (arr.sentinel() != null) @compileError("DNA arrays cannot have sentinels: " ++ @typeName(T));
                desc.kind = eSDNA_Type.array;
                desc.elem_type = self.intern(arr.child);
                desc.elem_count = fitU16(arr.len, @typeName(T));
            },
            .@"struct" => |s| {
                if (s.layout != .@"extern" or s.is_tuple) @compileError("DNA rows must be extern structs: " ++ @typeName(T));
                desc.kind = eSDNA_Type.structure;
                desc.field_start = fitU16(self.field_len, @typeName(T));
                inline for (s.field_names, s.field_types, s.field_attrs) |field_name, FieldType, attrs| {
                    if (attrs.@"comptime") @compileError("comptime fields are not DNA: " ++ @typeName(T));
                    const field_type_index = self.intern(FieldType);
                    if (self.field_len >= max_fields) @compileError("too many DNA fields");
                    _ = self.internName(field_name);
                    self.fields[self.field_len] = .{
                        .name = field_name,
                        .type_index = field_type_index,
                        .offset = fitU16(@offsetOf(T, field_name), field_name),
                        .size = fitU16(@sizeOf(FieldType), @typeName(FieldType)),
                    };
                    self.field_len += 1;
                }
                desc.field_count = fitU16(self.field_len - desc.field_start, @typeName(T));
            },
            else => @compileError("unsupported DNA type " ++ @typeName(T)),
        }
        _ = self.internName(desc.name);
        self.types[index] = desc;
        self.visiting[index] = false;
        return index;
    }
};

const RootField = struct { child: type, single: bool };

fn rootField(comptime FieldType: type) RootField {
    switch (@typeInfo(FieldType)) {
        .pointer => |pointer| {
            if (pointer.size != .slice or pointer.sentinel() != null)
                @compileError("DNA document fields must be slices: " ++ @typeName(FieldType));
            return .{ .child = pointer.child, .single = false };
        },
        .@"struct" => |s| {
            if (s.layout != .@"extern")
                @compileError("a singular DNA field must be an extern struct: " ++ @typeName(FieldType));
            return .{ .child = FieldType, .single = true };
        },
        else => @compileError("DNA document fields must be slices or extern structs: " ++ @typeName(FieldType)),
    }
}

fn build(comptime Root: type) Schema {
    @setEvalBranchQuota(100_000);
    const info = @typeInfo(Root).@"struct";
    if (info.is_tuple) @compileError("DNA document must be a struct");
    if (info.field_names.len > max_tables) @compileError("too many DNA tables");

    var state: State = .{};
    var tables: [max_tables]TableDesc = @splat(.{});
    inline for (info.field_names, info.field_types, 0..) |field_name, FieldType, i| {
        const spec = rootField(FieldType);
        const type_index = state.intern(spec.child);
        _ = state.internName(field_name);
        tables[i] = .{
            .name = field_name,
            .type_index = type_index,
            .single = spec.single,
            .stride = @sizeOf(spec.child),
            .alignment = @alignOf(spec.child),
        };
    }
    return .{
        .name_len = state.name_len,
        .names = state.names,
        .type_len = state.type_len,
        .types = state.types,
        .field_len = state.field_len,
        .fields = state.fields,
        .table_len = info.field_names.len,
        .tables = tables,
    };
}

fn fitU16(value: usize, label: []const u8) u16 {
    if (value > std.math.maxInt(u16)) @compileError("DNA size exceeds u16: " ++ label);
    return @intCast(value);
}

fn fitU8(value: usize, label: []const u8) u8 {
    if (value > std.math.maxInt(u8)) @compileError("DNA alignment exceeds u8: " ++ label);
    return @intCast(value);
}

fn nameIndex(comptime schema: Schema, name: []const u8) u16 {
    for (schema.names[0..schema.name_len], 0..) |existing, i| {
        if (std.mem.eql(u8, existing, name)) return @intCast(i);
    }
    unreachable;
}

fn dnaBytes(comptime schema: Schema) usize {
    var size: usize = 4;
    for (schema.names[0..schema.name_len]) |name| {
        size += 2 + name.len;
        if (name.len % 2 != 0) size += 1;
    }
    size += 4 + schema.type_len * type_rec_size;
    size += 4 + schema.field_len * field_rec_size;
    return size;
}

fn HashDocument(comptime T: type) type {
    return struct { value: T };
}

/// Comptime fingerprint of the DNA header for one `extern` struct.
pub fn schemaHash(comptime T: type) u64 {
    @setEvalBranchQuota(1_000_000);
    const schema = build(HashDocument(T));
    const len = dnaBytes(schema);
    var dna: [len]u8 = undefined;
    writeDna(&dna, schema);
    return std.hash.Wyhash.hash(0, &dna);
}

fn writeDna(dst: []u8, comptime schema: Schema) void {
    var cursor: usize = 0;
    std.mem.writeInt(u32, dst[cursor..][0..4], @intCast(schema.name_len), .little);
    cursor += 4;
    for (schema.names[0..schema.name_len]) |name| {
        std.mem.writeInt(u16, dst[cursor..][0..2], @intCast(name.len), .little);
        cursor += 2;
        @memcpy(dst[cursor..][0..name.len], name);
        cursor += name.len;
        if (name.len % 2 != 0) {
            dst[cursor] = 0;
            cursor += 1;
        }
    }
    std.mem.writeInt(u32, dst[cursor..][0..4], @intCast(schema.type_len), .little);
    cursor += 4;
    for (schema.types[0..schema.type_len]) |desc| {
        std.mem.writeInt(u16, dst[cursor..][0..2], nameIndex(schema, desc.name), .little);
        std.mem.writeInt(u16, dst[cursor + 2 ..][0..2], desc.size, .little);
        dst[cursor + 4] = desc.alignment;
        dst[cursor + 5] = desc.kind;
        std.mem.writeInt(u16, dst[cursor + 6 ..][0..2], desc.elem_type, .little);
        std.mem.writeInt(u16, dst[cursor + 8 ..][0..2], desc.elem_count, .little);
        std.mem.writeInt(u16, dst[cursor + 10 ..][0..2], desc.field_start, .little);
        std.mem.writeInt(u16, dst[cursor + 12 ..][0..2], desc.field_count, .little);
        std.mem.writeInt(u16, dst[cursor + 14 ..][0..2], 0, .little);
        cursor += type_rec_size;
    }
    std.mem.writeInt(u32, dst[cursor..][0..4], @intCast(schema.field_len), .little);
    cursor += 4;
    for (schema.fields[0..schema.field_len]) |field| {
        std.mem.writeInt(u16, dst[cursor..][0..2], nameIndex(schema, field.name), .little);
        std.mem.writeInt(u16, dst[cursor + 2 ..][0..2], field.type_index, .little);
        std.mem.writeInt(u16, dst[cursor + 4 ..][0..2], field.offset, .little);
        std.mem.writeInt(u16, dst[cursor + 6 ..][0..2], field.size, .little);
        cursor += field_rec_size;
    }
}

pub fn write(allocator: std.mem.Allocator, value: anytype) ![]align(@alignOf(u64)) u8 {
    const Root = @TypeOf(value);
    const schema = comptime build(Root);
    const dna_len = comptime dnaBytes(schema);
    const info = @typeInfo(Root).@"struct";

    var counts: [max_tables]u32 = @splat(0);
    inline for (info.field_names, 0..) |field_name, i| {
        const field = @field(value, field_name);
        const count: usize = if (schema.tables[i].single) 1 else field.len;
        if (count > std.math.maxInt(u32)) return error.DnaTooLarge;
        counts[i] = @intCast(count);
    }

    const directory = std.mem.alignForward(usize, header_size + dna_len, 4);
    var cursor: u32 = @intCast(directory + schema.table_len * dir_entry_size);
    var offsets: [max_tables]u32 = undefined;
    inline for (0..schema.table_len) |i| {
        const alignment: u32 = @intCast(schema.tables[i].alignment);
        cursor = std.mem.alignForward(u32, cursor, alignment);
        offsets[i] = cursor;
        const bytes = @as(u64, counts[i]) * schema.tables[i].stride;
        const end = @as(u64, cursor) + bytes;
        if (end > std.math.maxInt(u32)) return error.DnaTooLarge;
        cursor = @intCast(end);
    }

    const out = try allocator.alignedAlloc(u8, .of(u64), cursor);
    errdefer allocator.free(out);
    @memset(out, 0);
    @memcpy(out[0..magic.len], magic);
    std.mem.writeInt(u32, out[magic.len..][0..4], container_version, .little);
    std.mem.writeInt(u32, out[12..][0..4], @intCast(dna_len), .little);
    std.mem.writeInt(u32, out[16..][0..4], @intCast(schema.table_len), .little);
    writeDna(out[header_size..][0..dna_len], schema);

    inline for (info.field_names, 0..) |field_name, i| {
        const table = schema.tables[i];
        const at = directory + i * dir_entry_size;
        std.mem.writeInt(u16, out[at..][0..2], nameIndex(schema, table.name), .little);
        std.mem.writeInt(u16, out[at + 2 ..][0..2], table.type_index, .little);
        std.mem.writeInt(u32, out[at + 4 ..][0..4], counts[i], .little);
        std.mem.writeInt(u32, out[at + 8 ..][0..4], offsets[i], .little);
        std.mem.writeInt(u32, out[at + 12 ..][0..4], table.stride, .little);

        const field = @field(value, field_name);
        if (table.single) {
            @memcpy(out[offsets[i]..][0..table.stride], std.mem.asBytes(&field));
        } else if (counts[i] != 0) {
            const bytes = std.mem.sliceAsBytes(field);
            @memcpy(out[offsets[i]..][0..bytes.len], bytes);
        }
    }
    return out;
}

const Parsed = struct {
    names: [][]const u8,
    types: []FileType,
    fields: []FileField,
    tables: []FileTable,

    fn deinit(self: *Parsed, allocator: std.mem.Allocator) void {
        allocator.free(self.tables);
        allocator.free(self.fields);
        allocator.free(self.types);
        allocator.free(self.names);
    }
};

pub fn read(comptime Root: type, allocator: std.mem.Allocator, bytes: []const u8) !Root {
    var parsed = try parse(allocator, bytes);
    defer parsed.deinit(allocator);
    const schema = comptime build(Root);
    const info = @typeInfo(Root).@"struct";
    var out: Root = undefined;
    inline for (info.field_names, info.field_types) |field_name, FieldType| {
        const spec = rootField(FieldType);
        if (spec.single) {
            @field(out, field_name) = std.mem.zeroes(FieldType);
        } else {
            @field(out, field_name) = &.{};
        }
    }
    errdefer free(Root, allocator, &out, bytes);

    const in_place = mapInPlace(Root, bytes, &parsed);
    inline for (info.field_names, info.field_types, 0..) |field_name, FieldType, i| {
        const spec = rootField(FieldType);
        const table = findTable(parsed.tables, schema.tables[i].name);
        if (spec.single) {
            if (table) |found| {
                if (found.count != 1) return error.IncompatibleDna;
                const row = try rowBytes(bytes, found, 0);
                try readValue(spec.child, &@field(out, field_name), row, &parsed, found.type_index);
            }
        } else if (table) |found| {
            @field(out, field_name) = try readColumn(spec.child, allocator, bytes, found, &parsed, in_place);
        }
    }
    return out;
}

/// Releases copied tables. Slices that still address `source` stay with that buffer.
pub fn free(comptime Root: type, allocator: std.mem.Allocator, value: *Root, source: []const u8) void {
    const info = @typeInfo(Root).@"struct";
    inline for (info.field_names, info.field_types) |field_name, FieldType| {
        if (@typeInfo(FieldType) != .pointer) continue;
        const slice = @field(value, field_name);
        if (!contained(std.mem.sliceAsBytes(slice), source)) allocator.free(slice);
    }
}

/// True when every non-empty slice of `value` addresses `source`.
pub fn slicesMapped(comptime Root: type, value: Root, source: []const u8) bool {
    const info = @typeInfo(Root).@"struct";
    inline for (info.field_names, info.field_types) |field_name, FieldType| {
        if (@typeInfo(FieldType) == .pointer) {
            const slice = @field(value, field_name);
            if (slice.len != 0 and !contained(std.mem.sliceAsBytes(slice), source)) return false;
        }
    }
    return true;
}

fn contained(slice: []const u8, source: []const u8) bool {
    if (slice.len == 0 or source.len == 0) return false;
    const start = @intFromPtr(source.ptr);
    const at = @intFromPtr(slice.ptr);
    return at >= start and at + slice.len <= start + source.len;
}

fn mapInPlace(comptime Root: type, bytes: []const u8, parsed: *const Parsed) bool {
    const schema = comptime build(Root);
    const info = @typeInfo(Root).@"struct";
    inline for (info.field_types, 0..) |FieldType, i| {
        const spec = rootField(FieldType);
        if (!spec.single) {
            if (findTable(parsed.tables, schema.tables[i].name)) |table| {
                if (table.count != 0) {
                    const how = matchType(spec.child, parsed, table.type_index) catch return false;
                    if (how != .identical or table.stride != @sizeOf(spec.child)) return false;
                    const raw = tableBytes(bytes, table) catch return false;
                    if (@intFromPtr(raw.ptr) % @alignOf(spec.child) != 0) return false;
                }
            }
        }
    }
    return true;
}

fn parse(allocator: std.mem.Allocator, bytes: []const u8) !Parsed {
    if (bytes.len < header_size or !std.mem.eql(u8, bytes[0..magic.len], magic)) return error.InvalidDna;
    const version = std.mem.readInt(u32, bytes[magic.len..][0..4], .little);
    if (version != container_version) return error.UnsupportedDnaVersion;
    const dna_len = std.mem.readInt(u32, bytes[12..][0..4], .little);
    const table_count = std.mem.readInt(u32, bytes[16..][0..4], .little);
    const dna_end = header_size + @as(usize, dna_len);
    if (dna_end > bytes.len) return error.InvalidDna;
    const directory = std.mem.alignForward(usize, dna_end, 4);
    const directory_end = directory + @as(usize, table_count) * dir_entry_size;
    if (directory_end > bytes.len) return error.InvalidDna;

    var cursor: usize = header_size;
    const dna_limit = dna_end;
    const name_count = try readU32(bytes, &cursor, dna_limit);
    const names = try allocator.alloc([]const u8, name_count);
    errdefer allocator.free(names);
    for (names) |*name| {
        if (cursor + 2 > dna_limit) return error.InvalidDna;
        const len = std.mem.readInt(u16, bytes[cursor..][0..2], .little);
        cursor += 2;
        if (cursor + len > dna_limit) return error.InvalidDna;
        name.* = bytes[cursor..][0..len];
        cursor += len;
        if (len % 2 != 0) {
            if (cursor >= dna_limit) return error.InvalidDna;
            cursor += 1;
        }
    }

    const type_count = try readU32(bytes, &cursor, dna_limit);
    const file_types = try allocator.alloc(FileType, type_count);
    errdefer allocator.free(file_types);
    for (file_types) |*desc| {
        if (cursor + type_rec_size > dna_limit) return error.InvalidDna;
        const name_at = std.mem.readInt(u16, bytes[cursor..][0..2], .little);
        if (name_at >= names.len) return error.InvalidDna;
        desc.* = .{
            .name = names[name_at],
            .size = std.mem.readInt(u16, bytes[cursor + 2 ..][0..2], .little),
            .alignment = bytes[cursor + 4],
            .kind = bytes[cursor + 5],
            .elem_type = std.mem.readInt(u16, bytes[cursor + 6 ..][0..2], .little),
            .elem_count = std.mem.readInt(u16, bytes[cursor + 8 ..][0..2], .little),
            .field_start = std.mem.readInt(u16, bytes[cursor + 10 ..][0..2], .little),
            .field_count = std.mem.readInt(u16, bytes[cursor + 12 ..][0..2], .little),
        };
        cursor += type_rec_size;
    }

    const field_count = try readU32(bytes, &cursor, dna_limit);
    const fields = try allocator.alloc(FileField, field_count);
    errdefer allocator.free(fields);
    for (fields) |*field| {
        if (cursor + field_rec_size > dna_limit) return error.InvalidDna;
        const name_at = std.mem.readInt(u16, bytes[cursor..][0..2], .little);
        const type_index = std.mem.readInt(u16, bytes[cursor + 2 ..][0..2], .little);
        if (name_at >= names.len or type_index >= file_types.len) return error.InvalidDna;
        field.* = .{
            .name = names[name_at],
            .type_index = type_index,
            .offset = std.mem.readInt(u16, bytes[cursor + 4 ..][0..2], .little),
            .size = std.mem.readInt(u16, bytes[cursor + 6 ..][0..2], .little),
        };
        if (field.size != file_types[type_index].size) return error.InvalidDna;
        cursor += field_rec_size;
    }
    for (file_types) |desc| {
        if (@as(usize, desc.field_start) + desc.field_count > fields.len) return error.InvalidDna;
        if (desc.kind == eSDNA_Type.array and desc.elem_type >= file_types.len) return error.InvalidDna;
    }

    const tables = try allocator.alloc(FileTable, table_count);
    errdefer allocator.free(tables);
    for (tables, 0..) |*table, i| {
        const at = directory + i * dir_entry_size;
        const name_at = std.mem.readInt(u16, bytes[at..][0..2], .little);
        const type_index = std.mem.readInt(u16, bytes[at + 2 ..][0..2], .little);
        if (name_at >= names.len or type_index >= file_types.len) return error.InvalidDna;
        table.* = .{
            .name = names[name_at],
            .type_index = type_index,
            .count = std.mem.readInt(u32, bytes[at + 4 ..][0..4], .little),
            .offset = std.mem.readInt(u32, bytes[at + 8 ..][0..4], .little),
            .stride = std.mem.readInt(u32, bytes[at + 12 ..][0..4], .little),
        };
        const bytes_len = @as(u64, table.count) * table.stride;
        if (table.stride < file_types[type_index].size) return error.InvalidDna;
        if (@as(u64, table.offset) + bytes_len > bytes.len) return error.InvalidDna;
        for (tables[0..i]) |earlier| if (std.mem.eql(u8, earlier.name, table.name)) return error.InvalidDna;
    }
    return .{ .names = names, .types = file_types, .fields = fields, .tables = tables };
}

fn readU32(bytes: []const u8, cursor: *usize, limit: usize) !u32 {
    if (cursor.* + 4 > limit) return error.InvalidDna;
    const value = std.mem.readInt(u32, bytes[cursor.*..][0..4], .little);
    cursor.* += 4;
    return value;
}

fn findTable(tables: []const FileTable, name: []const u8) ?FileTable {
    for (tables) |table| if (std.mem.eql(u8, table.name, name)) return table;
    return null;
}

fn findField(parsed: *const Parsed, desc: FileType, name: []const u8) ?FileField {
    const fields = parsed.fields[desc.field_start..][0..desc.field_count];
    for (fields) |field| if (std.mem.eql(u8, field.name, name)) return field;
    return null;
}

fn structHasField(comptime T: type, name: []const u8) bool {
    inline for (@typeInfo(T).@"struct".field_names) |field_name| {
        if (std.mem.eql(u8, field_name, name)) return true;
    }
    return false;
}

fn matchType(comptime T: type, parsed: *const Parsed, type_index: usize) !Match {
    if (type_index >= parsed.types.len) return error.InvalidDna;
    const file = parsed.types[type_index];
    switch (@typeInfo(T)) {
        .int => |int| {
            const kind: u8 = if (int.signedness == .unsigned) eSDNA_Type.uint else eSDNA_Type.sint;
            if (file.kind != kind or file.size != @sizeOf(T) or int.bits != @as(u16, file.size) * 8) return error.IncompatibleDna;
            return .identical;
        },
        .float => |float| {
            if (file.kind != eSDNA_Type.float or file.size != @sizeOf(T) or float.bits != @as(u16, file.size) * 8) return error.IncompatibleDna;
            return .identical;
        },
        .bool => {
            if (file.kind != eSDNA_Type.boolean or file.size != 1) return error.IncompatibleDna;
            return .identical;
        },
        .array => |arr| {
            if (file.kind != eSDNA_Type.array or file.elem_count != arr.len) return error.IncompatibleDna;
            const child = try matchType(arr.child, parsed, file.elem_type);
            if (child == .identical and file.size == @sizeOf(T)) return .identical;
            return .remap;
        },
        .@"struct" => |s| {
            if (file.kind != eSDNA_Type.structure or s.layout != .@"extern") return error.IncompatibleDna;
            var identical = file.size == @sizeOf(T);
            inline for (s.field_names, s.field_types) |field_name, FieldType| {
                if (findField(parsed, file, field_name)) |field| {
                    if (field.offset + field.size > file.size) return error.InvalidDna;
                    const child = try matchType(FieldType, parsed, field.type_index);
                    if (field.offset != @offsetOf(T, field_name) or field.size != @sizeOf(FieldType) or child != .identical)
                        identical = false;
                } else identical = false;
            }
            for (parsed.fields[file.field_start..][0..file.field_count]) |field| {
                if (!structHasField(T, field.name)) identical = false;
            }
            return if (identical) .identical else .remap;
        },
        else => @compileError("unsupported DNA type " ++ @typeName(T)),
    }
}

fn readColumn(comptime T: type, allocator: std.mem.Allocator, bytes: []const u8, table: FileTable, parsed: *const Parsed, in_place: bool) ![]T {
    const how = try matchType(T, parsed, table.type_index);
    if (table.count == 0) return &.{};
    const raw = try tableBytes(bytes, table);
    if (in_place and how == .identical and table.stride == @sizeOf(T)) {
        const aligned: []align(@alignOf(T)) const u8 = @alignCast(raw);
        return @constCast(std.mem.bytesAsSlice(T, aligned));
    }
    const slice = try allocator.alloc(T, table.count);
    errdefer allocator.free(slice);
    if (how == .identical and table.stride == @sizeOf(T)) {
        @memcpy(std.mem.sliceAsBytes(slice), raw);
        return slice;
    }
    for (slice, 0..) |*item, i| {
        const row = raw[i * table.stride ..][0..table.stride];
        try readValue(T, item, row, parsed, table.type_index);
    }
    return slice;
}

fn readValue(comptime T: type, dst: *T, src: []const u8, parsed: *const Parsed, type_index: usize) !void {
    const how = try matchType(T, parsed, type_index);
    const file = parsed.types[type_index];
    if (src.len < file.size) return error.InvalidDna;
    if (how == .identical) {
        @memcpy(std.mem.asBytes(dst), src[0..@sizeOf(T)]);
        return;
    }
    switch (@typeInfo(T)) {
        .array => |arr| {
            const elem_size = parsed.types[file.elem_type].size;
            for (0..arr.len) |i| {
                const elem_src = src[i * elem_size ..][0 .. elem_size];
                try readValue(arr.child, &dst[i], elem_src, parsed, file.elem_type);
            }
        },
        .@"struct" => |s| {
            @memset(std.mem.asBytes(dst), 0);
            inline for (s.field_names, s.field_types) |field_name, FieldType| {
                if (findField(parsed, file, field_name)) |field| {
                    const field_src = src[field.offset..][0..field.size];
                    const at = @offsetOf(T, field_name);
                    const field_dst: *FieldType = @ptrCast(@alignCast(std.mem.asBytes(dst)[at..].ptr));
                    try readValue(FieldType, field_dst, field_src, parsed, field.type_index);
                }
            }
        },
        else => return error.IncompatibleDna,
    }
}

fn tableBytes(bytes: []const u8, table: FileTable) ![]const u8 {
    const len = @as(u64, table.count) * table.stride;
    if (@as(u64, table.offset) + len > bytes.len) return error.InvalidDna;
    return bytes[table.offset..][0..@intCast(len)];
}

fn rowBytes(bytes: []const u8, table: FileTable, index: usize) ![]const u8 {
    const raw = try tableBytes(bytes, table);
    return raw[index * table.stride ..][0..table.stride];
}

/// Field conventions shared by catalog rows: a name in the string table, an `owner`
/// column, and packed geometry. These are not part of the Zig type layout.
pub const rna = struct {
    pub fn string(strings: []const u8, row: anytype) []const u8 {
        return strings[row.name_offset..][0..row.name_len];
    }

    pub fn owned(rows: anytype, owner: u32) usize {
        var n: usize = 0;
        for (rows) |row| {
            if (row.owner == owner) n += 1;
        }
        return n;
    }

    pub fn ownedAt(rows: anytype, owner: u32, nth: usize) @TypeOf(&rows[0]) {
        var seen: usize = 0;
        for (rows, 0..) |row, i| {
            if (row.owner != owner) continue;
            if (seen == nth) return &rows[i];
            seen += 1;
        }
        unreachable;
    }

    pub fn vec3s(positions: []const f32, start: u32, count: u32) []const [3]f32 {
        const floats = positions[@as(usize, start) * 3 ..][0 .. @as(usize, count) * 3];
        return std.mem.bytesAsSlice([3]f32, std.mem.sliceAsBytes(floats));
    }

    pub fn triangles(indices: []const u32, start: u32, count: u32) []const types.Triangle {
        const idxs = indices[@as(usize, start) * 3 ..][0 .. @as(usize, count) * 3];
        return std.mem.bytesAsSlice(types.Triangle, std.mem.sliceAsBytes(idxs));
    }
};
