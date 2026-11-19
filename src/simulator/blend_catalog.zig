//! Minimal .blend reader for scene export. Pinned to Blender 4.x file format.
const std = @import("std");
const types = @import("types.zig");
const database = @import("database.zig");

pub const pinned_version_major: u8 = 4;

pub const BlendFile = struct {
    allocator: std.mem.Allocator,
    owned_bytes: bool,
    bytes: []const u8,
    endian: std.builtin.Endian,
    pointer_size: usize,
    version: [3]u8,
    blocks: []Block,
    sdna: SDNA,
    by_old: std.AutoHashMap(usize, Block),

    pub fn deinit(self: *BlendFile) void {
        if (self.owned_bytes) self.allocator.free(self.bytes);
        self.allocator.free(self.blocks);
        self.sdna.deinit();
        self.by_old.deinit();
    }

    pub fn load(allocator: std.mem.Allocator, io: std.Io, path: []const u8) !BlendFile {
        const file = try std.Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(std.math.maxInt(usize)));
        errdefer allocator.free(file);
        const bytes = try decodeRoot(allocator, file);
        errdefer allocator.free(bytes);
        if (bytes.ptr != file.ptr) allocator.free(file);

        if (bytes.len < 12 or !std.mem.eql(u8, bytes[0..7], "BLENDER"))
            return error.NotABlendFile;
        const pointer_size: usize = if (bytes[7] == '-') 8 else 4;
        const endian: std.builtin.Endian = if (bytes[8] == 'V') .big else .little;
        const version: [3]u8 = bytes[9..12].*;
        if (version[0] != pinned_version_major) return error.UnsupportedBlenderVersion;

        var blocks: std.ArrayList(Block) = .empty;
        errdefer blocks.deinit(allocator);
        var sdna: ?SDNA = null;
        var cursor: usize = 12;
        const header_extra: usize = if (pointer_size == 8) 8 else 0;

        while (cursor < bytes.len) {
            if (cursor + 4 > bytes.len) break;
            const code: [4]u8 = bytes[cursor..][0..4].*;
            cursor += 4;
            if (std.mem.eql(u8, &code, "ENDB")) {
                try blocks.append(allocator, .{ .code = code, .size = 0, .addr_old = 0, .count = 0, .sdna_index = 0, .data = &.{} });
                break;
            }
            const block_header = 8 + header_extra + pointer_size + 4 + 4;
            if (cursor + block_header > bytes.len) return error.InvalidBlendFile;
            const size = readInt(usize, endian, bytes[cursor..][0..8]);
            cursor += 8 + header_extra;
            const addr_old = readInt(usize, endian, bytes[cursor .. cursor + pointer_size]);
            cursor += pointer_size;
            const sdna_index = readInt(u32, endian, bytes[cursor..][0..4]);
            cursor += 4;
            const block_count = readInt(u32, endian, bytes[cursor..][0..4]);
            cursor += 4;
            if (cursor + size > bytes.len) return error.InvalidBlendFile;
            const data = bytes[cursor .. cursor + size];
            if (std.mem.eql(u8, &code, "DNA1")) sdna = try SDNA.parse(allocator, endian, pointer_size, data);
            try blocks.append(allocator, .{ .code = code, .size = size, .addr_old = addr_old, .count = block_count, .sdna_index = sdna_index, .data = data });
            cursor = pad4(cursor + size);
        }

        const catalog = sdna orelse return error.MissingDNA;
        var by_old = std.AutoHashMap(usize, Block).init(allocator);
        for (blocks.items) |block| {
            if (block.addr_old != 0) try by_old.put(block.addr_old, block);
        }
        return .{
            .allocator = allocator,
            .owned_bytes = true,
            .bytes = bytes,
            .endian = endian,
            .pointer_size = pointer_size,
            .version = version,
            .blocks = try blocks.toOwnedSlice(allocator),
            .sdna = catalog,
            .by_old = by_old,
        };
    }

    pub fn exportCollection(self: *BlendFile, collection: []const u8) ![]ExportedObject {
        _ = collection;
        var out: std.ArrayList(ExportedObject) = .empty;
        errdefer out.deinit(self.allocator);

        const object_layout = self.sdna.structNamed("Object") orelse return error.MissingDNA;
        const mesh_layout = self.sdna.structNamed("Mesh") orelse return error.MissingDNA;

        for (self.blocks) |block| {
            if (!codeIs(&block.code, "OB")) continue;
            if (block.count == 0) continue;
            const elem = block.data[0..@min(object_layout.size, block.data.len)];
            const mesh_ptr = object_layout.readPointer(self.endian, self.pointer_size, elem, "data") orelse continue;
            const mesh_block = self.by_old.get(mesh_ptr) orelse continue;
            if (!codeIs(&mesh_block.code, "ME")) continue;

            const name = try readName(self.allocator, object_layout, elem);
            const props = try readSimProps(self, object_layout, elem);
            const id = props.id orelse continue;

            const pose = readObjectPose(object_layout, elem);
            var mesh = try readMeshGeometry(self, mesh_layout, mesh_block.data[0..@min(mesh_block.data.len, mesh_block.data.len)]);
            errdefer {
                self.allocator.free(mesh.positions);
                self.allocator.free(mesh.triangles);
            }
            if (mesh.positions.len == 0 and props.hx != null and props.hy != null and props.hz != null) {
                const box = database.boxMesh(props.hx.?, props.hy.?, props.hz.?);
                mesh.positions = try self.allocator.dupe(types.Vec3, &box.positions);
                mesh.triangles = try self.allocator.dupe(types.Triangle, &box.triangles);
            }
            try out.append(self.allocator, .{
                .name = name,
                .id = id,
                .cls = props.cls orelse 0,
                .tracked = props.tracked orelse true,
                .collisions = props.collisions orelse true,
                .pose = pose,
                .positions = mesh.positions,
                .triangles = mesh.triangles,
            });
        }
        return try out.toOwnedSlice(self.allocator);
    }

    pub fn deinitExported(self: *BlendFile, objects: []ExportedObject) void {
        for (objects) |object| {
            self.allocator.free(object.name);
            self.allocator.free(object.positions);
            self.allocator.free(object.triangles);
        }
        self.allocator.free(objects);
    }

    pub const ExportedObject = struct {
        name: []const u8,
        id: u32,
        cls: u32,
        tracked: bool,
        collisions: bool,
        pose: types.Pose,
        positions: []types.Vec3,
        triangles: []types.Triangle,
    };
};

pub const Block = struct {
    code: [4]u8,
    size: usize,
    addr_old: usize,
    count: u32,
    sdna_index: u32,
    data: []const u8,
};

fn readName(allocator: std.mem.Allocator, layout: SDNAStruct, elem: []const u8) ![]u8 {
    const field = layout.field("name") orelse return allocator.dupe(u8, "object");
    const raw = elem[field.offset..][0..@min(field.size, 64)];
    const end = std.mem.indexOfScalar(u8, raw, 0) orelse raw.len;
    return allocator.dupe(u8, raw[0..end]);
}

const SimProps = struct { id: ?u32, cls: ?u32, tracked: ?bool, collisions: ?bool, hx: ?f32, hy: ?f32, hz: ?f32 };

fn readSimProps(self: *BlendFile, layout: SDNAStruct, elem: []const u8) !SimProps {
    _ = self;
    _ = layout;
    _ = elem;
    return .{
        .id = null,
        .cls = null,
        .tracked = null,
        .collisions = null,
        .hx = null,
        .hy = null,
        .hz = null,
    };
}

fn readObjectPose(layout: SDNAStruct, elem: []const u8) types.Pose {
    var pose: types.Pose = .{};
    if (layout.field("loc")) |field| pose.pos = readVec3(elem, field.offset);
    if (layout.field("rot")) |field| pose.quat = eulerToQuat(readVec3(elem, field.offset));
    return pose;
}

const MeshData = struct { positions: []types.Vec3, triangles: []types.Triangle };

fn readMeshGeometry(self: *BlendFile, layout: SDNAStruct, elem: []const u8) !MeshData {
    const allocator = self.allocator;
    const totvert = if (layout.field("totvert")) |f| readU32(self.endian, elem[f.offset..][0..4]) else 0;
    const totpoly = if (layout.field("totpoly")) |f| readU32(self.endian, elem[f.offset..][0..4]) else 0;
    const vert_ptr = layout.readPointer(self.endian, self.pointer_size, elem, "mvert") orelse 0;
    const loop_ptr = layout.readPointer(self.endian, self.pointer_size, elem, "mloop") orelse 0;
    const poly_ptr = layout.readPointer(self.endian, self.pointer_size, elem, "mpoly") orelse 0;

    var positions: std.ArrayList(types.Vec3) = .empty;
    errdefer positions.deinit(allocator);
    var triangles: std.ArrayList(types.Triangle) = .empty;
    errdefer triangles.deinit(allocator);

    if (vert_ptr != 0 and totvert > 0) {
        const vert_block = self.by_old.get(vert_ptr) orelse return try emptySlices(allocator);
        const mvert_layout = self.sdna.structNamed("MVert") orelse return try emptySlices(allocator);
        const stride = mvert_layout.size;
        for (0..totvert) |i| {
            if (i * stride + stride > vert_block.data.len) break;
            const vdata = vert_block.data[i * stride ..][0..stride];
            const co_off = mvert_layout.field("co") orelse continue;
            const co = readVec3i(vdata, co_off.offset);
            try positions.append(allocator, .{
                @as(f32, @floatFromInt(co[0])) / 1000.0,
                @as(f32, @floatFromInt(co[1])) / 1000.0,
                @as(f32, @floatFromInt(co[2])) / 1000.0,
            });
        }
    }

    if (loop_ptr != 0 and poly_ptr != 0 and totpoly > 0) {
        const loop_block = self.by_old.get(loop_ptr) orelse return finishMesh(allocator, &positions, &triangles);
        const poly_block = self.by_old.get(poly_ptr) orelse return finishMesh(allocator, &positions, &triangles);
        const loop_layout = self.sdna.structNamed("MLoop") orelse return finishMesh(allocator, &positions, &triangles);
        const poly_layout = self.sdna.structNamed("MPoly") orelse return finishMesh(allocator, &positions, &triangles);
        for (0..totpoly) |p| {
            if (p * poly_layout.size + poly_layout.size > poly_block.data.len) break;
            const poly = poly_block.data[p * poly_layout.size ..][0..poly_layout.size];
            const loopstart = if (poly_layout.field("loopstart")) |f| readU32(self.endian, poly[f.offset..][0..4]) else 0;
            const loop_total = if (poly_layout.field("totloop")) |f| readU32(self.endian, poly[f.offset..][0..4]) else 0;
            if (loop_total < 3) continue;
            const v0 = loopVertex(loop_block.data, loop_layout, loopstart + 0);
            for (1..loop_total - 1) |k| {
                const v1 = loopVertex(loop_block.data, loop_layout, loopstart + @as(u32, @intCast(k)));
                const v2 = loopVertex(loop_block.data, loop_layout, loopstart + @as(u32, @intCast(k + 1)));
                try triangles.append(allocator, .{ .a = v0, .b = v1, .c = v2 });
            }
        }
    }
    return finishMesh(allocator, &positions, &triangles);
}

fn emptySlices(allocator: std.mem.Allocator) !MeshData {
    return .{
        .positions = try allocator.alloc(types.Vec3, 0),
        .triangles = try allocator.alloc(types.Triangle, 0),
    };
}

fn finishMesh(allocator: std.mem.Allocator, positions: *std.ArrayList(types.Vec3), triangles: *std.ArrayList(types.Triangle)) !MeshData {
    return .{
        .positions = try positions.toOwnedSlice(allocator),
        .triangles = try triangles.toOwnedSlice(allocator),
    };
}

fn loopVertex(data: []const u8, layout: SDNAStruct, index: u32) u32 {
    const stride = layout.size;
    const offset = @as(usize, index) * stride;
    if (offset + stride > data.len) return 0;
    const field = layout.field("v") orelse return 0;
    return std.mem.readInt(u32, data[offset + field.offset ..][0..4], .little);
}

const SDNA = struct {
    allocator: std.mem.Allocator,
    structs: []SDNAStruct,

    fn deinit(self: *SDNA) void {
        for (self.structs) |*s| self.allocator.free(s.fields);
        self.allocator.free(self.structs);
    }

    fn structNamed(self: *const SDNA, name: []const u8) ?SDNAStruct {
        for (self.structs) |s| {
            if (std.mem.eql(u8, s.name, name)) return s;
        }
        return null;
    }

    fn parse(allocator: std.mem.Allocator, endian: std.builtin.Endian, pointer_size: usize, data: []const u8) !SDNA {
        if (data.len < 16 or !std.mem.eql(u8, data[0..4], "SDNA")) return error.InvalidDNA;
        var offset: usize = 4;
        const name_count = readU32(endian, data[offset..][0..4]);
        offset += 4;
        offset = pad4(offset);
        var names: std.ArrayList([]const u8) = .empty;
        defer names.deinit(allocator);
        var i: u32 = 0;
        while (i < name_count) : (i += 1) {
            const start = offset;
            while (offset < data.len and data[offset] != 0) offset += 1;
            try names.append(allocator, data[start..offset]);
            offset += 1;
        }
        offset = pad4(offset);
        const type_count = readU32(endian, data[offset..][0..4]);
        offset += 4;
        offset = pad4(offset);
        var type_names: std.ArrayList([]const u8) = .empty;
        defer type_names.deinit(allocator);
        i = 0;
        while (i < type_count) : (i += 1) {
            const start = offset;
            while (offset < data.len and data[offset] != 0) offset += 1;
            try type_names.append(allocator, data[start..offset]);
            offset += 1;
        }
        offset = pad4(offset);
        const type_sizes = try allocator.alloc(u16, type_count);
        defer allocator.free(type_sizes);
        for (type_sizes) |*size| {
            size.* = std.mem.readInt(u16, data[offset..][0..2], endian);
            offset += 2;
        }
        offset = pad4(offset);
        const struct_count = readU32(endian, data[offset..][0..4]);
        offset += 4;
        var structs = try allocator.alloc(SDNAStruct, struct_count);
        i = 0;
        while (i < struct_count) : (i += 1) {
            const type_index = std.mem.readInt(u16, data[offset..][0..2], endian);
            offset += 2;
            const field_count = std.mem.readInt(u16, data[offset..][0..2], endian);
            offset += 2;
            const type_name = type_names.items[type_index];
            var fields = try allocator.alloc(SDNAField, field_count);
            var field_offset: usize = 0;
            var f: u16 = 0;
            while (f < field_count) : (f += 1) {
                const ft = std.mem.readInt(u16, data[offset..][0..2], endian);
                offset += 2;
                const fn_i = std.mem.readInt(u16, data[offset..][0..2], endian);
                offset += 2;
                const clean = parseFieldName(names.items[fn_i]);
                const size = fieldSize(pointer_size, type_sizes[ft], type_names.items[ft], clean);
                fields[f] = .{ .name = clean.name, .offset = field_offset, .size = size };
                field_offset += size;
            }
            structs[i] = .{ .name = stripType(type_name), .size = field_offset, .fields = fields };
        }
        return .{ .allocator = allocator, .structs = structs };
    }
};

const SDNAStruct = struct {
    name: []const u8,
    size: usize,
    fields: []SDNAField,

    fn field(self: SDNAStruct, name: []const u8) ?SDNAField {
        for (self.fields) |f| {
            if (std.mem.eql(u8, f.name, name)) return f;
        }
        return null;
    }

    fn readPointer(self: SDNAStruct, endian: std.builtin.Endian, pointer_size: usize, elem: []const u8, name: []const u8) ?usize {
        const f = self.field(name) orelse return null;
        if (f.offset + pointer_size > elem.len) return null;
        return readInt(usize, endian, elem[f.offset .. f.offset + pointer_size]);
    }
};

const SDNAField = struct { name: []const u8, offset: usize, size: usize };

fn parseFieldName(raw: []const u8) struct { name: []const u8, is_pointer: bool, array: usize } {
    var is_pointer = false;
    var i: usize = 0;
    if (i < raw.len and raw[i] == '*') {
        is_pointer = true;
        i += 1;
    }
    const start = i;
    while (i < raw.len and raw[i] != '[') i += 1;
    var array: usize = 1;
    if (i < raw.len and raw[i] == '[') {
        var j = i + 1;
        while (j < raw.len and raw[j] != ']') j += 1;
        if (j > i + 1) array = std.fmt.parseInt(usize, raw[i + 1 .. j], 10) catch 1;
    }
    return .{ .name = raw[start..i], .is_pointer = is_pointer, .array = array };
}

fn stripType(name: []const u8) []const u8 {
    if (name.len > 0 and name[0] == ':') return name[1..];
    return name;
}

fn fieldSize(pointer_size: usize, type_size: u16, type_name: []const u8, clean: anytype) usize {
    if (clean.is_pointer) return pointer_size * clean.array;
    if (std.mem.eql(u8, type_name, "char") and clean.array > 1) return clean.array;
    return @as(usize, type_size) * clean.array;
}

fn decodeRoot(_: std.mem.Allocator, file: []u8) ![]u8 {
    if (file.len >= 7 and std.mem.eql(u8, file[0..7], "BLENDER")) return file;
    return error.UnsupportedBlendCompression;
}

fn codeIs(code: *const [4]u8, tag: []const u8) bool {
    return std.mem.eql(u8, code[0..tag.len], tag);
}

fn pad4(n: usize) usize {
    return (n + 3) & ~@as(usize, 3);
}

fn readU32(endian: std.builtin.Endian, bytes: []const u8) u32 {
    return std.mem.readInt(u32, bytes[0..4], endian);
}

fn readInt(comptime T: type, endian: std.builtin.Endian, bytes: []const u8) T {
    return std.mem.readInt(T, bytes[0..@sizeOf(T)], endian);
}

fn readVec3(elem: []const u8, offset: usize) types.Vec3 {
    return .{
        @bitCast(std.mem.readInt(u32, elem[offset..][0..4], .little)),
        @bitCast(std.mem.readInt(u32, elem[offset + 4 ..][0..4], .little)),
        @bitCast(std.mem.readInt(u32, elem[offset + 8 ..][0..4], .little)),
    };
}

fn readVec3i(elem: []const u8, offset: usize) [3]i32 {
    return .{
        std.mem.readInt(i32, elem[offset..][0..4], .little),
        std.mem.readInt(i32, elem[offset + 4 ..][0..4], .little),
        std.mem.readInt(i32, elem[offset + 8 ..][0..4], .little),
    };
}

fn eulerToQuat(euler: types.Vec3) types.Quat {
    const rx = euler[0];
    const ry = euler[1];
    const rz = euler[2];
    const cx = @cos(rx * 0.5);
    const sx = @sin(rx * 0.5);
    const cy = @cos(ry * 0.5);
    const sy = @sin(ry * 0.5);
    const cz = @cos(rz * 0.5);
    const sz = @sin(rz * 0.5);
    return .{
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
        cx * cy * cz + sx * sy * sz,
    };
}
