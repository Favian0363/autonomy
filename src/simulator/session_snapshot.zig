//! SIMSESS001 session capture, save, load, and restore.
const std = @import("std");
const abi = @import("abi.zig");
const Session = @import("Session.zig");

pub const Snapshot = struct {
    allocator: std.mem.Allocator,
    meta: abi.SessionMeta,
    robot_name: []const u8,
    scene_name: []const u8,
    robot_catalog: []const u8,
    scene_catalog: []const u8,
    body_states: []const abi.BodyState,
    wrenches: []const abi.Wrench,
    fluid_wrenches: []const abi.Wrench,
    prev_linear: []const abi.Vec3Slot,
    prev_angular: []const abi.Vec3Slot,
    rotors: []const abi.RotorState,
    contacts: []const abi.Contact,
    owned_body_states: []align(@alignOf(abi.BodyState)) u8,
    owned_wrenches: []align(@alignOf(abi.Wrench)) u8,
    owned_fluid: []align(@alignOf(abi.Wrench)) u8,
    owned_prev_linear: []align(@alignOf(abi.Vec3Slot)) u8,
    owned_prev_angular: []align(@alignOf(abi.Vec3Slot)) u8,
    owned_rotors: []align(@alignOf(abi.RotorState)) u8,
    owned_contacts: []align(@alignOf(abi.Contact)) u8,

    pub fn deinit(self: *Snapshot) void {
        self.allocator.free(self.robot_name);
        self.allocator.free(self.scene_name);
        self.allocator.free(self.robot_catalog);
        self.allocator.free(self.scene_catalog);
        self.allocator.free(self.owned_body_states);
        self.allocator.free(self.owned_wrenches);
        self.allocator.free(self.owned_fluid);
        self.allocator.free(self.owned_prev_linear);
        self.allocator.free(self.owned_prev_angular);
        self.allocator.free(self.owned_rotors);
        self.allocator.free(self.owned_contacts);
        self.* = undefined;
    }
};

fn dupRecords(comptime R: type, allocator: std.mem.Allocator, slice: []const R) ![]align(@alignOf(R)) u8 {
    const bytes = std.mem.sliceAsBytes(slice);
    return try allocator.alignedAlloc(u8, std.mem.Alignment.of(R), bytes.len);
}

fn dupVec3Slots(allocator: std.mem.Allocator, slice: []const abi.Vec3) ![]align(@alignOf(abi.Vec3Slot)) u8 {
    const out = try allocator.alignedAlloc(u8, std.mem.Alignment.of(abi.Vec3Slot), slice.len * @sizeOf(abi.Vec3Slot));
    const records = std.mem.bytesAsSlice(abi.Vec3Slot, out);
    for (slice, records) |v, *r| r.v = v;
    return out;
}

pub fn capture(allocator: std.mem.Allocator, session: *const Session) !Snapshot {
    if (session.backend.contacts.items.len > abi.max_snapshot_contacts) return error.TooManyContacts;
    const robot_catalog = if (session.catalogs_owned)
        try session.robot_db.save(allocator)
    else
        try allocator.dupe(u8, &.{});
    errdefer allocator.free(robot_catalog);
    const scene_catalog = if (session.catalogs_owned)
        try session.scene_db.save(allocator)
    else
        try allocator.dupe(u8, &.{});
    errdefer allocator.free(scene_catalog);
    const robot_name = try allocator.dupe(u8, session.preset_robot);
    errdefer allocator.free(robot_name);
    const scene_name = try allocator.dupe(u8, session.preset_scene);
    errdefer allocator.free(scene_name);
    if (robot_name.len > std.math.maxInt(u8) or scene_name.len > std.math.maxInt(u8)) return error.NameTooLong;

    const meta: abi.SessionMeta = .{
        .tick = session.tick,
        .thruster_len = session.thruster_len,
        .robot_name_len = @intCast(robot_name.len),
        .scene_name_len = @intCast(scene_name.len),
        .pad0 = .{0, 0, 0, 0, 0},
        .commands = session.commands,
    };

    const owned_body_states = try dupRecords(abi.BodyState, allocator, session.backend.states);
    errdefer allocator.free(owned_body_states);
    const owned_wrenches = try dupRecords(abi.Wrench, allocator, session.wrenches);
    errdefer allocator.free(owned_wrenches);
    const owned_fluid = try dupRecords(abi.Wrench, allocator, session.fluid);
    errdefer allocator.free(owned_fluid);
    const owned_prev_linear = try dupVec3Slots(allocator, session.previous_linear);
    errdefer allocator.free(owned_prev_linear);
    const owned_prev_angular = try dupVec3Slots(allocator, session.previous_angular);
    errdefer allocator.free(owned_prev_angular);
    const owned_rotors = try dupRecords(abi.RotorState, allocator, session.rotors[0..]);
    errdefer allocator.free(owned_rotors);
    const owned_contacts = try dupRecords(abi.Contact, allocator, session.backend.contacts.items);
    errdefer allocator.free(owned_contacts);

    @memcpy(owned_body_states, std.mem.sliceAsBytes(session.backend.states));
    @memcpy(owned_wrenches, std.mem.sliceAsBytes(session.wrenches));
    @memcpy(owned_fluid, std.mem.sliceAsBytes(session.fluid));
    @memcpy(owned_rotors, std.mem.sliceAsBytes(session.rotors[0..]));
    @memcpy(owned_contacts, std.mem.sliceAsBytes(session.backend.contacts.items));

    return .{
        .allocator = allocator,
        .meta = meta,
        .robot_name = robot_name,
        .scene_name = scene_name,
        .robot_catalog = robot_catalog,
        .scene_catalog = scene_catalog,
        .body_states = std.mem.bytesAsSlice(abi.BodyState, owned_body_states),
        .wrenches = std.mem.bytesAsSlice(abi.Wrench, owned_wrenches),
        .fluid_wrenches = std.mem.bytesAsSlice(abi.Wrench, owned_fluid),
        .prev_linear = std.mem.bytesAsSlice(abi.Vec3Slot, owned_prev_linear),
        .prev_angular = std.mem.bytesAsSlice(abi.Vec3Slot, owned_prev_angular),
        .rotors = std.mem.bytesAsSlice(abi.RotorState, owned_rotors),
        .contacts = std.mem.bytesAsSlice(abi.Contact, owned_contacts),
        .owned_body_states = owned_body_states,
        .owned_wrenches = owned_wrenches,
        .owned_fluid = owned_fluid,
        .owned_prev_linear = owned_prev_linear,
        .owned_prev_angular = owned_prev_angular,
        .owned_rotors = owned_rotors,
        .owned_contacts = owned_contacts,
    };
}

fn sessionPayloadEnd(entries: []const abi.TableEntry) usize {
    var end: usize = abi.session_directory_at + abi.session_chunk_count * abi.session_entry_size;
    for (entries) |entry| {
        const chunk_end = entry.offset + entry.count * entry.stride;
        if (chunk_end > end) end = chunk_end;
    }
    return end;
}

fn placeSessionChunks(counts: []const u32) ![abi.session_chunk_count]abi.TableEntry {
    var entries: [abi.session_chunk_count]abi.TableEntry = undefined;
    var offset: u32 = abi.session_directory_at + @as(u32, @intCast(abi.session_chunk_count * abi.session_entry_size));
    for (counts, abi.session_strides[0..], &entries) |count, stride, *entry| {
        entry.* = .{ .offset = offset, .count = count, .stride = stride };
        const bytes = count * stride;
        offset = std.mem.alignForward(u32, offset + bytes, 8);
    }
    return entries;
}

fn readSessionHeader(bytes: []const u8) ![abi.session_chunk_count]abi.TableEntry {
    if (bytes.len < abi.session_directory_at + 4) return error.InvalidSession;
    if (!std.mem.eql(u8, bytes[0..abi.session_magic.len], abi.session_magic)) return error.InvalidSession;
    const version = std.mem.readInt(u32, bytes[abi.session_magic.len..][0..4], .little);
    if (version != abi.session_version) return error.UnsupportedSessionVersion;
    const chunk_count = std.mem.readInt(u32, bytes[abi.session_magic.len + 4 ..][0..4], .little);
    if (chunk_count != abi.session_chunk_count) return error.InvalidSession;
    var entries: [abi.session_chunk_count]abi.TableEntry = undefined;
    for (0..abi.session_chunk_count) |i| {
        const at = abi.session_directory_at + i * abi.session_entry_size;
        entries[i] = .{
            .offset = std.mem.readInt(u32, bytes[at..][0..4], .little),
            .count = std.mem.readInt(u32, bytes[at + 4 ..][0..4], .little),
            .stride = std.mem.readInt(u32, bytes[at + 8 ..][0..4], .little),
        };
        if (entries[i].stride != abi.session_strides[i]) return error.InvalidSession;
    }
  const end = sessionPayloadEnd(&entries);
    if (end > bytes.len) return error.InvalidSession;
    return entries;
}

pub fn save(allocator: std.mem.Allocator, snap: *const Snapshot) ![]u8 {
    const strings_len: u32 = @intCast(snap.robot_name.len + snap.scene_name.len);
    const counts = [_]u32{
        strings_len,
        1,
        @intCast(snap.robot_catalog.len),
        @intCast(snap.scene_catalog.len),
        @intCast(snap.body_states.len),
        @intCast(snap.wrenches.len),
        @intCast(snap.fluid_wrenches.len),
        @intCast(snap.prev_linear.len),
        @intCast(snap.prev_angular.len),
        @intCast(snap.rotors.len),
        @intCast(snap.contacts.len),
    };
    const entries = try placeSessionChunks(&counts);
    const end = sessionPayloadEnd(&entries);
    const out = try allocator.alloc(u8, end);
    errdefer allocator.free(out);
    @memset(out, 0);
    @memcpy(out[0..abi.session_magic.len], abi.session_magic);
    std.mem.writeInt(u32, out[abi.session_magic.len..][0..4], abi.session_version, .little);
    std.mem.writeInt(u32, out[abi.session_magic.len + 4 ..][0..4], abi.session_chunk_count, .little);

    const strings_buf = try allocator.alloc(u8, snap.robot_name.len + snap.scene_name.len);
    defer allocator.free(strings_buf);
    @memcpy(strings_buf[0..snap.robot_name.len], snap.robot_name);
    @memcpy(strings_buf[snap.robot_name.len..], snap.scene_name);

    const columns = [_][]const u8{
        strings_buf,
        std.mem.asBytes(&snap.meta),
        snap.robot_catalog,
        snap.scene_catalog,
        std.mem.sliceAsBytes(snap.body_states),
        std.mem.sliceAsBytes(snap.wrenches),
        std.mem.sliceAsBytes(snap.fluid_wrenches),
        std.mem.sliceAsBytes(snap.prev_linear),
        std.mem.sliceAsBytes(snap.prev_angular),
        std.mem.sliceAsBytes(snap.rotors),
        std.mem.sliceAsBytes(snap.contacts),
    };

    for (entries, columns, 0..) |entry, data, i| {
        const at = abi.session_directory_at + i * abi.session_entry_size;
        std.mem.writeInt(u32, out[at..][0..4], entry.offset, .little);
        std.mem.writeInt(u32, out[at + 4 ..][0..4], entry.count, .little);
        std.mem.writeInt(u32, out[at + 8 ..][0..4], entry.stride, .little);
        if (data.len != 0) @memcpy(out[entry.offset..][0..data.len], data);
    }
    return out;
}

pub fn load(allocator: std.mem.Allocator, bytes: []const u8) !Snapshot {
    const entries = try readSessionHeader(bytes);
    const meta_ptr: *const abi.SessionMeta = @ptrCast(@alignCast(
        bytes[entries[abi.session_chunk.meta].offset..].ptr,
    ));
    const meta = meta_ptr.*;
    const strings_entry = entries[abi.session_chunk.strings];
    if (strings_entry.count != meta.robot_name_len + meta.scene_name_len) return error.InvalidSession;
    const strings = bytes[strings_entry.offset..][0..strings_entry.count];
    const robot_name = try allocator.dupe(u8, strings[0..meta.robot_name_len]);
    errdefer allocator.free(robot_name);
    const scene_name = try allocator.dupe(u8, strings[meta.robot_name_len ..][0..meta.scene_name_len]);
    errdefer allocator.free(scene_name);

    const robot_catalog = blk: {
        const e = entries[abi.session_chunk.robot_catalog];
        const slice = try allocator.dupe(u8, bytes[e.offset..][0..e.count]);
        break :blk slice;
    };
    errdefer allocator.free(robot_catalog);
    const scene_catalog = blk: {
        const e = entries[abi.session_chunk.scene_catalog];
        const slice = try allocator.dupe(u8, bytes[e.offset..][0..e.count]);
        break :blk slice;
    };
    errdefer allocator.free(scene_catalog);

    const owned_body_states = try copyChunkOwned(abi.BodyState, allocator, bytes, entries[abi.session_chunk.body_states]);
    errdefer allocator.free(owned_body_states);
    const owned_wrenches = try copyChunkOwned(abi.Wrench, allocator, bytes, entries[abi.session_chunk.wrenches]);
    errdefer allocator.free(owned_wrenches);
    const owned_fluid = try copyChunkOwned(abi.Wrench, allocator, bytes, entries[abi.session_chunk.fluid_wrenches]);
    errdefer allocator.free(owned_fluid);
    const owned_prev_linear = try copyChunkOwned(abi.Vec3Slot, allocator, bytes, entries[abi.session_chunk.prev_linear]);
    errdefer allocator.free(owned_prev_linear);
    const owned_prev_angular = try copyChunkOwned(abi.Vec3Slot, allocator, bytes, entries[abi.session_chunk.prev_angular]);
    errdefer allocator.free(owned_prev_angular);
    const owned_rotors = try copyChunkOwned(abi.RotorState, allocator, bytes, entries[abi.session_chunk.rotors]);
    errdefer allocator.free(owned_rotors);
    const owned_contacts = try copyChunkOwned(abi.Contact, allocator, bytes, entries[abi.session_chunk.contacts]);
    errdefer allocator.free(owned_contacts);

    return .{
        .allocator = allocator,
        .meta = meta,
        .robot_name = robot_name,
        .scene_name = scene_name,
        .robot_catalog = robot_catalog,
        .scene_catalog = scene_catalog,
        .body_states = std.mem.bytesAsSlice(abi.BodyState, owned_body_states),
        .wrenches = std.mem.bytesAsSlice(abi.Wrench, owned_wrenches),
        .fluid_wrenches = std.mem.bytesAsSlice(abi.Wrench, owned_fluid),
        .prev_linear = std.mem.bytesAsSlice(abi.Vec3Slot, owned_prev_linear),
        .prev_angular = std.mem.bytesAsSlice(abi.Vec3Slot, owned_prev_angular),
        .rotors = std.mem.bytesAsSlice(abi.RotorState, owned_rotors),
        .contacts = std.mem.bytesAsSlice(abi.Contact, owned_contacts),
        .owned_body_states = owned_body_states,
        .owned_wrenches = owned_wrenches,
        .owned_fluid = owned_fluid,
        .owned_prev_linear = owned_prev_linear,
        .owned_prev_angular = owned_prev_angular,
        .owned_rotors = owned_rotors,
        .owned_contacts = owned_contacts,
    };
}

fn copyChunkOwned(comptime R: type, allocator: std.mem.Allocator, bytes: []const u8, entry: abi.TableEntry) ![]align(@alignOf(R)) u8 {
    if (entry.count == 0) return &[_]u8{};
    const size = entry.count * entry.stride;
    if (entry.offset + size > bytes.len) return error.InvalidSession;
    const owned = try allocator.alignedAlloc(u8, std.mem.Alignment.of(R), size);
    @memcpy(owned, bytes[entry.offset..][0..size]);
    return owned;
}

pub fn restore(session: *Session, snap: *const Snapshot) !void {
    if (!std.mem.eql(u8, snap.robot_name, session.preset_robot) or !std.mem.eql(u8, snap.scene_name, session.preset_scene))
        return error.SnapshotMismatch;
    if (snap.body_states.len != session.backend.states.len) return error.SnapshotMismatch;
    if (snap.wrenches.len != session.wrenches.len) return error.SnapshotMismatch;
    if (snap.fluid_wrenches.len != session.fluid.len) return error.SnapshotMismatch;
    if (snap.prev_linear.len != session.previous_linear.len) return error.SnapshotMismatch;
    if (snap.prev_angular.len != session.previous_angular.len) return error.SnapshotMismatch;

    try session.backend.reset(snap.body_states);
    @memcpy(session.wrenches, snap.wrenches);
    @memcpy(session.fluid, snap.fluid_wrenches);
    for (snap.prev_linear, session.previous_linear) |r, *v| v.* = r.v;
    for (snap.prev_angular, session.previous_angular) |r, *v| v.* = r.v;
    const rotor_count = @min(snap.rotors.len, session.rotors.len);
    @memcpy(session.rotors[0..rotor_count], snap.rotors[0..rotor_count]);
    session.tick = snap.meta.tick;
    session.commands = snap.meta.commands;
    session.thruster_len = snap.meta.thruster_len;
    session.backend.contacts.clearRetainingCapacity();
    try session.backend.contacts.appendSlice(session.scene.allocator, snap.contacts);
    session.refreshSensors();
}