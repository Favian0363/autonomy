const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const MissionConfig = @import("MissionConfig.zig");

/// The compiled profile definitions shared by the runtime and viewer.
pub const CompiledConfig = MissionConfig;
pub const Math = math;
pub const TelemetryTransport = CompiledConfig.TelemetryTransport;

pub const DynLib = if (builtin.os.tag == .windows) WindowsDynLib else std.DynLib;

pub fn loadError() [:0]const u8 {
    if (builtin.os.tag == .windows) return "LoadLibrary failed";
    return std.mem.span(std.c.dlerror()) orelse "NO_DL_ERROR";
}

const WindowsDynLib = struct {
    handle: std.os.windows.HMODULE,

    pub fn open(path: []const u8) error{ FileNotFound, NameTooLong, InvalidWtf8 }!WindowsDynLib {
        var wide: [std.os.windows.PATH_MAX_WIDE]u16 = undefined;
        const len = std.unicode.wtf8ToWtf16Le(wide[0 .. wide.len - 1], path) catch |err| switch (err) {
            error.InvalidWtf8 => return error.InvalidWtf8,
        };
        if (len >= wide.len) return error.NameTooLong;
        wide[len] = 0;
        const handle = load(wide[0..len :0]) orelse return error.FileNotFound;
        return .{ .handle = handle };
    }

    pub fn close(self: *WindowsDynLib) void {
        _ = FreeLibrary(self.handle);
        self.* = undefined;
    }

    pub fn lookup(self: *WindowsDynLib, comptime T: type, name: [:0]const u8) ?T {
        const symbol = GetProcAddress(self.handle, name.ptr) orelse return null;
        return @as(T, @ptrCast(@alignCast(symbol)));
    }
};

extern "kernel32" fn LoadLibraryExW(
    name: [*:0]const u16,
    file: ?std.os.windows.HANDLE,
    flags: u32,
) callconv(.winapi) ?std.os.windows.HMODULE;
extern "kernel32" fn LoadLibraryW(name: [*:0]const u16) callconv(.winapi) ?std.os.windows.HMODULE;
extern "kernel32" fn GetFullPathNameW(
    name: [*:0]const u16,
    buf_len: u32,
    buf: [*]u16,
    file_part: ?*?[*:0]u16,
) callconv(.winapi) u32;
extern "kernel32" fn FreeLibrary(module: std.os.windows.HMODULE) callconv(.winapi) std.os.windows.BOOL;
extern "kernel32" fn GetProcAddress(
    module: std.os.windows.HMODULE,
    name: [*:0]const u8,
) callconv(.winapi) ?*anyopaque;

const dll_load_dir: u32 = 0x00000100;
const dll_default_dirs: u32 = 0x00001000;

fn load(path: [*:0]const u16) ?std.os.windows.HMODULE {
    var has_dir = path[0] != 0 and path[1] == ':';
    var i: usize = 0;
    while (path[i] != 0) : (i += 1) {
        if (path[i] == '\\' or path[i] == '/') has_dir = true;
    }
    if (!has_dir) return LoadLibraryW(path);

    var full: [std.os.windows.PATH_MAX_WIDE]u16 = undefined;
    const n = GetFullPathNameW(path, full.len, &full, null);
    if (n == 0 or n >= full.len)
        return LoadLibraryExW(path, null, dll_load_dir | dll_default_dirs);
    full[n] = 0;
    return LoadLibraryExW(full[0..n :0].ptr, null, dll_load_dir | dll_default_dirs);
}

pub const SerialBinary = struct {
    // Little-endian field order. Slices use a u8 length followed by their items.
    // Struct fields omit C layout padding.
    pub fn write(w: *Io.Writer, value: anytype) !void {
        const T = @TypeOf(value);
        switch (@typeInfo(T)) {
            .int => {
                var buf: [@sizeOf(T)]u8 = undefined;
                std.mem.writeInt(T, &buf, value, .little);
                try w.writeAll(&buf);
            },
            .float => try write(w, @as(u32, @bitCast(@as(f32, @floatCast(value))))),
            .bool => try write(w, @as(u8, @intFromBool(value))),
            .@"enum" => |info| try write(w, @as(info.tag_type, @backingInt(value))),
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
            .@"struct" => |info| inline for (info.field_names) |name| {
                try write(w, @field(value, name));
            },
            else => @compileError("unsupported serial type " ++ @typeName(T)),
        }
    }

    pub fn read(r: *Io.Reader, comptime T: type) !T {
        switch (@typeInfo(T)) {
            .int => {
                var buf: [@sizeOf(T)]u8 = undefined;
                try r.readSliceAll(&buf);
                return std.mem.readInt(T, &buf, .little);
            },
            .float => {
                const bits = try read(r, u32);
                const narrow = @as(f32, @bitCast(bits));
                if (T == f32) return narrow;
                if (T == f64) return @as(f64, narrow);
                @compileError("unsupported float type " ++ @typeName(T));
            },
            .bool => return try read(r, u8) != 0,
            .@"enum" => |info| {
                const raw = try read(r, info.tag_type);
                return std.enums.fromInt(T, raw) orelse return error.InvalidEnum;
            },
            .array => |info| {
                if (info.child == u8) {
                    var out: T = undefined;
                    try r.readSliceAll(&out);
                    return out;
                }
                var out: T = undefined;
                for (&out) |*item| item.* = try read(r, info.child);
                return out;
            },
            .vector => |info| {
                var array: [info.len]info.child = undefined;
                inline for (0..info.len) |i| array[i] = try read(r, info.child);
                return @bitCast(array);
            },
            .pointer => |info| {
                if (info.size != .slice) @compileError("only slices can be read");
                const len: u8 = try read(r, u8);
                if (info.child == u8) {
                    var buf: [256]u8 = undefined;
                    try r.readSliceAll(buf[0..len]);
                    return buf[0..len];
                }
                @compileError("only u8 slices can be read from pointer");
            },
            .@"struct" => |info| {
                var out: T = undefined;
                inline for (info.field_names, info.field_types) |name, field_type| {
                    @field(out, name) = try read(r, field_type);
                }
                return out;
            },
            else => @compileError("unsupported serial type " ++ @typeName(T)),
        }
    }

    pub fn readFromBuffer(comptime T: type, buf: []const u8) !struct { value: T, consumed: usize } {
        var stream: Io.Reader = .fixed(buf);
        const value = try read(&stream, T);
        return .{ .value = value, .consumed = stream.seek };
    }
};

pub const MissionDiynLoader = struct {
    auv_path: []const u8,
    dynlib: DynLib,

    pub fn init(auv_path: []const u8) MissionDiynLoader {
        return .{
            .auv_path = auv_path,
            .dynlib = undefined,
        };
    }

    pub fn load(loader: *MissionDiynLoader) !Auv {
        loader.dynlib = try DynLib.open(loader.auv_path);
        errdefer loader.dynlib.close();

        return .{
            .init_impl = loader.dynlib.lookup(
                *const Auv.InitFunc,
                "auv_init",
            ) orelse return error.MissingAuvInit,
            .yield_until_next_frame_impl = loader.dynlib.lookup(
                *const Auv.YieldUntilNextFrameFunc,
                "auv_yield_until_next_frame",
            ) orelse return error.MissingAuvYieldUntilNextFrame,
            .deinit_impl = loader.dynlib.lookup(
                *const Auv.DeinitFunc,
                "auv_deinit",
            ) orelse return error.MissingAuvDeinit,
        };
    }
};

/// f32 control fields on the live-config wire; runtime `MissionConfig` stores f64.
pub const OdometryPidWire = struct {
    rotation_gate: f32,
    kp: [6]f32,
    ki: [6]f32,
    kd: [6]f32,
};

pub const IbvsWire = struct {
    center_deadband: f32,
    target_height: f32,
    interaction_cols: u8,
    _pad: [3]u8 = .{ 0, 0, 0 },
    interaction: [math.interaction_output_len][MissionConfig.max_ibvs_error_cols]f32,
};

fn widen3(v: [3]f32) math.Vector3f {
    return .{ v[0], v[1], v[2] };
}

fn narrow3(v: math.Vector3f) [3]f32 {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]) };
}

fn widen6(v: [6]f32) math.Vector6f {
    return .{ v[0], v[1], v[2], v[3], v[4], v[5] };
}

fn narrow6(v: math.Vector6f) [6]f32 {
    return .{
        @floatCast(v[0]),
        @floatCast(v[1]),
        @floatCast(v[2]),
        @floatCast(v[3]),
        @floatCast(v[4]),
        @floatCast(v[5]),
    };
}

fn odometryFromWire(wire: OdometryPidWire) MissionConfig.OdometryPid {
    return .{
        .rotation_gate = wire.rotation_gate,
        .kp = widen6(wire.kp),
        .ki = widen6(wire.ki),
        .kd = widen6(wire.kd),
    };
}

fn odometryToWire(odometry: MissionConfig.OdometryPid) OdometryPidWire {
    return .{
        .rotation_gate = @floatCast(odometry.rotation_gate),
        .kp = narrow6(odometry.kp),
        .ki = narrow6(odometry.ki),
        .kd = narrow6(odometry.kd),
    };
}

fn ibvsFromWire(wire: IbvsWire) MissionConfig.Ibvs {
    var interaction: [math.interaction_output_len][MissionConfig.max_ibvs_error_cols]f64 = @splat(@splat(0));
    for (&interaction, &wire.interaction) |*row, source| {
        inline for (0..MissionConfig.max_ibvs_error_cols) |i| row[i] = source[i];
    }
    return .{
        .center_deadband = wire.center_deadband,
        .target_height = wire.target_height,
        .interaction_cols = wire.interaction_cols,
        .interaction = interaction,
    };
}

fn ibvsToWire(ibvs: MissionConfig.Ibvs) IbvsWire {
    var interaction: [math.interaction_output_len][MissionConfig.max_ibvs_error_cols]f32 = @splat(@splat(0));
    for (&interaction, &ibvs.interaction) |*row, source| {
        inline for (0..MissionConfig.max_ibvs_error_cols) |i| row[i] = @floatCast(source[i]);
    }
    return .{
        .center_deadband = @floatCast(ibvs.center_deadband),
        .target_height = @floatCast(ibvs.target_height),
        .interaction_cols = ibvs.interaction_cols,
        .interaction = interaction,
    };
}

/// Editable runtime configuration used by the viewer and the control protocol.
pub const LiveConfigPacket = struct {
    magic: [4]u8 = magic_value,
    wire_version: u16 = wire_version_value,
    camera_forward: [3]f32,
    rotation_limit: [3]f32,
    tam: [MissionConfig.thruster_count][MissionConfig.tam_cols]f32,
    tam_len: u8,
    odometry: OdometryPidWire,
    ibvs: IbvsWire,
    goal_dist_threshold: f32,
    thrustor_saturate: f32,
    log_freq_div: u16,
    queue_capacity: u16,
    print_stuff: bool,
    clear_print_stuff: bool,
    telemetry_transport: TelemetryTransport,
    telemetry_addr: MissionConfig.FixedText(MissionConfig.telemetry_addr_max),
    telemetry_log_path: MissionConfig.FixedText(MissionConfig.telemetry_log_path_max),

    mission_id: MissionConfig.MissionId,
    mission_profile: MissionConfig.MissionProfileId,
    sensor_source: MissionConfig.SensorSource,
    sensor_library: MissionConfig.FixedText(MissionConfig.path_max),
    serial_port: MissionConfig.FixedText(MissionConfig.path_max),
    sim_robot: MissionConfig.FixedText(MissionConfig.name_max),
    sim_scene: MissionConfig.FixedText(MissionConfig.name_max),
    sim_realtime: bool,

    revision: u64 = 0,

    const wire_version_value: u16 = 7;
    const magic_value: [4]u8 = "MCF1".*;

    const DecodeError = error{
        BufferTooShort,
        BadMagic,
        UnsupportedWireVersion,
        InvalidEnum,
        TooLong,
    };

    const ValidateError = error{
        InvalidTamLen,
        InvalidQueueCapacity,
        InvalidInteractionCols,
        InvalidTextLength,
        NonFinite,
    };

    pub fn decode(buf: []const u8) DecodeError!LiveConfigPacket {
        if (buf.len < 4 + 2) return error.BufferTooShort;
        if (!std.mem.eql(u8, buf[0..4], &magic_value)) return error.BadMagic;
        const version = std.mem.readInt(u16, buf[4..6], .little);
        if (version != wire_version_value) return error.UnsupportedWireVersion;

        const parsed = SerialBinary.readFromBuffer(LiveConfigPacket, buf) catch return error.BufferTooShort;
        if (parsed.consumed != buf.len) return error.TooLong;
        if (!std.mem.eql(u8, &parsed.value.magic, &magic_value)) return error.BadMagic;
        if (parsed.value.wire_version != wire_version_value) return error.UnsupportedWireVersion;
        return parsed.value;
    }

    fn requireFinite(v: f32) ValidateError!void {
        if (!std.math.isFinite(v)) return error.NonFinite;
    }

    fn validateTam(
        tam: *const [MissionConfig.thruster_count][MissionConfig.tam_cols]f32,
        tam_len: u8,
    ) ValidateError!void {
        for (tam, 0..) |row, i| {
            if (i >= tam_len) break;
            for (row) |v| try requireFinite(v);
        }
    }

    fn validateOdometry(odometry: *const OdometryPidWire) ValidateError!void {
        try requireFinite(odometry.rotation_gate);
        for (odometry.kp) |v| try requireFinite(v);
        for (odometry.ki) |v| try requireFinite(v);
        for (odometry.kd) |v| try requireFinite(v);
    }

    fn validateIbvs(ibvs: *const IbvsWire) ValidateError!void {
        if (ibvs.interaction_cols == 0 or ibvs.interaction_cols > MissionConfig.max_ibvs_error_cols)
            return error.InvalidInteractionCols;
        try requireFinite(ibvs.center_deadband);
        try requireFinite(ibvs.target_height);
        for (&ibvs.interaction) |row| {
            for (row[0..ibvs.interaction_cols]) |v| try requireFinite(v);
        }
    }

    pub fn validate(packet: *const LiveConfigPacket) !void {
        if (!std.mem.eql(u8, &packet.magic, &magic_value)) return error.BadMagic;
        if (packet.wire_version != wire_version_value) return error.UnsupportedWireVersion;
        if (packet.tam_len == 0 or packet.tam_len > MissionConfig.thruster_count)
            return error.InvalidTamLen;
        if (packet.queue_capacity == 0 or packet.queue_capacity > MissionConfig.queue_capacity_max)
            return error.InvalidQueueCapacity;
        var forward_energy: f32 = 0;
        for (packet.camera_forward) |value| {
            try requireFinite(value);
            forward_energy += value * value;
        }
        if (!std.math.isFinite(forward_energy) or forward_energy <= 1e-12) return error.NonFinite;
        for (packet.rotation_limit) |value| if (std.math.isNan(value) or value < 0) return error.NonFinite;
        try validateTam(&packet.tam, packet.tam_len);
        try validateOdometry(&packet.odometry);
        try validateIbvs(&packet.ibvs);
        try requireFinite(packet.goal_dist_threshold);
        try requireFinite(packet.thrustor_saturate);
        switch (packet.telemetry_transport) {
            .udp => {
                const address = try packet.telemetry_addr.slice();
                if (address.len == 0) return error.InvalidTextLength;
            },
            .stdout, .disabled => {},
        }
        _ = try packet.telemetry_log_path.slice();
        var config = MissionConfig.compiledProfile(packet.mission_profile).*;
        packet.applyToRuntimeConfig(&config);
        try config.validate();
    }

    pub fn fromRuntimeConfig(config: *const MissionConfig.MissionConfig) LiveConfigPacket {
        var tam_wire: [MissionConfig.thruster_count][MissionConfig.tam_cols]f32 = @splat(@splat(0));
        for (0..config.tam_len) |i| {
            inline for (0..MissionConfig.tam_cols) |j| tam_wire[i][j] = @floatCast(config.tam[i][j]);
        }
        return .{
            .magic = magic_value,
            .wire_version = wire_version_value,
            .mission_id = config.mission_id,
            .mission_profile = config.mission_profile,
            .sensor_source = config.sensor_source,
            .sensor_library = config.sensor_library,
            .serial_port = config.serial_port,
            .sim_robot = config.sim_robot,
            .sim_scene = config.sim_scene,
            .sim_realtime = config.sim_realtime,
            .camera_forward = narrow3(config.camera_forward),
            .rotation_limit = narrow3(config.rotation_limit),
            .tam = tam_wire,
            .tam_len = config.tam_len,
            .odometry = odometryToWire(config.odometry),
            .ibvs = ibvsToWire(config.ibvs),
            .goal_dist_threshold = @floatCast(config.goal_dist_threshold),
            .thrustor_saturate = @floatCast(config.thrustor_saturate),
            .log_freq_div = config.log_freq_div,
            .queue_capacity = config.queue_capacity,
            .print_stuff = config.print_stuff,
            .clear_print_stuff = config.clear_print_stuff,
            .telemetry_transport = config.telemetry_transport,
            .telemetry_addr = config.telemetry_addr,
            .telemetry_log_path = config.telemetry_log_path,
        };
    }

    pub fn applyToRuntimeConfig(packet: LiveConfigPacket, dst: *MissionConfig.MissionConfig) void {
        const teleop_port = dst.teleop_port;

        dst.mission_id = packet.mission_id;
        dst.mission_profile = packet.mission_profile;
        dst.sensor_source = packet.sensor_source;
        dst.sensor_library = packet.sensor_library;
        dst.serial_port = packet.serial_port;
        dst.sim_robot = packet.sim_robot;
        dst.sim_scene = packet.sim_scene;
        dst.sim_realtime = packet.sim_realtime;
        dst.camera_forward = widen3(packet.camera_forward);
        dst.rotation_limit = widen3(packet.rotation_limit);
        var tam: [MissionConfig.thruster_count]math.Vector6f = @splat(@splat(0));
        for (0..packet.tam_len) |i| {
            inline for (0..MissionConfig.tam_cols) |j| tam[i][j] = packet.tam[i][j];
        }
        dst.tam = tam;
        dst.tam_len = packet.tam_len;
        dst.odometry = odometryFromWire(packet.odometry);
        dst.ibvs = ibvsFromWire(packet.ibvs);
        dst.goal_dist_threshold = packet.goal_dist_threshold;
        dst.thrustor_saturate = packet.thrustor_saturate;
        dst.log_freq_div = packet.log_freq_div;
        dst.queue_capacity = packet.queue_capacity;
        dst.print_stuff = packet.print_stuff;
        dst.clear_print_stuff = packet.clear_print_stuff;
        dst.telemetry_transport = packet.telemetry_transport;
        dst.telemetry_addr = packet.telemetry_addr;
        dst.telemetry_log_path = packet.telemetry_log_path;

        dst.teleop_port = teleop_port;
    }
};

pub const Goal = struct {
    position: math.Vector3f,
    look_at: math.Vector3f,
};

/// Runtime telemetry data produced by the mission loop for publication.
pub const telemetry_step_max = 64;
pub const telemetry_frame_abi_version: u32 = 2;

pub const TelemetryPose = extern struct {
    pos: [3]f32,
    quat: [4]f32,
};

pub const TelemetryObject = extern struct {
    pose: TelemetryPose,
    size: [3]f32,
    id: u32,
    cls: u8,
    _pad: [3]u8,
};

pub fn telemetryVec3(v: math.Vector3f) [3]f32 {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]) };
}

pub fn telemetryQuat(v: math.Quaternionf) [4]f32 {
    return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]), @floatCast(v[3]) };
}

pub fn telemetryPose(p: math.Pose) TelemetryPose {
    return .{ .pos = telemetryVec3(p.pos), .quat = telemetryQuat(p.quat) };
}

pub fn telemetryVec6(v: math.Vector6f) [6]f32 {
    return .{
        @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]),
        @floatCast(v[3]), @floatCast(v[4]), @floatCast(v[5]),
    };
}

pub fn telemetryFloat(v: f64) f32 {
    return @floatCast(v);
}

pub const TelemetryPoint2u = extern struct {
    x: u32,
    y: u32,
};

pub const TelemetryObject2d = extern struct {
    top_left: TelemetryPoint2u,
    bottom_right: TelemetryPoint2u,
    id: u32,
    cls: u8,
    _pad: [3]u8,
};

pub const TelemetryAxisOk = extern struct {
    x: bool,
    y: bool,
    z: bool,
    _pad: u8,
};

pub const TelemetryGoal = extern struct {
    position: [3]f32,
    look_at: [3]f32,
};

/// Canonical fixed-layout runtime telemetry shared with the viewer.
/// All variable-length data is stored inline with explicit lengths so the same
/// type can be used for direct display, UDP, and logs.
pub const TelemetryFrame = extern struct {
    magic: [4]u8,
    abi_version: u32,
    timestamp: u64,
    camera_pose: TelemetryPose,
    position_ok: TelemetryAxisOk,
    attitude_ok: TelemetryAxisOk,
    depth_ok: bool,
    _pad0: u8,
    _pad1: u16,
    depth: f32,
    objects_len: u16,
    boxes_len: u16,
    thrusters_len: u8,
    step_len: u8,
    goals_len: u16,
    objects: [Auv.Frame.max_objects]TelemetryObject,
    boxes: [Auv.Frame.max_objects]TelemetryObject2d,
    thrusters: [MissionConfig.thruster_count]f32,
    step: [telemetry_step_max]u8,
    force_error: [3]f32,
    goals: [MissionConfig.queue_capacity_max]TelemetryGoal,
    clear_screen: bool,
    _debug_pad: [7]u8,
    cycle_us: u64,
    has_dt: bool,
    _dt_pad: [3]u8,
    dt: f32,
    kp: [6]f32,
    ki: [6]f32,
    kd: [6]f32,
    tam: [MissionConfig.thruster_count][6]f32,
    tam_len: u8,
    _tam_pad: [7]u8,
    control_pose: [6]f32,
    translation_error: [3]f32,
    target_orientation: [4]f32,
    target_quaternion: [4]f32,
    attitude_error: [3]f32,
    rotation_limit: [3]f32,
    input: [6]f32,
};

comptime {
    // LiveConfigPacket is serialized field-by-field by SerialBinary. Keep
    // these anchors in order when changing the control wire schema.
    const live_fields = @typeInfo(LiveConfigPacket).@"struct".field_names;
    if (live_fields.len != 26 or
        !std.mem.eql(u8, live_fields[2], "camera_forward") or
        !std.mem.eql(u8, live_fields[14], "telemetry_transport") or
        !std.mem.eql(u8, live_fields[16], "telemetry_log_path"))
        @compileError("live config packet control fields changed order");
    // Preserve the existing telemetry frame layout for runtime and viewer.
    if (@offsetOf(TelemetryFrame, "timestamp") != 8)
        @compileError("telemetry timestamp ABI drift");
    if (@offsetOf(TelemetryFrame, "objects") <= @offsetOf(TelemetryFrame, "goals_len"))
        @compileError("telemetry payload ABI drift");
    if (@offsetOf(TelemetryFrame, "cycle_us") <= @offsetOf(TelemetryFrame, "clear_screen") or
        @offsetOf(TelemetryFrame, "input") <= @offsetOf(TelemetryFrame, "rotation_limit"))
        @compileError("telemetry diagnostics ABI drift");
}

const outbound_buf_len = 64 * 1024;

pub const UdpPublisher = struct {
    socket: Io.net.Socket,
    dest: Io.net.IpAddress,

    fn init(io: Io, config: MissionConfig.MissionConfig) !UdpPublisher {
        const dest = try Io.net.IpAddress.parseLiteral(try config.telemetryAddr());
        const bind_addr: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
        return .{
            .socket = try bind_addr.bind(io, .{ .mode = .dgram }),
            .dest = dest,
        };
    }

    fn publishEncoded(self: *UdpPublisher, io: Io, encoded: []const u8) void {
        self.socket.send(io, &self.dest, encoded) catch {};
    }

    fn deinit(self: *UdpPublisher, io: Io) void {
        self.socket.close(io);
        self.* = undefined;
    }
};

pub const StdoutPublisher = struct {
    file: Io.File,
    clear_before_frame: bool,

    fn init(io: Io) !StdoutPublisher {
        const file = Io.File.stdout();
        return .{
            .file = file,
            .clear_before_frame = try file.isTty(io),
        };
    }

    fn publishEncoded(self: *StdoutPublisher, io: Io, bytes: []const u8) void {
        if (self.clear_before_frame) {
            self.file.writeStreamingAll(io, "\x1b[2J\x1b[H") catch |err| {
                std.log.err("telemetry stdout clear failed: {s}", .{@errorName(err)});
                return;
            };
        }
        self.file.writeStreamingAll(io, bytes) catch |err| {
            std.log.err("telemetry stdout write failed: {s}", .{@errorName(err)});
        };
    }

    fn deinit(self: *StdoutPublisher) void {
        // stdout is owned by the process, not by the telemetry publisher.
        self.* = undefined;
    }
};

/// Runtime-owned transport selected from the compiled startup configuration.
pub const TelemetryPublisher = union(TelemetryTransport) {
    udp: UdpPublisher,
    stdout: StdoutPublisher,
    disabled: void,

    fn init(io: Io, config: MissionConfig.MissionConfig) !TelemetryPublisher {
        return switch (config.telemetry_transport) {
            .udp => .{ .udp = try .init(io, config) },
            .stdout => .{ .stdout = try .init(io) },
            .disabled => .disabled,
        };
    }

    fn publishEncoded(
        self: *TelemetryPublisher,
        io: Io,
        encoded: ?[]const u8,
        ascii_encoded: ?[]const u8,
    ) void {
        switch (self.*) {
            .udp => |*publisher| if (encoded) |bytes| publisher.publishEncoded(io, bytes),
            .stdout => |*publisher| if (ascii_encoded) |bytes| publisher.publishEncoded(io, bytes),
            .disabled => {},
        }
    }

    fn deinit(self: *TelemetryPublisher, io: Io) void {
        switch (self.*) {
            .udp => |*publisher| publisher.deinit(io),
            .stdout => |*publisher| publisher.deinit(),
            .disabled => {},
        }
        self.* = undefined;
    }
};

pub fn encodeTelemetryFrame(buf: []u8, frame: TelemetryFrame) ![]const u8 {
    var writer: Io.Writer = .fixed(buf);
    try SerialBinary.write(&writer, frame);
    return writer.buffered();
}

fn roundDisplayFloat(value: f32) f32 {
    return @round(value * 100000.0) / 100000.0;
}

fn roundDisplayArray(comptime len: usize, value: [len]f32) [len]f32 {
    var rounded: [len]f32 = undefined;
    for (value, &rounded) |source, *dest| dest.* = roundDisplayFloat(source);
    return rounded;
}

fn roundDisplayPose(value: TelemetryPose) TelemetryPose {
    return .{
        .pos = roundDisplayArray(3, value.pos),
        .quat = roundDisplayArray(4, value.quat),
    };
}

fn writeAsciiFloatList(writer: *Io.Writer, values: anytype) !void {
    try writer.writeAll("[");
    for (values, 0..) |value, i| {
        if (i != 0) try writer.writeAll(",");
        try writer.print("{d:.5}", .{roundDisplayFloat(value)});
    }
    try writer.writeAll("]");
}

fn writeAsciiIntList(writer: *Io.Writer, values: anytype) !void {
    try writer.writeAll("[");
    for (values, 0..) |value, i| {
        if (i != 0) try writer.writeAll(",");
        try writer.print("{}", .{value});
    }
    try writer.writeAll("]");
}

pub fn encodeTelemetryFrameAscii(buf: []u8, frame: TelemetryFrame) ![]const u8 {
    var objects: [Auv.Frame.max_objects]struct {
        pose: TelemetryPose,
        size: [3]f32,
        id: u32,
        cls: u8,
    } = undefined;
    for (frame.objects[0..frame.objects_len], objects[0..frame.objects_len]) |source, *dest| {
        dest.* = .{
            .pose = roundDisplayPose(source.pose),
            .size = roundDisplayArray(3, source.size),
            .id = source.id,
            .cls = source.cls,
        };
    }

    var writer: Io.Writer = .fixed(buf);
    try writer.writeAll("FRAME_BEGIN\n");
    try writer.print("magic={s}\nabi_version={}\ntimestamp={}\n", .{
        frame.magic[0..],
        frame.abi_version,
        frame.timestamp,
    });
    try writer.writeAll("camera_pose.pos=");
    try writeAsciiFloatList(&writer, frame.camera_pose.pos);
    try writer.writeAll("\ncamera_pose.quat=");
    try writeAsciiFloatList(&writer, frame.camera_pose.quat);
    try writer.print("\nposition_ok={},{},{}\nattitude_ok={},{},{}\n", .{
        frame.position_ok.x,
        frame.position_ok.y,
        frame.position_ok.z,
        frame.attitude_ok.x,
        frame.attitude_ok.y,
        frame.attitude_ok.z,
    });
    try writer.print("depth_ok={}\ndepth={d:.5}\n", .{
        frame.depth_ok,
        roundDisplayFloat(frame.depth),
    });
    try writer.print("objects_len={}\n", .{frame.objects_len});
    for (objects[0..frame.objects_len], 0..) |object, i| {
        try writer.print("object[{}].pose.pos=", .{i});
        try writeAsciiFloatList(&writer, object.pose.pos);
        try writer.print("\nobject[{}].pose.quat=", .{i});
        try writeAsciiFloatList(&writer, object.pose.quat);
        try writer.print("\nobject[{}].size=", .{i});
        try writeAsciiFloatList(&writer, object.size);
        try writer.print("\nobject[{}].id={}\nobject[{}].cls={}\n", .{
            i,
            object.id,
            i,
            object.cls,
        });
    }
    try writer.print("boxes_len={}\n", .{frame.boxes_len});
    for (frame.boxes[0..frame.boxes_len], 0..) |box, i| {
        try writer.print("box[{}].top_left=[{},{}]\nbox[{}].bottom_right=[{},{}]\nbox[{}].id={}\nbox[{}].cls={}\n", .{
            i,
            box.top_left.x,
            box.top_left.y,
            i,
            box.bottom_right.x,
            box.bottom_right.y,
            i,
            box.id,
            i,
            box.cls,
        });
    }
    try writer.print("thrusters_len={}\nthrusters=", .{frame.thrusters_len});
    try writeAsciiFloatList(&writer, frame.thrusters[0..frame.thrusters_len]);
    try writer.print("\nstep_len={}\nstep_bytes=", .{frame.step_len});
    try writeAsciiIntList(&writer, frame.step[0..frame.step_len]);
    try writer.writeAll("\nforce_error=");
    try writeAsciiFloatList(&writer, frame.force_error);
    try writer.print("\ngoals_len={}\n", .{frame.goals_len});
    for (frame.goals[0..frame.goals_len], 0..) |goal, i| {
        try writer.print("goal[{}].position=", .{i});
        try writeAsciiFloatList(&writer, goal.position);
        try writer.print("\ngoal[{}].look_at=", .{i});
        try writeAsciiFloatList(&writer, goal.look_at);
        try writer.writeAll("\n");
    }
    try writer.print(
        "clear_screen={}\ncycle_us={}\nhas_dt={}\ndt={d:.5}\n",
        .{ frame.clear_screen, frame.cycle_us, frame.has_dt, roundDisplayFloat(frame.dt) },
    );
    try writer.writeAll("kp=");
    try writeAsciiFloatList(&writer, frame.kp);
    try writer.writeAll("\nki=");
    try writeAsciiFloatList(&writer, frame.ki);
    try writer.writeAll("\nkd=");
    try writeAsciiFloatList(&writer, frame.kd);
    try writer.print("\ntam_len={}\n", .{frame.tam_len});
    for (frame.tam[0..frame.tam_len], 0..) |row, i| {
        try writer.print("tam[{}]=", .{i});
        try writeAsciiFloatList(&writer, row);
        try writer.writeAll("\n");
    }
    try writer.writeAll("control_pose=");
    try writeAsciiFloatList(&writer, frame.control_pose);
    try writer.writeAll("\ntranslation_error=");
    try writeAsciiFloatList(&writer, frame.translation_error);
    try writer.writeAll("\ntarget_orientation=");
    try writeAsciiFloatList(&writer, frame.target_orientation);
    try writer.writeAll("\ntarget_quaternion=");
    try writeAsciiFloatList(&writer, frame.target_quaternion);
    try writer.writeAll("\nattitude_error=");
    try writeAsciiFloatList(&writer, frame.attitude_error);
    try writer.writeAll("\nrotation_limit=");
    try writeAsciiFloatList(&writer, frame.rotation_limit);
    try writer.writeAll("\ninput=");
    try writeAsciiFloatList(&writer, frame.input);
    try writer.writeAll("\nFRAME_END\n");
    return writer.buffered();
}

pub fn encodeMissionCommand(buf: []u8, command: MissionCommand) ![]const u8 {
    var writer: Io.Writer = .fixed(buf);
    var packet = command;
    try packet.validate();
    try SerialBinary.write(&writer, packet);
    return writer.buffered();
}

fn processTelemetry(
    io: Io,
    publisher: *TelemetryPublisher,
    log_file: ?Io.File,
    frame: TelemetryFrame,
    encoded_buf: *[outbound_buf_len]u8,
    ascii_buf: *[outbound_buf_len]u8,
    stdout_enabled: bool,
) void {
    const encoded: ?[]const u8 = if (publisher.* == .udp or log_file != null)
        encodeTelemetryFrame(encoded_buf, frame) catch |err| blk: {
            std.log.err("telemetry encode failed: {s}", .{@errorName(err)});
            break :blk null;
        }
    else
        null;
    const ascii_encoded: ?[]const u8 = if (publisher.* == .stdout and stdout_enabled)
        encodeTelemetryFrameAscii(ascii_buf, frame) catch |err| blk: {
            std.log.err("telemetry ASCII encode failed: {s}", .{@errorName(err)});
            break :blk null;
        }
    else
        null;

    publisher.publishEncoded(io, encoded, ascii_encoded);
    if (log_file) |file| {
        if (encoded) |bytes| file.writeStreamingAll(io, bytes) catch |err| {
            std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
        };
    }
}

/// Decode the fixed telemetry frame used for UDP and logs.
pub fn decodeTelemetryFrame(buf: []const u8, frame: *TelemetryFrame) !void {
    var reader: Io.Reader = .fixed(buf);
    frame.* = try SerialBinary.read(&reader, TelemetryFrame);
    if (!std.mem.eql(u8, &frame.magic, "AUV1")) return error.BadMagic;
    if (frame.abi_version != telemetry_frame_abi_version) return error.UnsupportedAbiVersion;
    if (frame.objects_len > frame.objects.len or frame.boxes_len > frame.boxes.len or
        frame.thrusters_len > frame.thrusters.len or frame.step_len > frame.step.len or
        frame.goals_len > frame.goals.len or frame.tam_len > frame.tam.len)
        return error.InvalidTelemetryLengths;
}

/// Direct command sent over the control socket; values use the active TAM order.
pub const ThrusterCommand = struct {
    magic: [4]u8 = "AUT1".*,
    run_id: u64,
    client_id: u64,
    sequence: u64,
    count: u8,
    values: [MissionConfig.thruster_count]f32,
};

pub const TeleopCommand = struct {
    translation: [3]f32,
    orientation: [4]f32,
};

pub const MissionCommandKind = enum(u8) {
    point,
    object,
};
pub fn teleopCommand(translation: math.Vector3f, orientation: math.Quaternionf) TeleopCommand {
    return .{
        .translation = .{
            @floatCast(translation[0]),
            @floatCast(translation[1]),
            @floatCast(translation[2]),
        },
        .orientation = .{
            @floatCast(orientation[0]),
            @floatCast(orientation[1]),
            @floatCast(orientation[2]),
            @floatCast(orientation[3]),
        },
    };
}

pub fn teleopTranslation(command: TeleopCommand) math.Vector3f {
    return .{ command.translation[0], command.translation[1], command.translation[2] };
}

pub fn teleopOrientation(command: TeleopCommand) math.Quaternionf {
    return .{ command.orientation[0], command.orientation[1], command.orientation[2], command.orientation[3] };
}


pub const MissionManeuver = enum(u8) {
    move,
    pass,
    circle,
};

pub const MissionCommand = struct {
    magic: [4]u8 = mission_command_magic,
    wire_version: u16 = mission_command_wire_version,
    kind: MissionCommandKind,
    maneuver: MissionManeuver,
    position: math.Vector3f = @splat(0),
    look_direction: math.Vector3f = .{ 0, 0, 1 },
    object_class: u8 = 0,

    pub const DecodeError = error{
        BufferTooShort,
        InvalidLength,
        BadMagic,
        UnsupportedWireVersion,
        InvalidEnum,
        NonFinite,
        InvalidLookDirection,
        InvalidPointManeuver,
    };

    pub fn decode(buf: []const u8) DecodeError!MissionCommand {
        if (buf.len < 6) return error.BufferTooShort;
        if (!std.mem.eql(u8, buf[0..4], &mission_command_magic)) return error.BadMagic;
        if (std.mem.readInt(u16, buf[4..6], .little) != mission_command_wire_version)
            return error.UnsupportedWireVersion;

        const parsed = SerialBinary.readFromBuffer(MissionCommand, buf) catch |err| switch (err) {
            error.InvalidEnum => return error.InvalidEnum,
            else => return error.BufferTooShort,
        };
        if (parsed.consumed != buf.len) return error.InvalidLength;

        var command = parsed.value;
        try command.validate();
        return command;
    }

    pub fn validate(self: *MissionCommand) DecodeError!void {
        if (!std.mem.eql(u8, &self.magic, &mission_command_magic)) return error.BadMagic;
        if (self.wire_version != mission_command_wire_version) return error.UnsupportedWireVersion;
        if (self.kind == .point and self.maneuver != .move)
            return error.InvalidPointManeuver;

        inline for (0..3) |i| {
            if (!std.math.isFinite(self.position[i])) return error.NonFinite;
            if (!std.math.isFinite(self.look_direction[i])) return error.NonFinite;
        }

        if (self.kind == .point) {
            const length = math.length3f(self.look_direction);
            if (!std.math.isFinite(length) or length <= 1e-6)
                return error.InvalidLookDirection;
            self.look_direction /= @as(math.Vector3f, @splat(length));
        }
    }
};

const mission_command_magic: [4]u8 = "AUM1".*;
const mission_command_wire_version: u16 = 1;

pub const Telemetry = struct {
    state: RuntimeState = .idle,
    result: RuntimeResult = .none,
    revision: u64 = 1,
    last_error: MissionConfig.FixedText(256) = .{},
    telemetry_error: MissionConfig.FixedText(256) = .{},
    run_id: u64 = 0,
    direct_command: ?ThrusterCommand = null,
    direct_last_received: ?Io.Clock.Timestamp = null,
    teleop_command: ?TeleopCommand = null,
    mission_command: ?MissionCommand = null,
    pending_lifecycle: ?error{ MissionRestart, MissionStopped } = null,
    accepted_client_id: u64 = 0,
    accepted_request_id: u64 = 0,
    clients: [16]struct { id: u64 = 0, request: u64 = 0 } = @splat(.{}),
    client_slot: usize = 0,

    publisher: TelemetryPublisher,
    log_file: ?Io.File,
    encoded_buf: [outbound_buf_len]u8 = undefined,
    ascii_buf: [outbound_buf_len]u8 = undefined,
    stdout_log_counter: u64 = 0,
    stdout_log_freq_div: u16,
    control_socket: ?Io.net.Socket = null,

    initial_config_packet: LiveConfigPacket = undefined,

    pub fn init(io: Io, config: MissionConfig.MissionConfig, enable_udp_control: bool) !Telemetry {
        var publisher = try TelemetryPublisher.init(io, config);
        errdefer publisher.deinit(io);

        const control_socket: ?Io.net.Socket = if (enable_udp_control) blk: {
            var control_addr_buf: [48]u8 = undefined;
            const control_addr_text = try std.fmt.bufPrint(&control_addr_buf, "0.0.0.0:{d}", .{config.teleop_port});
            const control_bind = try Io.net.IpAddress.parseLiteral(control_addr_text);
            break :blk control_bind.bind(io, .{ .mode = .dgram, .protocol = .udp }) catch |err| switch (err) {
                error.AddressInUse => {
                    std.log.err("control UDP port {d} is already in use", .{config.teleop_port});
                    return error.AddressInUse;
                },
                else => return err,
            };
        } else null;
        errdefer if (control_socket) |socket| socket.close(io);

        const log_file: ?Io.File = if (try config.telemetryLogPath()) |path|
            try Io.Dir.cwd().createFile(io, path, .{})
        else
            null;
        errdefer if (log_file) |file| file.close(io);

        const telemetry: Telemetry = .{
            .publisher = publisher,
            .log_file = log_file,
            .stdout_log_freq_div = config.log_freq_div,
            .control_socket = control_socket,
            .initial_config_packet = LiveConfigPacket.fromRuntimeConfig(&config),
        };
        return telemetry;
    }

    pub fn deinit(self: *Telemetry, io: Io) void {
        if (self.log_file) |file| file.close(io);
        self.publisher.deinit(io);
        if (self.control_socket) |socket| socket.close(io);
        self.* = undefined;
    }

    /// Replace outputs transactionally; the listener never changes ownership.
    pub fn reconfigure(self: *Telemetry, io: Io, config: MissionConfig.MissionConfig) !void {
        const previous = self.initial_config_packet;
        const publisher_changed = config.telemetry_transport != previous.telemetry_transport or
            (config.telemetry_transport == .udp and !MissionConfig.textEqual(config.telemetry_addr, previous.telemetry_addr));
        var replacement: ?TelemetryPublisher = if (publisher_changed) try TelemetryPublisher.init(io, config) else null;
        errdefer if (replacement) |*publisher| publisher.deinit(io);
        const log_changed = !MissionConfig.textEqual(config.telemetry_log_path, previous.telemetry_log_path);
        const log_file: ?Io.File = if (log_changed) blk: {
            if (try config.telemetryLogPath()) |path| break :blk try Io.Dir.cwd().createFile(io, path, .{ .truncate = false });
            break :blk null;
        } else self.log_file;
        if (replacement) |publisher| {
            self.publisher.deinit(io);
            self.publisher = publisher;
        }
        if (log_changed) {
            if (self.log_file) |file| file.close(io);
            self.log_file = log_file;
        }
        self.stdout_log_freq_div = config.log_freq_div;
        self.initial_config_packet = LiveConfigPacket.fromRuntimeConfig(&config);
    }

    pub fn report(self: *Telemetry, err: anyerror) void {
        const detail = if (err == error.SensorInitFailed or err == error.SensorFrameFailed) Auv.lastError() else "";
        setText(&self.last_error, if (detail.len != 0) detail else @errorName(err));
        std.log.err("mission: {s}", .{self.last_error.slice() catch @errorName(err)});
    }

    pub fn clearCommands(self: *Telemetry) void {
        self.direct_command = null;
        self.direct_last_received = null;
        self.teleop_command = null;
        self.mission_command = null;
    }

    pub fn takeTeleopCommand(self: *Telemetry) ?TeleopCommand {
        const command = self.teleop_command;
        self.teleop_command = null;
        return command;
    }

    pub fn takeMissionCommand(self: *Telemetry) ?MissionCommand {
        const command = self.mission_command;
        self.mission_command = null;
        return command;
    }

    pub fn applyConfig(self: *Telemetry, io: Io, current: *MissionConfig.MissionConfig, packet: LiveConfigPacket) !void {
        try packet.validate();
        if (packet.sensor_source != current.sensor_source) return error.SensorSourceIsCompiled;
        if (packet.revision != self.revision) {
            // A repeated accepted packet is an acknowledgment retry, not a new edit.
            var current_packet = LiveConfigPacket.fromRuntimeConfig(current);
            current_packet.revision = packet.revision;
            if (std.meta.eql(packet, current_packet)) return;
            return error.StaleConfig;
        }
        var next = current.*;
        packet.applyToRuntimeConfig(&next);
        const restart = !MissionConfig.MissionConfig.operationallyEqual(current, &next);
        // Open replacement telemetry resources before committing the new config.
        try self.reconfigure(io, next);
        self.last_error = .{};
        self.telemetry_error = .{};
        current.* = next;
        self.revision += 1;
        if (restart) self.pending_lifecycle = error.MissionRestart;
    }

    pub fn acceptRequest(self: *Telemetry, request: RuntimeRequest) void {
        if (request.action == .query) {
            self.accepted_client_id = request.client_id;
            self.accepted_request_id = 0;
            for (self.clients) |client| if (client.id == request.client_id) {
                self.accepted_request_id = client.request;
                break;
            };
            return;
        }
        if (request.client_id == 0 or request.request_id == 0) {
            self.report(error.InvalidRequestId);
            return;
        }
        var slot: ?usize = null;
        for (self.clients, 0..) |client, i| if (client.id == request.client_id) {
            slot = i;
            break;
        };
        const index = slot orelse blk: {
            const index = self.client_slot;
            self.client_slot = (self.client_slot + 1) % self.clients.len;
            self.clients[index] = .{ .id = request.client_id };
            break :blk index;
        };
        if (request.request_id <= self.clients[index].request) {
            self.accepted_client_id = request.client_id;
            self.accepted_request_id = self.clients[index].request;
            return;
        }
        self.clients[index].request = request.request_id;
        self.accepted_client_id = request.client_id;
        self.accepted_request_id = request.request_id;
        switch (request.action) {
            .query => unreachable,
            .start => if (self.state == .idle or (if (self.pending_lifecycle) |pending| pending == error.MissionStopped else false)) {
                self.pending_lifecycle = error.MissionRestart;
            },
            .stop => self.pending_lifecycle = error.MissionStopped,
            .restart => self.pending_lifecycle = error.MissionRestart,
        }
    }

    pub fn status(self: *const Telemetry, current: *const MissionConfig.MissionConfig, config_reply_revision: u64, config_rejected: bool) RuntimeStatus {
        var desired = LiveConfigPacket.fromRuntimeConfig(current);
        desired.revision = self.revision;
        var active = LiveConfigPacket.fromRuntimeConfig(current);
        // Publication resources belong to the application and can differ from the
        // desired defaults after a failed startup publisher open.
        active.telemetry_transport = self.initial_config_packet.telemetry_transport;
        active.telemetry_addr = self.initial_config_packet.telemetry_addr;
        active.telemetry_log_path = self.initial_config_packet.telemetry_log_path;
        return .{
            .run_id = self.run_id,
            .state = if (self.pending_lifecycle != null and self.state != .idle) .stopping else self.state,
            .result = self.result,
            .revision = self.revision,
            .accepted_client_id = self.accepted_client_id,
            .accepted_request_id = self.accepted_request_id,
            .config_reply_revision = config_reply_revision,
            .config_rejected = config_rejected,
            .active = self.state != .idle,
            .desired = desired,
            .active_config = active,
            .last_error = if (self.last_error.len != 0) self.last_error else self.telemetry_error,
        };
    }

    fn reply(self: *Telemetry, io: Io, current: *const MissionConfig.MissionConfig, destination: *const Io.net.IpAddress, config_reply_revision: u64, config_rejected: bool) void {
        const socket = self.control_socket orelse return;
        var buffer: [8192]u8 = undefined;
        var writer: Io.Writer = .fixed(&buffer);
        SerialBinary.write(&writer, self.status(current, config_reply_revision, config_rejected)) catch return;
        socket.send(io, destination, writer.buffered()) catch {};
    }

    pub fn acceptThrusters(self: *Telemetry, io: Io, current: *const MissionConfig.MissionConfig, command: ThrusterCommand) void {
        if (self.state != .running or self.pending_lifecycle != null or
            current.mission_id != .thruster_control or command.run_id != self.run_id or
            command.client_id == 0 or command.sequence == 0 or command.count != current.tam_len or
            command.count > command.values.len) return;
        for (command.values[0..command.count]) |value| {
            if (!std.math.isFinite(value) or value < -1 or value > 1) return;
        }
        if (self.direct_command) |previous| {
            if (command.client_id != previous.client_id or command.sequence < previous.sequence) return;
            if (command.sequence == previous.sequence and !std.mem.eql(f32, &command.values, &previous.values)) return;
        }
        self.direct_command = command;
        self.direct_last_received = .now(io, .awake);
    }

    pub fn acceptTeleop(self: *Telemetry, command: TeleopCommand) void {
        if (self.state == .running and self.pending_lifecycle == null) self.teleop_command = command;
    }

    pub fn acceptMission(self: *Telemetry, input: MissionCommand) !void {
        var command = input;
        try command.validate();
        self.queueMission(command);
    }

    fn queueMission(self: *Telemetry, command: MissionCommand) void {
        if (self.state == .running and self.pending_lifecycle == null) self.mission_command = command;
    }

    /// Drain a bounded batch before unwinding, so only the latest requested
    /// configuration is opened by the next mission attempt.
    pub fn pollControl(self: *Telemetry, io: Io, current: *MissionConfig.MissionConfig) !void {
        if (self.direct_last_received) |last| {
            if (last.untilNow(io).raw.toMilliseconds() >= 2000) {
                setText(&self.last_error, "Thruster control connection timed out");
                self.clearCommands();
                return error.MissionStopped;
            }
        }
        var buffer: [4096]u8 = undefined;
        const timeout: Io.Timeout = .{ .duration = .{ .clock = .awake, .raw = .fromNanoseconds(0) } };
        if (self.control_socket) |socket| {
            for (0..64) |_| {
                const message = socket.receiveTimeout(io, &buffer, timeout) catch |err| switch (err) {
                    error.Timeout => break,
                    else => {
                        self.report(err);
                        break;
                    },
                };
                var config_reply_revision: u64 = 0;
                var config_rejected = false;
                if (message.data.len >= 4 and std.mem.eql(u8, message.data[0..4], "ACR1")) {
                    const decoded = SerialBinary.readFromBuffer(RuntimeRequest, message.data) catch |err| {
                        self.report(err);
                        self.reply(io, current, &message.from, 0, false);
                        continue;
                    };
                    if (decoded.consumed != message.data.len) self.report(error.InvalidLength) else self.acceptRequest(decoded.value);
                } else if (message.data.len >= 4 and std.mem.eql(u8, message.data[0..4], "MCF1")) {
                    const packet = LiveConfigPacket.decode(message.data) catch |err| {
                        self.report(err);
                        self.reply(io, current, &message.from, 0, false);
                        continue;
                    };
                    config_reply_revision = packet.revision;
                    self.applyConfig(io, current, packet) catch |err| {
                        self.report(err);
                        config_rejected = true;
                    };
                } else if (message.data.len >= 4 and std.mem.eql(u8, message.data[0..4], "AUT1")) {
                    const decoded = SerialBinary.readFromBuffer(ThrusterCommand, message.data) catch continue;
                    if (decoded.consumed == message.data.len) self.acceptThrusters(io, current, decoded.value);
                    continue;
                } else if (message.data.len == 28) {
                    const decoded = SerialBinary.readFromBuffer(TeleopCommand, message.data) catch continue;
                    self.acceptTeleop(decoded.value);
                    continue;
                } else {
                    const command = MissionCommand.decode(message.data) catch |err| {
                        self.report(err);
                        self.reply(io, current, &message.from, 0, false);
                        continue;
                    };
                    self.queueMission(command);
                    continue;
                }
                self.reply(io, current, &message.from, config_reply_revision, config_rejected);
            }
        }
        const lifecycle = self.pending_lifecycle;
        self.pending_lifecycle = null;
        if (lifecycle) |err| {
            self.clearCommands();
            return err;
        }
    }

    /// Publish one runtime telemetry frame synchronously.
    pub fn publishTelemetry(
        self: *Telemetry,
        io: Io,
        frame: *const TelemetryFrame,
    ) void {
        if (self.publisher == .disabled and self.log_file == null) return;
        self.stdout_log_counter +%= 1;
        const stdout_enabled = self.stdout_log_freq_div != 0 and
            self.stdout_log_counter % self.stdout_log_freq_div == 0;
        processTelemetry(
            io,
            &self.publisher,
            self.log_file,
            frame.*,
            &self.encoded_buf,
            &self.ascii_buf,
            stdout_enabled,
        );
    }
};

// Control replies use the control socket, including when no sensor is available.
pub const RuntimeState = enum(u8) { idle, starting, running, stopping };
pub const RuntimeResult = enum(u8) { none, completed, stopped, failed };
pub const RuntimeAction = enum(u8) { query, start, stop, restart };
pub const RuntimeRequest = struct {
    magic: [4]u8 = "ACR1".*,
    client_id: u64,
    request_id: u64,
    action: RuntimeAction,
};
pub const RuntimeStatus = struct {
    magic: [4]u8 = "ACS1".*,
    state: RuntimeState,
    result: RuntimeResult,
    revision: u64,
    accepted_client_id: u64,
    accepted_request_id: u64,
    config_reply_revision: u64,
    config_rejected: bool,
    active: bool,
    desired: LiveConfigPacket,
    active_config: LiveConfigPacket,
    last_error: MissionConfig.FixedText(256),
    run_id: u64,
};

pub fn setText(destination: anytype, message: []const u8) void {
    destination.* = .{};
    const len = @min(destination.bytes.len, message.len);
    @memcpy(destination.bytes[0..len], message[0..len]);
    destination.len = @intCast(len);
}

test "lifecycle requests deduplicate and the latest request wins" {
    var telemetry: Telemetry = .{ .publisher = undefined, .log_file = null, .stdout_log_freq_div = 1, .state = .running };
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 1, .action = .start });
    try std.testing.expect(telemetry.pending_lifecycle == null);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 2, .action = .restart });
    try std.testing.expectEqual(error.MissionRestart, telemetry.pending_lifecycle.?);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 3, .action = .stop });
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 2, .action = .restart });
    try std.testing.expectEqual(error.MissionStopped, telemetry.pending_lifecycle.?);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 3, .action = .query });
    try std.testing.expectEqual(@as(u64, 3), telemetry.accepted_request_id);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 4, .action = .start });
    try std.testing.expectEqual(error.MissionRestart, telemetry.pending_lifecycle.?);
}

test "direct control works without UDP and clears commands on restart" {
    const io = std.testing.io;
    var config = MissionConfig.startup_config;
    config.telemetry_transport = .disabled;
    var telemetry: Telemetry = try .init(io, config, false);
    defer telemetry.deinit(io);
    try std.testing.expect(telemetry.control_socket == null);

    telemetry.state = .running;
    try telemetry.acceptMission(.{ .kind = .point, .maneuver = .move, .look_direction = .{ 0, 4, 0 } });
    try std.testing.expectEqual(math.Vector3f{ 0, 1, 0 }, telemetry.takeMissionCommand().?.look_direction);

    var edit = LiveConfigPacket.fromRuntimeConfig(&config);
    edit.revision = telemetry.revision;
    edit.mission_id = .teleop;
    try telemetry.applyConfig(io, &config, edit);
    telemetry.acceptTeleop(.{ .translation = .{ 1, 0, 0 }, .orientation = .{ 0, 0, 0, 1 } });
    try std.testing.expect(telemetry.takeTeleopCommand() == null);
    try std.testing.expectError(error.MissionRestart, telemetry.pollControl(io, &config));
    try std.testing.expectEqual(MissionConfig.MissionId.teleop, config.mission_id);
}
