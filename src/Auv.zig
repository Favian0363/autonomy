const std = @import("std");
const math = @import("math.zig");

/// Constants shared with `include/auv.h`/`include/object_classes.h`.
/// Zig 0.17 removed `@cImport`; the ABI structs below remain `extern` and the
/// C header is compiled into every AUV backend.
pub const c = struct {
    pub const AUV_FRAME_MAX_OBJECTS = 128;
    pub const AUV_COCO_PERSON = 0;
    pub const AUV_COCO_BOTTLE = 39;
    pub const AUV_COCO_TOOTHBRUSH = 79;
    pub const AUV_COCO_COUNT = 80;
    pub const AUV_ROBOSUB_GATE = 0;
    pub const AUV_ROBOSUB_MARKER = 1;
    pub const AUV_ROBOSUB_GATE_OPENING = 2;
};

pub const Object = struct {
    bbox: math.BoundingBox,
    id: u32,
    cls: u8,
};

pub const Point2u = extern struct {
    x: u32,
    y: u32,
};

pub const Object2d = extern struct {
    top_left: Point2u,
    bottom_right: Point2u,
    id: u32,
    cls: u8,
};

/// One flag per axis. False means that component of the pose must not be used.
pub const AxisOk = extern struct {
    x: bool,
    y: bool,
    z: bool,
};

pub const FrameError = extern struct {
    pub const none: u32 = 0;
    pub const sensor: u32 = 2;
    pub const end_of_stream: u32 = 3;
    pub const not_initialized: u32 = 4;
    pub const timeout: u32 = 5;
    pub const initialization: u32 = 6;

    code: u32 = none,
    message: [256]u8 = @splat(0),
};

pub const Frame = struct {
    pub const max_objects = c.AUV_FRAME_MAX_OBJECTS;

    objects: [max_objects]Object,
    objects_len: u8,
    camera_pose: math.Pose,
    timestamp: u64,
    /// World X, Y, Z. A false axis means `camera_pose.pos` on that axis is not usable.
    position_ok: AxisOk,
    /// Roll, pitch, yaw. A false axis means that attitude component is not usable.
    attitude_ok: AxisOk,
    accel: math.Vector3f,
    gyro: math.Vector3f,
    imu_quat: math.Quaternionf,
    objects2d: [max_objects]Object2d,
    objects2d_len: u8,
    image_width: u32,
    image_height: u32,
    pressure_depth: f32,
    pressure_depth_ok: bool,
    frame_error: FrameError = .{},
};

pub const InitFunc = fn () callconv(.c) void;
var diagnostic: [256]u8 = @splat(0);
pub fn lastError() []const u8 {
    return std.mem.sliceTo(&diagnostic, 0);
}

const AbiVector3f = extern struct {
    buf: [3]f32 align(16),
};

const AbiQuaternionf = extern struct {
    buf: [4]f32 align(16),
};

const AbiPose = extern struct {
    pos: AbiVector3f,
    quat: AbiQuaternionf,
};

const AbiBoundingBox = extern struct {
    pose: AbiPose,
    size: AbiVector3f,
};

const AbiObject = extern struct {
    bbox: AbiBoundingBox,
    id: u32,
    cls: u8,
};

const AbiFrame = extern struct {
    objects: [Frame.max_objects]AbiObject,
    objects_len: u8,
    camera_pose: AbiPose,
    timestamp: u64,
    position_ok: AxisOk,
    attitude_ok: AxisOk,
    accel: AbiVector3f,
    gyro: AbiVector3f,
    imu_quat: AbiQuaternionf,
    objects2d: [Frame.max_objects]Object2d,
    objects2d_len: u8,
    image_width: u32,
    image_height: u32,
    pressure_depth: f32,
    pressure_depth_ok: bool,
    frame_error: FrameError,
};

pub const YieldUntilNextFrameFunc = fn (*AbiFrame) callconv(.c) void;
pub const DeinitFunc = fn () callconv(.c) void;

const Auv = @This();

init_impl: *const InitFunc,
yield_until_next_frame_impl: *const YieldUntilNextFrameFunc,
deinit_impl: *const DeinitFunc,

pub fn init(self: Auv) void {
    diagnostic = @splat(0);
    self.init_impl();
}

pub fn yieldUntilNextFrame(self: Auv, frame: *Frame) !void {
    var raw: AbiFrame = undefined;
    diagnostic = @splat(0);
    raw.frame_error = .{};
    self.yield_until_next_frame_impl(&raw);
    frame.frame_error = raw.frame_error;
    if (frame.frame_error.code != FrameError.none) {
        diagnostic = frame.frame_error.message;
        if (frame.frame_error.code == FrameError.initialization) return error.SensorInitFailed;
        return error.SensorFrameFailed;
    }
    const objects_len = @min(@as(usize, raw.objects_len), frame.objects.len);
    for (raw.objects[0..objects_len], frame.objects[0..objects_len]) |source, *dest| {
        dest.* = .{
            .bbox = .{
                .pose = .{ .pos = source.bbox.pose.pos.buf, .quat = source.bbox.pose.quat.buf },
                .size = source.bbox.size.buf,
            },
            .id = source.id,
            .cls = source.cls,
        };
    }
    frame.objects_len = @intCast(objects_len);
    frame.camera_pose = .{ .pos = raw.camera_pose.pos.buf, .quat = raw.camera_pose.quat.buf };
    frame.timestamp = raw.timestamp;
    frame.position_ok = raw.position_ok;
    frame.attitude_ok = raw.attitude_ok;
    frame.accel = raw.accel.buf;
    frame.gyro = raw.gyro.buf;
    frame.imu_quat = raw.imu_quat.buf;
    const objects2d_len = @min(@as(usize, raw.objects2d_len), frame.objects2d.len);
    @memcpy(frame.objects2d[0..objects2d_len], raw.objects2d[0..objects2d_len]);
    frame.objects2d_len = @intCast(objects2d_len);
    frame.image_width = raw.image_width;
    frame.image_height = raw.image_height;
    frame.pressure_depth = raw.pressure_depth;
    frame.pressure_depth_ok = raw.pressure_depth_ok;
}

pub fn deinit(self: Auv) void {
    self.deinit_impl();
}

test "yield preserves plugin errors without reading incomplete sensor data" {
    const Backend = struct {
        fn yield(raw: *AbiFrame) callconv(.c) void {
            // Only the error field is initialized on this failure path.
            raw.frame_error = .{ .code = 0x1234 };
            const message = "custom camera failure";
            @memcpy(raw.frame_error.message[0..message.len], message);
        }
    };
    const auv: Auv = .{ .init_impl = undefined, .yield_until_next_frame_impl = Backend.yield, .deinit_impl = undefined };
    var frame = std.mem.zeroes(Frame);
    frame.timestamp = 42;
    try std.testing.expectError(error.SensorFrameFailed, auv.yieldUntilNextFrame(&frame));
    try std.testing.expectEqual(@as(u32, 0x1234), frame.frame_error.code);
    try std.testing.expectEqualStrings("custom camera failure", lastError());
    try std.testing.expectEqual(@as(u64, 42), frame.timestamp);
}

test "successful yield clears a previous frame error" {
    const Backend = struct {
        fn yield(raw: *AbiFrame) callconv(.c) void {
            raw.* = std.mem.zeroes(AbiFrame);
            raw.timestamp = 123;
            raw.camera_pose.quat.buf[3] = 1;
        }
    };
    const auv: Auv = .{ .init_impl = undefined, .yield_until_next_frame_impl = Backend.yield, .deinit_impl = undefined };
    var frame = std.mem.zeroes(Frame);
    frame.frame_error.code = FrameError.timeout;
    try auv.yieldUntilNextFrame(&frame);
    try std.testing.expectEqual(FrameError.none, frame.frame_error.code);
    try std.testing.expectEqual(@as(u64, 123), frame.timestamp);
    try std.testing.expectEqual(@as(f32, 1), frame.camera_pose.quat[3]);
    try std.testing.expectEqualStrings("", lastError());
}

test "yield reports initialization errors through the frame" {
    const Backend = struct {
        fn yield(raw: *AbiFrame) callconv(.c) void {
            raw.frame_error = .{ .code = FrameError.initialization };
        }
    };
    const auv: Auv = .{ .init_impl = undefined, .yield_until_next_frame_impl = Backend.yield, .deinit_impl = undefined };
    var frame: Frame = undefined;
    try std.testing.expectError(error.SensorInitFailed, auv.yieldUntilNextFrame(&frame));
    try std.testing.expectEqual(FrameError.initialization, frame.frame_error.code);
}

comptime {
    if (c.AUV_COCO_TOOTHBRUSH != 79)
        @compileError("coco class ids must stay in dataset order");
    if (c.AUV_COCO_COUNT != 80)
        @compileError("coco class count must be 80");
    if (c.AUV_ROBOSUB_GATE != 0)
        @compileError("robosub gate must be 0");

    if (@sizeOf(AxisOk) != 3) @compileError("AuvAxisOk ABI drift");
    if (@sizeOf(Point2u) != 8) @compileError("AuvPoint2u ABI drift");
    if (@sizeOf(AbiVector3f) != 16 or @alignOf(AbiVector3f) != 16)
        @compileError("MathVector3f ABI drift");
    if (@sizeOf(AbiQuaternionf) != 16 or @alignOf(AbiQuaternionf) != 16)
        @compileError("MathQuaternionf ABI drift");
    if (@sizeOf(AbiPose) != 32) @compileError("MathPose ABI drift");
    if (@sizeOf(AbiBoundingBox) != 48) @compileError("MathBoundingBox ABI drift");
    if (@sizeOf(AbiObject) != 64) @compileError("AuvObject ABI drift");
    if (@sizeOf(Object2d) != 24) @compileError("AuvObject2d ABI drift");
    if (@offsetOf(AbiFrame, "camera_pose") != 8208) @compileError("AuvFrame camera_pose ABI drift");
    if (@offsetOf(AbiFrame, "objects2d") != 8304) @compileError("AuvFrame objects2d ABI drift");
    if (@sizeOf(FrameError) != 260) @compileError("AuvFrameError ABI drift");
    if (@offsetOf(AbiFrame, "frame_error") != 11396) @compileError("AuvFrame error ABI drift");
    if (@sizeOf(AbiFrame) != 11664) @compileError("AuvFrame ABI drift");
}
