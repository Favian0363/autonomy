const std = @import("std");
const math = @import("math.zig");

const c = @cImport({
    @cInclude("auv.h");
});

pub const ObjectCls = enum(u8) {
    cube,
    rect,
    gate,
};
pub const Object = extern struct {
    bbox: math.BoundingBox,
    id: u32,
    cls: ObjectCls,
};

pub const Point2u = extern struct {
    x: u32,
    y: u32,
};

pub const Object2d = extern struct {
    top_left: Point2u,
    bottom_right: Point2u,
    id: u32,
    cls: ObjectCls,
};

pub const Frame = extern struct {
    pub const max_objects = c.AUV_FRAME_MAX_OBJECTS;

    objects: [max_objects]Object,
    objects_len: u8,
    camera_pose: math.Pose,
    timestamp: u64,
    tracking_ok: bool,
    accel: math.Vector3f,
    gyro: math.Vector3f,
    imu_quat: math.Quaternionf,
    objects2d: [max_objects]Object2d,
    objects2d_len: u8,
    image_width: u32,
    image_height: u32,
    pressure_depth: f32,
    pressure_depth_ok: bool,
};

pub const InitFunc = fn () callconv(.c) void;
pub const YieldUntilNextFrameFunc = fn (frame: *Frame) callconv(.c) void;
pub const SetThrustorValuesFunc = fn (thrustor_values: [*c]const f32, thrustor_values_len: u8) callconv(.c) void;
pub const DeinitFunc = fn () callconv(.c) void;

const Auv = @This();

init: *const InitFunc,
yieldUntilNextFrame: *const YieldUntilNextFrameFunc,
setThrustorValues: *const SetThrustorValuesFunc,
deinit: *const DeinitFunc,

// below I'm double checking the ABI matches
fn assertSameLayout(comptime Zig: type, comptime C: type) void {
    if (@sizeOf(Zig) != @sizeOf(C))
        @compileError("size mismatch");

    if (@alignOf(Zig) != @alignOf(C))
        @compileError("alignment mismatch");

    const zig_info = @typeInfo(Zig);
    const c_info = @typeInfo(C);

    if (zig_info == .vector) {
        if (c_info != .@"struct")
            @compileError("expected C struct for Zig vector");

        const c_fields = c_info.@"struct".fields;

        if (c_fields.len != 1)
            @compileError("expected C struct with one field");

        const CField = c_fields[0].type;
        const c_field_info = @typeInfo(CField);

        if (c_field_info != .array)
            @compileError("expected C vector field to be an array");

        if (c_field_info.array.child != zig_info.vector.child)
            @compileError("vector element type mismatch");

        return;
    }

    if (zig_info != .@"struct" or c_info != .@"struct")
        return;

    const zig_fields = zig_info.@"struct".fields;

    if (zig_fields.len != c_info.@"struct".fields.len)
        @compileError("field count mismatch");

    inline for (zig_fields) |field| {
        const name = field.name;
        const ZigField = field.type;
        const CField = @FieldType(C, name);

        if (@offsetOf(Zig, name) != @offsetOf(C, name))
            @compileError("field offset mismatch");

        if (@sizeOf(ZigField) != @sizeOf(CField))
            @compileError("field size mismatch");

        assertSameLayout(ZigField, CField);
    }
}

fn assertSameFunctionLayout(comptime Zig: type, comptime C: type) void {
    const zig_info = @typeInfo(Zig).@"fn";
    const c_info = @typeInfo(C).@"fn";

    if (zig_info.params.len != c_info.params.len)
        @compileError("parameter count mismatch");

    inline for (zig_info.params, c_info.params) |zp, cp| {
        assertSameLayout(
            zp.type orelse @compileError("missing Zig parameter type"),
            cp.type orelse @compileError("missing C parameter type"),
        );
    }

    assertSameLayout(
        zig_info.return_type orelse @compileError("missing Zig return type"),
        c_info.return_type orelse @compileError("missing C return type"),
    );
}

comptime {
    assertSameLayout(math.Vector3f, c.MathVector3f);
    assertSameLayout(math.Quaternionf, c.MathQuaternionf);
    assertSameLayout(math.Vector6f, c.MathVector6f);
    assertSameLayout(math.Pose, c.MathPose);
    assertSameLayout(math.BoundingBox, c.MathBoundingBox);
    assertSameLayout(ObjectCls, c.AuvObjectCls);
    assertSameLayout(Object, c.AuvObject);
    assertSameLayout(Point2u, c.AuvPoint2u);
    assertSameLayout(Object2d, c.AuvObject2d);
    assertSameLayout(Frame, c.AuvFrame);

    assertSameFunctionLayout(InitFunc, c.AuvInitFunc);
    assertSameFunctionLayout(YieldUntilNextFrameFunc, c.AuvYieldUntilNextFrameFunc);
    assertSameFunctionLayout(SetThrustorValuesFunc, c.AuvSetThrustorsInputFunc);
    assertSameFunctionLayout(DeinitFunc, c.AuvDeinitFunc);
}
