const std = @import("std");
const types = @import("types.zig");
const S = @import("support.zig");
const Scene = @import("Scene.zig");
const Runtime = @import("Runtime.zig");
const State = @This();

fn extend(lo: *types.Vec3, hi: *types.Vec3, p: types.Vec3) void {
    var k: usize = 0;
    while (k < 3) : (k += 1) {
        lo[k] = @min(lo[k], p[k]);
        hi[k] = @max(hi[k], p[k]);
    }
}

fn objectBounds(object: types.Object, lo: *types.Vec3, hi: *types.Vec3) void {
    @setFloatMode(.strict);
    lo.* = S.v(std.math.inf(f64), std.math.inf(f64), std.math.inf(f64));
    hi.* = S.v(-std.math.inf(f64), -std.math.inf(f64), -std.math.inf(f64));
    for (object.colliders) |shape| {
        if (shape.shape == .mesh) {
            for (shape.shape.mesh.positions) |position| extend(lo, hi, S.point(shape.pose, position));
        } else {
            var size = shape.size;
            if (shape.shape == .sphere) size = S.v(size[0], size[0], size[0]);
            if (shape.shape == .cylinder) size[1] = size[0];
            var k: u32 = 0;
            while (k < 8) : (k += 1) {
                extend(lo, hi, S.point(shape.pose, S.v(
                    if (k & 1 != 0) size[0] else -size[0],
                    if (k & 2 != 0) size[1] else -size[1],
                    if (k & 4 != 0) size[2] else -size[2],
                )));
            }
        }
    }
}

fn project(camera: types.Camera, corners: [8]types.Vec3, out: *types.Detection) bool {
    @setFloatMode(.strict);
    var x0: f64 = std.math.inf(f64);
    var y0: f64 = std.math.inf(f64);
    var x1: f64 = -std.math.inf(f64);
    var y1: f64 = -std.math.inf(f64);
    const focal = @as(f64, @floatFromInt(camera.width)) * 0.5 / @tan(camera.horizontal_fov * std.math.pi / 360);
    const extendPixel = struct {
        fn apply(px0: *f64, py0: *f64, px1: *f64, py1: *f64, cam: types.Camera, f: f64, p: types.Vec3) void {
            const x = @as(f64, @floatFromInt(cam.width)) * 0.5 + f * p[0] / p[2];
            const y = @as(f64, @floatFromInt(cam.height)) * 0.5 + f * p[1] / p[2];
            px0.* = @min(px0.*, x);
            px1.* = @max(px1.*, x);
            py0.* = @min(py0.*, y);
            py1.* = @max(py1.*, y);
        }
    }.apply;
    var a: u32 = 0;
    while (a < 8) : (a += 1) {
        var axis: u32 = 0;
        while (axis < 3) : (axis += 1) {
            const b = a ^ (@as(u32, 1) << @intCast(axis));
            if (a > b) continue;
            const p = corners[a];
            const delta = S.sub(corners[b], p);
            var first: f64 = 0;
            var last: f64 = 1;
            if (@abs(delta[2]) < 1e-15) {
                if (p[2] < camera.near_range or p[2] > camera.far_range) continue;
            } else {
                var near = (camera.near_range - p[2]) / delta[2];
                var far = (camera.far_range - p[2]) / delta[2];
                if (near > far) std.mem.swap(f64, &near, &far);
                first = @max(first, near);
                last = @min(last, far);
                if (first > last) continue;
            }
            extendPixel(&x0, &y0, &x1, &y1, camera, focal, S.add(p, S.scale(delta, first)));
            extendPixel(&x0, &y0, &x1, &y1, camera, focal, S.add(p, S.scale(delta, last)));
        }
    }
    if (!std.math.isFinite(x0) or x1 < 0 or y1 < 0 or x0 >= @as(f64, @floatFromInt(camera.width)) or y0 >= @as(f64, @floatFromInt(camera.height))) return false;
    const width: f64 = @floatFromInt(camera.width - 1);
    const height: f64 = @floatFromInt(camera.height - 1);
    out.x0 = @intFromFloat(@floor(std.math.clamp(x0, 0, width)));
    out.y0 = @intFromFloat(@floor(std.math.clamp(y0, 0, height)));
    out.x1 = @intFromFloat(@ceil(std.math.clamp(x1, 0, width)));
    out.y1 = @intFromFloat(@ceil(std.math.clamp(y1, 0, height)));
    return true;
}

fn cameraSample(
    scene: *const Scene,
    sensor: types.Sensor,
    camera: types.Camera,
    bodies: []const types.BodyState,
    objects: []types.Detection,
    mount: types.Pose,
) []const types.Detection {
    @setFloatMode(.strict);
    var count: usize = 0;
    const inverse = S.conjugate(mount.quat);
    for (scene.objects, bodies, 0..) |object, body, index| {
        if (!object.body.tracked or index == @backingInt(sensor.body)) continue;
        var lo: types.Vec3 = undefined;
        var hi: types.Vec3 = undefined;
        objectBounds(object, &lo, &hi);
        var observation: types.Detection = .{
            .id = object.body.id,
            .cls = object.body.cls,
            .pose = .{ .pos = S.point(body.pose, S.scale(S.add(lo, hi), 0.5)), .quat = body.pose.quat },
            .size = S.sub(hi, lo),
        };
        var corners: [8]types.Vec3 = undefined;
        var k: u32 = 0;
        while (k < 8) : (k += 1) {
            corners[k] = S.rotate(inverse, S.sub(S.point(body.pose, S.v(
                if (k & 1 != 0) hi[0] else lo[0],
                if (k & 2 != 0) hi[1] else lo[1],
                if (k & 4 != 0) hi[2] else lo[2],
            )), mount.pos));
        }
        if (project(camera, corners, &observation)) {
            objects[count] = observation;
            count += 1;
        }
    }
    return objects[0..count];
}

samples: []types.SensorSample,
detections: []types.Detection,
observation: types.Observation = .{},

pub fn init(allocator: std.mem.Allocator, scene: *const Scene) !State {
    const samples = try allocator.alloc(types.SensorSample, scene.sensors.len);
    errdefer allocator.free(samples);
    @memset(samples, .{});
    return .{
        .samples = samples,
        .detections = try allocator.alloc(types.Detection, scene.sensors.len * scene.objects.len),
    };
}

pub fn deinit(self: *State, allocator: std.mem.Allocator) void {
    allocator.free(self.detections);
    allocator.free(self.samples);
    self.* = undefined;
}

fn assemble(self: *State, scene: *const Scene) void {
    @setFloatMode(.strict);
    var out: types.Observation = .{};
    const tracking_index = if (scene.routing.get(.pose)) |ref| @backingInt(ref) else null;
    const tracking = if (tracking_index) |index| self.samples[index].value else null;
    if (tracking) |value| {
        const mount = scene.sensors[tracking_index.?].pose;
        out.body.quat = S.multiply(value.pose.pose.quat, S.conjugate(mount.quat));
        out.attitude_axes = value.pose.axes.attitude_axes;
    }
    if (scene.routing.get(.imu)) |ref| if (self.samples[@backingInt(ref)].value) |value| {
        const mount = scene.sensors[@backingInt(ref)].pose;
        out.imu = S.multiply(value.imu.orientation, S.conjugate(mount.quat));
        out.gyro = S.rotate(mount.quat, value.imu.gyro);
        out.accel = S.rotate(mount.quat, value.imu.accel);
        if (out.attitude_axes != 7) {
            out.body.quat = out.imu;
            out.attitude_axes = 7;
        }
    };
    if (tracking) |value| {
        const mount = scene.sensors[tracking_index.?].pose;
        if (out.attitude_axes == 7 or S.norm(mount.pos) == 0) {
            out.body.pos = S.sub(value.pose.pose.pos, S.rotate(out.body.quat, mount.pos));
            out.position_axes = value.pose.axes.position_axes;
        }
    }
    if (scene.routing.get(.pressure)) |ref| if (self.samples[@backingInt(ref)].value) |value| {
        const mount = scene.sensors[@backingInt(ref)].pose;
        out.depth = value.pressure;
        out.depth_ok = true;
        if (out.position_axes & 4 == 0 and (out.attitude_axes == 7 or S.norm(mount.pos) == 0)) {
            out.body.pos[2] = @floatCast(out.depth - S.rotate(out.body.quat, mount.pos)[2]);
            out.position_axes |= 4;
        }
    };
    if (scene.routing.get(.camera)) |ref| {
        const index = @backingInt(ref);
        const camera = scene.sensors[index].settings.camera;
        out.image_width = camera.width;
        out.image_height = camera.height;
        if (self.samples[index].value) |value| {
            out.camera = value.camera.pose;
            out.objects = value.camera.objects;
        }
    }
    self.observation = out;
}

pub fn sample(runtime: *Runtime) void {
    @setFloatMode(.strict);
    const self = &runtime.sensors;
    const scene = runtime.scene;
    const bodies = runtime.backend.states;
    const previous_linear = runtime.previous_linear;
    const previous_angular = runtime.previous_angular;
    const tick = runtime.tick;
    const dt = scene.settings.timestep();
    for (scene.sensors, self.samples, 0..) |sensor, *reading, index| {
        if (tick != 0 and tick % sensor.period_ticks != 0) continue;
        reading.* = .{ .tick = tick };
        if (!sensor.enabled) continue;

        const body_index = @backingInt(sensor.body);
        const body = bodies[body_index];
        const mount = S.compose(body.pose, sensor.pose);
        reading.value = switch (sensor.settings) {
            .pose => |axes| .{ .pose = .{ .pose = mount, .axes = axes } },
            .imu => blk: {
                const arm = S.sub(mount.pos, body.center_of_mass);
                const alpha = S.scale(S.sub(body.angular_velocity, previous_angular[body_index]), 1 / dt);
                const acceleration = S.add(S.add(S.scale(S.sub(body.linear_velocity, previous_linear[body_index]), 1 / dt), S.cross(alpha, arm)), S.cross(body.angular_velocity, S.cross(body.angular_velocity, arm)));
                break :blk .{ .imu = .{
                    .orientation = mount.quat,
                    .gyro = S.rotate(S.conjugate(mount.quat), body.angular_velocity),
                    .accel = S.rotate(S.conjugate(mount.quat), S.sub(acceleration, scene.settings.gravity)),
                } };
            },
            .pressure => .{ .pressure = mount.pos[2] },
            .camera => |camera| .{ .camera = .{
                .pose = mount,
                .objects = cameraSample(scene, sensor, camera, bodies, self.detections[index * bodies.len ..][0..bodies.len], mount),
            } },
        };
        const sensor_index: types.SensorIndex = @fromBackingInt(@intCast(index));
        for (scene.faults) |fault| {
            if (!fault.active(tick)) continue;
            switch (fault.effect) {
                .disable_sensor => |target| if (target == sensor_index) {
                    reading.value = null;
                    break;
                },
                .gyro_bias => |gyro| if (gyro.sensor == sensor_index) {
                    reading.value.?.imu.gyro[0] += @floatCast(gyro.bias);
                },
                .depth_bias => |depth| if (depth.sensor == sensor_index) {
                    reading.value.?.pressure += depth.bias;
                },
                .dead, .weak => {},
            }
        }
    }
    self.assemble(scene);
}

pub fn hashSamples(runtime: *const Runtime, h: *S.Hash) void {
    const self = &runtime.sensors;
    for (runtime.scene.sensors, self.samples) |sensor, reading| {
        h.scalar(u32, sensor.id);
        h.scalar(u32, @backingInt(sensor.kind()));
        h.scalar(u64, reading.tick);
        const value = reading.value orelse {
            for (0..3) |_| h.scalar(u32, 0);
            continue;
        };
        h.scalar(u32, @as(u32, 1) << @intCast(@backingInt(sensor.kind())));
        h.scalar(u32, if (value == .pose) value.pose.axes.position_axes else 0);
        h.scalar(u32, if (value == .pose) value.pose.axes.attitude_axes else 0);
        switch (value) {
            .pose => |pose| {
                h.vec(pose.pose.pos);
                h.quat(pose.pose.quat);
            },
            .imu => |imu| {
                h.quat(imu.orientation);
                h.vec(imu.gyro);
                h.vec(imu.accel);
            },
            .pressure => |depth| h.scalar(f64, depth),
            .camera => |camera| {
                h.vec(camera.pose.pos);
                h.quat(camera.pose.quat);
                h.scalar(u32, @intCast(camera.objects.len));
                for (camera.objects) |obj| {
                    h.scalar(u32, obj.id);
                    h.scalar(u32, obj.cls);
                    h.vec(obj.pose.pos);
                    h.quat(obj.pose.quat);
                    h.vec(obj.size);
                    h.scalar(u32, obj.x0);
                    h.scalar(u32, obj.y0);
                    h.scalar(u32, obj.x1);
                    h.scalar(u32, obj.y1);
                }
            },
        }
    }
}
