// Bench check before using the IMU for roll and pitch (odometry.attitude_from_imu): loads a plugin,
// reads frames for a while and prints the roll and pitch from the camera pose and from the IMU
// side by side, twice a second. Tilt the robot by hand, roll then pitch, about 20 degrees each
// way: the two should agree within a degree or two, with the same sign. A sign that disagrees, or
// roll and pitch swapped, means the plugin gives the IMU orientation in other axes than the
// camera pose. It never sends thruster values.
//
// usage: imu_check <plugin.so> [seconds, default 30]
#include "../../include/auv.h"
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// the same conversion as quaternionToEuler in src/math.zig, in degrees
static void roll_pitch(MathQuaternionf q, double *roll, double *pitch) {
  const double x = q.buf[0], y = q.buf[1], z = q.buf[2], w = q.buf[3];
  const double n = sqrt(x * x + y * y + z * z + w * w);
  if (n < 0.5) { // no orientation given
    *roll = *pitch = NAN;
    return;
  }
  const double qx = x / n, qy = y / n, qz = z / n, qw = w / n;
  *roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)) * 180 / M_PI;
  const double sinp = 2 * (qw * qy - qz * qx);
  *pitch = (fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp)) * 180 / M_PI;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <plugin.so> [seconds]\n", argv[0]);
    return 2;
  }
  const double seconds = argc > 2 ? atof(argv[2]) : 30;
  void *plugin = dlopen(argv[1], RTLD_NOW);
  if (plugin == NULL) {
    fprintf(stderr, "cannot load %s: %s\n", argv[1], dlerror());
    return 1;
  }
  AuvInitFunc *init = (AuvInitFunc *)dlsym(plugin, "auv_init");
  AuvYieldUntilNextFrameFunc *yield = (AuvYieldUntilNextFrameFunc *)dlsym(plugin, "auv_yield_until_next_frame");
  AuvDeinitFunc *deinit = (AuvDeinitFunc *)dlsym(plugin, "auv_deinit");
  if (!init || !yield || !deinit) {
    fprintf(stderr, "%s is missing auv_init, auv_yield_until_next_frame or auv_deinit\n", argv[1]);
    return 1;
  }

  static AuvFrame frame;
  init();
  yield(&frame);
  const uint64_t start = frame.timestamp;
  uint64_t next_print = start;
  double worst_roll = 0, worst_pitch = 0;
  int have_imu = 0;
  printf("time_s,tracking_ok,camera_roll,imu_roll,camera_pitch,imu_pitch\n");
  while (frame.timestamp - start < (uint64_t)(seconds * 1e9)) {
    yield(&frame);
    if (frame.timestamp < next_print)
      continue;
    next_print = frame.timestamp + 500000000;
    double cam_roll, cam_pitch, imu_roll, imu_pitch;
    roll_pitch(frame.camera_pose.quat, &cam_roll, &cam_pitch);
    roll_pitch(frame.imu_quat, &imu_roll, &imu_pitch);
    printf("%.1f,%d,%.1f,%.1f,%.1f,%.1f\n", (frame.timestamp - start) / 1e9, frame.tracking_ok, cam_roll, imu_roll,
           cam_pitch, imu_pitch);
    fflush(stdout);
    if (!isnan(imu_roll))
      have_imu = 1;
    if (frame.tracking_ok && !isnan(imu_roll)) {
      worst_roll = fmax(worst_roll, fabs(cam_roll - imu_roll));
      worst_pitch = fmax(worst_pitch, fabs(cam_pitch - imu_pitch));
    }
  }
  deinit();
  if (!have_imu)
    printf("the plugin gives no IMU orientation\n");
  else
    printf("largest difference while tracking: roll %.1f deg, pitch %.1f deg\n", worst_roll, worst_pitch);
  return 0;
}
