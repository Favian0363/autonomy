#include "../../include/auv.h"
#include <time.h>
#include <stdio.h>
#ifndef FRAME_INTERVAL_NS
#define FRAME_INTERVAL_NS (1000000000ULL / 60)
#endif
static const uint64_t FRAME_TIMEOUT_NS = 1000000000ULL;
static uint64_t next_frame;
static uint64_t now_ns(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
}
void auv_init(void) {
  next_frame = now_ns() + FRAME_INTERVAL_NS;
}
void auv_yield_until_next_frame(AuvFrame *frame) {
  *frame = (AuvFrame){0};
  const uint64_t started = now_ns();
  uint64_t now;
  while (1) {
    now = now_ns();
    if (now - started >= FRAME_TIMEOUT_NS) {
      frame->frame_error.code = AUV_FRAME_ERROR_TIMEOUT;
      snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "Sensor frame timed out after 1000 ms");
      return;
    }
    if (now >= next_frame) break;
    const struct timespec delay = {0, 1000000};
    nanosleep(&delay, NULL);
  }
  next_frame = now + FRAME_INTERVAL_NS;
  frame->camera_pose = (MathPose){.pos = {0,0,0}, .quat = {0,0,0,1}};
  frame->imu_quat = frame->camera_pose.quat;
  frame->timestamp = now;
  frame->position_ok = (AuvAxisOk){true,true,true};
  frame->attitude_ok = (AuvAxisOk){true,true,true};
}
void auv_deinit(void) { }
