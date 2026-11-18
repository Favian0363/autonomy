#include "../include/auv.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#ifndef TEST_MODE
#define TEST_MODE 0
#endif
static AuvFrameError initialization_error;
static unsigned frames;
static const uint64_t FRAME_TIMEOUT_NS = 100000000ULL;
static uint64_t now_ns(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
}
_Static_assert(sizeof(AuvFrameError) == 260, "AuvFrameError ABI drift");
_Static_assert(offsetof(AuvFrame, frame_error) == 11396, "AuvFrame error ABI drift");
_Static_assert(sizeof(AuvFrame) == 11664, "AuvFrame ABI drift");
static void trace(char event) {
  const char *path = getenv("AUV_TEST_SENSOR_TRACE");
  if (!path) return;
  FILE *out = fopen(path, "a");
  if (!out) return;
  fprintf(out, "%c%d\n", event, TEST_MODE);
  fclose(out);
}
void auv_init(void) {
  initialization_error = (AuvFrameError){0};
  frames = 0;
  trace('I');
#if TEST_MODE == 1
  for (int i = 0; i < 500; ++i) {
    struct timespec delay = {0, 1000000}; nanosleep(&delay, NULL);
  }
#elif TEST_MODE == 2
  initialization_error.code = AUV_FRAME_ERROR_INITIALIZATION;
  snprintf(initialization_error.message, sizeof(initialization_error.message), "test initialization failure");
  return;
#endif
  trace('R');
}
void auv_yield_until_next_frame(AuvFrame *frame) {
  *frame = (AuvFrame){0};
  if (initialization_error.code != AUV_FRAME_ERROR_NONE) {
    frame->frame_error = initialization_error;
    return;
  }
#if TEST_MODE == 0 || TEST_MODE == 1
  trace('W');
  const uint64_t started = now_ns();
  while (now_ns() - started < FRAME_TIMEOUT_NS) {
    const struct timespec delay = {0, 1000000};
    nanosleep(&delay, NULL);
  }
  frame->frame_error.code = AUV_FRAME_ERROR_TIMEOUT;
  snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "test frame timeout");
  return;
#else
  ++frames;
#if TEST_MODE == 3
  if (frames > 2) {
    frame->frame_error.code = 0x1000; // Plugin-defined error code.
    snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "test frame failure");
    return;
  }
#endif
  *frame = (AuvFrame){0};
  frame->camera_pose.quat.buf[3] = 1;
  frame->imu_quat.buf[3] = 1;
  frame->timestamp = frames * 16666667ULL;
  frame->position_ok = (AuvAxisOk){true,true,true};
  frame->attitude_ok = (AuvAxisOk){true,true,true};
#if TEST_MODE == 4
  frame->objects_len = 2;
  frame->objects[0].bbox.pose.pos.buf[1] = 1;
  frame->objects[1].bbox.pose.pos.buf[1] = 2;
  for (unsigned i = 0; i < 2; ++i) {
    frame->objects[i].bbox.pose.quat.buf[3] = 1;
    frame->objects[i].bbox.size = (MathVector3f){{1,1,1}};
    frame->objects[i].id = i;
  }
  frame->objects[0].cls = AUV_COCO_PERSON;
  frame->objects[1].cls = AUV_COCO_BOTTLE;
#endif
  return;
#endif
}
void auv_deinit(void) { trace('D'); initialization_error = (AuvFrameError){0}; }
