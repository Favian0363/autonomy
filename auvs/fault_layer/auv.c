// Fault layer: an auv plugin that loads another auv plugin (usually a simulator) and passes its
// frames to the mission runner, but makes them look like the real robot's first:
//   - fewer frames per second, like the camera (the simulator makes 360 per second)
//   - every frame arrives late, like on the Jetson (measured: ~65 ms typical, ~115 ms worst)
// The mission runner and the simulator are unchanged; the layer just sits between them.
//
// env: AUV_FAULT_TARGET=<plugin.so>  the plugin to wrap (required)
//      AUV_FAULT_RATE_HZ=<hz>         frames per second the mission gets (default 0: every frame)
//      AUV_FAULT_DELAY_MS=<ms>        how old each frame is when the mission gets it (default 0)
#include "../../include/auv.h"
#include <dlfcn.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

// Frames kept for the delay. At 30 frames per second this covers about 2 seconds.
#define HISTORY 64

static AuvInitFunc *target_init;
static AuvYieldUntilNextFrameFunc *target_yield;
static AuvSetThrustorsInputFunc *target_set_thrustors;
static AuvDeinitFunc *target_deinit;

static uint64_t period_ns; // 0: pass every frame
static uint64_t delay_ns;

// Frames that passed the rate limit, kept in a ring: `next` is the slot written next and
// `count` how many slots hold frames. frame_ago(0) is the newest, frame_ago(1) the one before.
static AuvFrame history[HISTORY];
static int next;
static int count;

static AuvFrame *frame_ago(int k) {
  return &history[(next - 1 - k + HISTORY) % HISTORY];
}

static void fail(const char *message) {
  fprintf(stderr, "[fault] %s\n", message);
  exit(1);
}

// Nonnegative number from the environment, or 0 if unset; exits on anything else.
static double env_number(const char *name) {
  const char *text = getenv(name);
  if (text == NULL || *text == '\0')
    return 0;
  char *end = NULL;
  errno = 0;
  const double value = strtod(text, &end);
  if (*end != '\0' || errno != 0 || !(value >= 0)) {
    fprintf(stderr, "[fault] %s must be a nonnegative number, got '%s'\n", name, text);
    exit(1);
  }
  return value;
}

static void load_target(void) {
  // loaded once: auv_init runs again on every hot reload of the mission, and the target
  // keeps its own state between init and deinit
  if (target_init != NULL)
    return;
  const char *path = getenv("AUV_FAULT_TARGET");
  if (path == NULL || *path == '\0')
    fail("AUV_FAULT_TARGET must name the plugin to wrap, e.g. ./zig-out/lib/libauv_stonefish_sim.so");
  void *target = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (target == NULL) {
    fprintf(stderr, "[fault] cannot load %s: %s\n", path, dlerror());
    exit(1);
  }
  target_init = (AuvInitFunc *)dlsym(target, "auv_init");
  target_yield = (AuvYieldUntilNextFrameFunc *)dlsym(target, "auv_yield_until_next_frame");
  target_set_thrustors = (AuvSetThrustorsInputFunc *)dlsym(target, "auv_set_thrustor_values");
  target_deinit = (AuvDeinitFunc *)dlsym(target, "auv_deinit");
  if (!target_init || !target_yield || !target_set_thrustors || !target_deinit)
    fail("the target plugin is missing one of the four auv_* functions");
}

void auv_init(void) {
  load_target();
  const double rate_hz = env_number("AUV_FAULT_RATE_HZ");
  const double delay_ms = env_number("AUV_FAULT_DELAY_MS");
  period_ns = rate_hz > 0 ? (uint64_t)(1e9 / rate_hz) : 0;
  delay_ns = (uint64_t)(delay_ms * 1e6);
  next = 0;
  count = 0;
  fprintf(stderr, "[fault] target=%s rate_hz=%g delay_ms=%g\n", getenv("AUV_FAULT_TARGET"), rate_hz, delay_ms);
  target_init();
}

// Reads the target's frames into `slot` until one passes the rate limit. The target's frames
// in between are skipped, and the target keeps applying the last thruster values meanwhile.
static void next_rate_limited_frame(AuvFrame *slot) {
  if (count == 0 || period_ns == 0) {
    target_yield(slot);
    return;
  }
  const uint64_t due = frame_ago(0)->timestamp + period_ns;
  do
    target_yield(slot);
  while (slot->timestamp < due);
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  // normally one new frame per call; the first call reads ahead until some frame is delay_ns
  // old, like a camera pipeline that is already running, so the mission never gets a repeat
  do {
    next_rate_limited_frame(&history[next]);
    next = (next + 1) % HISTORY;
    if (count < HISTORY)
      count++;
  } while (count < HISTORY && frame_ago(0)->timestamp - frame_ago(count - 1)->timestamp < delay_ns);

  // hand out the newest frame that is at least delay_ns old
  const uint64_t now = frame_ago(0)->timestamp;
  for (int k = 0; k < count; k++) {
    if (now - frame_ago(k)->timestamp >= delay_ns) {
      *frame = *frame_ago(k);
      return;
    }
  }
  fail("AUV_FAULT_DELAY_MS is longer than the frame history covers; lower it or AUV_FAULT_RATE_HZ");
}

void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len) {
  target_set_thrustors(thrustor_values, thrustor_values_len);
}

void auv_deinit(void) {
  if (target_deinit != NULL)
    target_deinit();
}
