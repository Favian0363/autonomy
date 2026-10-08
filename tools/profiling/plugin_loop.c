// Control program for benchmarks: loads an auv plugin and only asks it for frames, with no
// mission logic in between. Comparing it with the mission runner on the same plugin shows
// whether a timing problem comes from the runner or from the camera stack below it.
//
// build: cc -O2 -Wall -Wextra -o zig-out/bin/plugin_loop tools/profiling/plugin_loop.c -ldl
// usage: plugin_loop <plugin.so> <seconds>s [warmup frames, ignored]
//        (same arguments as Ian's camera_check, so bench.sh can run it with CHECKER=)
#include "../../include/auv.h"
#include <dlfcn.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static volatile sig_atomic_t stop = 0;

static void on_signal(int sig) {
  (void)sig;
  stop = 1;
}

static double now_s(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s <plugin.so> <seconds>s\n", argv[0]);
    return 2;
  }
  const double duration = atof(argv[2]);
  if (duration <= 0) {
    fprintf(stderr, "duration must be positive, e.g. 300s\n");
    return 2;
  }

  void *plugin = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (!plugin) {
    fprintf(stderr, "dlopen: %s\n", dlerror());
    return 1;
  }
  AuvInitFunc *init = (AuvInitFunc *)dlsym(plugin, "auv_init");
  AuvYieldUntilNextFrameFunc *next = (AuvYieldUntilNextFrameFunc *)dlsym(plugin, "auv_yield_until_next_frame");
  AuvDeinitFunc *deinit = (AuvDeinitFunc *)dlsym(plugin, "auv_deinit");
  if (!init || !next || !deinit) {
    fprintf(stderr, "plugin is missing auv_init, auv_yield_until_next_frame or auv_deinit\n");
    return 1;
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  static AuvFrame frame;
  unsigned long frames = 0;
  init();
  const double start = now_s();
  while (!stop && now_s() - start < duration) {
    next(&frame);
    frames++;
  }
  deinit();
  fprintf(stderr, "[plugin_loop] %lu frames in %.1f s\n", frames, now_s() - start);
  dlclose(plugin);
  return 0;
}
