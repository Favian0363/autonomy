#include "raylib.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

int udp_open(int port);
int udp_read(int fd, char *buf, int size);

static int read_array(const char *line, const char *key, float *out, int max) {
  const char *p = std::strstr(line, key);
  if (!p) return 0;

  p += std::strlen(key);
  int n = 0;
  while (n < max && *p != ']') {
    char *end;
    out[n] = std::strtof(p, &end);
    if (end == p) break;

    n++;
    p = *end == ',' ? end + 1 : end;
  }

  return n;
}

static void read_string(const char *line, const char *key, char *out, int size) {
  out[0] = '\0';
  const char *p = std::strstr(line, key);
  if (!p) return;

  p += std::strlen(key);
  int n = 0;
  while (n < size - 1 && p[n] && p[n] != '"') {
    out[n] = p[n];
    n++;
  }

  out[n] = '\0';
}

int main(int argc, char **argv) {
  const int port = argc > 1 ? std::atoi(argv[1]) : 5600;
  const int fd = udp_open(port);
  if (fd < 0) {
    std::fprintf(stderr, "can't listen on udp port %d\n", port);
    return 1;
  }

  InitWindow(960, 540, "viewer");
  SetTargetFPS(60);

  static char packet[64 * 1024];
  static char line[64 * 1024];
  double last_packet = -1;

  while (!WindowShouldClose()) {
    while (udp_read(fd, packet, sizeof(packet))) {
      std::strcpy(line, packet);
      last_packet = GetTime();
    }

    float pose[7] = {};
    float thr[8] = {};
    char step[64];
    read_array(line, "\"pose\":[", pose, 7);
    const int thr_len = read_array(line, "\"thr\":[", thr, 8);
    read_string(line, "\"step\":\"", step, sizeof(step));

    int objects = 0;
    for (const char *p = std::strstr(line, "\"id\":"); p; p = std::strstr(p + 1, "\"id\":"))
      objects++;

    BeginDrawing();
    ClearBackground(BLACK);

    if (last_packet < 0 || GetTime() - last_packet > 1.0)
      DrawText(TextFormat("no telemetry on udp %d", port), 20, 20, 20, RED);
    else
      DrawText(TextFormat("udp %d", port), 20, 20, 20, GREEN);

    DrawText(TextFormat("step  %s", step), 20, 60, 20, RAYWHITE);
    DrawText(TextFormat("pos   %.2f %.2f %.2f", pose[0], pose[1], pose[2]), 20, 90, 20, RAYWHITE);
    DrawText(TextFormat("quat  %.2f %.2f %.2f %.2f", pose[3], pose[4], pose[5], pose[6]), 20, 120, 20,
             RAYWHITE);
    DrawText(TextFormat("objs  %d", objects), 20, 150, 20, RAYWHITE);

    for (int i = 0; i < thr_len; i++) {
      const int y = 200 + i * 30;
      const int w = static_cast<int>(thr[i] * 150);
      DrawText(TextFormat("t%d %+.2f", i, thr[i]), 20, y, 20, RAYWHITE);
      DrawRectangle(w < 0 ? 300 + w : 300, y, w < 0 ? -w : w, 20, SKYBLUE);
      DrawLine(300, y - 2, 300, y + 22, GRAY);
    }

    EndDrawing();
  }

  CloseWindow();
}
