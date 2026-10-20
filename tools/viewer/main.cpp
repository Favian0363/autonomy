#include "raylib.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

int udp_open(int port);
int udp_read(int fd, char *buf, int size);
bool video_open(const char *source, int *width, int *height);
bool video_frame(unsigned char *out);
void video_close();

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

// ZED streaming binds an even port and the next one; keep separate from telemetry udp (5600).
static constexpr int kDefaultTelemetryPort = 5600;
static constexpr int kDefaultStreamPort = 5610;

int main(int argc, char **argv) {
  const int port = argc > 1 ? std::atoi(argv[1]) : kDefaultTelemetryPort;
  const int fd = udp_open(port);
  if (fd < 0) {
    std::fprintf(stderr, "can't listen on udp port %d\n", port);
    return 1;
  }

  InitWindow(1280, 720, "viewer");
  SetTargetFPS(60);

  char default_video[64];
  const char *video_source = argc > 2 ? argv[2] : nullptr;
  if (!video_source) {
    std::snprintf(default_video, sizeof(default_video), "127.0.0.1:%d", kDefaultStreamPort);
    video_source = default_video;
  }

  int video_w = 0, video_h = 0;
  Texture2D video{};
  unsigned char *pixels = nullptr;

  static char packet[64 * 1024];
  static char line[64 * 1024];
  double last_packet = -1;
  double last_frame = -1;
  double last_video_try = -10;

  while (!WindowShouldClose()) {
    if (!video_w && GetTime() - last_video_try > 2.0) {
      last_video_try = GetTime();
      int w = 0, h = 0;
      if (video_open(video_source, &w, &h)) {
        video_w = w;
        video_h = h;
        Image blank = GenImageColor(video_w, video_h, BLACK);
        video = LoadTextureFromImage(blank);
        pixels = static_cast<unsigned char *>(blank.data);
        std::fprintf(stderr, "zed stream connected: %s\n", video_source);
      }
    }
    while (udp_read(fd, packet, sizeof(packet))) {
      std::strcpy(line, packet);
      last_packet = GetTime();
    }

    if (pixels && video_frame(pixels)) {
      UpdateTexture(video, pixels);
      last_frame = GetTime();
    }

    float pose[7] = {};
    float thr[8] = {};
    char step[64];
    const bool tracking = std::strstr(line, "\"tracking\":true") != nullptr;
    const char *depth = std::strstr(line, "\"depth\":");
    const bool depth_ok = depth && depth[8] != 'n';
    read_array(line, "\"pose\":[", pose, 7);
    const int thr_len = read_array(line, "\"thr\":[", thr, 8);
    read_string(line, "\"step\":\"", step, sizeof(step));

    int objects = 0;
    for (const char *p = std::strstr(line, "\"size\":"); p; p = std::strstr(p + 1, "\"size\":"))
      objects++;

    BeginDrawing();
    ClearBackground(BLACK);

    if (video_w) {
      const float scale = 900.0f / video_w;
      DrawTextureEx(video, {0, 0}, 0, scale, WHITE);

      for (const char *p = std::strstr(line, "\"box\":["); p; p = std::strstr(p + 1, "\"box\":[")) {
        float box[4];
        if (read_array(p, "\"box\":[", box, 4) < 4) continue;

        DrawRectangleLinesEx({box[0] * scale, box[1] * scale, (box[2] - box[0]) * scale, (box[3] - box[1]) * scale},
                             2, YELLOW);
      }

      if (last_frame < 0 || GetTime() - last_frame > 1.0)
        DrawText(TextFormat("no frames from %s", video_source), 20, 20, 20, RED);
    } else {
      DrawText(TextFormat("video: connecting to %s", video_source), 20, 20, 20, YELLOW);
    }

    const int x = 920;
    if (last_packet < 0 || GetTime() - last_packet > 1.0)
      DrawText(TextFormat("no telemetry on udp %d", port), x, 20, 20, RED);
    else
      DrawText(TextFormat("udp %d", port), x, 20, 20, GREEN);

    DrawText(TextFormat("zed  %s", video_source), x, 40, 18, video_w ? GREEN : ORANGE);
    DrawText(TextFormat("step  %s", step), x, 60, 20, RAYWHITE);
    DrawText(TextFormat("pos   %.2f %.2f %.2f", pose[0], pose[1], pose[2]), x, 90, 20, RAYWHITE);
    DrawText(TextFormat("quat  %.2f %.2f %.2f %.2f", pose[3], pose[4], pose[5], pose[6]), x, 120, 20,
             RAYWHITE);
    DrawText(TextFormat("objs  %d", objects), x, 150, 20, RAYWHITE);
    DrawText(tracking ? "tracking  ok" : "tracking  lost", x, 180, 20, tracking ? GREEN : ORANGE);
    DrawText(depth_ok ? TextFormat("depth %.2f m", std::strtof(depth + 8, nullptr)) : "depth -", x, 210,
             20, RAYWHITE);

    for (int i = 0; i < thr_len; i++) {
      const int y = 260 + i * 30;
      const int w = static_cast<int>(thr[i] * 100);
      DrawText(TextFormat("t%d %+.2f", i, thr[i]), x, y, 20, RAYWHITE);
      DrawRectangle(w < 0 ? x + 240 + w : x + 240, y, w < 0 ? -w : w, 20, SKYBLUE);
      DrawLine(x + 240, y - 2, x + 240, y + 22, GRAY);
    }

    EndDrawing();
  }

  video_close();
  CloseWindow();
}
