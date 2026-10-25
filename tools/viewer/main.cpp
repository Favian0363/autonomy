#include "raylib.h"
#include "rlgl.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

int udp_open(int port);
int udp_read(int fd, char *buf, int size);
int udp_send_open(const char *ip, int port);
int udp_send(int fd, const void *buf, int size);
bool video_open(const char *source, int *width, int *height);
bool video_frame(unsigned char *out);
void video_close();

struct Box2d {
  float x0, y0, x1, y1;
};

struct Object3d {
  float pos[3];
  float quat[4];
  float size[3];
  unsigned char cls;
};

struct Telemetry {
  float pos[3];
  float quat[4];
  bool tracking;
  bool depth_ok;
  float depth;
  int objects;
  Object3d objs[128];
  int boxes_len;
  Box2d boxes[128];
  float thr[8];
  int thr_len;
  char step[64];
};

struct Cursor {
  const unsigned char *p;
  const unsigned char *end;
};

static bool take(Cursor *c, int n, const unsigned char **out) {
  if (c->end - c->p < n) return false;
  *out = c->p;
  c->p += n;
  return true;
}

static bool read_u8(Cursor *c, unsigned char *out) {
  const unsigned char *b;
  if (!take(c, 1, &b)) return false;
  *out = b[0];
  return true;
}

static bool read_u32(Cursor *c, std::uint32_t *out) {
  const unsigned char *b;
  if (!take(c, 4, &b)) return false;
  *out = static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
         (static_cast<std::uint32_t>(b[2]) << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
  return true;
}

static bool read_f32(Cursor *c, float *out) {
  std::uint32_t bits;
  if (!read_u32(c, &bits)) return false;
  std::memcpy(out, &bits, sizeof(*out));
  return true;
}

static bool read_vec3(Cursor *c, float out[3]) {
  return read_f32(c, &out[0]) && read_f32(c, &out[1]) && read_f32(c, &out[2]);
}

// Matches Packet field order in src/Telemetry.zig. A slice is a u8 count, then its items.
static bool parse_telemetry(const unsigned char *bytes, int size, Telemetry *out) {
  Cursor c{bytes, bytes + size};
  const unsigned char *magic;
  unsigned char version;
  if (!take(&c, 4, &magic) || std::memcmp(magic, "AUV1", 4) != 0 || !read_u8(&c, &version) || version != 1)
    return false;

  const unsigned char *timestamp;
  if (!take(&c, 8, &timestamp)) return false;
  (void)timestamp;
  if (!read_vec3(&c, out->pos)) return false;
  for (int i = 0; i < 4; i++) {
    if (!read_f32(&c, &out->quat[i])) return false;
  }

  unsigned char tracking, depth_ok, nobj, nbox, nthr, nstep;
  if (!read_u8(&c, &tracking) || !read_u8(&c, &depth_ok) || !read_f32(&c, &out->depth)) return false;
  if (!read_u8(&c, &nobj)) return false;
  out->tracking = tracking != 0;
  out->depth_ok = depth_ok != 0;
  out->objects = nobj;

  for (int i = 0; i < nobj; i++) {
    float pos[3], quat[4], size[3];
    std::uint32_t id;
    unsigned char cls;
    if (!read_vec3(&c, pos)) return false;
    for (float &q : quat) {
      if (!read_f32(&c, &q)) return false;
    }
    if (!read_vec3(&c, size) || !read_u32(&c, &id) || !read_u8(&c, &cls)) return false;
    if (i < 128) {
      Object3d &obj = out->objs[i];
      std::memcpy(obj.pos, pos, sizeof(pos));
      std::memcpy(obj.quat, quat, sizeof(quat));
      std::memcpy(obj.size, size, sizeof(size));
      obj.cls = cls;
    }
  }
  if (out->objects > 128) out->objects = 128;

  if (!read_u8(&c, &nbox)) return false;
  out->boxes_len = nbox < 128 ? nbox : 128;
  for (int i = 0; i < nbox; i++) {
    std::uint32_t x0, y0, x1, y1, id;
    unsigned char cls;
    if (!read_u32(&c, &x0) || !read_u32(&c, &y0) || !read_u32(&c, &x1) || !read_u32(&c, &y1) ||
        !read_u32(&c, &id) || !read_u8(&c, &cls))
      return false;
    if (i < out->boxes_len)
      out->boxes[i] = {static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(x1), static_cast<float>(y1)};
  }

  if (!read_u8(&c, &nthr)) return false;
  out->thr_len = nthr < 8 ? nthr : 8;
  for (int i = 0; i < nthr; i++) {
    float value;
    if (!read_f32(&c, &value)) return false;
    if (i < out->thr_len) out->thr[i] = value;
  }

  if (!read_u8(&c, &nstep)) return false;
  const unsigned char *step;
  if (!take(&c, nstep, &step)) return false;
  const int step_len = nstep < 63 ? nstep : 63;
  std::memcpy(out->step, step, static_cast<std::size_t>(step_len));
  out->step[step_len] = '\0';
  return true;
}

// ZED streaming binds an even port and the next one; keep separate from telemetry udp (5600).
static constexpr int kDefaultTelemetryPort = 5600;
static constexpr int kDefaultStreamPort = 5610;
static constexpr int kDefaultTeleopPort = 8080;
static constexpr float kTeleopStep = 0.05f;
static constexpr double kTeleopPeriod = 0.05;

// Vehicle frame is X forward, Y right, Z down. Raylib is Y up.
static Vector3 to_ray(float x, float y, float z) { return {y, -z, -x}; }

static void rotate_quat(const float q[4], float x, float y, float z, float *ox, float *oy, float *oz) {
  const float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  const float cx = qy * z - qz * y;
  const float cy = qz * x - qx * z;
  const float cz = qx * y - qy * x;
  const float tx = 2 * cx, ty = 2 * cy, tz = 2 * cz;
  *ox = x + qw * tx + (qy * tz - qz * ty);
  *oy = y + qw * ty + (qz * tx - qx * tz);
  *oz = z + qw * tz + (qx * ty - qy * tx);
}

static void draw_box(const float pos[3], const float quat[4], const float size[3], Color color) {
  Vector3 corners[8];
  int n = 0;
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        float x, y, z;
        rotate_quat(quat, sx * size[0] * 0.5f, sy * size[1] * 0.5f, sz * size[2] * 0.5f, &x, &y, &z);
        corners[n++] = to_ray(pos[0] + x, pos[1] + y, pos[2] + z);
      }
    }
  }
  static const int edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                    {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto &edge : edges) DrawLine3D(corners[edge[0]], corners[edge[1]], color);
}

static Color object_color(unsigned char cls) {
  return ColorFromHSV(static_cast<float>((cls * 47) % 360), 0.75f, 0.95f);
}

// Six little-endian floats: x, y, z, roll, pitch, yaw. Same packet as teleop_sender.
static void encode_goal(const float goal[6], unsigned char out[24]) {
  for (int i = 0; i < 6; i++) {
    std::uint32_t bits;
    std::memcpy(&bits, &goal[i], sizeof(bits));
    out[i * 4 + 0] = static_cast<unsigned char>(bits);
    out[i * 4 + 1] = static_cast<unsigned char>(bits >> 8);
    out[i * 4 + 2] = static_cast<unsigned char>(bits >> 16);
    out[i * 4 + 3] = static_cast<unsigned char>(bits >> 24);
  }
}

int main(int argc, char **argv) {
  const int port = argc > 1 ? std::atoi(argv[1]) : kDefaultTelemetryPort;
  const int teleop_port = argc > 3 ? std::atoi(argv[3]) : kDefaultTeleopPort;
  const int fd = udp_open(port);
  if (fd < 0) {
    std::fprintf(stderr, "can't listen on udp port %d\n", port);
    return 1;
  }

  const int teleop_fd = udp_send_open("127.0.0.1", teleop_port);
  if (teleop_fd < 0)
    std::fprintf(stderr, "can't send teleop to 127.0.0.1:%d\n", teleop_port);
  else
    std::fprintf(stderr, "teleop goal -> 127.0.0.1:%d\n", teleop_port);

  InitWindow(1280, 1000, "viewer");
  SetTargetFPS(60);
  RenderTexture2D scene = LoadRenderTexture(900, 470);
  float orbit_yaw = 0.8f;
  float orbit_pitch = 0.55f;
  float orbit_dist = 8.f;

  char default_video[64];
  const char *video_source = argc > 2 ? argv[2] : nullptr;
  if (!video_source) {
    std::snprintf(default_video, sizeof(default_video), "127.0.0.1:%d", kDefaultStreamPort);
    video_source = default_video;
  }

  int video_w = 0, video_h = 0;
  Texture2D video{};
  unsigned char *pixels = nullptr;

  static unsigned char packet[64 * 1024];
  Telemetry tel{};
  float goal[6] = {0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
  double last_packet = -1;
  double last_frame = -1;
  double last_video_try = -10;
  double last_teleop = -1;

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
    for (;;) {
      const int n = udp_read(fd, reinterpret_cast<char *>(packet), sizeof(packet));
      if (n <= 0) break;
      Telemetry next{};
      if (parse_telemetry(packet, n, &next)) {
        tel = next;
        last_packet = GetTime();
      }
    }

    if (pixels && video_frame(pixels)) {
      UpdateTexture(video, pixels);
      last_frame = GetTime();
    }

    if (teleop_fd >= 0 && GetTime() - last_teleop >= kTeleopPeriod) {
      last_teleop = GetTime();
      // Hold keys in this window. Each tick moves the absolute goal by 0.05:
      // W/S = +X/-X, D/A = +Y/-Y, R/F = +Z/-Z,
      // E/Q = +roll/-roll, Up/Down = +pitch/-pitch, Right/Left = +yaw/-yaw.
      if (IsKeyDown(KEY_W)) goal[0] += kTeleopStep;
      if (IsKeyDown(KEY_S)) goal[0] -= kTeleopStep;
      if (IsKeyDown(KEY_D)) goal[1] += kTeleopStep;
      if (IsKeyDown(KEY_A)) goal[1] -= kTeleopStep;
      if (IsKeyDown(KEY_R)) goal[2] += kTeleopStep;
      if (IsKeyDown(KEY_F)) goal[2] -= kTeleopStep;
      if (IsKeyDown(KEY_E)) goal[3] += kTeleopStep;
      if (IsKeyDown(KEY_Q)) goal[3] -= kTeleopStep;
      if (IsKeyDown(KEY_UP)) goal[4] += kTeleopStep;
      if (IsKeyDown(KEY_DOWN)) goal[4] -= kTeleopStep;
      if (IsKeyDown(KEY_RIGHT)) goal[5] += kTeleopStep;
      if (IsKeyDown(KEY_LEFT)) goal[5] -= kTeleopStep;

      unsigned char goal_packet[24];
      encode_goal(goal, goal_packet);
      if (udp_send(teleop_fd, goal_packet, sizeof(goal_packet)) != static_cast<int>(sizeof(goal_packet))) {
        static bool logged = false;
        if (!logged) {
          std::fprintf(stderr, "teleop send failed\n");
          logged = true;
        }
      }
    }

    const Rectangle view3d{0, 520, 900, 470};
    if (CheckCollisionPointRec(GetMousePosition(), view3d)) {
      if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        const Vector2 delta = GetMouseDelta();
        orbit_yaw -= delta.x * 0.01f;
        orbit_pitch = std::fmin(1.35f, std::fmax(0.15f, orbit_pitch - delta.y * 0.01f));
      }
      orbit_dist = std::fmax(1.5f, orbit_dist - GetMouseWheelMove());
    }

    const Vector3 sub = to_ray(tel.pos[0], tel.pos[1], tel.pos[2]);
    Camera3D cam{};
    cam.position = {sub.x + orbit_dist * std::cos(orbit_pitch) * std::sin(orbit_yaw),
                    sub.y + orbit_dist * std::sin(orbit_pitch),
                    sub.z + orbit_dist * std::cos(orbit_pitch) * std::cos(orbit_yaw)};
    cam.target = sub;
    cam.up = {0, 1, 0};
    cam.fovy = 50;
    cam.projection = CAMERA_PERSPECTIVE;

    BeginTextureMode(scene);
    ClearBackground({12, 18, 28, 255});
    BeginMode3D(cam);
    rlPushMatrix();
    rlTranslatef(std::floor(sub.x), 0, std::floor(sub.z));
    DrawGrid(20, 1);
    rlPopMatrix();
    DrawLine3D(sub, {sub.x, 0, sub.z}, Fade(SKYBLUE, 0.45f));
    {
      const float hull[3] = {0.8f, 0.35f, 0.22f};
      draw_box(tel.pos, tel.quat, hull, SKYBLUE);
      float fx, fy, fz;
      rotate_quat(tel.quat, 0.7f, 0, 0, &fx, &fy, &fz);
      DrawLine3D(sub, to_ray(tel.pos[0] + fx, tel.pos[1] + fy, tel.pos[2] + fz), ORANGE);
    }
    for (int i = 0; i < tel.objects; i++) {
      const Object3d &obj = tel.objs[i];
      float size[3] = {obj.size[0], obj.size[1], obj.size[2]};
      for (float &axis : size)
        if (axis < 0.05f) axis = 0.3f;
      draw_box(obj.pos, obj.quat, size, object_color(obj.cls));
    }
    EndMode3D();
    DrawText("sub blue   objects colored by coco class   right-drag orbit   wheel zoom", 8, 8, 16, LIGHTGRAY);
    DrawText("x forward   y right   z down", 8, 28, 16, GRAY);
    EndTextureMode();

    BeginDrawing();
    ClearBackground(BLACK);

    if (video_w) {
      float scale = 900.0f / video_w;
      if (video_h * scale > 500.f) scale = 500.f / video_h;
      DrawTextureEx(video, {0, 0}, 0, scale, WHITE);

      for (int i = 0; i < tel.boxes_len; i++) {
        const Box2d &box = tel.boxes[i];
        DrawRectangleLinesEx({box.x0 * scale, box.y0 * scale, (box.x1 - box.x0) * scale, (box.y1 - box.y0) * scale},
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
    DrawText(TextFormat("step  %s", tel.step), x, 60, 20, RAYWHITE);
    DrawText(TextFormat("pos   %.2f %.2f %.2f", tel.pos[0], tel.pos[1], tel.pos[2]), x, 90, 20, RAYWHITE);
    DrawText(TextFormat("quat  %.2f %.2f %.2f %.2f", tel.quat[0], tel.quat[1], tel.quat[2], tel.quat[3]), x, 120, 20,
             RAYWHITE);
    DrawText(TextFormat("objs  %d", tel.objects), x, 150, 20, RAYWHITE);
    DrawText(tel.tracking ? "tracking  ok" : "tracking  lost", x, 180, 20, tel.tracking ? GREEN : ORANGE);
    DrawText(tel.depth_ok ? TextFormat("depth %.2f m", tel.depth) : "depth -", x, 210, 20, RAYWHITE);
    DrawText(TextFormat("goal  %.2f %.2f %.2f", goal[0], goal[1], goal[2]), x, 240, 20,
             teleop_fd >= 0 ? RAYWHITE : RED);
    DrawText(TextFormat("rpy   %.2f %.2f %.2f", goal[3], goal[4], goal[5]), x, 270, 20, RAYWHITE);
    DrawText(TextFormat("teleop %d  W/S X  A/D Y  R/F Z", teleop_port), x, 300, 16, GRAY);
    DrawText("Q/E roll   arrows pitch/yaw", x, 318, 16, GRAY);
    DrawTextureRec(scene.texture, {0, 0, 900, -470}, {view3d.x, view3d.y}, WHITE);
    DrawRectangleLinesEx(view3d, 1, GRAY);

    for (int i = 0; i < tel.thr_len; i++) {
      const int y = 350 + i * 30;
      const int w = static_cast<int>(tel.thr[i] * 100);
      DrawText(TextFormat("t%d %+.2f", i, tel.thr[i]), x, y, 20, RAYWHITE);
      DrawRectangle(w < 0 ? x + 240 + w : x + 240, y, w < 0 ? -w : w, 20, SKYBLUE);
      DrawLine(x + 240, y - 2, x + 240, y + 22, GRAY);
    }

    EndDrawing();
  }

  UnloadRenderTexture(scene);
  video_close();
  CloseWindow();
}
