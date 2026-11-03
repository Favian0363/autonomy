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
  bool position_ok[3];
  bool attitude_ok[3];
  bool depth_ok;
  float depth;
  int objects;
  Object3d objs[128];
  int boxes_len;
  Box2d boxes[128];
  float thr[8];
  int thr_len;
  char step[64];
  float force_error[3];
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

// Matches TelemetryPacket field order in src/MissionContext.zig. A slice is a u8 count, then its items.
static bool parse_telemetry(const unsigned char *bytes, int size, Telemetry *out) {
  Cursor c{bytes, bytes + size};
  const unsigned char *magic;
  if (!take(&c, 4, &magic) || std::memcmp(magic, "AUV1", 4) != 0) return false;

  const unsigned char *timestamp;
  if (!take(&c, 8, &timestamp)) return false;
  (void)timestamp;
  if (!read_vec3(&c, out->pos)) return false;
  for (int i = 0; i < 4; i++) {
    if (!read_f32(&c, &out->quat[i])) return false;
  }

  unsigned char depth_ok, nobj, nbox, nthr, nstep;
  for (int i = 0; i < 3; i++) {
    unsigned char axis;
    if (!read_u8(&c, &axis)) return false;
    out->position_ok[i] = axis != 0;
  }
  for (int i = 0; i < 3; i++) {
    unsigned char axis;
    if (!read_u8(&c, &axis)) return false;
    out->attitude_ok[i] = axis != 0;
  }
  if (!read_u8(&c, &depth_ok) || !read_f32(&c, &out->depth)) return false;
  if (!read_u8(&c, &nobj)) return false;
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
  return read_vec3(&c, out->force_error);
}

// ZED streaming binds an even port and the next one; keep separate from telemetry udp (5600).
static constexpr int kDefaultTelemetryPort = 5600;
static constexpr int kDefaultStreamPort = 5610;
static constexpr int kDefaultTeleopPort = 8080;

// "none", "off", or "-" skips the camera so the viewer can follow simulation telemetry.
static bool video_disabled(const char *source) {
  return std::strcmp(source, "none") == 0 || std::strcmp(source, "off") == 0 || std::strcmp(source, "-") == 0;
}
static constexpr float kTeleopStep = 0.05f;
static constexpr double kTeleopPeriod = 0.05;

// Stonefish body and world: X right, Y forward, Z down. Raylib is Y up.
static Vector3 to_ray(float x, float y, float z) { return {x, -z, -y}; }

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

static void draw_arrow(const float origin[3], float x, float y, float z, float radius, Color color) {
  const Vector3 from = to_ray(origin[0], origin[1], origin[2]);
  const Vector3 to = to_ray(origin[0] + x, origin[1] + y, origin[2] + z);
  DrawCylinderEx(from, to, radius, radius, 8, color);
  DrawSphere(to, radius * 2.5f, color);
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

struct Quat {
  float x;
  float y;
  float z;
  float w;
};

static Quat quat_mul(Quat a, Quat b) {
  return {
      a.w * b.x + b.w * a.x + (a.y * b.z - a.z * b.y),
      a.w * b.y + b.w * a.y + (a.z * b.x - a.x * b.z),
      a.w * b.z + b.w * a.z + (a.x * b.y - a.y * b.x),
      a.w * b.w - (a.x * b.x + a.y * b.y + a.z * b.z),
  };
}

static Quat axis_angle(float ax, float ay, float az, float angle) {
  const float half = angle * 0.5f;
  const float s = std::sin(half);
  return {ax * s, ay * s, az * s, std::cos(half)};
}

static void normalize_quat(Quat *q) {
  const float n = std::sqrt(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
  if (n > 1e-6f) {
    q->x /= n;
    q->y /= n;
    q->z /= n;
    q->w /= n;
  }
}

static void normalize3(float v[3]) {
  const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n > 1e-6f) {
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
  }
}

// Seven little-endian floats: body translation xyz, world orientation quaternion xyzw.
static void encode_goal(const float goal[7], unsigned char out[28]) {
  for (int i = 0; i < 7; i++) {
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
    std::fprintf(stderr, "teleop command -> 127.0.0.1:%d\n", teleop_port);

  char default_video[64];
  const bool read_video = !(argc > 2 && video_disabled(argv[2]));
  const char *video_source = argc > 2 ? argv[2] : nullptr;
  if (read_video && !video_source) {
    std::snprintf(default_video, sizeof(default_video), "127.0.0.1:%d", kDefaultStreamPort);
    video_source = default_video;
  }

  InitWindow(1280, 1000, "viewer");
  SetTargetFPS(60);
  const int scene_w = 900;
  const int scene_h = read_video ? 470 : 990;
  RenderTexture2D scene = LoadRenderTexture(scene_w, scene_h);
  const Rectangle view3d{0, read_video ? 520.f : 0.f, static_cast<float>(scene_w), static_cast<float>(scene_h)};
  float orbit_yaw = 0.8f;
  float orbit_pitch = 0.55f;
  float orbit_dist = 8.f;

  int video_w = 0, video_h = 0;
  Texture2D video{};
  unsigned char *pixels = nullptr;

  static unsigned char packet[64 * 1024];
  Telemetry tel{};
  float translation[3] = {0.0f, 0.0f, 0.0f};
  Quat orientation{0.f, 0.f, 0.f, 1.f};
  double last_packet = -1;
  double last_frame = -1;
  double last_video_try = -10;
  double last_teleop = -1;

  while (!WindowShouldClose()) {
    if (read_video && !video_w && GetTime() - last_video_try > 2.0) {
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
      // Body translation while held. Camera forward is body +Y. Z is down.
      translation[0] = 0.f;
      translation[1] = 0.f;
      translation[2] = 0.f;
      if (IsKeyDown(KEY_D)) translation[0] += 1.f;
      if (IsKeyDown(KEY_A)) translation[0] -= 1.f;
      if (IsKeyDown(KEY_W)) translation[1] += 1.f;
      if (IsKeyDown(KEY_S)) translation[1] -= 1.f;
      if (IsKeyDown(KEY_R)) translation[2] += 1.f;
      if (IsKeyDown(KEY_F)) translation[2] -= 1.f;
      normalize3(translation);

      // Yaw about world +Z. Pitch about body X. Roll about body +Y (camera forward).
      if (IsKeyDown(KEY_RIGHT)) orientation = quat_mul(axis_angle(0.f, 0.f, 1.f, -kTeleopStep), orientation);
      if (IsKeyDown(KEY_LEFT)) orientation = quat_mul(axis_angle(0.f, 0.f, 1.f, kTeleopStep), orientation);
      if (IsKeyDown(KEY_UP)) orientation = quat_mul(orientation, axis_angle(1.f, 0.f, 0.f, -kTeleopStep));
      if (IsKeyDown(KEY_DOWN)) orientation = quat_mul(orientation, axis_angle(1.f, 0.f, 0.f, kTeleopStep));
      if (IsKeyDown(KEY_I)) orientation = quat_mul(orientation, axis_angle(0.f, 1.f, 0.f, kTeleopStep));
      if (IsKeyDown(KEY_K)) orientation = quat_mul(orientation, axis_angle(0.f, 1.f, 0.f, -kTeleopStep));
      normalize_quat(&orientation);

      const float command[7] = {
          translation[0], translation[1], translation[2], orientation.x, orientation.y, orientation.z, orientation.w,
      };
      unsigned char goal_packet[28];
      encode_goal(command, goal_packet);
      if (udp_send(teleop_fd, goal_packet, sizeof(goal_packet)) != static_cast<int>(sizeof(goal_packet))) {
        static bool logged = false;
        if (!logged) {
          std::fprintf(stderr, "teleop send failed\n");
          logged = true;
        }
      }
    }

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
      const float hull[3] = {0.35f, 0.8f, 0.22f};
      draw_box(tel.pos, tel.quat, hull, SKYBLUE);
      float fx, fy, fz;
      rotate_quat(tel.quat, 0, 0.7f, 0, &fx, &fy, &fz);
      DrawLine3D(sub, to_ray(tel.pos[0] + fx, tel.pos[1] + fy, tel.pos[2] + fz), ORANGE);

      const float cmd[4] = {orientation.x, orientation.y, orientation.z, orientation.w};
      float cx, cy, cz;
      rotate_quat(cmd, 0, 2.f, 0, &cx, &cy, &cz);
      draw_arrow(tel.pos, cx, cy, cz, 0.02f, GREEN);
      float mx, my, mz;
      rotate_quat(cmd, translation[0], translation[1], translation[2], &mx, &my, &mz);
      if (mx * mx + my * my + mz * mz > 1e-6f) draw_arrow(tel.pos, mx * 1.2f, my * 1.2f, mz * 1.2f, 0.045f, MAGENTA);
      float ex, ey, ez;
      rotate_quat(tel.quat, tel.force_error[0], tel.force_error[1], tel.force_error[2], &ex, &ey, &ez);
      if (ex * ex + ey * ey + ez * ez > 1e-6f) draw_arrow(tel.pos, ex, ey, ez, 0.03f, YELLOW);
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
    DrawText("y forward   x right   z down", 8, 28, 16, GRAY);
    DrawText("orange nose   green cmd   pink move   yellow err", 8, 48, 16, GRAY);
    EndTextureMode();

    BeginDrawing();
    ClearBackground(BLACK);

    if (read_video && video_w) {
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
    } else if (read_video) {
      DrawText(TextFormat("video: connecting to %s", video_source), 20, 20, 20, YELLOW);
    }

    const int x = 920;
    if (last_packet < 0 || GetTime() - last_packet > 1.0)
      DrawText(TextFormat("no telemetry on udp %d", port), x, 20, 20, RED);
    else
      DrawText(TextFormat("udp %d", port), x, 20, 20, GREEN);

    const Color zed_color = !read_video ? GRAY : (video_w ? GREEN : ORANGE);
    DrawText(read_video ? TextFormat("zed  %s", video_source) : "zed  off", x, 40, 18, zed_color);
    DrawText(TextFormat("step  %s", tel.step), x, 60, 20, RAYWHITE);
    DrawText(TextFormat("pos   %.2f %.2f %.2f", tel.pos[0], tel.pos[1], tel.pos[2]), x, 90, 20, RAYWHITE);
    DrawText(TextFormat("quat  %.2f %.2f %.2f %.2f", tel.quat[0], tel.quat[1], tel.quat[2], tel.quat[3]), x, 120, 20,
             RAYWHITE);
    DrawText(TextFormat("objs  %d", tel.objects), x, 150, 20, RAYWHITE);
    DrawText(TextFormat("pos xyz  %s %s %s", tel.position_ok[0] ? "ok" : "--", tel.position_ok[1] ? "ok" : "--",
                        tel.position_ok[2] ? "ok" : "--"),
             x, 180, 20, (tel.position_ok[0] && tel.position_ok[1] && tel.position_ok[2]) ? GREEN : ORANGE);
    DrawText(TextFormat("att xyz  %s %s %s", tel.attitude_ok[0] ? "ok" : "--", tel.attitude_ok[1] ? "ok" : "--",
                        tel.attitude_ok[2] ? "ok" : "--"),
             x, 210, 20, (tel.attitude_ok[0] && tel.attitude_ok[1] && tel.attitude_ok[2]) ? GREEN : ORANGE);
    DrawText(tel.depth_ok ? TextFormat("depth %.2f m", tel.depth) : "depth -", x, 240, 20, RAYWHITE);
    DrawText(TextFormat("move  %.2f %.2f %.2f", translation[0], translation[1], translation[2]), x, 270, 20,
             teleop_fd >= 0 ? RAYWHITE : RED);
    DrawText(TextFormat("cmd   %.2f %.2f %.2f %.2f", orientation.x, orientation.y, orientation.z, orientation.w), x, 300,
             20, RAYWHITE);
    DrawText(TextFormat("err   %.2f %.2f %.2f", tel.force_error[0], tel.force_error[1], tel.force_error[2]), x, 330, 20,
             YELLOW);
    DrawText(TextFormat("teleop %d  W/S surge  A/D sway  R/F heave  I/K roll", teleop_port), x, 360, 16, GRAY);
    DrawText("arrows yaw and pitch", x, 378, 16, GRAY);
    DrawTextureRec(scene.texture, {0, 0, static_cast<float>(scene_w), -static_cast<float>(scene_h)},
                   {view3d.x, view3d.y}, WHITE);
    DrawRectangleLinesEx(view3d, 1, GRAY);

    for (int i = 0; i < tel.thr_len; i++) {
      const int y = 410 + i * 30;
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
