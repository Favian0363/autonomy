// zed 2i camera plugin. talks to the stereolabs sdk directly
// coordinates are meters, world frame, x forward, y right, z down
//
// auv_init takes no arguments, so configuration comes from the environment:
//
//AUV_ZED_ONNX        detector model path. unset = pose only
//AUV_CLS_MAP         label:class pairs, comma separated. required with a
//                       model and classes are 0 cube, 1 rect, 2 gate
//AUV_ZED_SVO         recording to replay instead of the live camera
//AUV_ZED_RESOLUTION  HD720 (default), HD1080, HD2K, VGA
//AUV_ZED_FPS         30
//AUV_ZED_DEPTH       NEURAL (default), NEURAL_LIGHT, NEURAL_PLUS
//AUV_ZED_AREA_MEMORY 1 (default) or 0 to disable tracking area memory
//AUV_LOG_CLS         1 to print model labels and their mapped classes
//AUV_ZED_METRICS     1 to print per frame timing rows on stderr

#include "../../include/auv.h"
#include <sl/Camera.hpp>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <inttypes.h>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <vector>

struct ClassMapping {
  int label;
  AuvObjectCls cls;
};

struct Config {
  const char *onnx;
  const char *svo;
  sl::RESOLUTION resolution;
  sl::DEPTH_MODE depth_mode;
  const char *depth_name;
  int fps;
  bool log_classes;
  bool metrics;
  bool area_memory;
  std::vector<ClassMapping> classes;
};

static sl::Camera zed;
static sl::Pose pose;
static sl::Objects objects;
static sl::CustomObjectDetectionRuntimeParameters object_params;
static Config config;
static bool zed_open = false;
static bool detection_enabled = false;
static uint64_t last_timestamp = 0;

static Config load_config();
static int map_class(const Config &config, int label);
static bool copy_pose(const sl::Pose &source, MathPose &destination);
static bool copy_object(const sl::ObjectData &source, int cls, AuvObject &destination);

[[noreturn]] static void fail(const char *message) {
  std::fprintf(stderr, "[zed] %s\n", message);
  auv_deinit();
  std::exit(EXIT_FAILURE);
}

static uint64_t now_ns() {
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    fail("monotonic clock failed");
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}

static void check(sl::ERROR_CODE error, const char *operation) {
  if (error != sl::ERROR_CODE::SUCCESS) {
    std::fprintf(stderr, "[zed] %s: %s\n", operation, sl::toString(error).c_str());
    fail("SDK operation failed");
  }
}

void auv_init(void) {
  try {
    auv_deinit();
    config = load_config();
    sl::InitParameters params;
    params.coordinate_units = sl::UNIT::METER;
    params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    params.depth_mode = config.depth_mode;
    params.camera_resolution = config.resolution;
    params.camera_fps = config.fps;
    params.sdk_gpu_id = 0;
    if (*config.svo) params.input.setFromSVOFile(config.svo);

    std::fprintf(stderr, "[zed] depth=%s area_memory=%d\n", config.depth_name, config.area_memory ? 1 : 0);
    check(zed.open(params), "open camera");
    zed_open = true;
    sl::PositionalTrackingParameters tracking;
    tracking.enable_area_memory = config.area_memory;
    check(zed.enablePositionalTracking(tracking), "enable positional tracking");

    if (*config.onnx) {
      sl::ObjectDetectionParameters detection;
      detection.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS;
      detection.custom_onnx_file = sl::String(config.onnx);
      detection.custom_onnx_dynamic_input_shape = sl::Resolution(640, 640);
      detection.enable_tracking = true;
      detection.enable_segmentation = false;
      detection.allow_reduced_precision_inference = true;
      detection.max_range = 10.0f;
      std::fputs("[zed] loading detector; first load may build a TensorRT engine\n", stderr);
      check(zed.enableObjectDetection(detection), "enable custom detection");
      detection_enabled = true;
    } else {
      std::fputs("[zed] pose-only mode: AUV_ZED_ONNX is not configured\n", stderr);
    }
    if (config.metrics)
      std::fputs("zed_frame,timestamp_ns,sdk_ns,conversion_ns,retries,objects\n", stderr);
  } catch (const std::exception &error) {
    fail(error.what());
  } catch (...) {
    fail("unexpected initialization exception");
  }
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  try {
    if (!frame || !zed_open) fail("capture requires an initialized camera and frame");
    const uint64_t started = now_ns();
    uint64_t retries = 0;
    sl::RuntimeParameters runtime;
    runtime.measure3D_reference_frame = sl::REFERENCE_FRAME::WORLD;
    AuvFrame next{};

    for (;;) {
      const auto error = zed.grab(runtime);
      if (error == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        fail("recording finished; restart the checker for another run");
      bool usable = false;
      if (error == sl::ERROR_CODE::SUCCESS) {
        const auto tracking = zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
        next.timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
        usable = tracking == sl::POSITIONAL_TRACKING_STATE::OK &&
                  next.timestamp > last_timestamp && copy_pose(pose, next.camera_pose);
      }
      if (usable) break;
      if (now_ns() - started >= 5000000000ULL)
        fail("no usable frame for 5 seconds (grab, tracking, pose, or timestamp)");
      retries++;
      const timespec delay{0, 10000000};
      nanosleep(&delay, nullptr);
    }

    if (detection_enabled)
      check(zed.retrieveCustomObjects(objects, object_params), "retrieve custom objects");
    const uint64_t sdk_done = config.metrics ? now_ns() : 0;
    if (detection_enabled) {
      for (const auto &object : objects.object_list) {
        if (next.objects_len == AUV_FRAME_MAX_OBJECTS) break;
        const int cls = map_class(config, object.raw_label);
        if (config.log_classes)
          std::fprintf(stderr, "[zed] label=%d mission_class=%d confidence=%.1f\n",
                        object.raw_label, cls, object.confidence);
        if (copy_object(object, cls, next.objects[next.objects_len]))
          next.objects_len++;
      }
    }
    *frame = next;
    last_timestamp = next.timestamp;
    if (config.metrics)
      std::fprintf(stderr, "zed_frame,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u\n",
                    next.timestamp, sdk_done - started, now_ns() - sdk_done,
                    retries, static_cast<unsigned>(next.objects_len));
  } catch (const std::exception &error) {
    fail(error.what());
  } catch (...) {
    fail("unexpected capture exception");
  }
}

void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len) {
  // Motor output is not implemented
  (void)thrustor_values;
  (void)thrustor_values_len;
}

void auv_deinit(void) {
  if (zed_open) {
    zed_open = false;
    try {
      zed.close();
    } catch (...) {
      std::fputs("[zed] camera close failed\n", stderr);
      std::exit(EXIT_FAILURE);
    }
  }
  detection_enabled = false;
  last_timestamp = 0;
  objects.object_list.clear();
}

static const char *env_or(const char *name, const char *fallback = "") {
  const char *value = std::getenv(name);
  return value ? value : fallback;
}

static int parse_nonnegative(std::string_view text) {
  int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value < 0)
    throw std::runtime_error("expected a nonnegative integer");
  return value;
}

static bool env_flag(const char *name, const char *fallback = "0") {
  const std::string_view value = env_or(name, fallback);
  if (value != "0" && value != "1")
    throw std::runtime_error("boolean environment settings must be 0 or 1");
  return value == "1";
}

static void check_file(const char *path) {
  struct stat info{};
  if (stat(path, &info) != 0 || !S_ISREG(info.st_mode) || access(path, R_OK) != 0)
    throw std::runtime_error("model or recording must be a readable regular file");
}

static Config load_config() {
  Config config{};
  config.onnx = env_or("AUV_ZED_ONNX");
  config.svo = env_or("AUV_ZED_SVO");
  config.fps = parse_nonnegative(env_or("AUV_ZED_FPS", "30"));
  if (config.fps == 0) throw std::runtime_error("AUV_ZED_FPS must be positive");
  config.log_classes = env_flag("AUV_LOG_CLS");
  config.metrics = env_flag("AUV_ZED_METRICS");

  const std::string_view resolution = env_or("AUV_ZED_RESOLUTION", "HD720");
  if (resolution == "HD720") config.resolution = sl::RESOLUTION::HD720;
  else if (resolution == "HD1080") config.resolution = sl::RESOLUTION::HD1080;
  else if (resolution == "HD2K") config.resolution = sl::RESOLUTION::HD2K;
  else if (resolution == "VGA") config.resolution = sl::RESOLUTION::VGA;
  else throw std::runtime_error("unknown AUV_ZED_RESOLUTION");

  config.depth_name = env_or("AUV_ZED_DEPTH", "NEURAL");
  const std::string_view depth = config.depth_name;
  if (depth == "NEURAL") config.depth_mode = sl::DEPTH_MODE::NEURAL;
  else if (depth == "NEURAL_PLUS") config.depth_mode = sl::DEPTH_MODE::NEURAL_PLUS;
#if defined(ZED_SDK_MAJOR_VERSION) && ZED_SDK_MAJOR_VERSION >= 5
  else if (depth == "NEURAL_LIGHT") config.depth_mode = sl::DEPTH_MODE::NEURAL_LIGHT;
#endif
  else throw std::runtime_error("unknown AUV_ZED_DEPTH");
  config.area_memory = env_flag("AUV_ZED_AREA_MEMORY", "1");

  if (*config.onnx) check_file(config.onnx);
  if (*config.svo) check_file(config.svo);

  std::string_view map = env_or("AUV_CLS_MAP");
  if (*config.onnx && map.empty())
    throw std::runtime_error("AUV_CLS_MAP is required with AUV_ZED_ONNX");
  while (!map.empty()) {
    const auto comma = map.find(',');
    const auto entry = map.substr(0, comma);
    const auto colon = entry.find(':');
    if (colon == std::string_view::npos)
      throw std::runtime_error("AUV_CLS_MAP entries must be label:class");
    const int label = parse_nonnegative(entry.substr(0, colon));
    const int cls = parse_nonnegative(entry.substr(colon + 1));
    // src/Auv.zig: cube = 0, rect = 1, gate = 2
    if (cls > 2) throw std::runtime_error("unknown mission class in AUV_CLS_MAP");
    for (const auto &item : config.classes)
      if (item.label == label) throw std::runtime_error("duplicate label in AUV_CLS_MAP");
    config.classes.push_back({label, static_cast<AuvObjectCls>(cls)});
    if (comma == std::string_view::npos) break;
    map.remove_prefix(comma + 1);
    if (map.empty()) throw std::runtime_error("trailing comma in AUV_CLS_MAP");
  }
  return config;
}

static int map_class(const Config &config, int label) {
  for (const auto &item : config.classes)
    if (item.label == label) return item.cls;
  return -1;
}

static bool copy_pose(const sl::Pose &source, MathPose &destination) {
  const auto t = source.getTranslation();
  const auto q = source.getOrientation();
  const float position[] = {t.tx, t.ty, t.tz};
  const float quaternion[] = {q.ox, q.oy, q.oz, q.ow};
  double length_squared = 0;
  for (float value : position)
    if (!std::isfinite(value)) return false;
  for (float value : quaternion) {
    if (!std::isfinite(value)) return false;
    length_squared += static_cast<double>(value) * value;
  }
  if (length_squared < 1e-12) return false;
  // negating y and z is a 180 degree rotation about x, so still right handed
  const float flip[] = {1, -1, -1, 1};
  for (int i = 0; i < 3; i++) destination.pos.buf[i] = flip[i] * position[i];
  const double length = std::sqrt(length_squared);
  for (int i = 0; i < 4; i++)
    destination.quat.buf[i] = static_cast<float>(flip[i] * quaternion[i] / length);
  return true;
}

static bool copy_object(const sl::ObjectData &source, int cls, AuvObject &destination) {
  if (cls < 0 || source.id < 0 || source.tracking_state != sl::OBJECT_TRACKING_STATE::OK)
    return false;
  if (!std::isfinite(source.position.x) || !std::isfinite(source.position.y) ||
      !std::isfinite(source.position.z) || source.bounding_box.size() != 8)
    return false;

  double low[3] = {INFINITY, INFINITY, INFINITY};
  double high[3] = {-INFINITY, -INFINITY, -INFINITY};
  for (const auto &corner : source.bounding_box) {
    const float values[] = {corner.x, corner.y, corner.z};
    for (int axis = 0; axis < 3; axis++) {
      if (!std::isfinite(values[axis])) return false;
      low[axis] = std::min(low[axis], static_cast<double>(values[axis]));
      high[axis] = std::max(high[axis], static_cast<double>(values[axis]));
    }
  }

  AuvObject result{};
  result.id = static_cast<uint32_t>(source.id);
  result.cls = static_cast<AuvObjectCls>(cls);
  // TODO: calculate AuvBoundingBox from 8 points
  result.bbox.pose.quat.buf[3] = 1;
  // world axis aligned, so the flip moves the center but not the sizes
  const float flip[] = {1, -1, -1};
  for (int axis = 0; axis < 3; axis++) {
    const double size = high[axis] - low[axis];
    if (size <= 0 || size > std::numeric_limits<float>::max()) return false;
    result.bbox.pose.pos.buf[axis] =
        static_cast<float>(flip[axis] * (low[axis] + high[axis]) / 2);
    result.bbox.size.buf[axis] = static_cast<float>(size);
  }
  destination = result;
  return true;
}
