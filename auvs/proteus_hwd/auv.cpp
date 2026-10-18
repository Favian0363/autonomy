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
//AUV_ZED_STREAM_PORT even port to stream h264 to the laptop. unset = off
//AUV_ZED_RECORD      svo2 path to record h264 onboard. unset = off
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
  const char *record;
  int stream_port;
  sl::RESOLUTION resolution;
  int fps;
  bool log_classes;
  bool metrics;
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
static float surface_pressure_hpa = 0.f;
static bool surface_pressure_set = false;
static constexpr float kWaterDensity = 997.0f;
static constexpr float kGravity = 9.80665f;

static Config load_config();
static int map_class(const Config &config, int label);

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
    params.depth_mode = sl::DEPTH_MODE::NEURAL;
    params.camera_resolution = config.resolution;
    params.camera_fps = config.fps;
    params.sdk_gpu_id = 0;
    if (*config.svo) params.input.setFromSVOFile(config.svo);

    check(zed.open(params), "open camera");
    zed_open = true;
    if (config.stream_port) {
      sl::StreamingParameters stream;
      stream.codec = sl::STREAMING_CODEC::H264;
      stream.port = static_cast<unsigned short>(config.stream_port);
      check(zed.enableStreaming(stream), "enable streaming");
    }
    if (*config.record) {
      sl::RecordingParameters record;
      record.video_filename = sl::String(config.record);
      record.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
      check(zed.enableRecording(record), "enable recording");
    }
    sl::PositionalTrackingParameters tracking;
    tracking.enable_area_memory = true;
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
      detection_enabled = false;
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
  if (!frame || !zed_open) {
    std::fprintf(stderr, "Error Zed has not been initialized yet and we dont have our first grabbed frame\n");
    return;
  }

  sl::RuntimeParameters runtime;
  runtime.measure3D_reference_frame = sl::REFERENCE_FRAME::WORLD;
  const sl::ERROR_CODE error = zed.grab(runtime);
  sl::POSITIONAL_TRACKING_STATE tracking = sl::POSITIONAL_TRACKING_STATE::SEARCHING;
  sl::ERROR_CODE customObjectError = sl::ERROR_CODE::FAILURE;
  sl::CameraInformation camera_info{};
  sl::Translation translation{};
  sl::Orientation orientation{};
  sl::Orientation imu_orientation{};
  sl::SensorsData sensors_data{};
  double quat_length = 1.0;
  double imu_quat_length = 1.0;
  const float pose_flip[] = {1, -1, -1, 1};

  switch (error) {
  case sl::ERROR_CODE::END_OF_SVOFILE_REACHED:
    std::fprintf(stderr, "Recording has ended\n");
    break;

  case sl::ERROR_CODE::SUCCESS:
    tracking = zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    frame->timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    frame->tracking_ok = (tracking == sl::POSITIONAL_TRACKING_STATE::OK);
    translation = pose.getTranslation();
    orientation = pose.getOrientation();
    frame->camera_pose.pos.buf[0] = pose_flip[0] * translation.tx;
    frame->camera_pose.pos.buf[1] = pose_flip[1] * translation.ty;
    frame->camera_pose.pos.buf[2] = pose_flip[2] * translation.tz;
    quat_length = std::sqrt(orientation.ox * orientation.ox + orientation.oy * orientation.oy +
                            orientation.oz * orientation.oz + orientation.ow * orientation.ow);
    frame->camera_pose.quat.buf[0] =
        static_cast<float>(pose_flip[0] * orientation.ox / quat_length);
    frame->camera_pose.quat.buf[1] =
        static_cast<float>(pose_flip[1] * orientation.oy / quat_length);
    frame->camera_pose.quat.buf[2] =
        static_cast<float>(pose_flip[2] * orientation.oz / quat_length);
    frame->camera_pose.quat.buf[3] =
        static_cast<float>(pose_flip[3] * orientation.ow / quat_length);

    camera_info = zed.getCameraInformation();
    frame->image_width = camera_info.camera_configuration.resolution.width;
    frame->image_height = camera_info.camera_configuration.resolution.height;

    if (zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS) {
      frame->accel.buf[0] = sensors_data.imu.linear_acceleration.x;
      frame->accel.buf[1] = sensors_data.imu.linear_acceleration.y;
      frame->accel.buf[2] = sensors_data.imu.linear_acceleration.z;
      frame->gyro.buf[0] = sensors_data.imu.angular_velocity.x;
      frame->gyro.buf[1] = sensors_data.imu.angular_velocity.y;
      frame->gyro.buf[2] = sensors_data.imu.angular_velocity.z;

      imu_orientation = sensors_data.imu.pose.getOrientation();
      imu_quat_length = std::sqrt(imu_orientation.ox * imu_orientation.ox +
                                  imu_orientation.oy * imu_orientation.oy +
                                  imu_orientation.oz * imu_orientation.oz +
                                  imu_orientation.ow * imu_orientation.ow);
      frame->imu_quat.buf[0] = static_cast<float>(imu_orientation.ox / imu_quat_length);
      frame->imu_quat.buf[1] = static_cast<float>(imu_orientation.oy / imu_quat_length);
      frame->imu_quat.buf[2] = static_cast<float>(imu_orientation.oz / imu_quat_length);
      frame->imu_quat.buf[3] = static_cast<float>(imu_orientation.ow / imu_quat_length);

      frame->pressure_depth_ok = false;
      if (sensors_data.barometer.is_available) {
        const float hpa = sensors_data.barometer.pressure;
        if (!surface_pressure_set) {
          surface_pressure_hpa = hpa;
          surface_pressure_set = true;
        }
        const float delta_pa = (hpa - surface_pressure_hpa) * 100.0f;
        if (delta_pa > 0.0f) {
          frame->pressure_depth = delta_pa / (kWaterDensity * kGravity);
          frame->pressure_depth_ok = true;
        }
      }
    }

    frame->objects_len = 0;
    frame->objects2d_len = 0;
    if (detection_enabled)
      customObjectError = zed.retrieveCustomObjects(objects, object_params);
    if (detection_enabled && customObjectError == sl::ERROR_CODE::SUCCESS) {
      for (const auto &object : objects.object_list) {
        if (frame->objects_len < AUV_FRAME_MAX_OBJECTS) {
          double low[3] = {INFINITY, INFINITY, INFINITY};
          double high[3] = {-INFINITY, -INFINITY, -INFINITY};
          for (const auto &corner : object.bounding_box) {
            const float values[] = {corner.x, corner.y, corner.z};
            for (int axis = 0; axis < 3; axis++) {
              low[axis] = std::min(low[axis], static_cast<double>(values[axis]));
              high[axis] = std::max(high[axis], static_cast<double>(values[axis]));
            }
          }

          const uint8_t i = frame->objects_len;
          frame->objects[i].id = static_cast<uint32_t>(object.id);
          frame->objects[i].cls = static_cast<AuvObjectCls>(object.raw_label);
          frame->objects[i].bbox = {};
          frame->objects[i].bbox.pose.quat.buf[3] = 1;
          const float flip[] = {1, -1, -1};
          for (int axis = 0; axis < 3; axis++) {
            const double size = high[axis] - low[axis];
            frame->objects[i].bbox.pose.pos.buf[axis] =
                static_cast<float>(flip[axis] * (low[axis] + high[axis]) / 2);
            frame->objects[i].bbox.size.buf[axis] = static_cast<float>(size);
          }
          frame->objects_len++;
        }

        if (frame->objects2d_len < AUV_FRAME_MAX_OBJECTS &&
            object.bounding_box_2d.size() >= 3) {
          const auto &bbox = object.bounding_box_2d;
          const uint8_t j = frame->objects2d_len;
          frame->objects2d[j].top_left.x = bbox[0].x;
          frame->objects2d[j].top_left.y = bbox[0].y;
          frame->objects2d[j].bottom_right.x = bbox[2].x;
          frame->objects2d[j].bottom_right.y = bbox[2].y;
          frame->objects2d[j].id = static_cast<uint32_t>(object.id);
          frame->objects2d[j].cls = static_cast<AuvObjectCls>(object.raw_label);
          frame->objects2d_len++;
        }
      }
    }
    last_timestamp = frame->timestamp;
    break;
  default:
    break;
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
      zed.disableRecording();
      zed.disableStreaming();
      zed.close();
    } catch (...) {
      std::fputs("[zed] camera close failed\n", stderr);
      std::exit(EXIT_FAILURE);
    }
  }
  detection_enabled = false;
  last_timestamp = 0;
  surface_pressure_hpa = 0.f;
  surface_pressure_set = false;
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

static bool env_flag(const char *name) {
  const std::string_view value = env_or(name, "0");
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
  config.stream_port = parse_nonnegative(env_or("AUV_ZED_STREAM_PORT", "0"));
  config.record = env_or("AUV_ZED_RECORD");
  if (config.fps == 0) throw std::runtime_error("AUV_ZED_FPS must be positive");
  config.log_classes = env_flag("AUV_LOG_CLS");
  config.metrics = env_flag("AUV_ZED_METRICS");

  const std::string_view resolution = env_or("AUV_ZED_RESOLUTION", "HD720");
  if (resolution == "HD720") config.resolution = sl::RESOLUTION::HD720;
  else if (resolution == "HD1080") config.resolution = sl::RESOLUTION::HD1080;
  else if (resolution == "HD2K") config.resolution = sl::RESOLUTION::HD2K;
  else if (resolution == "VGA") config.resolution = sl::RESOLUTION::VGA;
  else throw std::runtime_error("unknown AUV_ZED_RESOLUTION");

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
