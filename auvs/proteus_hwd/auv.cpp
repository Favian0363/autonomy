// zed 2i camera plugin. talks to the stereolabs sdk directly
// coordinates are meters, world frame, x forward, y right, z down
//
// Edit the kZed* constants below. Telemetry udp defaults to 127.0.0.1:5600;
// viewer video stream defaults to 127.0.0.1:5610 (even port, uses the next port too).

#include "../../include/auv.h"
#include "arduino.hpp"
#include <cstdint>
#include <sl/Camera.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <inttypes.h>
#include <limits>
#include <stdexcept>
#include <sys/stat.h>
#include <thread>
#include <time.h>
#include <cinttypes>

struct Config {
  const char *onnx;
  const char *svo;
  const char *record;
  int stream_port;
  sl::RESOLUTION resolution;
  int fps;
  bool log_classes;
  bool metrics;
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
static uint64_t frames = 0;
static std::thread arduino_reader_thread;
static std::thread arduino_writer_thread;
static constexpr float kWaterDensity = 997.0f;
static constexpr float kGravity = 9.80665f;
static constexpr uintmax_t kMinFreeDisk = 1ULL << 30;

static constexpr const char *kZedOnnx = "yolov8n.onnx";
static constexpr const char *kZedSvo = ""; // svo replay path instead of live camera
static constexpr const char *kZedRecord = ""; // onboard svo2 recording path; empty = off
static constexpr int kZedFps = 30;
static constexpr int kZedStreamPort = 5610; // 0 = off
static constexpr bool kLogCls = false;
static constexpr bool kZedMetrics = false;
static constexpr sl::RESOLUTION kZedResolution = sl::RESOLUTION::HD720;

static Config load_config();
static bool coco_class(int label, AuvObjectCls *cls);

[[noreturn]] static void fail(const char *message) {
  std::fprintf(stderr, "[zed] %s\n", message);
  auv_deinit();
  std::exit(EXIT_FAILURE);
}

static uint64_t now_ns() {
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    fail("monotonic clock failed");
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + static_cast<uint64_t>(now.tv_nsec);
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
    arduino_init();
    arduino_reader_thread = std::thread(arduino_serial_reader);
    arduino_writer_thread = std::thread(arduino_serial_writer);

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
      const auto stream_err = zed.enableStreaming(stream);
      if (stream_err != sl::ERROR_CODE::SUCCESS) {
        std::fprintf(stderr,
                     "[zed] warning: streaming on port %d failed: %s (is the port free? telemetry uses udp "
                     "5600; zed needs this even port and the next one)\n",
                     config.stream_port, sl::toString(stream_err).c_str());
      }
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

  if (*config.record && ++frames % 30 == 0) {
    std::error_code err;
    const auto disk = std::filesystem::space(config.record, err);
    if (!err && disk.available < kMinFreeDisk && zed.getRecordingStatus().is_recording) {
      std::fputs("[zed] disk almost full, stopped recording\n", stderr);
      zed.disableRecording();
    }
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
  bool imu_ok = false;
  bool depth_this_frame = false;
  const float pose_flip[] = {1, -1, -1, 1};

  switch (error) {
  case sl::ERROR_CODE::END_OF_SVOFILE_REACHED:
    std::fprintf(stderr, "Recording has ended\n");
    break;

  case sl::ERROR_CODE::SUCCESS: {
    const uint64_t frame_start = config.metrics ? now_ns() : 0;
    tracking = zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    frame->timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
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
      imu_ok = std::isfinite(imu_quat_length) && imu_quat_length > 1e-6;

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
          depth_this_frame = true;
        }
      }
    }

    if (tracking == sl::POSITIONAL_TRACKING_STATE::OK) {
      frame->position_ok = {true, true, true};
      frame->attitude_ok = {true, true, true};
    } else {
      frame->position_ok = {false, false, false};
      frame->attitude_ok = {false, false, false};
      if (depth_this_frame) {
        // pose_flip already makes world Z positive down. Pressure depth is positive down.
        frame->camera_pose.pos.buf[2] = frame->pressure_depth;
        frame->position_ok.z = true;
      }
      if (imu_ok) {
        frame->camera_pose.quat.buf[0] =
            static_cast<float>(pose_flip[0] * imu_orientation.ox / imu_quat_length);
        frame->camera_pose.quat.buf[1] =
            static_cast<float>(pose_flip[1] * imu_orientation.oy / imu_quat_length);
        frame->camera_pose.quat.buf[2] =
            static_cast<float>(pose_flip[2] * imu_orientation.oz / imu_quat_length);
        frame->camera_pose.quat.buf[3] =
            static_cast<float>(pose_flip[3] * imu_orientation.ow / imu_quat_length);
        frame->attitude_ok = {true, true, true};
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

          AuvObjectCls cls;
          if (!coco_class(object.raw_label, &cls)) {
            if (config.log_classes)
              std::fprintf(stderr, "[zed] label %d is outside the coco classes\n", object.raw_label);
            continue;
          }

          const uint8_t i = frame->objects_len;
          frame->objects[i].id = static_cast<uint32_t>(object.id);
          frame->objects[i].cls = cls;
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
          AuvObjectCls cls2d;
          if (!coco_class(object.raw_label, &cls2d)) continue;

          const auto &bbox = object.bounding_box_2d;
          const uint8_t j = frame->objects2d_len;
          frame->objects2d[j].top_left.x = bbox[0].x;
          frame->objects2d[j].top_left.y = bbox[0].y;
          frame->objects2d[j].bottom_right.x = bbox[2].x;
          frame->objects2d[j].bottom_right.y = bbox[2].y;
          frame->objects2d[j].id = static_cast<uint32_t>(object.id);
          frame->objects2d[j].cls = cls2d;
          frame->objects2d_len++;
        }
      }
    }
    last_timestamp = frame->timestamp;
    if (config.metrics) {
      const uint64_t frame_end = now_ns();
      std::fprintf(stderr, "zed_frame,%" PRIu64 ",%llu,0,0,%u\n", frame->timestamp,
                   static_cast<unsigned long long>(frame_end - frame_start),
                   frame->objects_len);
    }
    break;
  }
  default:
    break;
  }
}

void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len) {
  enqueue_serial_command_packet(thrustor_values, thrustor_values_len);
}

void auv_deinit(void) {
  request_stop();
  if (arduino_reader_thread.joinable())
    arduino_reader_thread.join();
  if (arduino_writer_thread.joinable())
    arduino_writer_thread.join();
  arduino_deinit();

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

static bool coco_class(int label, AuvObjectCls *cls) {
  if (label < static_cast<int>(AUV_COCO_PERSON) || label >= static_cast<int>(AUV_COCO_COUNT))
    return false;
  *cls = static_cast<AuvObjectCls>(label);
  return true;
}

static void check_file(const char *path) {
  struct stat info{};
  if (stat(path, &info) != 0 || !S_ISREG(info.st_mode) || access(path, R_OK) != 0)
    throw std::runtime_error("model or recording must be a readable regular file");
}

static Config load_config() {
  Config config{};
  config.onnx = kZedOnnx;
  config.svo = kZedSvo;
  config.record = kZedRecord;
  config.fps = kZedFps;
  config.stream_port = kZedStreamPort;
  if (config.fps == 0) throw std::runtime_error("kZedFps must be positive");
  config.log_classes = kLogCls;
  config.metrics = kZedMetrics;
  config.resolution = kZedResolution;

  if (*config.onnx) check_file(config.onnx);
  if (*config.svo) check_file(config.svo);

  return config;
}
