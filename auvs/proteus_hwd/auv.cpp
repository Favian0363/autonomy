// ZED 2i camera plugin (Stereolabs SDK). camera_pose is the vehicle/body frame
// (no separate camera extrinsic): +Y forward (optical axis), +X starboard, +Z down.
// Init uses RIGHT_HANDED_Z_UP_X_FWD (ROS: X forward, Y left, Z up); points and attitudes
// are converted into the autonomy frame below.
//
// Edit the kZed* constants below. Telemetry udp defaults to 127.0.0.1:5600;
// viewer video stream defaults to 127.0.0.1:5610 (even port, uses the next port too).

#include "../../include/auv.h"
#include <exception>
#include <stdexcept>
static AuvFrameError initialization_error{};
static void sensor_init();
static void sensor_read(AuvFrame *frame);
static void sensor_close();
#include "object_map.hpp"
#include "coordinates.hpp"
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
#include <time.h>
#include <cinttypes>
#include <chrono>
#include <thread>

#ifdef AUV_STANDARD_RTP
#include "rtp_relay.hpp"
#if ZED_SDK_MAJOR_VERSION < 5 || (ZED_SDK_MAJOR_VERSION == 5 && ZED_SDK_MINOR_VERSION < 4)
#error "-Dzed-rtp requires ZED SDK 5.4 or newer for encoded packet access"
#endif
static bool rtp_relay_enabled = false;
#endif

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
static constexpr auto kFrameTimeout = std::chrono::seconds(1);
static constexpr auto kFrameRetryInterval = std::chrono::milliseconds(1);
static constexpr uint32_t kZedFrameGrabError = 0x1000;

static Config load_config();
static bool coco_class(int label, AuvObjectCls *cls);

[[noreturn]] static void fail(const char *message) {
  std::fprintf(stderr, "[zed] %s\n", message);
  throw std::runtime_error(message);
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

static void sensor_init(void) {
  try {
    sensor_close();

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
      object_map_init(nullptr);
    } else {
      std::fputs("[zed] pose-only mode: AUV_ZED_ONNX is not configured\n", stderr);
      detection_enabled = false;
    }

    const char *rtp_dest = std::getenv("AUV_VIDEO_RTP_DEST");
    bool standard_rtp = rtp_dest && *rtp_dest;
#ifndef AUV_STANDARD_RTP
    if (standard_rtp) {
      std::fputs("[rtp] AUV_VIDEO_RTP_DEST requires a build with -Dzed-rtp\n", stderr);
      standard_rtp = false;
    }
#endif
    if (config.stream_port) {
      sl::RuntimeParameters warmup;
      warmup.measure3D_reference_frame = sl::REFERENCE_FRAME::WORLD;
      if (zed.grab(warmup) != sl::ERROR_CODE::SUCCESS)
        std::fputs("[zed] warning: warmup grab before streaming failed\n", stderr);

      sl::StreamingParameters stream;
      // The standard relay uses H264 for broad receiver compatibility.
      stream.codec = standard_rtp ? sl::STREAMING_CODEC::H264 : sl::STREAMING_CODEC::H265;
      stream.port = static_cast<unsigned short>(config.stream_port);
      stream.chunk_size = 1400;
      stream.adaptative_bitrate = true;
      auto stream_err = zed.enableStreaming(stream);
      if (stream_err != sl::ERROR_CODE::SUCCESS && !standard_rtp) {
        zed.disableStreaming();
        stream.codec = sl::STREAMING_CODEC::H264;
        stream_err = zed.enableStreaming(stream);
      }
      if (stream_err != sl::ERROR_CODE::SUCCESS) {
        std::fprintf(stderr, "[zed] warning: streaming on port %d failed: %s\n",
                     config.stream_port, sl::toString(stream_err).c_str());
      } else {
        std::fprintf(stderr, "[zed] streaming on port %d (%s)\n", config.stream_port,
                     stream.codec == sl::STREAMING_CODEC::H264 ? "H264" : "H265");
#ifdef AUV_STANDARD_RTP
        if (standard_rtp) rtp_relay_enabled = rtp_relay_start(rtp_dest);
#endif
      }
    } else if (standard_rtp) {
      std::fputs("[rtp] standard relay requires the ZED stream encoder to be enabled\n", stderr);
    }

    if (config.metrics)
      std::fputs("zed_frame,timestamp_ns,sdk_ns,conversion_ns,retries,objects\n", stderr);
  } catch (const std::exception &error) {
    fail(error.what());
  } catch (...) {
    fail("unexpected initialization exception");
  }
}

static void sensor_read(AuvFrame *frame) {
  if (!frame || !zed_open) {
    if (frame) {
      frame->frame_error.code = AUV_FRAME_ERROR_NOT_INITIALIZED;
      std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "ZED camera is not initialized");
    }
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
  switch (error) {
  case sl::ERROR_CODE::END_OF_SVOFILE_REACHED:
    frame->frame_error.code = AUV_FRAME_ERROR_END_OF_STREAM;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "Recording has ended");
    return;

  case sl::ERROR_CODE::SUCCESS: {
#ifdef AUV_STANDARD_RTP
    if (rtp_relay_enabled) {
      sl::EncodedStreamPacket packet;
      if (zed.retrieveEncodedStreamPacket(packet, sl::ENCODED_STREAM_SOURCE::SENDING) == sl::ERROR_CODE::SUCCESS &&
          packet.codec == sl::STREAMING_CODEC::H264)
        rtp_relay_submit(packet.data, packet.size, packet.is_keyframe);
    }
#endif
    const uint64_t frame_start = config.metrics ? now_ns() : 0;
    tracking = zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    frame->timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    translation = pose.getTranslation();
    orientation = pose.getOrientation();
    zed_world_to_autonomy(static_cast<float>(translation.tx), static_cast<float>(translation.ty),
                          static_cast<float>(translation.tz), frame->camera_pose.pos.buf);
    quat_length = std::sqrt(orientation.ox * orientation.ox + orientation.oy * orientation.oy +
                            orientation.oz * orientation.oz + orientation.ow * orientation.ow);
    zed_quat_to_autonomy(static_cast<float>(orientation.ox / quat_length),
                         static_cast<float>(orientation.oy / quat_length),
                         static_cast<float>(orientation.oz / quat_length),
                         static_cast<float>(orientation.ow / quat_length), frame->camera_pose.quat.buf);

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
        frame->camera_pose.pos.buf[2] = frame->pressure_depth;
        frame->position_ok.z = true;
      }
      if (imu_ok) {
        zed_quat_to_autonomy(static_cast<float>(imu_orientation.ox / imu_quat_length),
                             static_cast<float>(imu_orientation.oy / imu_quat_length),
                             static_cast<float>(imu_orientation.oz / imu_quat_length),
                             static_cast<float>(imu_orientation.ow / imu_quat_length),
                             frame->camera_pose.quat.buf);
        frame->attitude_ok = {true, true, true};
      }
    }

    frame->objects_len = 0;
    frame->objects2d_len = 0;
    MappedDetection mapped_detections[AUV_FRAME_MAX_OBJECTS];
    size_t mapped_detection_count = 0;
    if (detection_enabled)
      customObjectError = zed.retrieveCustomObjects(objects, object_params);
    if (detection_enabled && customObjectError == sl::ERROR_CODE::SUCCESS) {
      for (const auto &object : objects.object_list) {
        AuvObjectCls cls;
        if (!coco_class(object.raw_label, &cls)) {
          if (config.log_classes)
            std::fprintf(stderr, "[zed] label %d is outside the coco classes\n", object.raw_label);
          continue;
        }
        if (!object_map_class_enabled(cls))
          continue;

        if (mapped_detection_count < AUV_FRAME_MAX_OBJECTS &&
            object.bounding_box.size() >= 8) {
          float low[3] = {INFINITY, INFINITY, INFINITY};
          float high[3] = {-INFINITY, -INFINITY, -INFINITY};
          for (const auto &corner : object.bounding_box) {
            float autonomy[3];
            zed_world_to_autonomy(corner.x, corner.y, corner.z, autonomy);
            for (int axis = 0; axis < 3; axis++) {
              low[axis] = std::min(low[axis], autonomy[axis]);
              high[axis] = std::max(high[axis], autonomy[axis]);
            }
          }

          MappedDetection &detection = mapped_detections[mapped_detection_count++];
          detection.cls = cls;
          detection.conf = object.confidence / 100.f;
          detection.has_box2d = false;
          for (int axis = 0; axis < 3; axis++) {
            detection.pos[axis] = (low[axis] + high[axis]) * 0.5f;
            detection.size[axis] = high[axis] - low[axis];
          }
          detection.distance = std::hypot(detection.pos[0], std::hypot(detection.pos[1], detection.pos[2]));

          if (object.bounding_box_2d.size() >= 3) {
            const auto &bbox = object.bounding_box_2d;
            detection.has_box2d = true;
            detection.box2d_top_left[0] = bbox[0].x;
            detection.box2d_top_left[1] = bbox[0].y;
            detection.box2d_bottom_right[0] = bbox[2].x;
            detection.box2d_bottom_right[1] = bbox[2].y;
          }
        }
      }

      object_map_update(mapped_detections, mapped_detection_count);
      object_map_fill_frame(frame);
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
    frame->frame_error.code = kZedFrameGrabError;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "ZED grab: %s", sl::toString(error).c_str());
    return;
  }
}



static void sensor_close(void) {
#ifdef AUV_STANDARD_RTP
  rtp_relay_enabled = false;
  rtp_relay_stop();
#endif

  if (zed_open) {
    zed_open = false;
    try {
      zed.disableRecording();
      zed.disableStreaming();
      zed.close();
    } catch (...) {
      std::fputs("[zed] camera close failed\n", stderr);
      // Cleanup errors are reported without terminating the application.
    }
  }
  detection_enabled = false;
  last_timestamp = 0;
  surface_pressure_hpa = 0.f;
  surface_pressure_set = false;
  objects.object_list.clear();
  object_map_reset();
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

// SDK calls run directly on the mission thread. Control is polled between calls
// by the host. Frame deadlines are measured here; SDK calls can still block.
void auv_init(void) {
  initialization_error = {};
  try {
    sensor_init();
  } catch (const std::exception &e) {
    initialization_error.code = AUV_FRAME_ERROR_INITIALIZATION;
    std::snprintf(initialization_error.message, sizeof(initialization_error.message), "%s", e.what());
  } catch (...) {
    initialization_error.code = AUV_FRAME_ERROR_INITIALIZATION;
    std::snprintf(initialization_error.message, sizeof(initialization_error.message), "unexpected sensor initialization exception");
  }
}
void auv_yield_until_next_frame(AuvFrame *frame) {
  *frame = {};
  if (initialization_error.code != AUV_FRAME_ERROR_NONE) {
    frame->frame_error = initialization_error;
    return;
  }
  try {
    const auto started = std::chrono::steady_clock::now();
    while (true) {
      if (std::chrono::steady_clock::now() - started >= kFrameTimeout) break;
      sensor_read(frame);
      if (frame->frame_error.code != AUV_FRAME_ERROR_NONE) return;
      if (std::chrono::steady_clock::now() - started >= kFrameTimeout) break;
      if (frame->timestamp) return;
      std::this_thread::sleep_for(kFrameRetryInterval);
    }
    frame->frame_error.code = AUV_FRAME_ERROR_TIMEOUT;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message),
                  "ZED frame timed out after %lld ms",
                  static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(kFrameTimeout).count()));
  } catch (const std::exception &e) {
    frame->frame_error.code = AUV_FRAME_ERROR_SENSOR;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "%s", e.what());
  } catch (...) {
    frame->frame_error.code = AUV_FRAME_ERROR_SENSOR;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "unexpected sensor frame exception");
  }
}
void auv_deinit(void) {
  try { sensor_close(); }
  catch (const std::exception &e) { std::fprintf(stderr, "sensor cleanup: %s\n", e.what()); }
  catch (...) { std::fputs("unexpected sensor cleanup exception\n", stderr); }
  initialization_error = {};
}
