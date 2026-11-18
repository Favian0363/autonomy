#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
std::thread worker;
std::atomic<bool> running{false};
std::atomic<bool> failed{false};
std::mutex frame_mutex;
std::vector<unsigned char> frame;
int frame_width = 0;
int frame_height = 0;
bool fresh = false;
std::string bind_address;
unsigned int bind_port = 0;

bool parse_source(const char *source) {
  const std::string input(source ? source : "");
  const std::string prefix = "rtp://";
  if (input.compare(0, prefix.size(), prefix) != 0) return false;
  const auto colon = input.rfind(':');
  if (colon == std::string::npos || colon <= prefix.size()) return false;
  const auto address = input.substr(prefix.size(), colon - prefix.size());
  in_addr addr{};
  if (inet_pton(AF_INET, address.c_str(), &addr) != 1) return false;
  const auto port_string = input.substr(colon + 1);
  if (port_string.empty() || port_string.find_first_not_of("0123456789") != std::string::npos) return false;
  try {
    const auto port = std::stoul(port_string);
    if (port == 0 || port > 65535) return false;
    bind_address = address;
    bind_port = static_cast<unsigned int>(port);
  } catch (...) { return false; }
  return true;
}

GstElement *make_pipeline(bool hardware) {
  const std::string decode = hardware
      ? "nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGBA"
      : "avdec_h264 ! videoconvert ! video/x-raw,format=RGBA";
  const std::string desc = "udpsrc address=" + bind_address + " port=" + std::to_string(bind_port) +
      " caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" ! "
      "rtpjitterbuffer latency=80 drop-on-latency=true ! rtph264depay ! "
      "h264parse config-interval=1 ! " + decode +
      " ! appsink name=frames max-buffers=1 drop=true sync=false";
  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(desc.c_str(), &error);
  if (error) {
    g_printerr("[video] GStreamer pipeline: %s\n", error->message);
    g_error_free(error);
    if (pipeline) gst_object_unref(pipeline);
    return nullptr;
  }
  return pipeline;
}

void run() {
  GstElementFactory *factory = gst_element_factory_find("nvv4l2decoder");
  bool hardware = factory != nullptr;
  if (factory) gst_object_unref(factory);
  while (running) {
    GstElement *pipeline = make_pipeline(hardware);
    if (!pipeline && hardware) { hardware = false; continue; }
    if (!pipeline) { failed = true; std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "frames");
    GstBus *bus = gst_element_get_bus(pipeline);
    if (!sink || gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      failed = true;
      if (sink) gst_object_unref(sink);
      gst_object_unref(bus);
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      if (hardware) { hardware = false; continue; }
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    g_printerr("[video] receiving RTP/H264 on %s:%u (%s decoder)\n", bind_address.c_str(), bind_port,
               hardware ? "Jetson hardware" : "software");
    bool pipeline_error = false;
    while (running && !pipeline_error) {
      GstMessage *message = gst_bus_pop_filtered(bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
      if (message) {
        if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
          GError *error = nullptr; gchar *details = nullptr;
          gst_message_parse_error(message, &error, &details);
          g_printerr("[video] %s\n", error ? error->message : "pipeline error");
          if (error) g_error_free(error);
          g_free(details);
        }
        gst_message_unref(message); pipeline_error = true; failed = true; break;
      }
      GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 100 * GST_MSECOND);
      if (!sample) continue;
      GstVideoInfo info{};
      GstVideoFrame mapped{};
      GstCaps *caps = gst_sample_get_caps(sample);
      GstBuffer *buffer = gst_sample_get_buffer(sample);
      if (caps && buffer && gst_video_info_from_caps(&info, caps) &&
          GST_VIDEO_INFO_FORMAT(&info) == GST_VIDEO_FORMAT_RGBA &&
          GST_VIDEO_INFO_WIDTH(&info) >= 2 && GST_VIDEO_INFO_WIDTH(&info) % 2 == 0 &&
          gst_video_frame_map(&mapped, &info, buffer, GST_MAP_READ)) {
        const int width = GST_VIDEO_INFO_WIDTH(&info) / 2; // ZED stream is left/right side-by-side.
        const int height = GST_VIDEO_INFO_HEIGHT(&info);
        const auto *pixels = static_cast<const unsigned char *>(GST_VIDEO_FRAME_PLANE_DATA(&mapped, 0));
        const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&mapped, 0);
        if (width > 0 && height > 0 && width <= 8192 && height <= 8192 && stride >= width * 8) {
          std::lock_guard<std::mutex> guard(frame_mutex);
          frame.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
          for (int y = 0; y < height; ++y)
            std::memcpy(frame.data() + static_cast<size_t>(y) * width * 4,
                        pixels + static_cast<size_t>(y) * stride, static_cast<size_t>(width) * 4);
          frame_width = width; frame_height = height; fresh = true; failed = false;
        }
        gst_video_frame_unmap(&mapped);
      }
      gst_sample_unref(sample);
    }
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(sink); gst_object_unref(bus); gst_object_unref(pipeline);
    if (pipeline_error && hardware) hardware = false;
    if (running) std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
} // namespace

extern "C" const char *video_default_source() { return "rtp://0.0.0.0:5612"; }
extern "C" void video_close();
extern "C" bool video_open(const char *source, int *width, int *height) {
  video_close();
  *width = *height = 0;
  if (!parse_source(source)) { g_printerr("[video] use rtp://<local-ip>:<port>\n"); failed = true; return false; }
  gst_init(nullptr, nullptr);
  failed = false; running = true;
  try {
    worker = std::thread([] {
      try { run(); }
      catch (const std::exception &error) { g_printerr("[video] receiver stopped: %s\n", error.what()); failed = true; }
      catch (...) { g_printerr("[video] receiver stopped unexpectedly\n"); failed = true; }
    });
  } catch (...) { running = false; failed = true; return false; }
  return true;
}
extern "C" bool video_get_size(int *width, int *height) {
  std::lock_guard<std::mutex> guard(frame_mutex);
  *width = frame_width; *height = frame_height;
  return frame_width > 0 && frame_height > 0;
}
extern "C" bool video_frame(unsigned char *out, size_t capacity, int width, int height) {
  std::lock_guard<std::mutex> guard(frame_mutex);
  if (!fresh || width != frame_width || height != frame_height || capacity < frame.size()) return false;
  std::memcpy(out, frame.data(), frame.size()); fresh = false; return true;
}
extern "C" bool video_has_error() { return failed; }
extern "C" void video_close() {
  running = false;
  if (worker.joinable()) worker.join();
  std::lock_guard<std::mutex> guard(frame_mutex);
  frame.clear(); frame_width = frame_height = 0; fresh = false;
}
