#include <sl/Camera.hpp>
#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

static sl::Camera zed;
static std::thread grabber;
static std::atomic<bool> running;
static std::mutex lock;
static std::vector<unsigned char> latest;
static bool fresh = false;
static bool opened = false;

void video_close();

static void grab_loop() {
  sl::Mat image;
  while (running) {
    const auto err = zed.grab();
    if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) zed.setSVOPosition(0);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    zed.retrieveImage(image, sl::VIEW::LEFT);
    const auto *bgra = image.getPtr<sl::uchar1>();
    const auto width = image.getWidth();
    const auto step = image.getStepBytes();

    std::lock_guard<std::mutex> guard(lock);
    for (size_t y = 0; y < image.getHeight(); y++) {
      for (size_t x = 0; x < width; x++) {
        const auto *from = bgra + y * step + x * 4;
        auto *to = latest.data() + (y * width + x) * 4;
        to[0] = from[2];
        to[1] = from[1];
        to[2] = from[0];
        to[3] = 255;
      }
    }

    fresh = true;
  }
}

bool video_open(const char *source, int *width, int *height) {
  video_close();

  sl::InitParameters init;
  init.depth_mode = sl::DEPTH_MODE::NONE;
  init.svo_real_time_mode = true;
  init.async_grab_camera_recovery = true;
  init.sdk_verbose = 0;

  const char *colon = std::strrchr(source, ':');
  if (colon && !std::strstr(source, ".svo")) {
    init.input.setFromStream(sl::String(std::string(source, colon).c_str()),
                             static_cast<unsigned short>(std::atoi(colon + 1)));
  } else {
    init.input.setFromSVOFile(sl::String(source));
  }

  const auto err = zed.open(init);
  if (err != sl::ERROR_CODE::SUCCESS) {
    std::fprintf(stderr, "can't open video %s: %s\n", source, sl::toString(err).c_str());
    return false;
  }

  const auto size = zed.getCameraInformation().camera_configuration.resolution;
  *width = static_cast<int>(size.width);
  *height = static_cast<int>(size.height);
  latest.resize(size.width * size.height * 4);

  running = true;
  opened = true;
  grabber = std::thread(grab_loop);
  return true;
}

bool video_frame(unsigned char *out) {
  std::lock_guard<std::mutex> guard(lock);
  if (!fresh) return false;

  std::memcpy(out, latest.data(), latest.size());
  fresh = false;
  return true;
}

void video_close() {
  if (!opened) return;

  running = false;
  if (grabber.joinable()) grabber.join();
  zed.close();
  opened = false;
  fresh = false;
  latest.clear();
}
