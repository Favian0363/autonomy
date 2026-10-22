#include "arduino.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kThrusterCount = 6;
constexpr int kSendIntervalMs = 50;

void print_help() {
  std::puts("Enter six thrust values from -1.0 to 1.0, separated by spaces.");
  std::puts("Commands: neutral, help, quit");
}

bool parse_values(const std::string &line, std::vector<float> &values) {
  std::istringstream input(line);
  std::vector<float> parsed;
  float value = 0.0f;
  while (input >> value)
    parsed.push_back(value);

  if (!input.eof() || parsed.size() != kThrusterCount)
    return false;
  for (const float item : parsed)
    if (!std::isfinite(item) || item < -1.0f || item > 1.0f)
      return false;

  values = std::move(parsed);
  return true;
}

} // namespace

int main() {
  arduino_init();
  std::thread arduino_reader_thread(arduino_serial_reader);
  std::thread arduino_writer_thread(arduino_serial_writer);

  print_help();
  std::vector<float> values(kThrusterCount, 0.0f);
  bool command_active = false;
  bool running = true;

  while (running) {
    if (command_active)
      enqueue_serial_command_packet(values.data(), values.size());

    pollfd stdin_poll{STDIN_FILENO, POLLIN, 0};
    int poll_result;
    do {
      poll_result = ::poll(&stdin_poll, 1, kSendIntervalMs);
    } while (poll_result < 0 && errno == EINTR);

    if (poll_result < 0) {
      std::perror("arduino_sender: polling stdin failed");
      break;
    }
    if (poll_result == 0)
      continue;
    if ((stdin_poll.revents & (POLLIN | POLLHUP)) == 0)
      continue;

    std::string line;
    if (!std::getline(std::cin, line))
      break;
    if (line == "quit" || line == "q") {
      running = false;
    } else if (line == "help") {
      print_help();
    } else if (line == "neutral") {
      values.assign(kThrusterCount, 0.0f);
      command_active = true;
      enqueue_serial_command_packet(values.data(), values.size());
    } else if (parse_values(line, values)) {
      command_active = true;
      enqueue_serial_command_packet(values.data(), values.size());
    } else {
      std::fputs("Expected exactly six finite values between -1.0 and 1.0.\n",
                 stderr);
    }
  }

  values.assign(kThrusterCount, 0.0f);
  enqueue_serial_command_packet(values.data(), values.size());
  usleep(20'000);
  request_stop();
  arduino_reader_thread.join();
  arduino_writer_thread.join();
  arduino_deinit();
  return 0;
}
