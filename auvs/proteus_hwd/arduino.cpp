#include "arduino.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <semaphore.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t kPacketMagic0 = 0xa5;
constexpr uint8_t kPacketMagic1 = 0x5a;
constexpr uint8_t kPacketVersion = 2;
constexpr uint8_t kReadyPacketType = 0x80;
constexpr uint8_t kErrorPacketType = 0xe0;
constexpr size_t kReadyPacketSize = 6;
constexpr size_t kErrorPacketSize = 8;
constexpr size_t kThrusterCount = 6;
constexpr size_t kFloat32Size = 4;
constexpr size_t kCommandPacketMaxSize = 6 + kFloat32Size * kThrusterCount;
constexpr size_t kCommandQueueCapacity = 32;
constexpr auto kReadyTimeout = std::chrono::seconds(10);
constexpr const char *kPort = "/tmp/autonomy-sim/arduino-auv";
constexpr uint32_t kBaudRate = 115200;

static_assert(sizeof(float) == kFloat32Size);
static_assert(std::numeric_limits<float>::is_iec559);

enum class ErrorCode : uint8_t {
  InvalidCrc = 1,
  InvalidValue = 2,
  UnsupportedVersion = 3,
  InvalidCount = 4,
  CommandTimeout = 9,
};

struct CommandPacket {
  std::array<uint8_t, kCommandPacketMaxSize> bytes{};
  size_t size = 0;
};

int arduino_fd = -1;
std::atomic<bool> io_running{false};
std::atomic<bool> ready_received{false};
sem_t command_semaphore;
bool command_semaphore_initialized = false;

// Single-producer/single-consumer queue. enqueue_serial_command_packet() is the
// only producer and arduino_serial_writer() is the only consumer.
std::array<CommandPacket, kCommandQueueCapacity> tx_command_queue;
std::atomic<size_t> tx_command_head{0};
std::atomic<size_t> tx_command_tail{0};

std::vector<uint8_t> rx_packet;
size_t rx_expected_size = 0;

uint16_t crc16_ccitt(const uint8_t *data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

void append_command_crc(CommandPacket &packet) {
  const uint16_t crc =
      crc16_ccitt(packet.bytes.data() + 2, packet.size - 2);
  packet.bytes[packet.size++] = static_cast<uint8_t>(crc & 0xff);
  packet.bytes[packet.size++] = static_cast<uint8_t>(crc >> 8);
}

void consume_rx_byte(uint8_t value) {
  if (rx_packet.empty()) {
    if (value == kPacketMagic0)
      rx_packet.push_back(value);
    return;
  }

  if (rx_packet.size() == 1) {
    if (value == kPacketMagic1) {
      rx_packet.push_back(value);
      return;
    }

    if (value != kPacketMagic0)
      rx_packet.clear();
    return;
  }

  rx_packet.push_back(value);
  if (rx_packet.size() == 4) {
    if (rx_packet[2] != kPacketVersion) {
      std::fputs("[arduino] unsupported packet version\n", stderr);
      rx_packet.clear();
      rx_expected_size = 0;
      return;
    }
    if (rx_packet[3] == kReadyPacketType)
      rx_expected_size = kReadyPacketSize;
    else if (rx_packet[3] == kErrorPacketType)
      rx_expected_size = kErrorPacketSize;
    else {
      std::fputs("[arduino] unknown packet type\n", stderr);
      rx_packet.clear();
      rx_expected_size = 0;
      return;
    }
  }

  if (rx_expected_size == 0 || rx_packet.size() != rx_expected_size)
    return;

  const uint16_t received_crc =
      static_cast<uint16_t>(rx_packet[rx_expected_size - 2]) |
      (static_cast<uint16_t>(rx_packet[rx_expected_size - 1]) << 8);
  const uint16_t calculated_crc =
      crc16_ccitt(rx_packet.data() + 2, rx_expected_size - 4);
  if (received_crc != calculated_crc) {
    std::fputs("[arduino] invalid packet CRC\n", stderr);
  } else if (rx_packet[3] == kReadyPacketType) {
    ready_received.store(true, std::memory_order_release);
  } else {
    switch (static_cast<ErrorCode>(rx_packet[4])) {
    case ErrorCode::InvalidCrc:
      std::fputs("[arduino] rejected command with invalid CRC\n", stderr);
      break;
    case ErrorCode::InvalidValue:
      std::fprintf(stderr, "[arduino] invalid float at index %u\n",
                   rx_packet[5]);
      break;
    case ErrorCode::UnsupportedVersion:
      std::fprintf(stderr, "[arduino] unsupported command version %u\n",
                   rx_packet[5]);
      break;
    case ErrorCode::InvalidCount:
      std::fprintf(stderr, "[arduino] invalid thrust count %u\n", rx_packet[5]);
      break;
    case ErrorCode::CommandTimeout:
      std::fputs("[arduino] command timeout; thrusters neutralized\n", stderr);
      break;
    default:
      std::fprintf(stderr, "[arduino] error %u (detail %u)\n", rx_packet[4],
                   rx_packet[5]);
      break;
    }
  }
  rx_packet.clear();
  rx_expected_size = 0;
}

bool write_tx_command_queue() {
  size_t tail = tx_command_tail.load(std::memory_order_relaxed);
  const size_t head = tx_command_head.load(std::memory_order_acquire);
  while (tail != head) {
    const CommandPacket &packet = tx_command_queue[tail];
    size_t offset = 0;
    while (offset < packet.size) {
      const ssize_t bytes_written =
          write(arduino_fd, packet.bytes.data() + offset, packet.size - offset);
      if (bytes_written > 0) {
        offset += static_cast<size_t>(bytes_written);
        continue;
      }
      if (bytes_written < 0 && errno == EINTR)
        continue;

      if (bytes_written < 0)
        std::fprintf(stderr, "[arduino] write failed: %s\n",
                     std::strerror(errno));
      else
        std::fputs("[arduino] write returned zero bytes\n", stderr);
      return false;
    }

    tail = (tail + 1) % kCommandQueueCapacity;
    tx_command_tail.store(tail, std::memory_order_release);
  }
  return true;
}

void read_serial_data() {
  char chunk[256];
  const ssize_t bytes_read = read(arduino_fd, chunk, sizeof(chunk));
  if (bytes_read > 0) {
    for (ssize_t i = 0; i < bytes_read; ++i)
      consume_rx_byte(static_cast<uint8_t>(chunk[i]));
  } else if (bytes_read < 0 && errno != EINTR) {
    std::fprintf(stderr, "[arduino] read failed: %s\n", std::strerror(errno));
  }
}

void notify_serial_writer() {
  if (!command_semaphore_initialized)
    return;
  if (sem_post(&command_semaphore) < 0)
    std::fprintf(stderr, "[arduino] sem_post failed: %s\n",
                 std::strerror(errno));
}

} // namespace

void arduino_serial_reader() {
  const auto ready_deadline = std::chrono::steady_clock::now() + kReadyTimeout;
  bool ready_logged = false;
  bool timeout_logged = false;

  while (io_running.load(std::memory_order_acquire)) {
    read_serial_data();

    if (ready_received.load(std::memory_order_acquire) && !ready_logged) {
      std::fputs("[arduino] ready\n", stderr);
      ready_logged = true;
    }

    if (!ready_received.load(std::memory_order_acquire) && !timeout_logged &&
        std::chrono::steady_clock::now() >= ready_deadline) {
      std::fputs("[arduino] READY timeout; continuing without acknowledgment\n",
                 stderr);
      timeout_logged = true;
    }
  }

  ready_received.store(false, std::memory_order_release);
  io_running.store(false, std::memory_order_release);
  notify_serial_writer();
}

void arduino_serial_writer() {
  if (!io_running.load(std::memory_order_acquire))
    return;
  while (true) {
    if (sem_wait(&command_semaphore) < 0) {
      std::fprintf(stderr, "[arduino] sem_wait failed: %s\n",
                   std::strerror(errno));
      io_running.store(false, std::memory_order_release);
      return;
    }
    if (!io_running.load(std::memory_order_acquire))
      return;

    if (!write_tx_command_queue()) {
      io_running.store(false, std::memory_order_release);
      return;
    }
  }
}

void arduino_init() {
  arduino_deinit();

  arduino_fd = open(kPort, O_RDWR | O_NOCTTY);
  if (arduino_fd < 0) {
    std::fprintf(stderr, "[arduino] failed to open %s: %s\n", kPort,
                 std::strerror(errno));
    return;
  }
  termios settings{};
  if (tcgetattr(arduino_fd, &settings) < 0) {
    std::fprintf(stderr, "[arduino] tcgetattr failed: %s\n", std::strerror(errno));
    arduino_deinit();
    return;
  }

  // Make sure there is no terminal wrapper around the data. Send data in raw
  // mode using 8N1 framing: 8 data bits (CS8), no parity (PARENB cleared), and
  // one stop bit (CSTOPB cleared). Also disable RTS/CTS hardware flow control.
  // POSIX termios control modes:
  // https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/termios.h.html
  cfmakeraw(&settings);
  settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);
  // Block for incoming data, but wake at least every 100 ms so the reader can
  // observe request_stop() without poll() or cross-thread descriptor closure.
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 1;

  speed_t baud_rate = 0;
  switch (kBaudRate) {
  case 9600:
    baud_rate = B9600;
    break;
  case 19200:
    baud_rate = B19200;
    break;
  case 38400:
    baud_rate = B38400;
    break;
  case 57600:
    baud_rate = B57600;
    break;
  case 115200:
    baud_rate = B115200;
    break;
#ifdef B230400
  case 230400:
    baud_rate = B230400;
    break;
#endif
  default:
    break;
  }

  if (baud_rate == 0) {
    std::fprintf(stderr, "[arduino] unsupported baud rate: %u\n", kBaudRate);
    arduino_deinit();
    return;
  }

  if (cfsetispeed(&settings, baud_rate) < 0 ||
      cfsetospeed(&settings, baud_rate) < 0) {
    std::fprintf(stderr, "[arduino] setting baud rate failed: %s\n",
                 std::strerror(errno));
    arduino_deinit();
    return;
  }

  if (tcsetattr(arduino_fd, TCSANOW, &settings) < 0) {
    std::fprintf(stderr, "[arduino] tcsetattr failed: %s\n",
                 std::strerror(errno));
    arduino_deinit();
    return;
  }
  std::fprintf(stderr, "[arduino] opened %s at %u baud\n", kPort, kBaudRate);

  if (sem_init(&command_semaphore, 0, 0) < 0) {
    std::fprintf(stderr, "[arduino] sem_init failed: %s\n", std::strerror(errno));
    arduino_deinit();
    return;
  }
  command_semaphore_initialized = true;
  io_running.store(true, std::memory_order_release);
}

void request_stop() {
  io_running.store(false, std::memory_order_release);
  notify_serial_writer();
}

void enqueue_serial_command_packet(const float *values, uint8_t values_len) {
  if (!io_running.load(std::memory_order_acquire) ||
      !ready_received.load(std::memory_order_acquire))
    return;

  const uint8_t count =
      static_cast<uint8_t>(std::min<size_t>(values_len, kThrusterCount));

  // Command packet layout (all multi-byte fields are little-endian):
  //   bytes 0..1                 magic: A5 5A
  //   byte  2                    protocol version
  //   byte  3                    number of thrust values (N)
  //   bytes 4..(4 + 4*N - 1)     N IEEE-754 float32 thrust values
  //   bytes (4 + 4*N)..(5 + 4*N) CRC-16 over version, count, and values
  // Total size: 6 + 4*N bytes (30 bytes when N is 6).
  CommandPacket packet;
  packet.bytes[packet.size++] = kPacketMagic0;
  packet.bytes[packet.size++] = kPacketMagic1;
  packet.bytes[packet.size++] = kPacketVersion;
  packet.bytes[packet.size++] = count;

  // Add the IEEE-754 float32 values
  for (size_t i = 0; i < count; ++i) {
    const float value = std::isfinite(values[i]) ? values[i] : 0.0f;
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    packet.bytes[packet.size++] = static_cast<uint8_t>(bits & 0xff);
    packet.bytes[packet.size++] = static_cast<uint8_t>((bits >> 8) & 0xff);
    packet.bytes[packet.size++] = static_cast<uint8_t>((bits >> 16) & 0xff);
    packet.bytes[packet.size++] = static_cast<uint8_t>(bits >> 24);
  }
  //  calculate and add the Checksum field
  append_command_crc(packet);

  const size_t head = tx_command_head.load(std::memory_order_relaxed);
  const size_t next_head = (head + 1) % kCommandQueueCapacity;
  if (next_head == tx_command_tail.load(std::memory_order_acquire)) {
    std::fputs("[arduino] command queue full; dropping command\n", stderr);
    return;
  }

  tx_command_queue[head] = packet;
  tx_command_head.store(next_head, std::memory_order_release);
  notify_serial_writer();
}

void arduino_deinit() {
  request_stop();

  if (arduino_fd >= 0)
    close(arduino_fd);
  arduino_fd = -1;

  if (command_semaphore_initialized) {
    sem_destroy(&command_semaphore);
    command_semaphore_initialized = false;
  }

  tx_command_head.store(0, std::memory_order_relaxed);
  tx_command_tail.store(0, std::memory_order_relaxed);
  rx_packet.clear();
  rx_expected_size = 0;
  ready_received.store(false, std::memory_order_release);
}
