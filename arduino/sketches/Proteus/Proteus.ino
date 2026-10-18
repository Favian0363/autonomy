#include <Arduino.h>
#include <Servo.h>
#include <string.h>

#define PULSE_WIDTH_MIN 1400
#define PULSE_WIDTH_MAX 1600
#define PULSE_WIDTH_NEUTRAL 1500
// original delta
// #define PULSE_WIDTH_DELTA 400

// reduced delta
#define PULSE_WIDTH_DELTA 100
#define PULSE_WIDTH_MIN PULSE_WIDTH_NEUTRAL - PULSE_WIDTH_DELTA
#define PULSE_WIDTH_MAX PULSE_WIDTH_NEUTRAL + PULSE_WIDTH_DELTA
#define PINS_COUNT 6

const int PINS[PINS_COUNT] = {7, 6, 3, 2, 4, 5};
Servo servos[PINS_COUNT];

const uint8_t PACKET_MAGIC_0 = 0xA5;
const uint8_t PACKET_MAGIC_1 = 0x5A;
const uint8_t PACKET_VERSION = 2;
const uint8_t READY_PACKET_TYPE = 0x80;
const uint8_t ERROR_PACKET_TYPE = 0xE0;
const unsigned long COMMAND_TIMEOUT_MS = 500;
const size_t FLOAT32_SIZE = 4;
const size_t PACKET_MAX_SIZE = 6 + FLOAT32_SIZE * PINS_COUNT;

enum class ErrorCode : uint8_t {
  InvalidCrc = 1,
  InvalidValue = 2,
  UnsupportedVersion = 3,
  InvalidCount = 4,
  CommandTimeout = 9,
};

uint8_t packetBuffer[PACKET_MAX_SIZE];
size_t packetLength = 0;
size_t expectedPacketLength = 0;
unsigned long lastValidCommandMs = 0;
bool receivedCommand = false;
bool watchdogTripped = false;

uint16_t readU16LE(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

float readF32LE(const uint8_t *data) {
  const uint32_t bits = static_cast<uint32_t>(data[0]) |
                        (static_cast<uint32_t>(data[1]) << 8) |
                        (static_cast<uint32_t>(data[2]) << 16) |
                        (static_cast<uint32_t>(data[3]) << 24);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

uint16_t crc16Ccitt(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

void sendReadyPacket() {
  uint8_t packet[6] = {
      PACKET_MAGIC_0, PACKET_MAGIC_1, PACKET_VERSION, READY_PACKET_TYPE, 0, 0};
  const uint16_t crc = crc16Ccitt(packet + 2, 2);
  packet[4] = static_cast<uint8_t>(crc & 0xFF);
  packet[5] = static_cast<uint8_t>(crc >> 8);
  Serial.write(packet, sizeof(packet));
}

void sendErrorPacket(ErrorCode errorCode, uint8_t detail) {
  uint8_t packet[8] = {PACKET_MAGIC_0, PACKET_MAGIC_1, PACKET_VERSION,
                       ERROR_PACKET_TYPE, static_cast<uint8_t>(errorCode),
                       detail, 0, 0};
  const uint16_t crc = crc16Ccitt(packet + 2, 4);
  packet[6] = static_cast<uint8_t>(crc & 0xFF);
  packet[7] = static_cast<uint8_t>(crc >> 8);
  Serial.write(packet, sizeof(packet));
}

void neutralizeThrusters() {
  for (uint8_t i = 0; i < PINS_COUNT; ++i)
    servos[i].writeMicroseconds(PULSE_WIDTH_NEUTRAL);
}

void applyThrusterPacket() {
  const uint8_t count = packetBuffer[3];
  const uint16_t receivedCrc = readU16LE(packetBuffer + expectedPacketLength - 2);
  const uint16_t calculatedCrc = crc16Ccitt(packetBuffer + 2, expectedPacketLength - 4);
  if (receivedCrc != calculatedCrc) {
    sendErrorPacket(ErrorCode::InvalidCrc, 0);
    return;
  }

  float thrustValues[PINS_COUNT] = {};
  for (uint8_t i = 0; i < count; ++i) {
    const float thrust = readF32LE(packetBuffer + 4 + FLOAT32_SIZE * i);
    if (!isfinite(thrust)) {
      sendErrorPacket(ErrorCode::InvalidValue, i);
      return;
    }
    thrustValues[i] = thrust;
  }

  for (uint8_t i = 0; i < count; ++i) {
    const long pulseWidth = PULSE_WIDTH_NEUTRAL +
                            lroundf(PULSE_WIDTH_DELTA * thrustValues[i]);
    servos[i].writeMicroseconds(constrain(pulseWidth, PULSE_WIDTH_MIN, PULSE_WIDTH_MAX));
  }
  lastValidCommandMs = millis();
  receivedCommand = true;
  watchdogTripped = false;
}

void consumeSerialByte(uint8_t value) {
  if (packetLength == 0) {
    if (value == PACKET_MAGIC_0)
      packetBuffer[packetLength++] = value;
    return;
  }

  if (packetLength == 1) {
    if (value == PACKET_MAGIC_1)
      packetBuffer[packetLength++] = value;
    else if (value != PACKET_MAGIC_0)
      packetLength = 0;
    return;
  }

  packetBuffer[packetLength++] = value;
  if (packetLength == 4) {
    const uint8_t count = packetBuffer[3];
    if (packetBuffer[2] != PACKET_VERSION) {
      sendErrorPacket(ErrorCode::UnsupportedVersion, packetBuffer[2]);
      packetLength = 0;
      expectedPacketLength = 0;
      return;
    }
    if (count > PINS_COUNT) {
      sendErrorPacket(ErrorCode::InvalidCount, count);
      packetLength = 0;
      expectedPacketLength = 0;
      return;
    }
    expectedPacketLength = 6 + FLOAT32_SIZE * count;
  }

  if (expectedPacketLength != 0 && packetLength == expectedPacketLength) {
    applyThrusterPacket();
    packetLength = 0;
    expectedPacketLength = 0;
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < PINS_COUNT; i += 1){
      servos[i].attach(PINS[i]);
      servos[i].writeMicroseconds(PULSE_WIDTH_NEUTRAL);  // This sets the thrusters output force to 0 lbf
  }
  delay(5000);
  sendReadyPacket();
}

void serialEvent() {
  while (Serial.available() > 0)
    consumeSerialByte(static_cast<uint8_t>(Serial.read()));
}

void loop() {
  if (receivedCommand && !watchdogTripped &&
      millis() - lastValidCommandMs >= COMMAND_TIMEOUT_MS) {
    neutralizeThrusters();
    watchdogTripped = true;
    sendErrorPacket(ErrorCode::CommandTimeout, 0);
  }
}
