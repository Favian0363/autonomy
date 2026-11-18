#include <Arduino.h>
#include <avr/interrupt.h>
#include <string.h>

/*
 * The Timer1 servo pulse scheduler below is derived from Arduino's Servo
 * library AVR implementation, Copyright (c) 2009 Michael Margolis.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

// Proteus firmware protocol and actuator configuration.
const uint8_t PROTEUS_THRUSTER_COUNT = 6;
const uint8_t PROTOCOL_V2_SYNC_BYTE_0 = 0xA5;
const uint8_t PROTOCOL_V2_SYNC_BYTE_1 = 0x5A;
const uint8_t PROTOCOL_V2_READY_REPLY_TYPE = 0x80;
const uint8_t PROTOCOL_V2_ERROR_REPLY_TYPE = 0xE0;
const unsigned long PROTOCOL_V2_SERIAL_BAUD_RATE = 115200;
const unsigned long PROTOCOL_V2_COMMAND_TIMEOUT_MS = 500;
const size_t PROTOCOL_V2_FLOAT32_SIZE_BYTES = 4;
const size_t PROTOCOL_V2_COMMAND_HEADER_SIZE_BYTES = 3;
const size_t PROTOCOL_V2_COMMAND_CRC_SIZE_BYTES = 2;
const size_t PROTOCOL_V2_MAX_COMMAND_PACKET_SIZE_BYTES =
    PROTOCOL_V2_COMMAND_HEADER_SIZE_BYTES +
    PROTOCOL_V2_FLOAT32_SIZE_BYTES * PROTEUS_THRUSTER_COUNT +
    PROTOCOL_V2_COMMAND_CRC_SIZE_BYTES;
const size_t PROTOCOL_V2_READY_REPLY_SIZE_BYTES = 5;
const size_t PROTOCOL_V2_ERROR_REPLY_SIZE_BYTES = 7;

enum class ProtocolV2ErrorCode : uint8_t {
  InvalidCrc = 1,
  InvalidValue = 2,
  InvalidCount = 4,
  CommandTimeout = 9,
};

const int PROTEUS_THRUSTER_PINS[PROTEUS_THRUSTER_COUNT] = {7, 6, 3, 2, 4, 5};
const float PROTEUS_COMMAND_MIN = -1.0f;
const float PROTEUS_COMMAND_MAX = 1.0f;
const int PROTEUS_PULSE_WIDTH_NEUTRAL_US = 1500;
const int PROTEUS_PULSE_WIDTH_DELTA_US = 100;
const int PROTEUS_PULSE_WIDTH_MIN_US =
    PROTEUS_PULSE_WIDTH_NEUTRAL_US - PROTEUS_PULSE_WIDTH_DELTA_US;
const int PROTEUS_PULSE_WIDTH_MAX_US =
    PROTEUS_PULSE_WIDTH_NEUTRAL_US + PROTEUS_PULSE_WIDTH_DELTA_US;
const uint16_t PROTEUS_SERVO_REFRESH_US = 20000;
const uint8_t PROTEUS_SERVO_TIMER_PRESCALER = 8;
const uint8_t PROTEUS_SERVO_WRITE_TRIM_US = 2;

volatile uint16_t thrusterPulseTicks[PROTEUS_THRUSTER_COUNT];
volatile int8_t activeThrusterPulse = -1;

uint16_t microsecondsToTimerTicks(uint16_t microseconds) {
  return (clockCyclesPerMicrosecond() * microseconds) /
         PROTEUS_SERVO_TIMER_PRESCALER;
}

void writeThrusterMicroseconds(uint8_t thruster, uint16_t microseconds) {
  const uint16_t ticks =
      microsecondsToTimerTicks(microseconds - PROTEUS_SERVO_WRITE_TRIM_US);

  // A 16-bit store is not atomic on the 8-bit ATmega328P. Prevent Timer1's
  // interrupt from seeing one byte from the old value and one from the new.
  const uint8_t savedStatus = SREG;
  cli();
  thrusterPulseTicks[thruster] = ticks;
  SREG = savedStatus;
}

ISR(TIMER1_COMPA_vect) {
  if (activeThrusterPulse < 0)
    TCNT1 = 0;
  else
    digitalWrite(PROTEUS_THRUSTER_PINS[activeThrusterPulse], LOW);

  ++activeThrusterPulse;
  if (activeThrusterPulse < PROTEUS_THRUSTER_COUNT) {
    OCR1A = TCNT1 + thrusterPulseTicks[activeThrusterPulse];
    digitalWrite(PROTEUS_THRUSTER_PINS[activeThrusterPulse], HIGH);
    return;
  }

  const uint16_t refreshTicks =
      microsecondsToTimerTicks(PROTEUS_SERVO_REFRESH_US);
  OCR1A = TCNT1 + 4 < refreshTicks ? refreshTicks : TCNT1 + 4;
  activeThrusterPulse = -1;
}

// Keep Arduino's automatic prototype generator from declaring this function
// before ProtocolV2ErrorCode is defined.
void sendErrorPacket(ProtocolV2ErrorCode errorCode, uint8_t detail);

uint8_t packetBuffer[PROTOCOL_V2_MAX_COMMAND_PACKET_SIZE_BYTES];
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
  uint8_t packet[PROTOCOL_V2_READY_REPLY_SIZE_BYTES] = {
      PROTOCOL_V2_SYNC_BYTE_0, PROTOCOL_V2_SYNC_BYTE_1,
      PROTOCOL_V2_READY_REPLY_TYPE, 0, 0};
  const uint16_t crc = crc16Ccitt(packet + 2, 1);
  packet[PROTOCOL_V2_READY_REPLY_SIZE_BYTES -
         PROTOCOL_V2_COMMAND_CRC_SIZE_BYTES] = static_cast<uint8_t>(crc & 0xFF);
  packet[PROTOCOL_V2_READY_REPLY_SIZE_BYTES - 1] =
      static_cast<uint8_t>(crc >> 8);
  Serial.write(packet, sizeof(packet));
}

void sendErrorPacket(ProtocolV2ErrorCode errorCode, uint8_t detail) {
  uint8_t packet[PROTOCOL_V2_ERROR_REPLY_SIZE_BYTES] = {
      PROTOCOL_V2_SYNC_BYTE_0, PROTOCOL_V2_SYNC_BYTE_1,
      PROTOCOL_V2_ERROR_REPLY_TYPE, static_cast<uint8_t>(errorCode), detail,
      0, 0};
  const uint16_t crc = crc16Ccitt(packet + 2, 3);
  packet[PROTOCOL_V2_ERROR_REPLY_SIZE_BYTES -
         PROTOCOL_V2_COMMAND_CRC_SIZE_BYTES] = static_cast<uint8_t>(crc & 0xFF);
  packet[PROTOCOL_V2_ERROR_REPLY_SIZE_BYTES - 1] =
      static_cast<uint8_t>(crc >> 8);
  Serial.write(packet, sizeof(packet));
}

void neutralizeThrusters() {
  for (uint8_t i = 0; i < PROTEUS_THRUSTER_COUNT; ++i)
    writeThrusterMicroseconds(i, PROTEUS_PULSE_WIDTH_NEUTRAL_US);
}

void applyThrusterPacket() {
  const uint8_t count = packetBuffer[2];
  const uint16_t receivedCrc = readU16LE(packetBuffer + expectedPacketLength - 2);
  const uint16_t calculatedCrc = crc16Ccitt(packetBuffer + 2, expectedPacketLength - 4);
  if (receivedCrc != calculatedCrc) {
    neutralizeThrusters();
    sendErrorPacket(ProtocolV2ErrorCode::InvalidCrc, 0);
    return;
  }

  float thrustValues[PROTEUS_THRUSTER_COUNT] = {};
  for (uint8_t i = 0; i < count; ++i) {
    const float thrust = readF32LE(
        packetBuffer + PROTOCOL_V2_COMMAND_HEADER_SIZE_BYTES +
        PROTOCOL_V2_FLOAT32_SIZE_BYTES * i);
    if (!isfinite(thrust) || thrust < PROTEUS_COMMAND_MIN ||
        thrust > PROTEUS_COMMAND_MAX) {
      neutralizeThrusters();
      sendErrorPacket(ProtocolV2ErrorCode::InvalidValue, i);
      return;
    }
    thrustValues[i] = thrust;
  }

  for (uint8_t i = 0; i < count; ++i) {
    const long pulseWidth = PROTEUS_PULSE_WIDTH_NEUTRAL_US +
                            lroundf(PROTEUS_PULSE_WIDTH_DELTA_US *
                                    thrustValues[i]);
    writeThrusterMicroseconds(i, pulseWidth);
  }
  lastValidCommandMs = millis();
  receivedCommand = true;
  watchdogTripped = false;
}

void consumeSerialByte(uint8_t value) {
  if (packetLength == 0) {
    if (value == PROTOCOL_V2_SYNC_BYTE_0)
      packetBuffer[packetLength++] = value;
    return;
  }

  if (packetLength == 1) {
    if (value == PROTOCOL_V2_SYNC_BYTE_1)
      packetBuffer[packetLength++] = value;
    else if (value != PROTOCOL_V2_SYNC_BYTE_0)
      packetLength = 0;
    return;
  }

  packetBuffer[packetLength++] = value;
  if (packetLength == PROTOCOL_V2_COMMAND_HEADER_SIZE_BYTES) {
    const uint8_t count = packetBuffer[2];
    if (count != PROTEUS_THRUSTER_COUNT) {
      neutralizeThrusters();
      sendErrorPacket(ProtocolV2ErrorCode::InvalidCount, count);
      packetLength = 0;
      expectedPacketLength = 0;
      return;
    }
    expectedPacketLength = PROTOCOL_V2_COMMAND_HEADER_SIZE_BYTES +
                           PROTOCOL_V2_FLOAT32_SIZE_BYTES * count +
                           PROTOCOL_V2_COMMAND_CRC_SIZE_BYTES;
  }

  if (expectedPacketLength != 0 && packetLength == expectedPacketLength) {
    applyThrusterPacket();
    packetLength = 0;
    expectedPacketLength = 0;
  }
}

void setup() {
  Serial.begin(PROTOCOL_V2_SERIAL_BAUD_RATE);

  for (uint8_t i = 0; i < PROTEUS_THRUSTER_COUNT; ++i) {
    pinMode(PROTEUS_THRUSTER_PINS[i], OUTPUT);
    writeThrusterMicroseconds(i, PROTEUS_PULSE_WIDTH_NEUTRAL_US);
  }

  // Timer1 runs at 2 MHz (16 MHz / 8). Its compare interrupt raises each
  // thruster output in turn, keeps it high for its configured pulse width,
  // then waits until the 20 ms servo frame is complete.
  TCCR1A = 0;
  TCCR1B = _BV(CS11);
  TCNT1 = 0;
  OCR1A = 4;
  TIFR1 = _BV(OCF1A);
  TIMSK1 |= _BV(OCIE1A);

  delay(5000);
  sendReadyPacket();
}

void serialEvent() {
  while (Serial.available() > 0)
    consumeSerialByte(static_cast<uint8_t>(Serial.read()));
}

void loop() {
  if (receivedCommand && !watchdogTripped &&
      millis() - lastValidCommandMs >= PROTOCOL_V2_COMMAND_TIMEOUT_MS) {
    neutralizeThrusters();
    // Discard an incomplete stale frame so the next valid command can resume.
    packetLength = 0;
    expectedPacketLength = 0;
    watchdogTripped = true;
    sendErrorPacket(ProtocolV2ErrorCode::CommandTimeout, 0);
  }
}
