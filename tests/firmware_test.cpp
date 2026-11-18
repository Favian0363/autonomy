// Run the real sketch with only Arduino I/O, clock, and registers substituted.
#include "../arduino/sketches/Proteus/Proteus.ino"
#include <array>
#include <cassert>
#include <cstdio>
#include <limits>

using Values = std::array<float, 6>;
static const Values moving = {0.25f, -0.25f, 0.5f, -0.5f, 1.f, -1.f};

static std::vector<uint8_t> packet(const Values& values, uint8_t count = 6) {
    std::vector<uint8_t> result = {0xa5, 0x5a, count};
    for (size_t i = 0; i < count; ++i) {
        uint32_t bits = 0;
        const float value = values[i % values.size()];
        memcpy(&bits, &value, sizeof(bits));
        for (int byte = 0; byte < 4; ++byte)
            result.push_back(static_cast<uint8_t>(bits >> (8 * byte)));
    }
    const uint16_t crc = crc16Ccitt(result.data() + 2, result.size() - 2);
    result.push_back(static_cast<uint8_t>(crc));
    result.push_back(static_cast<uint8_t>(crc >> 8));
    return result;
}

static void feed(const std::vector<uint8_t>& bytes) {
    for (const auto byte : bytes) {
        Serial.input.push_back(byte);
        serialEvent(); // Exercise fragmentation through the actual serial entry point.
    }
}

static void expectNeutral() {
    for (size_t i = 0; i < moving.size(); ++i)
        assert(thrusterPulseTicks[i] == microsecondsToTimerTicks(1500 - 2));
}

static void expectMoving(const Values& values = moving) {
    for (size_t i = 0; i < values.size(); ++i) {
        const auto pulse = static_cast<uint16_t>(1500 + lroundf(100 * values[i]) - 2);
        assert(thrusterPulseTicks[i] == microsecondsToTimerTicks(pulse));
    }
}

static void expectError(ProtocolV2ErrorCode code, uint8_t detail) {
    assert(Serial.output.size() == 7);
    assert(Serial.output[0] == 0xa5 && Serial.output[1] == 0x5a);
    assert(Serial.output[2] == 0xe0);
    assert(Serial.output[3] == static_cast<uint8_t>(code));
    assert(Serial.output[4] == detail);
    assert(readU16LE(Serial.output.data() + 5) == crc16Ccitt(Serial.output.data() + 2, 3));
    Serial.output.clear();
}

static void resume() {
    feed(packet(moving));
    expectMoving();
    assert(Serial.output.empty());
    assert(lastValidCommandMs == testMillis && !watchdogTripped);
}

int main() {
    setup();
    assert(Serial.baud == 115200);
    assert((Serial.output == std::vector<uint8_t>{0xa5, 0x5a, 0x80, 0x78, 0x70}));
    Serial.output.clear();
    expectNeutral();
    for (const auto pin : PROTEUS_THRUSTER_PINS) assert(testPinModes[pin] == OUTPUT);
    testMillis += 1000;
    loop(); // Startup stays neutral without requiring any host command.
    assert(Serial.output.empty());
    resume();

    for (const uint8_t count : {0, 1, 5, 7, 255}) {
        const auto previous = lastValidCommandMs;
        testMillis += 10;
        // Count rejection occurs immediately at the header, before a payload.
        feed({0xa5, 0x5a, count});
        expectNeutral();
        expectError(ProtocolV2ErrorCode::InvalidCount, count);
        assert(lastValidCommandMs == previous);
        resume();
    }

    auto corrupt = packet(moving);
    corrupt.back() ^= 1;
    const auto previous = lastValidCommandMs;
    testMillis += 10;
    feed(corrupt);
    expectNeutral();
    expectError(ProtocolV2ErrorCode::InvalidCrc, 0);
    assert(lastValidCommandMs == previous);
    resume();

    for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::infinity(),
                               -std::numeric_limits<float>::infinity(), -1.01f, 1.01f}) {
        Values values = moving;
        values[5] = invalid; // Earlier channels must not be applied before rejection.
        const auto timestamp = lastValidCommandMs;
        testMillis += 10;
        feed(packet(values));
        expectNeutral();
        expectError(ProtocolV2ErrorCode::InvalidValue, 5);
        assert(lastValidCommandMs == timestamp);
        resume();
    }

    // A valid packet takes effect only after all its bytes and CRC arrive.
    const auto full = packet(Values{});
    for (size_t i = 0; i + 1 < full.size(); ++i) consumeSerialByte(full[i]);
    expectMoving();
    consumeSerialByte(full.back());
    expectNeutral();
    resume();

    const auto last = testMillis;
    testMillis = last + 499;
    loop();
    expectMoving();
    assert(Serial.output.empty());
    // A rejected packet does not postpone the watchdog deadline.
    feed(corrupt);
    expectNeutral();
    expectError(ProtocolV2ErrorCode::InvalidCrc, 0);
    testMillis = last + 500;
    loop();
    expectNeutral();
    expectError(ProtocolV2ErrorCode::CommandTimeout, 0);
    loop();
    assert(Serial.output.empty()); // Report timeout once until a valid command.
    resume();
    testMillis += 500;
    loop();
    expectNeutral();
    expectError(ProtocolV2ErrorCode::CommandTimeout, 0);
    resume();

    // An incomplete command cannot survive a watchdog timeout and corrupt recovery.
    const auto partial = packet(moving);
    feed(std::vector<uint8_t>(partial.begin(), partial.begin() + 10));
    testMillis += 500;
    loop();
    expectNeutral();
    expectError(ProtocolV2ErrorCode::CommandTimeout, 0);
    resume();

    // Garbage and overlapping sync bytes do not prevent the next valid frame.
    feed({0, 0xa5, 0xa5});
    const auto command = packet(moving);
    feed(std::vector<uint8_t>(command.begin() + 1, command.end()));
    expectMoving();
    assert(Serial.output.empty());
    std::puts("PASS real firmware parser, validation, neutralization, watchdog and recovery");
}
