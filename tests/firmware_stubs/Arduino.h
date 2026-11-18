#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
using std::isfinite;

constexpr int LOW = 0, HIGH = 1, OUTPUT = 1;
constexpr uint8_t CS11 = 1, OCF1A = 1, OCIE1A = 1;
inline uint8_t SREG = 0x80, TCCR1A = 0, TCCR1B = 0, TIFR1 = 0, TIMSK1 = 0;
inline uint16_t TCNT1 = 0, OCR1A = 0;
inline unsigned long testMillis = 0;
inline int testPins[20] = {};
inline int testPinModes[20] = {};
inline unsigned long millis() { return testMillis; }
inline void delay(unsigned long ms) { testMillis += ms; }
inline unsigned long clockCyclesPerMicrosecond() { return 16; }
inline void digitalWrite(int pin, int value) { testPins[pin] = value; }
inline void pinMode(int pin, int mode) { testPinModes[pin] = mode; }
struct TestSerial {
    unsigned long baud = 0;
    std::deque<uint8_t> input;
    std::vector<uint8_t> output;
    void begin(unsigned long rate) { baud = rate; }
    int available() { return static_cast<int>(input.size()); }
    int read() { const auto byte = input.front(); input.pop_front(); return byte; }
    size_t write(const uint8_t* bytes, size_t size) {
        output.insert(output.end(), bytes, bytes + size);
        return size;
    }
};
inline TestSerial Serial;
