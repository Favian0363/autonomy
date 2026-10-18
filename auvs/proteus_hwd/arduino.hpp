#pragma once

#include <cstdint>

void arduino_init();
void arduino_serial_reader();
void arduino_serial_writer();
void request_stop();
void enqueue_serial_command_packet(const float *values, uint8_t values_len);
// Call arduino_deinit() only after both serial workers have returned or their
// threads have been joined.
void arduino_deinit();
