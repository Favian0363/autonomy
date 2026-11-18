#pragma once
#define ISR(vector) void vector(void)
#define _BV(bit) (1U << (bit))
inline void cli() { SREG &= 0x7f; }
