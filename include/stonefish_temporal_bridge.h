#ifndef STONEFISH_TEMPORAL_BRIDGE_H
#define STONEFISH_TEMPORAL_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stonefish-only: host feeds thrusters when there is no serial output.
void stonefish_temporal_bridge_set_thrusters(const float *values, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif // STONEFISH_TEMPORAL_BRIDGE_H