#ifndef AUV_H
#define AUV_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "math.h"
#include "object_classes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t AuvObjectCls;

typedef struct {
  uint32_t x;
  uint32_t y;
} AuvPoint2u;

typedef struct {
  AuvPoint2u top_left;
  AuvPoint2u bottom_right;
  uint32_t id;
  AuvObjectCls cls;
} AuvObject2d;

typedef struct {
  MathBoundingBox bbox;
  uint32_t id;
  AuvObjectCls cls;
} AuvObject;

/// One flag per axis. False means that component of the pose must not be used.
typedef struct {
  bool x;
  bool y;
  bool z;
} AuvAxisOk;

#define AUV_FRAME_MAX_OBJECTS 128
#define AUV_FRAME_ERROR_MESSAGE_CAPACITY 256

typedef enum {
  AUV_FRAME_ERROR_NONE = 0,
  AUV_FRAME_ERROR_SENSOR = 2,
  AUV_FRAME_ERROR_END_OF_STREAM = 3,
  AUV_FRAME_ERROR_NOT_INITIALIZED = 4,
  AUV_FRAME_ERROR_TIMEOUT = 5,
  AUV_FRAME_ERROR_INITIALIZATION = 6
} AuvFrameErrorCode;

typedef struct {
  // Zero means success. Plugins may define additional custom error codes.
  uint32_t code;
  char message[AUV_FRAME_ERROR_MESSAGE_CAPACITY];
} AuvFrameError;

typedef struct {
  AuvObject objects[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects_len;
  MathPose camera_pose;
  uint64_t timestamp;
  /// World X, Y, Z. A false axis means `camera_pose.pos` on that axis is not usable.
  AuvAxisOk position_ok;
  /// Roll, pitch, yaw. A false axis means that attitude component is not usable.
  AuvAxisOk attitude_ok;
  MathVector3f accel;
  MathVector3f gyro;
  MathQuaternionf imu_quat;
  AuvObject2d objects2d[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects2d_len;
  uint32_t image_width;
  uint32_t image_height;
  float pressure_depth;
  bool pressure_depth_ok;
  AuvFrameError frame_error;
} AuvFrame;

// Initialization failures are reported in frame_error on the next yield call.
void auv_init(void);
// Wait for a complete frame or set frame_error. Each plugin measures elapsed
// time against its compiled deadline.
// A blocking SDK call can only be checked after it returns.
void auv_yield_until_next_frame(AuvFrame *frame);
void auv_deinit(void);

#ifdef __cplusplus

using AuvInitFunc = decltype(auv_init);
using AuvYieldUntilNextFrameFunc = decltype(auv_yield_until_next_frame);
using AuvDeinitFunc = decltype(auv_deinit);

#else

typedef typeof(auv_init) AuvInitFunc;
typedef typeof(auv_yield_until_next_frame) AuvYieldUntilNextFrameFunc;
typedef typeof(auv_deinit) AuvDeinitFunc;

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // AUV_H
