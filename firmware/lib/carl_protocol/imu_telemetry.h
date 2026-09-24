// ─────────────────────────────────────────────────────────────────────────────
// imu_telemetry.h — wire-only MPU-6500 telemetry payloads
//
// A complete MPU-6500 motion sample contains three accelerometer axes and three
// gyroscope axes. Six signed 16-bit values need 12 bytes, but CARL deliberately
// limits every framed wire payload to the classic-CAN limit of 8 bytes. The
// sample is therefore split into two messages:
//
//   0x230: sequence, status, accelerometer X/Y/Z
//   0x231: sequence, status, gyroscope X/Y/Z
//
// Both halves carry the same rolling sequence number. The companion computer
// publishes a ROS IMU sample only after it has received both valid halves with
// matching sequence numbers. These IDs travel only over the framed master-to-
// companion wire link; the master does not place them on CARL's CAN bus.
//
// This file is pure C++ with no Arduino dependency, so the exact byte layout is
// covered by the native unit tests. All multi-byte fields are little-endian,
// matching every existing AVL_CARL protocol payload.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

#include "common.h"

namespace carl {

// Wire-only message IDs reserved for locally measured master telemetry. They
// are separate from CanId because these frames never consume CAN bus bandwidth.
static const uint16_t WIREID_IMU_ACCEL = 0x230;
static const uint16_t WIREID_IMU_GYRO  = 0x231;

// Status is a bitfield so later diagnostics can add information without
// changing the fixed eight-byte payload. Normal samples carry both bits.
enum ImuStatusFlag : uint8_t {
  IMU_STATUS_INITIALIZED  = 0x01,
  IMU_STATUS_SAMPLE_VALID = 0x02,
};

// Common payload used by both IMU message IDs:
//   byte 0    rolling sequence number
//   byte 1    ImuStatusFlag bitfield
//   bytes 2-3 X axis, signed int16
//   bytes 4-5 Y axis, signed int16
//   bytes 6-7 Z axis, signed int16
struct ImuAxisTelemetry {
  uint8_t sequence;
  uint8_t status;
  int16_t x;
  int16_t y;
  int16_t z;
};

inline uint8_t pack_imu_axis(uint8_t* buf, const ImuAxisTelemetry& sample) {
  buf[0] = sample.sequence;
  buf[1] = sample.status;
  put_i16(&buf[2], sample.x);
  put_i16(&buf[4], sample.y);
  put_i16(&buf[6], sample.z);
  return 8;
}

// Require exactly eight bytes. Accepting extra or missing bytes could hide a
// sender/receiver protocol mismatch and produce believable but incorrect data.
inline bool unpack_imu_axis(const uint8_t* buf, uint8_t len,
                            ImuAxisTelemetry* sample) {
  if (buf == nullptr || sample == nullptr || len != 8) return false;

  ImuAxisTelemetry decoded;
  decoded.sequence = buf[0];
  decoded.status = buf[1];
  decoded.x = get_i16(&buf[2]);
  decoded.y = get_i16(&buf[4]);
  decoded.z = get_i16(&buf[6]);

  *sample = decoded;
  return true;
}

}  // namespace carl
