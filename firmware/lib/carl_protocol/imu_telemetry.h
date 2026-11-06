// ─────────────────────────────────────────────────────────────────────────────
// imu_telemetry.h — BNO08x and legacy MPU wire telemetry payloads
//
// Current BNO08x: 0x234/0x235 carry Q8 acceleration/Q9 gyro; 0x236/0x237
// carry Q14 quaternion XYZ/W and Q12 heading accuracy. All four parts share
// a sequence. Health, heading status and SPI snapshots use 0x238/0x239/0x23a.
// The legacy layouts below remain available for protocol compatibility only.
//
// A complete MPU-6500 motion sample contains three accelerometer axes and three
// gyroscope axes. Six signed 16-bit values need 12 bytes, but CARL deliberately
// limits every framed wire payload to the classic-CAN limit of 8 bytes. The
// sample is therefore split into separate wire messages:
//
//   0x230: sequence, status, accelerometer X/Y/Z
//   0x231: sequence, status, gyroscope X/Y/Z
//   0x232: sequence, status, raw die temperature
//
// All three frames carry the same rolling sequence number. The companion
// publishes a ROS IMU sample only after it has received both valid motion
// frames with matching sequence numbers; temperature is published separately.
// These IDs travel only over the framed master-to-companion wire link; the
// master does not place them on CARL's CAN bus.
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
static const uint16_t WIREID_IMU_TEMPERATURE = 0x232;
static const uint16_t WIREID_IMU_DIAGNOSTICS = 0x233;

// BNO08x uses distinct IDs: its fixed-point SI units MUST NOT be decoded as
// MPU-6500 ADC counts by an older companion bridge. Legacy IDs stay reserved.
static const uint16_t WIREID_BNO_ACCEL = 0x234;  // axis payload, Q8 m/s^2
static const uint16_t WIREID_BNO_GYRO = 0x235;   // axis payload, Q9 rad/s
static const uint16_t WIREID_BNO_QUAT_XYZ = 0x236;  // axis payload, Q14
static const uint16_t WIREID_BNO_QUAT_W = 0x237; // seq,status,w:i16,accuracy:u16
static const uint16_t WIREID_BNO_DIAGNOSTICS = 0x238;
static const uint16_t WIREID_HEADING_STATUS = 0x239;
static const uint16_t WIREID_BNO_SPI_DIAGNOSTICS = 0x23a;
static const uint16_t WIREID_MPU_SPI_DIAGNOSTICS = 0x23b;
// SPI schema 1: version,result,last receive header[4],GPIO snapshot,reserved.
// result: 0=no read,1=empty,2=packet,3=invalid length.
// GPIO bits before CS assertion: 0=INT low,1=RST high,2=WAKE high,3=CS high.
// Heading hold (10 Hz): state:u8, ready:u8, yaw:i16 milliradians,
// target-minus-yaw:i16 milliradians, signed left-subtract/right-add trim:i16.
// BNO health: schema=1,stage,failed_stage,error,detail,ready,count:u16.
// Stages and errors are defined in bno08x.h. No I2C address or WHO_AM_I.

// Independent of sample validity: failures must be observable even when no
// motion/temperature frames can be produced. Stage/error codes: mpu6500.h.
struct ImuDiagnostics {
  uint8_t stage;
  uint8_t failed_stage;
  uint8_t address;
  uint8_t identity;
  uint8_t error;
  uint8_t reg;
  uint16_t error_count;  // saturates at 65535; clears on STM32 restart
};

inline uint8_t pack_imu_diagnostics(uint8_t* buf, const ImuDiagnostics& d) {
  buf[0] = d.stage;
  buf[1] = d.failed_stage;
  buf[2] = d.address;
  buf[3] = d.identity;
  buf[4] = d.error;
  buf[5] = d.reg;
  buf[6] = (uint8_t)d.error_count;
  buf[7] = (uint8_t)(d.error_count >> 8);
  return 8;
}

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

// Temperature payload:
//   byte 0    rolling sequence number shared with acceleration/gyroscope
//   byte 1    ImuStatusFlag bitfield
//   bytes 2-3 signed raw TEMP_OUT value
struct ImuTemperatureTelemetry {
  uint8_t sequence;
  uint8_t status;
  int16_t temperature;
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

inline uint8_t pack_imu_temperature(
    uint8_t* buf, const ImuTemperatureTelemetry& sample) {
  buf[0] = sample.sequence;
  buf[1] = sample.status;
  put_i16(&buf[2], sample.temperature);
  return 4;
}

inline bool unpack_imu_temperature(
    const uint8_t* buf, uint8_t len, ImuTemperatureTelemetry* sample) {
  if (buf == nullptr || sample == nullptr || len != 4) return false;

  ImuTemperatureTelemetry decoded;
  decoded.sequence = buf[0];
  decoded.status = buf[1];
  decoded.temperature = get_i16(&buf[2]);

  *sample = decoded;
  return true;
}

}  // namespace carl
