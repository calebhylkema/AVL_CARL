// ─────────────────────────────────────────────────────────────────────────────
// mpu6500.h — minimal MPU-6500 driver for the NUCLEO-F767ZI master
//
// The driver deliberately has a small job:
//   • verify that the device at I2C address 0x68 identifies as an MPU-6500;
//   • configure a 100 Hz accelerometer/gyroscope sample stream; and
//   • read one coherent 14-byte accelerometer/temperature/gyroscope sample.
//
// It does not print, publish, or send CAN/serial frames. Keeping communication
// out of the driver prevents IMU failures from changing CARL's existing safety
// and transport behavior. The raw sample decoder is platform-independent so it
// can also be checked by the host/native unit tests.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stddef.h>
#include <stdint.h>

class TwoWire;

namespace carl {

// One raw sensor sample exactly as it appears in registers 0x3B through 0x48.
// The selected full-scale ranges are ±2 g and ±250 degrees/second. Conversion
// to SI units is intentionally left to the companion computer in a later step.
struct Mpu6500RawSample {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temperature;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

// Convert the MPU-6500's big-endian register bytes into signed host integers.
// This helper performs no I2C work, which keeps the byte-order logic testable
// on a development computer. A short or null input is rejected without writing
// a partial sample to the caller's output structure.
inline bool decode_mpu6500_sample(const uint8_t* data, size_t len,
                                  Mpu6500RawSample* sample) {
  if (data == nullptr || sample == nullptr || len < 14) return false;

  Mpu6500RawSample decoded;
  decoded.accel_x = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  decoded.accel_y = (int16_t)((uint16_t)data[2] << 8 | data[3]);
  decoded.accel_z = (int16_t)((uint16_t)data[4] << 8 | data[5]);
  decoded.temperature = (int16_t)((uint16_t)data[6] << 8 | data[7]);
  decoded.gyro_x = (int16_t)((uint16_t)data[8] << 8 | data[9]);
  decoded.gyro_y = (int16_t)((uint16_t)data[10] << 8 | data[11]);
  decoded.gyro_z = (int16_t)((uint16_t)data[12] << 8 | data[13]);

  *sample = decoded;
  return true;
}

class Mpu6500 {
 public:
  static const uint8_t I2C_ADDRESS = 0x68;
  static const uint8_t EXPECTED_WHO_AM_I = 0x70;

  Mpu6500() : wire_(nullptr), identity_(0) {}

  // Reset, identify, and configure the device. Returns false after any failed
  // I2C transaction or if WHO_AM_I is not the MPU-6500 value 0x70.
  bool begin(TwoWire& wire);

  // Read WHO_AM_I without changing the device configuration.
  bool readWhoAmI(uint8_t* identity);

  // Read all motion axes and temperature in one burst so every field belongs
  // to the same sample instant.
  bool readSample(Mpu6500RawSample* sample);

  // Last identity byte observed during begin(). A zero means begin() did not
  // reach a successful identity read.
  uint8_t identity() const { return identity_; }

 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t start_reg, uint8_t* data, size_t len);

  TwoWire* wire_;
  uint8_t identity_;
};

}  // namespace carl
