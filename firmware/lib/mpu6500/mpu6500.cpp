// ─────────────────────────────────────────────────────────────────────────────
// mpu6500.cpp — MPU-6500 register access and startup configuration
//
// All bus traffic happens from normal foreground code. The master polls this
// driver from loop() because CARL intentionally leaves the module's INT output
// unconnected.
// ─────────────────────────────────────────────────────────────────────────────
#include "mpu6500.h"

// The native unit-test environment has no Arduino or Wire implementation. It
// uses only the platform-independent decoder in mpu6500.h. The hardware driver
// below is compiled only for the STM32 Arduino firmware target.
#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>
#include <Wire.h>

namespace carl {

namespace {

// Register addresses used by the minimal production configuration.
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_INT_PIN_CFG  = 0x37;
const uint8_t REG_INT_ENABLE   = 0x38;
const uint8_t REG_INT_STATUS   = 0x3A;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_PWR_MGMT_2   = 0x6C;
const uint8_t REG_WHO_AM_I     = 0x75;

}  // namespace

bool Mpu6500::begin(TwoWire& wire) {
  wire_ = &wire;
  identity_ = 0;

  // Reset first so a previous firmware run cannot leave behind an unknown
  // sample rate, range, interrupt mode, or clock selection.
  if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
  delay(100);  // The data sheet requires time for reset and register restore.

  // Wake the device and select the X-axis gyroscope PLL as its clock source.
  // The PLL is more stable for sampled motion data than the internal oscillator.
  if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
  if (!writeRegister(REG_PWR_MGMT_2, 0x00)) return false;
  delay(10);  // Let the selected clock settle before configuring sampling.

  if (!readWhoAmI(&identity_)) return false;
  if (identity_ != EXPECTED_WHO_AM_I) return false;

  // Keep the optional data-ready output disabled while the remaining
  // registers are being changed.
  if (!writeRegister(REG_INT_ENABLE, 0x00)) return false;

  // DLPF configuration 3 enables filtering and gives a 1 kHz internal sample
  // rate. Dividing that rate by (1 + 9) produces the requested 100 Hz output.
  if (!writeRegister(REG_CONFIG, 0x03)) return false;
  if (!writeRegister(REG_SMPLRT_DIV, 9)) return false;

  // FS_SEL=0 and AFS_SEL=0 select ±250 degrees/second and ±2 g. These narrow
  // ranges provide the best raw resolution for initial vehicle measurements.
  if (!writeRegister(REG_GYRO_CONFIG, 0x00)) return false;
  if (!writeRegister(REG_ACCEL_CONFIG, 0x00)) return false;

  // Keep the interrupt pin at its default electrical configuration even though
  // it is not connected in CARL's wiring.
  if (!writeRegister(REG_INT_PIN_CFG, 0x00)) return false;

  // Reading INT_STATUS clears any data-ready condition accumulated while the
  // device was being configured. Failure here is treated as an I2C failure.
  uint8_t ignored_status = 0;
  if (!readRegisters(REG_INT_STATUS, &ignored_status, 1)) return false;

  // Leave RAW_DATA_RDY_INT disabled because the physical INT output is not
  // connected. The sample registers still update at the configured 100 Hz.
  if (!writeRegister(REG_INT_ENABLE, 0x00)) return false;

  return true;
}

bool Mpu6500::readWhoAmI(uint8_t* identity) {
  if (identity == nullptr) return false;
  return readRegisters(REG_WHO_AM_I, identity, 1);
}

bool Mpu6500::readSample(Mpu6500RawSample* sample) {
  if (sample == nullptr) return false;

  uint8_t data[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) return false;
  return decode_mpu6500_sample(data, sizeof(data), sample);
}

bool Mpu6500::writeRegister(uint8_t reg, uint8_t value) {
  if (wire_ == nullptr) return false;

  wire_->beginTransmission(I2C_ADDRESS);
  if (wire_->write(reg) != 1 || wire_->write(value) != 1) {
    // Finish the transaction even after a local buffer failure so Wire returns
    // to an idle state before a later retry or other peripheral operation.
    wire_->endTransmission(true);
    return false;
  }

  // Arduino Wire returns zero only when the address and both data bytes were
  // acknowledged by the sensor.
  return wire_->endTransmission(true) == 0;
}

bool Mpu6500::readRegisters(uint8_t start_reg, uint8_t* data, size_t len) {
  if (wire_ == nullptr || data == nullptr || len == 0) return false;

  wire_->beginTransmission(I2C_ADDRESS);
  if (wire_->write(start_reg) != 1) {
    wire_->endTransmission(true);
    return false;
  }

  // A repeated start keeps control of the bus between selecting the first
  // register and reading its bytes. This is the normal MPU-6500 burst-read form.
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(I2C_ADDRESS, len, true) != len) {
    // Drain any partial response so the next operation starts with an empty
    // receive buffer. The caller receives no partially decoded sample.
    while (wire_->available()) wire_->read();
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    if (!wire_->available()) return false;
    data[i] = (uint8_t)wire_->read();
  }

  return true;
}

}  // namespace carl

#endif  // ARDUINO_ARCH_STM32
