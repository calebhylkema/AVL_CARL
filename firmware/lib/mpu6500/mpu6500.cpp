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

#include <Wire.h>

namespace carl {

namespace {

// Register addresses used by the minimal production configuration.
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
const uint8_t REG_INT_PIN_CFG  = 0x37;
const uint8_t REG_INT_ENABLE   = 0x38;
const uint8_t REG_INT_STATUS   = 0x3A;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_PWR_MGMT_2   = 0x6C;
const uint8_t REG_WHO_AM_I     = 0x75;

}  // namespace

void Mpu6500::begin(TwoWire& wire, uint32_t now, RecoverBus recoverBus) {
  wire_ = &wire;
  recoverBus_ = recoverBus;
  identity_ = 0;
  stage_ = POWER_WAIT;
  failedStage_ = POWER_WAIT;
  error_ = OK;
  errorCount_ = 0;
  lastRegister_ = 0;
  stageSince_ = now;
  configIndex_ = 0;
}

void Mpu6500::fail(uint32_t now) {
  failedStage_ = stage_;
  stage_ = RETRY_WAIT;
  stageSince_ = now;
  if (errorCount_ != UINT16_MAX) ++errorCount_;
}

void Mpu6500::poll(uint32_t now) {
  if (wire_ == nullptr) return;
  switch (stage_) {
    case POWER_WAIT:
      // The sensor may need up to 100 ms after power-up before register access.
      if (now - stageSince_ >= 100) stage_ = IDENTIFY;
      break;
    case RETRY_WAIT:
      if (now - stageSince_ >= 1000) {
        stage_ = RECOVER;
      }
      break;
    case RECOVER:
      // Retrying WHO_AM_I alone cannot clear a HAL/controller left busy by a
      // timed-out transfer. Board code releases/checks the lines and restarts
      // this dedicated I2C peripheral before another sensor transaction.
      error_ = recoverBus_ ? recoverBus_(*wire_) : (uint8_t)BUS_ERROR;
      if (error_ != OK) { fail(now); break; }
      identity_ = 0;
      stage_ = IDENTIFY;
      break;
    case IDENTIFY:
      // Identify before reset so a wrong sensor produces useful diagnostics.
      if (!readWhoAmI(&identity_)) { fail(now); break; }
      if (identity_ != EXPECTED_WHO_AM_I) {
        error_ = WRONG_IDENTITY;
        fail(now);
        break;
      }
      stage_ = RESET;
      break;
    case RESET:
      if (!writeRegister(REG_PWR_MGMT_1, 0x80)) { fail(now); break; }
      stageSince_ = now;
      stage_ = RESET_WAIT;
      break;
    case RESET_WAIT:
      if (now - stageSince_ >= 100) stage_ = WAKE;
      break;
    case WAKE:
      if (!writeRegister(REG_PWR_MGMT_1, 0x01)) { fail(now); break; }
      stageSince_ = now;
      stage_ = WAKE_WAIT;
      break;
    case WAKE_WAIT:
      if (now - stageSince_ >= 100) {
        configIndex_ = 0;
        stage_ = CONFIGURE;
      }
      break;
    case CONFIGURE: {
      // 100 Hz, ±2 g, ±250 deg/s; no interrupt required. One write per pass.
      static const uint8_t config[][2] = {
        {REG_PWR_MGMT_2, 0x00}, {REG_INT_ENABLE, 0x00},
        {REG_CONFIG, 0x03}, {REG_SMPLRT_DIV, 9},
        {REG_GYRO_CONFIG, 0x00}, {REG_ACCEL_CONFIG, 0x00},
        {REG_ACCEL_CONFIG2, 0x03}, {REG_INT_PIN_CFG, 0x00},
      };
      if (configIndex_ < sizeof(config) / sizeof(config[0])) {
        if (!writeRegister(config[configIndex_][0], config[configIndex_][1])) {
          fail(now);
          break;
        }
        ++configIndex_;
      } else {
        uint8_t ignored;
        if (!readRegisters(REG_INT_STATUS, &ignored, 1)) { fail(now); break; }
        stage_ = READY;
        error_ = OK;
      }
      break;
    }
    case READY:
    case SAMPLE:
      break;
  }
}

bool Mpu6500::readWhoAmI(uint8_t* identity) {
  if (identity == nullptr) return false;
  return readRegisters(REG_WHO_AM_I, identity, 1);
}

bool Mpu6500::readSample(Mpu6500RawSample* sample, uint32_t now) {
  if (sample == nullptr || !ready()) return false;

  uint8_t data[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) {
    stage_ = SAMPLE;
    fail(now);
    return false;
  }
  return decode_mpu6500_sample(data, sizeof(data), sample);
}

bool Mpu6500::writeRegister(uint8_t reg, uint8_t value) {
  if (wire_ == nullptr) return false;
  lastRegister_ = reg;

  wire_->beginTransmission(I2C_ADDRESS);
  if (wire_->write(reg) != 1 || wire_->write(value) != 1) {
    // Finish the transaction even after a local buffer failure so Wire returns
    // to an idle state before a later retry or other peripheral operation.
    wire_->endTransmission(true);
    error_ = BUFFER_ERROR;
    return false;
  }

  // Arduino Wire returns zero only when the address and both data bytes were
  // acknowledged by the sensor.
  error_ = wire_->endTransmission(true);
  return error_ == OK;
}

bool Mpu6500::readRegisters(uint8_t start_reg, uint8_t* data, size_t len) {
  if (wire_ == nullptr || data == nullptr || len == 0) return false;
  lastRegister_ = start_reg;

  wire_->beginTransmission(I2C_ADDRESS);
  if (wire_->write(start_reg) != 1) {
    wire_->endTransmission(true);
    error_ = BUFFER_ERROR;
    return false;
  }

  // A repeated start keeps control of the bus between selecting the first
  // register and reading its bytes. This is the normal MPU-6500 burst-read form.
  error_ = wire_->endTransmission(false);
  if (error_ != OK) return false;
  if (wire_->requestFrom(I2C_ADDRESS, len, true) != len) {
    // Drain any partial response so the next operation starts with an empty
    // receive buffer. The caller receives no partially decoded sample.
    while (wire_->available()) wire_->read();
    error_ = SHORT_READ;
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    if (!wire_->available()) { error_ = SHORT_READ; return false; }
    data[i] = (uint8_t)wire_->read();
  }

  error_ = OK;
  return true;
}

}  // namespace carl

#endif  // ARDUINO_ARCH_STM32
