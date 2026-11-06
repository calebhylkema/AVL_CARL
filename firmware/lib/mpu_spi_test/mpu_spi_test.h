#pragma once
#include <Arduino.h>
#include <SPI.h>

// Diagnostic MPU-6500/9250 SPI acquisition. No fusion, tare, or motor control.
// Startup waits are scheduled; each poll performs at most three short transfers.
class MpuSpiTest {
 public:
  enum Stage : uint8_t { POWER_WAIT, IDENTIFY, RESET_WAIT, WAKE_WAIT, CONFIGURE, READY, RETRY_WAIT };
  enum Error : uint8_t { OK, WRONG_IDENTITY, VERIFY_FAILED, DATA_TIMEOUT, IDENTITY_LOST };
  struct Sample { int16_t accel[3], temperature, gyro[3]; };
  MpuSpiTest(SPIClass& spi, uint32_t cs) : spi_(spi), cs_(cs) {}
  void begin(uint32_t now) {
    digitalWrite(cs_, HIGH); pinMode(cs_, OUTPUT); spi_.begin();
    stage_ = POWER_WAIT; error_ = OK; errors_ = 0; identity_ = 0;
    since_ = now; pending_ = false;
  }
  void poll(uint32_t now) {
    if (stage_ == POWER_WAIT || stage_ == RETRY_WAIT) {
      if (now - since_ >= (stage_ == POWER_WAIT ? 100U : 1000U)) stage_ = IDENTIFY;
      return;
    }
    if (stage_ == IDENTIFY) {
      identity_ = read(0x75);
      // Never write configuration into an unidentified part.
      if (!supported(identity_)) { fail(WRONG_IDENTITY, now); return; }
      write(0x6b, 0x80); stage_ = RESET_WAIT; since_ = now; return;
    }
    if (stage_ == RESET_WAIT) {
      if (now - since_ >= 100) {
        write(0x6b, 0x01); stage_ = WAKE_WAIT; since_ = now;
      }
      return;
    }
    if (stage_ == WAKE_WAIT) {
      if (now - since_ >= 100) { stage_ = CONFIGURE; config_ = 0; }
      return;
    }
    if (stage_ == CONFIGURE) {
      // Disable I2C for SPI; enable all axes, 100 Hz, +/-2 g, +/-250 deg/s.
      // Data ready is polled via INT_STATUS; the external INT is optional.
      static const uint8_t config[][2] = {
        {0x6b, 1}, {0x6a, 0x10}, {0x6c, 0}, {0x1a, 3}, {0x19, 9},
        {0x1b, 0}, {0x1c, 0}, {0x1d, 3}, {0x37, 0}, {0x38, 1},
      };
      const uint8_t reg = config[config_][0], value = config[config_][1];
      write(reg, value);
      if (read(reg) != value) { fail(VERIFY_FAILED, now); return; }
      if (++config_ == sizeof(config) / sizeof(config[0])) {
        stage_ = READY; error_ = OK;
        last_sample_ = last_poll_ = last_identity_ = now;
      }
      return;
    }
    if (now - last_identity_ >= 1000) {
      last_identity_ = now;
      const uint8_t identity = read(0x75);
      if (identity != identity_) { identity_ = identity; fail(IDENTITY_LOST, now); return; }
    }
    if (now - last_poll_ >= 5) {
      last_poll_ = now;
      const uint8_t status = read(0x3a);
      if (status == 0xff) { fail(DATA_TIMEOUT, now); return; }
      if (status & 1) {
        uint8_t data[14]; read(0x3b, data, sizeof(data));
        for (unsigned i = 0; i < 3; ++i) sample_.accel[i] = signed16(data + i * 2);
        sample_.temperature = signed16(data + 6);
        for (unsigned i = 0; i < 3; ++i) sample_.gyro[i] = signed16(data + 8 + i * 2);
        last_sample_ = now; pending_ = true;
      }
    }
    if (now - last_sample_ >= 250) fail(DATA_TIMEOUT, now);
  }
  bool takeSample(Sample& sample) {
    if (!pending_ || stage_ != READY) return false;
    sample = sample_; pending_ = false; return true;
  }
  void diagnostics(uint8_t* p) const {
    // Wire 0x23b, schema 1: version,stage,identity,error,register,value,count LE.
    p[0] = 1; p[1] = stage_; p[2] = identity_; p[3] = error_;
    p[4] = reg_; p[5] = value_; p[6] = uint8_t(errors_); p[7] = uint8_t(errors_ >> 8);
  }
  Stage stage() const { return stage_; }
  uint16_t errorCount() const { return errors_; }
  static bool supported(uint8_t id) { return id == 0x70 || id == 0x71; }
 private:
  static int16_t signed16(const uint8_t* p) { return int16_t((uint16_t(p[0]) << 8) | p[1]); }
  void open() {
    spi_.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_, LOW);
  }
  void close() { digitalWrite(cs_, HIGH); spi_.endTransaction(); }
  uint8_t read(uint8_t reg) { uint8_t value; read(reg, &value, 1); return value; }
  void read(uint8_t reg, uint8_t* p, size_t n) {
    reg_ = reg; open(); spi_.transfer(reg | 0x80);
    for (size_t i = 0; i < n; ++i) p[i] = spi_.transfer(0);
    close(); value_ = p[0];
  }
  void write(uint8_t reg, uint8_t value) {
    reg_ = reg; value_ = value;
    open(); spi_.transfer(reg & 0x7f); spi_.transfer(value); close();
  }
  void fail(Error error, uint32_t now) {
    error_ = error; if (errors_ != UINT16_MAX) ++errors_;
    stage_ = RETRY_WAIT; since_ = now; pending_ = false;
  }
  SPIClass& spi_; uint32_t cs_;
  Stage stage_ = POWER_WAIT; Error error_ = OK;
  uint32_t since_ = 0, last_sample_ = 0, last_poll_ = 0, last_identity_ = 0;
  uint16_t errors_ = 0;
  uint8_t identity_ = 0, config_ = 0, reg_ = 0, value_ = 0;
  Sample sample_ = {}; bool pending_ = false;
};
