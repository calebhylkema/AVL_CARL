#pragma once

#include <stddef.h>
#include <stdint.h>

// Small, allocation-free SH-2/SHTP subset. No blocking boot/INT waits and no
// dependency on Arduino, so the production state machine can be host-tested.
class Bno08xTransport {
 public:
  virtual ~Bno08xTransport() {}
  virtual void reset(bool asserted) = 0;
  virtual void wake(bool asserted) = 0;
  virtual bool interruptActive() = 0;
  // Complete SHTP packet including its four-byte header; 0 means no packet.
  virtual int read(uint8_t* packet, size_t capacity) = 0;
  // Nonblocking: false means the device has not yet acknowledged WAKE.
  virtual bool write(uint8_t channel, uint8_t sequence,
                     const uint8_t* data, uint8_t length) = 0;
};

struct Bno08xSample {
  int16_t accel[3];       // Q8 m/s^2, including gravity
  int16_t gyro[3];        // Q9 rad/s
  int16_t quaternion[4];  // Q14 x, y, z, w
  uint16_t heading_accuracy;  // Q12 radians (not a covariance)
  uint8_t accuracy[3];    // SH-2 calibration status: accel, gyro, rotation
};

// Control-only, magnetometer-free orientation. Independent of the original
// rotation-vector telemetry; timestamps are host reception times in millis.
struct Bno08xHeadingSample {
  int16_t quaternion[4];  // Game Rotation Vector, Q14 x,y,z,w
  int16_t gyro[3];        // Calibrated gyro, Q9 rad/s in sensor coordinates
  uint8_t accuracy, gyro_accuracy;
  uint32_t orientation_ms, gyro_ms;
};

class Bno08x {
 public:
  enum Stage : uint8_t {
    POWER_WAIT, RESET_LOW, BOOT_WAIT, IDENTIFY, CONFIGURE,
    STREAM_WAIT, READY, RETRY_WAIT
  };
  enum Error : uint8_t {
    OK, BOOT_TIMEOUT, IDENTIFY_TIMEOUT, CONFIGURE_TIMEOUT,
    INVALID_PACKET, UNSUPPORTED_FRAGMENT, REPORT_TIMEOUT, SENSOR_RESET
  };
  static const uint16_t MAX_PACKET = 512;
  void begin(Bno08xTransport& transport, uint32_t now);
  void poll(uint32_t now);
  bool takeSample(Bno08xSample& sample, uint32_t now);
  bool takeHeadingSample(Bno08xHeadingSample& sample, uint32_t now);
  bool ready() const { return stage_ == READY; }
  Stage stage() const { return stage_; }
  Stage failedStage() const { return failed_stage_; }
  Error error() const { return error_; }
  uint8_t detail() const { return detail_; }
  uint16_t errorCount() const { return error_count_; }
  uint32_t product() const { return product_; }

 private:
  void startReset(uint32_t now);
  void fail(Error error, uint8_t detail, uint32_t now);
  void parse(const uint8_t* packet, uint16_t size, uint32_t now);
  Bno08xTransport* transport_ = nullptr;
  Stage stage_ = POWER_WAIT, failed_stage_ = POWER_WAIT;
  Error error_ = OK;
  uint8_t detail_ = 0, control_sequence_ = 0, feature_ = 0;
  uint16_t error_count_ = 0;
  uint32_t since_ = 0, product_ = 0, last_report_[3] = {};
  uint8_t report_sequence_[3] = {}, seen_ = 0, dirty_ = 0;
  bool identified_ = false, requested_ = false;
  Bno08xSample sample_ = {};
  Bno08xHeadingSample heading_ = {};
  bool heading_seen_ = false, heading_dirty_ = false;
  uint8_t heading_sequence_ = 0;
  uint8_t packet_[MAX_PACKET];
};
