#pragma once

#include <math.h>
#include <stdint.h>

namespace carl {

// Planar relative heading for the MPU's +/-2 g, +/-250 deg/s configuration.
// No magnetometer/absolute north. Calibrate only with neutral operator intent,
// zero reported motor output, and fresh, unchanged rear encoder feedback.
// World-up is measured at rest, so a flat upside-down mount works as well.
class MpuHeading {
 public:
  static const uint32_t CALIBRATION_MS = 3000;
  static const uint32_t MAX_GAP_MS = 30;
  static const uint32_t MAX_RUN_MS = 30000;
  static const uint32_t CALIBRATION_LIFETIME_MS = 60000;
  static constexpr float GYRO_SCALE = 0.01745329252f / 131.0f;
  enum State : uint8_t { CALIBRATING, READY, TIMING_FAULT, MOTION_FAULT, RUN_EXPIRED };

  void invalidate(State reason = TIMING_FAULT) {
    calibrated_ = haveSample_ = running_ = usedForRun_ = false;
    state_ = reason;
    count_ = 0;
    yaw_ = rate_ = 0.0f;
  }

  bool accept(const int16_t accel[3], const int16_t gyro[3], int16_t temperature,
              uint32_t now, bool stationaryAllowed) {
    if (calibrated_ && ((stationaryAllowed && usedForRun_) ||
                       now - calibratedMs_ >= CALIBRATION_LIFETIME_MS)) {
      invalidate(CALIBRATING);
    }
    const uint32_t elapsed = now - lastMs_;
    if (haveSample_ && (elapsed == 0 || elapsed > MAX_GAP_MS)) {
      invalidate();
      // Start a new calibration window; never integrate across missing data.
    }
    haveSample_ = true;
    lastMs_ = now;

    float a[3], g[3], norm2 = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
      a[i] = accel[i] / 16384.0f;
      g[i] = gyro[i] * GYRO_SCALE;
      norm2 += a[i] * a[i];
      if (gyro[i] <= -32700 || gyro[i] >= 32700) {
        invalidate(MOTION_FAULT); return false;
      }
    }
    // Installed unit reads about 1.086 g at rest. Do not assume exact factory
    // accel calibration. Reject large acceleration, free fall and non-flat mount.
    if (norm2 < 0.64f || norm2 > 1.44f ||
        fabsf(a[2]) < 0.8660254f * sqrtf(norm2)) {
      invalidate(MOTION_FAULT); return false;
    }
    const float norm = sqrtf(norm2);
    for (unsigned i = 0; i < 3; ++i) a[i] /= norm;

    if (!calibrated_) {
      if (!stationaryAllowed) { count_ = 0; return false; }
      // A steady offset is learned, never hard-coded from one live capture.
      // All three axes matter when the board is slightly tilted. The installed
      // module has sizeable X/Y offsets; wheel/output evidence guards calibration.
      for (unsigned i = 0; i < 3; ++i) {
        if (fabsf(g[i]) > 0.34906585f) { count_ = 0; return false; } // 20 deg/s
        if (count_ && (fabsf(g[i] - gyroMean_[i]) > 0.00872665f ||
                       fabsf(a[i] - upMean_[i]) > 0.035f)) {
          count_ = 0;
        }
      }
      if (!count_) {
        sinceMs_ = now;
        for (unsigned i = 0; i < 3; ++i) {
          gyroMean_[i] = g[i]; upMean_[i] = a[i]; gyroM2_[i] = 0.0f;
        }
        temperature_ = temperature;
        count_ = 1;
        return false;
      }
      ++count_;
      for (unsigned i = 0; i < 3; ++i) {
        const float d = g[i] - gyroMean_[i];
        gyroMean_[i] += d / count_;
        gyroM2_[i] += d * (g[i] - gyroMean_[i]);
        upMean_[i] += (a[i] - upMean_[i]) / count_;
      }
      if (now - sinceMs_ < CALIBRATION_MS || count_ < 250) return false;
      float length = 0.0f;
      for (unsigned i = 0; i < 3; ++i) {
        if (gyroM2_[i] / count_ > 0.00000761544f) { // stddev >0.158 deg/s
          count_ = 0; return false;
        }
        length += upMean_[i] * upMean_[i];
      }
      for (unsigned i = 0; i < 3; ++i) up_[i] = upMean_[i] / sqrtf(length);
      calibrated_ = true;
      calibratedMs_ = now;
      yaw_ = rate_ = 0.0f;
      state_ = READY;
      return true;
    }

    float vertical = 0.0f, rate = 0.0f, totalRate2 = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
      vertical += a[i] * up_[i];
      const float corrected = g[i] - gyroMean_[i];
      rate += corrected * up_[i];
      totalRate2 += corrected * corrected;
    }
    // Planar assumption: release on >15 deg gravity change, rapid roll/pitch,
    // excessive yaw, or a >5 C thermal change since bias calibration.
    if (vertical < 0.9659258f || totalRate2 - rate * rate > 0.25f ||
        fabsf(rate) > 3.0f || fabsf(float(temperature) - temperature_) > 1669.35f) {
      invalidate(MOTION_FAULT); return false;
    }
    // Trapezoidal integration; timestamp subtraction handles millis rollover.
    yaw_ = wrap(yaw_ + (rate_ + rate) * 0.5f * elapsed * 0.001f);
    rate_ = rate;
    return true;
  }

  // Each straight run gets at most 30 s of gyro-only correction. A new
  // stationary calibration is required afterward to bound unobserved drift.
  bool allowRun(uint32_t now, bool straightRun) {
    if (!straightRun) { running_ = false; return calibrated_; }
    if (!calibrated_) return false;
    if (!running_) { running_ = usedForRun_ = true; runMs_ = now; }
    if (now - runMs_ >= MAX_RUN_MS) { invalidate(RUN_EXPIRED); return false; }
    return true;
  }
  bool ready(uint32_t now) const {
    return calibrated_ && now - lastMs_ <= MAX_GAP_MS &&
           now - calibratedMs_ < CALIBRATION_LIFETIME_MS;
  }
  State state() const { return state_; }
  float yaw() const { return yaw_; }
  float rate() const { return rate_; }
  float bias(unsigned axis) const { return axis < 3 ? gyroMean_[axis] : 0.0f; }
  uint16_t calibrationSamples() const { return count_; }

 private:
  static float wrap(float a) {
    if (a > 3.14159265359f) a -= 6.28318530718f;
    if (a < -3.14159265359f) a += 6.28318530718f;
    return a;
  }
  bool calibrated_ = false, haveSample_ = false, running_ = false, usedForRun_ = false;
  State state_ = CALIBRATING;
  uint32_t lastMs_ = 0, sinceMs_ = 0, runMs_ = 0, calibratedMs_ = 0;
  uint16_t count_ = 0;
  float gyroMean_[3] = {}, gyroM2_[3] = {}, upMean_[3] = {}, up_[3] = {};
  float yaw_ = 0.0f, rate_ = 0.0f, temperature_ = 0.0f;
};

}  // namespace carl
