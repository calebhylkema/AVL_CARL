#pragma once

#include <math.h>
#include <stdint.h>
#include "bno08x.h"
#include "messages.h"

namespace carl {

// Positive vehicle yaw is counterclockwise viewed from above. With the
// existing motor map, signed left-=trim/right+=trim produces positive yaw in
// BOTH forward and reverse. Verify that physical response before ground use.
struct HeadingHoldConfig {
  bool enabled = true;
  float steering_sign = 1.0f;  // Set -1 only if a physical sign check requires it.
  float kp = 180.0f;          // per-mille / radian of heading error
  float ki = 20.0f;           // per-mille / (radian * second)
  float kd = 40.0f;           // per-mille / (radian / second)
  float max_trim = 100.0f;    // 10 percentage points, also limited to 25% of input
  float slew_per_second = 200.0f;
};

class HeadingHold {
 public:
  enum State : uint8_t { DISABLED, IDLE, HOLDING, MANUAL_TURN, WAIT_IMU, REARM };
  static const uint32_t FRESH_MS = 100;
  static const uint32_t WARMUP_MS = 500;

  explicit HeadingHold(const HeadingHoldConfig& config = HeadingHoldConfig()) : config_(config) {}

  void invalidate() {
    if (active_) require_neutral_ = true;
    valid_ = false;
    resetControl();
  }

  bool accept(const Bno08xHeadingSample& s, uint32_t now) {
    if (s.accuracy < 2 || s.gyro_accuracy < 2 ||
        now - s.orientation_ms > FRESH_MS || now - s.gyro_ms > FRESH_MS) {
      invalidate();
      return false;
    }
    float q[4];
    float norm = 0.0f;
    for (int i = 0; i < 4; ++i) { q[i] = s.quaternion[i] / 16384.0f; norm += q[i] * q[i]; }
    if (norm < 0.9f || norm > 1.1f) { invalidate(); return false; }
    const float inv = 1.0f / sqrtf(norm);
    for (int i = 0; i < 4; ++i) q[i] *= inv;
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    // Last row of sensor-to-world rotation: world up expressed in sensor
    // coordinates. Projection handles either face of a flat mounting upward.
    const float ux = 2.0f * (x * z - w * y);
    const float uy = 2.0f * (y * z + w * x);
    const float uz = 1.0f - 2.0f * (x * x + y * y);
    if (fabsf(uz) < 0.8660254f) { invalidate(); return false; }  // >30 deg from flat
    const float yaw = atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z));
    const float rate = (ux * s.gyro[0] + uy * s.gyro[1] + uz * s.gyro[2]) / 512.0f;
    if (fabsf(rate) > 3.0f) { invalidate(); return false; }

    const uint32_t oldestAge = now - s.orientation_ms > now - s.gyro_ms
                                   ? now - s.orientation_ms : now - s.gyro_ms;
    return acceptYaw(yaw, rate, s.orientation_ms, now, oldestAge);
  }

  // A sensor adapter must establish calibration and mounting validity before
  // supplying relative yaw. Raw acceleration cannot establish absolute yaw.
  bool acceptYaw(float yaw, float rate, uint32_t sampleMs, uint32_t now,
                 uint32_t age = 0) {
    if (!isfinite(yaw) || !isfinite(rate) || fabsf(yaw) > 3.141593f ||
        fabsf(rate) > 3.0f || now - sampleMs > FRESH_MS || age > FRESH_MS) {
      invalidate();
      return false;
    }
    if (valid_ && now - lastSampleMs_ <= FRESH_MS) {
      const float dt = (sampleMs - orientationMs_) * 0.001f;
      // Detect sensor resets/tare jumps rather than steering toward an old
      // reference after the coordinate frame unexpectedly changes.
      if (dt > 0.0f && fabsf(wrap(yaw - yaw_) - rate * dt) > 0.15f) {
        invalidate();
        return false;
      }
    } else {
      if (active_) { require_neutral_ = true; resetControl(); }
      goodSinceMs_ = now;
    }
    yaw_ = yaw;
    rate_ = rate;
    orientationMs_ = sampleMs;
    lastSampleMs_ = now - (now - sampleMs > age ? now - sampleMs : age);
    valid_ = true;
    return true;
  }

  // Called at the master's existing 20 ms command tick. Original operator
  // intent is passed each time, never last tick's corrected outputs.
  ThrottleCmd update(uint32_t now, ThrottleCmd c, bool armed) {
    c = {clampv<int16_t>(c.fl, THROTTLE_MIN, THROTTLE_MAX),
         clampv<int16_t>(c.fr, THROTTLE_MIN, THROTTLE_MAX),
         clampv<int16_t>(c.rl, THROTTLE_MIN, THROTTLE_MAX),
         clampv<int16_t>(c.rr, THROTTLE_MIN, THROTTLE_MAX)};
    if (!config_.enabled) { resetControl(); state_ = DISABLED; return armed ? c : zero(); }
    const bool neutral = abs_i16(c.fl) < THROTTLE_DEADBAND &&
                         abs_i16(c.fr) < THROTTLE_DEADBAND &&
                         abs_i16(c.rl) < THROTTLE_DEADBAND &&
                         abs_i16(c.rr) < THROTTLE_DEADBAND;
    if (!armed || neutral) {
      resetControl();
      if (neutral) require_neutral_ = false;
      state_ = IDLE;
      return armed ? c : zero();
    }
    const bool straight = c.fl == c.fr && c.fl == c.rl && c.fl == c.rr;
    if (!straight) { resetControl(); state_ = MANUAL_TURN; return c; }
    if (require_neutral_) { resetControl(); state_ = REARM; return c; }
    if (!ready(now)) {
      resetControl();
      require_neutral_ = true;  // Recovery must not engage unexpectedly mid-run.
      state_ = WAIT_IMU;
      return c;
    }
    const int8_t direction = c.fl > 0 ? 1 : -1;
    if (!active_ || direction != direction_) {
      resetControl();
      target_ = yaw_;  // Software yaw zero; no sensor tare or flash writes.
      direction_ = direction;
      lastControlMs_ = now;
      active_ = true;
      state_ = HOLDING;
      return c;
    }
    const uint32_t elapsed = now - lastControlMs_;
    if (elapsed > FRESH_MS) {
      resetControl(); require_neutral_ = true; state_ = REARM; return c;
    }
    const float dt = elapsed * 0.001f;
    lastControlMs_ = now;
    const float base = abs_i16(c.fl);
    float limit = base * 0.25f;
    if (limit > config_.max_trim) limit = config_.max_trim;
    // Never move a small operator command across the throttle input deadband.
    if (limit > base - THROTTLE_DEADBAND) limit = base - THROTTLE_DEADBAND;
    error_ = wrap(target_ - yaw_);
    const float pError = fabsf(error_) < 0.00872665f ? 0.0f : error_;  // 0.5 degree
    const float candidateI = clampv<float>(integral_ + config_.ki * pError * dt, -limit, limit);
    float demand = config_.kp * pError + candidateI - config_.kd * rate_;
    // Conditional integration avoids winding up against the trim limit.
    if (!((demand > limit && pError > 0.0f) || (demand < -limit && pError < 0.0f))) {
      integral_ = candidateI;
    }
    demand = clampv<float>(config_.kp * pError + integral_ - config_.kd * rate_, -limit, limit);
    trim_ += clampv<float>(demand - trim_, -config_.slew_per_second * dt,
                          config_.slew_per_second * dt);
    trim_ = clampv<float>(trim_, -limit, limit);
    const int16_t correction = static_cast<int16_t>(roundf(trim_ * config_.steering_sign));
    c.fl = c.rl = clampv<int16_t>(c.fl - correction, THROTTLE_MIN, THROTTLE_MAX);
    c.fr = c.rr = clampv<int16_t>(c.fr + correction, THROTTLE_MIN, THROTTLE_MAX);
    state_ = HOLDING;
    return c;
  }

  bool ready(uint32_t now) const {
    return valid_ && now - lastSampleMs_ <= FRESH_MS && now - goodSinceMs_ >= WARMUP_MS;
  }
  State state() const { return state_; }
  float yaw() const { return yaw_; }
  float error() const { return error_; }
  int16_t correction() const { return static_cast<int16_t>(roundf(trim_ * config_.steering_sign)); }
  static float wrap(float a) {
    const float pi = 3.14159265358979323846f;
    if (a > pi) a -= 2.0f * pi;
    if (a < -pi) a += 2.0f * pi;
    return a;
  }

 private:
  static ThrottleCmd zero() { return ThrottleCmd{0, 0, 0, 0}; }
  void resetControl() { active_ = false; integral_ = trim_ = error_ = 0.0f; }
  HeadingHoldConfig config_;
  State state_ = IDLE;
  bool valid_ = false, active_ = false, require_neutral_ = false;
  int8_t direction_ = 0;
  uint32_t lastSampleMs_ = 0, orientationMs_ = 0, goodSinceMs_ = 0, lastControlMs_ = 0;
  float yaw_ = 0.0f, rate_ = 0.0f, target_ = 0.0f, integral_ = 0.0f, trim_ = 0.0f, error_ = 0.0f;
};

}  // namespace carl
