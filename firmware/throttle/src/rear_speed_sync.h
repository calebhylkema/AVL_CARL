#pragma once

#include <stdint.h>
#include "carl_protocol.h"
#include "common.h"

namespace carl {

// Relative RPM control for equal, same-direction rear commands. No guessed
// maximum wheel RPM or gearbox ratio: the two measured speeds are compared
// directly. Different commands (turns / individual-wheel tests) pass through.
class RearSpeedSync {
 public:
  static const uint32_t PERIOD_MS = 20;
  static const uint32_t COUNTS_PER_REV = 2400;  // 600 PPR, Encoder's 4x decode
  static const uint32_t FEEDBACK_FRESH_MS = 200;
  static const uint32_t SETTLE_MS = 160;
  enum State : uint8_t { BYPASS, QUALIFYING, REGULATING, FEEDBACK_UNAVAILABLE };

  struct Output { int16_t left; int16_t right; };
  State state() const { return state_; }

  Output update(uint32_t now, int32_t leftCount, int32_t rightCount,
                int16_t left, int16_t right, bool armed) {
    left = clampv<int16_t>(left, THROTTLE_MIN, THROTTLE_MAX);
    right = clampv<int16_t>(right, THROTTLE_MIN, THROTTLE_MAX);
    // These checks run on EVERY foreground loop, even between control ticks.
    const bool eligible = armed && left == right && abs_i16(left) >= THROTTLE_DEADBAND;
    if (!eligible) {
      reset(now, leftCount, rightCount);
      return armed ? Output{left, right} : Output{0, 0};
    }

    const int8_t direction = left > 0 ? 1 : -1;
    if (!active_ || direction != direction_) {
      reset(now, leftCount, rightCount);
      active_ = true;
      direction_ = direction;
      startMs_ = now;
      state_ = QUALIFYING;
    }

    const uint32_t elapsed = now - lastMs_;
    const float base = left > 0 ? left : -left;
    // Discard stale samples after a foreground delay; never integrate a long
    // catch-up interval or retain a correction based on old wheel motion.
    if (elapsed > FEEDBACK_FRESH_MS) {
      reset(now, leftCount, rightCount);
      state_ = FEEDBACK_UNAVAILABLE;
      return Output{left, right};
    }
    if (elapsed >= PERIOD_MS) {
      const float dt = elapsed * 0.001f;
      const float alpha = dt / (0.080f + dt);  // 80 ms low-pass, low-speed counts
      const float scale = 60000.0f / (COUNTS_PER_REV * elapsed);
      const uint32_t dl = countDistance(leftCount, lastLeft_);
      const uint32_t dr = countDistance(rightCount, lastRight_);
      lastMs_ = now;
      lastLeft_ = leftCount;
      lastRight_ = rightCount;
      // Reject impossible counter jumps before they enter the speed filter.
      // 3000 shaft RPM is a deliberately generous ceiling for these geared
      // motors, not an assumed normal wheel speed.
      if (dl * scale > 3000.0f || dr * scale > 3000.0f) degraded_ = true;
      leftRpm_ += alpha * (dl * scale - leftRpm_);
      rightRpm_ += alpha * (dr * scale - rightRpm_);
      if (dl) { leftEdgeMs_ = now; haveLeftEdge_ = true; }
      if (dr) { rightEdgeMs_ = now; haveRightEdge_ = true; }
      leftEvidence_ = dl >= 4U - leftEvidence_ ? 4 : leftEvidence_ + dl;
      rightEvidence_ = dr >= 4U - rightEvidence_ ? 4 : rightEvidence_ + dr;
    }

    const bool fresh = haveLeftEdge_ && haveRightEdge_ &&
                       now - leftEdgeMs_ < FEEDBACK_FRESH_MS &&
                       now - rightEdgeMs_ < FEEDBACK_FRESH_MS &&
                       leftEvidence_ >= 4 && rightEvidence_ >= 4;
    if (qualified_ && !fresh) degraded_ = true;
    if (!fresh || degraded_) {
      // Missing counts are NOT a target of zero RPM for the other wheel.
      // Restore both nominal commands together. After an in-run dropout,
      // remain in fallback until neutral/turn/reversal resets this run.
      state_ = FEEDBACK_UNAVAILABLE;
      qualified_ = false;
      integral_ = trim_ = 0.0f;
      startMs_ = now;
      return Output{left, right};
    }
    if (!qualified_ && now - startMs_ < SETTLE_MS) {
      state_ = QUALIFYING;
      return Output{left, right};
    }
    qualified_ = true;
    state_ = REGULATING;
    if (elapsed >= PERIOD_MS) {
      const float dt = elapsed * 0.001f;
      // Fractional error makes the loop responsive at light throttle too.
      // A 5 RPM denominator floor limits amplification of individual counts.
      const float mean = (leftRpm_ + rightRpm_) * 0.5f;
      const float error = (leftRpm_ - rightRpm_) / (mean > 5.0f ? mean : 5.0f);
      const float p = KP * error;
      // Trim is a fraction of the requested PWM. Clamping the integrator
      // against the available output range prevents windup at saturation.
      integral_ = clampv<float>(integral_ + KI * error * dt,
                               -0.5f - p, 0.5f - p);
      const float desired = clampv<float>(p + integral_, -0.5f, 0.5f);
      trim_ += clampv<float>(desired - trim_, -0.5f * dt, 0.5f * dt);
    }

    // Regulation is a bounded trim, never authority to stop a commanded
    // wheel. Retain >=75% of nominal and never cross the input deadband.
    // Slower-wheel boost remains capped at 50% AND 100 per-mille.
    const float boost = base * 0.5f < 100.0f ? base * 0.5f : 100.0f;
    const float minimum = base * 0.75f > THROTTLE_DEADBAND ? base * 0.75f : THROTTLE_DEADBAND;
    const float correction = trim_ * base;
    const float maximum = clampv<float>(base + boost, minimum, THROTTLE_MAX);
    const float l = clampv<float>(base - correction, minimum, maximum);
    const float r = clampv<float>(base + correction, minimum, maximum);
    return Output{static_cast<int16_t>(direction * static_cast<int16_t>(l + 0.5f)),
                  static_cast<int16_t>(direction * static_cast<int16_t>(r + 0.5f))};
  }

 private:
  static constexpr float KP = 0.20f;
  static constexpr float KI = 0.80f;  // fraction of command / second / relative error

  // Unsigned subtraction handles signed encoder rollover without signed
  // overflow. Magnitude is independent of each encoder's A/B polarity. Keep
  // raw signed counts unchanged in CAN telemetry for existing consumers.
  static uint32_t countDistance(int32_t count, int32_t previous) {
    const uint32_t delta = static_cast<uint32_t>(count) - static_cast<uint32_t>(previous);
    return delta <= 0x7fffffffU ? delta : 0U - delta;
  }

  void reset(uint32_t now, int32_t left, int32_t right) {
    active_ = false;
    lastMs_ = now;
    lastLeft_ = left;
    lastRight_ = right;
    leftRpm_ = rightRpm_ = integral_ = trim_ = 0.0f;
    haveLeftEdge_ = haveRightEdge_ = false;
    qualified_ = degraded_ = false;
    leftEvidence_ = rightEvidence_ = 0;
    state_ = BYPASS;
  }

  bool active_ = false, haveLeftEdge_ = false, haveRightEdge_ = false;
  bool qualified_ = false, degraded_ = false;
  uint8_t leftEvidence_ = 0, rightEvidence_ = 0;
  State state_ = BYPASS;
  int8_t direction_ = 0;
  uint32_t lastMs_ = 0, startMs_ = 0, leftEdgeMs_ = 0, rightEdgeMs_ = 0;
  int32_t lastLeft_ = 0, lastRight_ = 0;
  float leftRpm_ = 0.0f, rightRpm_ = 0.0f, integral_ = 0.0f, trim_ = 0.0f;
};

}  // namespace carl
