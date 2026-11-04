#include "../../../throttle/src/rear_speed_sync.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

using carl::RearSpeedSync;

// Deliberately mismatched motor gain + friction, inertia, quadrature count
// quantization, and opposite encoder polarity. This exercises convergence,
// rather than just checking the sign of a calculated correction.
static void simulate(int command, bool swapMotors) {
  RearSpeedSync sync;
  double rpm[2] = {}, counts[2] = {};
  const double gains[2] = {swapMotors ? 0.27 : 0.20, swapMotors ? 0.20 : 0.27};
  const double friction[2] = {swapMotors ? 10.0 : 30.0, swapMotors ? 30.0 : 10.0};
  RearSpeedSync::Output output = {0, 0};
  double errorSum = 0, speedSum = 0;
  for (uint32_t now = 0; now <= 10000; now += 20) {
    output = sync.update(now, static_cast<int32_t>(counts[0]),
                         -static_cast<int32_t>(counts[1]), command, command, true);
    const int duty[2] = {std::abs(output.left), std::abs(output.right)};
    for (int i = 0; i < 2; ++i) {
      assert(duty[i] <= 1000);
      assert(duty[i] <= std::abs(command) + 100);
      assert(output.left * command >= 0 && output.right * command >= 0);
      const double target = gains[i] * std::fmax(0.0, duty[i] - friction[i]);
      rpm[i] += (target - rpm[i]) * 0.02 / 0.20;
      counts[i] += rpm[i] * 2400.0 / 60.0 * 0.02;
    }
    if (now >= 8000) {
      errorSum += std::abs(rpm[0] - rpm[1]);
      speedSum += (rpm[0] + rpm[1]) * 0.5;
    }
  }
  assert(speedSum > 100);  // Matching at a standstill is not convergence.
  const double relativeError = errorSum / speedSum;
  std::printf("command=%d swapped=%d: final RPM %.2f / %.2f, mean mismatch %.2f%%\n",
              command, swapMotors, rpm[0], rpm[1], relativeError * 100);
  assert(relativeError < 0.035);
}

static void limitsAndReset() {
  RearSpeedSync sync;
  int32_t left = 0, right = 0;
  RearSpeedSync::Output out;
  // One missing encoder must leave BOTH nominal commands intact.
  for (uint32_t now = 0; now <= 1000; now += 20) {
    left += 20;
    out = sync.update(now, left, right, 100, 100, true);
    assert(out.right == 100 && out.left == 100);
  }
  assert(sync.state() == RearSpeedSync::FEEDBACK_UNAVAILABLE);
  sync = RearSpeedSync{};
  for (uint32_t now = 1020; now <= 3000; now += 20) {
    left += 20; right += 10;
    out = sync.update(now, left, right, 100, 100, true);
    assert(out.left >= 75 && out.right >= 75);
  }
  assert(out.left < 100 && sync.state() == RearSpeedSync::REGULATING);
  // Exercise each immediate reset with a nonzero learned correction, with
  // less than a single sample period elapsed since the last update.
  for (auto pair : {RearSpeedSync::Output{0, 0}, {-100, -100},
                    {50, 100}, {100, -100}, {0, 100}, {19, 19}}) {
    RearSpeedSync trial = sync;
    out = trial.update(3001, left, right, pair.left, pair.right, true);
    assert(out.left == pair.left && out.right == pair.right);
    out = trial.update(3002, left, right, 100, 100, true);
    assert(out.left == 100 && out.right == 100);
  }
  RearSpeedSync disarmed = sync;
  out = disarmed.update(3001, left, right, 100, 100, false);
  assert(out.left == 0 && out.right == 0);
  out = disarmed.update(3002, left, right, 100, 100, true);
  assert(out.left == 100 && out.right == 100);
  // A fresh command after a long foreground pause starts without old trim.
  out = sync.update(3300, left, right, 100, 100, true);
  assert(out.left == 100 && out.right == 100);
  // Restart with monotonically increasing time below.
  sync = RearSpeedSync{};
  out = sync.update(3001, left, right, 0, 0, true);
  assert(out.left == 0 && out.right == 0);
  out = sync.update(3002, left, right, -100, -100, true);
  assert(out.left == -100 && out.right == -100);
  out = sync.update(3003, left, right, 100, 100, false);
  assert(out.left == 0 && out.right == 0);
  out = sync.update(3004, left, right, 100, 100, true);
  assert(out.left == 100 && out.right == 100);
  for (auto pair : {RearSpeedSync::Output{50, 100}, {100, -100}, {0, 100}, {19, 19}}) {
    out = sync.update(3005, left, right, pair.left, pair.right, true);
    assert(out.left == pair.left && out.right == pair.right);
  }

  // No feedback from either encoder: no invented speed / increasing throttle.
  for (uint32_t now = 4000; now < 6000; now += 20) {
    out = sync.update(now, left, right, 80, 80, true);
    assert(out.left == 80 && out.right == 80);
  }
  // Once feedback stops, a formerly boosted wheel returns to at most request.
  RearSpeedSync stale;
  left = right = 0;
  for (uint32_t now = 0; now <= 800; now += 20) {
    left += 20;
    right += 10;
    out = stale.update(now, left, right, 100, 100, true);
  }
  assert(out.right > 100);
  for (uint32_t now = 820; now <= 1200; now += 20) {
    left += 20;
    out = stale.update(now, left, right, 100, 100, true);
  }
  assert(out.left == 100 && out.right == 100);
  assert(stale.state() == RearSpeedSync::FEEDBACK_UNAVAILABLE);
  for (uint32_t now = 1220; now < 1800; now += 20) {
    left += 20; right += 10;
    out = stale.update(now, left, right, 100, 100, true);
    assert(out.left == 100 && out.right == 100); // Dropout fallback is latched for this run.
  }
  stale.update(1800, left, right, 0, 0, true);
  for (uint32_t now = 1820; now <= 2300; now += 20) {
    left += 20; right += 10;
    out = stale.update(now, left, right, 100, 100, true);
  }
  assert(out.left < 100 && stale.state() == RearSpeedSync::REGULATING);
  out = stale.update(2320, left + 100000, right, 100, 100, true);
  assert(out.left == 100 && out.right == 100);
  // Both signed encoder rollover and millis rollover, with identical speeds.
  RearSpeedSync rollover;
  for (uint32_t i = 0; i < 100; ++i) {
    const int32_t c = static_cast<int32_t>(0x7fffff00U + i * 20);
    out = rollover.update(0xffffff00U + i * 20, c, c, 100, 100, true);
    assert(out.left == 100 && out.right == 100);
  }
  // Clamp malformed commands before abs/sign operations.
  out = sync.update(6100, left, right, std::numeric_limits<int16_t>::min(),
                    std::numeric_limits<int16_t>::max(), true);
  assert(out.left == -1000 && out.right == 1000);
}

int main() {
  for (int magnitude : {40, 80, 150, 400, 900, 1000}) {
    for (int sign : {-1, 1}) {
      simulate(sign * magnitude, false);
      simulate(sign * magnitude, true);
    }
  }
  limitsAndReset();
  std::puts("Rear speed synchronization: convergence and limits passed");
}
