#include "../../../master/src/mpu_heading.h"
#include "../../../master/src/heading_hold.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace carl;

static bool sample(MpuHeading& m, uint32_t t, float rate = 0, bool stationary = true,
                   bool down = true, int16_t temp = 0) {
  const int16_t accel[] = {-407, -221, int16_t(down ? -17788 : 17788)};
  const float length = sqrtf(407.0f * 407 + 221.0f * 221 + 17788.0f * 17788);
  const int16_t bias[] = {963, -1247, 72};
  int16_t gyro[3];
  for (unsigned i = 0; i < 3; ++i)
    gyro[i] = int16_t(roundf(bias[i] + rate * accel[i] / length / MpuHeading::GYRO_SCALE));
  return m.accept(accel, gyro, temp, t, stationary);
}
static void warm(MpuHeading& m, HeadingHold& h, bool down = true, uint32_t base = 0) {
  for (uint32_t elapsed = 0; elapsed <= 4000; elapsed += 10) {
    if (sample(m, base + elapsed, 0, true, down))
      assert(h.acceptYaw(m.yaw(), m.rate(), base + elapsed, base + elapsed));
    else h.invalidate();
    h.update(base + elapsed, {}, true);
  }
  assert(m.ready(base + 4000) && h.ready(base + 4000));
}
static void guards() {
  MpuHeading m; HeadingHold h;
  for (uint32_t t = 0; t <= 5000; t += 10) sample(m, t, 0, false);
  assert(!m.ready(5000));
  warm(m, h);
  assert(fabsf(m.bias(2) - 72 * MpuHeading::GYRO_SCALE) < 1e-6f);
  assert(!sample(m, 4100, 0, false) && !m.ready(4100)); // missing data
  warm(m, h);
  assert(!sample(m, 4010, 0, false, true, 1700)); // thermal drift
  warm(m, h);
  const int16_t zeros[3] = {}, flat[] = {0, 0, -16384}, tilt[] = {12000, 0, -12000};
  assert(!m.accept(zeros, zeros, 0, 4010, false)); // free fall/disconnection
  warm(m, h); assert(!m.accept(tilt, zeros, 0, 4010, false));
  warm(m, h);
  const int16_t rail[] = {0, 0, INT16_MIN};
  assert(!m.accept(flat, rail, 0, 4010, false));
  warm(m, h); assert(!sample(m, 4010, 3.1f, false));
  warm(m, h);
  assert(m.allowRun(4000, true));
  for (uint32_t t = 4010; t < 34000; t += 10) {
    assert(sample(m, t, 0, false)); assert(m.allowRun(t, true));
  }
  assert(!m.allowRun(34000, true) && m.state() == MpuHeading::RUN_EXPIRED);
  assert(!sample(m, 34010, 0, false));
  warm(m, h); assert(m.allowRun(4000, true));
  assert(!sample(m, 4010)); // neutral after a run starts fresh calibration
  warm(m, h);
  for (uint32_t t = 4010; t < 63000; t += 10) assert(sample(m, t));
  assert(!sample(m, 63000) && !m.ready(63000)); // calibration age is bounded
  assert(!h.acceptYaw(NAN, 0, 0, 0));
  assert(!h.acceptYaw(0, INFINITY, 0, 0));
  assert(!h.acceptYaw(0, 0, 0, 101));

  // Unstable gyro windows and rolling neutral never establish calibration.
  m = MpuHeading();
  for (uint32_t t = 0; t <= 10000; t += 10) sample(m, t, (t % 20) ? 0.03f : -0.03f);
  assert(!m.ready(10000));
}

static void model(bool down, int command, uint32_t base) {
  MpuHeading m; HeadingHold h; warm(m, h, down, base);
  const ThrottleCmd c = {int16_t(command), int16_t(command), int16_t(command), int16_t(command)};
  h.update(base + 4000, c, true);
  float yaw = 0, rate = 0, peak = 0;
  for (uint32_t t = 4010; t <= 24000; t += 10) {
    assert(sample(m, base + t, rate, false, down));
    assert(h.acceptYaw(m.yaw(), m.rate(), base + t, base + t));
    assert(m.allowRun(base + t, true));
    const auto out = h.update(base + t, c, true);
    assert((command > 0 ? out.fl > 0 && out.fr > 0 : out.fl < 0 && out.fr < 0));
    assert(std::abs(out.fl) <= 1000 && std::abs(out.fr) <= 1000);
    assert(std::abs(out.fl - command) <= std::min(100.0f, std::abs(command) * 0.25f));
    const float targetRate = 0.003f * (out.fr - out.fl) + 0.025f;
    rate += (targetRate - rate) * 0.01f / 0.2f;
    yaw += rate * 0.01f;
    peak = std::max(peak, fabsf(yaw));
  }
  assert(peak < 0.055f && fabsf(yaw) < 0.03f);
  std::printf("MPU heading model: command=%d down=%d peak=%.2f final=%.2f deg\n",
              command, down, peak * 57.29578f, yaw * 57.29578f);
}
int main() {
  guards();
  for (bool down : {false, true})
    for (int c : {-900, -200, -40, 40, 200, 900}) model(down, c, UINT32_MAX - 10000U);
  std::puts("MPU estimator: calibration, mounting, integration, guards, drift bounds and closed-loop convergence passed");
}
