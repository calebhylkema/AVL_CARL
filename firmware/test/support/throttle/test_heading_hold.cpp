#include "../../../master/src/heading_hold.h"
#include "../../../throttle/src/rear_speed_sync.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace carl;

static ThrottleCmd straight(int16_t value) { return {value, value, value, value}; }
static bool equal(const ThrottleCmd& a, const ThrottleCmd& b) {
  return a.fl == b.fl && a.fr == b.fr && a.rl == b.rl && a.rr == b.rr;
}
static Bno08xHeadingSample sample(float yaw, float rate, uint32_t now, bool upsideDown = false) {
  Bno08xHeadingSample s = {};
  // Fixed 0.7 rad mounting offset deliberately does not align X to the vehicle.
  const float a = (yaw + 0.7f) * 0.5f;
  if (upsideDown) {
    s.quaternion[0] = (int16_t)roundf(cosf(a) * 16384);
    s.quaternion[1] = (int16_t)roundf(sinf(a) * 16384);
  } else {
    s.quaternion[2] = (int16_t)roundf(sinf(a) * 16384);
    s.quaternion[3] = (int16_t)roundf(cosf(a) * 16384);
  }
  s.gyro[2] = (int16_t)roundf(rate * 512 * (upsideDown ? -1 : 1));
  s.accuracy = s.gyro_accuracy = 3;
  s.orientation_ms = s.gyro_ms = now;
  return s;
}
static void warm(HeadingHold& h, float yaw = 0, bool down = false, uint32_t base = 0) {
  for (uint32_t t = 0; t <= 600; t += 20) {
    assert(h.accept(sample(yaw, 0, base + t, down), base + t));
    assert(equal(h.update(base + t, straight(0), true), straight(0)));
  }
  assert(h.ready(base + 600));
}

static void behavior() {
  for (bool down : {false, true}) {
    for (int sign : {-1, 1}) {
      HeadingHold h;
      warm(h, 0, down);
      const auto command = straight(sign * 200);
      assert(equal(h.update(600, command, true), command));
      for (uint32_t now = 620; now <= 800; now += 20) {
        assert(h.accept(sample(0.05f, 0, now, down), now));
        const auto out = h.update(now, command, true);
        assert(out.fl > out.fr); // Positive yaw drift -> negative yaw correction.
        assert(out.fl == out.rl && out.fr == out.rr);
        assert(std::abs(out.fl - command.fl) <= 50);
        assert(out.fl * sign > 0 && out.fr * sign > 0);
      }
      h.update(800, straight(sign * 100), true); // Magnitude change keeps reference.
      assert(std::abs(h.error() + 0.05f) < 0.001f);
      // Manual steering passes through, including front/rear-independent tests.
      const ThrottleCmd turn = {200, 100, 150, 50};
      assert(equal(h.update(801, turn, true), turn));
      assert(h.state() == HeadingHold::MANUAL_TURN);
      assert(equal(h.update(802, command, true), command)); // New heading reference.
      assert(equal(h.update(803, straight(0), true), straight(0)));
      assert(equal(h.update(804, command, true), command));
      assert(equal(h.update(805, straight(-sign * 200), true), straight(-sign * 200)));
      assert(equal(h.update(806, command, false), straight(0)));
      assert(h.correction() == 0);
    }
  }
  HeadingHold h;
  warm(h);
  h.update(600, straight(200), true);
  assert(equal(h.update(701, straight(200), true), straight(200))); // Sensor stale.
  assert(h.state() == HeadingHold::WAIT_IMU);
  for (uint32_t t = 720; t <= 1320; t += 20) h.accept(sample(0.1f, 0, t), t);
  assert(equal(h.update(1320, straight(200), true), straight(200)));
  assert(h.state() == HeadingHold::REARM); // No automatic mid-run reengagement.
  h.update(1321, straight(0), true);
  h.update(1322, straight(200), true);
  assert(h.state() == HeadingHold::HOLDING);
  auto bad = sample(0.1f, 0, 1340); bad.accuracy = 0;
  assert(!h.accept(bad, 1340));
  assert(equal(h.update(1340, straight(200), true), straight(200)));
  assert(h.state() == HeadingHold::REARM);

  warm(h, 0, false, 2000);
  h.update(2600, straight(200), true);
  assert(!h.accept(sample(1.0f, 0, 2620), 2620)); // Sensor reference jump.
  bad = sample(0, 0, 2640); bad.quaternion[3] = 0;
  assert(!h.accept(bad, 2640)); // Non-unit/zero quaternion.
  bad = sample(0, 0, 2660); bad.gyro_ms -= 101;
  assert(!h.accept(bad, 2660)); // Fresh quaternion cannot hide old gyro.
  bad = sample(0, 0, 2680);
  bad.quaternion[0] = 6270; bad.quaternion[1] = bad.quaternion[2] = 0;
  bad.quaternion[3] = 15137;
  assert(!h.accept(bad, 2680)); // Tilt beyond the flat-mount operating envelope.

  HeadingHold wrapped;
  const uint32_t base = UINT32_MAX - 620;
  warm(wrapped, 2.43f, false, base); // Mounted yaw near +pi.
  wrapped.update(base + 600, straight(100), true);
  assert(wrapped.accept(sample(2.46f, 0, base + 640), base + 640)); // Cross +pi.
  auto out = wrapped.update(base + 640, straight(100), true);
  assert(out.fl > out.fr && wrapped.error() > -0.04f && wrapped.error() < 0);
  // Quaternion q and -q encode the same orientation.
  auto negated = sample(2.46f, 0, base + 660);
  for (auto& q : negated.quaternion) q = -q;
  assert(wrapped.accept(negated, base + 660));
  wrapped.update(base + 660, straight(100), true);
  assert(wrapped.state() == HeadingHold::HOLDING);

  HeadingHoldConfig cfg; cfg.enabled = false;
  HeadingHold disabled(cfg);
  assert(equal(disabled.update(0, straight(100), true), straight(100)));
  assert(equal(disabled.update(1, straight(100), false), straight(0)));

  // Saturation, low-throttle authority and changing input magnitude must not
  // reverse a wheel, cross the input deadband, or exceed full PWM.
  for (int magnitude : {20, 21, 40, 200, 990, 1000}) {
    for (int sign : {-1, 1}) {
      HeadingHold limited;
      warm(limited);
      limited.update(600, straight(sign * magnitude), true);
      for (uint32_t t = 620; t <= 2000; t += 20) {
        const float angle = std::fmin((t - 600) * 0.001f, 0.4f);
        const float rate = t < 1000 ? 1.0f : 0.0f;
        assert(limited.accept(sample(angle, rate, t), t));
        const auto c = limited.update(t, straight(sign * magnitude), true);
        assert(c.fl * sign >= THROTTLE_DEADBAND && c.fr * sign >= THROTTLE_DEADBAND);
        assert(std::abs(c.fl) <= 1000 && std::abs(c.fr) <= 1000);
        assert(std::abs(limited.correction()) <= 100);
        assert(std::abs(limited.correction()) <= magnitude / 4 + 1);
      }
    }
  }
}

// Closed-loop planar yaw model, including the existing rear controller. Unequal
// heading commands must bypass its equal-RPM objective. Disturbance represents
// motor/ground asymmetry; these values are a test model, not vehicle calibration.
static void simulate(int16_t throttle, bool down) {
  HeadingHold h;
  RearSpeedSync sync;
  warm(h, 0, down);
  float yaw = 0, rate = 0;
  double counts[2] = {};
  float maxError = 0, finalError = 0;
  const int sign = throttle > 0 ? 1 : -1;
  for (uint32_t t = 600; t <= 20600; t += 20) {
    assert(h.accept(sample(yaw, rate, t, down), t));
    const auto c = h.update(t, straight(throttle), true);
    assert(h.state() == HeadingHold::HOLDING);
    const auto rear = sync.update(t, (int32_t)counts[0], (int32_t)counts[1], c.rl, c.rr, true);
    const float left = (c.fl + rear.left) * 0.5f;
    const float right = (c.fr + rear.right) * 0.5f;
    const float disturbance = sign * (t < 10600 ? 0.035f : -0.025f);
    const float wantedRate = (right - left) * 0.0025f + disturbance;
    rate += (wantedRate - rate) * 0.02f / 0.15f;
    yaw += rate * 0.02f;
    counts[0] += rear.left * 0.15 * 2400 / 60 * 0.02;
    counts[1] += rear.right * 0.15 * 2400 / 60 * 0.02;
    maxError = std::fmax(maxError, std::abs(yaw));
    if (t >= 19600) finalError = std::fmax(finalError, std::abs(yaw));
  }
  std::printf("Heading model throttle=%d upside-down=%d peak=%.2f deg final=%.2f deg\n",
              throttle, down, maxError * 57.29578f, finalError * 57.29578f);
  assert(maxError < 0.12f && finalError < 0.035f);
}

int main() {
  behavior();
  for (int throttle : {-900, -200, -40, 40, 200, 900}) {
    simulate(throttle, false);
    simulate(throttle, true);
  }
  std::puts("Heading hold: yaw sign, mounting, wrap, stale data, overrides and closed-loop convergence passed");
}
