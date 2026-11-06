#include "fixture.h"
#include <cmath>

static void feedback(uint32_t t, int32_t count = 0, bool idle = true) {
  CAN_message_t f;
  f.id = CANID_ENCODER_FB;
  f.len = pack_encoder_fb(f.buf, {count, -count});
  Can1.rx.push_back(f);
  f.id = CANID_THROTTLE_STATUS;
  f.len = pack_throttle_status(f.buf, {uint8_t(idle ? TST_IDLE : TST_DRIVING), TF_NONE,
                                      int16_t(idle ? 0 : 200), int16_t(idle ? 0 : 200)});
  if (t % 50 == 0) Can1.rx.push_back(f);
}
static void sensor(float rate = 0.0f) {
  // Downward-facing installed sensor with measured-order X/Y/Z gyro biases.
  data(0x3b, -407); data(0x3d, -221); data(0x3f, -17788);
  const float length = sqrtf(407.0f * 407 + 221.0f * 221 + 17788.0f * 17788);
  data(0x43, int16_t(roundf(963 - rate * 407 / length / MpuHeading::GYRO_SCALE)));
  data(0x45, int16_t(roundf(-1247 - rate * 221 / length / MpuHeading::GYRO_SCALE)));
  data(0x47, int16_t(roundf(72 - rate * 17788 / length / MpuHeading::GYRO_SCALE)));
  data(0x41, 0);
  ImuSpi.registers[0x3a] = 1;
}
static bool equal(ThrottleCmd a, ThrottleCmd b) {
  return a.fl == b.fl && a.fr == b.fr && a.rl == b.rl && a.rr == b.rr;
}
static void frameCommand(ThrottleCmd c) {
  uint8_t p[8], f[WIRE_MAX_FRAME];
  const auto n = pack_throttle_cmd(p, c);
  const auto size = wire_frame(f, CANID_THROTTLE_CMD, p, n);
  Serial.rx.insert(Serial.rx.end(), f, f + size);
}
static void run(uint32_t base, uint32_t first, uint32_t last, ThrottleCmd c,
                float rate = 0, bool evidence = true, bool idle = true) {
  for (uint32_t elapsed = first; elapsed <= last; ++elapsed) {
    const uint32_t t = base + elapsed;
    if (elapsed % 10 == 0) sensor(rate);
    if (elapsed % 20 == 0) {
      frameCommand(c);
      if (evidence) feedback(elapsed, 0, idle);
    }
    tick(t);
  }
}
static ThrottleCmd straight(int16_t c) { return {c, c, c, c}; }

static void behavior(uint32_t base, int direction) {
  reset(0x71, base); configure(base);
  run(base, 312, 4500, {});
  assert(mpuHeading.ready(base + 4500) && headingHold.ready(base + 4500));
  assert(fabsf(mpuHeading.bias(0) - 963 * MpuHeading::GYRO_SCALE) < 0.00001f);
  assert(fabsf(mpuHeading.bias(1) + 1247 * MpuHeading::GYRO_SCALE) < 0.00001f);
  run(base, 4501, 4800, straight(direction * 200), 0, true, false);
  assert(equal(output(), straight(direction * 200)));
  run(base, 4801, 5300, straight(direction * 200), 0.2f, true, false);
  const auto corrected = output();
  assert(corrected.fl > corrected.fr && corrected.fl == corrected.rl && corrected.fr == corrected.rr);
  assert(std::abs(corrected.fl - direction * 200) <= 50);
  assert(currentCmd.fl == direction * 200 && currentCmd.fr == direction * 200);
  assert(headingHold.state() == HeadingHold::HOLDING);
  // Deliberate steering, pivots and individual-wheel commands retain all values.
  uint32_t manualMs = 5320;
  for (auto c : {ThrottleCmd{120, 80, 110, 70}, {200, -200, 200, -200}, {0, 0, 70, 0}}) {
    sensor(); frameCommand(c); tick(base + manualMs); assert(equal(output(), c));
    manualMs += 20;
  }
  // Reacquire, then assert actual bus e-stop during nonzero correction.
  run(base, 5361, 5600, straight(direction * 200), 0, true, false);
  run(base, 5601, 5900, straight(direction * 200), 0.2f, true, false);
  assert(headingHold.correction() < 0);
  CAN_message_t stop; stop.id = CANID_ESTOP; stop.len = 1; stop.buf[0] = ESTOP_OPERATOR;
  Can1.rx.push_back(stop); tick(base + 5920);
  assert(equal(output(), {}) && estopLatched && headingHold.correction() == 0);
  const uint8_t clear = 0; handleWireFrame(CANID_ESTOP, &clear, 1);
  tick(base + 5940); assert(equal(output(), {}));
  // A missing sample drops correction after 30 ms, before the driver timeout.
  run(base, 5941, 10000, {});
  assert(headingHold.ready(base + 10000));
  run(base, 10001, 10300, straight(200), 0.2f, true, false);
  assert(headingHold.correction() < 0);
  for (uint32_t elapsed = 10301; elapsed <= 10500; ++elapsed) {
    if (elapsed % 20 == 0) frameCommand(straight(200));
    tick(base + elapsed);
  }
  assert(mpu.stage() == MpuSpiTest::READY && equal(output(), straight(200)));
  assert(headingHold.correction() == 0);
  // Recovery while driving never enables correction without neutral calibration.
  run(base, 10501, 14500, straight(200), 0.2f, true, false);
  assert(equal(output(), straight(200)) && !headingHold.ready(base + 14500));
  run(base, 14501, 19000, {});
  assert(headingHold.ready(base + 19000));
  run(base, 19001, 19300, straight(-200), 0.2f, true, false);
  assert(headingHold.correction() < 0 && output().fl > output().fr);
  // IMU traffic cannot feed the teleop watchdog.
  for (uint32_t elapsed = 19301; elapsed <= 19700; ++elapsed) {
    if (elapsed % 10 == 0) sensor();
    tick(base + elapsed);
  }
  assert(equal(output(), {}) && currentCmd.fl == 0 && headingHold.correction() == 0);
}

static void calibration_guards() {
  for (unsigned scenario = 0; scenario < 4; ++scenario) {
    reset(0x71); configure();
    for (uint32_t t = 312; t <= 5000; ++t) {
      if (t % 10 == 0) sensor();
      if (t % 20 == 0) {
        frameCommand(scenario == 3 ? straight(200) : straight(0));
        if (scenario != 0) feedback(t, scenario == 1 ? int32_t(t) : 0, scenario != 2);
      }
      tick(t);
    }
    assert(!headingHold.ready(5000) && !mpuHeading.ready(5000));
  }
  // Stationary evidence expires even when encoder counts used to be unchanged.
  reset(0x71); configure();
  run(0, 312, 1500, {});
  run(0, 1501, 6000, {}, 0, false);
  assert(!headingHold.ready(6000));
}

static void endurance() {
  reset(0x71); configure();
  float rate = 0, yaw = 0;
  uint32_t lastCommand = 301, lastBeat = 301;
  unsigned commands = 0, beats = 0, controlled = 0;
  ThrottleCmd lastOutput = {};
  // Ten minutes of repeated rest/calibration and forward/reverse straight runs
  // through the actual SPI driver, serial parser, master loop and CAN writer.
  for (uint32_t t = 312; t <= 600311; ++t) {
    const uint32_t phase = t % 30000;
    const bool neutral = phase < 5000;
    const int direction = (t / 30000) % 2 ? -1 : 1;
    const auto desired = straight(neutral ? 0 : direction * 200);
    if (t % 10 == 0) {
      const auto c = lastOutput;
      const float targetRate = neutral ? 0.0f : 0.003f * (c.fr - c.fl) + 0.025f;
      rate += (targetRate - rate) * 0.01f / 0.2f;
      yaw += rate * 0.01f;
      sensor(rate);
    }
    if (t % 20 == 0) { frameCommand(desired); feedback(t, 0, neutral); }
    const auto before = Can1.sent.size(); tick(t);
    for (size_t index = before; index < Can1.sent.size(); ++index) {
      const auto& f = Can1.sent[index];
      if (f.id == CANID_THROTTLE_CMD) {
        assert(t - lastCommand == 20); lastCommand = t; ++commands;
        ThrottleCmd c; assert(unpack_throttle_cmd(f.buf, f.len, &c));
        lastOutput = c;
        if (neutral) assert(equal(c, {}));
        else {
          assert(c.fl * direction >= 150 && c.fr * direction >= 150);
          assert(c.fl * direction <= 250 && c.fr * direction <= 250);
          if (phase > 6000) {
            assert(headingHold.state() == HeadingHold::HOLDING);
            ++controlled;
          }
        }
      } else if (f.id == CANID_HEARTBEAT) {
        assert(t - lastBeat == 50); lastBeat = t; ++beats;
      } else assert(false);
    }
    if (phase == 29999) {
      assert(fabsf(headingHold.error()) < 0.04f);
      assert(fabsf(yaw) < 0.06f);
    }
    if (phase == 4999) yaw = 0;
    // Keep host memory bounded; retaining old transmissions is not MCU work.
    if (t % 1000 == 0) { Serial.tx.clear(); Can1.sent.clear(); }
  }
  assert(commands == 30000 && beats == 12000 && controlled > 23000 && mpu.errorCount() == 0);
  std::puts("Production MPU heading: ten simulated minutes of calibrated forward/reverse control, 30,000 commands and 12,000 on-time heartbeats passed");
}

int main() {
  calibration_guards();
  for (auto base : {0U, UINT32_MAX - 4900U})
    for (int direction : {-1, 1}) behavior(base, direction);
  endurance();
  std::puts("Production MPU heading: stationary evidence, bias/sign, framed commands, forward/reverse, turns, dropout/recovery, e-stop, watchdog and timer wrap passed");
}
