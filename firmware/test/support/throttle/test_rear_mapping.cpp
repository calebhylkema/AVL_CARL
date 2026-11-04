// Exercise actual motor GPIO -> physical motor model -> encoder PIN -> firmware.
// Unlike tests assigning encRL.count directly, this catches crossed pin maps.
#include "../../../throttle/src/main.cpp"
#include <cmath>
#include <cstdio>

static void tick(int16_t throttle) {
  CAN_message_t frame;
  frame.id = CANID_HEARTBEAT;
  frame.len = pack_heartbeat(frame.buf, Heartbeat{1, HB_FLAG_OK, MODE_TELEOP});
  handleFrame(frame);
  frame.id = CANID_THROTTLE_CMD;
  frame.len = pack_throttle_cmd(frame.buf, {throttle, throttle, throttle, throttle});
  handleFrame(frame);
  loop();
}

static void simulate(int16_t throttle, bool leftFaster = false) {
  tick(0);
  rearSpeedSync = RearSpeedSync{};
  fake_encoder::countByPinA[14] = fake_encoder::countByPinA[16] = 0;
  double rpmL = 0, rpmR = 0, countsL = 0, countsR = 0;
  double difference = 0, speed = 0;
  const int sign = throttle > 0 ? 1 : -1;
  for (unsigned elapsed = 0; elapsed <= 12000; elapsed += 20) {
    fake_io::now += 20;
    // Confirmed on CARL by hand rotation: physical LEFT encoder A=16/B=17;
    // the other pair A=14/B=15 is the physical RIGHT encoder.
    fake_encoder::countByPinA[16] = (int32_t)countsL;
    fake_encoder::countByPinA[14] = -(int32_t)countsR; // Mirrored A/B polarity.
    tick(throttle);
    // Physical motor pins are fixed in this test, not looked up from kMotor.
    const double dutyL = (fake_io::pwm[8] - fake_io::pwm[9]) * 1000.0 / PWM_MAX;
    const double dutyR = (fake_io::pwm[7] - fake_io::pwm[6]) * 1000.0 / PWM_MAX;
    assert(dutyL * sign > 0 && dutyR * sign > 0);
    const double targetL = sign * (leftFaster ? 0.27 : 0.20) *
                           std::fmax(0.0, std::abs(dutyL) - (leftFaster ? 10.0 : 30.0));
    const double targetR = sign * (leftFaster ? 0.20 : 0.27) *
                           std::fmax(0.0, std::abs(dutyR) - (leftFaster ? 30.0 : 10.0));
    rpmL += (targetL - rpmL) * 0.02 / 0.20;
    rpmR += (targetR - rpmR) * 0.02 / 0.20;
    countsL += rpmL * 2400.0 / 60.0 * 0.02;
    countsR += rpmR * 2400.0 / 60.0 * 0.02;
    if (elapsed >= 10000) {
      difference += std::abs(rpmL - rpmR);
      speed += (std::abs(rpmL) + std::abs(rpmR)) * 0.5;
    }
  }
  assert(speed > 100);
  std::printf("Physical pin model: cmd=%d left-faster=%d RPM=%.2f/%.2f error=%.2f%% outputs=%d/%d\n",
              throttle, leftFaster, rpmL, rpmR, difference / speed * 100,
              appliedCmd.rl, appliedCmd.rr);
  assert(difference / speed < 0.035);
}

static void telemetryMapping() {
  tick(0);
  fake_encoder::countByPinA[16] = 123;
  fake_encoder::countByPinA[14] = 0;
  sendEncoders();
  EncoderFb fb;
  assert(unpack_encoder_fb(can1.sent.back().buf, can1.sent.back().len, &fb));
  assert(fb.left_count == 123 && fb.right_count == 0);
  fake_encoder::countByPinA[16] = 0;
  fake_encoder::countByPinA[14] = -456;
  sendEncoders();
  assert(unpack_encoder_fb(can1.sent.back().buf, can1.sent.back().len, &fb));
  assert(fb.left_count == 0 && fb.right_count == -456);
  assert(encRL.pinA == 16 && encRL.pinB == 17);
  assert(encRR.pinA == 14 && encRR.pinB == 15);
}

int main() {
  setup();
  simulate(400); // Reported full-vertical joystick case first.
  for (int magnitude : {40, 80, 150, 400, 900, 1000}) {
    for (int sign : {-1, 1}) {
      simulate(sign * magnitude);
      simulate(sign * magnitude, true);
    }
  }
  telemetryMapping();
  std::puts("Physical encoder pin mapping, rear RPM convergence, and CAN labels passed");
}
