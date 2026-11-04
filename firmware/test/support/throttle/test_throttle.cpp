// Compile the actual throttle firmware against GPIO/CAN test doubles.
#include "../../../throttle/src/main.cpp"
#include <cstdio>

static void heartbeat(uint8_t flags) {
  CAN_message_t frame;
  frame.id = CANID_HEARTBEAT;
  frame.len = pack_heartbeat(frame.buf, Heartbeat{1, flags, MODE_IDLE});
  handleFrame(frame);
}
static void command(ThrottleCmd value) {
  CAN_message_t frame;
  frame.id = CANID_THROTTLE_CMD;
  frame.len = pack_throttle_cmd(frame.buf, value);
  handleFrame(frame);
}
static void disabled() {
  for (const auto& motor : kMotor) {
    assert(fake_io::level[motor.en] == LOW);
    assert(fake_io::pwm[motor.rpwm] == 0);
    assert(fake_io::pwm[motor.lpwm] == 0);
  }
}
static void statusZero(uint8_t state, uint8_t faults) {
  sendStatus(state, faults);
  const auto& frame = can1.sent.back();
  ThrottleStatus status;
  assert(unpack_throttle_status(frame.buf, frame.len, &status));
  assert(status.applied_left == 0 && status.applied_right == 0);
}
int main() {
  setup();
  disabled();
  assert(fake_io::resolution == 12);
  for (const auto& motor : kMotor) {
    assert(fake_io::mode[motor.en] == OUTPUT);
    assert(fake_io::frequency[motor.rpwm] == 20000);
    assert(fake_io::frequency[motor.lpwm] == 20000);
  }
  assert(kMotor[1].rpwm == 4 && kMotor[1].lpwm == 5 && kMotor[1].en == 11);
  fake_io::now = 1;
  heartbeat(HB_FLAG_OK);
  for (int sign : {1, -1}) {
    command(ThrottleCmd{int16_t(250 * sign), int16_t(250 * sign),
                        int16_t(250 * sign), int16_t(250 * sign)});
    loop();
    for (uint8_t i = 0; i < 4; ++i) {
      const auto& motor = kMotor[i];
      const bool forwardPwm = sign * kMotorDir[i] > 0;
      assert(fake_io::level[motor.en] == HIGH);
      assert(fake_io::pwm[motor.rpwm] == (forwardPwm ? 1023 : 0));
      assert(fake_io::pwm[motor.lpwm] == (forwardPwm ? 0 : 1023));
    }
  }
  // Prove the FR bytes reach FR pins, independently of every other wheel.
  command(ThrottleCmd{0, 250, 0, 0});
  loop();
  assert(cmd.fr == 250);
  assert(fake_io::level[11] == HIGH);
  assert(fake_io::pwm[4] == 0 && fake_io::pwm[5] == 1023);
  for (uint8_t i : {0, 2, 3}) {
    assert(fake_io::pwm[kMotor[i].rpwm] == 0);
    assert(fake_io::pwm[kMotor[i].lpwm] == 0);
  }
  command(ThrottleCmd{0, -250, 0, 0});
  loop();
  assert(fake_io::pwm[4] == 1023 && fake_io::pwm[5] == 0);

  // Exercise the real loop, pins, and status with unequal rear feedback.
  // Opposite encoder count signs reflect a mirrored physical installation.
  command(ThrottleCmd{100, 100, 100, 100});
  loop();
  for (int tick = 0; tick < 30; ++tick) {
    fake_io::now += 20;
    encRL.count += 12;
    encRR.count -= 6;
    heartbeat(HB_FLAG_OK);
    command(ThrottleCmd{100, 100, 100, 100});
    loop();
  }
  assert(appliedCmd.fl == 100 && appliedCmd.fr == 100);
  assert(appliedCmd.rl < 100 && appliedCmd.rr > 100 && appliedCmd.rr <= 150);
  assert(fake_io::pwm[kMotor[0].rpwm] == 409);
  assert(fake_io::pwm[kMotor[1].lpwm] == 409);
  assert(fake_io::pwm[kMotor[2].rpwm] < 409);
  assert(fake_io::pwm[kMotor[3].lpwm] > 409);
  sendStatus(TST_DRIVING, TF_NONE);
  ThrottleStatus regulatedStatus;
  const auto& regulatedFrame = can1.sent.back();
  assert(unpack_throttle_status(regulatedFrame.buf, regulatedFrame.len, &regulatedStatus));
  assert(regulatedStatus.applied_left == (100 + appliedCmd.rl) / 2);
  assert(regulatedStatus.applied_right == (100 + appliedCmd.rr) / 2);
  sendEncoders();
  EncoderFb feedback;
  const auto& feedbackFrame = can1.sent.back();
  assert(unpack_encoder_fb(feedbackFrame.buf, feedbackFrame.len, &feedback));
  assert(feedback.left_count == encRL.count && feedback.right_count == encRR.count);

  // A neutral command must act within the same millisecond/control tick.
  command(ThrottleCmd{0, 0, 0, 0});
  loop();
  for (const auto& motor : kMotor) {
    assert(fake_io::pwm[motor.rpwm] == 0 && fake_io::pwm[motor.lpwm] == 0);
  }
  command(ThrottleCmd{-100, -100, -100, -100});
  loop();
  assert(appliedCmd.rl == -100 && appliedCmd.rr == -100);
  for (int tick = 0; tick < 20; ++tick) {
    fake_io::now += 20;
    encRL.count -= 12;
    encRR.count += 6;
    heartbeat(HB_FLAG_OK);
    command(ThrottleCmd{-100, -100, -100, -100});
    loop();
  }
  assert(appliedCmd.rl > -100 && appliedCmd.rr < -100);
  CAN_message_t stop;
  stop.id = CANID_ESTOP;
  stop.len = 1;
  stop.buf[0] = ESTOP_OPERATOR;
  handleFrame(stop);
  loop();
  disabled();
  statusZero(TST_ESTOP, TF_ESTOP); // Retained nonzero commands are not applied.
  command(ThrottleCmd{0, 0, 0, 0});
  heartbeat(HB_FLAG_OK);
  loop();
  assert(!estopLatched);
  command(ThrottleCmd{250, 250, 250, 250});
  loop();
  fake_io::now += HEARTBEAT_TIMEOUT_MS;
  loop();
  disabled();
  statusZero(TST_SAFE_STOP, TF_HEARTBEAT_LOST);
  heartbeat(HB_FLAG_OK);
  loop();
  disabled(); // Heartbeat alone does not reapply an expired command.
  command(ThrottleCmd{250, 250, 250, 250});
  loop();
  assert(appliedCmd.rl == 250 && appliedCmd.rr == 250);
  for (uint32_t age = 20; age <= THROTTLE_COMMAND_TIMEOUT_MS + 20; age += 20) {
    fake_io::now += 20;
    heartbeat(HB_FLAG_OK);
    loop();
  }
  disabled();
  statusZero(TST_SAFE_STOP, TF_COMMAND_LOST);
  command(ThrottleCmd{-32768, 32767, -32768, 32767});
  loop();
  assert(cmd.fl == -1000 && cmd.fr == 1000 && cmd.rl == -1000 && cmd.rr == 1000);
  // If CAN draining crosses a millisecond boundary, freshly fed watchdogs
  // must not underflow against a timestamp captured before their feed.
  CAN_message_t queuedHb, queuedCmd;
  queuedHb.id = CANID_HEARTBEAT;
  queuedHb.len = pack_heartbeat(queuedHb.buf, Heartbeat{1, HB_FLAG_OK, MODE_TELEOP});
  queuedCmd.id = CANID_THROTTLE_CMD;
  queuedCmd.len = pack_throttle_cmd(queuedCmd.buf, {250, 250, 250, 250});
  can1.incoming.push_back(queuedHb); can1.incoming.push_back(queuedCmd);
  can1.advanceOnRead = 1;
  loop();
  assert(appliedCmd.fl == 250 && appliedCmd.fr == 250);
  assert(appliedCmd.rl == 250 && appliedCmd.rr == 250);
  std::puts("Throttle firmware: outputs/directions, rear regulation, telemetry, neutral, e-stop and heartbeat loss passed");
}
