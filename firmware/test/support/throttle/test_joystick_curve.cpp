// Verify the joystick's quadratic, 75%-limited wire commands on actual firmware.
#include "../../../throttle/src/main.cpp"
#include <cstdio>

static void heartbeat() {
  CAN_message_t frame;
  frame.id = CANID_HEARTBEAT;
  frame.len = pack_heartbeat(frame.buf, Heartbeat{1, HB_FLAG_OK, MODE_TELEOP});
  handleFrame(frame);
}

static void command(ThrottleCmd wheels) {
  CAN_message_t frame;
  frame.id = CANID_THROTTLE_CMD;
  frame.len = pack_throttle_cmd(frame.buf, wheels);
  handleFrame(frame);
  loop();
}

static void motor(uint8_t index, int16_t expected) {
  const auto& pins = kMotor[index];
  const int16_t directed = expected * kMotorDir[index];
  const uint16_t duty = uint32_t(abs_i16(expected)) * PWM_MAX / THROTTLE_MAX;
  assert(fake_io::level[pins.en] == HIGH);
  assert(fake_io::pwm[pins.rpwm] == (directed > 0 ? duty : 0));
  assert(fake_io::pwm[pins.lpwm] == (directed < 0 ? duty : 0));
}

int main() {
  setup();
  fake_io::now = 1;
  heartbeat();
  // Rounded bridge outputs for 0%, 5.1%, 10%, 20%, 20.5%, 25%, 50%,
  // 75%, 95%, 100% joystick travel, including both 2% deadband edges.
  for (int sign : {-1, 1}) {
    for (int magnitude : {0, 2, 19, 20, 33, 168, 407, 673, 750}) {
      command({0, 0, 0, 0}); // Reset rear regulation between independent inputs.
      const int16_t request = sign * magnitude;
      command({request, request, request, request});
      const int16_t applied = magnitude < THROTTLE_DEADBAND ? 0 : request;
      for (uint8_t wheel = 0; wheel < 4; ++wheel) motor(wheel, applied);
      sendStatus(applied ? TST_DRIVING : TST_IDLE, TF_NONE);
      ThrottleStatus status;
      const auto& frame = can1.sent.back();
      assert(unpack_throttle_status(frame.buf, frame.len, &status));
      assert(status.applied_left == applied && status.applied_right == applied);
    }
    // Turning stays at 40%; full combined forward/turn stays within 75%.
    command({int16_t(sign * 400), int16_t(-sign * 400),
             int16_t(sign * 400), int16_t(-sign * 400)});
    motor(0, sign * 400); motor(1, -sign * 400);
    motor(2, sign * 400); motor(3, -sign * 400);
    command({int16_t(sign * 750), int16_t(sign * 228),
             int16_t(sign * 750), int16_t(sign * 228)});
    motor(0, sign * 750); motor(1, sign * 228);
    motor(2, sign * 750); motor(3, sign * 228);
    // Intermediate forward/turn can pivot; a rim diagonal stays below the cap.
    command({int16_t(sign * 358), int16_t(-sign * 21),
             int16_t(sign * 358), int16_t(-sign * 21)});
    motor(0, sign * 358); motor(1, -sign * 21);
    motor(2, sign * 358); motor(3, -sign * 21);
    command({int16_t(sign * 699), int16_t(sign * 236),
             int16_t(sign * 699), int16_t(sign * 236)});
    motor(0, sign * 699); motor(1, sign * 236);
    motor(2, sign * 699); motor(3, sign * 236);
  }
  // Releasing a full-throttle command clears PWM in the same control tick.
  command({750, 750, 750, 750});
  command({0, 0, 0, 0});
  for (uint8_t wheel = 0; wheel < 4; ++wheel) motor(wheel, 0);
  std::puts("Joystick curve: light through 75% throttle, reverse, turn, PWM, telemetry and release passed");
}
