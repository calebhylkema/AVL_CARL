// Regression for y=+/-1, |x|<=0.05: the server sends equal +/-400 commands.
#include "../../../throttle/src/main.cpp"
#include <cstdio>

static void tick(const ThrottleCmd& c) {
  CAN_message_t f;
  f.id = CANID_HEARTBEAT;
  f.len = pack_heartbeat(f.buf, Heartbeat{1, HB_FLAG_OK, MODE_TELEOP});
  handleFrame(f);
  f.id = CANID_THROTTLE_CMD;
  f.len = pack_throttle_cmd(f.buf, c);
  handleFrame(f);
  loop();
}

int main() {
  setup();
  for (int sign : {-1, 1}) {
    for (int rightCounts : {0, 2, 20, 80}) {
      rearSpeedSync = RearSpeedSync{};
      encRL.count = encRR.count = 0;
      const int16_t base = sign * 400;
      for (int i = 0; i < 300; ++i) {
        fake_io::now += 20;
        encRL.count += sign * 20;
        encRR.count -= sign * rightCounts;
        tick({base, base, base, base});
        // Even persistently mismatched/missing feedback cannot selectively
        // zero a wheel, flip direction, or reduce it below 75% of request.
        assert(appliedCmd.rl * sign >= 300 && appliedCmd.rr * sign >= 300);
        assert(appliedCmd.fl == base && appliedCmd.fr == base);
      }
      if (!rightCounts) assert(appliedCmd.rl == base && appliedCmd.rr == base);
      // Just outside x=0.05: smallest quantized unequal wheel request.
      tick({base, int16_t(base - sign), base, int16_t(base - sign)});
      assert(appliedCmd.rl == base && appliedCmd.rr == base - sign);
      tick({base, base, base, base});
      assert(appliedCmd.rl == base && appliedCmd.rr == base);
      tick({0, 0, 0, 0});
      for (auto& motor : kMotor) {
        assert(fake_io::pwm[motor.rpwm] == 0 && fake_io::pwm[motor.lpwm] == 0);
      }
    }
  }
  std::puts("Joystick center-band firmware regression passed in forward and reverse");
}
