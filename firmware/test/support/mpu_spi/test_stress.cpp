// Production master flags are supplied by run_host_regressions.py from platformio.ini.
#include "fixture.h"
#include <cstring>

static bool equal(ThrottleCmd a, ThrottleCmd b) {
  return a.fl == b.fl && a.fr == b.fr && a.rl == b.rl && a.rr == b.rr;
}
static void enqueue(uint16_t id, const uint8_t* p, uint8_t n, bool corrupt = false) {
  uint8_t f[WIRE_MAX_FRAME]; const auto size = wire_frame(f, id, p, n);
  if (corrupt) f[size - 1] ^= 0xff;
  Serial.rx.insert(Serial.rx.end(), f, f + size);
}
static void drive(ThrottleCmd c, bool corrupt = false) {
  uint8_t p[8]; const auto n = pack_throttle_cmd(p, c);
  enqueue(CANID_THROTTLE_CMD, p, n, corrupt);
}
static void sensor() {
  data(0x3b, -16384); data(0x3d, 123); data(0x3f, 16384);
  data(0x41, -32768); data(0x43, -131); data(0x45, 0); data(0x47, 32767);
  ImuSpi.registers[0x3a] = 1;
}
static void feedback(uint32_t now) {
  CAN_message_t f;
  if (now % 20 == 0) {
    f.id = CANID_STEERING_FB;
    f.len = pack_steering_feedback(f.buf, {SF_SAMPLE_VALID, 2048, uint16_t(now / 20)});
    Can1.rx.push_back(f);
    f.id = CANID_ENCODER_FB;
    f.len = pack_encoder_fb(f.buf, {int32_t(now), -int32_t(now)});
    Can1.rx.push_back(f);
  }
  if (now % 50 == 0) {
    f.id = CANID_THROTTLE_STATUS;
    f.len = pack_throttle_status(f.buf, {TST_IDLE, TF_NONE, 0, 0});
    Can1.rx.push_back(f);
    f.id = CANID_STEERING_STATUS; f.len = 2; f.buf[0] = f.buf[1] = 0;
    Can1.rx.push_back(f);
  }
}
static void stop(bool active, bool overCan = false) {
  const uint8_t reason = active ? ESTOP_OPERATOR : 0;
  if (overCan) {
    CAN_message_t f; f.id = CANID_ESTOP; f.len = 1; f.buf[0] = reason;
    Can1.rx.push_back(f);
  } else enqueue(CANID_ESTOP, &reason, 1);
}

static void faults() {
  for (unsigned fault = 0; fault < 6; ++fault) {
    reset(fault == 0 ? 0 : fault == 1 ? 0xff : 0x71);
    ImuSpi.ignoreWrites = fault == 2;
    const ThrottleCmd desired = {120, -80, 55, -30};
    for (uint32_t now = 1; now <= 5000; ++now) {
      if (fault == 4 && mpu.stage() == MpuSpiTest::READY) ImuSpi.registers[0x3a] = 0xff;
      if (fault == 5) {
        if (mpu.stage() == MpuSpiTest::READY && now % 10 == 0) sensor();
        if (now >= 1500) ImuSpi.registers[0x75] = 0;
      }
      if (now % 20 == 0) {
        if (now < 4000) drive(desired);
        else drive({1000, 1000, 1000, 1000}, true); // Corrupt commands cannot keep drive alive.
        feedback(now);
      }
      if (now == 1000) stop(true, true);
      if (now == 1100) stop(false);
      tick(now);
      if (now % 20 == 0) {
        const bool stopped = (now >= 1000 && now <= 1100) || now >= 4280;
        assert(equal(output(), stopped ? ThrottleCmd{} : desired));
      }
    }
    assert(mpu.errorCount() >= 2 && !headingHold.ready(5000));
    if (fault < 2) assert(ImuSpi.writes == 0);
    unsigned beats = 0, commands = 0;
    for (const auto& f : Can1.sent) {
      if (f.id == CANID_HEARTBEAT) ++beats;
      else if (f.id == CANID_THROTTLE_CMD) ++commands;
      else assert(f.id == CANID_ESTOP);
    }
    assert(beats == 100 && commands == 250);
    // No IMU frames can consume CAN bandwidth, even during retries.
    std::printf("MPU fault %u: 250 scheduled commands, 100 heartbeats, e-stop and timeout preserved\n", fault);
  }
}

static void recovery_and_wrap() {
  for (uint32_t base : {0U, UINT32_MAX - 400U}) {
    reset(0x71, base); configure(base);
    for (uint32_t elapsed = 312; elapsed <= 5000; ++elapsed) {
      const uint32_t now = base + elapsed;
      if (elapsed % 20 == 0) drive({-300, -200, -100, -50});
      // Simulate a power interruption: identity vanishes, followed by a replug.
      if (elapsed >= 600 && elapsed < 2000) ImuSpi.registers[0x75] = 0;
      if (elapsed == 2000) ImuSpi.registers[0x75] = 0x71;
      if (mpu.stage() == MpuSpiTest::READY && elapsed % 10 == 0
          && (elapsed < 600 || elapsed >= 2000)) sensor();
      const auto before = Can1.sent.size(); tick(now);
      for (size_t index = before; index < Can1.sent.size(); ++index) {
        const auto& frame = Can1.sent[index];
        if (frame.id != CANID_THROTTLE_CMD) continue;
        ThrottleCmd c; assert(unpack_throttle_cmd(frame.buf, frame.len, &c));
        assert(equal(c, {-300, -200, -100, -50}));
      }
    }
    assert(mpu.stage() == MpuSpiTest::READY && mpu.errorCount() > 0);
    const auto errors = mpu.errorCount();
    for (uint32_t elapsed = 5001; elapsed <= 6000; ++elapsed) {
      if (elapsed % 10 == 0) sensor();
      tick(base + elapsed);
    }
    assert(mpu.errorCount() == errors); // Recovery keeps the cumulative count.
    assert(equal(output(), {})); // Good IMU data is never a teleop keepalive.
  }
  reset(0);
  uint32_t now = 100; tick(now); tick(++now);
  for (unsigned retry = 0; retry < 65540; ++retry) {
    now += 1000; tick(now); tick(++now);
  }
  assert(mpu.errorCount() == UINT16_MAX && ImuSpi.writes == 0);
}

static void long_run(const char* capture) {
  reset(0x71); configure(); Serial.tx.clear(); Can1.sent.clear();
  // Thirty simulated minutes, mixing forward/reverse, turns, pivots, one-wheel
  // commands and neutral with 100 Hz IMU and normal CAN feedback traffic.
  const ThrottleCmd requests[] = {{200, 200, 200, 200}, {-200, -200, -200, -200},
      {300, 100, 250, 50}, {200, -200, 200, -200}, {1000, 0, 0, 0}, {0, 0, 0, 0}};
  ThrottleCmd desired = {};
  unsigned commands = 0, beats = 0;
  // configure() last ran the command/heartbeat timers at millisecond 301.
  uint32_t lastCommand = 301, lastBeat = 301;
  uint8_t expectedBeat = hb.counter();
  for (uint32_t now = 312; now <= 1800311; ++now) {
    if (now % 10 == 0) sensor();
    if (now % 20 == 0) {
      desired = requests[(now / 20000) % 6]; drive(desired);
      // Garbage and bad CRCs interleaved with valid commands must resynchronize.
      if (now % 1000 == 0) drive({1000, 1000, 1000, 1000}, true);
    }
    if (now % 10 == 0) feedback(now);
    const auto before = Can1.sent.size(); tick(now);
    for (size_t index = before; index < Can1.sent.size(); ++index) {
      const auto& f = Can1.sent[index];
      if (f.id == CANID_THROTTLE_CMD) {
        assert(now - lastCommand == 20); lastCommand = now; ++commands;
        ThrottleCmd c; assert(unpack_throttle_cmd(f.buf, f.len, &c)); assert(equal(c, desired));
      } else {
        assert(f.id == CANID_HEARTBEAT && now - lastBeat == 50); lastBeat = now; ++beats;
        Heartbeat h; assert(unpack_heartbeat(f.buf, f.len, &h));
        assert(h.counter == uint8_t(++expectedBeat) && h.flags == (HB_FLAG_OK | HB_FLAG_TELEOP_OK));
        const bool moving = desired.fl || desired.fr || desired.rl || desired.rr;
        assert(h.mode == (moving ? MODE_TELEOP : MODE_IDLE));
      }
    }
  }
  assert(commands == 90000 && beats == 36000 && mpu.errorCount() == 0);
  WireParser p; unsigned samples = 0, parts = 0, steering = 0, encoders = 0, status = 0;
  uint8_t sequence = 0;
  for (auto byte : Serial.tx) {
    const auto f = p.feed(byte); if (!f.valid) continue;
    if (f.msg_id >= WIREID_IMU_ACCEL && f.msg_id <= WIREID_IMU_TEMPERATURE) {
      assert(f.msg_id == WIREID_IMU_ACCEL + parts);
      assert(f.payload[0] == sequence && f.payload[1] == 3);
      if (++parts == 3) { ++samples; ++sequence; parts = 0; }
    } else if (f.msg_id == CANID_STEERING_FB) {
      SteeringFeedback s; assert(unpack_steering_feedback(f.payload, f.len, &s));
      assert(s.raw_adc == 2048); ++steering;
    } else if (f.msg_id == CANID_ENCODER_FB) {
      EncoderFb e; assert(unpack_encoder_fb(f.payload, f.len, &e));
      assert(e.left_count == -e.right_count); ++encoders;
    } else if (f.msg_id == CANID_THROTTLE_STATUS || f.msg_id == CANID_STEERING_STATUS) ++status;
    else assert(f.msg_id == CANID_HEARTBEAT || f.msg_id == WIREID_MPU_SPI_DIAGNOSTICS || f.msg_id == WIREID_HEADING_STATUS);
  }
  assert(samples == 180000 && parts == 0 && steering == 90000 && encoders == steering && status == 72000);
  if (capture) {
    auto* file = std::fopen(capture, "wb"); assert(file);
    assert(std::fwrite(Serial.tx.data(), 1, Serial.tx.size(), file) == Serial.tx.size());
    assert(std::fclose(file) == 0);
  }
  std::puts("MPU master: 30 simulated minutes, 180,000 samples, 90,000 commands, 36,000 heartbeats and concurrent CAN feedback passed");
}

int main(int argc, char** argv) {
  faults(); recovery_and_wrap(); long_run(argc == 2 ? argv[1] : nullptr);
}
