// Actual master loop + SPI transport + BNO state machine, without vehicle I/O.
#include "../../../master/src/main.cpp"
#include <cstdio>
#include <initializer_list>
#include "steering_telemetry.h"

static void reset(uint32_t now = 0) {
  fake_master::now = now;
  currentCmd = {0, 0, 0, 0}; estopLatched = false;
  hb = HeartbeatTx(HEARTBEAT_PERIOD_MS);
  teleopWatch = Watchdog(TELEOP_TIMEOUT_MS); parser = WireParser();
  lastCmdMs = now;
  headingHold = HeadingHold(); imu = Bno08x();
  imuTransport.wake(false);
  ImuSpi.rx.clear(); ImuSpi.tx.clear(); ImuSpi.transactions = 0;
  imuSequence = 0; imuSampleValid = imuTelemetryPending = false;
  lastImuDiagnosticMs = lastHeadingStatusMs = now;
  reportedImuErrors = 0; reportedImuReady = false;
  reportedImuStage = Bno08x::POWER_WAIT;
  Can1.sent.clear(); Can1.rx.clear(); Serial.tx.clear(); Serial.rx.clear();
  Serial.advanceOnRead = 0;
  setup();
  assert(ImuSpi.begun && ImuSpi.mosi == PE6 && ImuSpi.miso == PE5 && ImuSpi.sck == PE2);
  assert(digitalRead(PE4) == HIGH && digitalRead(PE3) == LOW && digitalRead(PF7) == HIGH);
  assert(fake_master::mode[PF8] == INPUT_PULLUP);
  assert(fake_master::mode[PE4] == OUTPUT && fake_master::mode[PE3] == OUTPUT
      && fake_master::mode[PF7] == OUTPUT);
  assert((CAN1->MCR & CAN_MCR_ABOM) != 0); // Existing CAN bus-off recovery retained.
}

static void enqueue(uint16_t id, const uint8_t* p, uint8_t n) {
  uint8_t frame[WIRE_MAX_FRAME];
  const auto size = wire_frame(frame, id, p, n);
  Serial.rx.insert(Serial.rx.end(), frame, frame + size);
}
static void command(ThrottleCmd c) {
  uint8_t p[8]; const auto n = pack_throttle_cmd(p, c);
  enqueue(CANID_THROTTLE_CMD, p, n);
}
static void step(uint32_t now) {
  fake_master::now = now;
  const auto before = ImuSpi.transactions;
  loop();
  // No sensor state can spin on INT or drain an unbounded receive queue.
  assert(ImuSpi.transactions - before <= 2);
  assert(!ImuSpi.active && digitalRead(PE4) == HIGH);
}
static void packet(uint8_t channel, const std::vector<uint8_t>& payload) {
  assert(ImuSpi.rx.empty());
  const auto size = payload.size() + 4;
  ImuSpi.rx = {uint8_t(size), uint8_t(size >> 8), channel, 0};
  ImuSpi.rx.insert(ImuSpi.rx.end(), payload.begin(), payload.end());
  digitalWrite(PF8, LOW);
}
static void receive(uint32_t now, uint8_t channel, const std::vector<uint8_t>& payload) {
  packet(channel, payload); step(now);
  assert(ImuSpi.rx.empty()); digitalWrite(PF8, HIGH);
}
static void configure(uint32_t base = 0) {
  step(base + 100); assert(imu.stage() == Bno08x::RESET_LOW);
  step(base + 102); assert(digitalRead(PE3) == HIGH && digitalRead(PF7) == HIGH);
  receive(base + 103, 1, {1}); assert(imu.stage() == Bno08x::IDENTIFY);
  digitalWrite(PF8, LOW); step(base + 104);
  assert((ImuSpi.tx.back() == std::vector<uint8_t>{6, 0, 2, 0, 0xf9, 0}));
  std::vector<uint8_t> product(16, 0); product[0] = 0xf8; product[4] = 0x85;
  receive(base + 105, 2, product);
  assert(imu.product() == 0x85 && imu.stage() == Bno08x::CONFIGURE);
  digitalWrite(PF8, LOW);
  for (uint32_t i = 0; i < 4; ++i) {
    step(base + 106 + i);
    const auto& tx = ImuSpi.tx.back();
    assert(tx.size() == 21 && tx[2] == 2 && tx[3] == i + 1 && tx[4] == 0xfd);
    assert(tx[5] == (i == 0 ? 1 : i == 1 ? 2 : i == 2 ? 5 : 8));
    assert(tx[9] == 0x10 && tx[10] == 0x27); // 10,000 us = 100 Hz.
  }
  digitalWrite(PF8, HIGH);
  assert(imu.stage() == Bno08x::STREAM_WAIT);
}
static void append16(std::vector<uint8_t>& out, int16_t v) {
  out.push_back(uint8_t(v)); out.push_back(uint8_t(uint16_t(v) >> 8));
}
static std::vector<uint8_t> reports(uint8_t seq, float yaw = 0, bool game = true) {
  std::vector<uint8_t> out = {0xfb, 0, 0, 0, 0};
  for (uint8_t id : {1, 2, 5, 8}) {
    if (id == 8 && !game) continue;
    out.insert(out.end(), {id, seq, 3, 0});
    if (id == 1) {
      for (auto value : {256, -512, 2511}) append16(out, value);
    } else if (id == 2) {
      for (int i = 0; i < 3; ++i) append16(out, 0);
    } else {
      append16(out, 0); append16(out, 0);
      append16(out, int16_t(roundf(sinf(yaw / 2) * 16384)));
      append16(out, int16_t(roundf(cosf(yaw / 2) * 16384)));
      if (id == 5) append16(out, 2048);
    }
  }
  return out;
}
static ThrottleCmd lastCommand() {
  for (auto it = Can1.sent.rbegin(); it != Can1.sent.rend(); ++it) {
    if (it->id != CANID_THROTTLE_CMD) continue;
    ThrottleCmd c; assert(unpack_throttle_cmd(it->buf, it->len, &c)); return c;
  }
  assert(false); return {};
}
static void equal(ThrottleCmd a, ThrottleCmd b) {
  if (a.fl != b.fl || a.fr != b.fr || a.rl != b.rl || a.rr != b.rr)
    std::fprintf(stderr, "t=%u actual=%d/%d/%d/%d expected=%d/%d/%d/%d\n",
                 fake_master::now, a.fl, a.fr, a.rl, a.rr, b.fl, b.fr, b.rl, b.rr);
  assert(a.fl == b.fl && a.fr == b.fr && a.rl == b.rl && a.rr == b.rr);
}
static void healthy() {
  reset(); configure(); receive(110, 3, reports(0));
  assert(imu.ready());
  for (uint32_t t = 120; t <= 720; t += 10) {
    command({0, 0, 0, 0}); receive(t, 3, reports(uint8_t(t / 10)));
  }
  assert(headingHold.ready(720));
}

static void faults_and_safety() {
  for (int fault = 0; fault < 4; ++fault) {
    reset();
    // Absent sensor, stuck-low INT, all-FF MISO, and invalid short length.
    for (uint32_t t = 1; t <= 6000; ++t) {
      if (fault != 0) digitalWrite(PF8, LOW);
      if (fault >= 2 && imu.stage() == Bno08x::BOOT_WAIT && ImuSpi.rx.empty())
        ImuSpi.rx = fault == 2 ? std::deque<uint8_t>{255, 255, 255, 255}
                             : std::deque<uint8_t>{3, 0, 3, 0};
      if (t % 20 == 0 && t < 5000) command({120, 80, 110, 70});
      if (t == 2000 || t == 2100) {
        const uint8_t reason = t == 2000 ? ESTOP_OPERATOR : 0;
        enqueue(CANID_ESTOP, &reason, 1);
      }
      step(t);
      if (t % 20 == 0) {
        const auto expected = (t >= 2000 && t <= 2100) || t >= 5280
            ? ThrottleCmd{0, 0, 0, 0} : ThrottleCmd{120, 80, 110, 70};
        equal(lastCommand(), expected);
      }
    }
    unsigned beats = 0, commands = 0;
    for (const auto& frame : Can1.sent) {
      if (frame.id == CANID_HEARTBEAT) ++beats;
      else if (frame.id == CANID_THROTTLE_CMD) ++commands;
      else assert(frame.id == CANID_ESTOP);
    }
    assert(beats == 120 && commands == 300 && imu.errorCount() >= 2);
    assert(!imu.ready() && headingHold.correction() == 0);
  }
}

static void recovery_and_overrides() {
  healthy(); command({200, 200, 200, 200}); step(740);
  for (uint32_t t = 750; t <= 900; t += 10) {
    command({200, 200, 200, 200}); receive(t, 3, reports(uint8_t(t / 10), 0.05f));
  }
  assert(headingHold.correction() < 0 && lastCommand().fl > lastCommand().fr);
  CAN_message_t steering;
  steering.id = CANID_STEERING_FB;
  steering.len = pack_steering_feedback(steering.buf, {SF_SAMPLE_VALID, 2048, 42});
  Can1.rx.push_back(steering);
  command({120, 80, 110, 70}); step(920);
  equal(lastCommand(), {120, 80, 110, 70});
  command({200, 200, 200, 200}); step(940);
  // A physical sensor reset invalidates assist while leaving command transport alive.
  receive(950, 1, {1});
  assert(imu.error() == Bno08x::SENSOR_RESET && headingHold.correction() == 0);
  command({200, 200, 200, 200}); step(960);
  equal(lastCommand(), {200, 200, 200, 200});
  // Recover through the actual SPI handshake; error count survives.
  configure(1850); receive(1960, 3, reports(0));
  for (uint32_t t = 1970; t <= 2600; t += 10) {
    command({200, 200, 200, 200}); receive(t, 3, reports(uint8_t(t / 10), 0.1f));
  }
  assert(imu.ready() && imu.errorCount() == 1 && headingHold.ready(2600));
  assert(headingHold.state() == HeadingHold::REARM);
  equal(lastCommand(), {200, 200, 200, 200});
  command({0, 0, 0, 0}); step(2620);
  command({200, 200, 200, 200}); step(2640);
  assert(headingHold.state() == HeadingHold::HOLDING);
  for (uint32_t t = 2650; t <= 2740; t += 10) {
    command({200, 200, 200, 200}); receive(t, 3, reports(uint8_t(t / 10), 0.15f));
  }
  assert(headingHold.correction() < 0);
  const uint8_t reason = ESTOP_OPERATOR;
  enqueue(CANID_ESTOP, &reason, 1); step(2760);
  equal(lastCommand(), {0, 0, 0, 0}); assert(headingHold.correction() == 0);
}

static void long_run() {
  reset(); configure(); receive(110, 3, reports(0, 0, false));
  // Ten simulated minutes: normal 100 Hz telemetry, repeated sequence wraps,
  // 50 Hz operator/CAN traffic, steering feedback, and missing optional GRV.
  Serial.tx.clear(); Can1.sent.clear();
  for (uint32_t t = 111; t <= 600110; ++t) {
    if (t % 20 == 0) {
      command({200, 200, 200, 200});
      CAN_message_t steering; steering.id = CANID_STEERING_FB;
      steering.len = pack_steering_feedback(steering.buf, {SF_SAMPLE_VALID, 2048, uint16_t(t / 20)});
      Can1.rx.push_back(steering);
    }
    if (t % 10 == 0) {
      packet(3, reports(uint8_t(t / 10), 0, false)); step(t);
      digitalWrite(PF8, HIGH);
    } else step(t);
    if (t % 20 == 0) equal(lastCommand(), {200, 200, 200, 200});
  }
  unsigned beats = 0, commands = 0;
  for (const auto& frame : Can1.sent) {
    if (frame.id == CANID_HEARTBEAT) ++beats;
    else { assert(frame.id == CANID_THROTTLE_CMD); ++commands; }
  }
  assert(beats == 12000 && commands == 30000);
  assert(imu.ready() && imu.errorCount() == 0 && headingHold.correction() == 0);
  WireParser p; unsigned snapshots = 0, parts = 0, feedback = 0;
  uint8_t expected = 1;
  for (auto byte : Serial.tx) {
    const auto frame = p.feed(byte);
    if (!frame.valid) continue;
    if (frame.msg_id == CANID_STEERING_FB) { ++feedback; continue; }
    if (frame.msg_id < WIREID_BNO_ACCEL || frame.msg_id > WIREID_BNO_QUAT_W) continue;
    assert(frame.msg_id == WIREID_BNO_ACCEL + parts);
    assert(frame.payload[0] == expected && (frame.payload[1] & 3) == 3);
    if (parts == 0) assert(get_i16(frame.payload + 2) == 256 && get_i16(frame.payload + 4) == -512);
    if (parts == 3) {
      assert(frame.len == 6 && get_i16(frame.payload + 2) == 16384);
      ++snapshots; ++expected; parts = 0;
    } else { assert(frame.len == 8); ++parts; }
  }
  assert(snapshots == 60000 && feedback == 30000 && parts == 0);
}

int main(int argc, char** argv) {
  faults_and_safety(); recovery_and_overrides(); long_run();
  // Optional raw capture for the Python bridge interoperability test.
  if (argc == 2) {
    auto* file = std::fopen(argv[1], "wb"); assert(file);
    assert(std::fwrite(Serial.tx.data(), 1, Serial.tx.size(), file) == Serial.tx.size());
    assert(std::fclose(file) == 0);
  }
  std::puts("Master IMU: SPI4 pins, real startup/stream/recovery, sensor faults, safety overrides and 10-minute traffic passed");
}
