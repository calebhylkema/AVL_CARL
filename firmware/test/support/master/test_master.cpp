// Compile the actual master to verify serial ingress, safety gates and CAN egress.
#include "../../../master/src/main.cpp"
#include <cstdio>
#include "steering_telemetry.h"

static bool equal(ThrottleCmd a, ThrottleCmd b) {
  return a.fl == b.fl && a.fr == b.fr && a.rl == b.rl && a.rr == b.rr;
}
static void reset(uint32_t now = 0) {
  fake_master::now = now;
  currentCmd = {0, 0, 0, 0};
  estopLatched = false;
  hb = HeartbeatTx(HEARTBEAT_PERIOD_MS);
  teleopWatch = Watchdog(TELEOP_TIMEOUT_MS);
  parser = WireParser();
  lastCmdMs = now;
  Can1.sent.clear(); Can1.rx.clear();
  Serial.tx.clear(); Serial.rx.clear(); Serial.advanceOnRead = 0;
}
static void input(ThrottleCmd c) {
  uint8_t bytes[8];
  handleWireFrame(CANID_THROTTLE_CMD, bytes, pack_throttle_cmd(bytes, c));
}
static void enqueue(uint16_t id, const uint8_t* payload, uint8_t len, bool corrupt = false) {
  uint8_t frame[WIRE_MAX_FRAME];
  const auto size = wire_frame(frame, id, payload, len);
  if (corrupt) frame[size - 1] ^= 0xff;
  for (uint8_t i = 0; i < size; ++i) Serial.rx.push_back(frame[i]);
}
static ThrottleCmd sent() {
  for (auto it = Can1.sent.rbegin(); it != Can1.sent.rend(); ++it) {
    if (it->id != CANID_THROTTLE_CMD) continue;
    ThrottleCmd c;
    assert(unpack_throttle_cmd(it->buf, it->len, &c));
    return c;
  }
  assert(false && "No throttle command sent");
  return {0, 0, 0, 0};
}
static void stopped() { assert(equal(sent(), {0, 0, 0, 0})); }

static void test_passthrough_and_safety() {
  reset();
  for (const auto cmd : {ThrottleCmd{0, 0, 0, 0}, {200, 200, 200, 200},
       {-200, -200, -200, -200}, {120, 80, 110, 70}, {-1000, 1000, -500, 500}}) {
    // Repeated commands and long runs must preserve all four original values.
    for (unsigned i = 0; i < 100; ++i) {
      fake_master::now += THROTTLE_CMD_PERIOD_MS;
      input(cmd); sendThrottleCmd(true);
      assert(equal(sent(), cmd) && equal(currentCmd, cmd));
    }
  }
  input({200, 200, 200, 200});
  uint8_t reason = ESTOP_OPERATOR;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  assert(estopLatched && Can1.sent.back().id == CANID_ESTOP);
  // New commands received during a latched stop must still produce zero.
  input({200, 200, 200, 200}); sendThrottleCmd(true); stopped();
  reason = 0;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  assert(!estopLatched);
  sendThrottleCmd(true); stopped(); // Reset cannot revive retained commands.
  input({-200, -200, -200, -200});
  fake_master::now += TELEOP_TIMEOUT_MS;
  assert(teleopWatch.expired(fake_master::now));
  sendThrottleCmd(false); stopped();
  const auto count = Can1.sent.size();
  handleWireFrame(CANID_ESTOP, &reason, 1);
  assert(Can1.sent.size() == count && teleopWatch.expired(fake_master::now));

  input({200, 200, 200, 200});
  CAN_message_t externalStop;
  externalStop.id = CANID_ESTOP; externalStop.len = 1;
  Can1.rx.push_back(externalStop); pollCan();
  assert(estopLatched && equal(currentCmd, {0, 0, 0, 0}));
  sendThrottleCmd(true); stopped();
  handleWireFrame(CANID_ESTOP, &reason, 1);
  sendThrottleCmd(true); stopped();
}

static void test_loop_and_watchdog() {
  reset(1000);
  uint8_t bytes[8];
  const ThrottleCmd cmd = {200, 150, 100, 50};
  enqueue(CANID_THROTTLE_CMD, bytes, pack_throttle_cmd(bytes, cmd));
  Serial.advanceOnRead = 1;
  fake_master::now += THROTTLE_CMD_PERIOD_MS;
  loop();
  bool sawDrive = false;
  for (const auto& frame : Can1.sent) {
    if (frame.id != CANID_THROTTLE_CMD) continue;
    ThrottleCmd drive;
    assert(unpack_throttle_cmd(frame.buf, frame.len, &drive));
    assert(equal(drive, cmd)); sawDrive = true;
  }
  assert(sawDrive && !teleopWatch.expired(fake_master::now));
  // Once input is quiet, the production loop clears the retained command.
  fake_master::now += TELEOP_TIMEOUT_MS;
  loop();
  assert(equal(currentCmd, {0, 0, 0, 0}));
  stopped();
  for (const auto& frame : Can1.sent) {
    if (frame.id != CANID_HEARTBEAT) continue;
    Heartbeat status;
    assert(unpack_heartbeat(frame.buf, frame.len, &status));
    if (!(status.flags & HB_FLAG_TELEOP_OK)) assert(status.mode == MODE_IDLE);
  }
  sendThrottleCmd(false); stopped();
  // Malformed commands and bad CRCs cannot feed the watchdog or move the kart.
  enqueue(CANID_THROTTLE_CMD, bytes, 7);
  enqueue(CANID_THROTTLE_CMD, bytes, 8, true);
  loop();
  assert(teleopWatch.expired(fake_master::now));
  assert(equal(currentCmd, {0, 0, 0, 0}));
  // First boot also stays stopped until a valid command arrives.
  reset(); fake_master::now = THROTTLE_CMD_PERIOD_MS; loop(); stopped();
  // Timer wrap does not expire a fresh command or suppress scheduled commands.
  reset(UINT32_MAX - 10); input(cmd);
  fake_master::now += THROTTLE_CMD_PERIOD_MS;
  loop();
  assert(equal(sent(), cmd));
  fake_master::now += TELEOP_TIMEOUT_MS; loop(); stopped();
}

static void test_heartbeat_stream() {
  reset();
  for (uint32_t now = 0; now <= 1000; ++now) {
    fake_master::now = now;
    loop();
  }
  unsigned commands = 0, beats = 0;
  for (const auto& frame : Can1.sent) {
    if (frame.id == CANID_THROTTLE_CMD) {
      ++commands;
      ThrottleCmd c;
      assert(unpack_throttle_cmd(frame.buf, frame.len, &c));
      assert(equal(c, {0, 0, 0, 0}));
    } else { assert(frame.id == CANID_HEARTBEAT); ++beats; }
  }
  assert(commands == 50 && beats == 20);
  WireParser p;
  unsigned serialBeats = 0;
  for (auto byte : Serial.tx) {
    const auto result = p.feed(byte);
    if (!result.valid) continue;
    // No extra local telemetry is emitted on the idle companion link.
    assert(result.msg_id == CANID_HEARTBEAT);
    Heartbeat h;
    assert(unpack_heartbeat(result.payload, result.len, &h));
    assert(h.counter == ++serialBeats && h.flags == HB_FLAG_OK && h.mode == MODE_IDLE);
  }
  assert(serialBeats == beats);
  input({200, 200, 200, 200});
  sendHeartbeat(fake_master::now, true);
  Heartbeat h;
  assert(unpack_heartbeat(Can1.sent.back().buf, Can1.sent.back().len, &h));
  assert(h.flags == (HB_FLAG_OK | HB_FLAG_TELEOP_OK) && h.mode == MODE_TELEOP);
  const uint8_t reason = ESTOP_OPERATOR;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  sendHeartbeat(fake_master::now, true);
  assert(unpack_heartbeat(Can1.sent.back().buf, Can1.sent.back().len, &h));
  assert((h.flags & HB_FLAG_ESTOP) && h.mode == MODE_ESTOP);
}

static void test_telemetry_forwarding() {
  reset();
  for (auto id : {CANID_THROTTLE_STATUS, CANID_ENCODER_FB, CANID_STEERING_STATUS, CANID_STEERING_FB}) {
    Serial.tx.clear();
    CAN_message_t frame;
    frame.id = id; frame.len = 8;
    for (unsigned i = 0; i < 8; ++i) frame.buf[i] = 0x10 + i;
    if (id == CANID_STEERING_FB)
      frame.len = pack_steering_feedback(frame.buf, {SF_SAMPLE_VALID, 2048, 65535});
    Can1.rx.push_back(frame); pollCan();
    WireParser p;
    unsigned forwarded = 0;
    for (auto byte : Serial.tx) {
      const auto result = p.feed(byte);
      if (!result.valid) continue;
      assert(result.msg_id == id && result.len == frame.len);
      for (unsigned i = 0; i < frame.len; ++i) assert(result.payload[i] == frame.buf[i]);
      ++forwarded;
    }
    assert(forwarded == 1);
    handleWireFrame(id, frame.buf, frame.len);
    assert(Can1.sent.empty()); // Feedback cannot inject actuator commands.
  }
  Serial.tx.clear();
  CAN_message_t unknown; unknown.id = 0x7ff; unknown.len = 8;
  Can1.rx.push_back(unknown); pollCan();
  handleWireFrame(unknown.id, unknown.buf, unknown.len);
  assert(Serial.tx.empty() && Can1.sent.empty() && !teleopWatch.everFed());
}

int main() {
  test_passthrough_and_safety();
  test_loop_and_watchdog();
  test_heartbeat_stream();
  test_telemetry_forwarding();
  std::puts("Master: passthrough, framing, heartbeat timing/flags, telemetry, e-stop, watchdog and rollover passed");
}
