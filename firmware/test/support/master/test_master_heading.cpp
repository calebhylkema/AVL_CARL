// Compile the actual master to verify the wire-command -> heading -> CAN path.
#include "../../../master/src/main.cpp"
#include <cstdio>
#include "steering_telemetry.h"

static void input(ThrottleCmd c) {
  uint8_t bytes[8];
  const uint8_t n = pack_throttle_cmd(bytes, c);
  handleWireFrame(CANID_THROTTLE_CMD, bytes, n);
}
static void sensor(uint32_t now, float yaw) {
  fake_master::now = now;
  Bno08xHeadingSample s = {};
  s.quaternion[2] = (int16_t)roundf(sinf(yaw * 0.5f) * 16384);
  s.quaternion[3] = (int16_t)roundf(cosf(yaw * 0.5f) * 16384);
  s.accuracy = s.gyro_accuracy = 3;
  s.orientation_ms = s.gyro_ms = now;
  assert(headingHold.accept(s, now));
}
static ThrottleCmd sent() {
  const auto& frame = Can1.sent.back();
  assert(frame.id == CANID_THROTTLE_CMD);
  ThrottleCmd c;
  assert(unpack_throttle_cmd(frame.buf, frame.len, &c));
  return c;
}
static void stopped() {
  const auto c = sent();
  assert(c.fl == 0 && c.fr == 0 && c.rl == 0 && c.rr == 0);
}

int main() {
  for (uint32_t t = 0; t <= 600; t += 20) {
    sensor(t, 0);
    input({0, 0, 0, 0});
    sendThrottleCmd(t, true);
    stopped();
  }
  input({200, 200, 200, 200});
  sendThrottleCmd(600, true);
  assert(sent().fl == 200 && sent().fr == 200);
  for (uint32_t t = 620; t <= 800; t += 20) {
    sensor(t, 0.05f);
    // Repeated operator messages must not re-zero an existing straight run.
    input({200, 200, 200, 200});
    sendThrottleCmd(t, true);
  }
  auto c = sent();
  assert(c.fl > 200 && c.fr < 200 && c.fl == c.rl && c.fr == c.rr);
  assert(currentCmd.fl == 200 && currentCmd.fr == 200);
  assert(headingHold.state() == HeadingHold::HOLDING);
  sendHeadingStatus(800);
  WireParser statusParser;
  bool sawStatus = false;
  for (auto b : Serial.tx) {
    const auto result = statusParser.feed(b);
    if (result.valid && result.msg_id == WIREID_HEADING_STATUS) {
      assert(result.len == 8 && result.payload[0] == HeadingHold::HOLDING);
      assert(result.payload[1] == 1 && get_i16(result.payload + 6) < 0);
      sawStatus = true;
    }
  }
  assert(sawStatus);
  input({120, 80, 120, 80});
  sendThrottleCmd(801, true);
  c = sent();
  assert(c.fl == 120 && c.fr == 80 && c.rl == 120 && c.rr == 80);
  input({200, 200, 200, 200});
  sendThrottleCmd(802, true);
  assert(headingHold.correction() == 0);

  // Assert e-stop while an actual learned heading correction is being sent.
  for (uint32_t t = 820; t <= 1000; t += 20) {
    sensor(t, 0.10f);
    input({200, 200, 200, 200});
    sendThrottleCmd(t, true);
  }
  assert(headingHold.correction() < 0 && sent().fl > sent().fr);

  uint8_t reason = ESTOP_OPERATOR;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  assert(estopLatched && Can1.sent.back().id == CANID_ESTOP);
  sendThrottleCmd(1001, true); stopped();
  reason = 0;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  assert(!estopLatched);
  sendThrottleCmd(1002, true); stopped();
  input({-200, -200, -200, -200});
  sendThrottleCmd(1003, true);
  assert(sent().fl == -200 && sent().fr == -200);
  fake_master::now = 1003 + TELEOP_TIMEOUT_MS;
  sendThrottleCmd(fake_master::now, !teleopWatch.expired(fake_master::now));
  stopped();
  input({0, 0, 0, 0}); sendThrottleCmd(fake_master::now, true); stopped();
  input({200, 200, 200, 200});
  headingHold.invalidate(); // Sensor failure allows manual PWM, clears correction.
  sendThrottleCmd(fake_master::now, true);
  assert(sent().fl == 200 && sent().fr == 200 && headingHold.correction() == 0);

  CAN_message_t externalStop;
  externalStop.id = CANID_ESTOP; externalStop.len = 1;
  Can1.rx.push_back(externalStop); pollCan();
  assert(currentCmd.fl == 0 && currentCmd.fr == 0);
  sendThrottleCmd(fake_master::now, true); stopped();
  reason = 0;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  sendThrottleCmd(fake_master::now, true); stopped(); // No stale command revival.
  fake_master::now += TELEOP_TIMEOUT_MS;
  handleWireFrame(CANID_ESTOP, &reason, 1);
  assert(teleopWatch.expired(fake_master::now)); // Reset is not a drive keepalive.

  // Read a real framed command across several millis boundaries, then run
  // the production loop: its watchdog timestamp must be read AFTER ingress.
  uint8_t commandBytes[8], framed[WIRE_MAX_FRAME];
  const auto bytes = pack_throttle_cmd(commandBytes, {200, 200, 200, 200});
  const auto frameSize = wire_frame(framed, CANID_THROTTLE_CMD, commandBytes, bytes);
  for (uint8_t i = 0; i < frameSize; ++i) Serial.rx.push_back(framed[i]);
  Serial.advanceOnRead = 1;
  Can1.sent.clear();
  loop();
  bool sawDrive = false;
  for (const auto& frame : Can1.sent) {
    if (frame.id != CANID_THROTTLE_CMD) continue;
    ThrottleCmd drive;
    assert(unpack_throttle_cmd(frame.buf, frame.len, &drive));
    assert(drive.fl == 200 && drive.fr == 200 && drive.rl == 200 && drive.rr == 200);
    sawDrive = true;
  }
  assert(sawDrive && !teleopWatch.expired(fake_master::now));
  // Actual CAN receive -> serial frame path forwards the new feedback ID.
  Serial.tx.clear();
  CAN_message_t steering;
  steering.id = CANID_STEERING_FB;
  steering.len = pack_steering_feedback(steering.buf, {SF_SAMPLE_VALID, 2048, 65535});
  Can1.rx.push_back(steering);
  pollCan();
  WireParser steeringParser;
  bool sawSteering = false;
  for (auto byte : Serial.tx) {
    const auto result = steeringParser.feed(byte);
    if (!result.valid) continue;
    assert(result.msg_id == CANID_STEERING_FB);
    SteeringFeedback sample;
    assert(unpack_steering_feedback(result.payload, result.len, &sample));
    assert(sample.raw_adc == 2048 && sample.sequence == 65535);
    sawSteering = true;
  }
  assert(sawSteering);
  const auto count = Can1.sent.size();
  handleWireFrame(CANID_STEERING_FB, steering.buf, steering.len);
  assert(Can1.sent.size() == count); // Telemetry cannot inject an actuator command.
  Serial.tx.clear();
  sendImuDiagnostics(fake_master::now + 2000);
  WireParser spiParser;
  bool sawSpiDiagnostic = false;
  for (auto byte : Serial.tx) {
    const auto result = spiParser.feed(byte);
    if (!result.valid || result.msg_id != WIREID_BNO_SPI_DIAGNOSTICS) continue;
    assert(result.len == 8 && result.payload[0] == 1);
    assert(result.payload[1] == Bno08xSpi::NO_READ);
    sawSpiDiagnostic = true;
  }
  assert(sawSpiDiagnostic);
  std::puts("Master firmware: command correction, status framing, manual turn, estop and teleop watchdog passed");
}
