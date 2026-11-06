#pragma once
#include "../../../master/src/main.cpp"
#include <cstdio>
#include <initializer_list>
#include "steering_telemetry.h"

inline void reset(uint8_t identity, uint32_t now = 0) {
  fake_master::now = now;
  std::fill(std::begin(ImuSpi.registers), std::end(ImuSpi.registers), 0);
  ImuSpi.registers[0x75] = identity; ImuSpi.ignoreWrites = false;
  ImuSpi.transactions = ImuSpi.writes = 0;
  currentCmd = {}; estopLatched = false;
  hb = HeartbeatTx(HEARTBEAT_PERIOD_MS); teleopWatch = Watchdog(TELEOP_TIMEOUT_MS);
  parser = WireParser(); headingHold = HeadingHold();
  mpuHeading = MpuHeading();
  haveEncoderEvidence = haveOutputEvidence = outputsIdle = false;
  previousEncoders = {};
  lastEncoderEvidenceMs = lastEncoderMotionMs = lastOutputEvidenceMs = now;
  lastCmdMs = lastHeadingStatusMs = now;
  Can1.sent.clear(); Can1.rx.clear(); Serial.tx.clear(); Serial.rx.clear();
  imuSequence = 0; reportedImuErrors = 0; reportedMpuStage = MpuSpiTest::POWER_WAIT;
  setup();
  assert(ImuSpi.mosi == PE6 && ImuSpi.miso == PE5 && ImuSpi.sck == PE2);
  assert(digitalRead(PE4) == HIGH);
  assert(fake_master::mode[PE3] == 0 && fake_master::mode[PF7] == 0);
}
inline void tick(uint32_t now) {
  fake_master::now = now;
  const auto n = ImuSpi.transactions; loop();
  assert(ImuSpi.transactions - n <= 3 && !ImuSpi.active && digitalRead(PE4) == HIGH);
}
inline void configure(uint32_t base = 0) {
  tick(base + 99); assert(ImuSpi.writes == 0);
  tick(base + 100); tick(base + 101); assert(mpu.stage() == MpuSpiTest::RESET_WAIT);
  tick(base + 200); assert(mpu.stage() == MpuSpiTest::RESET_WAIT);
  tick(base + 201); tick(base + 301);
  for (uint32_t t = 302; t < 312; ++t) tick(base + t);
  assert(mpu.stage() == MpuSpiTest::READY);
  assert(ImuSpi.registers[0x6a] == 0x10 && ImuSpi.registers[0x19] == 9);
}
inline void data(uint8_t reg, int16_t value) {
  ImuSpi.registers[reg] = uint8_t(uint16_t(value) >> 8);
  ImuSpi.registers[reg + 1] = uint8_t(value);
}
inline void command(ThrottleCmd c) {
  uint8_t p[8]; const auto size = pack_throttle_cmd(p, c);
  handleWireFrame(CANID_THROTTLE_CMD, p, size);
}
inline ThrottleCmd output() {
  for (auto i = Can1.sent.rbegin(); i != Can1.sent.rend(); ++i) {
    if (i->id == CANID_THROTTLE_CMD) {
      ThrottleCmd c; assert(unpack_throttle_cmd(i->buf, i->len, &c)); return c;
    }
  }
  assert(false); return {};
}
