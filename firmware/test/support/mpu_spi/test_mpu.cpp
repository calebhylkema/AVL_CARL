#include "fixture.h"
int main() {
  for (auto identity : {0, 0xff, 0x68, 0x73}) {
    reset(identity);
    for (uint32_t t = 1; t <= 2200; ++t) tick(t);
    assert(ImuSpi.writes == 0 && mpu.errorCount() >= 2);
    uint8_t diag[8]; mpu.diagnostics(diag);
    assert(diag[2] == identity && diag[3] == MpuSpiTest::WRONG_IDENTITY && diag[4] == 0x75);
  }
  for (auto identity : {0x70, 0x71}) {
    reset(identity); configure();
    Serial.tx.clear();
    for (uint32_t t = 312; t <= 3312; ++t) {
      if (t % 10 == 0) {
        data(0x3b, -16384); data(0x3d, 123); data(0x3f, 16384);
        data(0x41, -32768); data(0x43, -131); data(0x45, 0); data(0x47, 32767);
        ImuSpi.registers[0x3a] = 1;
      }
      if (t % 20 == 0) command({200, 100, -50, 25});
      tick(t);
    }
    assert(mpu.errorCount() == 0 && mpu.stage() == MpuSpiTest::READY);
    auto c = output(); assert(c.fl == 200 && c.fr == 100 && c.rl == -50 && c.rr == 25);
    WireParser p; unsigned accel = 0, gyro = 0, temperature = 0;
    for (auto b : Serial.tx) {
      const auto f = p.feed(b); if (!f.valid) continue;
      if (f.msg_id == WIREID_IMU_ACCEL) {
        assert(f.len == 8 && get_i16(f.payload + 2) == -16384 && get_i16(f.payload + 6) == 16384); ++accel;
      } else if (f.msg_id == WIREID_IMU_GYRO) {
        assert(get_i16(f.payload + 2) == -131 && get_i16(f.payload + 6) == 32767); ++gyro;
      } else if (f.msg_id == WIREID_IMU_TEMPERATURE) {
        assert(f.len == 4 && get_i16(f.payload + 2) == -32768); ++temperature;
      } else assert(f.msg_id == CANID_HEARTBEAT || f.msg_id == WIREID_MPU_SPI_DIAGNOSTICS || f.msg_id == WIREID_HEADING_STATUS);
    }
    assert(accel == 300 && gyro == accel && temperature == accel);
    tick(3620); c = output(); assert(c.fl == 0 && c.fr == 0 && c.rl == 0 && c.rr == 0);
    assert(mpu.stage() == MpuSpiTest::RETRY_WAIT);
    // Real command transport remains usable after sensor failure; e-stop wins.
    command({200, 200, 200, 200}); tick(3640); assert(output().fl == 200);
    const uint8_t reason = ESTOP_OPERATOR; handleWireFrame(CANID_ESTOP, &reason, 1);
    tick(3660); assert(output().fl == 0);
  }
  reset(0x70); ImuSpi.ignoreWrites = true;
  tick(100); tick(101); tick(201); tick(301); tick(302); tick(303);
  assert(mpu.stage() == MpuSpiTest::RETRY_WAIT);
  uint8_t diag[8]; mpu.diagnostics(diag); assert(diag[3] == MpuSpiTest::VERIFY_FAILED);
  reset(0x70, UINT32_MAX - 105); configure(UINT32_MAX - 105);
  tick(uint32_t(UINT32_MAX - 105 + 562U)); assert(mpu.stage() == MpuSpiTest::RETRY_WAIT);
  std::puts("MPU SPI: identities, no writes to unknown chips, config verification, raw framing, timeout, safety and rollover passed");
}
