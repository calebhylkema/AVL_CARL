// Host (native) unit tests for the AVL_CARL protocol.
//   pio test -e native
//
// These run on the PC — no board required. They guard the single source of
// truth in lib/carl_protocol so a sender and receiver can never drift on layout,
// endianness, sign handling, or wire framing.
#include <unity.h>
#include <string.h>

#include "common.h"
#include "messages.h"
#include "wire.h"

using namespace carl;

void setUp() {}
void tearDown() {}

// ── Little-endian primitives ────────────────────────────────────────────────
void test_i16_le_roundtrip() {
  uint8_t b[2];
  put_i16(b, -1234);
  TEST_ASSERT_EQUAL_INT16(-1234, get_i16(b));
  put_i16(b, 0x0102);
  TEST_ASSERT_EQUAL_UINT8(0x02, b[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, b[1]);
}

void test_i32_le_roundtrip() {
  uint8_t b[4];
  put_i32(b, -123456789);
  TEST_ASSERT_EQUAL_INT32(-123456789, get_i32(b));
  put_i32(b, 0x04030201);
  TEST_ASSERT_EQUAL_UINT8(0x01, b[0]);
  TEST_ASSERT_EQUAL_UINT8(0x04, b[3]);
}

// ── Throttle command ────────────────────────────────────────────────────────
void test_throttle_cmd_roundtrip() {
  ThrottleCmd in = {1000, -1000, 250, -250};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(8, pack_throttle_cmd(buf, in));

  ThrottleCmd out;
  TEST_ASSERT_TRUE(unpack_throttle_cmd(buf, 8, &out));
  TEST_ASSERT_EQUAL_INT16(in.fl, out.fl);
  TEST_ASSERT_EQUAL_INT16(in.fr, out.fr);
  TEST_ASSERT_EQUAL_INT16(in.rl, out.rl);
  TEST_ASSERT_EQUAL_INT16(in.rr, out.rr);
}

void test_throttle_cmd_rejects_short() {
  uint8_t buf[8] = {0};
  ThrottleCmd out;
  TEST_ASSERT_FALSE(unpack_throttle_cmd(buf, 7, &out));
}

// ── Throttle status ─────────────────────────────────────────────────────────
void test_throttle_status_roundtrip() {
  ThrottleStatus in = {TST_DRIVING, TF_NONE, 600, -600};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(6, pack_throttle_status(buf, in));

  ThrottleStatus out;
  TEST_ASSERT_TRUE(unpack_throttle_status(buf, 6, &out));
  TEST_ASSERT_EQUAL_UINT8(in.state, out.state);
  TEST_ASSERT_EQUAL_UINT8(in.faults, out.faults);
  TEST_ASSERT_EQUAL_INT16(in.applied_left, out.applied_left);
  TEST_ASSERT_EQUAL_INT16(in.applied_right, out.applied_right);
}

// ── Encoder feedback ────────────────────────────────────────────────────────
void test_encoder_fb_roundtrip() {
  EncoderFb in = {123456, -987654};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(8, pack_encoder_fb(buf, in));

  EncoderFb out;
  TEST_ASSERT_TRUE(unpack_encoder_fb(buf, 8, &out));
  TEST_ASSERT_EQUAL_INT32(in.left_count, out.left_count);
  TEST_ASSERT_EQUAL_INT32(in.right_count, out.right_count);
}

// ── Heartbeat ───────────────────────────────────────────────────────────────
void test_heartbeat_roundtrip() {
  Heartbeat in = {42, (uint8_t)(HB_FLAG_OK | HB_FLAG_TELEOP_OK), MODE_TELEOP};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(3, pack_heartbeat(buf, in));

  Heartbeat out;
  TEST_ASSERT_TRUE(unpack_heartbeat(buf, 3, &out));
  TEST_ASSERT_EQUAL_UINT8(in.counter, out.counter);
  TEST_ASSERT_EQUAL_UINT8(in.flags, out.flags);
  TEST_ASSERT_EQUAL_UINT8(in.mode, out.mode);
}

// ── Wire framing ────────────────────────────────────────────────────────────
void test_wire_frame_and_parse() {
  const uint16_t id = CANID_THROTTLE_CMD;
  ThrottleCmd cmd = {500, -500, 250, -250};
  uint8_t payload[8];
  uint8_t plen = pack_throttle_cmd(payload, cmd);

  uint8_t frame[WIRE_MAX_FRAME];
  uint8_t flen = wire_frame(frame, id, payload, plen);
  TEST_ASSERT_EQUAL_UINT8(WIRE_HEADER_LEN + 8 + 1, flen);
  TEST_ASSERT_EQUAL_UINT8(WIRE_SOF, frame[0]);

  WireParser p;
  WireParser::Result r = {false, 0, 0, nullptr};
  for (uint8_t i = 0; i < flen; i++) {
    r = p.feed(frame[i]);
  }
  TEST_ASSERT_TRUE(r.valid);
  TEST_ASSERT_EQUAL_UINT16(id, r.msg_id);
  TEST_ASSERT_EQUAL_UINT8(8, r.len);

  ThrottleCmd out;
  TEST_ASSERT_TRUE(unpack_throttle_cmd(r.payload, r.len, &out));
  TEST_ASSERT_EQUAL_INT16(cmd.fl, out.fl);
  TEST_ASSERT_EQUAL_INT16(cmd.rr, out.rr);
}

void test_wire_rejects_bad_crc() {
  const uint16_t id = CANID_ESTOP;
  uint8_t payload[1] = {ESTOP_OPERATOR};
  uint8_t frame[WIRE_MAX_FRAME];
  uint8_t flen = wire_frame(frame, id, payload, 1);

  frame[flen - 1] ^= 0xFF;  // corrupt CRC

  WireParser p;
  WireParser::Result r = {false, 0, 0, nullptr};
  for (uint8_t i = 0; i < flen; i++) r = p.feed(frame[i]);
  TEST_ASSERT_FALSE(r.valid);
}

void test_wire_resyncs_after_garbage() {
  const uint16_t id = CANID_HEARTBEAT;
  Heartbeat hb_in = {7, HB_FLAG_OK, MODE_IDLE};
  uint8_t payload[8];
  uint8_t plen = pack_heartbeat(payload, hb_in);
  uint8_t frame[WIRE_MAX_FRAME];
  uint8_t flen = wire_frame(frame, id, payload, plen);

  WireParser p;
  // Feed junk that isn't SOF — parser should stay in S_SOF.
  p.feed(0x00); p.feed(0x11); p.feed(0x22);
  WireParser::Result r = {false, 0, 0, nullptr};
  for (uint8_t i = 0; i < flen; i++) r = p.feed(frame[i]);
  TEST_ASSERT_TRUE(r.valid);
  TEST_ASSERT_EQUAL_UINT16(id, r.msg_id);
}

// ── Differential mix sanity (now lives in ROS2, but math is still valid) ────
void test_clamp_and_mix() {
  int16_t vx = 900, wz = 300;
  TEST_ASSERT_EQUAL_INT16(1000, clampv<int16_t>(vx + wz, THROTTLE_MIN, THROTTLE_MAX));
  TEST_ASSERT_EQUAL_INT16(600, clampv<int16_t>(vx - wz, THROTTLE_MIN, THROTTLE_MAX));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_i16_le_roundtrip);
  RUN_TEST(test_i32_le_roundtrip);
  RUN_TEST(test_throttle_cmd_roundtrip);
  RUN_TEST(test_throttle_cmd_rejects_short);
  RUN_TEST(test_throttle_status_roundtrip);
  RUN_TEST(test_encoder_fb_roundtrip);
  RUN_TEST(test_heartbeat_roundtrip);
  RUN_TEST(test_wire_frame_and_parse);
  RUN_TEST(test_wire_rejects_bad_crc);
  RUN_TEST(test_wire_resyncs_after_garbage);
  RUN_TEST(test_clamp_and_mix);
  return UNITY_END();
}
