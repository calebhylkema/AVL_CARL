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
#include "imu_telemetry.h"
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

void test_imu_axis_telemetry_layout_and_roundtrip() {
  const ImuAxisTelemetry input = {
      0xA5,
      (uint8_t)(IMU_STATUS_INITIALIZED | IMU_STATUS_SAMPLE_VALID),
      0x1234,
      -2,
      INT16_MIN,
  };

  uint8_t payload[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(8, pack_imu_axis(payload, input));

  // Confirm the exact documented little-endian wire representation.
  TEST_ASSERT_EQUAL_HEX8(0xA5, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x03, payload[1]);
  TEST_ASSERT_EQUAL_HEX8(0x34, payload[2]);
  TEST_ASSERT_EQUAL_HEX8(0x12, payload[3]);
  TEST_ASSERT_EQUAL_HEX8(0xFE, payload[4]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, payload[5]);
  TEST_ASSERT_EQUAL_HEX8(0x00, payload[6]);
  TEST_ASSERT_EQUAL_HEX8(0x80, payload[7]);

  ImuAxisTelemetry output;
  TEST_ASSERT_TRUE(unpack_imu_axis(payload, sizeof(payload), &output));
  TEST_ASSERT_EQUAL_UINT8(input.sequence, output.sequence);
  TEST_ASSERT_EQUAL_UINT8(input.status, output.status);
  TEST_ASSERT_EQUAL_INT16(input.x, output.x);
  TEST_ASSERT_EQUAL_INT16(input.y, output.y);
  TEST_ASSERT_EQUAL_INT16(input.z, output.z);
}

void test_imu_axis_telemetry_rejects_invalid_input() {
  uint8_t payload[9] = {0};
  ImuAxisTelemetry output = {9, 8, 7, 6, 5};

  TEST_ASSERT_FALSE(unpack_imu_axis(payload, 7, &output));
  TEST_ASSERT_FALSE(unpack_imu_axis(payload, 9, &output));
  TEST_ASSERT_FALSE(unpack_imu_axis(nullptr, 8, &output));
  TEST_ASSERT_FALSE(unpack_imu_axis(payload, 8, nullptr));

  // Rejected payloads must not partially change the previous output value.
  TEST_ASSERT_EQUAL_UINT8(9, output.sequence);
  TEST_ASSERT_EQUAL_INT16(5, output.z);
}

void test_imu_temperature_telemetry_layout_and_roundtrip() {
  const ImuTemperatureTelemetry input = {0x5A, 0x03, -1234};
  uint8_t payload[4] = {0};

  TEST_ASSERT_EQUAL_UINT8(4, pack_imu_temperature(payload, input));
  TEST_ASSERT_EQUAL_HEX8(0x5A, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x03, payload[1]);
  TEST_ASSERT_EQUAL_HEX8(0x2E, payload[2]);
  TEST_ASSERT_EQUAL_HEX8(0xFB, payload[3]);

  ImuTemperatureTelemetry output;
  TEST_ASSERT_TRUE(
      unpack_imu_temperature(payload, sizeof(payload), &output));
  TEST_ASSERT_EQUAL_UINT8(input.sequence, output.sequence);
  TEST_ASSERT_EQUAL_UINT8(input.status, output.status);
  TEST_ASSERT_EQUAL_INT16(input.temperature, output.temperature);
}

void test_imu_temperature_telemetry_rejects_invalid_input() {
  uint8_t payload[5] = {0};
  ImuTemperatureTelemetry output = {9, 8, 7};

  TEST_ASSERT_FALSE(unpack_imu_temperature(payload, 3, &output));
  TEST_ASSERT_FALSE(unpack_imu_temperature(payload, 5, &output));
  TEST_ASSERT_FALSE(unpack_imu_temperature(nullptr, 4, &output));
  TEST_ASSERT_FALSE(unpack_imu_temperature(payload, 4, nullptr));

  TEST_ASSERT_EQUAL_UINT8(9, output.sequence);
  TEST_ASSERT_EQUAL_INT16(7, output.temperature);
}

void test_imu_wire_ids_frame_and_parse() {
  const ImuAxisTelemetry input = {255, 0x03, 100, -200, 300};
  uint8_t payload[8];
  uint8_t payload_len = pack_imu_axis(payload, input);

  const uint16_t ids[2] = {WIREID_IMU_ACCEL, WIREID_IMU_GYRO};
  for (uint8_t id_index = 0; id_index < 2; ++id_index) {
    uint8_t frame[WIRE_MAX_FRAME];
    uint8_t frame_len = wire_frame(frame, ids[id_index], payload, payload_len);

    WireParser parser;
    WireParser::Result result = {false, 0, 0, nullptr};
    for (uint8_t i = 0; i < frame_len; ++i) result = parser.feed(frame[i]);

    TEST_ASSERT_TRUE(result.valid);
    TEST_ASSERT_EQUAL_UINT16(ids[id_index], result.msg_id);
    TEST_ASSERT_EQUAL_UINT8(8, result.len);

    ImuAxisTelemetry output;
    TEST_ASSERT_TRUE(unpack_imu_axis(result.payload, result.len, &output));
    TEST_ASSERT_EQUAL_UINT8(255, output.sequence);
    TEST_ASSERT_EQUAL_INT16(-200, output.y);
  }
}

void test_imu_temperature_wire_frame_and_parse() {
  const ImuTemperatureTelemetry input = {255, 0x03, 3339};
  uint8_t payload[4];
  const uint8_t payload_len = pack_imu_temperature(payload, input);
  uint8_t frame[WIRE_MAX_FRAME];
  const uint8_t frame_len = wire_frame(
      frame, WIREID_IMU_TEMPERATURE, payload, payload_len);

  WireParser parser;
  WireParser::Result result = {false, 0, 0, nullptr};
  for (uint8_t i = 0; i < frame_len; ++i) result = parser.feed(frame[i]);

  TEST_ASSERT_TRUE(result.valid);
  TEST_ASSERT_EQUAL_UINT16(WIREID_IMU_TEMPERATURE, result.msg_id);
  TEST_ASSERT_EQUAL_UINT8(4, result.len);

  ImuTemperatureTelemetry output;
  TEST_ASSERT_TRUE(
      unpack_imu_temperature(result.payload, result.len, &output));
  TEST_ASSERT_EQUAL_UINT8(255, output.sequence);
  TEST_ASSERT_EQUAL_INT16(3339, output.temperature);
}

void test_imu_diagnostics_layout_and_framing() {
  const ImuDiagnostics diagnostic = {8, 1, 0x68, 0x70, 2, 0x75, 0x1234};
  uint8_t payload[8];
  TEST_ASSERT_EQUAL_UINT8(8, pack_imu_diagnostics(payload, diagnostic));
  const uint8_t expected[] = {8, 1, 0x68, 0x70, 2, 0x75, 0x34, 0x12};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, payload, 8);
  uint8_t frame[WIRE_MAX_FRAME];
  const uint8_t len = wire_frame(frame, WIREID_IMU_DIAGNOSTICS, payload, 8);
  WireParser parser;
  WireParser::Result result = {false, 0, 0, nullptr};
  for (uint8_t i = 0; i < len; ++i) result = parser.feed(frame[i]);
  TEST_ASSERT_TRUE(result.valid);
  TEST_ASSERT_EQUAL_UINT16(0x233, result.msg_id);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, result.payload, 8);
}

void test_bno_ids_preserve_si_payloads_and_legacy_separation() {
  const uint16_t ids[] = {WIREID_BNO_ACCEL, WIREID_BNO_GYRO,
      WIREID_BNO_QUAT_XYZ, WIREID_BNO_QUAT_W, WIREID_BNO_DIAGNOSTICS};
  const uint8_t payload[8] = {0xff, 0x33, 0x00, 0x80, 0xff, 0x7f, 0, 0};
  for (unsigned i = 0; i < 5; ++i) {
    TEST_ASSERT_EQUAL_UINT16(0x234 + i, ids[i]);
    const uint8_t size = i == 3 ? 6 : 8;
    uint8_t frame[WIRE_MAX_FRAME];
    uint8_t n = wire_frame(frame, ids[i], payload, size);
    WireParser parser;
    WireParser::Result result = {false, 0, 0, nullptr};
    for (uint8_t j = 0; j < n; ++j) result = parser.feed(frame[j]);
    TEST_ASSERT_TRUE(result.valid);
    TEST_ASSERT_EQUAL_UINT16(ids[i], result.msg_id);
    TEST_ASSERT_EQUAL_UINT8(size, result.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, result.payload, size);
  }
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
  RUN_TEST(test_imu_axis_telemetry_layout_and_roundtrip);
  RUN_TEST(test_imu_axis_telemetry_rejects_invalid_input);
  RUN_TEST(test_imu_temperature_telemetry_layout_and_roundtrip);
  RUN_TEST(test_imu_temperature_telemetry_rejects_invalid_input);
  RUN_TEST(test_imu_wire_ids_frame_and_parse);
  RUN_TEST(test_imu_temperature_wire_frame_and_parse);
  RUN_TEST(test_imu_diagnostics_layout_and_framing);
  RUN_TEST(test_bno_ids_preserve_si_payloads_and_legacy_separation);
  return UNITY_END();
}
