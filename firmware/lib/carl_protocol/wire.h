// ─────────────────────────────────────────────────────────────────────────────
// wire.h — framed binary link between the master and a companion computer.
//
// Most traffic is proxied unchanged between the companion and CAN, so those
// wire message IDs reuse their CAN IDs. The master may also send locally
// measured, wire-only telemetry such as the two IMU messages declared in
// imu_telemetry.h. Wire-only messages are never placed on the vehicle CAN bus.
//
// Frame layout (little-endian):
//   [SOF=0xAA][MSG_ID: u16 LE][LEN: u8][PAYLOAD ... LEN bytes][CRC8]
//   CRC8 covers header + payload (poly 0x07, init 0).
//
// This header MUST stay in sync with carl_bridge/carl_bridge/protocol.py on the
// ROS2 side. The native test `test_wire_framing` guards known-good bytes.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

namespace carl {

static const uint8_t  WIRE_SOF        = 0xAA;
static const uint8_t  WIRE_CRC_POLY   = 0x07;
static const uint8_t  WIRE_HEADER_LEN = 4;                    // SOF + id(2) + len(1)
static const uint8_t  WIRE_MAX_PAYLOAD = 8;                   // max CAN payload
static const uint8_t  WIRE_MAX_FRAME   = WIRE_HEADER_LEN + WIRE_MAX_PAYLOAD + 1;

inline uint8_t wire_crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ WIRE_CRC_POLY) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Serialize (msg_id, payload) into a wire frame. Returns bytes written.
// out must be at least WIRE_MAX_FRAME bytes.
inline uint8_t wire_frame(uint8_t* out, uint16_t msg_id, const uint8_t* payload, uint8_t len) {
  if (len > WIRE_MAX_PAYLOAD) len = WIRE_MAX_PAYLOAD;
  out[0] = WIRE_SOF;
  out[1] = (uint8_t)(msg_id & 0xFF);
  out[2] = (uint8_t)((msg_id >> 8) & 0xFF);
  out[3] = len;
  for (uint8_t i = 0; i < len; i++) out[WIRE_HEADER_LEN + i] = payload[i];
  out[WIRE_HEADER_LEN + len] = wire_crc8(out, WIRE_HEADER_LEN + len);
  return (uint8_t)(WIRE_HEADER_LEN + len + 1);
}

// Byte-at-a-time parser. Feed bytes as they arrive; a completed frame is
// signaled by a non-negative return from feed().
class WireParser {
 public:
  struct Result {
    bool     valid;
    uint16_t msg_id;
    uint8_t  len;
    const uint8_t* payload;
  };

  // Feed one byte. If a complete valid frame just finished, returns
  // {valid=true, ...}; caller must consume `payload` before the next feed().
  Result feed(uint8_t b) {
    Result r = {false, 0, 0, nullptr};
    switch (state_) {
      case S_SOF:
        if (b == WIRE_SOF) { buf_[0] = b; idx_ = 1; state_ = S_ID_LO; }
        break;
      case S_ID_LO:
        buf_[idx_++] = b; state_ = S_ID_HI; break;
      case S_ID_HI:
        buf_[idx_++] = b; state_ = S_LEN; break;
      case S_LEN:
        buf_[idx_++] = b;
        expected_len_ = b;
        if (expected_len_ > WIRE_MAX_PAYLOAD) { state_ = S_SOF; idx_ = 0; break; }
        state_ = (expected_len_ == 0) ? S_CRC : S_PAYLOAD;
        break;
      case S_PAYLOAD:
        buf_[idx_++] = b;
        if (idx_ == WIRE_HEADER_LEN + expected_len_) state_ = S_CRC;
        break;
      case S_CRC: {
        uint8_t want = wire_crc8(buf_, WIRE_HEADER_LEN + expected_len_);
        if (want == b) {
          r.valid = true;
          r.msg_id = (uint16_t)buf_[1] | ((uint16_t)buf_[2] << 8);
          r.len = expected_len_;
          r.payload = &buf_[WIRE_HEADER_LEN];
        }
        state_ = S_SOF;
        idx_ = 0;
        break;
      }
    }
    return r;
  }

 private:
  enum State { S_SOF, S_ID_LO, S_ID_HI, S_LEN, S_PAYLOAD, S_CRC };
  State   state_ = S_SOF;
  uint8_t buf_[WIRE_MAX_FRAME] = {0};
  uint8_t idx_ = 0;
  uint8_t expected_len_ = 0;
};

}  // namespace carl
