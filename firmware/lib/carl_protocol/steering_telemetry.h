#pragma once

#include "common.h"

namespace carl {

static const uint8_t STEERING_SCHEMA = 1;
static const uint16_t STEERING_ADC_MAX = 4095;
enum SteeringFeedbackFlag : uint8_t {
  SF_SAMPLE_VALID = 0x01,
  SF_ADC_RAIL_LOW = 0x02,
  SF_ADC_RAIL_HIGH = 0x04,
};

// CAN 0x221: schema, flags, 12-bit ADC count (u16 LE), sequence (u16 LE).
// Conversion/calibration belongs to the bridge; no fabricated sensor health.
struct SteeringFeedback {
  uint8_t status;
  uint16_t raw_adc;
  uint16_t sequence;
};

inline uint8_t pack_steering_feedback(uint8_t* out, const SteeringFeedback& s) {
  out[0] = STEERING_SCHEMA;
  out[1] = s.status;
  out[2] = (uint8_t)s.raw_adc;
  out[3] = (uint8_t)(s.raw_adc >> 8);
  out[4] = (uint8_t)s.sequence;
  out[5] = (uint8_t)(s.sequence >> 8);
  return 6;
}

inline bool unpack_steering_feedback(const uint8_t* in, uint8_t len,
                                    SteeringFeedback* s) {
  if (!in || !s || len != 6 || in[0] != STEERING_SCHEMA) return false;
  s->status = in[1];
  s->raw_adc = (uint16_t)in[2] | ((uint16_t)in[3] << 8);
  s->sequence = (uint16_t)in[4] | ((uint16_t)in[5] << 8);
  return s->raw_adc <= STEERING_ADC_MAX;
}

}  // namespace carl
