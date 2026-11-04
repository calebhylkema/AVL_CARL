// Exercise the production steering loop, ADC pin and CAN payload.
#include "../../../steering/src/main.cpp"
#include "wire.h"
#include <cstdio>
#include <cstring>

static SteeringFeedback feedback() {
  const auto& frame = can1.sent.back();
  assert(frame.id == CANID_STEERING_FB && frame.len == 6);
  SteeringFeedback sample;
  assert(unpack_steering_feedback(frame.buf, frame.len, &sample));
  return sample;
}

int main() {
  setup();
  assert(fake_io::adcBits == 12 && fake_io::averaging == 16);
  fake_io::adc = 2048;
  fake_io::now = 19; loop();
  assert(fake_io::adcReads == 0);
  steeringSequence = 65535;
  fake_io::now = 20; loop();
  auto sample = feedback();
  assert(sample.raw_adc == 2048 && sample.sequence == 65535 && sample.status == SF_SAMPLE_VALID);
  const uint8_t golden[] = {1, 1, 0, 8, 255, 255};
  assert(std::memcmp(can1.sent.back().buf, golden, sizeof(golden)) == 0);
  uint8_t framed[WIRE_MAX_FRAME];
  const auto size = wire_frame(framed, CANID_STEERING_FB, golden, sizeof(golden));
  WireParser parser;
  bool parsed = false;
  for (uint8_t i = 0; i < size; ++i) {
    const auto result = parser.feed(framed[i]);
    if (result.valid) {
      assert(result.msg_id == 0x221 && result.len == 6);
      assert(std::memcmp(result.payload, golden, sizeof(golden)) == 0);
      parsed = true;
    }
  }
  assert(parsed);
  fake_io::adc = 0; fake_io::now = 40; loop();
  sample = feedback();
  assert(sample.status == SF_ADC_RAIL_LOW && sample.sequence == 0);
  fake_io::adc = 4095; fake_io::now = 60; loop();
  assert(feedback().status == SF_ADC_RAIL_HIGH);
  for (uint16_t id : {CANID_STEERING_CMD, CANID_ESTOP, CANID_HEARTBEAT}) {
    CAN_message_t rx;
    rx.id = id; rx.len = 8;
    std::memset(rx.buf, 255, 8);
    can1.incoming.push_back(rx);
  }
  fake_io::adc = 1; fake_io::now = 80; loop();
  assert(feedback().status == SF_SAMPLE_VALID);
  fake_io::now = 1000; loop(); // Missing master never enables the stepper.
  assert(fake_io::highWrites[PIN_STEP] == 0 && fake_io::highWrites[PIN_ENA] == 0);
  assert(fake_io::level[PIN_STEP] == LOW && fake_io::level[PIN_ENA] == LOW);
  bool legacy = false;
  for (const auto& frame : can1.sent) {
    if (frame.id == CANID_STEERING_STATUS) {
      assert(frame.len == 2 && frame.buf[0] == 0 && frame.buf[1] == 0);
      legacy = true;
    }
  }
  assert(legacy);
  lastSteeringFeedbackMs = UINT32_MAX - 9;
  fake_io::now = 10; loop();
  assert(feedback().raw_adc == 1); // 20 ms elapsed across millis wrap.
  SteeringFeedback out;
  assert(!unpack_steering_feedback(golden, 5, &out));
  assert(!unpack_steering_feedback(nullptr, 6, &out));
  assert(!unpack_steering_feedback(golden, 6, nullptr));
  uint8_t bad[6] = {2, 1, 0, 8, 0, 0};
  assert(!unpack_steering_feedback(bad, 6, &out));
  bad[0] = 1; bad[3] = 16;
  assert(!unpack_steering_feedback(bad, 6, &out));
  std::puts("Steering: ADC, wire/CAN framing, rail faults, rollover and motor-disabled behavior passed");
}
