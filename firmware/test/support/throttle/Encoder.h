#pragma once
#include <cstdint>
#include <cassert>
namespace fake_encoder {
inline int32_t countByPinA[64] = {};
}
struct Encoder {
  const uint8_t pinA, pinB;
  int32_t& count;
  Encoder(uint8_t a, uint8_t b) : pinA(a), pinB(b), count(fake_encoder::countByPinA[a]) {
    assert(a < 64 && b < 64);
  }
  int32_t read() { return count; }
};
