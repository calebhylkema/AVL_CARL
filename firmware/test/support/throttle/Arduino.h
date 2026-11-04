#pragma once
#include <cassert>
#include <cstdint>
namespace fake_io {
inline uint32_t now = 0;
inline int mode[64] = {}, level[64] = {}, pwm[64] = {};
inline uint32_t frequency[64] = {};
inline uint8_t resolution = 0;
}
static const int LOW = 0, HIGH = 1, OUTPUT = 1, LED_BUILTIN = 13;
#define F(s) s
inline uint32_t millis() { return fake_io::now; }
inline void pinMode(uint8_t p, int v) { assert(p < 64); fake_io::mode[p] = v; }
inline void digitalWrite(uint8_t p, int v) { assert(p < 64); fake_io::level[p] = v; }
inline int digitalRead(uint8_t p) { return fake_io::level[p]; }
inline void analogWrite(uint8_t p, int v) {
  assert(p < 64);
  // Actual bridge PWM pairs are (2,3), (4,5), (6,7), (8,9).
  if (p >= 2 && p <= 9 && v) assert(fake_io::pwm[p ^ 1] == 0);
  fake_io::pwm[p] = v;
}
inline void analogWriteFrequency(uint8_t p, uint32_t hz) { fake_io::frequency[p] = hz; }
inline void analogWriteResolution(uint8_t bits) { fake_io::resolution = bits; }
struct FakeSerial { void begin(int) {} void println(const char*) {} };
inline FakeSerial Serial;
