#pragma once
#include <cassert>
#include <cstdint>
namespace fake_io {
inline uint32_t now = 0;
inline int mode[64] = {}, level[64] = {}, highWrites[64] = {};
inline int adc = 0, adcReads = 0, adcBits = 0, averaging = 0;
}
static const int LOW = 0, HIGH = 1, OUTPUT = 1, INPUT = 2, LED_BUILTIN = 13;
#define F(s) s
inline uint32_t millis() { return fake_io::now; }
inline void pinMode(uint8_t p, int v) { assert(p < 64); fake_io::mode[p] = v; }
inline void digitalWrite(uint8_t p, int v) {
  assert(p < 64);
  fake_io::level[p] = v;
  if (v) ++fake_io::highWrites[p];
}
inline int digitalRead(uint8_t p) { return fake_io::level[p]; }
inline void analogReadResolution(int n) { fake_io::adcBits = n; }
inline void analogReadAveraging(int n) { fake_io::averaging = n; }
inline int analogRead(uint8_t p) {
  assert(p == 14 && fake_io::mode[p] == INPUT);
  ++fake_io::adcReads;
  return fake_io::adc;
}
struct FakeSerial { void begin(int) {} void println(const char*) {} };
inline FakeSerial Serial;
