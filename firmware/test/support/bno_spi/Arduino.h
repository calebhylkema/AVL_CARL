#pragma once
#include <stdint.h>
#include <assert.h>
enum { LOW = 0, HIGH = 1, INPUT_PULLUP = 2, OUTPUT = 3 };
namespace gpio {
static int level[4] = {HIGH, HIGH, LOW, HIGH}; // CS, INT, RST, WAKE
}
inline void digitalWrite(uint32_t pin, int value) {
  assert(pin < 4); gpio::level[pin] = value;
}
inline int digitalRead(uint32_t pin) { assert(pin < 4); return gpio::level[pin]; }
inline void pinMode(uint32_t pin, int) { assert(pin < 4); }
