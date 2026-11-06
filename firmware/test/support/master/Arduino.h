#pragma once
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>
namespace fake_master {
inline uint32_t now = 0;
inline int level[96] = {};
inline int mode[96] = {};
}
enum { LOW = 0, HIGH = 1, INPUT_PULLUP = 2, OUTPUT = 3 };
enum { PE2 = 66, PE3 = 67, PE4 = 68, PE5 = 69, PE6 = 70, PF7 = 87, PF8 = 88 };
inline uint32_t millis() { return fake_master::now; }
inline void pinMode(uint32_t pin, int mode) {
  assert(pin < 96); fake_master::mode[pin] = mode;
  if (mode == INPUT_PULLUP) fake_master::level[pin] = HIGH;
}
inline void digitalWrite(uint32_t pin, int value) {
  assert(pin < 96); fake_master::level[pin] = value;
}
inline int digitalRead(uint32_t pin) { assert(pin < 96); return fake_master::level[pin]; }
struct FakeSerial {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  uint32_t advanceOnRead = 0;
  void begin(unsigned) {}
  unsigned available() const { return rx.size(); }
  uint8_t read() {
    fake_master::now += advanceOnRead;
    auto b = rx.front(); rx.pop_front(); return b;
  }
  void write(const uint8_t* p, unsigned n) { tx.insert(tx.end(), p, p + n); }
};
inline FakeSerial Serial;
