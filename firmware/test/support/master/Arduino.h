#pragma once
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>
namespace fake_master {
inline uint32_t now = 0;
}
inline uint32_t millis() { return fake_master::now; }
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
