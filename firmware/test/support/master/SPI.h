#pragma once
#include <Arduino.h>

enum { MSBFIRST = 1, SPI_MODE3 = 3 };
struct SPISettings {
  SPISettings(uint32_t clock, int order, int mode) {
    assert(clock == 1000000 && order == MSBFIRST && mode == SPI_MODE3);
  }
};
struct SPIClass {
  const uint32_t mosi, miso, sck;
  bool begun = false, active = false;
  unsigned transactions = 0;
  std::deque<uint8_t> rx;
  std::vector<std::vector<uint8_t>> tx;
  SPIClass(uint32_t out, uint32_t in, uint32_t clock) : mosi(out), miso(in), sck(clock) {}
  void begin() { begun = true; }
  void beginTransaction(SPISettings) {
    assert(!active); active = true; ++transactions; tx.emplace_back();
  }
  void endTransaction() { assert(active); active = false; }
  uint8_t transfer(uint8_t value) {
    assert(active && digitalRead(PE4) == LOW);
    tx.back().push_back(value);
    if (rx.empty()) return 0;
    const auto answer = rx.front(); rx.pop_front(); return answer;
  }
};
