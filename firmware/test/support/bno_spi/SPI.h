#pragma once
#include <stdint.h>
#include <deque>
#include <vector>
enum { MSBFIRST = 1, SPI_MODE3 = 3 };
struct SPISettings {
  SPISettings(uint32_t clock, int order, int mode) {
    assert(clock == 1000000 && order == MSBFIRST && mode == SPI_MODE3);
  }
};
struct SPIClass {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  bool active = false;
  unsigned transactions = 0;
  void begin() {}
  void beginTransaction(SPISettings) { assert(!active); active = true; ++transactions; }
  void endTransaction() { assert(active); active = false; }
  uint8_t transfer(uint8_t value) {
    assert(active && gpio::level[0] == LOW); tx.push_back(value);
    if (rx.empty()) return 0;
    uint8_t answer = rx.front(); rx.pop_front(); return answer;
  }
};
