#pragma once
#include <Arduino.h>
#include <algorithm>
enum { MSBFIRST = 1, SPI_MODE3 = 3 };
struct SPISettings {
  SPISettings(uint32_t clock, int order, int mode) {
    assert(clock == 1000000 && order == MSBFIRST && mode == SPI_MODE3);
  }
};
struct SPIClass {
  uint32_t mosi, miso, sck;
  uint8_t registers[128] = {}, address = 0;
  bool active = false, first = false, reading = false, ignoreWrites = false;
  unsigned transactions = 0, writes = 0;
  SPIClass(uint32_t o, uint32_t i, uint32_t c) : mosi(o), miso(i), sck(c) {}
  void begin() {}
  void beginTransaction(SPISettings) { assert(!active); active = first = true; ++transactions; }
  void endTransaction() { assert(active); active = false; }
  uint8_t transfer(uint8_t v) {
    assert(active && digitalRead(PE4) == LOW);
    if (first) { first = false; reading = v & 0x80; address = v & 0x7f; return 0; }
    const uint8_t reg = address++ & 0x7f;
    if (reading) {
      const auto result = registers[reg];
      if (reg == 0x3a) registers[reg] = 0;
      return result;
    }
    ++writes;
    if (!ignoreWrites) {
      if (reg == 0x6b && v == 0x80) {
        const auto identity = registers[0x75];
        std::fill(std::begin(registers), std::end(registers), 0);
        registers[0x75] = identity; registers[0x6b] = 0x40;
      } else registers[reg] = v;
    }
    return 0;
  }
};
