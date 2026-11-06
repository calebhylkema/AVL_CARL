#include "bno08x_spi.h"
#include <stdio.h>

int main() {
  SPIClass spi; Bno08xSpi bus(spi, 0, 1, 2, 3); bus.begin();
  uint8_t packet[512];
  assert(gpio::level[0] == HIGH && gpio::level[2] == LOW);
  assert(bus.read(packet, sizeof(packet)) == 0 && spi.transactions == 0);
  uint8_t diagnostic[8];
  bus.packReadDiagnostics(diagnostic);
  assert(diagnostic[0] == 1 && diagnostic[1] == Bno08xSpi::NO_READ);
  bus.reset(false);
  gpio::level[1] = LOW;
  spi.rx = {0, 0, 0, 0}; // Empty packet must still release CS.
  assert(bus.read(packet, sizeof(packet)) == 0);
  assert(gpio::level[0] == HIGH && !spi.active);
  bus.packReadDiagnostics(diagnostic);
  assert(diagnostic[1] == Bno08xSpi::EMPTY && diagnostic[6] == 15);
  for (unsigned i = 2; i < 6; ++i) assert(diagnostic[i] == 0);
  spi.rx = {0xff, 0x7f, 3, 0};
  assert(bus.read(packet, sizeof(packet)) == -1);
  assert(gpio::level[0] == HIGH && spi.rx.empty()); // Only four bytes clocked.
  bus.packReadDiagnostics(diagnostic);
  assert(diagnostic[1] == Bno08xSpi::BAD_LENGTH && diagnostic[2] == 0xff);
  assert(diagnostic[3] == 0x7f && diagnostic[4] == 3 && diagnostic[6] == 15);
  bus.reset(true); // Failure recovery must preserve the original pin/header snapshot.
  bus.packReadDiagnostics(diagnostic);
  assert(diagnostic[1] == Bno08xSpi::BAD_LENGTH && diagnostic[6] == 15);
  bus.reset(false);
  spi.rx = {6, 0, 2, 0, 0xf8, 0};
  assert(bus.read(packet, sizeof(packet)) == 6 && packet[4] == 0xf8);
  bus.packReadDiagnostics(diagnostic);
  assert(diagnostic[1] == Bno08xSpi::PACKET && diagnostic[2] == 6);
  assert(diagnostic[3] == 0 && diagnostic[4] == 2 && diagnostic[7] == 0);
  assert(gpio::level[0] == HIGH && !spi.active);
  gpio::level[1] = HIGH;
  const uint8_t data[] = {0xf9, 0};
  assert(!bus.write(2, 0, data, 2));
  assert(gpio::level[3] == LOW); // Asynchronous WAKE awaiting INT.
  gpio::level[1] = LOW;
  const unsigned before = spi.transactions;
  assert(bus.read(packet, sizeof(packet)) == 0 && spi.transactions == before);
  spi.tx.clear();
  assert(bus.write(2, 7, data, 2));
  assert((spi.tx == std::vector<uint8_t>{6, 0, 2, 7, 0xf9, 0}));
  assert(gpio::level[3] == HIGH && gpio::level[0] == HIGH && !spi.active);
  puts("BNO08x SPI: framing, bounds, chip-select and asynchronous wake passed");
}
