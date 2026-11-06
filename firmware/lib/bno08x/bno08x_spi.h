#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "bno08x.h"

class Bno08xSpi : public Bno08xTransport {
 public:
  enum ReadResult : uint8_t { NO_READ, EMPTY, PACKET, BAD_LENGTH };
  // Snapshot of the last attempted receive, before CS is asserted. Persist
  // across sensor resets so recovery does not erase the failing header.
  // Schema 1: version,result,header[4],line bits,reserved (8 wire bytes).
  void packReadDiagnostics(uint8_t* out) const {
    out[0] = 1;
    out[1] = read_result_;
    for (unsigned i = 0; i < 4; ++i) out[2 + i] = last_header_[i];
    out[6] = read_lines_;
    out[7] = 0;
  }
  Bno08xSpi(SPIClass& spi, uint32_t cs, uint32_t interrupt,
            uint32_t reset, uint32_t wake)
      : spi_(spi), cs_(cs), int_(interrupt), reset_(reset), wake_(wake) {}
  void begin() {
    // Preload levels before enabling outputs to avoid a spurious CS/reset.
    digitalWrite(cs_, HIGH); pinMode(cs_, OUTPUT);
    digitalWrite(wake_, HIGH); pinMode(wake_, OUTPUT);
    digitalWrite(reset_, LOW); pinMode(reset_, OUTPUT);
    pinMode(int_, INPUT_PULLUP);
    spi_.begin();
  }
  void reset(bool asserted) override { digitalWrite(reset_, asserted ? LOW : HIGH); }
  void wake(bool asserted) override {
    waking_ = asserted;
    digitalWrite(wake_, asserted ? LOW : HIGH);
  }
  bool interruptActive() override { return digitalRead(int_) == LOW; }
  int read(uint8_t* packet, size_t capacity) override {
    // An INT caused by our WAKE is permission to transmit. Do not consume
    // that handshake as an empty receive before the pending write executes.
    if (waking_ || !interruptActive()) return 0;
    read_lines_ = (digitalRead(int_) == LOW ? 1 : 0)
        | (digitalRead(reset_) == HIGH ? 2 : 0)
        | (digitalRead(wake_) == HIGH ? 4 : 0)
        | (digitalRead(cs_) == HIGH ? 8 : 0);
    open();
    for (unsigned i = 0; i < 4; ++i) {
      packet[i] = spi_.transfer(0);
      last_header_[i] = packet[i];
    }
    const uint16_t length = (uint16_t(packet[0]) | (uint16_t(packet[1]) << 8)) & 0x7fff;
    if (length == 0) { read_result_ = EMPTY; close(); return 0; }
    if (length < 4 || length > capacity) {
      read_result_ = BAD_LENGTH;
      close(); return -1;
    }
    read_result_ = PACKET;
    for (unsigned i = 4; i < length; ++i) packet[i] = spi_.transfer(0);
    close();
    return length;
  }
  bool write(uint8_t channel, uint8_t sequence,
             const uint8_t* data, uint8_t length) override {
    wake(true);
    if (!interruptActive()) return false;
    open();
    spi_.transfer(uint8_t(length + 4)); spi_.transfer(0);
    spi_.transfer(channel); spi_.transfer(sequence);
    for (unsigned i = 0; i < length; ++i) spi_.transfer(data[i]);
    close();
    wake(false);
    return true;
  }
 private:
  void open() {
    spi_.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_, LOW);
  }
  void close() { digitalWrite(cs_, HIGH); spi_.endTransaction(); }
  SPIClass& spi_;
  uint32_t cs_, int_, reset_, wake_;
  bool waking_ = false;
  uint8_t last_header_[4] = {};
  uint8_t read_result_ = NO_READ, read_lines_ = 0;
};
