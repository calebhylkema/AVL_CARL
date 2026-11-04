#pragma once
#include <cstdint>
#include <vector>
#include <deque>
static const int CAN1 = 1, RX_SIZE_256 = 256, TX_SIZE_16 = 16;
struct CAN_message_t { uint32_t id = 0; uint8_t len = 0; uint8_t buf[8] = {}; };
template <int, int, int> struct FlexCAN_T4 {
  std::vector<CAN_message_t> sent;
  std::deque<CAN_message_t> incoming;
  uint32_t advanceOnRead = 0;
  void begin() {} void setBaudRate(uint32_t) {} void setMaxMB(int) {} void enableFIFO() {}
  bool read(CAN_message_t& message) {
    if (incoming.empty()) return false;
    fake_io::now += advanceOnRead;
    message = incoming.front(); incoming.pop_front(); return true;
  }
  void write(const CAN_message_t& msg) { sent.push_back(msg); }
};
