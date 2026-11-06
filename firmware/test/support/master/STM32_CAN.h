#pragma once
#include <cstdint>
#include <deque>
#include <vector>
struct FakeCanRegisters {
  // bxCAN acknowledges initialization requests in hardware. Model that
  // handshake so the actual setup()/ABOM path can be exercised on the host.
  struct Control {
    uint32_t* status;
    uint32_t value = 0;
    operator uint32_t() const { return value; }
    void operator|=(uint32_t bits) { value |= bits; *status = value & 1; }
    void operator&=(uint32_t bits) { value &= bits; *status = value & 1; }
  };
  uint32_t MSR = 0;
  Control MCR{&MSR};
};
inline FakeCanRegisters fake_can_registers;
#define CAN1 (&fake_can_registers)
static const uint32_t CAN_MCR_INRQ = 1, CAN_MCR_ABOM = 64, CAN_MSR_INAK = 1;
enum { ALT_2 = 2 };
struct CAN_message_t { uint32_t id = 0; uint8_t len = 0; uint8_t buf[8] = {}; };
struct STM32_CAN {
  std::vector<CAN_message_t> sent;
  std::deque<CAN_message_t> rx;
  STM32_CAN(FakeCanRegisters*, int) {}
  void begin() {} void setBaudRate(uint32_t) {}
  void write(const CAN_message_t& frame) { sent.push_back(frame); }
  bool read(CAN_message_t& frame) {
    if (rx.empty()) return false;
    frame = rx.front(); rx.pop_front(); return true;
  }
};
