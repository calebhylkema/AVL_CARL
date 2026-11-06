#include "bno08x.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <deque>
#include <vector>

struct Fake : Bno08xTransport {
  bool held_reset = false, waking = false, interrupt = true, writable = true;
  unsigned reads = 0;
  std::deque<std::vector<uint8_t>> rx;
  std::vector<std::vector<uint8_t>> tx;
  void reset(bool value) override { held_reset = value; }
  void wake(bool value) override { waking = value; }
  bool interruptActive() override { return interrupt; }
  int read(uint8_t* p, size_t cap) override {
    ++reads;
    if (rx.empty()) return 0;
    auto data = rx.front(); rx.pop_front();
    if (data.size() > cap) return -1;
    memcpy(p, data.data(), data.size());
    return data.size();
  }
  bool write(uint8_t ch, uint8_t seq, const uint8_t* p, uint8_t n) override {
    if (!writable) return false;
    std::vector<uint8_t> data = {ch, seq};
    data.insert(data.end(), p, p + n); tx.push_back(data);
    return true;
  }
  void packet(uint8_t ch, std::vector<uint8_t> p) {
    const uint16_t n = p.size() + 4;
    p.insert(p.begin(), {uint8_t(n), uint8_t(n >> 8), ch, 0});
    rx.push_back(p);
  }
  void report(uint8_t id, uint8_t seq = 0) {
    std::vector<uint8_t> data = {0xfb, 0, 0, 0, 0, id, seq, 3, 0};
    data.insert(data.end(), {0x80, 2, 0, 0, 0, 0});
    if (id == 5) data.insert(data.end(), {0, 0x40, 0, 0x10});
    if (id == 8) data.insert(data.end(), {0, 0x40});
    packet(3, data);
  }
};

void configure(Bno08x& d, Fake& t, uint32_t base = 0) {
  d.begin(t, base);
  assert(t.held_reset && !t.waking);
  d.poll(base + 99); assert(t.tx.empty());
  d.poll(base + 100); assert(d.stage() == Bno08x::RESET_LOW);
  d.poll(base + 102); assert(!t.held_reset);
  t.packet(1, {1}); // Executable Reset Complete, not just the first INT.
  d.poll(base + 103); assert(d.stage() == Bno08x::IDENTIFY);
  d.poll(base + 104); assert(t.tx.back()[2] == 0xf9);
  std::vector<uint8_t> product(16, 0); product[0] = 0xf8; product[4] = 0x34;
  t.packet(2, product); d.poll(base + 105);
  assert(d.product() == 0x34 && d.stage() == Bno08x::CONFIGURE);
  d.poll(base + 106); d.poll(base + 107); d.poll(base + 108); d.poll(base + 109);
  assert(t.tx.size() == 5 && d.stage() == Bno08x::STREAM_WAIT);
  for (int i = 1; i <= 4; ++i) {
    assert(t.tx[i][0] == 2 && t.tx[i][1] == i);
    assert(t.tx[i][2] == 0xfd);
    assert(t.tx[i][7] == 0x10 && t.tx[i][8] == 0x27); // 100 Hz
  }
  assert(t.tx[4][3] == 8); // Separate magnetometer-free orientation for control.
}

void reports(Bno08x& d, Fake& t, uint32_t now = 110, uint8_t seq = 0) {
  t.report(1, seq); t.report(2, seq); t.report(5, seq);
  d.poll(now); d.poll(now + 1); d.poll(now + 2);
}

int main() {
  { // Startup packets/INT may precede Reset Complete: do not send commands yet.
    Fake t; Bno08x d; d.begin(t, 0); d.poll(100); d.poll(102);
    d.poll(103);
    assert(d.stage() == Bno08x::BOOT_WAIT && t.tx.empty() && !t.waking);
    t.packet(0, {0}); // SHTP advertisement.
    t.packet(2, {0xf1, 0, 0x84, 0, 0, 0}); // Unsolicited initialize response.
    t.report(1); // Never carry pre-handshake data into the first sample.
    d.poll(104); d.poll(105); d.poll(106);
    assert(d.stage() == Bno08x::BOOT_WAIT && t.tx.empty() && !t.waking);
    t.packet(1, {1}); d.poll(107);
    assert(d.stage() == Bno08x::IDENTIFY && d.errorCount() == 0);
    d.poll(108);
    assert(t.tx.size() == 1 && t.tx[0][2] == 0xf9);
    std::vector<uint8_t> product(16, 0); product[0] = 0xf8; product[4] = 0x34;
    t.packet(2, product); d.poll(109);
    d.poll(110); d.poll(111); d.poll(112); d.poll(113);
    t.report(2); t.report(5); d.poll(114); d.poll(115);
    assert(!d.ready()); // The pre-handshake accelerometer report was ignored.
    t.report(1); d.poll(116);
    Bno08xSample s;
    assert(d.ready() && d.errorCount() == 0 && d.takeSample(s, 116));
  }
  { // An asserted INT without Reset Complete must time out without any writes.
    Fake t; Bno08x d; d.begin(t, 0); d.poll(100); d.poll(102);
    t.packet(0, {0}); d.poll(103); d.poll(1102);
    assert(d.error() == Bno08x::BOOT_TIMEOUT && t.held_reset && t.tx.empty());
  }
  { // Full startup, signed fixed-point reports, publication is consume-once.
    Fake t; Bno08x d; configure(d, t); reports(d, t);
    Bno08xSample s; assert(d.ready() && d.takeSample(s, 112));
    assert(s.accel[0] == 640 && s.gyro[0] == 640);
    assert(s.quaternion[3] == 16384 && s.heading_accuracy == 4096);
    assert(s.accuracy[2] == 3 && !d.takeSample(s, 113));
    reports(d, t, 120); // Replayed report sequence must not create new data.
    assert(!d.takeSample(s, 123));
    reports(d, t, 130, 1); assert(d.takeSample(s, 132));
  }
  { // Missing INT cannot block; only the IMU is reset after timeout.
    Fake t; Bno08x d; t.interrupt = false;
    d.begin(t, 0); d.poll(100); d.poll(102); d.poll(1102);
    assert(d.error() == Bno08x::BOOT_TIMEOUT && d.errorCount() == 1);
    assert(t.held_reset && t.reads == 0);
    d.poll(2101); assert(d.stage() == Bno08x::RETRY_WAIT);
    d.poll(2102); assert(d.stage() == Bno08x::RESET_LOW);
  }
  { // Identity/write timeout is bounded and stage-specific.
    Fake t; Bno08x d; d.begin(t, 0); d.poll(100); d.poll(102);
    t.packet(1, {1}); d.poll(103);
    t.writable = false; d.poll(104); d.poll(1103);
    assert(d.error() == Bno08x::IDENTIFY_TIMEOUT);
  }
  { // A missing report must not be hidden by other fresh reports.
    Fake t; Bno08x d; configure(d, t); reports(d, t);
    Bno08xSample s; assert(d.takeSample(s, 112));
    t.report(1, 1); d.poll(362);
    assert(!d.ready() && d.error() == Bno08x::REPORT_TIMEOUT);
    assert(!d.takeSample(s, 360));
  }
  { // Malformed lengths and incomplete reports never publish stale vectors.
    Fake t; Bno08x d; configure(d, t);
    t.packet(3, {0xfb, 0, 0, 0, 0, 1, 0, 3, 0, 1}); d.poll(110);
    assert(d.error() == Bno08x::INVALID_PACKET && d.detail() == 1);
  }
  { // Unsupported SHTP fragmentation is explicit, not silently truncated.
    Fake t; Bno08x d; configure(d, t);
    t.packet(3, {0}); t.rx.front()[1] |= 0x80; d.poll(110);
    assert(d.error() == Bno08x::UNSUPPORTED_FRAGMENT);
  }
  { // Sensor resets invalidate all partial samples and trigger recovery.
    Fake t; Bno08x d; configure(d, t); reports(d, t);
    t.packet(1, {1}); d.poll(120);
    assert(d.error() == Bno08x::SENSOR_RESET && !d.ready());
  }
  { // A new reset during STREAM_WAIT remains a real failure, not boot chatter.
    Fake t; Bno08x d; configure(d, t);
    t.packet(1, {1}); d.poll(110);
    assert(d.error() == Bno08x::SENSOR_RESET && d.errorCount() == 1);
    assert(d.failedStage() == Bno08x::STREAM_WAIT && t.held_reset);
  }
  { // Multiple reports in one SHTP packet must all be decoded.
    Fake t; Bno08x d; configure(d, t);
    t.report(1); t.report(2); t.report(5);
    auto packet = t.rx.front(); t.rx.pop_front();
    while (!t.rx.empty()) {
      auto next = t.rx.front(); t.rx.pop_front();
      packet.insert(packet.end(), next.begin() + 9, next.end());
    }
    packet[0] = packet.size(); t.rx.push_back(packet); d.poll(110);
    Bno08xSample s; assert(d.takeSample(s, 110));
  }
  { // millis rollover does not stall startup or falsely expire reports.
    Fake t; Bno08x d; const uint32_t base = UINT32_MAX - 105;
    configure(d, t, base); reports(d, t, base + 110);
    Bno08xSample s; assert(d.takeSample(s, base + 112));
  }
  { // Incomplete initial configuration never announces ready.
    Fake t; Bno08x d; configure(d, t);
    t.report(1); d.poll(110); d.poll(1109);
    assert(d.error() == Bno08x::REPORT_TIMEOUT && !d.ready());
  }
  { // Reinitialization restores real samples without clearing cumulative errors.
    Fake t; Bno08x d; configure(d, t); reports(d, t);
    d.poll(400); assert(d.errorCount() == 1 && !d.ready());
    d.poll(1400); d.poll(1402);
    t.packet(1, {1}); d.poll(1403); d.poll(1404);
    std::vector<uint8_t> product(16, 0); product[0] = 0xf8; product[4] = 0x34;
    t.packet(2, product); d.poll(1405);
    d.poll(1406); d.poll(1407); d.poll(1408); d.poll(1409);
    reports(d, t, 1410);
    Bno08xSample s;
    assert(d.ready() && d.error() == Bno08x::OK && d.errorCount() == 1);
    assert(d.takeSample(s, 1412));
  }
  { // Independent GRV samples, consume-once, duplicates, and stale gyro.
    Fake t; Bno08x d; configure(d, t); reports(d, t);
    t.report(8, 255); d.poll(113);
    Bno08xHeadingSample h;
    assert(d.takeHeadingSample(h, 113));
    assert(h.quaternion[3] == 16384 && h.accuracy == 3 && h.gyro_accuracy == 3);
    assert(h.orientation_ms == 113 && h.gyro_ms == 111 && h.gyro[0] == 640);
    assert(!d.takeHeadingSample(h, 113));
    t.report(8, 255); d.poll(114);
    assert(!d.takeHeadingSample(h, 114));
    t.report(8, 0); d.poll(115); // Per-report sequence rollover is fresh.
    assert(d.takeHeadingSample(h, 115));
    t.report(8, 1); d.poll(170);
    assert(!d.takeHeadingSample(h, 170)); // Current orientation, old gyro.
    t.report(2, 1); d.poll(171);
    assert(d.takeHeadingSample(h, 171));
    t.packet(1, {1}); d.poll(172);
    assert(!d.takeHeadingSample(h, 172));
  }
  { // Truncated 12-byte GRV must not be mistaken for a 10-byte vector.
    Fake t; Bno08x d; configure(d, t);
    t.packet(3, {8, 1, 3, 0, 0, 0, 0, 0, 0, 0}); d.poll(110);
    assert(d.error() == Bno08x::INVALID_PACKET && d.detail() == 8);
  }
  { // A sensor that never acknowledges WAKE cannot hang configuration.
    Fake t; Bno08x d; configure(d, t);
    // Retry into IDENTIFY, accept identity, then withhold all feature writes.
    t.packet(1, {1}); d.poll(110);
    d.poll(1110); d.poll(1112);
    t.packet(1, {1}); d.poll(1113); d.poll(1114);
    std::vector<uint8_t> product(16, 0); product[0] = 0xf8;
    t.packet(2, product); d.poll(1115);
    t.writable = false; d.poll(1116); d.poll(2115);
    assert(d.error() == Bno08x::CONFIGURE_TIMEOUT && d.detail() == 1);
    assert(t.held_reset && !t.waking && !d.ready());
  }
  { // Independently fresh reports are required at the 50 ms snapshot boundary.
    Fake t; Bno08x d; configure(d, t); reports(d, t);
    Bno08xSample s; assert(d.takeSample(s, 112));
    t.report(1, 1); d.poll(120);
    t.report(2, 1); d.poll(121);
    t.report(5, 1); d.poll(171);
    assert(!d.takeSample(s, 171)); // Old acceleration cannot be relabeled fresh.
    t.report(1, 2); d.poll(172);
    assert(!d.takeSample(s, 172)); // Old gyro is also rejected.
    t.report(2, 2); d.poll(173);
    assert(d.takeSample(s, 173) && !d.takeSample(s, 174));
  }
  { // Error counters saturate and repeated disconnects survive millis wrap.
    Fake t; Bno08x d; t.interrupt = false;
    uint32_t now = UINT32_MAX - 100;
    d.begin(t, now); now += 100; d.poll(now);
    for (unsigned i = 0; i < 65540; ++i) {
      now += 2; d.poll(now);
      now += 1000; d.poll(now);
      assert(d.stage() == Bno08x::RETRY_WAIT);
      now += 1000; d.poll(now);
    }
    assert(d.errorCount() == UINT16_MAX && !d.ready());
  }
  { // Deterministic malformed-packet corpus: bounds and parsing under sanitizers.
    uint32_t random = 0x0850767;
    for (unsigned trial = 0; trial < 2000; ++trial) {
      Fake t; Bno08x d; configure(d, t);
      random = random * 1664525U + 1013904223U;
      const unsigned length = random % (Bno08x::MAX_PACKET + 2);
      std::vector<uint8_t> data(length);
      for (auto& b : data) { random = random * 1664525U + 1013904223U; b = random >> 24; }
      // Half the inputs have valid framing to exercise deeper report parsing.
      if (length >= 4 && (trial & 1)) {
        data[0] = uint8_t(length); data[1] = uint8_t(length >> 8); data[2] = 3;
      }
      t.rx.push_back(data); d.poll(110);
      assert(t.rx.empty() && t.reads == 8); // One receive per poll, including init.
      Bno08xSample s;
      assert(!d.takeSample(s, 110)); // Random partial data cannot become a snapshot.
    }
  }
  puts("BNO08x: startup, telemetry, game rotation vector and recovery scenarios passed");
}
