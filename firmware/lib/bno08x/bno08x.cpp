#include "bno08x.h"

namespace {
uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t u32(const uint8_t* p) {
  return uint32_t(u16(p)) | (uint32_t(u16(p + 2)) << 16);
}
const uint8_t REPORTS[4] = {0x01, 0x02, 0x05, 0x08};
}

void Bno08x::begin(Bno08xTransport& transport, uint32_t now) {
  transport_ = &transport;
  error_count_ = 0;
  error_ = OK;
  failed_stage_ = POWER_WAIT;
  stage_ = POWER_WAIT;
  since_ = now;
  heading_seen_ = heading_dirty_ = false;
  // PS1 is wired HIGH. PS0/WAKE must stay HIGH throughout reset/startup.
  transport_->wake(false);
  transport_->reset(true);
}

void Bno08x::startReset(uint32_t now) {
  transport_->wake(false);
  transport_->reset(true);
  stage_ = RESET_LOW;
  since_ = now;
  identified_ = requested_ = false;
  product_ = 0;
  feature_ = control_sequence_ = seen_ = dirty_ = 0;
  heading_seen_ = heading_dirty_ = false;
}

void Bno08x::fail(Error error, uint8_t detail, uint32_t now) {
  failed_stage_ = stage_;
  error_ = error;
  detail_ = detail;
  if (error_count_ != UINT16_MAX) ++error_count_;
  stage_ = RETRY_WAIT;
  since_ = now;
  dirty_ = seen_ = 0;
  heading_seen_ = heading_dirty_ = false;
  transport_->wake(false);
  // Quiesce the failed sensor only, never the STM32 or CAN bus.
  transport_->reset(true);
}

void Bno08x::poll(uint32_t now) {
  if (!transport_) return;
  if (stage_ == POWER_WAIT || stage_ == RETRY_WAIT) {
    if (now - since_ >= (stage_ == POWER_WAIT ? 100U : 1000U)) startReset(now);
    return;
  }
  if (stage_ == RESET_LOW) {
    if (now - since_ >= 2) {
      transport_->reset(false);
      stage_ = BOOT_WAIT;
      since_ = now;
    }
    return;
  }
  if (stage_ == BOOT_WAIT && !transport_->interruptActive()) {
    if (now - since_ >= 1000) fail(BOOT_TIMEOUT, 0, now);
    return;
  }

  const bool waiting_for_reset = stage_ == BOOT_WAIT;
  // One bounded SPI receive transaction per foreground loop, even if INT is
  // stuck LOW. Reject corrupt lengths before clocking arbitrary amounts.
  const int length = transport_->read(packet_, sizeof(packet_));
  if (length < 0 || length > MAX_PACKET) { fail(INVALID_PACKET, 0, now); return; }
  if (length > 0) {
    parse(packet_, uint16_t(length), now);
    if (stage_ == RETRY_WAIT) return;
  }
  if (waiting_for_reset) {
    // INT may announce an advertisement before the executable is ready. Drain
    // boot traffic with PS0 HIGH; only Reset Complete permits host commands.
    if (stage_ == BOOT_WAIT && now - since_ >= 1000) fail(BOOT_TIMEOUT, 1, now);
    return;
  }

  if (stage_ == IDENTIFY) {
    if (identified_) {
      stage_ = CONFIGURE;
      since_ = now;
    } else if (now - since_ >= 1000) {
      fail(IDENTIFY_TIMEOUT, 0xf9, now);
    } else if (!requested_) {
      const uint8_t request[] = {0xf9, 0};
      if (transport_->write(2, control_sequence_, request, sizeof(request))) {
        ++control_sequence_;
        requested_ = true;
      }
    }
    return;
  }
  if (stage_ == CONFIGURE) {
    if (now - since_ >= 1000) { fail(CONFIGURE_TIMEOUT, REPORTS[feature_], now); return; }
    // Set Feature, 10,000 us interval (100 Hz), no batching. One command per
    // loop. Ready requires actual fresh reports, not merely successful writes.
    uint8_t command[17] = {0xfd, REPORTS[feature_], 0, 0, 0, 0x10, 0x27};
    if (transport_->write(2, control_sequence_, command, sizeof(command))) {
      ++control_sequence_;
      if (++feature_ == 4) {
        stage_ = STREAM_WAIT;
        since_ = now;
      }
    }
    return;
  }
  if (stage_ == STREAM_WAIT) {
    if (seen_ == 7) { stage_ = READY; error_ = OK; detail_ = 0; }
    else if (now - since_ >= 1000) { fail(REPORT_TIMEOUT, seen_, now); return; }
  }
  if (stage_ == READY) {
    for (unsigned i = 0; i < 3; ++i) {
      if (now - last_report_[i] >= 250) { fail(REPORT_TIMEOUT, REPORTS[i], now); return; }
    }
  }
}

void Bno08x::parse(const uint8_t* p, uint16_t size, uint32_t now) {
  if (size < 4 || (u16(p) & 0x7fff) != size || p[2] > 5) {
    fail(INVALID_PACKET, 0, now); return;
  }
  if (p[1] & 0x80) { fail(UNSUPPORTED_FRAGMENT, p[2], now); return; }
  const uint8_t channel = p[2];
  p += 4;
  size -= 4;
  if (channel == 1 && size && p[0] == 1) {
    if (size != 1) { fail(INVALID_PACKET, 1, now); return; }
    if (stage_ == BOOT_WAIT) {
      stage_ = IDENTIFY;
      since_ = now;
    } else {
      fail(SENSOR_RESET, 1, now);
    }
    return;
  }
  if (channel == 2 && size && p[0] == 0xf8) {
    if (size < 16) { fail(INVALID_PACKET, 0xf8, now); return; }
    if (stage_ == IDENTIFY && requested_) {
      product_ = u32(p + 4);
      identified_ = true;
    }
    return;
  }
  if ((channel != 3 && channel != 4)
      || (stage_ != CONFIGURE && stage_ != STREAM_WAIT && stage_ != READY)) return;
  while (size) {
    const uint8_t report = p[0];
    uint8_t n = 0;
    int index = -1;
    if (report == 0xfb || report == 0xfa) n = 5;  // timestamp / rebase
    else {
      for (int i = 0; i < 4; ++i) if (report == REPORTS[i]) index = i;
      if (index < 0) return;  // Unknown report: never guess its size.
      n = index == 2 ? 14 : index == 3 ? 12 : 10;
    }
    if (size < n) { fail(INVALID_PACKET, report, now); return; }
    if (index == 3) {
      // GRV has four Q14 values and NO heading accuracy field (12 bytes).
      // Its absence disables heading assist; it does not take down telemetry
      // or manual throttle. The original three reports still define READY.
      if (!heading_seen_ || heading_sequence_ != p[1]) {
        for (int i = 0; i < 4; ++i) heading_.quaternion[i] = int16_t(u16(p + 4 + 2 * i));
        heading_.accuracy = p[2] & 3;
        heading_.orientation_ms = now;
        heading_sequence_ = p[1];
        heading_seen_ = heading_dirty_ = true;
      }
    } else if (index >= 0 && (!(seen_ & (1 << index)) || report_sequence_[index] != p[1])) {
      int16_t* values = index == 0 ? sample_.accel : index == 1 ? sample_.gyro : sample_.quaternion;
      for (int i = 0; i < (index == 2 ? 4 : 3); ++i) values[i] = int16_t(u16(p + 4 + 2 * i));
      if (index == 2) sample_.heading_accuracy = u16(p + 12);
      sample_.accuracy[index] = p[2] & 3;
      report_sequence_[index] = p[1];
      last_report_[index] = now;
      seen_ |= 1 << index;
      dirty_ |= 1 << index;
    }
    p += n;
    size -= n;
  }
}

bool Bno08x::takeSample(Bno08xSample& sample, uint32_t now) {
  if (!ready() || dirty_ != 7) return false;
  // These are independently scheduled reports, NOT a simultaneous ADC sample.
  // Never relabel an old vector as fresh if just one report keeps arriving.
  for (unsigned i = 0; i < 3; ++i) if (now - last_report_[i] > 50) return false;
  sample = sample_;
  dirty_ = 0;
  return true;
}

bool Bno08x::takeHeadingSample(Bno08xHeadingSample& sample, uint32_t now) {
  if (!ready() || !heading_dirty_ || !(seen_ & 2)) return false;
  if (now - heading_.orientation_ms > 50 || now - last_report_[1] > 50) return false;
  for (int i = 0; i < 3; ++i) heading_.gyro[i] = sample_.gyro[i];
  heading_.gyro_accuracy = sample_.accuracy[1];
  heading_.gyro_ms = last_report_[1];
  sample = heading_;
  heading_dirty_ = false;
  return true;
}
