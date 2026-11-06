// ─────────────────────────────────────────────────────────────────────────────
// AVL_CARL — MASTER / VCU node                  Board: NUCLEO-F767ZI (ID 0x01)
//
// The master proxies a companion computer (running ROS2) to the CAN bus,
// supervises safety, and acquires the local SPI IMU.
//
//   Companion (framed binary over serial) <──> Master <──> CAN bus <──> edge nodes
//
// ROS2 retains operator mixing/arbitration. Equal four-wheel commands engage
// bounded relative heading hold after stationary IMU calibration.
//
//   • Ingress: unwrap wire frames from the companion → drop matching CAN frame.
//   • Egress:  wrap incoming CAN frames from edge nodes → send to the companion.
//   • Emit the periodic heartbeat (0x010, 50 ms) — bus fail-safe for edge nodes.
//   • Latch e-stop (0x000) and hold it in the heartbeat flags. Software can
//     clear the latch by sending an ESTOP frame with reason=0 (REASON_RESET).
//     Any nonzero reason SETS the latch; reason=0 CLEARS it.
//   • TELEOP WATCHDOG: if the companion stops sending throttle commands for
//     TELEOP_TIMEOUT_MS, force the outbound throttle command to zero. No-brake
//     design => a stalled ROS2 stack must not leave the kart driving.
//   • Read the installed MPU-6500/9250 over SPI and send wire-only raw motion
//     and temperature to ROS2, and estimate relative yaw for straight runs.
//
// CAN: STM32_CAN (bxCAN) on CAN1 remapped to PD0 = CAN1_RX, PD1 = CAN1_TX.
//      Wire those to the TJA1051 transceiver. The master is the MIDDLE of the
//      3-node bus -> its transceiver 120 Ω termination must be DISABLED.
//
// Serial: 921600 8N1 on USART3 (ST-LINK Virtual COM Port, same USB cable used
//         to flash). Framed binary — see lib/carl_protocol/wire.h.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <STM32_CAN.h>
#include <SPI.h>

#include "carl_protocol.h"
#include "heartbeat.h"
#include "imu_telemetry.h"
#include "messages.h"
#include "bno08x.h"
#include "bno08x_spi.h"
#include "wire.h"
#include "heading_hold.h"
#ifdef CARL_IMU_MPU_SPI
#include "mpu_spi_test.h"
#include "mpu_heading.h"
#endif

using namespace carl;

// ── CAN ─────────────────────────────────────────────────────────────────────
STM32_CAN Can1(CAN1, ALT_2);  // ALT_2 = PD0 (RX) / PD1 (TX)

// ── IMU hardware ────────────────────────────────────────────────────────────
// SPI4 AF5 on the CN9 even row; MPU signals use CN9-14..20 (INT is polled).
// Constructor order: MOSI, MISO, SCK. GPIO CS is intentionally software-driven.
SPIClass ImuSpi(PE6, PE5, PE2);  // D59/CN9-20, D58/CN9-18, D56/CN9-14
#ifdef CARL_IMU_MPU_SPI
MpuSpiTest mpu(ImuSpi, PE4);
MpuHeading mpuHeading;
#else
Bno08xSpi imuTransport(ImuSpi, PE4, PF8, PE3, PF7);
// CS=D57/CN9-16, INT=D61/CN9-24, RST=D60/CN9-22, WAKE=D62/CN9-26.
// Module PS1 connects to 3.3 V; PS0 connects to WAKE, not permanently to VCC.
Bno08x imu;
#endif
HeadingHold headingHold;

// ── State (kept small on purpose) ───────────────────────────────────────────
ThrottleCmd currentCmd = {0, 0, 0, 0};  // last cmd from companion
bool        estopLatched = false;

HeartbeatTx hb(HEARTBEAT_PERIOD_MS);
Watchdog    teleopWatch(TELEOP_TIMEOUT_MS);
WireParser  parser;

uint32_t lastCmdMs = 0;

// ── IMU state ───────────────────────────────────────────────────────────────
// Retain IMU health and the shared wire sequence. The preserved BNO branch also
// stores its four-part orientation snapshot and separate heading-control report.
static uint8_t imuSequence = 0;
static uint32_t lastImuDiagnosticMs = 0;
static uint16_t reportedImuErrors = 0;
#ifdef CARL_IMU_MPU_SPI
static MpuSpiTest::Stage reportedMpuStage = MpuSpiTest::POWER_WAIT;
static bool haveEncoderEvidence = false, haveOutputEvidence = false;
static EncoderFb previousEncoders = {};
static uint32_t lastEncoderEvidenceMs = 0, lastEncoderMotionMs = 0;
static uint32_t lastOutputEvidenceMs = 0;
static bool outputsIdle = false;
#else
static bool imuSampleValid = false;
static bool imuTelemetryPending = false;
static bool reportedImuReady = false;
static Bno08x::Stage reportedImuStage = Bno08x::POWER_WAIT;
static Bno08xSample latestImuSample = {};
#endif
static uint32_t lastHeadingStatusMs = 0;

// ── CAN send ────────────────────────────────────────────────────────────────
static void canSend(uint16_t id, const uint8_t* buf, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  for (uint8_t i = 0; i < len && i < 8; i++) msg.buf[i] = buf[i];
  Can1.write(msg);
}

// ── Serial send (framed) ────────────────────────────────────────────────────
static void wireSend(uint16_t msg_id, const uint8_t* payload, uint8_t len) {
  uint8_t out[WIRE_MAX_FRAME];
  uint8_t n = wire_frame(out, msg_id, payload, len);
  Serial.write(out, n);
}

// ── Local IMU acquisition and wire telemetry ───────────────────────────────
// Service one bounded SHTP packet per loop. Reports are requested at 100 Hz.
#ifndef CARL_IMU_MPU_SPI
static void pollImu(uint32_t now) {
  imu.poll(now);
  imuSampleValid = imu.takeSample(latestImuSample, now);
  imuTelemetryPending = imuSampleValid;
  if (!imu.ready()) headingHold.invalidate();
  Bno08xHeadingSample heading;
  if (imu.takeHeadingSample(heading, now)) headingHold.accept(heading, now);
}
#endif

// Wire-only observability; older companion software can ignore this ID.
static void sendHeadingStatus(uint32_t now) {
  if (now - lastHeadingStatusMs < 100) return;
  lastHeadingStatusMs = now;
  uint8_t payload[8] = {(uint8_t)headingHold.state(), (uint8_t)headingHold.ready(now)};
  put_i16(payload + 2, (int16_t)roundf(headingHold.yaw() * 1000.0f));
  put_i16(payload + 4, (int16_t)roundf(headingHold.error() * 1000.0f));
  put_i16(payload + 6, headingHold.correction());
  wireSend(WIREID_HEADING_STATUS, payload, sizeof(payload));
}

#ifdef CARL_IMU_MPU_SPI
static bool operatorNeutral() {
  return currentCmd.fl > -THROTTLE_DEADBAND && currentCmd.fl < THROTTLE_DEADBAND &&
         currentCmd.fr > -THROTTLE_DEADBAND && currentCmd.fr < THROTTLE_DEADBAND &&
         currentCmd.rl > -THROTTLE_DEADBAND && currentCmd.rl < THROTTLE_DEADBAND &&
         currentCmd.rr > -THROTTLE_DEADBAND && currentCmd.rr < THROTTLE_DEADBAND;
}

static bool stationaryForCalibration(uint32_t now) {
  return operatorNeutral() && haveEncoderEvidence && haveOutputEvidence && outputsIdle &&
         now - lastEncoderEvidenceMs <= 100 && now - lastOutputEvidenceMs <= 150 &&
         now - lastEncoderMotionMs >= 500;
}

static void pollMpu(uint32_t now) {
  mpu.poll(now);
  if (mpu.stage() != MpuSpiTest::READY) {
    mpuHeading.invalidate();
    headingHold.invalidate();
  }
  MpuSpiTest::Sample s;
  if (mpu.takeSample(s)) {
    if (mpuHeading.accept(s.accel, s.gyro, s.temperature, now, stationaryForCalibration(now))) {
      headingHold.acceptYaw(mpuHeading.yaw(), mpuHeading.rate(), now, now);
    } else {
      headingHold.invalidate();
    }
    uint8_t p[8];
    const uint8_t status = IMU_STATUS_INITIALIZED | IMU_STATUS_SAMPLE_VALID;
    pack_imu_axis(p, {imuSequence, status, s.accel[0], s.accel[1], s.accel[2]});
    wireSend(WIREID_IMU_ACCEL, p, 8);
    pack_imu_axis(p, {imuSequence, status, s.gyro[0], s.gyro[1], s.gyro[2]});
    wireSend(WIREID_IMU_GYRO, p, 8);
    pack_imu_temperature(p, {imuSequence, status, s.temperature});
    wireSend(WIREID_IMU_TEMPERATURE, p, 4);
    ++imuSequence;
  }
  if (now - lastImuDiagnosticMs >= 1000 || reportedMpuStage != mpu.stage()
      || reportedImuErrors != mpu.errorCount()) {
    uint8_t p[8]; mpu.diagnostics(p);
    wireSend(WIREID_MPU_SPI_DIAGNOSTICS, p, 8);
    lastImuDiagnosticMs = now;
    reportedMpuStage = mpu.stage(); reportedImuErrors = mpu.errorCount();
  }
}
#else
static void sendImuDiagnostics(uint32_t now) {
  if (now - lastImuDiagnosticMs < 1000
      && reportedImuErrors == imu.errorCount()
      && reportedImuReady == imu.ready()
      && reportedImuStage == imu.stage()) return;
  const uint16_t count = imu.errorCount();
  const uint8_t payload[8] = {
      1, (uint8_t)imu.stage(), (uint8_t)imu.failedStage(), (uint8_t)imu.error(),
      imu.detail(), (uint8_t)imu.ready(), (uint8_t)count, (uint8_t)(count >> 8),
  };
  wireSend(WIREID_BNO_DIAGNOSTICS, payload, sizeof(payload));
  uint8_t spiPayload[8];
  imuTransport.packReadDiagnostics(spiPayload);
  wireSend(WIREID_BNO_SPI_DIAGNOSTICS, spiPayload, sizeof(spiPayload));
  lastImuDiagnosticMs = now;
  reportedImuErrors = imu.errorCount();
  reportedImuReady = imu.ready();
  reportedImuStage = imu.stage();
}

// Send acceleration, gyroscope, and quaternion back-to-back over the
// existing framed binary link. These messages are local master telemetry and
// are deliberately not sent over CAN.
static void sendImuTelemetry() {
  if (!imuTelemetryPending || !imuSampleValid) return;

  const uint8_t status = IMU_STATUS_INITIALIZED | IMU_STATUS_SAMPLE_VALID;
  const ImuAxisTelemetry accel = {
      imuSequence,
      status,
      latestImuSample.accel[0], latestImuSample.accel[1], latestImuSample.accel[2],
  };
  const ImuAxisTelemetry gyro = {
      imuSequence,
      status,
      latestImuSample.gyro[0], latestImuSample.gyro[1], latestImuSample.gyro[2],
  };
  const ImuAxisTelemetry quaternion = {
      imuSequence, (uint8_t)(status | (latestImuSample.accuracy[2] << 4)),
      latestImuSample.quaternion[0], latestImuSample.quaternion[1],
      latestImuSample.quaternion[2],
  };

  uint8_t payload[8];
  uint8_t len = pack_imu_axis(payload, accel);
  wireSend(WIREID_BNO_ACCEL, payload, len);

  len = pack_imu_axis(payload, gyro);
  wireSend(WIREID_BNO_GYRO, payload, len);

  len = pack_imu_axis(payload, quaternion);
  wireSend(WIREID_BNO_QUAT_XYZ, payload, len);
  payload[0] = imuSequence;
  payload[1] = quaternion.status;
  put_i16(payload + 2, latestImuSample.quaternion[3]);
  payload[4] = (uint8_t)latestImuSample.heading_accuracy;
  payload[5] = (uint8_t)(latestImuSample.heading_accuracy >> 8);
  wireSend(WIREID_BNO_QUAT_W, payload, 6);

  // Increment only after all four frames have been handed to the serial driver.
  // Unsigned rollover from 255 to 0 is intentional and understood by the bridge.
  imuTelemetryPending = false;
  ++imuSequence;
}
#endif

// ── Forward the current throttle command onto CAN (0x100) ───────────────────
static void sendThrottleCmd(uint32_t now, bool teleopOk) {
  // Preserve currentCmd as operator intent. Feeding corrected commands back
  // into the next tick would mistake our own yaw correction for a manual turn.
  const bool armed = teleopOk && !estopLatched;
#ifdef CARL_IMU_MPU_SPI
  const bool straightRun = armed && !operatorNeutral() && currentCmd.fl == currentCmd.fr &&
                           currentCmd.fl == currentCmd.rl && currentCmd.fl == currentCmd.rr;
  if (!mpuHeading.allowRun(now, straightRun) || !mpuHeading.ready(now)) headingHold.invalidate();
#endif
  const ThrottleCmd c = headingHold.update(now, currentCmd, armed);
  uint8_t buf[8];
  uint8_t len = pack_throttle_cmd(buf, c);
  canSend(CANID_THROTTLE_CMD, buf, len);
}

// ── Heartbeat: send to CAN AND to companion ─────────────────────────────────
static void sendHeartbeat(uint32_t now, bool teleopOk) {
  Heartbeat h;
  h.counter = hb.next(now);
  h.flags = HB_FLAG_OK;
  if (teleopOk)     h.flags |= HB_FLAG_TELEOP_OK;
  if (estopLatched) h.flags |= HB_FLAG_ESTOP;

  const bool moving = currentCmd.fl || currentCmd.fr || currentCmd.rl || currentCmd.rr;
  if (estopLatched)   h.mode = MODE_ESTOP;
  else if (moving)    h.mode = MODE_TELEOP;
  else                h.mode = MODE_IDLE;

  uint8_t buf[8];
  uint8_t len = pack_heartbeat(buf, h);
  canSend(CANID_HEARTBEAT, buf, len);
  wireSend(CANID_HEARTBEAT, buf, len);
}

// ── Ingress from companion ──────────────────────────────────────────────────
static void handleWireFrame(uint16_t msg_id, const uint8_t* payload, uint8_t len) {
  switch (msg_id) {
    case CANID_THROTTLE_CMD: {
      ThrottleCmd c;
      if (unpack_throttle_cmd(payload, len, &c)) {
        currentCmd = c;
        teleopWatch.feed(millis());
      }
      break;
    }
    case CANID_ESTOP:
      headingHold.invalidate();
      if (len >= 1 && payload[0] == 0) {
        // reason 0 = CLEAR the latch. Next heartbeat drops HB_FLAG_ESTOP so
        // edge nodes clear their own latch via the heartbeat handler. Do NOT
        // forward the clear onto CAN — the heartbeat carries the semantics.
        estopLatched = false;
        currentCmd = {0, 0, 0, 0}; // Reset never resurrects a retained drive command.
      } else {
        estopLatched = true;
        currentCmd = {0, 0, 0, 0};
        // Fire the high-priority CAN frame immediately (don't wait for next tick).
        canSend(CANID_ESTOP, payload, len);
      }
      break;
    default:
      break;  // ignore unknown IDs; forward-compat with future ROS2 msgs
  }
}

static void pollSerial() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    WireParser::Result r = parser.feed(b);
    if (r.valid) handleWireFrame(r.msg_id, r.payload, r.len);
  }
}

// ── CAN receive → forward to companion ──────────────────────────────────────
static void pollCan() {
  CAN_message_t rx;
  while (Can1.read(rx)) {
    switch (rx.id) {
      case CANID_THROTTLE_STATUS:
      case CANID_ENCODER_FB:
      case CANID_STEERING_STATUS:
      case CANID_STEERING_FB:
#ifdef CARL_IMU_MPU_SPI
        if (rx.id == CANID_ENCODER_FB) {
          EncoderFb e;
          if (unpack_encoder_fb(rx.buf, rx.len, &e)) {
            const uint32_t receivedMs = millis();
            if (!haveEncoderEvidence || receivedMs - lastEncoderEvidenceMs > 100 ||
                e.left_count != previousEncoders.left_count ||
                e.right_count != previousEncoders.right_count) lastEncoderMotionMs = receivedMs;
            previousEncoders = e;
            lastEncoderEvidenceMs = receivedMs;
            haveEncoderEvidence = true;
          }
        } else if (rx.id == CANID_THROTTLE_STATUS) {
          ThrottleStatus s;
          if (unpack_throttle_status(rx.buf, rx.len, &s)) {
            outputsIdle = s.state == TST_IDLE && s.faults == TF_NONE &&
                          s.applied_left == 0 && s.applied_right == 0;
            lastOutputEvidenceMs = millis();
            haveOutputEvidence = true;
          }
        }
#endif
        wireSend(rx.id, rx.buf, rx.len);
        break;
      case CANID_ESTOP:
        // Someone else asserted e-stop on the bus.
        headingHold.invalidate();
        estopLatched = true;
        currentCmd = {0, 0, 0, 0};
        wireSend(rx.id, rx.buf, rx.len);
        break;
      default:
        break;
    }
  }
}

// Enable STM32 bxCAN's automatic bus-off recovery (ABOM). Without this bit,
// once the TEC hits 256 (from motor-noise-induced errors, transceiver dropouts,
// etc.) the peripheral goes bus-off and stays there until manual reset. With
// ABOM set, hardware auto-recovers after 128 * 11 recessive bit times
// (~3 ms @ 500 kbit/s). pazi88's STM32_CAN library does not set this by
// default. Must be written while INRQ=1 (initialization mode).
static void enableCanAutoBusOffRecovery() {
  CAN1->MCR |= CAN_MCR_INRQ;                       // request init mode
  while (!(CAN1->MSR & CAN_MSR_INAK)) { }          // wait for ack
  CAN1->MCR |= CAN_MCR_ABOM;                       // enable auto bus-off management
  CAN1->MCR &= ~CAN_MCR_INRQ;                      // leave init mode
  while (CAN1->MSR & CAN_MSR_INAK) { }             // wait for normal mode
}

// ── Arduino entry points ────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  enableCanAutoBusOffRecovery();

  // IMU initialization failure is isolated: CARL's CAN, serial, heartbeat,
  // e-stop, watchdog, and throttle behavior continue exactly as before.
#ifdef CARL_IMU_MPU_SPI
  mpu.begin(millis());
#else
  imuTransport.begin();
  imu.begin(imuTransport, millis());
#endif
  lastImuDiagnosticMs = millis() - 1000;
}

void loop() {
  pollSerial();
  pollCan();
  const uint32_t now = millis(); // Must be at least as new as any watchdog feed.

  // Teleop watchdog: stale companion → force zero commands (kart stops).
  const bool teleopOk = !teleopWatch.expired(now);
  if (!teleopOk) {
    currentCmd = {0, 0, 0, 0};
  }

  if (now - lastCmdMs >= THROTTLE_CMD_PERIOD_MS) {
    lastCmdMs = now;
    sendThrottleCmd(now, teleopOk);
  }

  if (hb.due(now)) {
    sendHeartbeat(now, teleopOk);
  }

  // Keep all original command and safety work ahead of sensor acquisition and
  // telemetry. Initialization waits use a state machine rather than delay();
  // SPI packets have fixed size limits; missing INT never causes a busy wait.
#ifdef CARL_IMU_MPU_SPI
  pollMpu(now);
#else
  pollImu(now);
  sendImuTelemetry();
  sendImuDiagnostics(now);
#endif
  sendHeadingStatus(now);
}
