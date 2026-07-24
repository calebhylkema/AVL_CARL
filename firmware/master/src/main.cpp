// ─────────────────────────────────────────────────────────────────────────────
// AVL_CARL — MASTER / VCU node                  Board: NUCLEO-F767ZI (ID 0x01)
//
// The master is a DUMB PROXY between a companion computer (running ROS2) and
// the CAN bus, plus a safety supervisor.
//
//   Companion (framed binary over serial) <──> Master <──> CAN bus <──> edge nodes
//
// It does not do vehicle-level logic — no ASCII parsing, no differential mix,
// no wheel arbitration. That all lives in ROS2 now. What it DOES do:
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

#include "carl_protocol.h"
#include "heartbeat.h"
#include "messages.h"
#include "wire.h"

using namespace carl;

// ── CAN ─────────────────────────────────────────────────────────────────────
STM32_CAN Can1(CAN1, ALT_2);  // ALT_2 = PD0 (RX) / PD1 (TX)

// ── State (kept small on purpose) ───────────────────────────────────────────
ThrottleCmd currentCmd = {0, 0, 0, 0};  // last cmd from companion
bool        estopLatched = false;

HeartbeatTx hb(HEARTBEAT_PERIOD_MS);
Watchdog    teleopWatch(TELEOP_TIMEOUT_MS);
WireParser  parser;

uint32_t lastCmdMs = 0;

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

// ── Forward the current throttle command onto CAN (0x100) ───────────────────
static void sendThrottleCmd() {
  ThrottleCmd c = estopLatched ? ThrottleCmd{0, 0, 0, 0} : currentCmd;
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
      if (len >= 1 && payload[0] == 0) {
        // reason 0 = CLEAR the latch. Next heartbeat drops HB_FLAG_ESTOP so
        // edge nodes clear their own latch via the heartbeat handler. Do NOT
        // forward the clear onto CAN — the heartbeat carries the semantics.
        estopLatched = false;
      } else {
        estopLatched = true;
        currentCmd = {0, 0, 0, 0};
        // Fire the high-priority CAN frame immediately (don't wait for next tick).
        canSend(CANID_ESTOP, payload, len);
      }
      teleopWatch.feed(millis());
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
        wireSend(rx.id, rx.buf, rx.len);
        break;
      case CANID_ESTOP:
        // Someone else asserted e-stop on the bus.
        estopLatched = true;
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
}

void loop() {
  const uint32_t now = millis();

  pollSerial();
  pollCan();

  // Teleop watchdog: stale companion → force zero commands (kart stops).
  const bool teleopOk = !teleopWatch.expired(now);
  if (!teleopOk) {
    currentCmd = {0, 0, 0, 0};
  }

  if (now - lastCmdMs >= THROTTLE_CMD_PERIOD_MS) {
    lastCmdMs = now;
    sendThrottleCmd();
  }

  if (hb.due(now)) {
    sendHeartbeat(now, teleopOk);
  }
}
