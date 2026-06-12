// ─────────────────────────────────────────────────────────────────────────────
// AVL_CARL — MASTER / VCU node                  Board: NUCLEO-F767ZI (ID 0x01)
//
// Responsibilities (and nothing else):
//   • Be the single command source/arbiter for the bus.
//   • Take teleop input from the PC over USB serial (ST-LINK VCP) and mix it
//     into per-wheel differential-drive commands (0x100, 20 ms).
//   • Emit the periodic heartbeat the edge nodes watchdog on (0x010, 50 ms).
//   • Latch/relay e-stop (0x000) and hold the bus-wide safe state.
//   • Aggregate throttle status + encoder telemetry and stream it back to the PC.
//   • TELEOP WATCHDOG: if the PC stops sending for TELEOP_TIMEOUT_MS (crash /
//     unplug), force commands to zero so the kart stops. No-brake design.
//
// This node does NOT drive any motor directly — it only speaks CAN + serial.
//
// CAN: STM32_CAN (bxCAN) on CAN1 remapped to PD0 = CAN1_RX, PD1 = CAN1_TX.
//      Wire those to the TJA1051 transceiver. The master is the MIDDLE of the
//      3-node bus -> its transceiver 120 Ω termination must be DISABLED.
//      (If you relocate CAN1, change the pin enum below: DEF=PA11/PA12,
//       ALT=PB8/PB9, ALT_2=PD0/PD1.)
//
// Serial: the PC teleop talks to `Serial` (USART3 -> ST-LINK Virtual COM Port,
//         the same USB cable used to flash the board) at 115200 8N1.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <STM32_CAN.h>

#include "carl_protocol.h"
#include "heartbeat.h"
#include "messages.h"

using namespace carl;

// ── CAN ─────────────────────────────────────────────────────────────────────
STM32_CAN Can1(CAN1, ALT_2);  // ALT_2 = PD0 (RX) / PD1 (TX)

// ── Teleop command state (in per-mille, [-1000, 1000]) ──────────────────────
int16_t teleopVx = 0;  // forward(+) / reverse(-)
int16_t teleopWz = 0;  // turn right(+) / left(-)
bool estopLatched = false;

// Bring-up single-wheel test mode: when testIdx is 0..3, ONLY that wheel slot is
// commanded (others 0) so each motor can be identified/checked in isolation.
// -1 = normal differential driving. Slot order: 0=FL, 1=FR, 2=RL, 3=RR.
int testIdx = -1;
int16_t testVal = 0;

HeartbeatTx hb(HEARTBEAT_PERIOD_MS);
Watchdog teleopWatch(TELEOP_TIMEOUT_MS);

// Latest telemetry from the throttle node (for streaming back to the PC).
ThrottleStatus lastStatus = {TST_INIT, TF_NONE, 0, 0};
EncoderFb lastEnc = {0, 0};
bool haveStatus = false;

uint32_t lastCmdMs = 0;
uint32_t lastTlmMs = 0;

// Serial line assembly.
static const uint8_t LINE_MAX = 48;
char lineBuf[LINE_MAX];
uint8_t lineLen = 0;

// ── CAN send ────────────────────────────────────────────────────────────────
static void canSend(uint16_t id, const uint8_t* buf, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  for (uint8_t i = 0; i < len && i < 8; i++) msg.buf[i] = buf[i];
  Can1.write(msg);
}

static void sendEstopFrame() {
  uint8_t buf[8];
  uint8_t len = pack_estop(buf, ESTOP_OPERATOR);
  canSend(CANID_ESTOP, buf, len);
}

// Differential mix -> per-wheel command, then send 0x100.
static void sendThrottleCmd() {
  ThrottleCmd c = {0, 0, 0, 0};
  if (!estopLatched) {
    if (testIdx >= 0 && testIdx < 4) {
      // Single-wheel bring-up: drive only the selected slot.
      int16_t v[4] = {0, 0, 0, 0};
      v[testIdx] = clampv<int16_t>(testVal, THROTTLE_MIN, THROTTLE_MAX);
      c.fl = v[0];
      c.fr = v[1];
      c.rl = v[2];
      c.rr = v[3];
    } else {
      int16_t left = clampv<int16_t>(teleopVx + teleopWz, THROTTLE_MIN, THROTTLE_MAX);
      int16_t right = clampv<int16_t>(teleopVx - teleopWz, THROTTLE_MIN, THROTTLE_MAX);
      c.fl = left;
      c.rl = left;
      c.fr = right;
      c.rr = right;
    }
  }
  uint8_t buf[8];
  uint8_t len = pack_throttle_cmd(buf, c);
  canSend(CANID_THROTTLE_CMD, buf, len);
}

static void sendHeartbeat(uint32_t now, bool teleopOk) {
  Heartbeat h;
  h.counter = hb.next(now);
  h.flags = HB_FLAG_OK;
  if (teleopOk) h.flags |= HB_FLAG_TELEOP_OK;
  if (estopLatched) h.flags |= HB_FLAG_ESTOP;
  if (estopLatched)
    h.mode = MODE_ESTOP;
  else if (teleopVx != 0 || teleopWz != 0)
    h.mode = MODE_TELEOP;
  else
    h.mode = MODE_IDLE;
  uint8_t buf[8];
  uint8_t len = pack_heartbeat(buf, h);
  canSend(CANID_HEARTBEAT, buf, len);
}

// ── Teleop serial parsing ───────────────────────────────────────────────────
// Accepted lines (newline-terminated):
//   D <vx> <wz>   drive, ints per-mille [-1000,1000]
//   S             stop (vx = wz = 0)
//   E             e-stop (latch safe state bus-wide)
//   R             reset e-stop
static void parseLine(char* s) {
  // Skip leading spaces.
  while (*s == ' ') s++;
  char cmd = *s;

  switch (cmd) {
    case 'D':
    case 'd': {
      int vx = 0, wz = 0;
      if (sscanf(s + 1, "%d %d", &vx, &wz) == 2) {
        teleopVx = clampv<int16_t>((int16_t)vx, THROTTLE_MIN, THROTTLE_MAX);
        teleopWz = clampv<int16_t>((int16_t)wz, THROTTLE_MIN, THROTTLE_MAX);
        testIdx = -1;                // any normal drive cmd exits test mode
        teleopWatch.feed(millis());  // valid command => link alive
      }
      break;
    }
    case 'M':  // single-wheel test: "M <index 0-3> <value>"
    case 'm': {
      int i = 0, v = 0;
      if (sscanf(s + 1, "%d %d", &i, &v) == 2) {
        testIdx = i;
        testVal = (int16_t)v;
        teleopWatch.feed(millis());
      }
      break;
    }
    case 'S':
    case 's':
      teleopVx = 0;
      teleopWz = 0;
      testIdx = -1;
      teleopWatch.feed(millis());
      break;
    case 'E':
    case 'e':
      estopLatched = true;
      teleopVx = 0;
      teleopWz = 0;
      testIdx = -1;
      sendEstopFrame();  // fire the high-priority frame immediately
      teleopWatch.feed(millis());
      break;
    case 'R':
    case 'r':
      estopLatched = false;
      teleopWatch.feed(millis());
      break;
    default:
      break;
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        parseLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < LINE_MAX - 1) {
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;  // overflow -> drop the garbage line
    }
  }
}

// ── CAN receive ─────────────────────────────────────────────────────────────
static void pollCan() {
  CAN_message_t rx;
  while (Can1.read(rx)) {
    switch (rx.id) {
      case CANID_THROTTLE_STATUS:
        if (unpack_throttle_status(rx.buf, rx.len, &lastStatus)) haveStatus = true;
        break;
      case CANID_ENCODER_FB:
        unpack_encoder_fb(rx.buf, rx.len, &lastEnc);
        break;
      case CANID_ESTOP:
        estopLatched = true;  // someone else asserted e-stop
        break;
      default:
        break;
    }
  }
}

// Human-readable + machine-parseable telemetry line for the teleop UI.
static void streamTelemetry(bool teleopOk) {
  Serial.print(F("TLM hb="));
  Serial.print(hb.counter());
  Serial.print(F(" mode="));
  if (estopLatched)
    Serial.print("ESTOP");
  else if (testIdx >= 0)
    Serial.print("TEST");
  else
    Serial.print(teleopVx || teleopWz ? "TELEOP" : "IDLE");
  Serial.print(F(" test="));
  Serial.print(testIdx);
  Serial.print(F(" link="));
  Serial.print(teleopOk ? 1 : 0);
  Serial.print(F(" vx="));
  Serial.print(teleopVx);
  Serial.print(F(" wz="));
  Serial.print(teleopWz);
  Serial.print(F(" tstate="));
  Serial.print(haveStatus ? lastStatus.state : 255);
  Serial.print(F(" tflt="));
  Serial.print(haveStatus ? lastStatus.faults : 0);
  Serial.print(F(" L="));
  Serial.print(haveStatus ? lastStatus.applied_left : 0);
  Serial.print(F(" R="));
  Serial.print(haveStatus ? lastStatus.applied_right : 0);
  Serial.print(F(" encL="));
  Serial.print(lastEnc.left_count);
  Serial.print(F(" encR="));
  Serial.println(lastEnc.right_count);
}

// ── Arduino entry points ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);

  Serial.println(F("AVL_CARL master/VCU (NUCLEO-F767ZI) online"));
  Serial.println(F("Teleop: 'D vx wz' drive, 'S' stop, 'E' e-stop, 'R' reset"));
}

void loop() {
  const uint32_t now = millis();

  pollSerial();
  pollCan();

  // Teleop watchdog: stale link -> force zero (kart stops). E-stop, if latched,
  // stays latched regardless of link.
  const bool teleopOk = !teleopWatch.expired(now);
  if (!teleopOk) {
    teleopVx = 0;
    teleopWz = 0;
  }

  if (now - lastCmdMs >= THROTTLE_CMD_PERIOD_MS) {
    lastCmdMs = now;
    sendThrottleCmd();
  }

  if (hb.due(now)) {
    sendHeartbeat(now, teleopOk);
  }

  // Stream telemetry to the PC at ~10 Hz.
  if (now - lastTlmMs >= 100) {
    lastTlmMs = now;
    streamTelemetry(teleopOk);
  }
}
