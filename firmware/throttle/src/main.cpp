// ─────────────────────────────────────────────────────────────────────────────
// AVL_CARL — THROTTLE / MOTOR-DRIVE node            Board: Teensy 4.1 (ID 0x02)
//
// Responsibilities (and nothing else):
//   • Receive per-wheel throttle commands from the master over CAN (0x100).
//   • Drive 4× IBT_2 (BTS7960) H-bridges -> 4× MY4845 motors (8 PWM + 4 enables).
//   • Balance rear wheel RPM using the 2 encoders; report raw counts (0x210).
//   • Report node status + applied command (0x200).
//   • FAIL-SAFE: on heartbeat loss or e-stop, command 0 PWM and drop all enables
//     (coast). No-brake design => "stop" is "command 0 and hold".
//
// CAN: FlexCAN_T4 on CAN1 -> Teensy pin 22 = CAN1_TX, pin 23 = CAN1_RX.
//      Wire those to the TJA1051 transceiver TX/RX. This node is a BUS END:
//      enable the transceiver's on-board 120 Ω termination here.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Encoder.h>
#include <FlexCAN_T4.h>

#include "carl_protocol.h"
#include "heartbeat.h"
#include "messages.h"
#include "rear_speed_sync.h"

using namespace carl;

// ── Pin map (see docs/wiring.md) ────────────────────────────────────────────
// One IBT_2 per motor. RPWM/LPWM are the two PWM inputs (forward/reverse);
// EN drives BOTH R_EN and L_EN of that module (tied together) — HIGH = enabled.
struct MotorPins {
  uint8_t rpwm;  // forward PWM
  uint8_t lpwm;  // reverse PWM
  uint8_t en;    // R_EN + L_EN tied together
};

// Index order matches the wheels: 0=FL, 1=FR, 2=RL, 3=RR.
static const MotorPins kMotor[4] = {
    {2, 3, 10},  // slot 0 = FL — front-left
    {4, 5, 11},  // slot 1 = FR — front-right
    {8, 9, 24},  // slot 2 = RL — rear-left  (pins swapped with RR during bring-up)
    {6, 7, 12},  // slot 3 = RR — rear-right (pins swapped with RL during bring-up)
};

// Per-motor direction from bench bring-up: +1 normal, -1 reversed. The right
// side (FR, RR) was wired reversed, so we flip it here -> a positive (forward)
// command spins every corner forward. Applied in driveAll().
static const int8_t kMotorDir[4] = {+1, -1, +1, -1};  // FL, FR, RL, RR

// Rear encoders (white=A, green=B). Hand rotation confirmed the physical LEFT
// wheel drives 16/17, not 14/15. The old crossed assignment created positive
// feedback: slowing RL/boosting RR increased the measured "left minus right".
// Use physical wheel names for BOTH regulation and raw CAN count telemetry.
static const uint8_t ENC_RL_A = 16, ENC_RL_B = 17;
static const uint8_t ENC_RR_A = 14, ENC_RR_B = 15;

static const uint8_t PIN_LED = LED_BUILTIN;  // pin 13 status LED

// ── PWM configuration ───────────────────────────────────────────────────────
static const uint32_t PWM_FREQ_HZ = 20000;   // 20 kHz: inaudible, easy on the BTS7960
static const uint8_t  PWM_RES_BITS = 12;     // 12-bit -> 0..4095
static const uint16_t PWM_MAX = 4095;

// ── Globals ─────────────────────────────────────────────────────────────────
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

Encoder encRL(ENC_RL_A, ENC_RL_B);
Encoder encRR(ENC_RR_A, ENC_RR_B);

Watchdog hbWatch(HEARTBEAT_TIMEOUT_MS);
Watchdog cmdWatch(THROTTLE_COMMAND_TIMEOUT_MS);

ThrottleCmd cmd = {0, 0, 0, 0};  // last commanded per-wheel throttle (per-mille)
ThrottleCmd appliedCmd = {0, 0, 0, 0};  // post-regulation, vehicle direction
RearSpeedSync rearSpeedSync;
bool estopLatched = false;
uint8_t lastHbCounter = 0;
bool haveHb = false;

uint32_t lastStatusMs = 0;
uint32_t lastEncoderMs = 0;
uint32_t lastBlinkMs = 0;

// ── Motor output ────────────────────────────────────────────────────────────
// Apply a signed per-mille command to one IBT_2. armed=false -> coast (enable
// low, both PWM 0). Sign selects direction. Direct commands use the input
// deadband; regulated commands were already gated before the speed correction.
static void applyMotor(const MotorPins& m, int16_t mille, bool armed,
                       bool regulated = false) {
  if (!armed) {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, 0);
    digitalWrite(m.en, LOW);
    return;
  }

  mille = clampv<int16_t>(mille, THROTTLE_MIN, THROTTLE_MAX);

  if (mille == 0 || (!regulated && abs_i16(mille) < THROTTLE_DEADBAND)) {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, 0);
    digitalWrite(m.en, HIGH);  // armed and holding zero
    return;
  }

  uint16_t duty = (uint16_t)((uint32_t)abs_i16(mille) * PWM_MAX / THROTTLE_MAX);
  digitalWrite(m.en, HIGH);
  if (mille > 0) {  // forward
    analogWrite(m.lpwm, 0);
    analogWrite(m.rpwm, duty);  // inactive direction always goes low first
  } else {  // reverse
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, duty);
  }
}

// Push all four wheel commands to hardware, applying per-motor direction signs.
static void driveAll(const ThrottleCmd& c, bool armed, bool regulatedRear = false) {
  const int16_t v[4] = {c.fl, c.fr, c.rl, c.rr};
  for (uint8_t i = 0; i < 4; i++) {
    applyMotor(kMotor[i], (int16_t)(v[i] * kMotorDir[i]), armed,
               regulatedRear && i >= 2);
  }
}

static void coastAll() {
  ThrottleCmd zero = {0, 0, 0, 0};
  driveAll(zero, /*armed=*/false);
}

// ── CAN helpers ─────────────────────────────────────────────────────────────
static void canSend(uint16_t id, const uint8_t* buf, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  for (uint8_t i = 0; i < len && i < 8; i++) msg.buf[i] = buf[i];
  can1.write(msg);
}

static void sendStatus(uint8_t state, uint8_t faults) {
  // PWM-equivalent side average, not measured motor motion/current.
  // Disabled outputs must report zero even if an old command is retained.
  const bool outputsEnabled = state == TST_IDLE || state == TST_DRIVING;
  ThrottleStatus s;
  s.state = state;
  s.faults = faults;
  s.applied_left = outputsEnabled ? (int16_t)((appliedCmd.fl + appliedCmd.rl) / 2) : 0;
  s.applied_right = outputsEnabled ? (int16_t)((appliedCmd.fr + appliedCmd.rr) / 2) : 0;
  uint8_t buf[8];
  uint8_t len = pack_throttle_status(buf, s);
  canSend(CANID_THROTTLE_STATUS, buf, len);
}

static void sendEncoders() {
  EncoderFb e;
  e.left_count = (int32_t)encRL.read();
  e.right_count = (int32_t)encRR.read();
  uint8_t buf[8];
  uint8_t len = pack_encoder_fb(buf, e);
  canSend(CANID_ENCODER_FB, buf, len);
}

// ── CAN receive ─────────────────────────────────────────────────────────────
static void handleFrame(const CAN_message_t& msg) {
  switch (msg.id) {
    case CANID_ESTOP:
      estopLatched = true;
      break;

    case CANID_HEARTBEAT: {
      Heartbeat hb;
      if (unpack_heartbeat(msg.buf, msg.len, &hb)) {
        hbWatch.feed(millis());
        lastHbCounter = hb.counter;
        haveHb = true;
        // The heartbeat's ESTOP flag is authoritative: it latches us into the
        // safe state even if we missed the one-shot 0x000 frame, and clears the
        // latch when the master drops it (operator pressed reset).
        estopLatched = (hb.flags & HB_FLAG_ESTOP) != 0;
      }
      break;
    }

    case CANID_THROTTLE_CMD: {
      ThrottleCmd c;
      if (unpack_throttle_cmd(msg.buf, msg.len, &c)) {
        // Clamp before abs_i16/sign operations (including malformed INT16_MIN).
        cmd = {clampv<int16_t>(c.fl, THROTTLE_MIN, THROTTLE_MAX),
               clampv<int16_t>(c.fr, THROTTLE_MIN, THROTTLE_MAX),
               clampv<int16_t>(c.rl, THROTTLE_MIN, THROTTLE_MAX),
               clampv<int16_t>(c.rr, THROTTLE_MIN, THROTTLE_MAX)};
        cmdWatch.feed(millis());
      }
      break;
    }

    default:
      break;  // not ours — ignore (steering/status/etc.)
  }
}

// ── Arduino entry points ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(kMotor[i].en, OUTPUT);
    digitalWrite(kMotor[i].en, LOW);  // start disabled (coast) — safe
    analogWriteFrequency(kMotor[i].rpwm, PWM_FREQ_HZ);
    analogWriteFrequency(kMotor[i].lpwm, PWM_FREQ_HZ);
  }
  analogWriteResolution(PWM_RES_BITS);
  coastAll();

  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();  // accept-all RX FIFO (no filters)

  Serial.println(F("AVL_CARL throttle node (Teensy 4.1) online"));
}

void loop() {
  // 1) Drain all pending CAN frames.
  CAN_message_t rx;
  while (can1.read(rx)) handleFrame(rx);
  // Frame handlers feed watchdogs using millis(). Read 'now' AFTER draining
  // so crossing a millisecond boundary cannot make a fresh feed look expired.
  const uint32_t now = millis();

  // 2) Decide armed state. Safe state wins over everything.
  const bool hbLost = hbWatch.expired(now);
  const bool cmdLost = cmdWatch.expired(now);
  const bool armed = haveHb && !hbLost && !cmdLost && !estopLatched;

  uint8_t state;
  uint8_t faults = TF_NONE;
  if (estopLatched) {
    state = TST_ESTOP;
    faults |= TF_ESTOP;
  } else if (!haveHb) {
    state = TST_INIT;
  } else if (hbLost) {
    state = TST_SAFE_STOP;
    faults |= TF_HEARTBEAT_LOST;
  } else if (cmdLost) {
    state = TST_SAFE_STOP;
    faults |= TF_COMMAND_LOST;
  } else {
    const bool moving = abs_i16(cmd.fl) >= THROTTLE_DEADBAND ||
                        abs_i16(cmd.fr) >= THROTTLE_DEADBAND ||
                        abs_i16(cmd.rl) >= THROTTLE_DEADBAND ||
                        abs_i16(cmd.rr) >= THROTTLE_DEADBAND;
    state = moving ? TST_DRIVING : TST_IDLE;
  }

  // 3) Balance rear RPM for equal commands. Stop, turn, reversal, and disarm
  // reset the controller immediately. Front wheels remain open-loop until
  // they have encoders; rear corrections are not a front speed measurement.
  const RearSpeedSync::Output rear = rearSpeedSync.update(
      now, (int32_t)encRL.read(), (int32_t)encRR.read(), cmd.rl, cmd.rr, armed);
  appliedCmd = armed ? ThrottleCmd{cmd.fl, cmd.fr, rear.left, rear.right}
                     : ThrottleCmd{0, 0, 0, 0};
  // Apply the input deadband before regulation, not to a regulator's small
  // nonzero outputs; otherwise light-throttle corrections have a PWM step.
  if (abs_i16(cmd.rl) < THROTTLE_DEADBAND) appliedCmd.rl = 0;
  if (abs_i16(cmd.rr) < THROTTLE_DEADBAND) appliedCmd.rr = 0;
  if (abs_i16(cmd.fl) < THROTTLE_DEADBAND) appliedCmd.fl = 0;
  if (abs_i16(cmd.fr) < THROTTLE_DEADBAND) appliedCmd.fr = 0;
  if (armed) {
    driveAll(appliedCmd, /*armed=*/true, /*regulatedRear=*/true);
  } else {
    coastAll();
  }

  // 4) Periodic telemetry back to the master.
  if (now - lastEncoderMs >= ENCODER_FB_PERIOD_MS) {
    lastEncoderMs = now;
    sendEncoders();
  }
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus(state, faults);
  }

  // 5) Status LED: solid when armed, fast blink in any safe/fault state.
  if (armed) {
    digitalWrite(PIN_LED, HIGH);
  } else if (now - lastBlinkMs >= 100) {
    lastBlinkMs = now;
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
}
