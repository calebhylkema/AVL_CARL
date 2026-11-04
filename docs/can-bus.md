# CAN Bus Design — AVL_CARL

> Living document. Last updated: 2026-06-10.
> The message/ID map below is a **starting proposal** — finalize before firmware bring-up.

---

## 1. Physical layer

| Parameter | Value | Notes |
|-----------|-------|-------|
| Standard | Classic CAN 2.0 | Teensy FlexCAN + STM32 bxCAN are classic-only (no CAN-FD). |
| Bitrate | **500 kbit/s** (proposed) | Revisit for final bus length; lower if the bus is long. |
| Frame format | 11-bit standard IDs | Lower ID = higher priority. |
| Termination | **120 Ω at both physical ends** | Two resistors total, not per node. Enable the Adafruit board's on-board 120 Ω on the **two end nodes only**. |
| Transceiver | **Adafruit TJA1051T/3** ×3 (in hand) | 3.3 V logic side; on-board optional 5 V boost + switchable 120 Ω. |
| Wiring | Twisted pair (CAN_H / CAN_L) + common GND | Keep stubs short. |

---

## 2. Node IDs

| Node | Board | Node ID (proposed) |
|------|-------|--------------------|
| Master / VCU | NUCLEO-F767ZI | 0x01 |
| Throttle / Motor-drive | Teensy 4.1 | 0x02 |
| Steering | Teensy 4.1 | 0x03 |
| (Future 4th node — needs a new board) | TBD | 0x04 |

---

## 3. Message / ID map (LOCKED — matches firmware)

Priority is encoded in the ID (lower = higher priority). Reserve the lowest IDs for safety.
These layouts are the single source of truth and are implemented in
`firmware/lib/carl_protocol/` (`carl_protocol.h`, `messages.h`). **All multi-byte fields are
little-endian.** Change the header and this table together.

| CAN ID | Name | Direction | Period | Len | Payload (byte : field) |
|--------|------|-----------|--------|-----|------------------------|
| `0x000` | **E-STOP** | any → all | event | 1 | `0`:reason (1=operator) |
| `0x010` | **Heartbeat** | Master → all | 50 ms | 3 | `0`:counter · `1`:flags · `2`:mode |
| `0x100` | Throttle command | Master → Throttle | 20 ms | 8 | `0-1`:FL · `2-3`:FR · `4-5`:RL · `6-7`:RR (each `int16` per-mille, −1000…+1000) |
| `0x110` | Steering command | Master → Steering | 20 ms | — | *Phase 1: unused (differential drive)* |
| `0x200` | Throttle status | Throttle → Master | 50 ms | 6 | `0`:state · `1`:faults · `2-3`:applied-left · `4-5`:applied-right (`int16` per-mille) |
| `0x210` | Wheel encoder feedback | Throttle → Master | 20 ms | 8 | `0-3`:rear-L count · `4-7`:rear-R count (`int32`, signed quadrature counts) |
| `0x220` | Steering status | Steering → Master | 50 ms | 2 | `0`:state(0=idle) · `1`:homed *(Phase 1: idle stub)* |

Absolute steering feedback **0x221** (steering → master, 20 ms, 6 bytes):
byte 0 = schema 1; byte 1 = flags (1=sample valid, 2=ADC low rail,
4=ADC high rail); bytes 2–3 = unsigned 12-bit ADC count in u16 LE;
bytes 4–5 = u16 LE sample counter. The master forwards this frame unchanged
to the companion. Existing 0x220 idle status stays unchanged. See
[encoder wiring, ROS topic and calibration](steering-encoder.md).

### 3.1 Master-to-companion wire-only telemetry

SPI receive diagnostics `0x23a` accompany BNO health frames without changing
`0x238`. Eight bytes: schema=1, result (0=not attempted, 1=empty, 2=packet,
3=invalid length), four raw SHTP header bytes, GPIO bits, reserved=0.
GPIO bits captured before the last receive: bit 0=INT asserted low,
bit 1=RST high, bit 2=WAKE high, bit 3=CS high. The snapshot persists across
sensor retries and is not a current pin reading. The bridge publishes
`/carl/imu/spi_diagnostics` as JSON (`std_msgs/msg/String`).

The master uses the same numeric-ID convention for locally measured telemetry
sent over its framed serial/UDP link. The following messages are **not CAN
frames** and do not add traffic to the vehicle CAN bus. Their payloads remain
limited to eight bytes so the framing layer keeps one consistent maximum size.

| Wire ID | Name | Direction | Period | Len | Payload (byte : field) |
|---------|------|-----------|--------|-----|------------------------|
| `0x230` | IMU acceleration | Master → Companion | data-ready, 100 Hz | 8 | `0`:sequence · `1`:status · `2-3`:X · `4-5`:Y · `6-7`:Z (`int16`, raw ±2 g scale) |
| `0x231` | IMU gyroscope | Master → Companion | data-ready, 100 Hz | 8 | `0`:sequence · `1`:status · `2-3`:X · `4-5`:Y · `6-7`:Z (`int16`, raw ±250°/s scale) |
| `0x232` | IMU die temperature | Master → Companion | data-ready, 100 Hz | 4 | `0`:sequence · `1`:status · `2-3`:temperature (`int16`, raw TEMP_OUT) |
| `0x233` | IMU diagnostics | Master → Companion | 1 Hz and stage/error changes | 8 | `0`:stage · `1`:last failed stage · `2`:I2C address · `3`:WHO_AM_I · `4`:error · `5`:register · `6-7`:error count (`uint16`, little-endian, saturating) |
| `0x234` | BNO acceleration | Master → Companion | fresh snapshot, requested 100 Hz | 8 | sequence, status, X/Y/Z `int16`, Q8 m/s² including gravity |
| `0x235` | BNO gyro | Master → Companion | same snapshot | 8 | sequence, status, X/Y/Z `int16`, Q9 rad/s |
| `0x236` | BNO quaternion XYZ | Master → Companion | same snapshot | 8 | sequence, status, X/Y/Z `int16`, Q14 |
| `0x237` | BNO quaternion W | Master → Companion | same snapshot | 6 | sequence, status, W `int16` Q14, heading accuracy `uint16` Q12 rad |
| `0x238` | BNO SPI diagnostics | Master → Companion | 1 Hz and state/error changes | 8 | schema=1, stage, failed stage, error, detail, ready, cumulative count `uint16` |
| `0x239` | Heading hold status | Master → Companion | 10 Hz | 8 | state `u8`, ready `u8`, yaw `i16` mrad, reference-minus-yaw `i16` mrad, signed differential trim `i16` per-mille |

**Current firmware uses BNO `0x234`–`0x238`; `0x230`–`0x233` below describe
the retained legacy MPU protocol.** All integers are little-endian. The new
IDs intentionally prevent older bridges interpreting BNO SI units as MPU ADC
counts. Every BNO snapshot uses one sequence across all four parts. Status
bits 0 and 1 mean initialized/valid; quaternion status bits 4–5 preserve SH-2
rotation accuracy (0 unreliable, 1 low, 2 medium, 3 high). The bridge requires
all four valid parts within 100 ms and publishes no synthetic temperature.

The master additionally requests game rotation (`0x08`) for local heading hold.
This does not replace the magnetic rotation (`0x05`) in `0x236/0x237`. Control
needs fresh game orientation and gyro reports with calibration status >=2;
the existing three reports still determine BNO driver READY. Wire-only `0x239`
exposes heading states 0 disabled, 1 idle, 2 holding, 3 manual turn, 4 waiting
for IMU, 5 neutral required. `ready` indicates fresh, qualified control data
after its 500 ms warmup. Yaw is an arbitrary game-vector heading, not north.
Positive trim subtracts from signed left commands and adds to signed right
commands; error is wrapped to +/-pi. Older bridges may ignore this extra frame.

BNO stages: 0 power wait, 1 reset low, 2 boot wait, 3 identify, 4 configure,
5 stream wait, 6 ready, 7 retry wait. Errors: 0 OK, 1 boot timeout,
2 identify timeout, 3 configure timeout, 4 invalid packet,
5 unsupported fragment, 6 report timeout, 7 sensor reset.
Diagnostic detail identifies the failed command/report where applicable.
Report-timeout detail is the received-report bitmask during stream setup,
or report ID during steady-state monitoring. No I2C address/WHO_AM_I fields
exist for the new SPI schema. Error counts saturate at 65535 and survive
sensor recovery, but not STM32 restart. Unknown schemas are rejected.

All three frames from one IMU sample use the same rolling `uint8` sequence
number. Status bit 0 means the sensor initialized successfully; bit 1 means the sample
is valid. The companion must receive both valid halves with matching sequence
numbers before publishing one combined IMU sample.

Diagnostics are sent even when initialization fails and motion/temperature
frames cannot be produced. Stages: 0 power wait, 1 identify, 2 reset, 3 reset
wait, 4 wake, 5 wake wait, 6 configure, 7 ready, 8 retry wait, 9 sample read,
10 I2C bus recovery.
Errors: 0 OK, 1 local buffer error, 2 address NACK, 3 data NACK, 4 bus error,
5 timeout (reserved for cores reporting it separately), 6 incomplete read,
7 unexpected identity, 8 SCL held low during recovery, 9 SDA held low after
recovery clocks. The installed STM32 Wire core reports transmit timeouts
as error 4 and may collapse receive-side bus errors into an incomplete read;
this code does not
claim to distinguish the underlying electrical cause. The error count covers
both initialization and sampling failures and resets when the STM32 restarts.
`carl_bridge` publishes readable JSON in `std_msgs/msg/String` on
`/carl/imu/diagnostics`. Diagnostics never become motion or control messages.

**Flag / enum encodings** (see `carl_protocol.h`):
- Heartbeat `flags`: bit0 `OK`, bit1 `ESTOP`, bit2 `TELEOP_OK`. `mode`: 0 idle, 1 teleop, 2 estop.
- Throttle `state`: 0 init, 1 idle, 2 driving, 3 safe-stop, 4 estop. `faults`: bit0 heartbeat-lost, bit1 estop, bit2 command-lost.
- Throttle "per-mille": −1000 = full reverse, 0 = stop, +1000 = full forward.

> Max 8 data bytes per frame (classic CAN). The companion mixes operator throttle/turn
> into four per-wheel setpoints. The master adds bounded IMU heading correction
> for equal, same-direction four-wheel requests and sends `0x100`; the throttle node maps
> each setpoint to one IBT_2 (sign = direction, magnitude = nominal PWM duty).
> Equal, same-direction rear commands enable local encoder-based RPM balancing,
> which trims rear PWM; unequal rear commands retain direct PWM for steering.
> This includes deliberate differential commands from heading hold, preventing
> the rear equal-RPM controller from undoing the heading correction. IMU dropout
> removes heading correction and requires neutral before heading hold reengages;
> ordinary operator commands still pass through.
> `0x200` applied-left/right are the post-regulation front/rear PWM averages per
> side, not RPM measurements. `0x210` remains the raw signed rear encoder counts.
> See [`firmware/README.md`](../firmware/README.md#rear-wheel-rpm-balancing) for
> controller limits, calibration, and bench validation.

---

## 4. Safety conventions

- **Heartbeat watchdog:** if an edge node misses N consecutive heartbeats (e.g. 3 × 50 ms),
  it enters the safe state: throttle → 0 and hold; steering → hold position.
- **E-stop** (`0x000`) is the highest-priority frame and triggers the safe state immediately
  on every node.
- **Throttle command watchdog:** the throttle node coasts all four outputs after
  150 ms without a valid `0x100` command, even if heartbeat frames continue.
  Heartbeat-only recovery does not refresh an expired command.
- All command messages should carry a rolling counter and/or be idempotent so a stale frame
  cannot cause unintended motion.

---

## 5. Open questions

- Final bitrate after measuring bus length.
- Exact payload encoding (scaling, signedness, endianness) per message.
- Whether per-wheel throttle is independent (4 setpoints) or a single throttle value.
- Encoder feedback as raw counts vs computed speed.
- Need for a separate diagnostic/telemetry ID block for Phase-2 autonomy.
