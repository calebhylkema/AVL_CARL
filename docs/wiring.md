# Wiring Reference — AVL_CARL (Phase 1, differential drive)

> Living document. Last updated: 2026-06-11.
> This is the pin-level companion to [`hardware.md`](hardware.md) (BOM/ratings) and
> [`can-bus.md`](can-bus.md) (protocol). **The firmware pin maps in `firmware/` must match
> this file** — change both together.

Phase 1 drive actuation uses **throttle + master**. Power the steering Teensy
for absolute-encoder telemetry (its motor remains disabled); the kart
turns by **differential drive** (left-side vs right-side wheel speed) handled entirely on the
throttle node. ⚠️ All power and grounds are assumed already handled; this doc covers signal
wiring. **Common ground across all 24 V / 5 V / 3.3 V domains is mandatory.**

---

## 0. Pin map at a glance

| Function | Teensy 4.1 (THROTTLE, 0x02) | NUCLEO-F767ZI (MASTER, 0x01) |
|----------|-----------------------------|------------------------------|
| CAN TX | pin **22** (CAN1_TX) | **PD1** (CAN1_TX) |
| CAN RX | pin **23** (CAN1_RX) | **PD0** (CAN1_RX) |
| Teleop serial | — | ST-LINK **VCP** (USART3, PD8/PD9) over the USB cable |
| MPU-6500/9250 SPI | — | **PE6 MOSI / PE5 MISO / PE2 SCK**, SPI4, CN9 Zio cluster |
| Status LED | pin 13 (on-board) | on-board LED |

---

## 1. CAN bus (the backbone)

Three transceivers (Adafruit TJA1051T/3), one twisted pair `CAN_H` / `CAN_L` + common GND.

### 1a. Adafruit TJA1051T/3 breakout ↔ MCU

Breakout pads: `VCC` `GND` `RX` `TX` `SLNT` `CANH` `CANL`.

| Breakout pin | Throttle Teensy 4.1 | Master NUCLEO-F767ZI | Notes |
|--------------|---------------------|----------------------|-------|
| **VCC**  | 3V3 | 3V3 | 3.3 V logic; board boosts to the 5 V the bus side needs (see VCC note) |
| **GND**  | GND | GND | common ground — also tie to the bus GND wire |
| **RX**   | pin **23** = **CRX1** | **PD0** (CAN1_RX) | breakout RX = transceiver output → MCU CAN **RX** |
| **TX**   | pin **22** = **CTX1** | **PD1** (CAN1_TX) | breakout TX = transceiver input → MCU CAN **TX** |
| **SLNT** | GND | GND | silent-mode select — **must be LOW** for normal TX/RX |
| **CANH** | bus CANH | bus CANH | twisted pair, daisy-chained to all nodes |
| **CANL** | bus CANL | bus CANL | twisted pair |

**Verified CAN-controller pins.** Teensy 4.1 FlexCAN is hard-tied per controller (these are
the `CTXn/CRXn` pins on the Teensy pinout card — not remappable) — CAN1 = pin 22 (CTX1) /
pin 23 (CRX1) [CAN2 = 1/0, CAN3 = 31/30]. STM32F767 bxCAN CAN1 (AF9) can
map to PA11/PA12 (=USB OTG, avoid), **PD0/PD1**, PB8/PB9, or PI9/PH13; the master firmware
uses **PD0 (RX) / PD1 (TX)** (`STM32_CAN Can1(CAN1, ALT_2)`). To use PB8/PB9 instead (they sit
at the Arduino `D15/D14` pads on CN7), change the firmware to `... ALT`.

> **No crossover.** Wire `TX→TX` and `RX→RX`. The breakout labels are already from the MCU's
> point of view; the transceiver does the internal swap. Crossing them is the #1 mistake.
>
> **SLNT must be grounded.** On the TJA1051 the S pin HIGH/floating = silent (listen-only) —
> the node will receive but never ACK or transmit, which looks like a dead bus. Tie it to GND
> (or to a spare GPIO held LOW if you ever want software listen-only).
>
> **VCC voltage — confirm on your board.** The TJA1051T/3 is the 3.3 V-logic variant, so RX/TX/
> SLNT are native 3.3 V (no level shifting). This breakout takes 3.3 V on VCC and generates the
> 5 V CAN supply with its on-board boost. *If your specific board exposes a separate `VIO` pad*,
> put VIO = 3.3 V and VCC = 5 V instead. Check the silkscreen before powering.
>
> CANH/CANL of all transceivers tie to the same twisted pair.

### 1b. Termination — exactly TWO, at the two physical bus ends

```
  [ end node ]======twisted pair======[ middle node ]======twisted pair======[ end node ]
   120 Ω ON                              120 Ω OFF                              120 Ω ON
```

- Enable the on-board 120 Ω **only on the two nodes at the physical ends** of the cable.
- The node in the **middle** must have its 120 Ω **disabled**. Three terminators is the #1
  CAN bring-up failure.

**Your current setup:** the master is wired as the middle node (termination **OFF**). The
throttle node and the steering node are the two ends (termination **ON** on both). You have
already set the steering transceiver to terminate.

> ⚠️ **Gotcha with the unpowered steering end:** the 120 Ω resistor is passive and works even
> with that Teensy unpowered, but its **transceiver IC will be unpowered**, which can lightly
> load the bus. For a clean 2-node Phase-1 bring-up, prefer **one of these**:
> 1. Power just the steering transceiver breakout (3.3 V + 5 V + GND) even though its Teensy
>    stays unpowered, **or**
> 2. Make the two **powered** nodes the bus ends — i.e. terminate **master + throttle** and
>    leave steering as an unterminated mid/stub tap.
>
> Either gives two clean terminators between two live transceivers. If the bus is short and
> 500 kbit/s is solid on the scope, the as-wired arrangement will usually work too.

---

## 2. Motor drivers — 4× IBT_2 (BTS7960) ↔ throttle Teensy

Each IBT_2 has 8 logic pins: `RPWM`, `LPWM`, `R_EN`, `L_EN`, `R_IS`, `L_IS`, `VCC`, `GND`.
We tie **`R_EN` + `L_EN` of a module together to one Teensy GPIO** (HIGH = enabled).
`R_IS`/`L_IS` (current sense) are **not wired** in Phase 1.

| Motor (wheel) | IBT_2 `RPWM` → Teensy | IBT_2 `LPWM` → Teensy | IBT_2 `R_EN`+`L_EN` → Teensy | `VCC` | `GND` |
|---------------|------------------------|------------------------|------------------------------|-------|-------|
| **FL** front-left  | pin **2** | pin **3** | pin **10** | 5 V | GND |
| **FR** front-right | pin **4** | pin **5** | pin **11** | 5 V | GND |
| **RL** rear-left   | pin **8** | pin **9** | pin **24** | 5 V | GND |
| **RR** rear-right  | pin **6** | pin **7** | pin **12** | 5 V | GND |

These rear assignments match the bring-up swaps in the current throttle firmware.
Positive wheel commands mean vehicle-forward. The right motors are electrically
reversed, so firmware uses `kMotorDir={+1,-1,+1,-1}`: vehicle-forward FR drives
**LPWM pin 5**, vehicle-reverse FR drives **RPWM pin 4**, with both enables on
**pin 11**. Both PWM inputs are zero for coast/zero output.
Power side of each IBT_2: `B+`/`B-` to the 24 V traction rail, `M+`/`M-` to the motor.

For a front-right-only failure in both directions, see
[`joystick-troubleshooting.md`](joystick-troubleshooting.md). A firmware e-stop
disables all four motor channels, not a single corner.

> ⚠️ **Logic level:** BTS7960 inputs are 5 V parts. Teensy drives 3.3 V. Verify `VIH` on a
> scope; if the inputs are marginal, level-shift `RPWM`/`LPWM`/`EN` to 5 V (see
> [`hardware.md` §7.3](hardware.md#7-electrical-integration-notes-gotchas)). Each IBT_2 logic
> `GND` **must** tie to the Teensy GND.

PWM: 20 kHz, 12-bit (set in firmware). The two PWM pins of a wheel never drive simultaneously.

---

## 3. Wheel encoders — 2× E38S6-600-24G → BSS138 level shifter → throttle Teensy

Rear wheels only (600 PPR quadrature). Colors per [`hardware.md` §4](hardware.md#4-sensing--wheel-encoders-e38s6-600-24g).
Teensy 4.1 pins are **3.3 V only / NOT 5 V-tolerant**, and the E38S6 outputs swing to their
supply rail — so the A/B lines go through a **BSS138 4-channel bidirectional level shifter**
(the board with `5V GND HV1..HV4` on one side and `3V3 GND LV1..LV4` on the other).

**Decision: level shifter YES, separate pull-up resistors NO.** The BSS138 board has a 10 kΩ
pull-up built into every channel. On the HV side that pull-up doubles as the pull-up an
open-collector encoder needs; the MOSFET does the 5 V↔3.3 V translation. This works for an
open-collector **or** a push-pull encoder, so the output type does not need to be determined.

| Signal | Encoder wire | → Shifter `HV` | Shifter `LV` → | Teensy pin |
|--------|--------------|----------------|----------------|------------|
| Rear-Left  A  | white | **HV3** | **LV3** | **16** |
| Rear-Left  B  | green | **HV4** | **LV4** | **17** |
| Rear-Right A  | white | **HV1** | **LV1** | **14** |
| Rear-Right B  | green | **HV2** | **LV2** | **15** |

The encoder pairs above match the installed harness: rotating the physical
rear-left wheel by hand changed the old firmware's rear-right count (16/17).
The throttle firmware now assigns 16/17 to RL and 14/15 to RR. Leave the harness
as-is with this firmware; swapping the wires as well would cross the mapping
again. Both feedback regulation and CAN count labels use this physical mapping.

Level-shifter power/reference (all required — the pull-ups don't work otherwise):

| Shifter pin | Connect to |
|-------------|------------|
| `5V` (HV ref) | 5 V rail |
| `3V3` (LV ref) | Teensy **3.3 V** |
| `GND` (both sides) | common ground |

Each encoder: **red (VCC) → 5 V**, **black (0 V) → common GND**. **Never wire 24 V** anywhere
near the Teensy or the LV side.

> The built-in 10 kΩ pull-ups are fast enough for the encoder edge rate here (~10–25 kHz even
> at full kart speed). Rare exception: if your specific shifter lacks built-in pull-ups, add a
> 4.7–10 kΩ from each `HVx` to 5 V (the `5V/GND/HV1-4 + 3V3/GND/LV1-4` board is the BSS138 type,
> which has them). Confirm clean quadrature on a scope before trusting the counts.

---

## 4. Master ↔ PC (teleop)

A single USB cable from the PC to the NUCLEO's ST-LINK port does **both**:

1. Flashes the master (ST-LINK / SWD), and
2. Exposes the **Virtual COM Port** the ROS2 bridge talks to (921600 8N1).

No extra wiring. Find the port with `python firmware/test/teleop.py --list`.

### 4a. GY-BNO08X SPI ↔ master NUCLEO-F767ZI (historical)

**Current `master` firmware uses the replacement MPU module in §4b.** The
following BNO pinout is retained for reference and is incompatible with the
MPU configuration; the SDA/AD0 signal roles differ.

The current master uses the photographed purple **GY-BNO08X** breakout,
not the former MPU-6500. Power OFF before rewiring. Disconnect the MPU and its
former PB8/PB9 or PF0/PF1 wires. In SPI mode **AD0 is MOSI**, not an address
strap: do NOT retain the old AD0-to-GND connection.

The reinstated master uses **SPI4 on CN9**, replacing the previous CN7
PB5/PB4/PB3 SPI1 assignment and PD12 wake connection. All seven signal wires
and ground occupy consecutive sockets along **CN9's even row, pins 12–26**.
Power comes from CN8. Only CN8/CN9 are required; **CN11/CN12 are not used** and
no additional headers need soldering. Use the header's D labels and pin-1
marker, not left/right guesses from photos.

| GY-BNO08X pin | NUCLEO connection | Function |
|---|---|---|
| `VCC-3V3` | CN8 pin 7 / 3V3 | 3.3 V only |
| `GND` | CN9 pin 12 / GND | Common ground, beside the signal cluster |
| `SCL` | PE2 / D56 / CN9 pin 14 | SPI4 clock |
| `CS` | PE4 / D57 / CN9 pin 16 | Active-LOW chip select, software GPIO |
| `SDA` | PE5 / D58 / CN9 pin 18 | MISO: IMU → STM32 |
| `AD0` | PE6 / D59 / CN9 pin 20 | MOSI: STM32 → IMU |
| `RST` | PE3 / D60 / CN9 pin 22 | Active-LOW sensor reset |
| `INT` | PF8 / D61 / CN9 pin 24 | Active-LOW attention; required |
| `PS0` | PF7 / D62 / CN9 pin 26 | HIGH during reset/startup; active-LOW wake afterward |
| `PS1` | 3.3 V (same rail as VCC) | SPI startup selection |
| `BOOT` pad | Leave untouched | Normal firmware, not bootloader |

No solder bridges need to be closed for this wiring: PS1 is explicitly powered
HIGH and firmware controls PS0. The pictures show open PS0/PS1 solder pads and
a crystal can; they do not resolve the chip's exact 080/085/086 suffix or prove
the module is electrically good. Do not bridge a pad whose function is unknown.
External I2C pull-ups are not required for these SPI signals. Keep wiring short
and away from motor cables. A breadboard is suitable for the initial bench test.

The firmware explicitly uses `SPIClass(PE6, PE5, PE2)`, which selects
**SPI4, AF5** with the installed STM32 Arduino core. The board's SAI labels on
these sockets do not prevent their use for SPI/GPIO. PE2 is also exposed on
CN10 pin 25 / D31: leave that duplicate socket free. PF7/PF8 are GPIOs here,
not SPI5 or UART7 signals. No solder-bridge routing changes are required.
CAN PD0/PD1, companion USART3 PD8/PD9, and ST-LINK SWD PA13/PA14 are unchanged.

SPI runs MSB-first, mode 3, requested clock 1 MHz (hardware divider may make it
slower). PS0 stays HIGH while startup packets are drained. Firmware waits for
the executable channel's Reset Complete message before requesting product ID
or enabling reports; the first INT alone does not establish readiness. PS0
then provides wake handshaking. Startup, identification, and configuration are
scheduled states; there are no delay loops waiting for INT. The driver requests acceleration,
calibrated gyro, and rotation-vector reports at 100 Hz. A snapshot is sent only
after each report has updated and each is at most 50 ms old by STM32 receipt
time; the three sensor reports are not guaranteed simultaneous. Loss of any
report for 250 ms triggers sensor-only recovery after a one-second backoff.

For a startup `invalid_packet`, inspect
`ros2 topic echo /carl/imu/spi_diagnostics --once --full-length`.
It includes the last raw four-byte SHTP receive header and GPIO levels sampled
before that receive, retained across sensor reset attempts. `not_attempted`
means no receive snapshot exists yet. An all-FF header is consistent with an
undriven/high MISO line or wrong mode, but does not prove either; check wiring
and power or capture a logic-analyzer trace. `empty` with an all-zero header
is inconclusive. A nonempty length outside 4..512 or channel above 5 is rejected.
This instrumentation does not change SPI clock/mode, reset timing, or retry
behavior. It requires updated master firmware and bridge; older bridges ignore
the additional wire ID.

Inspect `ros2 topic echo /carl/imu/diagnostics --full-length`. Missing INT gives
`boot_timeout`; an INT without Reset Complete also times out in `boot_wait`
(`detail=0x01` when INT is asserted at timeout). Failure to identify gives
`identify_timeout`; missing streaming data gives `report_timeout`.
Errors retain a saturating count across recovery,
cleared only on STM32 restart. Only this sensor is reset, never CAN or STM32.
The BNO does not provide the old MPU die-temperature stream: a temperature
panel with no messages is expected, not proof of an IMU failure.

References: [ST UM1974, Table 19](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo-144-boards-mb1137-stmicroelectronics.pdf),
[STM32F767 alternate functions](https://www.st.com/resource/en/datasheet/stm32f767zi.pdf),
[CEVA BNO08x, section 1.2.4](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf),
[CEVA SH-2 startup handshake (`sh2_open`)](https://github.com/ceva-dsp/sh2/blob/master/sh2.c),
[SH-2 report definitions](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf).

---

## 5. Steering node (Phase 1: parked)

The CL57T stays disabled. Power the steering Teensy and CAN transceiver for
encoder feedback. Pin 14/A0 receives the analog encoder **through a 10k/10k
voltage divider**, never directly from a 0–5 V sensor. See
[absolute encoder wiring and verification](steering-encoder.md).

Reserved CL57T pins for when steering actuation comes online (verify opto logic levels first):
Teensy `STEP` = pin 3, `DIR` = pin 4, `ENA` = pin 5; CAN on pins 22/23 like the throttle node.
The safe-idle firmware keeps the CL57T disabled and never steps.

---

## 6. Power-on / bring-up order (bench, wheels off the ground)

1. Confirm common ground everywhere; confirm CAN termination = exactly two ends.
2. Power logic rails (3.3 V/5 V) **before** the 24 V traction rail.
3. Flash master (ST-LINK) and throttle (Teensy Loader). Boot protocols differ per chip —
   see [`firmware/README.md`](../firmware/README.md).
4. Open `teleop.py`; verify `TLM` telemetry streams and `link=1`.
5. With wheels **off the ground**, test level 1 + `w`/`s`/`a`/`d`, then `x`. Keep the hardware
   kill switch in hand.

### 4b. Installed GY-6500/GY-9250 module — master SPI

Use the standard firmware target **`master`** for this module. Power off before swapping modules; connect one IMU
at a time. The physical replacement returned WHO_AM_I **0x71** (MPU-9250).

| Replacement pin | NUCLEO connection | Function |
|---|---|---|
| `VCC` | CN8 pin 7 / 3V3 | 3.3 V power |
| `GND` | CN9 pin 12 / GND | Common ground |
| `SCL/SCLK` | PE2 / D56 / CN9 pin 14 | SPI4 clock |
| `NCS` | PE4 / D57 / CN9 pin 16 | Software chip select |
| `AD0/SDO` | PE5 / D58 / CN9 pin 18 | MISO: module → STM32 |
| `SDA/SDI` | PE6 / D59 / CN9 pin 20 | MOSI: STM32 → module |
| `INT` | PF8 / D61 / CN9 pin 24 | May remain connected; test polls INT_STATUS instead |
| `FSYNC` | CN8 pin 11 / GND | Ground when unused |
| `EDA`, `ECL` | Disconnected | Auxiliary external-sensor bus |

Disconnect the former BNO RST (PE3 / CN9-22), PS0/WAKE (PF7 / CN9-26),
and PS1 supply wire. The MPU has no matching reset/wake connections.
**The SDA/AD0 roles differ from the BNO:** move former BNO SDA to MPU AD0/SDO,
and former BNO AD0 to MPU SDA/SDI. Keep the STM32 ends in their existing sockets.
The MPU provides raw accelerometer/gyro/temperature telemetry. The master also
uses stationary-calibrated gyro data for bounded planar relative heading hold;
it does not publish BNO quaternions. See
[calibration and mounting limits](../firmware/README.md#mpu-relative-heading-hold).
