# Hardware Reference — AVL_CARL

> Living document. Update this in the same commit whenever hardware changes.
> Last updated: 2026-06-10.

This file is the single source of truth for the bill of materials, part numbers,
electrical ratings, wiring, and integration notes. `TBD` marks information the team is
still gathering.

---

## 1. Bill of Materials (BOM)

| Subsystem | Item | Part / Model | Qty | Key ratings | Status |
|-----------|------|--------------|-----|-------------|--------|
| Propulsion | Traction motor | **MY4845** | 4 | 24 V, 150 W, ~2300 rpm, w/ gearbox | On vehicle |
| Propulsion | Motor driver | TBD (PWM-input) | 4? | TBD | TBD |
| Steering | Stepper driver | **CL57T** closed-loop | 1 | 24 V actuator rail; step/dir/enable | On vehicle |
| Steering | Stepper motor | TBD (closed-loop NEMA) | 1 | TBD | On vehicle |
| Sensing | Wheel encoder | **E38S6-600-24G** (NO:1001119) | 2 (rear) | 600 PPR, quadrature, 5–24 V | On vehicle |
| Compute | Microcontroller | **Teensy 4.1** | 2 | 600 MHz Cortex-M7 | In hand |
| Compute | Microcontroller | **NUCLEO-F767ZI** | 2 | 216 MHz Cortex-M7, Ethernet | In hand |
| Comms | CAN transceiver | TBD (e.g. TJA1051T/3) | 3+ | 3.3 V VIO recommended | **Not yet purchased** |
| Power | Battery pack | **TBD** | 1 | ~24 V (capacity TBD) | TBD |

---

## 2. Propulsion

### Traction motors — MY4845
- 24 V DC motor, 150 W, ~2300 rpm rated, coupled to a **gearbox**.
- **4 units**, one per wheel → independent per-wheel drive.
- ⚠️ **Clarification:** MY4845 is the **motor**, not the battery. The main battery pack is a
  separate item (see Power) still to be documented.
- Driven via PWM-input motor drivers (model TBD), commanded by the **throttle node**.

**Open items**
- Motor driver model + PWM frequency/resolution requirements.
- Current draw per motor (for fusing / wire gauge).
- Whether all 4 wheels are driven or only the rear pair (rear has the encoders).

---

## 3. Steering

- **Geometry:** trapezoidal **Ackermann** linkage.
- **Actuator:** stepper motor driven by a **CL57T** closed-loop stepper driver.
  - Interface: `PUL/step`, `DIR`, `ENA/enable` (optocoupler inputs).
  - Powered from the **24 V actuator rail**.
  - Driver manual: <https://www.omc-stepperonline.com/download/CL57T_V4.0.pdf>
- Driven by the **steering node**.

**Open items**
- Stepper motor model + steps/rev + microstep setting on the CL57T.
- Steering end-stop / home strategy (limit switch vs current sensing vs position pot).
- CL57T input logic level vs 3.3 V MCU drive (see Integration Notes).

---

## 4. Sensing — Wheel encoders (E38S6-600-24G)

| Spec | Value |
|------|-------|
| Type | Incremental rotary, quadrature (A/B) |
| Resolution | **600 PPR** (pulses/rev) → 2400 counts/rev with 4× quadrature decoding |
| Supply / output | 5–24 V |
| Part / serial | E38S6-600-24G · NO:1001119 |
| Location | 2× rear wheels (front/additional TBD) |

**Wiring (color code)**

| Wire color | Signal |
|------------|--------|
| White | OUT A |
| Green | OUT B |
| Red | VCC (5–24 V) |
| Black | 0 V (GND) |

**Open items**
- Confirm total encoder count (front wheels?) — *user checking 2026-06-11*.
- Confirm output type (open-collector vs push-pull) to size pull-ups correctly.

---

## 5. Compute

| Board | Role | Notes |
|-------|------|-------|
| NUCLEO-F767ZI #1 | Master / VCU | Ethernet for autonomy link; 2× bxCAN |
| Teensy 4.1 | Throttle / motor-drive | HW quadrature + FlexPWM; 3× FlexCAN |
| Teensy 4.1 | Steering | step/dir/enable to CL57T |
| NUCLEO-F767ZI #2 | Spare / future 4th node | BMS / sensor hub / redundancy |

See [`architecture.md`](architecture.md) for the full capability comparison and rationale.

> Teensy models confirmed: **both are Teensy 4.1**.

---

## 6. Power

- **System voltage:** 24 V.
- **Actuator rail:** 24 V (powers CL57T).
- **Logic rails:** 5 V (encoder supply, possibly motor-driver logic) and 3.3 V (MCU logic).
- **Battery pack:** TBD — capacity, chemistry, BMS, fusing all to be documented.

**Open items**
- Battery pack specification + BMS.
- Main fuse / breaker rating and hardware kill switch location.
- DC-DC converters for 5 V / 3.3 V rails.

---

## 7. Electrical Integration Notes (gotchas)

These must be resolved during bring-up:

1. **Encoder level shifting — CRITICAL.**
   E38S6 outputs swing to the supply rail (5–24 V); MCU GPIO is **3.3 V tolerant only**.
   - Recommended: power the encoder at **5 V**, use pull-ups to 3.3 V (≈4.7 kΩ) if
     open-collector, or a divider to a clean 3.3 V swing if push-pull.
   - Verify with a scope/multimeter before connecting to the MCU. **Do not** wire 24 V
     encoder output directly to a 3.3 V pin.

2. **CL57T step/dir/enable logic level.**
   Optocoupler inputs. Confirm the driver version/mode accepts 3.3 V logic; otherwise use
   a series resistor sized for the opto LED current, or drive at 5 V.

3. **CAN transceiver logic level.**
   Use a transceiver with a 3.3 V `VIO` (e.g. **TJA1051T/3**) so the PHY matches MCU logic.
   Terminate the bus with **120 Ω at both physical ends**.

4. **Common ground.**
   All 24 V / 5 V / 3.3 V domains must share a common ground reference.

---

## 8. Reference links

- CL57T driver manual — <https://www.omc-stepperonline.com/download/CL57T_V4.0.pdf>
- Teensy tech specs — <https://www.pjrc.com/teensy/techspecs.html>
- NUCLEO-F767ZI — <https://www.st.com/en/evaluation-tools/nucleo-f767zi.html>
- TJA1051 CAN transceiver — <https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf>
