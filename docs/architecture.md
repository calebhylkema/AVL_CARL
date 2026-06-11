# Architecture — AVL_CARL

> Living document. Last updated: 2026-06-10.

This file documents the distributed control architecture, the microcontroller-to-node
assignment, and the engineering rationale behind it.

---

## 1. Topology

A single classic-CAN backbone connects three nodes. The master arbitrates commands;
edge nodes execute and report feedback. A fourth board is held as a spare.

```
   Companion compute (Phase 2) ──Ethernet──┐
                                           │
                                    ┌──────┴──────┐
                                    │ MASTER / VCU│  NUCLEO-F767ZI
                                    └──────┬──────┘
   ════════════════ CAN 2.0 bus (120 Ω both ends) ════════════════
            │                                         │
   ┌────────┴────────┐                       ┌────────┴────────┐
   │ THROTTLE/DRIVE  │ Teensy 4.1            │   STEERING      │ Teensy 4.0
   │ 4× motor PWM    │                       │ step/dir/enable │
   │ encoder decode  │                       │   → CL57T       │
   └─────────────────┘                       └─────────────────┘
```

## 2. Node responsibilities

| Node | Board | Responsibilities |
|------|-------|------------------|
| **Master / VCU** | NUCLEO-F767ZI | Source/arbitrate commands, emit heartbeat, run the watchdog/e-stop logic, expose the future autonomy interface (Ethernet to a companion computer), aggregate telemetry. |
| **Throttle / Motor-drive** | Teensy 4.1 | Receive throttle/motion commands over CAN; drive the 4 wheel motors via PWM; read the wheel encoders (quadrature) for speed/feedback; enforce fail-safe (motors → 0) on heartbeat loss. |
| **Steering** | Teensy 4.0 | Receive steering-angle commands over CAN; generate `step/dir/enable` to the CL57T; handle homing/limits; hold position on heartbeat loss. |
| **Spare** | NUCLEO-F767ZI | Reserved for a future 4th node (BMS, sensor hub, or redundancy). |

## 3. MCU-to-node rationale

The assignment maps each board's strengths to each node's dominant requirement:

- **Master = NUCLEO-F767ZI** — only board with a **built-in Ethernet MAC**, which is the
  natural Phase-2 autonomy/companion-computer link. Cortex-M7 + FPU and a large GPIO
  breakout give supervisory headroom. 2× bxCAN is sufficient (backbone + spare).
- **Throttle = Teensy 4.1** — the most I/O-heavy node. Teensy 4.x has **hardware
  quadrature decoders** (no CPU cost to track encoders) plus abundant **FlexPWM** channels
  for 4 motor drivers, all at 600 MHz with tightly-coupled memory for deterministic timing.
  3× FlexCAN. The 4.1 (vs 4.0) is preferred here for extra I/O and optional Ethernet.
- **Steering = Teensy 4.0** — simplest node; only needs clean timer-based step pulses plus
  a few GPIO. Same Teensyduino toolchain as the throttle node keeps firmware consistent.
- **Spare = NUCLEO-F767ZI** — identical to the VCU, so a future safety/BMS/sensor node or a
  redundant master is a drop-in.

## 4. Capability comparison

| Capability | Teensy 4.0 | Teensy 4.1 | NUCLEO-F767ZI |
|------------|-----------|-----------|----------------|
| Classic CAN 2.0 controllers | 2 (CAN1/2) | 3 (CAN1/2/3) | 2 (CAN1/2) |
| CAN-FD | No | No | No |
| Hardware quadrature encoders | Yes (ENC module, 4 ch) | Yes (ENC module, 4 ch) | Yes (timer encoder mode, many timers) |
| PWM channels | Many (FlexPWM + QuadTimer) | Many (FlexPWM + QuadTimer) | Many (14 timers × up to 4 ch) |
| Clock | 600 MHz | 600 MHz | 216 MHz |
| Flash / RAM | 2 MB / 1 MB | 8 MB / 1 MB | 2 MB / 512 KB |
| GPIO | 40 | 55 | up to 144 |
| Ethernet | None | Built-in MAC + PHY + RJ45 | Built-in MAC (+ on-board RJ45) |
| Toolchain | Teensyduino / PlatformIO | Teensyduino / PlatformIO | STM32CubeIDE/HAL / PlatformIO |
| Real-time | Bare-metal, DTCM, low jitter | Bare-metal, DTCM, low jitter | FreeRTOS-capable; deterministic with ISR tuning |

> Specs verified against PJRC (teensy techspecs), ST (STM32F767ZI / NUCLEO-F767ZI), and the
> FlexCAN_T4 / STM32 bxCAN documentation. Both MCU families are **classic CAN only**.

## 5. Fail-safe behavior

Consistent with the **no-brake** design (stop = command 0 and hold):

- The master emits a periodic **heartbeat** on the CAN bus.
- Each edge node runs a **watchdog** on that heartbeat. On timeout it enters a safe state:
  - Throttle node → command **0** to all motors and hold.
  - Steering node → **hold** current steering position (do not free-run the stepper).
- An **e-stop** message (highest CAN priority) forces the same safe state immediately.
- A hardware power cutoff / kill switch is the ultimate stop and must always be present.

## 6. Caveats

- **Confirm Teensy models.** The plan assumes one 4.1 (throttle) and one 4.0 (steering).
  If both are 4.0, the throttle node loses optional Ethernet and some GPIO — still workable.
- **No CAN-FD / no CAN3 on F767.** STM32F767 has only CAN1/CAN2. Payloads are capped at
  8 bytes (classic CAN) — adequate for the command/feedback set.
- **Encoder level shifting** (5–24 V → 3.3 V) and **CL57T input levels** (3.3 V vs opto)
  must be solved in hardware — see [`hardware.md`](hardware.md#7-electrical-integration-notes-gotchas).
- **CAN transceivers** are not yet in hand; pick 3.3 V-`VIO` parts (e.g. TJA1051T/3).

## 7. Future (Phase 2 — autonomy)

- Companion computer (e.g. SBC/Jetson-class) connects to the **master over Ethernet**.
- Master translates high-level autonomy commands into CAN command messages.
- The spare NUCLEO-F767ZI becomes a sensor-aggregation node (IMU, range sensors) or a
  redundant/safety controller.
