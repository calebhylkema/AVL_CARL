# AVL_CARL Firmware

PlatformIO monorepo for the three CAN nodes. The master proxies a ROS2 companion
computer (framed binary over serial) to the CAN bus. Operator mixing, arbitration,
and ROS telemetry parsing live in the ROS2 workspace; rear speed balancing,
safety, and hardware I/O run on the MCUs:

- **Master (NUCLEO-F767ZI)** — serial ↔ CAN proxy, heartbeat producer, e-stop
  latch, and teleop watchdog. Operator wheel commands pass through unchanged
  while the teleop link is healthy and e-stop is clear; otherwise they are zeroed.
- **Throttle (Teensy 4.1)** — 4× IBT_2 H-bridge control, 2× rear encoder
  quadrature decode, rear RPM balancing, fail-safe on heartbeat loss.
- **Steering (Teensy 4.1)** — safe-idle stub (Phase 1 is differential drive).

See [`../docs/wiring.md`](../docs/wiring.md) for the pin-level wiring and
[`../docs/can-bus.md`](../docs/can-bus.md) for the locked CAN message map.
The wire framing between master ↔ companion is defined in
[`lib/carl_protocol/wire.h`](lib/carl_protocol/wire.h) and mirrored in the ROS2
workspace at `../ros2_ws/src/carl_bridge/carl_bridge/protocol.py`.

## Build / flash

```bash
pio run -e throttle      # build ONLY the throttle node
pio run -e master        # build ONLY the master node
pio run -e steering      # build ONLY the steering node
pio run                  # build ALL three nodes

pio run -e throttle -t upload    # flash a connected Teensy throttle node
pio run -e master   -t upload    # flash the NUCLEO over ST-LINK
pio run -e steering -t upload    # flash a connected Teensy steering node
```

## Rear wheel RPM balancing

The throttle Teensy runs a relative PI speed controller in
[`throttle/src/rear_speed_sync.h`](throttle/src/rear_speed_sync.h). When RL and
RR commands are **equal**, have the same direction, and are outside the input
deadband, it adjusts rear PWM to bring their measured RPM magnitudes together.
This works in forward and reverse without assuming encoder A/B polarity. Each
encoder must be connected to its own wheel and have the same effective counts
per wheel revolution (currently 600 PPR × 4 = 2400). Raw signed encoder CAN
telemetry is unchanged.

**Installed encoder mapping:** rear-left uses Teensy A=16/B=17; rear-right
uses A=14/B=15. Hand rotation confirmed that the older assignment was crossed.
With crossed feedback the synchronizer slows the physically slower wheel and
boosts the faster one, driving its outputs to their limits. The corrected pin
assignment fixes control and `/carl/encoders` labels together; no harness swap
or master/ROS protocol change is needed. Flash only the throttle Teensy for
this pin-mapping correction if the master already has the previous updates.

The controller samples at 20 ms, filters speed with an 80 ms time constant,
qualifies feedback for 160 ms, and uses relative speed error to remain
responsive at low throttle. Proportional gain is `KP=0.20`; integral gain is
`KI=0.80` per second. The faster wheel retains at least 75% of requested PWM
and never crosses below the input deadband; the slower wheel
can receive at most an extra 50% of its command, capped at 100 per-mille
(10 percentage points of PWM). Total output stays within 0–100% in the commanded
direction. The integral and correction slew are bounded. Speed matching yields
to these output limits rather than stopping a commanded wheel.

Both encoders must have accumulated at least four counts and have edges within
200 ms before regulation can engage. Missing counts on either side restore
**both** nominal commands; missing feedback is never interpreted as a target of
zero RPM for the other wheel. After feedback is lost during regulation, direct
PWM remains active until neutral, a turn, or reversal starts a new qualification.
Counter jumps implying more than 3000 RPM also bypass regulation. A foreground
sampling gap over 200 ms discards old control state. These are feedback sanity
checks: a disconnected encoder,
an actual stall, and extremely slow rotation cannot be distinguished by counts
alone. Below motor breakaway torque, matching nonzero speed is not guaranteed;
do not compensate by blindly raising controller gains.

Neutral, disarm, heartbeat/command loss, direction reversal, and unequal commands reset
the controller immediately. Unequal commands, one-wheel operation, and pivot
turns retain direct PWM control, preserving differential steering. Front wheels
also retain direct PWM control: rear encoder measurements cannot identify front
motor speed error. Status `applied_left/right` reports the actual post-regulation
front/rear PWM average on each side, including zero during disabled states.

Throttle remains a nominal per-mille PWM request with rear speed balancing;
it is not an absolute RPM setpoint. Maximum geared wheel RPM has not been
calibrated. Large motor differences or output saturation can exceed the allowed
trim authority, leaving a residual speed mismatch. Verify each encoder tracks
its own motor; a swapped mapping can reverse the effect of the feedback loop.

The throttle node independently coasts **all four wheels** if no valid throttle
command arrives for 150 ms, even while heartbeats continue. This reports
`TF_COMMAND_LOST=0x04` / ROS `FAULT_COMMAND_LOST=4`. Heartbeat restoration alone
cannot reapply an expired motor command. Master and throttle watchdog timestamps
are sampled after input processing to avoid millisecond-boundary underflow.
E-stop/reset discards retained master commands, and reset messages do not refresh
the master teleop watchdog. When changing motor direction, the inactive PWM
input is driven low before setting the active input.

Before testing on the ground, support the vehicle with all wheels clear and
keep the e-stop accessible. Verify that rotating each rear wheel changes its
own encoder count, approximately 2400 counts per wheel revolution. Then test
equal forward and reverse commands at 5%, 10%, 25%, and higher throttle, returning
to neutral between directions. Estimate RPM from count differences:
`RPM = abs(delta_counts) * 60000 / (2400 * delta_ms)`, preferably over 200–500 ms
windows at low speed. Confirm the mismatch settles without hunting; reduce
`KP`/`KI` if it oscillates. The initial gains are simulation-tested and require
vehicle validation under load. Test turns, neutral, e-stop, and heartbeat loss
before a ground run.

## Host unit tests

From the repository root, run all six actual-source host regressions with
undefined-behavior checks (requires `g++`; no boards or ROS needed):

```bash
python3 firmware/test/run_host_regressions.py
```

Guards the shared protocol definitions and the wire framing. No hardware.

```bash
pio test -e native
```

The rear speed simulation covers both directions, swapped motor characteristics,
low/high throttle, output saturation, missing/stale counts, counter rollover,
and controller resets. The throttle integration test compiles the actual node
against GPIO/CAN/encoder doubles and checks corrected PWM, raw encoder telemetry,
applied-status values, and stop paths. Run from the repository root:

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/lib/common -Ifirmware/lib/carl_protocol \
  firmware/test/support/throttle/test_rear_speed_sync.cpp \
  -o /tmp/carl-rear-speed-tests
/tmp/carl-rear-speed-tests
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/test/support/throttle -Ifirmware/lib/carl_protocol \
  -Ifirmware/lib/common -Ifirmware/lib/heartbeat \
  firmware/test/support/throttle/test_throttle.cpp -o /tmp/carl-throttle-tests
/tmp/carl-throttle-tests
```

The physical pin-mapping regression runs the actual throttle node with motor
GPIO outputs feeding a motor/inertia model and encoder counts injected at
the installed **pin numbers**, rather than directly into named encoder objects.
It reproduces divergent speeds with the old crossed mapping and checks RPM
convergence in forward/reverse, low/high throttle, either motor being faster,
and raw CAN count labeling with the corrected map:

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/test/support/throttle -Ifirmware/lib/carl_protocol \
  -Ifirmware/lib/common -Ifirmware/lib/heartbeat \
  firmware/test/support/throttle/test_rear_mapping.cpp -o /tmp/carl-rear-mapping-tests
/tmp/carl-rear-mapping-tests
```

The master integration test checks unchanged forward/reverse and differential
commands, serial parsing, malformed input, e-stop/reset, watchdog expiry,
heartbeat flags and timing, and forwarding of throttle, encoder, and steering
telemetry. The idle serial stream contains only heartbeat frames.

## Layout

```
firmware/
├── platformio.ini       # one [env] per node
├── lib/                 # SHARED code — single source of truth across nodes
│   ├── carl_protocol/
│   │   ├── carl_protocol.h  # CAN IDs, timing, enums
│   │   ├── messages.h        # struct <-> CAN payload (de)serialization
│   │   └── wire.h            # serial framing (SOF + CRC8), ROS2 companion link
│   ├── heartbeat/       # HeartbeatTx producer + Watchdog consumer
│   └── common/          # little-endian byte helpers, clamp, map
├── master/   src/main.cpp     # NUCLEO-F767ZI — serial↔CAN proxy
├── throttle/ src/main.cpp     # Teensy 4.1 — motor drive + encoders
├── steering/ src/main.cpp     # Teensy 4.1 — safe-idle stub
└── test/                # host-run Unity tests
```

## Notes

- **Shared `lib/` is the point.** Every node includes `carl_protocol.h`, so
  senders and receivers cannot drift on IDs, timing, or byte layout. `wire.h`
  is the same discipline for the companion link.
- **Two chip families, one build system.** Master = STM32 (`platform = ststm32`),
  throttle & steering = Teensy 4.1 (`platform = teensy`).
- **CAN libraries:** Teensy nodes use the core-bundled `FlexCAN_T4`; the STM32
  master uses `pazi88/STM32_CAN`. The throttle node uses `paulstoffregen/Encoder`.
- **Boot/flash protocols differ per chip** — Teensy uses HalfKay (Teensy Loader;
  tap the button if it doesn't auto-reboot), the NUCLEO uses on-board ST-LINK
  (SWD). PlatformIO handles both via `-t upload`.
- **E-stop latches until explicitly reset.** An operator request with reason 1
  latches the master; reason 0 clears the latch after zeroing retained commands.
  Reset does not keep drive commands alive or restore the previous output.
  The physical e-stop remains independent. Zero output coasts the vehicle.

## Absolute steering encoder telemetry

The steering Teensy now reads the analog encoder on pin 14/A0 and publishes
50 Hz CAN feedback through the master to `/carl/steering_angle`. The CL57T
remains disabled. **Correct the 0–5 V output wiring before powering:** Teensy
4.1 inputs are not 5 V tolerant. See [wiring, calibration, deployment and
verification](../docs/steering-encoder.md).
