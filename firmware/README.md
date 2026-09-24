# AVL_CARL Firmware

PlatformIO monorepo for the three CAN nodes. The master is a **dumb proxy**
between a ROS2 companion computer (framed binary over serial) and the CAN bus;
all vehicle-level logic (teleop mixing, arbitration, telemetry parsing) lives
in the ROS2 workspace. Firmware only implements what has to run on the MCUs
for safety and hardware I/O:

- **Master (NUCLEO-F767ZI)** — serial ↔ CAN proxy, heartbeat producer, e-stop
  latch, teleop watchdog, and local MPU-6500 acquisition. IMU samples use
  wire-only telemetry and do not add traffic to the vehicle CAN bus.
- **Throttle (Teensy 4.1)** — 4× IBT_2 H-bridge control, 2× rear encoder
  quadrature decode, fail-safe on heartbeat loss.
- **Steering (Teensy 4.1)** — safe-idle stub (Phase 1 is differential drive).

See [`../docs/wiring.md`](../docs/wiring.md) for the pin-level wiring and
[`../docs/can-bus.md`](../docs/can-bus.md) for the locked CAN message map.
The wire framing between master ↔ companion is defined in
[`lib/carl_protocol/wire.h`](lib/carl_protocol/wire.h) and mirrored in the ROS2
workspace at `src/carl_bridge/carl_bridge/protocol.py`.

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

## Host unit tests

Guards the shared protocol definitions and the wire framing. No hardware.

```bash
pio test -e native
```

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
- **E-stop is one-way from software.** Once latched (via CAN `0x000` or a wire
  frame from ROS2), the master holds it until power cycle. There is no software
  reset command — a deliberate safety choice given the no-brake design.
