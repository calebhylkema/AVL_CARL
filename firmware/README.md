# AVL_CARL Firmware

PlatformIO monorepo for the three CAN nodes. Source files are currently **empty stubs** —
this is the folder/file skeleton; implementation comes later.

## Build / flash

```bash
pio run -e throttle      # build/flash ONLY the throttle node
pio run -e master        # build/flash ONLY the master node
pio run -e steering      # build/flash ONLY the steering node
pio run                  # build ALL three nodes
```

> `pio run -e throttle` builds/flashes just that node; `pio run` builds all three.

Add `-t upload` to flash a connected board (e.g. `pio run -e throttle -t upload`),
and `pio run -e throttle -t monitor` (or `pio device monitor`) for the serial console.

## Layout

```
firmware/
├── platformio.ini       # one [env] per node — builds all 3 from one config
├── lib/                 # SHARED code (single source of truth across nodes)
│   ├── carl_protocol/   # CAN IDs, node IDs, bitrate, heartbeat/e-stop constants
│   │   ├── carl_protocol.h
│   │   └── messages.h   # struct <-> CAN frame (de)serialization
│   ├── heartbeat/       # shared heartbeat + watchdog fail-safe logic
│   └── common/          # shared types, utils, fixed-point helpers
├── master/   src/main.cpp   include/   # NUCLEO-F767ZI — VCU / arbiter
├── throttle/ src/main.cpp   include/   # Teensy 4.1 — 4-motor drive + encoders
├── steering/ src/main.cpp   include/   # Teensy 4.1 — CL57T stepper
└── test/                # host-run unit tests (e.g. message pack/unpack)
```

## Notes

- **Shared `lib/` is the point.** The CAN message definitions in `carl_protocol/` are the
  single source of truth — every node depends on them so senders and receivers can't drift.
  Keep these in sync with [`../docs/can-bus.md`](../docs/can-bus.md).
- **Two chip families, one build system.** Master = STM32 (`platform = ststm32`),
  throttle & steering = Teensy 4.1 (`platform = teensy`). PlatformIO builds both from this
  one `platformio.ini`.
- The `framework = arduino` lines are a sensible default for bring-up; the master can be
  switched to the STM32Cube/HAL framework later if needed.
