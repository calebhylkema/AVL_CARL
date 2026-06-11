# CLAUDE.md — AVL_CARL

Project context for Claude Code. Keep this short; the detailed, living references live in
`docs/`. Update those files (and this one) as the project evolves.

## What this is
AVL_CARL is an Autonomous Vehicle Lab **electric go-kart**. Phase 1 is **drive-by-wire**
(electronic throttle + steering over a CAN bus); Phase 2 adds **autonomy** via a companion
computer on the master node. **There are no mechanical brakes** — the vehicle stops by
commanding 0 throttle and holding 0 to the motors. Treat fail-safe behavior as
safety-critical.

## Architecture in one paragraph
A single classic-CAN 2.0 backbone with three nodes: a **Master/VCU** (NUCLEO-F767ZI), a
**Throttle/motor-drive** node (Teensy 4.1) that drives the 4 wheel motors and reads the
wheel encoders, and a **Steering** node (Teensy 4.1) that drives a CL57T stepper for
Ackermann steering. A second NUCLEO-F767ZI is a spare/future 4th node.

## Source-of-truth docs (read these before answering hardware/architecture questions)
- `docs/hardware.md` — BOM, part numbers, wiring, electrical integration gotchas.
- `docs/architecture.md` — node topology, MCU-to-node rationale, capability comparison.
- `docs/can-bus.md` — CAN bitrate, node IDs, message/ID map.
- `docs/roadmap.md` — phased plan, milestones, open decisions.
- `README.md` — team-facing professional overview (keep in sync with the docs).

## Key facts to remember
- Inventory: 2× **Teensy 4.1** (confirmed) + 2× NUCLEO-F767ZI.
- Both MCU families are **classic CAN only** (no CAN-FD). STM32F767 has CAN1/CAN2 only.
- Encoders: E38S6-600-24G, 600 PPR quadrature, **5–24 V outputs → need level shifting to 3.3 V**.
  Wiring: white=A, green=B, red=VCC, black=0V.
- Motor: MY4845, 24 V 150 W ~2300 rpm + gearbox, ×4. (This is a **motor**, not the battery.)
- Steering driver: CL57T closed-loop stepper, 24 V actuator rail, step/dir/enable (opto inputs).
- CAN transceivers are **not yet purchased** (need 3.3 V-VIO parts, e.g. TJA1051T/3).

## Working agreements
- When hardware or architecture changes, **update the relevant `docs/` file in the same change**.
- Convert relative dates to absolute when recording them.
- `TBD` markers in the docs are real open items — surface them, don't silently fill them in.
- Firmware (not yet written) goes under `firmware/<node>/`.

## Repo / git notes
- The `.claude/` directory is git-ignored (local Claude Code config/state). This `CLAUDE.md`
  file **is** committed so the whole team's Claude Code shares context.
