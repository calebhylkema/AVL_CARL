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
| Termination | **120 Ω at both physical ends** | Two resistors total, not per node. |
| Transceiver | 3.3 V-`VIO` (e.g. TJA1051T/3) | External PHY; not yet purchased. |
| Wiring | Twisted pair (CAN_H / CAN_L) + common GND | Keep stubs short. |

---

## 2. Node IDs

| Node | Board | Node ID (proposed) |
|------|-------|--------------------|
| Master / VCU | NUCLEO-F767ZI | 0x01 |
| Throttle / Motor-drive | Teensy 4.1 | 0x02 |
| Steering | Teensy 4.1 | 0x03 |
| (Future 4th node) | NUCLEO-F767ZI | 0x04 |

---

## 3. Message / ID map (proposed)

Priority is encoded in the ID (lower = higher priority). Reserve the lowest IDs for safety.

| CAN ID | Name | Direction | Period | Payload (proposed) |
|--------|------|-----------|--------|--------------------|
| `0x000` | **E-STOP** | Master → all (or any → all) | event | 1 byte: reason code |
| `0x010` | **Heartbeat** | Master → all | 50 ms | 1 byte counter + status flags |
| `0x100` | Throttle command | Master → Throttle | 20 ms | per-wheel setpoints / throttle % + direction |
| `0x110` | Steering command | Master → Steering | 20 ms | target steering angle / step target |
| `0x200` | Throttle status | Throttle → Master | 50 ms | applied command, fault flags |
| `0x210` | Wheel encoder feedback | Throttle → Master | 20–50 ms | rear-L / rear-R counts or speed |
| `0x220` | Steering status | Steering → Master | 50 ms | current angle, homed flag, fault flags |

> Payload byte layouts are TBD — define exact scaling/endianness when firmware starts.
> Max 8 data bytes per frame (classic CAN). Split larger telemetry across multiple IDs.

---

## 4. Safety conventions

- **Heartbeat watchdog:** if an edge node misses N consecutive heartbeats (e.g. 3 × 50 ms),
  it enters the safe state: throttle → 0 and hold; steering → hold position.
- **E-stop** (`0x000`) is the highest-priority frame and triggers the safe state immediately
  on every node.
- All command messages should carry a rolling counter and/or be idempotent so a stale frame
  cannot cause unintended motion.

---

## 5. Open questions

- Final bitrate after measuring bus length.
- Exact payload encoding (scaling, signedness, endianness) per message.
- Whether per-wheel throttle is independent (4 setpoints) or a single throttle value.
- Encoder feedback as raw counts vs computed speed.
- Need for a separate diagnostic/telemetry ID block for Phase-2 autonomy.
