"""Wire framing for the Jetson <-> STM32 master serial link.

PLACEHOLDER — mirror this against the firmware source of truth:
    firmware/lib/carl_protocol/wire.h   (SOF + length + CRC8 framing)
    firmware/lib/carl_protocol/carl_protocol.h   (CAN IDs, enums)

Keep the framing/IDs here byte-for-byte identical to the firmware, or the master
will drop frames. Fill in the real implementation from wire.h.
"""

# --- Serial link -------------------------------------------------------------
BAUD = 921600           # USART3 on the master (ST-LINK Virtual COM Port)
SOF = 0xAB              # TODO: confirm start-of-frame byte against wire.h

# --- CAN IDs (mirror carl_protocol.h) ---------------------------------------
CAN_ESTOP = 0x000
CAN_HEARTBEAT = 0x010
CAN_THROTTLE_CMD = 0x100
CAN_STEERING_CMD = 0x110
CAN_THROTTLE_STATUS = 0x200
CAN_ENCODER_FEEDBACK = 0x210
CAN_STEERING_STATUS = 0x220


def crc8(data: bytes) -> int:
    """TODO: implement the same CRC8 polynomial used in wire.h."""
    raise NotImplementedError("Port CRC8 from firmware/lib/carl_protocol/wire.h")


def encode_frame(can_id: int, payload: bytes) -> bytes:
    """Wrap a CAN frame into a serial wire frame. TODO: match wire.h."""
    raise NotImplementedError("Implement wire framing to match firmware")


def decode_frame(buf: bytes):
    """Parse a serial wire frame into (can_id, payload). TODO: match wire.h."""
    raise NotImplementedError("Implement wire de-framing to match firmware")
