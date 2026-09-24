"""Framing and packing for the master <-> companion computer link.

Frame format (mirrors CAN semantics so firmware translation is trivial):

    [SOF=0xAA][MSG_ID: u16 LE][LEN: u8][PAYLOAD ...][CRC8]

MSG_IDs reuse the CAN IDs from docs/can-bus.md so the master just proxies
between wire format and CAN frames.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

SOF = 0xAA

MSG_ESTOP = 0x000
MSG_HEARTBEAT = 0x010
MSG_THROTTLE_CMD = 0x100
MSG_THROTTLE_STATUS = 0x200
MSG_ENCODER_FEEDBACK = 0x210
MSG_IMU_ACCEL = 0x230
MSG_IMU_GYRO = 0x231

# These status bits mirror firmware/lib/carl_protocol/imu_telemetry.h. The
# bridge publishes only complete samples marked valid by the STM32.
IMU_STATUS_INITIALIZED = 0x01
IMU_STATUS_SAMPLE_VALID = 0x02

# The STM32 configures the MPU-6500 for its narrowest accelerometer and
# gyroscope ranges. Keep these constants beside the decoder so the conversion
# cannot silently drift away from the firmware configuration.
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DEGREE_PER_SECOND = 131.0
STANDARD_GRAVITY_MPS2 = 9.80665

AxisTuple = Tuple[int, int, int]


@dataclass(frozen=True)
class ImuAxisTelemetry:
    """One decoded eight-byte acceleration or gyroscope wire payload."""

    sequence: int
    status: int
    x: int
    y: int
    z: int


@dataclass(frozen=True)
class ImuRawSample:
    """Matching raw accelerometer and gyroscope halves from one IMU sample."""

    sequence: int
    status: int
    accel: AxisTuple
    gyro: AxisTuple


def crc8(data: bytes, poly: int = 0x07) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def frame(msg_id: int, payload: bytes) -> bytes:
    header = struct.pack('<BHB', SOF, msg_id, len(payload))
    body = header + payload
    return body + bytes([crc8(body)])


def pack_throttle_cmd(fl: float, fr: float, rl: float, rr: float) -> bytes:
    """Wheel commands normalized [-1.0, 1.0] -> int16 per-mille."""
    def q(x: float) -> int:
        return max(-1000, min(1000, int(round(x * 1000))))
    return struct.pack('<hhhh', q(fl), q(fr), q(rl), q(rr))


def pack_estop(reason: int) -> bytes:
    return struct.pack('<B', reason)


def unpack_imu_axis(payload: bytes) -> ImuAxisTelemetry:
    """Decode one exact eight-byte IMU axis payload.

    Requiring the exact documented length catches protocol drift instead of
    accepting a prefix that might appear valid while ignoring unexpected data.
    The three axes are signed little-endian int16 values.
    """
    if len(payload) != 8:
        raise ValueError(
            f'IMU axis payload must be 8 bytes, got {len(payload)}'
        )

    sequence, status, x, y, z = struct.unpack('<BBhhh', payload)
    return ImuAxisTelemetry(sequence, status, x, y, z)


def accel_raw_to_mps2(raw: int) -> float:
    """Convert one ±2 g accelerometer reading to metres per second squared."""
    return raw * STANDARD_GRAVITY_MPS2 / ACCEL_LSB_PER_G


def gyro_raw_to_radps(raw: int) -> float:
    """Convert one ±250 degree/second gyroscope reading to radians/second."""
    degrees_per_second = raw / GYRO_LSB_PER_DEGREE_PER_SECOND
    return math.radians(degrees_per_second)


class ImuSampleAssembler:
    """Combine the two IMU wire messages without mixing sample instants.

    The STM32 sends acceleration immediately followed by gyroscope data with a
    shared rolling sequence number. This class retains at most one incomplete
    sample. If a different sequence arrives first, the incomplete sample is
    discarded. That bounded behavior prevents a dropped frame from surviving
    long enough to be incorrectly matched after the uint8 sequence wraps.
    """

    def __init__(self):
        """Start with no retained acceleration or gyroscope half."""
        self._reset()

    def add_accel(self, payload: bytes) -> Optional[ImuRawSample]:
        """Add acceleration and return a sample when its gyro matches."""
        return self._add(unpack_imu_axis(payload), is_accel=True)

    def add_gyro(self, payload: bytes) -> Optional[ImuRawSample]:
        """Add a gyroscope half and return a sample when its accel matches."""
        return self._add(unpack_imu_axis(payload), is_accel=False)

    def _add(
        self,
        part: ImuAxisTelemetry,
        *,
        is_accel: bool,
    ) -> Optional[ImuRawSample]:
        if not part.status & IMU_STATUS_SAMPLE_VALID:
            self._reset()
            return None

        if self._sequence != part.sequence:
            self._reset()
            self._sequence = part.sequence
            self._status = part.status
        else:
            # A completed sample is only as healthy as both of its wire halves.
            self._status &= part.status

        axes = (part.x, part.y, part.z)
        if is_accel:
            self._accel = axes
        else:
            self._gyro = axes

        if self._accel is None or self._gyro is None:
            return None

        sample = ImuRawSample(
            sequence=self._sequence,
            status=self._status,
            accel=self._accel,
            gyro=self._gyro,
        )
        self._reset()
        return sample

    def _reset(self) -> None:
        self._sequence: Optional[int] = None
        self._status = 0
        self._accel: Optional[AxisTuple] = None
        self._gyro: Optional[AxisTuple] = None


class Parser:
    """Incremental byte-stream parser. Feed bytes, yields (msg_id, payload)."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)
        while True:
            # scan for SOF
            i = self._buf.find(SOF)
            if i < 0:
                self._buf.clear()
                return
            if i > 0:
                del self._buf[:i]
            if len(self._buf) < 4:
                return
            msg_id, length = struct.unpack('<HB', bytes(self._buf[1:4]))
            frame_len = 4 + length + 1
            if len(self._buf) < frame_len:
                return
            body = bytes(self._buf[:frame_len - 1])
            expected = self._buf[frame_len - 1]
            if crc8(body) == expected:
                payload = bytes(self._buf[4:4 + length])
                del self._buf[:frame_len]
                yield msg_id, payload
            else:
                # bad CRC — drop SOF byte and retry
                del self._buf[0]
