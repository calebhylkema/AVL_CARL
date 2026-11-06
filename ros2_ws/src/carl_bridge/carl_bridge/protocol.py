"""Framing and packing for the master <-> companion computer link.

Frame format (mirrors CAN semantics so firmware translation is trivial):

    [SOF=0xAA][MSG_ID: u16 LE][LEN: u8][PAYLOAD ...][CRC8]

MSG_IDs reuse the CAN IDs from docs/can-bus.md so the master just proxies
between wire format and CAN frames.
"""
from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

SOF = 0xAA

MSG_ESTOP = 0x000
MSG_HEARTBEAT = 0x010
MSG_THROTTLE_CMD = 0x100
MSG_THROTTLE_STATUS = 0x200
MSG_ENCODER_FEEDBACK = 0x210
MSG_STEERING_FEEDBACK = 0x221
MSG_IMU_ACCEL = 0x230
MSG_IMU_GYRO = 0x231
MSG_IMU_TEMPERATURE = 0x232
MSG_IMU_DIAGNOSTICS = 0x233
MSG_BNO_ACCEL = 0x234
MSG_BNO_GYRO = 0x235
MSG_BNO_QUAT_XYZ = 0x236
MSG_BNO_QUAT_W = 0x237
MSG_BNO_DIAGNOSTICS = 0x238
MSG_BNO_SPI_DIAGNOSTICS = 0x23A
MSG_MPU_SPI_DIAGNOSTICS = 0x23B


def unpack_mpu_spi_diagnostics(payload: bytes) -> dict:
    if len(payload) != 8 or payload[0] != 1:
        raise ValueError('Expected eight-byte MPU SPI diagnostic schema 1')
    _, stage, identity, error, register, value, count = struct.unpack('<BBBBBBH', payload)
    stages = ('power_wait', 'identify', 'reset_wait', 'wake_wait', 'configure', 'ready', 'retry_wait')
    errors = ('ok', 'wrong_identity', 'verify_failed', 'data_timeout', 'identity_lost')
    return {
        'schema': 1, 'sensor': {0x70: 'MPU-6500', 0x71: 'MPU-9250'}.get(identity, 'unidentified MPU'),
        'interface': 'spi', 'who_am_i': f'0x{identity:02x}',
        'stage': stages[stage] if stage < len(stages) else f'unknown_{stage}',
        'error': errors[error] if error < len(errors) else f'unknown_{error}',
        'ready': stage == 5 and error == 0, 'register': f'0x{register:02x}',
        'value': f'0x{value:02x}', 'error_count': count,
        'orientation_supported': False,
    }

STEERING_SAMPLE_VALID = 0x01
STEERING_ADC_RAIL_LOW = 0x02
STEERING_ADC_RAIL_HIGH = 0x04
STEERING_OUT_OF_RANGE = 0x08
STEERING_LINK_STALE = 0x80


@dataclass(frozen=True)
class SteeringFeedback:
    status: int
    raw_adc: int
    sequence: int


def unpack_steering_feedback(payload: bytes) -> SteeringFeedback:
    if len(payload) != 6 or payload[0] != 1:
        raise ValueError('Expected six-byte steering feedback schema 1')
    _, status, raw, sequence = struct.unpack('<BBHH', payload)
    expected = (STEERING_ADC_RAIL_LOW if raw == 0 else
                STEERING_ADC_RAIL_HIGH if raw == 4095 else STEERING_SAMPLE_VALID)
    if raw > 4095 or status != expected:
        raise ValueError('Invalid steering ADC count or flags')
    return SteeringFeedback(status, raw, sequence)


BNO_STAGE_NAMES = (
    'power_wait', 'reset_low', 'boot_wait', 'identify', 'configure',
    'stream_wait', 'ready', 'retry_wait',
)
BNO_ERROR_NAMES = (
    'ok', 'boot_timeout', 'identify_timeout', 'configure_timeout',
    'invalid_packet', 'unsupported_fragment', 'report_timeout', 'sensor_reset',
)


def unpack_bno_spi_diagnostics(payload: bytes) -> dict:
    if len(payload) != 8 or payload[0] != 1:
        raise ValueError('Expected eight-byte BNO SPI diagnostic schema 1')
    result, header, lines = payload[1], payload[2:6], payload[6]
    names = ('not_attempted', 'empty', 'packet', 'invalid_length')
    observed = result != 0
    length = int.from_bytes(header[:2], 'little') & 0x7fff
    return {
        'schema': 1,
        'result': names[result] if result < len(names) else f'unknown_{result}',
        'header_hex': header.hex(' ') if observed else None,
        'declared_length': length if observed else None,
        'continuation': bool(header[1] & 0x80) if observed else None,
        'channel': header[2] if observed else None,
        'sequence': header[3] if observed else None,
        'int_asserted': bool(lines & 1) if observed else None,
        'reset_high': bool(lines & 2) if observed else None,
        'wake_high': bool(lines & 4) if observed else None,
        'cs_high_before_read': bool(lines & 8) if observed else None,
        'snapshot': 'last_receive_attempt',
    }


def unpack_bno_diagnostics(payload: bytes) -> dict:
    if len(payload) != 8 or payload[0] != 1:
        raise ValueError('Expected eight-byte BNO diagnostic schema 1')
    _, stage, failed, error, detail, ready, count = struct.unpack('<BBBBBBH', payload)

    def name(names, value):
        return names[value] if value < len(names) else f'unknown_{value}'

    return {
        'sensor': 'BNO08x', 'interface': 'spi', 'schema': 1,
        'stage': name(BNO_STAGE_NAMES, stage),
        'failed_stage': name(BNO_STAGE_NAMES, failed),
        'error': name(BNO_ERROR_NAMES, error),
        'detail': f'0x{detail:02x}', 'ready': bool(ready), 'error_count': count,
        'temperature_supported': False,
    }


@dataclass(frozen=True)
class BnoSample:
    sequence: int
    accel: tuple
    gyro: tuple
    quaternion: tuple  # normalized x,y,z,w
    orientation_accuracy: int
    heading_accuracy_rad: float


class BnoSampleAssembler:
    """Bounded assembly of a fresh BNO snapshot; never use MPU ADC scaling.

    Sensor reports are independently scheduled, combined by the STM32 only
    when each is new and at most 50 ms old. A wire snapshot expires after
    100 ms and cannot be paired across a dropped packet/sequence wrap.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self._sequence = None
        self._parts = {}
        self._started = 0.0
        self._completed = None

    def add(self, msg_id: int, payload: bytes, now=None):
        now = time.monotonic() if now is None else now
        expected = 6 if msg_id == MSG_BNO_QUAT_W else 8
        if msg_id not in (MSG_BNO_ACCEL, MSG_BNO_GYRO, MSG_BNO_QUAT_XYZ, MSG_BNO_QUAT_W):
            raise ValueError('Unknown BNO snapshot part')
        if len(payload) != expected:
            raise ValueError('Incorrect BNO snapshot payload length')
        sequence, status = payload[:2]
        if (status & 3) != 3:
            self.reset()
            return None
        if self._completed == sequence and now - self._started < 0.1:
            return None
        if self._sequence != sequence or now - self._started >= 0.1:
            self._sequence = sequence
            self._parts = {}
            self._started = now
        self._parts[msg_id] = payload
        if len(self._parts) != 4:
            return None
        parts = self._parts
        self._parts = {}
        self._completed = sequence
        accel = struct.unpack('<hhh', parts[MSG_BNO_ACCEL][2:])
        gyro = struct.unpack('<hhh', parts[MSG_BNO_GYRO][2:])
        xyz = struct.unpack('<hhh', parts[MSG_BNO_QUAT_XYZ][2:])
        w, heading = struct.unpack('<hH', parts[MSG_BNO_QUAT_W][2:])
        q = tuple(value / 16384.0 for value in (*xyz, w))
        norm = math.sqrt(sum(value * value for value in q))
        if not 0.9 <= norm <= 1.1:
            return None  # corrupted/zero orientation must not become valid ROS data
        return BnoSample(
            sequence, tuple(value / 256.0 for value in accel),
            tuple(value / 512.0 for value in gyro),
            tuple(value / norm for value in q),
            (parts[MSG_BNO_QUAT_XYZ][1] >> 4) & 3, heading / 4096.0,
        )


IMU_STAGE_NAMES = (
    'power_wait', 'identify', 'reset', 'reset_wait', 'wake', 'wake_wait',
    'configure', 'ready', 'retry_wait', 'sample', 'recover',
)
IMU_ERROR_NAMES = (
    'ok', 'buffer_error', 'address_nack', 'data_nack', 'bus_error',
    'timeout', 'short_read', 'wrong_identity', 'scl_held_low', 'sda_held_low',
)


def unpack_imu_diagnostics(payload: bytes) -> dict:
    """Decode the STM32 health frame even when the sensor cannot initialize."""
    if len(payload) != 8:
        raise ValueError(f'IMU diagnostics payload must be 8 bytes, got {len(payload)}')
    stage, failed, address, identity, error, register, count = struct.unpack(
        '<BBBBBBH', payload
    )
    return {
        'stage': IMU_STAGE_NAMES[stage] if stage < len(IMU_STAGE_NAMES) else f'unknown_{stage}',
        'failed_stage': (
            IMU_STAGE_NAMES[failed]
            if failed < len(IMU_STAGE_NAMES) else f'unknown_{failed}'
        ),
        'i2c_address': f'0x{address:02x}',
        'who_am_i': f'0x{identity:02x}',
        'error': IMU_ERROR_NAMES[error] if error < len(IMU_ERROR_NAMES) else f'unknown_{error}',
        'register': f'0x{register:02x}',
        'error_count': count,
    }

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
TEMPERATURE_LSB_PER_DEGREE_C = 333.87
TEMPERATURE_OFFSET_DEGREE_C = 21.0

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


@dataclass(frozen=True)
class ImuTemperatureTelemetry:
    """One raw MPU-6500 die-temperature sample from the STM32."""

    sequence: int
    status: int
    raw_temperature: int


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
    if not all(math.isfinite(x) for x in (fl, fr, rl, rr)):
        raise ValueError('Throttle commands must be finite')
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


def unpack_imu_temperature(payload: bytes) -> ImuTemperatureTelemetry:
    """Decode one exact four-byte MPU-6500 temperature payload."""
    if len(payload) != 4:
        raise ValueError(
            f'IMU temperature payload must be 4 bytes, got {len(payload)}'
        )

    sequence, status, raw_temperature = struct.unpack('<BBh', payload)
    return ImuTemperatureTelemetry(sequence, status, raw_temperature)


def accel_raw_to_mps2(raw: int) -> float:
    """Convert one ±2 g accelerometer reading to metres per second squared."""
    return raw * STANDARD_GRAVITY_MPS2 / ACCEL_LSB_PER_G


def gyro_raw_to_radps(raw: int) -> float:
    """Convert one ±250 degree/second gyroscope reading to radians/second."""
    degrees_per_second = raw / GYRO_LSB_PER_DEGREE_PER_SECOND
    return math.radians(degrees_per_second)


def temperature_raw_to_celsius(raw: int) -> float:
    """Convert MPU-6500 TEMP_OUT to degrees Celsius."""
    return (
        raw / TEMPERATURE_LSB_PER_DEGREE_C
        + TEMPERATURE_OFFSET_DEGREE_C
    )


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
        self.crc_errors = 0
        self.discarded_bytes = 0
        self.frames_parsed = 0
        self.last_crc_error = None

    def feed(self, data: bytes):
        self._buf.extend(data)
        while True:
            # scan for SOF
            i = self._buf.find(SOF)
            if i < 0:
                self.discarded_bytes += len(self._buf)
                self._buf.clear()
                return
            if i > 0:
                self.discarded_bytes += i
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
                self.frames_parsed += 1
                payload = bytes(self._buf[4:4 + length])
                del self._buf[:frame_len]
                yield msg_id, payload
            else:
                # bad CRC — drop SOF byte and retry
                self.crc_errors += 1
                self.discarded_bytes += 1
                self.last_crc_error = dict(message_id=msg_id, length=length,
                                           expected_crc=expected, computed_crc=crc8(body),
                                           frame_prefix_hex=body[:32].hex())
                del self._buf[0]
