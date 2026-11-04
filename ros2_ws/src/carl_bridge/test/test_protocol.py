"""Unit tests for CARL's platform-independent companion wire protocol.

These tests require neither a running ROS graph nor physical hardware. They
protect the exact STM32/Python byte layout, reject malformed IMU payloads,
verify that sequence numbers prevent mixed samples, and check the unit
conversions used by the ROS ``sensor_msgs/msg/Imu`` publisher.
"""

import math
import struct
import unittest

from carl_bridge import protocol


VALID_STATUS = (
    protocol.IMU_STATUS_INITIALIZED
    | protocol.IMU_STATUS_SAMPLE_VALID
)


def imu_payload(sequence, status, x, y, z):
    """Build a payload using the firmware's little-endian layout."""
    return struct.pack('<BBhhh', sequence, status, x, y, z)


def imu_temperature_payload(sequence, status, raw_temperature):
    """Build temperature payload using the firmware's byte layout."""
    return struct.pack('<BBh', sequence, status, raw_temperature)


class ImuProtocolTests(unittest.TestCase):
    """Check IMU decoding, pairing, framing, and physical-unit conversion."""

    def test_nonfinite_throttle_is_rejected_in_every_wheel_position(self):
        for index in range(4):
            for value in (math.nan, math.inf, -math.inf):
                wheels = [0.4] * 4
                wheels[index] = value
                with self.assertRaises(ValueError):
                    protocol.pack_throttle_cmd(*wheels)

    def test_joystick_center_band_byte_order_and_limits(self):
        self.assertEqual(struct.unpack('<hhhh', protocol.pack_throttle_cmd(*([0.4] * 4))),
                         (400, 400, 400, 400))
        self.assertEqual(struct.unpack('<hhhh', protocol.pack_throttle_cmd(*([-0.4] * 4))),
                         (-400, -400, -400, -400))

    def test_diagnostics_firmware_layout_and_failure_description(self):
        payload = bytes((8, 1, 0x68, 0x70, 2, 0x75, 0x34, 0x12))
        diagnostic = protocol.unpack_imu_diagnostics(payload)
        self.assertEqual(diagnostic, {
            'stage': 'retry_wait', 'failed_stage': 'identify',
            'i2c_address': '0x68', 'who_am_i': '0x70',
            'error': 'address_nack', 'register': '0x75', 'error_count': 0x1234,
        })
        frames = list(protocol.Parser().feed(
            protocol.frame(protocol.MSG_IMU_DIAGNOSTICS, payload)))
        self.assertEqual(frames, [(0x233, payload)])

    def test_diagnostics_rejects_bad_lengths_and_handles_unknown_codes(self):
        for length in (0, 7, 9):
            with self.assertRaises(ValueError):
                protocol.unpack_imu_diagnostics(bytes(length))
        diagnostic = protocol.unpack_imu_diagnostics(bytes([255] * 8))
        self.assertEqual(diagnostic['stage'], 'unknown_255')
        self.assertEqual(diagnostic['error'], 'unknown_255')
        self.assertEqual(diagnostic['error_count'], 65535)

    def test_diagnostics_names_bus_recovery_and_held_low_lines(self):
        for code, name in ((8, 'scl_held_low'), (9, 'sda_held_low')):
            diagnostic = protocol.unpack_imu_diagnostics(
                struct.pack('<BBBBBBH', 8, 10, 0x68, 0x70, code, 0x3b, 2))
            self.assertEqual(diagnostic['failed_stage'], 'recover')
            self.assertEqual(diagnostic['error'], name)
            self.assertEqual(diagnostic['error_count'], 2)

    def test_unpack_imu_axis_signed_little_endian_values(self):
        payload = bytes((0xA5, 0x03, 0x34, 0x12, 0xFE, 0xFF, 0x00, 0x80))

        part = protocol.unpack_imu_axis(payload)

        self.assertEqual(part.sequence, 0xA5)
        self.assertEqual(part.status, VALID_STATUS)
        self.assertEqual((part.x, part.y, part.z), (0x1234, -2, -32768))

    def test_unpack_imu_axis_requires_exact_length(self):
        for length in (0, 7, 9):
            with self.subTest(length=length):
                with self.assertRaisesRegex(ValueError, 'must be 8 bytes'):
                    protocol.unpack_imu_axis(bytes(length))

    def test_unpack_imu_temperature_signed_little_endian_value(self):
        sample = protocol.unpack_imu_temperature(
            imu_temperature_payload(0x5A, VALID_STATUS, -1234)
        )

        self.assertEqual(
            sample,
            protocol.ImuTemperatureTelemetry(
                sequence=0x5A,
                status=VALID_STATUS,
                raw_temperature=-1234,
            ),
        )

    def test_unpack_imu_temperature_requires_exact_length(self):
        for length in (0, 3, 5):
            with self.subTest(length=length):
                with self.assertRaisesRegex(ValueError, 'must be 4 bytes'):
                    protocol.unpack_imu_temperature(bytes(length))

    def test_assembler_pairs_accel_then_gyro(self):
        assembler = protocol.ImuSampleAssembler()

        self.assertIsNone(
            assembler.add_accel(imu_payload(7, VALID_STATUS, 1, 2, 3))
        )
        sample = assembler.add_gyro(
            imu_payload(7, VALID_STATUS, 4, 5, 6)
        )

        self.assertEqual(
            sample,
            protocol.ImuRawSample(
                sequence=7,
                status=VALID_STATUS,
                accel=(1, 2, 3),
                gyro=(4, 5, 6),
            ),
        )

    def test_assembler_pairs_gyro_then_accel(self):
        assembler = protocol.ImuSampleAssembler()

        self.assertIsNone(
            assembler.add_gyro(
                imu_payload(19, VALID_STATUS, -1, -2, -3)
            )
        )
        sample = assembler.add_accel(
            imu_payload(19, VALID_STATUS, 10, 20, 30)
        )

        self.assertIsNotNone(sample)
        self.assertEqual(sample.sequence, 19)
        self.assertEqual(sample.accel, (10, 20, 30))
        self.assertEqual(sample.gyro, (-1, -2, -3))

    def test_assembler_never_combines_different_sequences(self):
        assembler = protocol.ImuSampleAssembler()

        self.assertIsNone(
            assembler.add_accel(imu_payload(1, VALID_STATUS, 1, 2, 3))
        )
        self.assertIsNone(
            assembler.add_gyro(imu_payload(2, VALID_STATUS, 4, 5, 6))
        )

        # Sequence 1 was discarded when the first sequence-2 half arrived.
        sample = assembler.add_accel(
            imu_payload(2, VALID_STATUS, 7, 8, 9)
        )
        self.assertIsNotNone(sample)
        self.assertEqual(sample.sequence, 2)
        self.assertEqual(sample.accel, (7, 8, 9))
        self.assertEqual(sample.gyro, (4, 5, 6))

    def test_assembler_handles_uint8_sequence_rollover(self):
        assembler = protocol.ImuSampleAssembler()

        assembler.add_accel(imu_payload(255, VALID_STATUS, 1, 1, 1))
        last = assembler.add_gyro(
            imu_payload(255, VALID_STATUS, 2, 2, 2)
        )
        assembler.add_accel(imu_payload(0, VALID_STATUS, 3, 3, 3))
        first = assembler.add_gyro(
            imu_payload(0, VALID_STATUS, 4, 4, 4)
        )

        self.assertIsNotNone(last)
        self.assertEqual(last.sequence, 255)
        self.assertIsNotNone(first)
        self.assertEqual(first.sequence, 0)

    def test_invalid_status_clears_incomplete_sample(self):
        assembler = protocol.ImuSampleAssembler()
        invalid_status = protocol.IMU_STATUS_INITIALIZED

        assembler.add_accel(imu_payload(3, VALID_STATUS, 1, 2, 3))
        self.assertIsNone(
            assembler.add_gyro(imu_payload(3, invalid_status, 4, 5, 6))
        )
        self.assertIsNone(
            assembler.add_gyro(imu_payload(3, VALID_STATUS, 7, 8, 9))
        )

    def test_acceleration_conversion_uses_two_g_scale(self):
        self.assertEqual(protocol.accel_raw_to_mps2(0), 0.0)
        self.assertAlmostEqual(protocol.accel_raw_to_mps2(16384), 9.80665)
        self.assertAlmostEqual(protocol.accel_raw_to_mps2(-16384), -9.80665)

    def test_gyroscope_conversion_returns_radians_per_second(self):
        self.assertEqual(protocol.gyro_raw_to_radps(0), 0.0)
        self.assertAlmostEqual(
            protocol.gyro_raw_to_radps(131),
            math.pi / 180.0,
        )
        self.assertAlmostEqual(
            protocol.gyro_raw_to_radps(-131),
            -math.pi / 180.0,
        )

    def test_temperature_conversion_uses_mpu6500_scale(self):
        self.assertEqual(protocol.temperature_raw_to_celsius(0), 21.0)
        self.assertAlmostEqual(
            protocol.temperature_raw_to_celsius(3339),
            31.000898553,
            places=8,
        )

    def test_parser_preserves_three_back_to_back_imu_frames(self):
        accel = protocol.frame(
            protocol.MSG_IMU_ACCEL,
            imu_payload(11, VALID_STATUS, 100, 200, 300),
        )
        gyro = protocol.frame(
            protocol.MSG_IMU_GYRO,
            imu_payload(11, VALID_STATUS, -100, -200, -300),
        )
        temperature = protocol.frame(
            protocol.MSG_IMU_TEMPERATURE,
            imu_temperature_payload(11, VALID_STATUS, 3339),
        )

        parsed = list(protocol.Parser().feed(accel + gyro + temperature))

        self.assertEqual(
            parsed,
            [
                (protocol.MSG_IMU_ACCEL, accel[4:-1]),
                (protocol.MSG_IMU_GYRO, gyro[4:-1]),
                (protocol.MSG_IMU_TEMPERATURE, temperature[4:-1]),
            ],
        )


if __name__ == '__main__':
    unittest.main()
