import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from carl_bridge import protocol as p


def parts(sequence=1, status=0x33, w=16384):
    return [
        (p.MSG_BNO_ACCEL, struct.pack('<BBhhh', sequence, 3, 256, -512, 2511)),
        (p.MSG_BNO_GYRO, struct.pack('<BBhhh', sequence, 3, 512, -1024, 0)),
        (p.MSG_BNO_QUAT_XYZ, struct.pack('<BBhhh', sequence, status, 0, 0, 0)),
        (p.MSG_BNO_QUAT_W, struct.pack('<BBhH', sequence, status, w, 2048)),
    ]


class BnoProtocolTests(unittest.TestCase):
    def test_correct_si_units_and_orientation(self):
        a = p.BnoSampleAssembler()
        sample = None
        for msg_id, payload in reversed(parts()):
            sample = a.add(msg_id, payload, now=0)
        self.assertEqual(sample.accel, (1.0, -2.0, 2511 / 256.0))
        self.assertEqual(sample.gyro, (1.0, -2.0, 0.0))
        self.assertEqual(sample.quaternion, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(sample.orientation_accuracy, 3)
        self.assertEqual(sample.heading_accuracy_rad, 0.5)

    def test_all_four_parts_required_and_mixed_sequences_rejected(self):
        a = p.BnoSampleAssembler()
        for msg_id, payload in parts()[:-1]:
            self.assertIsNone(a.add(msg_id, payload, now=0))
        self.assertIsNone(a.add(*parts(sequence=2)[-1], now=0))

    def test_incomplete_samples_expire_before_sequence_wrap(self):
        a = p.BnoSampleAssembler()
        for msg_id, payload in parts()[:-1]:
            a.add(msg_id, payload, now=0)
        self.assertIsNone(a.add(*parts()[-1], now=0.2))

    def test_duplicate_snapshot_not_republished(self):
        a = p.BnoSampleAssembler()
        for msg_id, payload in parts():
            a.add(msg_id, payload, now=0)
        for msg_id, payload in parts():
            self.assertIsNone(a.add(msg_id, payload, now=0.01))

    def test_invalid_orientation_and_missing_validity(self):
        for payloads in (parts(w=0), parts(status=0)):
            a = p.BnoSampleAssembler()
            for msg_id, payload in payloads:
                self.assertIsNone(a.add(msg_id, payload, now=0))

    def test_bad_lengths_and_unknown_ids(self):
        a = p.BnoSampleAssembler()
        for msg_id, payload in parts():
            for bad in (payload[:-1], payload + b'\x00'):
                with self.assertRaises(ValueError):
                    a.add(msg_id, bad)
        with self.assertRaises(ValueError):
            a.add(0, bytes(8))

    def test_diagnostics_has_no_mpu_address_or_identity(self):
        d = p.unpack_bno_diagnostics(bytes((1, 7, 6, 6, 2, 0, 255, 0)))
        self.assertEqual(d['sensor'], 'BNO08x')
        self.assertEqual(d['error'], 'report_timeout')
        self.assertEqual(d['error_count'], 255)
        self.assertFalse(d['temperature_supported'])
        self.assertNotIn('i2c_address', d)
        for bad in (bytes(7), bytes(8)):
            with self.assertRaises(ValueError):
                p.unpack_bno_diagnostics(bad)


try:
    from carl_bridge.bridge_node import BridgeNode
except ModuleNotFoundError:
    BridgeNode = None


@unittest.skipIf(BridgeNode is None, 'Source ROS and CARL overlays')
class BnoDispatchTests(unittest.TestCase):
    def test_dispatch_and_sensor_only_recovery(self):
        bridge = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
            _bno_assembler=p.BnoSampleAssembler(), _publish_bno=Mock(),
            _pub_imu_diagnostics=SimpleNamespace(publish=Mock()),
            _last_imu_health=None, get_logger=lambda: Mock(),
        )
        for msg_id, payload in parts():
            BridgeNode._dispatch(bridge, msg_id, payload)
        bridge._publish_bno.assert_called_once()
        BridgeNode._dispatch(bridge, p.MSG_BNO_DIAGNOSTICS,
                             bytes((1, 7, 6, 6, 2, 0, 1, 0)))
        self.assertIsNone(bridge._bno_assembler._sequence)

    def test_published_ros_fields_are_si_units_and_quaternion(self):
        published = []
        bridge = SimpleNamespace(_pub_imu=SimpleNamespace(publish=published.append))
        sample = p.BnoSample(1, (1., 2., 3.), (4., 5., 6.), (0., 0., 0., 1.), 3, 0.1)
        BridgeNode._publish_bno(bridge, sample, None)
        self.assertEqual(published[0].linear_acceleration.x, 1.)
        self.assertEqual(published[0].angular_velocity.z, 6.)
        self.assertEqual(published[0].orientation.w, 1.)
        self.assertEqual(published[0].header.frame_id, 'imu_link')
