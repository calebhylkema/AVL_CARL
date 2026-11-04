"""Steering wire decoding, calibration, dispatch and loss/recovery."""
import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from carl_bridge import protocol as p
from carl_bridge.steering import SteeringConfig


def payload(raw=2048, sequence=65535, status=1):
    return struct.pack('<BBHH', 1, status, raw, sequence)


class SteeringProtocolTests(unittest.TestCase):
    def test_cpp_golden_payload_survives_fragmented_wire(self):
        golden = bytes((1, 1, 0, 8, 255, 255))
        frame = p.frame(p.MSG_STEERING_FEEDBACK, golden)
        parser = p.Parser()
        self.assertEqual(list(parser.feed(frame[:5])), [])
        self.assertEqual(list(parser.feed(frame[5:])), [(0x221, golden)])
        sample = p.unpack_steering_feedback(golden)
        self.assertEqual((sample.raw_adc, sample.sequence, sample.status), (2048, 65535, 1))

    def test_rejects_bad_length_schema_adc_and_flags(self):
        bad = [payload()[:-1], payload() + b'\0', bytes((2,)) + payload()[1:],
               payload(raw=4096), payload(status=0), payload(status=3),
               payload(raw=0), payload(raw=4095), payload(status=128)]
        for data in bad:
            with self.subTest(data=data), self.assertRaises(ValueError):
                p.unpack_steering_feedback(data)

    def test_voltage_and_absolute_angle_without_fabricated_calibration(self):
        result = SteeringConfig().convert(p.unpack_steering_feedback(payload()))
        self.assertAlmostEqual(result['sensor_voltage'], 2048 / 4095 * 6.6)
        self.assertAlmostEqual(result['absolute_angle_rad'], 2048 / 4095 * 6.6 / 5 * math.tau)
        self.assertTrue(result['valid'])
        self.assertFalse(result['calibrated'])
        self.assertTrue(math.isnan(result['angle_rad']))

    def test_center_sign_ratio_and_wrap(self):
        sample = p.unpack_steering_feedback(payload())
        absolute = SteeringConfig().convert(sample)['absolute_angle_rad']
        config = SteeringConfig(zero_offset_rad=absolute - 0.2, direction=-1.0,
                                ratio=2.0, calibrated=True)
        self.assertAlmostEqual(config.convert(sample)['angle_rad'], -0.1)
        config = SteeringConfig(zero_offset_rad=absolute + math.tau - 0.1,
                                calibrated=True)
        self.assertAlmostEqual(config.convert(sample)['angle_rad'], 0.1)

    def test_rails_and_overrange_have_no_angle(self):
        for raw, flag in ((0, 2), (4095, 4), (4000, 1)):
            result = SteeringConfig(calibrated=True).convert(
                p.unpack_steering_feedback(payload(raw=raw, status=flag)))
            self.assertFalse(result['valid'])
            self.assertTrue(math.isnan(result['absolute_angle_rad']))
            self.assertTrue(math.isnan(result['angle_rad']))
        self.assertEqual(result['status'], p.STEERING_OUT_OF_RANGE)

    def test_bad_calibration_is_rejected_at_startup(self):
        for name in ('adc_reference_voltage', 'divider_gain',
                     'sensor_full_scale_voltage', 'ratio', 'timeout_sec'):
            for value in (0.0, -1.0, math.nan, math.inf):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    SteeringConfig(**{name: value})
        for value in (0.0, 2.0, math.nan):
            with self.assertRaises(ValueError):
                SteeringConfig(direction=value)
        with self.assertRaises(ValueError):
            SteeringConfig(zero_offset_rad=math.inf)


try:
    from carl_bridge.bridge_node import BridgeNode
except ModuleNotFoundError:
    BridgeNode = None


@unittest.skipIf(BridgeNode is None, 'Build and source the ROS/CARL overlays')
class SteeringDispatchTests(unittest.TestCase):
    def setUp(self):
        self.messages = []
        self.bridge = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
            _steering_config=SteeringConfig(), _last_steering_rx=None,
            _last_steering_sequence=None, _steering_unavailable_published=False,
            _pub_steering=SimpleNamespace(publish=self.messages.append),
        )
        self.bridge._dispatch = lambda mid, data: BridgeNode._dispatch(self.bridge, mid, data)
        self.bridge._check_steering_timeout = lambda: BridgeNode._check_steering_timeout(self.bridge)

    def dispatch(self, data):
        self.bridge._dispatch(p.MSG_STEERING_FEEDBACK, data)

    def test_actual_ros_fields_and_sequence_rollover(self):
        self.dispatch(payload())
        self.dispatch(payload(raw=2000, sequence=0))
        self.assertEqual(len(self.messages), 2)
        msg = self.messages[-1]
        self.assertEqual(msg.header.frame_id, 'steering_encoder_link')
        self.assertEqual(msg.raw_adc, 2000)
        self.assertTrue(msg.valid)
        self.assertTrue(math.isnan(msg.angle_rad))
        self.assertAlmostEqual(msg.sensor_voltage, 2000 / 4095 * 6.6, places=5)

    def test_rails_and_malformed_data_do_not_become_valid(self):
        self.dispatch(payload()[:-1])
        self.assertEqual(self.messages, [])
        self.assertIsNone(self.bridge._last_steering_rx)
        self.dispatch(payload(raw=0, status=2))
        self.assertFalse(self.messages[-1].valid)
        self.assertEqual(self.messages[-1].status, 2)

    def test_timeout_duplicate_rejection_and_new_sequence_recovery(self):
        with patch('carl_bridge.bridge_node.time.monotonic', return_value=10.0):
            self.dispatch(payload())
        with patch('carl_bridge.bridge_node.time.monotonic', return_value=10.249):
            self.bridge._check_steering_timeout()
        self.assertEqual(len(self.messages), 1)
        with patch('carl_bridge.bridge_node.time.monotonic', return_value=10.25):
            self.dispatch(payload())  # A duplicate must not refresh receipt time.
            self.bridge._check_steering_timeout()
        self.assertEqual(len(self.messages), 2)
        self.assertEqual(self.messages[-1].status, p.STEERING_LINK_STALE)
        self.assertFalse(self.messages[-1].valid)
        self.assertTrue(math.isnan(self.messages[-1].sensor_voltage))
        self.bridge._check_steering_timeout()
        self.assertEqual(len(self.messages), 2)
        self.dispatch(payload())
        self.assertEqual(len(self.messages), 2)  # Replays stay stale indefinitely.
        self.dispatch(payload(sequence=0))
        self.assertTrue(self.messages[-1].valid)

    def test_idle_transport_still_reports_initial_missing_board(self):
        self.bridge._transport = SimpleNamespace(poll=lambda: b'')
        self.bridge._parser = p.Parser()
        BridgeNode._poll(self.bridge)
        self.assertEqual(self.messages[-1].status, p.STEERING_LINK_STALE)
