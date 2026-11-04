"""Exercise diagnostic dispatch with real ROS messages, without a ROS node."""

import json
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

try:
    from carl_bridge.bridge_node import BridgeNode
except ModuleNotFoundError:
    BridgeNode = None


@unittest.skipIf(BridgeNode is None, 'Source the ROS and CARL overlays for dispatch tests')
class ImuDiagnosticDispatchTests(unittest.TestCase):
    """Check the diagnostic topic and transition messages independently of motion."""

    def setUp(self):
        self.published = []
        self.logger = Mock()
        self.bridge = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: None)),
            get_logger=lambda: self.logger,
            _pub_imu_diagnostics=SimpleNamespace(publish=self.published.append),
            _last_imu_health=None,
            _publish_imu=Mock(),
            _publish_imu_temperature=Mock(),
        )

    def dispatch(self, stage, failed, error, count):
        payload = struct.pack('<BBBBBBH', stage, failed, 0x68, 0x70, error, 0x75, count)
        BridgeNode._dispatch(self.bridge, 0x233, payload)

    def test_failed_sensor_publishes_diagnostics_without_motion(self):
        self.dispatch(8, 1, 2, 1)
        self.dispatch(8, 1, 2, 2)
        self.assertEqual(len(self.published), 2)
        self.assertEqual(json.loads(self.published[-1].data)['error_count'], 2)
        self.logger.warning.assert_called_once()
        self.bridge._publish_imu.assert_not_called()
        self.bridge._publish_imu_temperature.assert_not_called()
        self.dispatch(7, 1, 0, 2)
        self.logger.info.assert_called_once()
        self.assertEqual(json.loads(self.published[-1].data)['stage'], 'ready')

    def test_bad_length_diagnostic_is_not_published(self):
        BridgeNode._dispatch(self.bridge, 0x233, bytes(7))
        self.assertEqual(self.published, [])
