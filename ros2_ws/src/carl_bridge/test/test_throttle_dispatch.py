"""Command serialization and invalid-input stop using the actual ROS callback."""
import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from carl_bridge import protocol
try:
    from carl_bridge.bridge_node import BridgeNode
except ModuleNotFoundError:
    BridgeNode = None


@unittest.skipIf(BridgeNode is None, 'Source the ROS and CARL overlays for dispatch tests')
class ThrottleDispatchTests(unittest.TestCase):
    def test_nonfinite_inputs_send_a_real_zero_frame(self):
        for index in range(4):
            for value in (math.nan, math.inf, -math.inf):
                sent = []
                logger = Mock()
                bridge = SimpleNamespace(_transport=SimpleNamespace(send=sent.append),
                                         get_logger=lambda: logger)
                fields = dict(front_left=0.4, front_right=0.4, rear_left=0.4, rear_right=0.4)
                fields[list(fields)[index]] = value
                BridgeNode._on_throttle_cmd(bridge, SimpleNamespace(**fields))
                frames = list(protocol.Parser().feed(sent[-1]))
                self.assertEqual(frames, [(protocol.MSG_THROTTLE_CMD, bytes(8))])
                logger.warning.assert_called_once()

    def test_valid_commands_keep_wheel_order(self):
        sent = []
        bridge = SimpleNamespace(_transport=SimpleNamespace(send=sent.append))
        message = SimpleNamespace(front_left=0.4, front_right=-0.3, rear_left=0.2, rear_right=-0.1)
        BridgeNode._on_throttle_cmd(bridge, message)
        frames = list(protocol.Parser().feed(sent[-1]))
        self.assertEqual(struct.unpack('<hhhh', frames[0][1]), (400, -300, 200, -100))
