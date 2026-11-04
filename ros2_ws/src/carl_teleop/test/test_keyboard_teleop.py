"""Exercise keyboard handlers with ROS messages and no hardware/ROS graph."""
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from carl_msgs.msg import CarlHeartbeat
from carl_teleop.keyboard_teleop_node import KeyboardTeleopNode


class KeyboardTest(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(KeyboardTeleopNode)
        n = self.node
        n._speed_level, n._mode, n._estop = 1, 'DRIVE', False
        n._reset_pending = False
        n._deadline = dict.fromkeys('wsad', 1000.0)
        n._last_press = dict.fromkeys('wsad', 10.0)
        self.commands, self.estops = [], []
        n._pub_throttle = SimpleNamespace(publish=self.commands.append)
        n._pub_estop = SimpleNamespace(publish=self.estops.append)
        from builtin_interfaces.msg import Time
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
        # Lyrical's logger has warning(), but no warn(). Reproduce that API.
        self.logger = SimpleNamespace(warning=Mock(), info=Mock())
        n.get_logger = lambda: self.logger
        n.get_parameter = lambda name: SimpleNamespace(value={
            'repeat_detect_s': 0.10, 'release_timeout_s': 0.10, 'grace_timeout_s': 0.55
        }[name])
        n._print_state = Mock()
        self.stdout_patch = patch('sys.stdout', new=io.StringIO())
        self.stdout_patch.start()
        self.addCleanup(self.stdout_patch.stop)

    def key(self, key):
        self.node._get_key = lambda timeout: key
        with patch('carl_teleop.keyboard_teleop_node.time.monotonic', return_value=10.0):
            return self.node.poll_key()

    def wheels(self):
        m = self.commands[-1]
        return m.front_left, m.front_right, m.rear_left, m.rear_right

    def test_estop_does_not_crash_and_publishes_zero(self):
        self.assertTrue(self.key('x'))
        self.assertTrue(self.node._estop)
        self.assertEqual(self.estops[-1].reason, 1)
        self.assertEqual(self.wheels(), (0, 0, 0, 0))
        self.logger.warning.assert_called_once()

    def test_reset_clears_held_keys_and_does_not_restore_motion(self):
        self.key('x')
        self.key('w')  # Held/repeated key while e-stop is active.
        self.assertTrue(self.key('r'))
        self.assertTrue(self.node._estop)
        self.assertTrue(self.node._reset_pending)
        self.assertEqual(self.estops[-1].reason, 0)
        self.assertTrue(all(v == 0 for v in self.node._deadline.values()))
        self.assertTrue(all(v == 0 for v in self.node._last_press.values()))
        self.node._publish()
        self.assertEqual(self.wheels(), (0, 0, 0, 0))
        old = CarlHeartbeat()
        old.flags = CarlHeartbeat.FLAG_ESTOP
        self.node._on_heartbeat(old)
        self.assertTrue(self.node._reset_pending)
        self.node._on_heartbeat(CarlHeartbeat())
        self.assertFalse(self.node._reset_pending)
        self.assertFalse(self.node._estop)
        self.node._publish()
        self.assertEqual(self.wheels(), (0, 0, 0, 0))

    def test_remote_estop_heartbeat_stops_all_wheels(self):
        msg = CarlHeartbeat()
        msg.flags = CarlHeartbeat.FLAG_ESTOP
        self.node._on_heartbeat(msg)
        self.assertTrue(self.node._estop)
        self.assertEqual(self.wheels(), (0, 0, 0, 0))
        msg.flags = 0
        self.node._on_heartbeat(msg)
        self.assertTrue(self.node._estop)  # Explicit r required locally.

    def test_front_right_command_matches_rear_right_in_both_directions(self):
        self.node._deadline = dict.fromkeys('wsad', 0.0)
        for key, expected in [('w', 0.25), ('s', -0.25)]:
            self.node._release_keys()
            self.node.get_parameter = lambda name: SimpleNamespace(value=0.55)
            self.key(key)
            with patch('carl_teleop.keyboard_teleop_node.time.monotonic', return_value=10.0):
                self.node._publish()
            self.assertEqual(self.wheels(), (expected,) * 4)

    def test_shutdown_restores_terminal_even_after_context_shutdown(self):
        self.node._context = SimpleNamespace(ok=lambda: False)
        self.node._stdin_settings = ['original settings']
        with patch('termios.tcsetattr') as restore:
            self.node.shutdown()
        restore.assert_called_once()
        self.assertEqual(self.commands, [])


if __name__ == '__main__':
    unittest.main()
