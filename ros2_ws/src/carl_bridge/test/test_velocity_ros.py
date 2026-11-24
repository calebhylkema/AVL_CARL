"""Isolated ROS pub/sub test; publishes encoder samples only, never commands."""
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from carl_msgs.msg import CarlEncoderFeedback, CarlVelocity
from carl_bridge.velocity_node import VelocityNode


class VelocityRosTests(unittest.TestCase):
    def test_valid_speed_then_stale_unavailable(self):
        rclpy.init()
        estimator = VelocityNode()
        probe = Node('velocity_test_probe')
        publisher = probe.create_publisher(CarlEncoderFeedback, 'carl/encoders', 1)
        received = []
        probe.create_subscription(CarlVelocity, 'carl/velocity', received.append, 1)
        executor = SingleThreadedExecutor()
        executor.add_node(estimator)
        executor.add_node(probe)
        try:
            ready_deadline = time.monotonic() + 2
            while publisher.get_subscription_count() == 0 and time.monotonic() < ready_deadline:
                executor.spin_once(timeout_sec=0.02)
            self.assertGreater(publisher.get_subscription_count(), 0)
            start = time.monotonic()
            next_publish = start
            count = 0
            while time.monotonic() - start < 0.8:
                if time.monotonic() >= next_publish:
                    message = CarlEncoderFeedback()
                    message.rear_left_counts = count
                    message.rear_right_counts = count
                    publisher.publish(message)
                    count += 100
                    next_publish += 0.02
                executor.spin_once(timeout_sec=0.005)
            valid = [message for message in received if message.valid]
            self.assertTrue(valid, 'No valid velocity published')
            self.assertGreater(max(message.linear_mps for message in valid), 0)
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline and not any(not message.valid for message in received):
                executor.spin_once(timeout_sec=0.05)
            self.assertFalse(received[-1].valid, 'Stale encoder speed stayed valid')
        finally:
            executor.remove_node(probe)
            executor.remove_node(estimator)
            probe.destroy_node()
            estimator.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
