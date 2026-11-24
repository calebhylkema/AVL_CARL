"""Publish a provisional rear-encoder velocity for the local GUI."""
import time

import rclpy
from rclpy.node import Node

from carl_msgs.msg import CarlEncoderFeedback, CarlVelocity
from carl_bridge.velocity import (VelocityEstimator, MAX_SAMPLE_GAP_S,
                                  COUNTS_PER_REV_LEFT, COUNTS_PER_REV_RIGHT,
                                  ROLLING_DISTANCE_M)


class VelocityNode(Node):
    def __init__(self):
        super().__init__('carl_velocity')
        self.declare_parameter('left_counts_per_rev', COUNTS_PER_REV_LEFT)
        self.declare_parameter('right_counts_per_rev', COUNTS_PER_REV_RIGHT)
        self.declare_parameter('rolling_distance_m', ROLLING_DISTANCE_M)
        self._estimator = VelocityEstimator(
            self.get_parameter('left_counts_per_rev').value,
            self.get_parameter('right_counts_per_rev').value,
            self.get_parameter('rolling_distance_m').value,
        )
        self._last_sample_s = None
        self._valid = False
        self._publisher = self.create_publisher(CarlVelocity, 'carl/velocity', 1)
        self.create_subscription(CarlEncoderFeedback, 'carl/encoders', self._on_encoders, 1)
        self.create_timer(0.05, self._check_stale)

    def _publish(self, velocity=None):
        message = CarlVelocity()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.valid = velocity is not None
        if velocity is not None:
            # Keep calculation unrounded; report three significant digits.
            message.linear_mps = float(f'{velocity.linear_mps:.3g}')
            message.rear_left_mps = float(f'{velocity.rear_left_mps:.3g}')
            message.rear_right_mps = float(f'{velocity.rear_right_mps:.3g}')
        self._publisher.publish(message)

    def _on_encoders(self, message):
        now = time.monotonic()
        self._last_sample_s = now
        velocity = self._estimator.add(now, message.rear_left_counts, message.rear_right_counts)
        if velocity is None:
            if self._valid:
                self._valid = False
                self._publish()
        else:
            self._valid = True
            self._publish(velocity)

    def _check_stale(self):
        if self._valid and self._last_sample_s is not None and time.monotonic() - self._last_sample_s > MAX_SAMPLE_GAP_S:
            self._valid = False
            self._estimator.reset()
            self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
