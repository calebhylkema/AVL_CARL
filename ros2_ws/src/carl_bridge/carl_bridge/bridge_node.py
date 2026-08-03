"""carl_bridge — ROS 2 node bridging the serial link to ROS 2 topics.

PLACEHOLDER scaffold. Intended responsibilities:
  * Subscribe to command topics (e.g. /cmd_vel) -> encode -> write to the master.
  * Read framed telemetry from the master -> publish (encoders, status, IMU).
  * Emit a teleop heartbeat so the master's watchdog does not zero the throttle.

Fill in using protocol.py (which mirrors firmware/lib/carl_protocol/wire.h).
"""

import rclpy
from rclpy.node import Node


class CarlBridge(Node):
    def __init__(self):
        super().__init__('carl_bridge')
        # TODO: open serial (protocol.BAUD), declare params (port), set up
        #       publishers/subscribers, and a timer for the teleop heartbeat.
        self.get_logger().info('carl_bridge placeholder node started (no I/O yet)')


def main(args=None):
    rclpy.init(args=args)
    node = CarlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
