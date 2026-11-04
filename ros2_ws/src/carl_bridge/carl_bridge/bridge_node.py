"""Bridge node: ROS2 topics <-> NUCLEO master over serial or UDP."""
from __future__ import annotations

import json
import math
import time
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import String

from carl_msgs.msg import (
    CarlEncoderFeedback,
    CarlEstop,
    CarlHeartbeat,
    CarlSteeringAngle,
    CarlThrottleCmd,
    CarlThrottleStatus,
)

from carl_bridge import protocol
from carl_bridge.steering import SteeringConfig
from carl_bridge.transports import make_transport


class BridgeNode(Node):
    def __init__(self):
        super().__init__('carl_bridge')

        self.declare_parameter('transport', 'serial')
        self.declare_parameter('serial.port', '/dev/ttyACM0')
        self.declare_parameter('serial.baud', 921600)
        self.declare_parameter('udp.remote_host', '192.168.1.10')
        self.declare_parameter('udp.remote_port', 5000)
        self.declare_parameter('udp.local_port', 5001)
        self.declare_parameter('poll_rate_hz', 200.0)
        steering_defaults = SteeringConfig()
        for name in steering_defaults.__dataclass_fields__:
            self.declare_parameter(f'steering.{name}', getattr(steering_defaults, name))
        self._steering_config = SteeringConfig(**{
            name: self.get_parameter(f'steering.{name}').value
            for name in steering_defaults.__dataclass_fields__
        })
        self._last_steering_rx = None
        self._last_steering_sequence = None
        self._steering_unavailable_published = False

        kind = self.get_parameter('transport').value
        if kind == 'serial':
            self._transport = make_transport(
                'serial',
                port=self.get_parameter('serial.port').value,
                baud=self.get_parameter('serial.baud').value,
            )
        else:
            self._transport = make_transport(
                'udp',
                remote_host=self.get_parameter('udp.remote_host').value,
                remote_port=self.get_parameter('udp.remote_port').value,
                local_port=self.get_parameter('udp.local_port').value,
            )

        self._parser = protocol.Parser()
        self._imu_assembler = protocol.ImuSampleAssembler()
        self._bno_assembler = protocol.BnoSampleAssembler()

        self.create_subscription(
            CarlThrottleCmd,
            'carl/throttle_cmd',
            self._on_throttle_cmd,
            1,
        )
        self.create_subscription(
            CarlEstop,
            'carl/estop',
            self._on_estop,
            10,
        )

        self._pub_throttle_status = self.create_publisher(
            CarlThrottleStatus,
            'carl/throttle_status',
            10,
        )
        self._pub_encoders = self.create_publisher(
            CarlEncoderFeedback,
            'carl/encoders',
            10,
        )
        self._pub_heartbeat = self.create_publisher(
            CarlHeartbeat,
            'carl/heartbeat',
            10,
        )
        self._pub_steering = self.create_publisher(
            CarlSteeringAngle, 'carl/steering_angle', 10,
        )
        self._pub_imu = self.create_publisher(Imu, 'carl/imu', 10)
        self._pub_imu_temperature = self.create_publisher(
            Temperature,
            'carl/imu/temperature',
            10,
        )
        self._pub_imu_diagnostics = self.create_publisher(
            String, 'carl/imu/diagnostics', 10,
        )
        self._last_imu_health = None
        self._pub_imu_spi_diagnostics = self.create_publisher(
            String, 'carl/imu/spi_diagnostics', 10,
        )
        self._last_bno_spi_fault = None

        poll_hz = self.get_parameter('poll_rate_hz').value
        self.create_timer(1.0 / poll_hz, self._poll)

        self.get_logger().info(f'carl_bridge up on transport={kind}')

    def _on_throttle_cmd(self, msg: CarlThrottleCmd):
        try:
            payload = protocol.pack_throttle_cmd(
                msg.front_left, msg.front_right, msg.rear_left, msg.rear_right,
            )
        except (ValueError, OverflowError):
            # Stop all wheels immediately if another ROS publisher supplies
            # NaN/Inf; do not leave its previous command running until timeout.
            payload = protocol.pack_throttle_cmd(0.0, 0.0, 0.0, 0.0)
            self.get_logger().warning('Non-finite throttle command rejected; sending zero.')
        self._transport.send(
            protocol.frame(protocol.MSG_THROTTLE_CMD, payload)
        )

    def _on_estop(self, msg: CarlEstop):
        payload = protocol.pack_estop(msg.reason)
        self._transport.send(protocol.frame(protocol.MSG_ESTOP, payload))

    def _poll(self):
        data = self._transport.poll()
        if data:
            for msg_id, payload in self._parser.feed(data):
                self._dispatch(msg_id, payload)
        self._check_steering_timeout()

    def _dispatch(self, msg_id: int, payload: bytes):
        stamp = self.get_clock().now().to_msg()
        if msg_id == protocol.MSG_HEARTBEAT and len(payload) == 3:
            counter, flags, mode = struct.unpack('<BBB', payload)
            m = CarlHeartbeat()
            m.header.stamp = stamp
            m.counter, m.flags, m.mode = counter, flags, mode
            self._pub_heartbeat.publish(m)
        elif msg_id == protocol.MSG_THROTTLE_STATUS and len(payload) == 6:
            state, faults, ap_l, ap_r = struct.unpack('<BBhh', payload)
            m = CarlThrottleStatus()
            m.header.stamp = stamp
            m.state, m.faults = state, faults
            m.applied_left = ap_l / 1000.0
            m.applied_right = ap_r / 1000.0
            self._pub_throttle_status.publish(m)
        elif msg_id == protocol.MSG_ENCODER_FEEDBACK and len(payload) == 8:
            rl, rr = struct.unpack('<ii', payload)
            m = CarlEncoderFeedback()
            m.header.stamp = stamp
            m.rear_left_counts, m.rear_right_counts = rl, rr
            self._pub_encoders.publish(m)
        elif msg_id == protocol.MSG_STEERING_FEEDBACK:
            try:
                sample = protocol.unpack_steering_feedback(payload)
            except ValueError:
                return
            # A repeated sample must not keep stale data alive.
            if sample.sequence == self._last_steering_sequence:
                return
            self._last_steering_sequence = sample.sequence
            self._last_steering_rx = time.monotonic()
            self._steering_unavailable_published = False
            msg = CarlSteeringAngle()
            msg.header.stamp = stamp
            msg.header.frame_id = 'steering_encoder_link'
            for name, value in self._steering_config.convert(sample).items():
                setattr(msg, name, value)
            self._pub_steering.publish(msg)
        elif msg_id in (protocol.MSG_BNO_ACCEL, protocol.MSG_BNO_GYRO,
                        protocol.MSG_BNO_QUAT_XYZ, protocol.MSG_BNO_QUAT_W):
            try:
                sample = self._bno_assembler.add(msg_id, payload)
            except ValueError:
                return
            if sample is not None:
                self._publish_bno(sample, stamp)
        elif msg_id == protocol.MSG_BNO_SPI_DIAGNOSTICS:
            try:
                diagnostics = protocol.unpack_bno_spi_diagnostics(payload)
            except ValueError:
                return
            msg = String()
            msg.data = json.dumps(diagnostics, sort_keys=True)
            self._pub_imu_spi_diagnostics.publish(msg)
            malformed = (diagnostics['result'] == 'invalid_length'
                         or (diagnostics['result'] == 'packet'
                             and diagnostics['channel'] > 5))
            fault = bytes(payload[1:7]) if malformed else None
            if fault != self._last_bno_spi_fault:
                self._last_bno_spi_fault = fault
                if malformed:
                    self.get_logger().warning(f'BNO08x SPI header diagnostic: {msg.data}')
        elif msg_id == protocol.MSG_BNO_DIAGNOSTICS:
            try:
                diagnostics = protocol.unpack_bno_diagnostics(payload)
            except ValueError:
                return
            if not diagnostics['ready']:
                self._bno_assembler.reset()
            msg = String()
            msg.data = json.dumps(diagnostics, sort_keys=True)
            self._pub_imu_diagnostics.publish(msg)
            health = (diagnostics['stage'], diagnostics['error'])
            if health != self._last_imu_health:
                self._last_imu_health = health
                if diagnostics['error'] != 'ok':
                    self.get_logger().warning(f'BNO08x SPI diagnostic: {msg.data}')
                elif diagnostics['ready']:
                    self.get_logger().info(f'BNO08x SPI ready: {msg.data}')
        elif msg_id == protocol.MSG_IMU_ACCEL and len(payload) == 8:
            sample = self._imu_assembler.add_accel(payload)
            if sample is not None:
                self._publish_imu(sample, stamp)
        elif msg_id == protocol.MSG_IMU_GYRO and len(payload) == 8:
            sample = self._imu_assembler.add_gyro(payload)
            if sample is not None:
                self._publish_imu(sample, stamp)
        elif msg_id == protocol.MSG_IMU_TEMPERATURE and len(payload) == 4:
            sample = protocol.unpack_imu_temperature(payload)
            if sample.status & protocol.IMU_STATUS_SAMPLE_VALID:
                self._publish_imu_temperature(sample, stamp)
        elif msg_id == protocol.MSG_IMU_DIAGNOSTICS and len(payload) == 8:
            diagnostics = protocol.unpack_imu_diagnostics(payload)
            msg = String()
            msg.data = json.dumps(diagnostics, sort_keys=True)
            self._pub_imu_diagnostics.publish(msg)
            # Report transitions, not the same fault every second. The complete
            # count and failing register remain available on the diagnostic topic.
            health = (diagnostics['stage'] == 'ready', diagnostics['error'])
            if health != self._last_imu_health:
                self._last_imu_health = health
                if diagnostics['error'] != 'ok':
                    self.get_logger().warning(f'IMU diagnostic: {msg.data}')
                elif diagnostics['stage'] == 'ready':
                    self.get_logger().info(f'MPU-6500 ready: {msg.data}')

    def _check_steering_timeout(self):
        if self._steering_unavailable_published:
            return
        if (self._last_steering_rx is not None
                and time.monotonic() - self._last_steering_rx < self._steering_config.timeout_sec):
            return
        msg = CarlSteeringAngle()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'steering_encoder_link'
        msg.status = protocol.STEERING_LINK_STALE
        msg.calibrated = self._steering_config.calibrated
        msg.sensor_voltage = math.nan
        msg.absolute_angle_rad = math.nan
        msg.angle_rad = math.nan
        self._pub_steering.publish(msg)
        self._steering_unavailable_published = True
        # Keep the last sequence: a repeated frame must remain stale. A
        # reboot with the same counter is accepted on its next 20 ms sample.

    def _publish_bno(self, sample: protocol.BnoSample, stamp) -> None:
        """Publish SH-2 SI units in sensor axes, not a vehicle mounting transform.

        Covariances remain zero (unknown). SH-2 heading accuracy is not a full
        orientation covariance; do not fabricate one from a single scalar.
        """
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = sample.accel
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.gyro
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = sample.quaternion
        self._pub_imu.publish(msg)

    def _publish_imu(self, sample: protocol.ImuRawSample, stamp) -> None:
        """Convert one paired raw sample and publish the standard ROS IMU type.

        The MPU-6500 does not contain a magnetometer, and this bridge does not
        estimate orientation. ROS represents that condition by placing -1 in
        the first orientation covariance element. Acceleration and angular-
        velocity covariance remain all-zero until measurements from the mounted
        sensor provide defensible calibration values.
        """
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = protocol.accel_raw_to_mps2(sample.accel[0])
        msg.linear_acceleration.y = protocol.accel_raw_to_mps2(sample.accel[1])
        msg.linear_acceleration.z = protocol.accel_raw_to_mps2(sample.accel[2])

        msg.angular_velocity.x = protocol.gyro_raw_to_radps(sample.gyro[0])
        msg.angular_velocity.y = protocol.gyro_raw_to_radps(sample.gyro[1])
        msg.angular_velocity.z = protocol.gyro_raw_to_radps(sample.gyro[2])

        msg.orientation_covariance[0] = -1.0
        self._pub_imu.publish(msg)

    def _publish_imu_temperature(
        self,
        sample: protocol.ImuTemperatureTelemetry,
        stamp,
    ) -> None:
        """Publish the MPU-6500 die temperature in degrees Celsius."""
        msg = Temperature()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        msg.temperature = protocol.temperature_raw_to_celsius(
            sample.raw_temperature
        )
        # Zero is the ROS convention for unknown temperature variance.
        msg.variance = 0.0
        self._pub_imu_temperature.publish(msg)


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
