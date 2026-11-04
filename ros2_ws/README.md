# AVL_CARL — ROS 2 Workspace

For browser joystick connection recovery, keyboard e-stop/reset behavior, and
front-right motor isolation, see
[`Joystick and keyboard diagnostics`](../docs/joystick-troubleshooting.md).

High-level autonomy and teleoperation for AVL_CARL. Runs on the **NVIDIA Jetson Nano**
(the companion computer) and talks to the **STM32 master** over a framed binary serial
link. All vehicle-level logic — teleop mixing, command arbitration, telemetry parsing —
lives here; the firmware master is a dumb serial↔CAN proxy (see [`../firmware`](../firmware)).

```
Jetson Nano (this workspace)  ──UART framed binary──▶  STM32 master  ──CAN 2.0──▶  Teensy nodes
```

## Layout

```
ros2_ws/
└── src/
    ├── carl_msgs/            ← custom interfaces (throttle cmd/status, encoder, e-stop, heartbeat)
    ├── carl_bridge/          ← serial ↔ ROS 2 bridge (protocol, transports, bridge node)
    │   ├── carl_bridge/
    │   │   ├── protocol.py     ← wire framing, mirrors firmware/lib/carl_protocol/wire.h
    │   │   ├── transports.py   ← serial transport abstraction
    │   │   └── bridge_node.py  ← ROS 2 node: serial <-> topics
    │   ├── config/bridge.yaml
    │   └── launch/bridge.launch.py
    ├── carl_teleop/          ← teleop drivers
    │   ├── carl_teleop/
    │   │   ├── keyboard_teleop_node.py
    │   │   └── joy_teleop_node.py
    │   ├── config/teleop.yaml
    │   └── launch/teleop.launch.py
    └── carl_bringup/         ← top-level launch (carl.launch.py)
```

## Build & run

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Bring up the whole stack
ros2 launch carl_bringup carl.launch.py

# Or run pieces individually
ros2 launch carl_bridge bridge.launch.py
ros2 launch carl_teleop teleop.launch.py
```

## Wire protocol

The serial framing (SOF + length + CRC8) between the Jetson and the STM32 master is the
single source of truth in [`../firmware/lib/carl_protocol/wire.h`](../firmware/lib/carl_protocol/wire.h)
and must be mirrored in [`src/carl_bridge/carl_bridge/protocol.py`](src/carl_bridge/carl_bridge/protocol.py).
Keep the two in sync. The CAN message/ID map is in [`../docs/can-bus.md`](../docs/can-bus.md).

The current master uses **GY-BNO08X over SPI**. Four wire-only snapshot parts
are emitted: `0x234` acceleration (Q8 m/s², including gravity), `0x235`
calibrated angular velocity (Q9 rad/s), `0x236` quaternion XYZ (Q14), and
`0x237` quaternion W plus heading accuracy. These are not CAN frames.
The bridge requires all four valid parts with matching sequence numbers
within 100 ms before publishing `/carl/imu` (`sensor_msgs/msg/Imu`). A dropped
part discards that snapshot, not the remaining fields' unit information.
Quaternions are checked and normalized. Axis directions are the module's
axes in `imu_link`; a vehicle mounting transform is not supplied. Covariances
remain unknown (zero); the heading-accuracy scalar is not a full covariance.

Acceleration, gyro, and orientation are independently scheduled at a requested
100 Hz. The STM32 groups only new reports received within 50 ms; this is not a
claim of simultaneous acquisition. ROS stamps are host receipt times, not the
original SH-2 sensor timestamps. Validate the measured rate on hardware.

`0x238` sends BNO SPI health independently of motion. The bridge publishes
JSON in `std_msgs/msg/String` on `/carl/imu/diagnostics`. Look for
`sensor=BNO08x`, `interface=spi`, `stage=ready`, `error=ok`, and a stable
`error_count`. Failure stages and the saturating count remain visible even if
the sensor cannot initialize. Timeout on missing INT is `boot_timeout`;
no identity response is `identify_timeout`; a missing report is
`report_timeout` (detail identifies report 0x01/0x02/0x05, or the seen-report
bitmask during initial stream setup). Recovery resets only the sensor.

**There is no MPU die-temperature stream with this sensor.** The legacy
temperature publisher/subscription is retained for compatibility, but a
temperature panel waiting for messages is expected. The old MPU decoder and
IDs `0x230`–`0x233` remain supported for older firmware; BNO values never use
the MPU conversion constants. See `../docs/wiring.md` before connecting SPI.

After flashing and rebuilding the bridge, test without repeatedly pressing
Reset. In a terminal with the updated ROS overlay sourced, observe:

```bash
ros2 topic echo /carl/imu/diagnostics --full-length
```

In a separate terminal, `ros2 topic hz /carl/imu` should show a rate near
100 Hz while healthy. Check for uninterrupted telemetry and whether the error
count stays constant over several minutes. `stage=ready` alone does not prove
that sampling remains healthy; use it together with fresh messages.

After changing this firmware/protocol, rebuild the bridge and restart it with
the updated overlay sourced. Stop existing CARL nodes first; do not start a
second hardware bridge on the same serial port:

```bash
source /opt/ros/lyrical/setup.bash
cd /home/izakulus/Documents/AVL/AVL_CARL/ros2_ws
colcon build --packages-select carl_bridge --symlink-install
source install/setup.bash
ros2 launch carl_bringup carl.launch.py
```

Use the ROS distribution and path appropriate to the deployment. Restart
rosbridge with the same overlay sourced. The browser's existing `/carl/imu`
subscription displays acceleration, gyro, and now orientation without changes
to the controls application. Use the terminal for diagnostics. Logging is not
implemented by this change.

## Absolute steering angle

`/carl/steering_angle` (`carl_msgs/msg/CarlSteeringAngle`) carries 50 Hz raw
ADC, reconstructed voltage, absolute shaft angle, calibrated steering angle,
and validity/status. Angles are radians. Centered `angle_rad` remains NaN until
mounting calibration is enabled; a missing stream produces an invalid stale
event after 250 ms. Rebuild/source `carl_msgs` and `carl_bridge` together. See
[encoder wiring, topic fields, calibration and bench checks](../docs/steering-encoder.md).
