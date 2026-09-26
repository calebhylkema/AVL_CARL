# AVL_CARL — ROS 2 Workspace

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

The master also emits three wire-only MPU-6500 messages that are not placed on
CAN: `0x230` carries raw acceleration, `0x231` carries raw angular velocity,
and `0x232` carries raw die temperature. Matching sequence numbers let
`carl_bridge` combine the motion frames and publish `/carl/imu` as
`sensor_msgs/msg/Imu`; die temperature is converted to degrees Celsius and
published separately on `/carl/imu/temperature` as
`sensor_msgs/msg/Temperature`.
