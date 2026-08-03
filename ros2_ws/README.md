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
    └── carl_bridge/          ← serial ↔ ROS 2 bridge (placeholder scaffold)
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── carl_bridge/
            ├── protocol.py    ← wire framing, mirrors firmware/lib/carl_protocol/wire.h
            └── bridge_node.py ← ROS 2 node: serial <-> topics
```

> **Status:** placeholder scaffold. The real bridge/teleop/telemetry packages are
> developed on the Jetson and will be committed here. Drop additional packages under
> `src/`.

## Build & run

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run carl_bridge bridge_node
```

## Wire protocol

The serial framing (SOF + length + CRC8) between the Jetson and the STM32 master is the
single source of truth in [`../firmware/lib/carl_protocol/wire.h`](../firmware/lib/carl_protocol/wire.h)
and must be mirrored in [`src/carl_bridge/carl_bridge/protocol.py`](src/carl_bridge/carl_bridge/protocol.py).
Keep the two in sync. The CAN message/ID map is in [`../docs/can-bus.md`](../docs/can-bus.md).
