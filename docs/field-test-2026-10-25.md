# Outdoor test 1 — October 25, 2026

## Report and scope

CARL barely climbed the reported inclines (estimated below 30 degrees) at the
controls application's nominal 40% joystick throttle limit. The phone showed
“Input paused,” then “another client was connected,” after a few seconds to
roughly a minute. Manually reconnecting restored control. CARL's companion was
a laptop; the Jetson was not installed. Network coverage has known weak areas.
No field logs, measured link latency, wheel-distance calibration or ground-truth
speed measurements accompanied this report.

Work in `AVL_CARL` stayed on **imu-pd12-integration**. Existing uncommitted IMU,
encoder, firmware and teleop changes were preserved. This update changes the
ROS bridge command queue and adds a firmware host-test runner; it does not
change firmware motor behavior or require a reflash. Companion changes are in
the sibling `CARL_Joystick` and `CARL_Controls_Application` repositories.

## Velocity request: deliberately not implemented

No velocity topic, estimator, IMU velocity fallback or 5 mph governor was added.
The nominal 40% joystick limit stays in place, so the incline torque issue is
not resolved by this update. It is a command limit, not a measured speed cap;
existing firmware heading/rear-wheel corrections can adjust individual PWM.

The current implementation cannot satisfy the requested reliability condition:

- Rear telemetry contains two raw int32 counts. The documented 2400 counts per
  encoder revolution does not establish loaded rolling circumference or a
  measured encoder-to-wheel distance calibration. Sign, scale and error across
  forward/reverse, turns, surfaces, loads and inclines need validation.
- Encoder frames contain no acquisition timestamp, sample sequence or explicit
  sensor health. ROS stamps them on receipt at the companion. Delay, backlog,
  resets or a frozen channel must be distinguished from a stationary wheel.
- Rear wheel rotation alone does not measure chassis speed during slip, wheel
  lock or sliding. Two rear encoders are not independent ground-speed sources.
- The IMU publishes sensor-axis acceleration/angular rate and orientation, with
  unknown covariance and no validated sensor-to-vehicle mounting transform,
  bias estimate, gravity compensation or velocity error bound. Acceleration
  integration is not a dependable replacement for missing wheel speed.
- Existing zero-output and e-stop behavior coasts. A speed estimate plus reduced
  drive PWM cannot guarantee 5 mph on a descent without validated braking
  authority. No brake behavior was introduced.

Future work needs measured wheel distance/timing, sensor fault detection and
uncertainty, synchronized vehicle-frame sensor calibration, ground-truth speed
comparisons under slip and slopes, and demonstrated stopping/overspeed control.
The IMU must not silently become an unaided long-term velocity fallback.
The [VectorNav inertial navigation primer](https://www.vectornav.com/docs/default-source/educational-material/inertial-navigation-primer-r3.pdf)
explains the bias and drift limitations underlying that decision.

## Connection findings and changes

Joystick commands travel directly from the phone to `carl_joystick` on port 8001;
rosbridge on port 9090 supplies browser telemetry. The observed sequence matches
an identified recovery defect: after a delayed response/reconnect, an old
half-open socket can retain controller ownership; the old browser permanently
stops retrying on the busy response. The precise cause of the initial outdoor
delay remains unmeasured. The regression reproduces the recovery defect with
actual WebSockets and the current browser code.

1. The server bounds receive silence and blocked replies to 3 seconds and frees
   ownership before a timeout close handshake. The phone retries
   `controller_busy` with 0.5–5 s backoff and starts neutral on every attempt.
   It never evicts another live controller or automatically resumes motion.
2. Drive/reset packets echo a single-use server-issued protocol-v2 token, valid
   for less than `input_timeout` on the server monotonic clock. Expired,
   replayed or missing tokens command zero and require neutral. This rejects a
   delayed initial drive even when there was no previously active command to
   expire. Neutral and e-stop remain accepted without a fresh token.
3. Browser telemetry requests a 100 ms interval and queue length 1 per topic.
   High-rate IMU telemetry consumes less shared Wi-Fi bandwidth. This changes
   browser sampling/counts, not ROS publisher rates or complete message fields.
4. The joystick WebSocket uses a one-packet input queue, disables compression,
   and sends protocol pings every 5 s with a 10 s timeout. The bridge's throttle
   subscription queue is reduced from ten samples to one to limit replay of
   obsolete ROS drive commands after scheduling stalls.
5. The phone displays last reply latency, pause count and reconnect attempts.
   ROS logs report motion-stop reasons and session closure codes/reasons.
   `/health` reports readiness, last-stop reason and motion-stop count.

The browser's 200 ms motion pause, server's 250 ms input watchdog (plus up to
50 ms timer tick), STM32's 300 ms teleop watchdog, and throttle's 150 ms command
watchdog are preserved. Closing a network session is independent of commanding
zero. Software cannot maintain a live link through a complete coverage outage;
these changes reduce traffic and recover safely when connectivity returns.

References for the transport options:
[Uvicorn settings](https://uvicorn.dev/settings/) and
[rosbridge subscribe protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md).

## Apply on the laptop

Stop the existing joystick server and hardware bridge. Keep other throttle
publishers stopped. In a terminal:

```bash
source /opt/ros/lyrical/setup.bash
cd /home/izakulus/Documents/AVL/AVL_CARL/ros2_ws
colcon build --symlink-install --packages-select carl_msgs carl_bridge carl_teleop carl_bringup
source install/setup.bash
cd /home/izakulus/Documents/AVL/CARL_Joystick
colcon build --symlink-install --packages-select carl_joystick
source install/setup.bash
```

Restart the usual processes in separate terminals sourced with ROS and the
appropriate overlays:

```bash
ros2 launch carl_bringup carl.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 launch carl_joystick joystick.launch.py controls_dir:=/home/izakulus/Documents/AVL/CARL_Controls_Application/src
```

**Restart the joystick server and reload the phone page together.** Protocol v2
is required for driving. A new browser refuses an old server; an old browser
cannot drive the updated server. Firmware upload is unnecessary for these
changes. The existing browser passcode and trusted-network assumptions remain.

## Verification and next field run

Completed software checks for this update:

| Check | Result |
| --- | --- |
| Browser transport and touch UI | 31 tests passed |
| Joystick controller, API and lease | 21 tests passed, no skips |
| ROS bridge protocol/dispatch | 31 tests passed, no skips |
| Keyboard teleop regressions | 5 tests passed, no skips |
| Real WebSocket + isolated ROS integration | 2 tests passed, including injected link failures and 65 s held-input soak |
| Firmware host regressions | 9 executables passed with undefined-behavior sanitizer enabled |
| PlatformIO native protocol | 21 tests passed |
| Firmware builds | master, throttle, steering passed |
| ROS workspace builds | carl_msgs, carl_bridge, carl_teleop, carl_bringup and carl_joystick passed |

The installed Uvicorn/websockets stack emits legacy-backend deprecation
warnings, and existing ROS package setup metadata emits `tests_require`
warnings. These checks completed successfully; no dependencies were upgraded
in the vehicle runtime. A temporary ROS-compatible test environment supplied
the missing `httpx` test dependency. API/integration tests needed execution
outside the restricted sandbox for asynchronous localhost networking and ROS
logging. No test opened a physical vehicle connection.

Software verification covers browser input/release/lifecycle, controller
ownership and limits, watchdogs and e-stop/reset, stale/replayed command
rejection, server API, ROS wire serialization, IMU telemetry, keyboard teleop,
and host simulations of actual master/throttle code. Firmware builds cover
master, throttle and steering; native protocol tests also run. Live integration
uses isolated ROS domain 213, real joystick/bridge nodes and real localhost
WebSockets with an in-memory vehicle transport, never a hardware connection.

Reproduce firmware host checks from `AVL_CARL` with:

```bash
python3 firmware/test/run_host_regressions.py
cd firmware
pio test -e native
pio run -e master -e throttle -e steering
```

The sibling joystick README documents browser, controller/API and ROS integration
commands. Set `CARL_SOAK_SECONDS=65` for the held-input endurance scenario. That
test also injects 350 ms delayed replies, a 300 ms delayed drive request, and a
half-open old phone session. It verifies automatic busy recovery, zero output
after input loss and a fresh touch before movement resumes.

Physical acceptance remains pending. With wheels supported, verify forward,
reverse, turns, release and e-stop/reset; interrupt phone Wi-Fi while commanding
motion and confirm output drops, then confirm recovery stays neutral. Repeat
phone/laptop operation beyond the previous one-minute interval at the actual
weak-coverage locations. Capture phone reply latency, pauses, retries and laptop
ROS logs, plus encoder/IMU/heartbeat/status rosbag data if available. Only then
evaluate low-speed operation on level ground. The existing incline performance
and coast-only descent limitations remain; this software validation does not
establish physical braking, ground-speed accuracy or outdoor link quality.
