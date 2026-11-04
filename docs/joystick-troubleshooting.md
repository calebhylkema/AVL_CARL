# Joystick and keyboard diagnostics

## Launch paths

`ros2 launch carl_bringup carl.launch.py` includes
`carl_bridge/launch/bridge.launch.py`. Launching the bridge directly starts the
same hardware node. Use one launch path. The repository does not define a
`carl_bringup bridge.launch.py` launch file.

Browser joystick mode additionally needs `carl_joystick` and rosbridge for page
telemetry. Do not run keyboard/gamepad teleop alongside the browser controller:
they publish the same `/carl/throttle_cmd` topic without arbitration.

## Rear-left stops at full vertical inside the +/-0.05 horizontal band

The browser server's `mix()` deliberately maps `abs(horizontal) <= 0.05` to
zero turn. With vertical `+1` and the default 40% limit it publishes **all four
wheel commands = +0.4**, encoded as four +400 values. Reverse produces -400.
Just outside the horizontal dead zone the sides become unequal. Neither the
server nor the bridge singles out rear-left in that band.

This transition exposed an overly aggressive rear encoder controller: equal
rear commands enabled synchronization, which could integrate a reported speed
difference until one wheel's PWM reached zero. Leaving the band bypassed
synchronization and restored normal PWM. Missing/misassigned encoder counts
could therefore selectively stop rear-left while the other wheels continued.
The first regression reproduced this path with constant/missing counts. A later
hand-rotation check confirmed the physical rear-left wheel changed the old
**rear-right** count. The installed encoder pairs were crossed in firmware.
The PWM floor prevented a complete stop but did not fix this positive feedback.

The throttle now assigns **rear-left A=16/B=17**, **rear-right A=14/B=15**.
This fixes the regulator's motor association and the CAN/ROS count labels at
their source. Keep the installed harness unchanged. With this correction,
slowing a wheel lowers its own measured speed instead of the other wheel's.
A physical-pin motor model reproduced the reported full-throttle straight
case at 300/500 PWM commands with the old map, and converges with the corrected
map. This correction needs only a throttle Teensy flash when the previous
master firmware has already been installed.

Rear regulation now requires fresh counts from **both** encoders, limits a
wheel's reduction to 25% of its nominal command, and never crosses the input
deadband or reverses direction. Either encoder going stale restores both
nominal rear commands and disables correction for the rest of that run. Boost
is still limited to 50% of request and 100 per-mille. This prevents the
controller from zeroing a nonzero rear command; motor breakaway torque,
hardware faults, or incorrect encoder mapping still require a bench check.

After flashing, with wheels supported, inspect `ros2 topic echo /carl/encoders`.
Turn each rear wheel separately and verify only its own count changes. Jog one
side at a time and confirm motor names match encoder names; both encoders should
produce about 2400 counts per wheel revolution. A mirrored encoder's sign may
be reversed; the regulator compares magnitudes. If the mapping is swapped,
correct the physical mapping/pin configuration rather than increasing gains.
Test both vertical signs, x=-0.05/0/+0.05, and just beyond the band.

Other command-path fixes in this update:

- The throttle node stops all wheels after 150 ms of missing commands even with
  healthy heartbeats (`faults & 4`), and watchdogs no longer falsely expire if
  input processing crosses a millisecond boundary.
- E-stop/reset cannot revive retained master commands; reset does not act as a
  drive-command keepalive. Direction changes clear the inactive PWM pin first.
- The server publishes each fresh accepted joystick position immediately without
  replaying the old wheel command first. Release still sends zero immediately.
- A rapid Reset click cannot overwrite a queued/unacknowledged browser E-stop.
  Malformed joystick events release held input.
- Non-finite ROS throttle fields produce an immediate all-zero bridge frame
  instead of raising a callback exception while the previous command runs.

## Front-right wheel does not move in either direction

The current software maps all four command fields independently through the
ROS bridge, STM32 relay, CAN payload and throttle firmware. Front-right is the
second signed int16, at CAN payload byte offset 2. In the throttle firmware it
drives Teensy pins **RPWM=4, LPWM=5, R_EN+L_EN=11**, with direction sign **-1**.
At a vehicle-forward command of +0.25, LPWM pin 5 receives about 25% duty at
20 kHz, RPWM pin 4 is zero, and pin 11 is HIGH. Reverse exchanges the PWM pins.

The software regression compiles the actual throttle source against GPIO/CAN
test doubles and verifies both FR directions, isolated FR commands, all four
wheel outputs, e-stop, heartbeat loss and disabled-output telemetry. This
does not measure signals on a physical Teensy or identify a failed driver.

Both forward and reverse failing makes the common enable, driver supply/ground,
driver output, motor cable and motor the next isolation points. With CARL
secured and wheels lifted:

1. With power removed, check continuity from Teensy **pin 11 to both R_EN and
   L_EN** at the FR driver. VCC=5 V alone does not enable an IBT_2. Check logic
   ground continuity back to the Teensy; confirm both motor output leads.
2. During a short low-output command, measure the enable at the driver relative
   to that driver's GND. Both enables should be HIGH. Inspect pins 4 and 5 at
   the Teensy and at the driver with a scope/logic analyser. One PWM input
   should switch at 20 kHz; reverse should swap it. Do not apply 5 V to Teensy
   GPIO. The existing driver logic-level caveat is documented in `wiring.md`.
3. If enable/PWM is correct at the driver but its output does not drive the
   motor, isolate the driver and motor. With power removed before each change,
   connect the FR motor to a known-working driver output and retest briefly
   with wheels lifted. If it works there, investigate the FR driver/control
   path; if it still fails, investigate the motor, its leads or mechanical load.

Neither heartbeat nor software e-stop can selectively disable FR in this
firmware. E-stop and heartbeat loss coast all four channels. A software e-stop
left latched after a crash prevents all four wheels from driving until reset.

`applied_left/right` are software PWM-equivalent side averages, not individual
wheel speed, current or electrical measurements. There are only rear encoders.
A normal right-side value does not prove the FR motor is turning. The updated
throttle firmware reports zero applied output when disabled, even if an old
nonzero command is retained.

## Idle joystick disconnects

Two old transport deadlines were separate from motor safety: the browser closed
the socket after 200 ms without a reply, and the server closed it after one
second without an application packet. Pointer movement is not required for
keepalive; the client sends at 20 Hz even when neutral or held still.

Healthy idle sessions stay open by sending neutral keepalives. Silent sessions
expire after three seconds; the browser automatically retries a busy server
while the old half-open session is being released. A reply delayed by 200 ms cancels held input
and requires neutral acknowledgement and a fresh touch; it does not immediately
disconnect. Three seconds without a reply causes automatic neutral reconnect
with 0.5–5 second backoff. Hiding the page disconnects; returning restores a
previously requested session at rest. Explicit Disconnect and sign-out cancel
reconnect. The server's independent 250 ms motion watchdog, checked every 50 ms,
and STM32's 300 ms companion watchdog are unchanged. After restarting the server,
reload the controls page to load the updated JavaScript.

The October 25 field-test update also requires protocol-v2 freshness tokens for
drive/reset, samples browser telemetry at 10 Hz with queue length one, and adds
phone link diagnostics and server stop/session logs. Update the joystick server
and phone scripts together. See [outdoor test 1](field-test-2026-10-25.md) for
verification results and the decision to defer velocity estimation.

## Keyboard crashes on x / e-stop

ROS 2 Lyrical's logger provides `warning()`, not the old `warn()` alias. The old
handler sent the e-stop and then raised `AttributeError` while logging. The master
correctly retained its e-stop latch after the operator process exited.

The handler now uses `warning()`, immediately publishes zero, and stays running.
`r` clears held keys, requests reason 0 reset and holds zero until the master
heartbeat confirms its e-stop flag cleared. A fresh movement key is then needed.
Remote e-stop heartbeat feedback also stops the keyboard locally. Shutdown
restores terminal settings even if the ROS context has already stopped.

## Apply updates

Stop existing ROS control processes, then rebuild:

```bash
source /opt/ros/lyrical/setup.bash
cd /home/izakulus/Documents/AVL/AVL_CARL/ros2_ws
colcon build --symlink-install --packages-select carl_msgs carl_bridge carl_teleop carl_bringup
source install/setup.bash
cd /home/izakulus/Documents/AVL/CARL_Joystick
colcon build --symlink-install
```

Source ROS and both overlays in new terminals and restart the existing
three-process setup. The normal hardware launch remains:

```bash
ros2 launch carl_bringup carl.launch.py
```

Flash **both** the master and throttle firmware for the center-band, watchdog,
and e-stop fixes while the vehicle is secured:

```bash
cd /home/izakulus/Documents/AVL/AVL_CARL/firmware
pio run -e throttle -t upload
pio run -e master -t upload
```

## Reproduce software checks

Keyboard tests use installed ROS interfaces but no hardware/ROS graph:

```bash
cd /home/izakulus/Documents/AVL/AVL_CARL
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
PYTHONPATH=ros2_ws/src/carl_teleop:$PYTHONPATH python3 -m unittest discover -s ros2_ws/src/carl_teleop/test -v
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/test/support/throttle -Ifirmware/lib/carl_protocol \
  -Ifirmware/lib/common -Ifirmware/lib/heartbeat \
  firmware/test/support/throttle/test_throttle.cpp -o /tmp/carl-throttle-regression
/tmp/carl-throttle-regression
```

The joystick README documents its controller/API and isolated-domain ROS tests;
the controls application runs `node --test tests/*.test.js`. Hardware movement
is not proof-tested by these simulations; use the FR isolation procedure above.

`firmware/test/support/throttle/test_center_band.cpp` compiles the actual throttle
node and checks nonzero rear outputs throughout the equal-command region with
missing/mismatched counts, both travel directions, and transitions into/out of
the band. Run it with the same compiler include flags as `test_throttle.cpp`.
