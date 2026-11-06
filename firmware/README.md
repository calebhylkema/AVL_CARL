# AVL_CARL Firmware

PlatformIO monorepo for the three CAN nodes. The master proxies a ROS2 companion
computer (framed binary over serial) to the CAN bus. Operator mixing, arbitration,
and ROS telemetry parsing live in the ROS2 workspace; IMU acquisition, rear speed balancing,
safety, and hardware I/O run on the MCUs:

- **Master (NUCLEO-F767ZI)** — serial ↔ CAN proxy, heartbeat producer, e-stop
  latch, teleop watchdog, and MPU-6500/9250 SPI acquisition. Calibrated relative
  heading hold adds bounded differential throttle during straight commands.
  E-stop or teleop timeout zeroes all outputs.
- **Throttle (Teensy 4.1)** — 4× IBT_2 H-bridge control, 2× rear encoder
  quadrature decode, rear RPM balancing, fail-safe on heartbeat loss.
- **Steering (Teensy 4.1)** — absolute steering encoder telemetry; motor remains disabled (Phase 1 is differential drive).

See [`../docs/wiring.md`](../docs/wiring.md) for the pin-level wiring and
[`../docs/can-bus.md`](../docs/can-bus.md) for the locked CAN message map.
The wire framing between master ↔ companion is defined in
[`lib/carl_protocol/wire.h`](lib/carl_protocol/wire.h) and mirrored in the ROS2
workspace at `../ros2_ws/src/carl_bridge/carl_bridge/protocol.py`.

## Build / flash

```bash
pio run -e throttle      # build ONLY the throttle node
pio run -e master        # build ONLY the master node
pio run -e steering      # build ONLY the steering node
pio run                  # build ALL three nodes

pio run -e throttle -t upload    # flash a connected Teensy throttle node
pio run -e master   -t upload    # flash the NUCLEO over ST-LINK
pio run -e steering -t upload    # flash a connected Teensy steering node
```

## MPU-6500/9250 SPI on master (2026-11-05)

The standard **`master`** target now uses the installed MPU-6500/9250 module
on SPI4. There is one master build target. It keeps the current CAN proxy,
steering feedback forwarding, heartbeat, e-stop and teleop watchdog, and does
not initialize the disconnected BNO reset/wake pins. MPU relative heading hold
uses stationary gyro calibration and planar yaw integration as described below.
There is no magnetometer readout or absolute heading. The preserved BNO implementation remains in source
and host regressions; `CARL_IMU_MPU_SPI` selects MPU support for `master`.

```bash
# From the repository root:
pio run -d firmware -e master -t upload
python3 firmware/tools/monitor_mpu_spi.py --seconds 60
```

The serial observer is read-only and requires pyserial. Stop an existing bridge
before using the observer; only one reader should own the ST-LINK serial port.
Keep the wiring and Reset button untouched during a continuous stability test.
No drive commands are sent by the observer.

SPI4 mapping: PE6 MOSI, PE5 MISO, PE2 SCK, PE4 NCS. The MPU's SDA/SDI connects
to MOSI; AD0/SDO connects to MISO. See the replacement-module wiring table below
in `../docs/wiring.md`. Reads use SPI mode 3 at a requested 1 MHz. The driver
reads WHO_AM_I at register 0x75 before making any sensor configuration writes:
0x70 is accepted as MPU-6500 and 0x71 as MPU-9250. Unknown IDs remain read-only
and report an error. Reset/wake waits are scheduled, configuration registers
are read back, and missing data triggers sensor-only retry.

Raw acceleration/gyro/temperature use the existing 0x230/0x231/0x232 wire
layouts at a requested 100 Hz, +/-2 g and +/-250 degrees/s. The existing
`/carl/imu` publisher marks orientation unavailable. New diagnostic wire ID
0x23b is schema 1: version, stage, identity, error, register, readback value,
cumulative error count u16 LE. Stages: 0 power wait, 1 identify, 2 reset wait,
3 wake wait, 4 configure, 5 ready, 6 retry wait. Errors: 0 OK, 1 wrong identity,
2 verify failed, 3 data timeout, 4 identity lost. The updated bridge publishes
this as SPI health on `/carl/imu/diagnostics`; rebuild/source `carl_bridge` and
restart the bridge to use the decoder. The observer also decodes these frames.

The installed replacement returned WHO_AM_I=0x71 and produced live motion data.
Early captures included wire reconnections/reset activity reported by the user.
A fresh undisturbed 60-second capture produced 6,042 complete samples (100.69 Hz),
with the cumulative sensor error count unchanged at 5 and a maximum host sample
gap of 16.4 ms. Steering/wheel telemetry continued. One serial CRC resynchronization
was recorded; it is distinct from sensor/SPI error accounting. A follow-up
20-second capture after draining the serial backlog produced 2,003 samples
(100.11 Hz), zero CRC errors, and no new sensor errors. See the
[hardware capture](../docs/assets/mpu-spi-test-2026-11-05.json). These results establish
communication and raw readout, not calibration or steering performance. The gyro
has a bias; the heading adapter now calibrates it at rest before control.

## MPU relative heading hold

The master uses [`master/src/mpu_heading.h`](master/src/mpu_heading.h) with the
shared [`master/src/heading_hold.h`](master/src/heading_hold.h) controller. Only
the **master** firmware changes; throttle CAN commands and raw IMU telemetry
keep their existing layouts. The bridge still marks `/carl/imu` orientation
unavailable: the local controller's relative yaw is not a fused ROS orientation.

"Straight" means all four original operator wheel commands are equal and at
least 20 per-mille in magnitude. At the start of each eligible forward/reverse
run, the **current vehicle heading** becomes its reference. Changing throttle
magnitude retains that reference. Neutral, an intentional turn, and reversing
direction release it; the next eligible run captures the current heading.
An IMU cannot determine whether the steering wheels are physically straight,
and wheel alignment alone does not define an absolute compass bearing.

Before engagement, keep CARL still and controls neutral. Calibration requires
three continuous seconds and at least 250 samples with stable acceleration and
gyro readings, plus **fresh unchanged rear encoders** (100 ms freshness, 500 ms
without a count change) and **fresh idle/zero-output throttle status** (150 ms).
All three gyro biases are learned in RAM. No live-capture bias is hard-coded
and nothing is written to sensor offset registers or flash. Allow another
500 ms of valid heading samples before commanding a straight run. Returning
to stationary neutral after a run starts a fresh calibration. If a run starts
too early, manual throttle remains available; return to neutral to engage hold.

Gyro scale is 131 LSB/(degree/s) at the configured +/-250 degree/s range; accel
scale is 16,384 LSB/g at +/-2 g, per the
[TDK register map](https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf).
The installed sensor's 20-second **read-only** rosbridge capture on 2026-11-05
contained 2,004 samples, unchanged rear counts, zero applied throttle, and no
new SPI errors. Mean gyro was approximately (+0.1283, -0.1661, +0.0096) rad/s;
gravity pointed mostly along negative Z. This sizeable three-axis bias and
upside-down mounting are handled by calibration and gyro projection onto
measured world-up. The compact raw-count fixture is
[`test/fixtures/mpu_stationary.csv`](test/fixtures/mpu_stationary.csv); its columns
are elapsed ROS timestamp in milliseconds, ax, ay, az, gx, gy, gz. Replay passes
through the production estimator/controller without accessing vehicle hardware.

This is a **planar, gyro-only relative hold**, with a rigid mount within 30 degrees
of flat, either face upward. It does not correct lateral displacement. At each
20 ms command tick the bounded PID trim applies to all four wheels, subtracting
from signed left and adding to signed right commands in forward **and** reverse.
Defaults: kp=180, ki=20, kd=40, 0.5-degree heading dead zone, at most 100 per-mille
or 25% of input trim, 200 per-mille/second trim slew. No wheel reverses or crosses
below the drive deadband. Minimum throttle has no trim authority. Rear equal-RPM
balancing bypasses unequal commands so it does not oppose a heading correction.
Manual turns, pivots and individual-wheel commands retain their original PWM.

Missing samples for more than 30 ms, SPI reset/failure, gyro saturation, excessive
yaw (>3 rad/s), rapid roll/pitch, acceleration outside 0.8–1.2 g, gravity direction
change >15 degrees, or temperature change >5 C releases correction. To bound
unobservable gyro drift, correction also releases after **30 seconds per straight
run**, and calibration expires after 60 seconds. These are conservative control
limits, not measured sensor accuracy guarantees. Manual PWM passes through on
any of these failures, with neutral/stationary recalibration required before
heading correction can resume. E-stop and teleop, heartbeat and throttle-command
watchdogs continue to take precedence and command zero/coast as before.

**Validation and commissioning:** the code was compiled for all three boards,
tested with host hardware doubles and a live-data replay, and checked with the
existing ROS regression suites. The heading change has **not been flashed** and
no powered drivetrain test was performed, per the operator's instruction. Sensor
sign under chassis motion, real vehicle response, gains under load and vibration
remain unvalidated. Before ground use, support all wheels clear with e-stop in
reach, verify forward/reverse correction signs, manual turns, neutral and all
stop paths, then tune at low speed in a controlled area. A counterclockwise yaw
disturbance should increase signed left command and decrease signed right
command. The known front-left misalignment may exceed the limited trim authority.

Wire-only `0x239` now reports this controller's state, readiness, relative yaw,
reference-minus-yaw and signed differential trim at 10 Hz. Existing ROS/GUI
software may ignore it; no companion rebuild is needed for the control change.

## Preserved GY-BNO08X implementation (historical)

The following BNO wiring and behavior describe the earlier sensor. The current
`master` build uses the MPU module above. BNO drivers and their regressions
remain available in source; they are not the installed sensor implementation.

The later SPI BNO driver, heading controller, health/SPI diagnostics, and their
regressions were recovered from the preserved `firmware-before` snapshot dated
2026-11-02. The older Git commit titled as a BNO integration still contains an
MPU driver and was not used as a rollback. The current master command/safety
paths and steering feedback forwarding remain in place. Throttle/steering
production sources and current ROS bridge changes are retained.

The restored hardware mapping is **SPI4 on CN9**: PE6 MOSI, PE5 MISO, PE2 clock,
PE4 chip select, PF8 INT, PE3 reset, PF7 PS0/WAKE. See the complete
[CN8/CN9 wiring table](../docs/wiring.md#4a-gy-bno08x-spi--master-nucleo-f767zi).
There are no CN11/CN12 connections. Transfer settings remain 1 MHz requested,
MSB-first, mode 3. Acceleration, calibrated gyro, rotation vector, and game
rotation vector are requested at 100 Hz. The existing bridge already supports
all four BNO snapshot parts and both diagnostic schemas; no bridge source
change is required for this reinstatement. BNO provides no die-temperature stream.

Only the **master** needs flashing for this change. Builds and host simulations
do not establish electrical communication, real-time timing on the vehicle,
calibration quality, mounting/yaw sign, or steering gains under load. After
rewiring/flashing, leave controls neutral and monitor:

```bash
ros2 topic echo /carl/imu/diagnostics --full-length
ros2 topic echo /carl/imu/spi_diagnostics --once --full-length
ros2 topic hz /carl/imu
```

Confirm `stage=ready`, `error=ok`, fresh telemetry near the requested rate,
and no increasing error count. Perform the heading sign and stop checks below
before driving. Firmware has been built/tested on the host; this change does
not automatically upload to a connected board.

## Rear wheel RPM balancing

The throttle Teensy runs a relative PI speed controller in
[`throttle/src/rear_speed_sync.h`](throttle/src/rear_speed_sync.h). When RL and
RR commands are **equal**, have the same direction, and are outside the input
deadband, it adjusts rear PWM to bring their measured RPM magnitudes together.
This works in forward and reverse without assuming encoder A/B polarity. Each
encoder must be connected to its own wheel and have the same effective counts
per wheel revolution (currently 600 PPR × 4 = 2400). Raw signed encoder CAN
telemetry is unchanged.

**Installed encoder mapping:** rear-left uses Teensy A=16/B=17; rear-right
uses A=14/B=15. Hand rotation confirmed that the older assignment was crossed.
With crossed feedback the synchronizer slows the physically slower wheel and
boosts the faster one, driving its outputs to their limits. The corrected pin
assignment fixes control and `/carl/encoders` labels together; no harness swap
or master/ROS protocol change is needed. Flash only the throttle Teensy for
this pin-mapping correction if the master already has the previous updates.

The controller samples at 20 ms, filters speed with an 80 ms time constant,
qualifies feedback for 160 ms, and uses relative speed error to remain
responsive at low throttle. Proportional gain is `KP=0.20`; integral gain is
`KI=0.80` per second. The faster wheel retains at least 75% of requested PWM
and never crosses below the input deadband; the slower wheel
can receive at most an extra 50% of its command, capped at 100 per-mille
(10 percentage points of PWM). Total output stays within 0–100% in the commanded
direction. The integral and correction slew are bounded. Speed matching yields
to these output limits rather than stopping a commanded wheel.

Both encoders must have accumulated at least four counts and have edges within
200 ms before regulation can engage. Missing counts on either side restore
**both** nominal commands; missing feedback is never interpreted as a target of
zero RPM for the other wheel. After feedback is lost during regulation, direct
PWM remains active until neutral, a turn, or reversal starts a new qualification.
Counter jumps implying more than 3000 RPM also bypass regulation. A foreground
sampling gap over 200 ms discards old control state. These are feedback sanity
checks: a disconnected encoder,
an actual stall, and extremely slow rotation cannot be distinguished by counts
alone. Below motor breakaway torque, matching nonzero speed is not guaranteed;
do not compensate by blindly raising controller gains.

Neutral, disarm, heartbeat/command loss, direction reversal, and unequal commands reset
the controller immediately. Unequal commands, one-wheel operation, and pivot
turns retain direct PWM control, preserving differential steering. Front wheels
also retain direct PWM control: rear encoder measurements cannot identify front
motor speed error. Status `applied_left/right` reports the actual post-regulation
front/rear PWM average on each side, including zero during disabled states.

Throttle remains a nominal per-mille PWM request with rear speed balancing;
it is not an absolute RPM setpoint. Maximum geared wheel RPM has not been
calibrated. Large motor differences or output saturation can exceed the allowed
trim authority, leaving a residual speed mismatch. Verify each encoder tracks
its own motor; a swapped mapping can reverse the effect of the feedback loop.

The throttle node independently coasts **all four wheels** if no valid throttle
command arrives for 150 ms, even while heartbeats continue. This reports
`TF_COMMAND_LOST=0x04` / ROS `FAULT_COMMAND_LOST=4`. Heartbeat restoration alone
cannot reapply an expired motor command. Master and throttle watchdog timestamps
are sampled after input processing to avoid millisecond-boundary underflow.
E-stop/reset discards retained master commands, and reset messages do not refresh
the master teleop watchdog. When changing motor direction, the inactive PWM
input is driven low before setting the active input.

Before testing on the ground, support the vehicle with all wheels clear and
keep the e-stop accessible. Verify that rotating each rear wheel changes its
own encoder count, approximately 2400 counts per wheel revolution. Then test
equal forward and reverse commands at 5%, 10%, 25%, and higher throttle, returning
to neutral between directions. Estimate RPM from count differences:
`RPM = abs(delta_counts) * 60000 / (2400 * delta_ms)`, preferably over 200–500 ms
windows at low speed. Confirm the mismatch settles without hunting; reduce
`KP`/`KI` if it oscillates. The initial gains are simulation-tested and require
vehicle validation under load. Test turns, neutral, e-stop, and heartbeat loss
before a ground run.

## Preserved BNO heading adapter (historical)

This section describes the retained BNO orientation adapter and commissioning
background. The current MPU build feeds its own calibrated planar yaw to the
same bounded controller; it does not use BNO reports or BNO calibration status.

[`master/src/heading_hold.h`](master/src/heading_hold.h) runs on the STM32 master,
where the GY-BNO08x SPI connection is located. Heading hold is enabled by default
and only engages when **all four original operator commands are equal** and
outside the throttle deadband. Its bounded left/right correction goes to all
four drive motors through the existing CAN `0x100`; no IMU data is sent over CAN
and no new throttle CAN protocol is needed. Keep the throttle Teensy firmware
from the rear-speed-balancing change installed. Flash the **master** to install
the restored behavior. Existing joystick commands and IMU telemetry remain compatible.

The driver requests an additional **Game Rotation Vector (`0x08`) at 100 Hz**
for control, while keeping the original `0x05` quaternion telemetry intact.
Game rotation uses accelerometer/gyro fusion without magnetic heading updates,
avoiding magnetic heading jumps near motor current wiring. It can drift over
long runs; see the [manufacturer datasheet, §2.2.2](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf).
The original acceleration/gyro/rotation reports still determine driver READY;
missing game rotation disables heading hold without disabling manual throttle.

Leave the joystick neutral and vehicle still after startup. Control requires
at least 500 ms of continuously fresh, valid game orientation and calibrated
gyro reports with SH-2 accuracy status >=2 (medium). This checks sensor-reported
calibration quality; it does not manufacture a calibration from a startup delay.
At the start of each straight forward or reverse run, the current heading becomes
the zero-error reference. Repeated equal-throttle messages and changes in throttle
magnitude keep that reference. Neutral, manual turning, or a change of drive
direction releases it; the next straight run captures a new reference. No
permanent sensor tare, magnetometer zero, or stored flash calibration is written.

Mount the IMU rigidly and flat. Any fixed rotation
within the mounting plane cancels when the heading reference is captured, so
sensor X need not point toward the front. The quaternion supplies world-up for
projecting gyro rate, including when sensor Z points downward. The controller
accepts either face of a flat mounting within 30 degrees of horizontal. Translation
away from the vehicle center does not change rigid-body yaw; acceleration is not
integrated into a position estimate. Secure the IMU rigidly to the chassis so
it cannot rotate independently. Moving it later within the same flat plane
requires a new neutral/start reference, not a center-of-rotation offset.

At the master's 20 ms command tick the heading error, yaw-rate damping, and a
bounded integral produce a differential correction. Defaults in
`HeadingHoldConfig`: `kp=180`, `ki=20`, `kd=40`, maximum correction 100 per-mille
and 25% of requested throttle, with a 200 per-mille/second correction slew limit.
A 0.5-degree proportional/integral error dead zone reduces chatter. The correction
does not reverse a wheel, drive past full PWM, or cross below the input deadband.
At the minimum drive command it has no steering authority. Positive trim subtracts
from signed left commands and adds to signed right commands **in both forward
and reverse**; reversing travel does not invert the yaw-controller sign.

Unequal rear commands deliberately bypass rear RPM balancing. This prevents
the equal-speed controller undoing an intentional heading correction. When
the master's outputs are equal, the rear encoder controller can balance them
normally. Thus heading hold can request different rear RPMs when needed to
counter the whole vehicle's yaw, including front-wheel/motor asymmetry. It does
not measure front wheel RPM or guarantee equal wheel speed during correction.

Stale data (>100 ms), low calibration status, invalid quaternion, excessive tilt,
excessive yaw rate (>3 rad/s), or an abrupt inconsistent orientation jump releases
the correction. Normal operator PWM passes through. Return to neutral before
heading hold can engage again after such a failure or if a straight run began
before the IMU was ready. E-stop and teleop watchdog still override motor output
with zero; the throttle Teensy still enforces heartbeat and command watchdogs. Heading hold
does not provide an obstacle stop or a mechanical steering lock. It holds yaw,
not lateral position, and cannot detect sideways sliding without a heading change.

**Commissioning:** first restrain the front steering with a mechanical fixture
and test with all wheels supported clear of the ground, e-stop accessible.
After the sensor is ready, request low equal forward throttle and rotate the
vehicle/IMU counterclockwise viewed from above while keeping the mount flat.
The left **signed** command should increase and the right decrease to request
clockwise correction. Verify the equivalent signed-command response in reverse.
If the physical response reinforces the turn, stop and check motor/wheel mapping;
`HeadingHoldConfig::steering_sign` can be set to `-1` if the physical yaw sign
requires inversion. Initial gains are model-tested and need vehicle tuning.
Start ground tests at low speed and check forward, reverse, deliberate turns,
neutral, sensor dropout, and e-stop. Heading hold cannot make loose wire/tape
steering restraint reliable.

Wire-only status `0x239` is emitted at 10 Hz: `state:u8`, `ready:u8`, `yaw:i16`
in milliradians, `target-minus-yaw:i16` in milliradians, and signed trim `i16`
in per-mille, all little-endian. States: 0 disabled, 1 idle, 2 holding, 3 manual
turn, 4 waiting for IMU, 5 neutral required. Yaw is in the game vector's arbitrary
world frame, not magnetic north; error is relative to the captured reference.
Older ROS/joystick software ignores this extra frame; the current bridge does
not publish it as a ROS topic. Existing `/carl/imu` remains the original magnetic
rotation-vector telemetry and is not the control heading.

## Host unit tests

From the repository root, run all eighteen actual-source host regressions with
undefined-behavior checks (requires `g++`; no boards or ROS needed):

```bash
python3 firmware/test/run_host_regressions.py
# Optional: also check memory access and leaks (requires an environment supporting ASan):
python3 firmware/test/run_host_regressions.py --address-sanitizer
```

The production MPU suites obtain their compiler flags from `[env:master]` in
`platformio.ini`, so they exercise the installed sensor path. The broad master
safety/framing suite now runs against both MPU and retained BNO implementations.
The MPU stress suite adds unknown/disconnected identities, failed configuration
readback, missing data, all-FF status, identity loss, reconnect/recovery, saturated
error counters, watchdog/e-stop behavior during faults, and timer rollover.
Its thirty-minute simulation checks 180,000 complete IMU/temperature samples,
90,000 scheduled commands, 36,000 heartbeats, and simultaneous 50 Hz steering
and encoder plus 20 Hz node status traffic. Real C++ output is decoded using
the production Python bridge to check units, sequence rollover, CRCs and framing.
No vehicle transport is opened by these host regressions.

The MPU heading suites additionally cover live stationary-data replay, both
mounting faces and travel directions, timestamp rollover, stationary-evidence
qualification, calibration/thermal/motion/freshness failures, finite run/calibration
lifetimes, framed commands, e-stop and watchdog overrides during correction,
and ten simulated minutes of active forward/reverse heading control. The latter
checks 30,000 scheduled commands and 12,000 on-time heartbeats. Simulated vehicle
convergence is model evidence, not physical vehicle validation.

The joystick-curve regression verifies quadratic forward/reverse light and intermediate
commands through the 75% joystick command limit against actual motor PWM,
applied-output telemetry, the existing 2% deadband, turning and immediate release.

The restored IMU suites exercise real SPI startup/configuration, report freshness,
sequence rollover, missing/stuck INT, corrupt MISO headers, sensor resets/recovery,
heading overrides, e-stop/watchdogs during faults, and ten simulated minutes of
100 Hz telemetry alongside 50 Hz steering feedback and command traffic. The
capture is also decoded by the production Python bridge to check units, quaternion
assembly, diagnostics, and CRCs across 60,000 snapshots. Heading simulations
cover forward/reverse, both flat mounting faces, limits, stale data, and interaction
with the current rear speed controller. Each host executable has a 60-second timeout.

Verification on 2026-11-05: all 15 host suites passed with address/leak and
undefined-behavior checks; 19 native protocol cases and 59 ROS bridge/teleop
tests plus 35 subtests passed. All three board targets built from a fresh
build directory. The resulting master binary is byte-for-byte identical to
the firmware already tested/flashed. A 180-second passive hardware capture
observed 18,030 IMU samples (100.16 Hz), zero sensor/CRC errors, no heartbeat
counter gaps, 9,000 steering and 9,000 encoder frames, and zero idle throttle
outputs throughout. Fault injection and motor-output checks were simulated;
loaded driving and IMU calibration were not performed. See the complete
[verification record](../docs/assets/master-firmware-verification-2026-11-05.json).

The native suite guards shared protocol definitions and wire framing. No hardware.

```bash
pio test -e native
```

The rear speed simulation covers both directions, swapped motor characteristics,
low/high throttle, output saturation, missing/stale counts, counter rollover,
and controller resets. The throttle integration test compiles the actual node
against GPIO/CAN/encoder doubles and checks corrected PWM, raw encoder telemetry,
applied-status values, and stop paths. Run from the repository root:

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/lib/common -Ifirmware/lib/carl_protocol \
  firmware/test/support/throttle/test_rear_speed_sync.cpp \
  -o /tmp/carl-rear-speed-tests
/tmp/carl-rear-speed-tests
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/test/support/throttle -Ifirmware/lib/carl_protocol \
  -Ifirmware/lib/common -Ifirmware/lib/heartbeat \
  firmware/test/support/throttle/test_throttle.cpp -o /tmp/carl-throttle-tests
/tmp/carl-throttle-tests
```

The physical pin-mapping regression runs the actual throttle node with motor
GPIO outputs feeding a motor/inertia model and encoder counts injected at
the installed **pin numbers**, rather than directly into named encoder objects.
It reproduces divergent speeds with the old crossed mapping and checks RPM
convergence in forward/reverse, low/high throttle, either motor being faster,
and raw CAN count labeling with the corrected map:

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/test/support/throttle -Ifirmware/lib/carl_protocol \
  -Ifirmware/lib/common -Ifirmware/lib/heartbeat \
  firmware/test/support/throttle/test_rear_mapping.cpp -o /tmp/carl-rear-mapping-tests
/tmp/carl-rear-mapping-tests
```

The master integration test checks unchanged forward/reverse and differential
commands, serial parsing, malformed input, e-stop/reset, watchdog expiry,
heartbeat flags and timing, and forwarding of throttle, encoder, and steering
telemetry. The idle serial stream contains only heartbeat frames.

## Layout

```
firmware/
├── platformio.ini       # one [env] per node
├── lib/                 # SHARED code — single source of truth across nodes
│   ├── carl_protocol/
│   │   ├── carl_protocol.h  # CAN IDs, timing, enums
│   │   ├── messages.h        # struct <-> CAN payload (de)serialization
│   │   └── wire.h            # serial framing (SOF + CRC8), ROS2 companion link
│   ├── heartbeat/       # HeartbeatTx producer + Watchdog consumer
│   └── common/          # little-endian byte helpers, clamp, map
├── master/   src/main.cpp     # NUCLEO-F767ZI — serial↔CAN proxy
├── throttle/ src/main.cpp     # Teensy 4.1 — motor drive + encoders
├── steering/ src/main.cpp     # Teensy 4.1 — safe-idle stub
└── test/                # host-run Unity tests
```

## Notes

- **Shared `lib/` is the point.** Every node includes `carl_protocol.h`, so
  senders and receivers cannot drift on IDs, timing, or byte layout. `wire.h`
  is the same discipline for the companion link.
- **Two chip families, one build system.** Master = STM32 (`platform = ststm32`),
  throttle & steering = Teensy 4.1 (`platform = teensy`).
- **CAN libraries:** Teensy nodes use the core-bundled `FlexCAN_T4`; the STM32
  master uses `pazi88/STM32_CAN`. The throttle node uses `paulstoffregen/Encoder`.
- **Boot/flash protocols differ per chip** — Teensy uses HalfKay (Teensy Loader;
  tap the button if it doesn't auto-reboot), the NUCLEO uses on-board ST-LINK
  (SWD). PlatformIO handles both via `-t upload`.
- **E-stop latches until explicitly reset.** An operator request with reason 1
  latches the master; reason 0 clears the latch after zeroing retained commands.
  Reset does not keep drive commands alive or restore the previous output.
  The physical e-stop remains independent. Zero output coasts the vehicle.

## Absolute steering encoder telemetry

The steering Teensy now reads the analog encoder on pin 14/A0 and publishes
50 Hz CAN feedback through the master to `/carl/steering_angle`. The CL57T
remains disabled. **Correct the 0–5 V output wiring before powering:** Teensy
4.1 inputs are not 5 V tolerant. See [wiring, calibration, deployment and
verification](../docs/steering-encoder.md).
