# Absolute steering encoder telemetry

The steering Teensy 4.1 reads the analog encoder on **pin 14 / A0** at 50 Hz
with 12-bit ADC resolution and 16-reading hardware averaging. CAN `0x221`
passes through the NUCLEO master to ROS 2 topic **`/carl/steering_angle`**
(`carl_msgs/msg/CarlSteeringAngle`). The CL57T remains disabled; encoder data
does not participate in throttle, heading hold, or steering actuation.

## Correct the wiring before powering or flashing

The installed sensor was identified as allegedly **P3022-V0**. This implementation
targets the **three-terminal analog, 0–5 V, full-360-degree variant**. The exact
V0 suffix has not been confirmed from an authoritative datasheet. Check the
label and measure its output; a PWM, serial, partial-angle, or other-output
variant needs a different reader or scaling.

**Do not connect encoder OUT directly to pin 14.** Teensy 4.1 analog and digital
inputs are limited to 3.3 V. Software cannot protect the input from 5 V or repair
overvoltage damage. Keep the sensor disconnected from pin 14 until the divider
is installed and checked with a meter.

Use this divider (1% resistors) with a common ground:

```text
Encoder VCC -------------------------- 5 V
Encoder GND -------------------------- Teensy GND
Encoder OUT ---- 10 kohm ----+--------- Teensy 14 / A0
                            |
                         10 kohm
                            |
                           GND
```

At 5 V OUT the ADC input is 2.5 V; at a 5.5 V supply/maximum output it is
2.75 V nominal. Verify the pin-side voltage stays below 3.3 V across the full
usable rotation **before** connecting it to the Teensy. The divider presents
a nominal 20 kohm load to the sensor. Confirm the actual sensor permits this
load. Power the steering Teensy and its CAN transceiver, retain the existing
CAN wiring/termination, and keep the CL57T disabled.

References: [PJRC input and ADC specifications](https://www.pjrc.com/store/teensy41.html),
[CALT P3022 analog family](https://caltsensor.com/customized-hall-angle-sensor-p3022/),
[DFRobot 360-degree analog sensor specifications](https://www.dfrobot.com/product-1553.html).
The CALT example uses a customized 90-degree span; it does not establish the
span of the installed V0. The firmware publishes ADC counts, leaving the assumed
0–5 V/full-turn conversion visible and configurable in the bridge.

## Topic fields and interpretation

| Field | Meaning |
| --- | --- |
| `header.stamp` | Bridge reception time; for a stale event, detection time |
| `header.frame_id` | `steering_encoder_link` |
| `raw_adc` | Native ADC count, 0–4095 |
| `sequence` | Sample counter, wrapping 65535 to 0 |
| `sensor_voltage` | Reconstructed voltage before the divider |
| `absolute_angle_rad` | Sensor shaft angle, 0–2*pi, available before mounting calibration |
| `angle_rad` | Centered steering angle, positive left; NaN until calibrated |
| `valid` | Electrical plausibility and fresh communication; not sensor-presence proof |
| `calibrated` | Operator explicitly enabled mounting calibration |
| `status` | 1=sample valid, 2=ADC low rail, 4=ADC high rail, 8=voltage out of range, 128=link stale |

Exactly 0 or 4095 counts is conservatively invalid. A legitimate electrical
endpoint can therefore be flagged invalid, as can an unplugged or shorted
wire. Valid intermediate voltage cannot prove that an analog encoder is
connected, powered correctly, mechanically coupled, or moving: test physical
rotation. Unplugged wiring can also pick up noise. No analog-only software can
reliably distinguish all these cases.

Malformed frames are ignored. Repeated sequence numbers do not refresh the
sample. After 250 ms without a new sample, the bridge publishes one invalid
`LINK_STALE` event with NaN voltage/angles, including when the board is absent
at startup. Fresh data automatically resumes publication. The other topics
and drive callbacks continue while steering data is missing or invalid.

## Build, install, and inspect

From the repository root, with the matching ROS 2 installation sourced:

```bash
python3 firmware/test/run_host_regressions.py
cd firmware
pio run -e master -e throttle -e steering
cd ..
colcon build --base-paths ros2_ws/src --symlink-install
source install/setup.bash
python3 -m pytest ros2_ws/src/carl_bridge/test ros2_ws/src/carl_teleop/test
```

After correcting the wiring, flash **both master and steering** firmware
using the existing board-specific upload process. The throttle firmware is
unchanged by this feature. Deploy/rebuild `carl_msgs` and `carl_bridge` together;
all consumers of the new message need the rebuilt `carl_msgs` overlay.

Start the normal bridge/bringup with the correct NUCLEO serial port, then:

```bash
ros2 topic info /carl/steering_angle
ros2 topic echo /carl/steering_angle
ros2 topic hz /carl/steering_angle
```

With the vehicle stationary and propulsion disabled, rotate the steering by
hand within its mechanical limits. Confirm `raw_adc`, voltage, and absolute
angle change smoothly and consistently; sequence advances, and nominal rate
is 50 Hz. With the default divider, a 2.5 V sensor output yields about
**1551 counts and pi radians**. Do not rotate past mechanical stops to test
the full electrical range. Record the normal heartbeat, throttle-status,
rear-encoder, and IMU topics alongside steering to check the vehicle after
deployment. Disconnect the steering CAN/data path only with the vehicle
stationary to verify invalid/stale indications; restore it and check recovery.

## Mounting calibration for GUI and telemetry consumers

Defaults in `ros2_ws/src/carl_bridge/config/bridge.yaml` assume a 3.3 V ADC
reference, a 10k/10k divider (`divider_gain: 2.0`), and a nominal 5.0 V full-scale
sensor output. Measure and adjust these values for the installed hardware.
Mounting calibration defaults to **false**, so a GUI can immediately display
absolute shaft angle and raw signal without mislabeling them as centered wheel
angle. Restart the bridge after editing these startup parameters.

1. Place the wheels straight ahead. Record `absolute_angle_rad` and set
   `steering.zero_offset_rad` to that value.
2. Measure the sensor-to-steering mechanical ratio. Set `steering.ratio` to
   **sensor radians per steering radian**, a positive number. A shaft-angle
   measurement is not automatically a road-wheel angle or an Ackermann model.
3. Set `steering.direction` to `1.0` or `-1.0` so physical left steering is
   positive, then set `steering.calibrated: true` and restart the bridge.
4. Verify zero, sign, and magnitude at several known positions on both sides.

Conversion: `sensor_voltage = raw_adc / 4095 * adc_reference_voltage * divider_gain`;
`absolute_angle_rad = sensor_voltage / sensor_full_scale_voltage * 2*pi`;
`angle_rad = direction * wrap_to_minus_pi_pi(absolute_angle_rad - zero_offset_rad) / ratio`.
Up to 2% full-scale voltage tolerance is clamped at the 360-degree endpoint;
larger excursions are invalid. This single-turn sensor cannot disambiguate
multiple turns. Ensure the usable mechanical range stays within a single
centered turn of the sensor, or use suitable gearing/a different sensor.

Consumers should check `valid`, `calibrated`, and receipt age before using
`angle_rad`; invalid or uncalibrated angles are NaN. Actual changes to the GUI
and joystick applications are a separate integration step.

## Validation scope

Host regressions compile the production steering and master source with
hardware doubles. They cover ADC configuration and scheduling, CAN/serial
framing, sample-counter and millis rollover, rails, forwarding, and keeping
STEP/ENA low while messages arrive or the heartbeat disappears. ROS tests
cover conversion, calibration, malformed payloads, duplicate samples,
timeouts/recovery, and the real custom message fields. Existing drive, IMU,
watchdog, and e-stop regressions remain required.

Builds and simulated tests do not validate the installed sensor identity,
divider, ADC integrity, CAN wiring, angle calibration, or real-time behavior
under vehicle load. A vehicle bench check is required before operational use.
