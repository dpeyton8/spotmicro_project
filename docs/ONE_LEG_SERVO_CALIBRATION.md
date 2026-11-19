# One-leg servo calibration (horns already glued into the printed legs)

The horn is fixed to the printed leg, so each servo is centered electrically first
and then pushed onto the horn with the leg held at its neutral pose. Servos stay
out of the leg until they have been centered.

Sketches:

- `projects/one_leg_testbench/firmware/one_leg_streaming/one_left_leg_streaming/one_left_leg_streaming.ino`
  is the firmware to flash for calibration and the ROS serial bridge.
- `projects/one_leg_testbench/firmware/one_left_leg_angle_streaming/one_left_leg_angle_streaming.ino`
  is an alternative direct-degree streaming experiment without `off` commands.
- `projects/one_leg_testbench/firmware/one_left_leg_angle_control/one_left_leg_angle_control.ino`
  is the blocking manual Serial Monitor angle test.

Channels: 0 = shoulder (q1), 1 = hip (q2), 2 = knee (q3).

## Before powering anything

- Check the servo's rated voltage and set the bench supply to it. Do not assume 5 V.
- Current limit 1-2 A.
- Servo power goes to the PCA9685 `V+` and `GND`; the ESP32 must share ground.
  Never power servos from the ESP32 3V3 or the PCA9685 logic VCC.
- Wiring: ESP32 3V3 -> VCC, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL.

## Procedure

1. Flash `one_leg_streaming/one_left_leg_streaming/one_left_leg_streaming.ino`. Open Serial Monitor at 115200
   baud. Outputs start OFF; this firmware has no boot/default joint angles.
2. Work on one servo at a time, loose on the bench, not attached to the leg.
   Plug it into channel 0, then repeat for channels 1 and 2.
3. Send `center 0` (then `center 1`, `center 2`). The servo goes to its 1500 us center
   and holds. Do not turn the shaft while it is powered.
4. Hold the leg at the pose that goes with PWM 306 on every joint. In the angle sketch,
   PWM 306 corresponds to shoulder -7.6, hip 38.6, knee -82.8 degrees. To see that pose,
   run the testbench `poses` command (see below); its first frame is the default position.
5. Push the servo spline into the glued horn at that pose and fasten the servo to the leg.
   If the teeth do not line up exactly, choose the nearest tooth. The remaining error
   (up to roughly 7 degrees each side for a 25-tooth spline) is trimmed in the next step.
6. Trim the center: with the servo mounted, use `set <ch> <usec>` in small steps to move
   the joint to the exact neutral pose. Record the pulse that gives it.
7. Find the stops with small `set` steps. Never push into a mechanical stop. Record the
   safe minimum, maximum, and direction (whether a larger pulse moves the joint up or down).
8. Use `off <ch>` or `offall` whenever the leg is moved by hand.
9. Record center pulse, minimum, maximum and direction for each joint.

Commands: `help`, `center <ch>`, `set <ch> <usec>` (500-2500 us commissioning
envelope), `off <ch>`, `offall`. This broad envelope exists to discover the real
limits and is **not a safe operating range** for the assembled leg. Approach an
unknown endpoint in small increments, one unloaded joint at a time, and replace the
broad values with measured per-joint limits before normal or unattended operation.

## Boot pose versus calibration neutral

The bridge firmware starts with every output OFF. After the ROS bridge connects, the first pose
comes from `/joint_states`; the normal testbench startup pose is stand `0.0,50.1,-98.7` degrees.
The separate calibration-neutral reference is `-7.6,38.6,-82.8` degrees.

## Check against the manual angle sketch

The bridge streaming sketch does not call `setOscillatorFrequency(27000000)`; the manual angle sketch does.
`center` (1500 us) and the angle sketch's neutral (PWM count 306) can therefore land on slightly
different shaft positions (calculated about 0.1 ms apart if the chip runs at 25 MHz; not measured).
After mounting, flash the angle sketch, let it go to neutral, and trim again if the leg is off.

## After measuring

The angle sketch's values (center PWM 306, ranges 389/397/387, direction 1) come from the
ROS config for Mike's servos. If yours differ, update the `Joint leg[]` table in the angle
sketch and the matching constants in `projects/one_leg_testbench/one_leg_control/leg.py`.
The testbench parity tests (`pytest tests -q`) fail if the two disagree.

## Testbench commands (simulation only; they do not talk to the ESP32)

Run from `projects/one_leg_testbench`, with `PY=../../mike_mujoco_ws/.venv/bin/python`:

```bash
$PY -m pytest tests -q
$PY -m one_leg_control.plan --points 25
$PY -m one_leg_control.sim --viewer
$PY -m one_leg_control.poses stand-sit --viewer
```

To move the real leg, paste a `shoulder,hip,knee` line into the Arduino Serial Monitor
with the angle sketch flashed (for example stand `0.0,50.1,-98.7`) and compare the PWM it
prints with the testbench output. Keep the leg suspended with the toe off the ground for
the first run. Note that `moveJoint()` in the sketch always restarts from the center
angle, so the real leg passes through the default position between commands.

The bridge firmware and one-leg commands have been exercised on the physical leg; the broad
commissioning endpoints still require measured per-joint safety limits.
