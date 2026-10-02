# ESP32/PCA9685 physical test notes

The firmware contains the LF calibration/commissioning sketches and a separate
RF angle-control sketch. Mount the test leg in a rigid fixture. The manual
commissioning sketch is not the MuJoCo stance controller and does not perform
an automatic sweep.

All three-motor sketches use the same PCA9685 order:

| Channel | Joint |
|---:|---|
| 0 | Shoulder / q1 |
| 1 | Hip pitch / q2 |
| 2 | Knee / q3 |

Sketches have different purposes:

- `one_left_leg_servo_test.ino`: manually commands one channel in microseconds;
   use it first to find center and conservative limits, with horns disconnected.
- `one_left_leg_angle_control/one_left_leg_angle_control.ino`: accepts
   `shoulder,hip,knee` angles with LF reference values.
- `esp32_one_left_leg/esp32_one_left_leg.ino`: automatically sweeps each LF
   joint by 25 degrees around its configured center; use only after calibration.
- `one_right_leg_angle_control/one_right_leg_angle_control.ino`: accepts
   `shoulder,hip,knee` angles with RF reference values.

The RF and LF angle-control sketches share channel order, but have different
center-angle, range, and direction references. Neither reference table is a
substitute for calibration of your actual MG996R servos.

## Connections

| ESP32 | PCA9685 |
|---|---|
| 3V3 | VCC |
| GND | GND |
| GPIO21 | SDA |
| GPIO22 | SCL |

Connect an external servo supply to PCA9685 `V+` and `GND`. The external
supply and ESP32 must share ground. Never power a servo from ESP32 3V3 or from
PCA9685 logic VCC.

Temporary channel assignment:

- channel 0: shoulder side swing (`q1`)
- channel 1: upper leg/hip pitch (`q2`)
- channel 2: knee (`q3`)

## First use

1. Disconnect all horns from the leg.
2. Connect only one servo to channel 0.
3. Apply logic power, then current-limited servo power.
4. Upload the sketch and open Serial Monitor at 115200 baud.
5. Send `center 0`; install the horn at the documented mechanical-neutral
   orientation only after motion stops.
6. Find conservative limits with small pulse changes. Never force the shaft
   against a stop.
7. Repeat separately for channels 1 and 2.
8. Record center, minimum, maximum, and direction before assembling the leg.

Commands:

```text
help
center 0
set 0 1450
off 0
offall
```

The initial 1100--1900 microsecond software bounds are deliberately
conservative, but they are not guaranteed safe for an assembled joint.

