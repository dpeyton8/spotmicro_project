# One-leg testbench (MuJoCo + ROS 2)

Simulation workspace used to confirm the two papers on the left-front (LF) leg before any hardware motion:

- 2017 inverse-kinematics paper: the IK in `one_leg_testbench/leg.py` (repo-corrected q1 equation, negative knee branch for the left leg).
- 2020 ground-compliance paper: Bezier swing (Table 1 control points) and sinusoidal stance in `one_leg_testbench/trajectory.py`.

## How this differs from `projects/one_leg`

| | `projects/one_leg` (prior work) | `projects/one_leg_testbench` (this) |
|---|---|---|
| Role | Hardware bring-up and confirmed firmware | Paper-confirmation testbench |
| Contents | ESP32 sketches, HTML kinematic preview, right-leg stance script, ROS front-left demo | Left-leg IK, paper trajectories, MuJoCo rig, ROS node, tests |
| Source of truth | Firmware in `firmware/` | Must match `one_left_leg_angle_control.ino` |

The testbench never redefines calibration: `tests/test_one_leg_testbench.py::FirmwareParity` parses
`../one_leg/firmware/one_left_leg_angle_control/one_left_leg_angle_control.ino` and fails if
the joint table (channels, center PWM, ranges, direction, center angles), `MAX_JOINT_ANGLE_DEG`
or `PWM_MIN/MAX` differ from `leg.py`.

## Scope

- Now: left-front leg only.
- Next: right leg (needs RF calibration: RF_1 center -5.4 deg / direction -1, RF_2 -27.6, RF_3 88.2; mirrored signs and knee branch).
- Then: all four legs.

## Use

```bash
cd projects/one_leg_testbench
../../mike_mujoco_ws/.venv/bin/python -m pytest tests -q
../../mike_mujoco_ws/.venv/bin/python -m one_leg_testbench.plan     # angles + predicted PWM per step
../../mike_mujoco_ws/.venv/bin/python -m one_leg_testbench.sim [--viewer]
```

`plan` prints lines that can be pasted into the sketch's Serial Monitor, followed by the PWM
values `leg.firmware_pwm()` predicts, so they can be compared with the sketch's own output.

ROS 2 (sim node running with `front_left_only_mode:=true`):

```bash
python3 -m one_leg_testbench.ros_node
ros2 topic pub --once /one_leg_testbench/command std_msgs/String "{data: walk}"
```

## Limits

- Position-command servos: no force, torque or encoder feedback, so the paper's impedance control is not reproduced.
- The MuJoCo mount is fixed (mocap), standing in for a gantry, not the paper's slide rail.
- Swing tracking error (~14 mm RMS) is servo lag in the model and is deliberately not tuned away.
- Nothing here has been run on the physical leg.
