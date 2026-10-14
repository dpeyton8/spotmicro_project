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

### MuJoCo testbench motion in Cometyang RViz

This runs physics and the suspended LF leg from `one_leg_testbench.sim`, while RViz displays the
matching full Cometyang robot. The other nine joints are published at zero. It does not run the
full-robot MuJoCo physics node. In ROS mode it starts in the stand pose and accepts the usual
Cometyang Bool commands: `/stand_cmd`, `/idle_cmd` (sit), and `/walk_cmd`.

For sit/stand, the testbench applies the Cometyang rate-limited first-order transition to a single
fixture mount and solves LF IK at each update while holding the LF toe at its initial world point.
The mount targets are `(0, 0, 0.173 m)` for stand and `(-0.050, 0, 0.101 m)` for sit: these use
the Cometyang 0.155/0.083 m body heights plus the testbench toe radius. This is a one-leg planted-
toe demonstration, not four-foot CoM stability or a full-body physics simulation.

In Terminal 1, start the suspended-leg MuJoCo viewer and Cometyang RViz together. `run_testbench`
automatically disables the full-robot simulator and preset LF controller, and uses a world-fixed
RViz view so the base-height transition is visible:

```bash
cd ~/Documents/spotmicro_project/mike_mujoco_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --packages-select spot_micro_mujoco_sim --symlink-install
source install/setup.bash
ros2 launch spot_micro_mujoco_sim front_left_leg_launch.py \
  run_testbench:=true
```

In Terminal 2, send one command at a time. `idle` is the sit pose; sending stand or idle exits
the walk cycle. Walk can be entered from stand; return to stand before starting it from sit:

```bash
ros2 topic pub --once /stand_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /idle_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /walk_cmd std_msgs/msg/Bool "{data: true}"
```

To alternate continuously between sit and stand using the selected sit/stand mathematics:

```bash
ros2 topic pub --once /sitstand_cycle_cmd std_msgs/msg/Bool "{data: true}"
```

The cycle runs until `/stand_cmd`, `/idle_cmd`, or `/walk_cmd` receives `true`. It can also be
stopped and returned to stand with:

```bash
ros2 topic pub --once /sitstand_cycle_cmd std_msgs/msg/Bool "{data: false}"
```

Do not start `one_leg_testbench.ros_node` in this mode; it is a separate gait publisher.

#### Two-track mathematics comparison

The integrated testbench can now hold the SpotMicro model, IK, actuator gains, endpoints, and
logging constant while selecting the trajectory mathematics independently:

| Motion | `repo` | Article method |
|---|---|---|
| Sit/stand | Cometyang rate-limited first-order filter | `hao`: Hao et al. Eq. 9 cycloid |
| Walk | Cometyang/Mike linear stance + triangular swing clearance | `zhang`: Zhang et al. Eq. 18 Bezier swing + Eq. 19 stance |

Select the initial methods when launching (defaults shown):

```bash
ros2 launch spot_micro_mujoco_sim front_left_leg_launch.py \
  run_testbench:=true sitstand_method:=repo walk_method:=zhang
```

They can also be changed while the testbench is running:

```bash
ros2 topic pub --once /one_leg_testbench/sitstand_math std_msgs/msg/String "{data: repo}"
ros2 topic pub --once /one_leg_testbench/sitstand_math std_msgs/msg/String "{data: hao}"
ros2 topic pub --once /one_leg_testbench/walk_math std_msgs/msg/String "{data: repo}"
ros2 topic pub --once /one_leg_testbench/walk_math std_msgs/msg/String "{data: zhang}"
```

After selecting a method, issue the usual `/stand_cmd`, `/idle_cmd`, or `/walk_cmd`. Sit/stand
uses identical stand and sit mount targets in both methods, holds the same toe world point, and
uses the same LF IK. Walk uses the same approximate stride, phase durations, IK, MuJoCo model,
and actuators. The repo mode isolates the LF portion of the eight-phase gait; it does not claim
to reproduce the missing three-leg support polygon on the one-leg fixture.

Run a headless comparison to produce CSV logs, metrics, and presentation-ready PNG plots:

```bash
cd ~/Documents/spotmicro_project/projects/one_leg_testbench
../../mike_mujoco_ws/.venv/bin/python -m one_leg_testbench.compare --cycles 2
```

Outputs are written to `outputs/two_track_comparison/`:

- `sitstand_comparison.csv` and `sitstand_mount_height.png`
- `walk_comparison.csv` and `walk_trajectory_comparison.png`
- `summary.txt`

The walk CSV includes desired and simulated toe position, commanded and simulated `q1/q2/q3`,
joint velocity, phase, contact, normal force, tracking error, and joint-limit margin. These are
simulation results, not physical-leg results.

### Alternative: Paper Gait on Cometyang MuJoCo

This alternative uses the Cometyang MuJoCo physics model and RViz; it does not run the suspended
`one_leg_testbench.sim` rig. Start `front_left_leg_launch.py` with `use_mujoco_sim:=true` and
`start_builtin_front_left_controller:=false`, then run `one_leg_testbench.ros_node` and publish
`walk`/`stop` on `/one_leg_testbench/command`:

```bash
cd ~/Documents/spotmicro_project/mike_mujoco_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --packages-select spot_micro_mujoco_sim --symlink-install
source install/setup.bash
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
ros2 launch spot_micro_mujoco_sim front_left_leg_launch.py \
	use_mujoco_viewer:=true start_builtin_front_left_controller:=false
```

In another sourced terminal, run the paper-gait node from this project:

```bash
cd ~/Documents/spotmicro_project
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source mike_mujoco_ws/.venv/bin/activate
source mike_mujoco_ws/install/setup.bash
export PYTHONPATH="$PWD/projects/one_leg_testbench:$PWD/mike_mujoco_ws/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
python -m one_leg_testbench.ros_node
```

In a third sourced terminal, command the paper gait:

```bash
ros2 topic pub --once /one_leg_testbench/command std_msgs/String "{data: walk}"
ros2 topic pub --once /one_leg_testbench/command std_msgs/String "{data: stop}"
```

The standalone `one_leg_testbench.sim` and `one_leg_testbench.poses` modes use the suspended
fixed-mount leg rig in `one_leg_testbench/model/left_leg_rig.xml`.

## Limits

- Position-command servos: no force, torque or encoder feedback, so the paper's impedance control is not reproduced.
- The MuJoCo mount is fixed (mocap), standing in for a gantry, not the paper's slide rail.
- Swing tracking error (~14 mm RMS) is servo lag in the model and is deliberately not tuned away.
- Nothing here has been run on the physical leg.
