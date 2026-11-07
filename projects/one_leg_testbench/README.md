# One-leg testbench (MuJoCo + ROS 2)

Simulation and hardware testbench for the left-front (LF) leg, including work on two papers:

- 2017 inverse-kinematics paper: the IK in `one_leg_testbench/leg.py` (repo-corrected q1 equation, negative knee branch for the left leg).
- 2020 ground-compliance paper: Bezier swing (Table 1 control points) and sinusoidal stance in `one_leg_testbench/trajectory.py`.

## Firmware calibration

The testbench never redefines calibration: `tests/test_one_leg_testbench.py::FirmwareParity` parses
`firmware/one_left_leg_angle_control/one_left_leg_angle_control.ino` and fails if
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

In this primary ROS mode, `/walk_cmd` runs a moving Cometyang-style LF step: the foot moves
rearward during seven 0.36 s stance phases, then returns forward during the 0.36 s phase-8 swing
with 0.030 m triangular clearance. The full-robot Cometyang gait uses 0.050 m; the one-leg value
is reduced to satisfy the LF testbench joint limits. Stance/touchdown displacement uses
`/cmd_vel` `linear.x`; the LF adapter defaults to 0.04 m/s and caps speed at 0.08 m/s. At the
default speed, the nominal stance stroke is about 0.101 m. Since the mount stays fixed, this is an
in-place leg step, not whole-robot forward translation; the four-leg body-shift controller is not
simulated.
The previous Zhang Bezier/sinusoidal gait remains available on
`/one_leg_testbench/command` (`walk`/`stop`).

## **TERMINAL 1 — MuJoCo and RViz**

Start the suspended-leg MuJoCo viewer and Cometyang RViz together. `run_testbench` starts its
own motion controller, disables the full-robot simulator and preset LF controller, and uses a
world-fixed RViz view so the base-height transition is visible. Leave this terminal running:

```bash
cd ~/Documents/spotmicro_project/mike_mujoco_ws
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch spot_micro_mujoco_sim front_left_leg_launch.py run_testbench:=true
```

## **TERMINAL 2 — ESP32 bridge**

For the physical leg, start the bridge in a fresh terminal without the virtual environment and
leave it running. The default serial port is `/dev/ttyS0` at 115200 baud. This setup uses the
ESP32 commissioning firmware (`set <channel> <usec>`); without `--enable`, the bridge only
prints commands.

```bash
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
python3 ~/Documents/spotmicro_project/projects/one_leg_testbench/esp32_serial_bridge.py --enable
```

The bridge forwards the simulated front-left `/joint_states` to the ESP32 and may immediately
command the current pose. Keep the leg clear and stay near the servo power switch; cut power if
anything strains or hits a limit. Ctrl+C normally sends `offall`, but a serial failure can
prevent it from arriving. The current pulse calibration and sit knee clamp (1100 microseconds)
can limit physical motion, so verify the real leg against the simulation.

For simulation only, omit Terminal 2.

## **TERMINAL 3 — ROS commands**

In a fresh terminal, source ROS:

```bash
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
```

Then send one command at a time. `idle` is the sit pose; sending stand or idle exits the walk
cycle. Walk can be entered from stand; return to stand and wait before starting it from sit:

```bash
# Stand
ros2 topic pub --once /stand_cmd std_msgs/msg/Bool "{data: true}"

# Sit
ros2 topic pub --once /idle_cmd std_msgs/msg/Bool "{data: true}"

# Walk
ros2 topic pub --once /walk_cmd std_msgs/msg/Bool "{data: true}"

# Stop walking and return to stand
ros2 topic pub --once /stand_cmd std_msgs/msg/Bool "{data: true}"
```

The current testbench ignores `/walk_cmd` with `false`; use `/stand_cmd` with `true` to stop.

The LF walk defaults to `linear.x = 0.04 m/s`, so the command above steps without extra setup.
To change the stride speed, publish the same Cometyang velocity command (limited to +/-0.08 m/s
for this one-leg IK test):

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
	"{linear: {x: 0.06, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

To select the Cometyang repo's lift-only phase pattern instead, publish:

```bash
ros2 topic pub --once /cometyang_walk_cmd std_msgs/msg/Bool "{data: true}"
```

`/walk_cmd` is the moving Cometyang-style LF step; `/cometyang_walk_cmd` is its lift-only phase
pattern; `/one_leg_testbench/command` with `walk`/`stop` is the Zhang gait. Do not start
`one_leg_testbench.ros_node` in this combined mode; the MuJoCo testbench handles both gait topics.

To command the angles directly using the Arduino sketch's serial format, publish
`shoulder,hip,knee` in degrees. For example, the configured neutral reference is:

```bash
ros2 topic pub --once /front_left_leg/angle_command std_msgs/msg/String \
	"{data: '-7.6,38.6,-82.8'}"
```

The ROS node checks values against the firmware/simulator joint limits and slews each target by
1 degree per 20 ms, matching the sketch's gradual `moveJoint` rate. The node itself publishes
simulated joint states; the optional bridge forwards them to the ESP32.

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
- The physical bridge mirrors commanded simulation joint states, not measured servo positions;
  calibration and tracking must be checked on the actual leg.
