# Project map

## Preserved physical reference

`physical/spotMicro` is the untouched `mike4192/spotMicro` master repository. It is the reference for the physical build, controller, servo calibration, and KDY0523 robot. Do not build, patch, or clean it as part of simulation setup.

## ROS 2 and MuJoCo workspace

`mike_mujoco_ws/src/spotMicro` is `cometyang/spotMicro` on `feature/mujoco-simulation`.

Relevant ROS 2 packages:

- `i2cpwm_board`: ROS 2 message/service definitions used by both the controller and simulator. It is not the physical driver.
- `spot_micro_mujoco_sim`: MuJoCo model and ROS bridge. Publishes `joint_states` and TF; subscribes to `servos_proportional` and `servos_absolute`; provides `config_servos`.
- `spot_micro_motion_cmd`: C++ motion state machine. Receives `/stand_cmd`, `/idle_cmd`, `/walk_cmd`, `/angle_cmd`, and `/cmd_vel`; publishes servo commands.
- `spot_micro_keyboard_command`: interactive command publisher.
- `spot_micro_rviz`: URDF/xacro and RViz configurations.
- `spot_micro_launch`: combined launch files, some of which include unrelated physical SLAM hardware and are not required for the initial simulation.
- `lcd_monitor`, `servo_move_keyboard`, `spot_micro_plot`: optional/support packages.

Important files:

- `spot_micro_mujoco_sim/models/spot_micro_sim.xml`: MJCF, 12 hinge joints plus a free root and 12 position actuators.
- `spot_micro_mujoco_sim/launch/mujoco_sim_launch.py`: simulator, robot state publisher, and optional RViz (`use_rviz`).
- `spot_micro_mujoco_sim/spot_micro_mujoco_sim/mujoco_sim_node.py`: ROS/MuJoCo bridge.
- `spot_micro_motion_cmd/launch/motion_cmd_launch.py`: motion controller launcher; `run_standalone:=true` prevents `i2cpwm_board_node` from starting.
- `spot_micro_keyboard_command/launch/keyboard_command_launch.py`: keyboard controller; optional plotting and RViz.

## Obsolete submodule

`ros-i2cpwmboard` is a ROS 1 Catkin physical PCA9685 driver. It has the same package name as the ROS 2 interface package and must be ignored by colcon later, without deletion or modification of Git history. Its committed URL is GitLab, while this checkout has a local URL override to `https://github.com/EricWiener/ros-i2cpwmboards.git`; it is nevertheless fully initialized at `c73f89a8687e2a9c7698522d6015e06f1228050e`.

