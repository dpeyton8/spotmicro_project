# Fresh workspace audit

Audit performed read-only on 2026-07-19 before any installation or host edits.

## Repository verification

| Repository | Origin | Branch | Full commit | Status |
|---|---|---|---|---|
| `physical/spotMicro` | `https://github.com/mike4192/spotMicro.git` | `master` | `2a34f5d303dff91b62180031b31ef512a672f3c3` | Clean; tracks `origin/master` |
| `mike_mujoco_ws/src/spotMicro` | `https://github.com/cometyang/spotMicro.git` | `feature/mujoco-simulation` | `abb9eeb9d353d1643b68027d02e43a8c65ff9a5b` | Clean; tracks matching origin branch |

Both repositories have all three initialized submodules. The physical checkout has no modifications, including no dirty submodule marker. No repository was changed during Phases 1–3.

Required MuJoCo files are present:

- `spot_micro_mujoco_sim/models/spot_micro_sim.xml` (8038 bytes)
- `spot_micro_mujoco_sim/launch/mujoco_sim_launch.py` (2383 bytes)
- `spot_micro_mujoco_sim/spot_micro_mujoco_sim/mujoco_sim_node.py` (18965 bytes)

## Host audit

- OS: Ubuntu 22.04.5 LTS (Jammy), `x86_64`.
- Disk: root/workspace filesystem 96 GiB total, 78 GiB used, 13 GiB available (86% used). Installation size must be reviewed before Gate 2.
- ROS: only `/opt/ros/foxy` exists. No `ROS_DISTRO` or other ROS environment variable was exported in the audit shell.
- Packages: 262 installed `ros-foxy-*` packages and zero installed `ros-humble-*` packages.
- Shell startup: the sole ROS source line in `~/.bashrc` is commented out (`source /opt/ros/foxy/setup.bash`), so it does not currently contaminate new shells.
- Apt: enabled `/etc/apt/sources.list.d/ros2.list` incorrectly targets `focal main`. This is unsafe on Jammy. `.distUpgrade` and `.save` files also contain Focal text but apt does not normally load those suffixes. Non-ROS legacy Focal PPA backup files exist; the active `.list` versions observed are commented. Ubuntu base entries are Jammy.
- ROS key: `/usr/share/keyrings/ros-archive-keyring.gpg` exists. The `ros2-apt-source` package is not installed.
- rosdep: executable installed, but system sources are not initialized and no user cache exists.
- colcon: `/usr/bin/colcon`, `python3-colcon-core 0.21.0+upstream-1`, and common extensions are installed. Its network-based `version-check` could not reach its server; local package discovery works.
- Python: 3.10.12. No workspace venv. No `mujoco` distribution or `simulate` executable. System NumPy is 1.21.5; user-site Pillow is 12.3.0.

## Branch inspection findings

The MJCF statically defines exactly 12 `<position>` actuators, in front-left, front-right, rear-left, rear-right order, with shoulder, leg, foot within each leg. The bridge maps Mike servo numbers to those named joints rather than relying on raw actuator array ordering.

The model cannot be compiled against the current official MuJoCo on this host yet because MuJoCo is not installed. Static XML inspection found standard MJCF constructs, but this is not a substitute for the required compile and 1000-step test.

The simulation node imports `mujoco`, `numpy`, and `PIL.Image` at module import time. Therefore MuJoCo, NumPy, and Pillow are required even for a non-video launch. `setup.py` declares only `setuptools`; these Python dependencies are missing from package metadata. `package.xml` declares ROS dependencies, including `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `i2cpwm_board`, robot/joint state publishers, and RViz. `xacro` is used by the launch file but is not declared by `spot_micro_mujoco_sim` itself (it is declared in `spot_micro_rviz`). Video mode also invokes `ffmpeg`, which is not declared.

The controller uses `i2cpwm_board/msg/Servo`, `ServoArray`, `ServoConfig`, and `i2cpwm_board/srv/ServosConfig`. Its primary command topics are `/stand_cmd`, `/idle_cmd`, `/walk_cmd`, `/angle_cmd`, and `/cmd_vel`; output/service names are `servos_proportional`, `servos_absolute`, and `config_servos`. The simulator publishes `joint_states` plus transforms through `TransformBroadcaster` (the `/tf` topic).

`spot_micro_motion_cmd/launch/motion_cmd_launch.py run_standalone:=true` applies an `UnlessCondition` to the physical `i2cpwm_board_node`, preventing it from launching. `run_lcd` defaults false and should remain false.

## Problems requiring gated fixes

1. An enabled Focal ROS apt repository conflicts with the Jammy host.
2. A large Foxy installation remains; it must not be sourced or mixed with Humble. Removal is not necessary for the first gate and would require its own explicit approval.
3. Humble is absent, and only 13 GiB is currently free.
4. rosdep is uninitialized.
5. MuJoCo and the isolated venv are absent.
6. Python runtime dependencies are incomplete in package metadata.
7. The obsolete ROS 1 submodule creates a duplicate `i2cpwm_board` package during colcon discovery and must later be ignored non-destructively.
8. No build, MJCF compile/step test, ROS launch, topic, TF, RViz, stand, or walking test has yet run.

## Gate 1 command block (not executed)

This first gate makes one targeted correction and preserves a dated rollback copy:

```bash
sudo cp -a /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.pre-mike-mujoco-20260719
sudo sed -i 's#ros2/ubuntu focal main#ros2/ubuntu jammy main#' /etc/apt/sources.list.d/ros2.list
grep -nH '^[[:space:]]*deb .*packages.ros.org/ros2/ubuntu' /etc/apt/sources.list.d/ros2.list
```

It changes only the active ROS 2 suite from Focal to Jammy. It does not run `apt update`, install/remove packages, source ROS, edit `.bashrc`, or initialize rosdep. Gate 2 will first run apt simulation and review space before any Humble install.

Rollback (also privileged; run only if needed and approved):

```bash
sudo cp -a /etc/apt/sources.list.d/ros2.list.pre-mike-mujoco-20260719 /etc/apt/sources.list.d/ros2.list
grep -nH '^[[:space:]]*deb .*packages.ros.org/ros2/ubuntu' /etc/apt/sources.list.d/ros2.list
```

