# SpotMicro Project — Member Handoff

## Purpose

This document onboards new members to the SpotMicro project. The repository
contains the ROS 2 Humble and MuJoCo simulation workspace, inverse-kinematics
implementations and verification, the preserved physical-robot reference, and
physical-build planning documents.

## Prerequisites — read before starting

Choose the environment based on the work you will do:

- **Windows only:** suitable for cloning the repository, VS Code, code review,
  documentation, spreadsheets, and ordinary source edits. Native Windows is not
  the supported environment for building or running the ROS 2/MuJoCo workspace.
- **Windows 11 with WSL2 and Ubuntu 22.04:** suitable for many builds and tests.
  WSLg may run RViz and MuJoCo, but graphics, ROS networking, USB, I2C, lidar,
  and other physical-hardware access can require additional troubleshooting.
- **Ubuntu 22.04 virtual machine:** suitable for development and simulation if
  3D acceleration is enabled. RViz and MuJoCo may run more slowly than on a
  native installation. Allocate at least 4 CPU cores, 8 GB RAM, and 30 GB disk;
  12 GB RAM is preferable when the host has enough memory.
- **Native Ubuntu 22.04 AMD64:** recommended for complete simulation work and
  strongly preferred for the physical robot, servos, PCA9685, lidar, USB, I2C,
  and ROS networking.

Every member needs:

- A 64-bit computer capable of running Ubuntu 22.04, directly or in a VM/WSL2.
- At least 30 GB of free disk space for Ubuntu, ROS, build products, and the
  Python environment. More space is recommended for logs and recordings.
- At least 8 GB RAM for simulation; 12 GB or more is preferable.
- Internet access for Git, apt, ROS packages, and Python packages.
- Git installed and permission to read the GitHub repository.
- A GitHub account with collaborator or fork access before attempting to push.
- VS Code or another source editor. Open the repository root
  `spotmicro_project`, not only `mike_mujoco_ws`.
- A terminal and basic ability to copy complete commands and error output.
- `sudo` access on the Ubuntu environment if installing ROS or system packages.
- A working graphical desktop/display for RViz and the native MuJoCo viewer.

The supported simulation software versions are:

- Ubuntu 22.04 LTS (Jammy), AMD64
- ROS 2 Humble; do not source ROS Foxy for this workspace
- Python 3.10
- MuJoCo 3.10.0
- NumPy 1.26.4
- Pillow 12.3.0
- RViz 2
- colcon

Members doing only Windows-based review do not need to install ROS or MuJoCo.
Members building or running the simulation must use an Ubuntu 22.04 environment
and install the software above. Members working with the physical robot must
also read the hardware-safety section before connecting power.

The preserved physical reference was originally written for Ubuntu 16.04 and
ROS Kinetic. It is reference material, not the active environment. Do not build
that preserved tree as the ROS 2 simulation workspace.

Main repository:

<https://github.com/dpeyton8/spotmicro_project>

Do not rely on a hard-coded commit ID in this document because `main` continues
to advance. Always pull `main`, record the resulting commit with
`git rev-parse HEAD`, and work on a new branch.

## Clone and identify the starting revision

```bash
git clone https://github.com/dpeyton8/spotmicro_project.git
cd spotmicro_project
git checkout main
git pull --ff-only
git rev-parse HEAD
```

Send the resulting commit ID to the project owner when reporting setup results.

## Repository map

- `mike_mujoco_ws/src/spotMicro`: active ROS 2 and MuJoCo source workspace.
- `physical/spotMicro`: preserved Mike4192 physical-build reference. Do not
  patch, build, or clean it as part of simulation work.
- `backup_mike4192_spotMicro`: additional preserved comparison copy.
- `docs/PROJECT_MAP.md`: package roles and repository layout.
- `docs/INSTALL_AND_RUN_MIKE_MUJOCO.md`: staged installation and validation
  runbook.
- `docs/IK_PAPER_TRACEABILITY.md`: mapping between the kinematics code and the
  reference paper, including verified discrepancies and limitations.

Read the root `README.md`, `README_PROJECT_STRUCTURE.md`, and the three documents
listed above before changing the implementation.

## Kinematics locations

Runtime C++ kinematics:

```text
mike_mujoco_ws/src/spotMicro/spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/
```

Python kinematics and tests:

```text
mike_mujoco_ws/src/spotMicro/spot_micro_plot/scripts/spot_micro_kinematics_python/
```

Reference paper:

<https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot>

Read `docs/IK_PAPER_TRACEABILITY.md` before changing formulas. The repository
intentionally does not reproduce several erroneous or ambiguous formulas from
the paper literally. The completed traceability review reported 48 passing
kinematics and traceability tests. Hardware validation, Python IK boundary
clamping, and final URDF/D-H joint-axis reconciliation are separate follow-up
tasks.

## Install Ubuntu and ROS packages

Install ROS 2 Humble using the official ROS instructions for Ubuntu 22.04. Do
not source ROS Foxy for this workspace.

Install the required development packages:

```bash
sudo apt update
sudo apt install -y \
  python3-venv \
  python3-pip \
  python3.10-venv \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

If a required package is missing or installation proposes unexpected removals,
stop and post the complete command and output before proceeding.

## Create the Python environment

From the cloned repository:

```bash
cd spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash

python3 -m venv .venv --system-site-packages
source .venv/bin/activate

python -m pip install --upgrade pip
python -m pip install \
  mujoco==3.10.0 \
  numpy==1.26.4 \
  Pillow==12.3.0
```

The `--system-site-packages` option is required so the virtual environment can
use the ROS 2 Python packages installed by apt.

## Build the ROS 2 workspace

```bash
cd spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate

.venv/bin/python /usr/bin/colcon build --symlink-install
source install/setup.bash
```

For each new terminal, restore the environment with:

```bash
cd spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Do not commit `.venv/`, `build/`, `install/`, `log/`, or debug screenshots.

## Initial simulation launch

Test MuJoCo without RViz first:

```bash
ros2 launch spot_micro_mujoco_sim mujoco_sim_launch.py \
  use_rviz:=false \
  use_mujoco_viewer:=true
```

After that succeeds, test RViz:

```bash
ros2 launch spot_micro_mujoco_sim mujoco_sim_launch.py \
  use_rviz:=true \
  use_mujoco_viewer:=false
```

The initial validation order is:

1. Compile the MJCF and confirm it contains 12 actuators.
2. Run headless MuJoCo steps and verify that the state remains finite.
3. Build the workspace successfully.
4. Launch the simulator without RViz.
5. Confirm `/joint_states` and `/tf` are publishing.
6. Launch with RViz and confirm the complete robot is visible at a plausible
   scale.
7. Start the motion controller with `run_standalone:=true` and
   `run_lcd:=false`.
8. Perform a controlled simulation standing test.
9. Review joint directions, contacts, pose, and stability.
10. Attempt walking only after the standing result has been reviewed.

Never send a walking command automatically during setup.

## Verified VM simulation workflow

From a fresh clone with ROS 2 Humble, MuJoCo, and the workspace already built,
the canonical launch commands are:

```bash
cd ~/Documents/spotmicro_project
./scripts/start_spotmicro_sim.sh
```

In a second terminal:

```bash
cd ~/Documents/spotmicro_project
./scripts/start_spotmicro_motion.sh
```

The scripts source the required ROS/workspace environments and apply the
simulation display and TF settings. Keep both terminals running.

Use one terminal for the combined MuJoCo and RViz launch:

```bash
cd ~/Documents/spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
ros2 launch spot_micro_mujoco_sim mujoco_sim_launch.py \
  use_rviz:=true use_mujoco_viewer:=true
```

Use a second terminal for the existing motion controller:

```bash
cd ~/Documents/spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch spot_micro_motion_cmd motion_cmd_launch.py \
  run_standalone:=true run_lcd:=false publish_tf:=false
```

Run only one simulation launch at a time. Duplicate simulation or
`robot_state_publisher` processes publish competing transforms and make the
legs appear to jump.

Sit and stand are existing features of the `spot_micro_motion_cmd` state
machine, not a separate project. Use a third terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /stand_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /idle_cmd std_msgs/msg/Bool "{data: true}"
```

The repository calls the sitting pose `idle`. The verified MuJoCo bridge
mapping is `LF_1` -> servo 12 and `LF_3` -> servo 10.

## Physical build resources

The physical frame is based on Deok-yeon Kim's KDY0523 SpotMicro:

<https://www.thingiverse.com/thing:3445283>

The OROBOT SpotMicro ESP32 project is related but is a different controller,
software stack, and build configuration:

<https://orobot.io/o/program/BROKER-2/spotmicro-esp32>

Physical-build planning files:

- `docs/SPOTMICRO_PHYSICAL_BOM.csv`: components, fasteners, tools, quantities,
  and purchasing information.
- `docs/SPOTMICRO_3D_PRINTS.csv`: general printing plan.
- `docs/SPOTMICRO_KDY0523_EXACT_STL_SELECTION.csv`: exact print/skip checklist.
- `docs/SPOTMICRO_KDY0523_DOWNLOAD_AUDIT.md`: audit of the two Thingiverse
  archives.
- `docs/SPOTMICRO_ORDER_CANDIDATES.md`: servo, battery, charger, and electrical
  candidates.
- `docs/SPOTMICRO_OROBOT_REPO_CHECK.md`: comparison with the ESP32 project.

The original STL collection is divided between these two archives:

```text
Spotmicro - robot dog - 3445283 - part 1 of 2.zip
Spotmicro - robot dog - 3445283 - part 2 of 2.zip
```

Use both archives together. Obtain them from the project owner if they are not
available in the repository. Do not manufacture parts from the URDF/RViz meshes.
For the planned Raspberry Pi and high-voltage-servo build, follow the exact
selection spreadsheet and use its `_cls` and `non-mega` choices. Confirm actual
servo dimensions, spline, horn, voltage, and fit before printing the entire set.

## Hardware safety

- Prove a pose in simulation before sending it to physical servos.
- Begin with one-servo and one-leg bench checks before powering all 12 servos.
- Support the robot off the ground during initial motion tests.
- Use current-limited power and keep an immediate battery disconnect accessible.
- Verify servo center, direction, limits, and horn orientation individually.
- Do not test singularity poses or unverified walking motions on hardware.
- Establish stable standing before attempting walking.
- Treat LiPo batteries as a fire risk and use an appropriate balance charger and
  fire-resistant charging/storage bag.

## Contribution rules

1. Pull the latest `main` and create a branch for every task.
2. Do not push directly to `main`.
3. Keep simulation, hardware, dependencies, and documentation changes separate.
4. Preserve unrelated local changes in a dirty worktree.
5. Run relevant tests and include exact commands and results in the pull request.
6. Document assumptions, unresolved discrepancies, and anything requiring human
   or hardware confirmation.
7. Do not merge your own pull request unless the project owner explicitly asks.
8. Post complete error output before applying speculative dependency or system
   changes.

## First check-in

Every new member should send the project owner:

- GitHub username.
- Operating system and version.
- Whether ROS 2 Humble is already installed.
- Whether the work is simulation/software, research/documentation, or physical
  assembly.
- The intended first task.
- Output of `git rev-parse HEAD`.
- Build and test commands used, with complete output for any failure.
