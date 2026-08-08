SpotMicro MuJoCo 

Purpose
-------
This export contains the working SpotMicro MuJoCo workspace (source + docs + local fixes) prepared on Ubuntu 22.04 amd64. It omits built artifacts and virtual environments.

Important modified files included (must be preserved in git):
- spot_micro_mujoco_sim/launch/mujoco_sim_launch.py  (adds use_mujoco_viewer launch arg)
- spot_micro_mujoco_sim/spot_micro_mujoco_sim/mujoco_sim_node.py (viewer launch + cleanup hardening)
- spot_micro_mujoco_sim/setup.cfg (adds Python install metadata so console script installs to lib/)
- spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/CMakeLists.txt (gtest warning suppression for GCC 11)
- ros-i2cpwmboard/COLCON_IGNORE (marker to ignore ROS1 submodule during colcon)

Where the kinematics live
-------------------------
- C++ kinematics (used by runtime controller):
  mike_mujoco_ws/src/spotMicro/spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/
  Key files: include/spot_micro_kinematics/*.h, src/spot_micro_kinematics.cpp, src/spot_micro_leg.cpp

- Python kinematics (plot / unit tests):
  mike_mujoco_ws/src/spotMicro/spot_micro_plot/scripts/spot_micro_kinematics_python/

The Python README in that folder references the paper: "Inverse Kinematic Analysis Of A Quadruped Robot" (Sen et al., 2017).

Prerequisites (recommended for running / testing)
------------------------------------------------
- Ubuntu 22.04 LTS (amd64)
- sudo access
- ~13 GiB free disk
- python3.10 (system)
- python3.10-venv
- ROS 2 Humble installed (see notes below)

Quick setup steps (on Ubuntu Jammy)
----------------------------------
# 1) Ensure ROS repo & Humble are available (managed ros2-apt-source used in my setup)
# Follow ROS official instructions for Jammy/Humble or use the ros2-apt-source package.

# 2) Install core packages
sudo apt update
sudo apt install -y python3-venv python3-pip python3.10-venv \
  ros-humble-desktop ros-dev-tools python3-colcon-common-extensions \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro

# 3) Prepare workspace venv and install Python wheels
cd /path/to/spotmicro_project_git_export/mike_mujoco_ws
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install --upgrade pip
pip install mujoco==3.10.0 numpy==1.26.4 Pillow==12.3.0

# 4) Build workspace (from within venv)
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 -m colcon build --symlink-install
source install/setup.bash

# 5) Run native MuJoCo viewer
ros2 launch spot_micro_mujoco_sim mujoco_sim_launch.py use_rviz:=false use_mujoco_viewer:=true

Notes and troubleshooting
-------------------------
- The export intentionally excludes .venv, build/, install/, and logs. Recreate the venv and rebuild locally.
- If you run into a missing system package for creating venv, install python3.10-venv.
- If mujoco Python wheel fails due to GL or display errors, ensure your machine has an accessible DISPLAY (or use headless runs) and that Mesa/GL libs are installed.

How to share this as git (optional)
----------------------------------
If you want to put this export on GitHub so Claude or other tools can analyze it:

cd /path/to/spotmicro_project_git_export
git init
git add .
git commit -m "Initial working SpotMicro MuJoCo export (includes local fixes)"
# Create a private repo on GitHub and push it, or share the folder as an archive.

Tasks Alanah can do (pick based on access)
------------------------------------------
If she has Linux (Ubuntu) access:
- Recreate venv, run unit tests under spot_micro_kinematics_python, run MuJoCo viewer, verify walk/stand behavior.
- Run the Python kinematics plotting scripts to visualize leg IK and compare against the paper.
- Run colcon build and exercise the keyboard control and motion nodes.

If she only has macOS or cannot run Ubuntu:
- Read and document the IK math in the C++ and Python kinematics modules (spot_micro_kinematics_cpp and spot_micro_kinematics_python).
- Write a formal IK derivation and mapping to this code (good for docs/research writeup).
- Use Claude to analyze the codebase and propose refactors or improvements.

What I (the sender) will do next
--------------------------------
- Provide a short message you can forward to Alanah with the exact steps above.
- If you want, I can also prepare a minimal GitHub-friendly commit history and a single branch archive ready to push.

Contact / logs
--------------
Export path on my machine: /home/davipeyton8/Documents/spotmicro_project_git_export

If anything fails when she tries to run things, send me the failing command output and I will help debug.
