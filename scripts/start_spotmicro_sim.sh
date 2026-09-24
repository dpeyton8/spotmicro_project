#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$ROOT_DIR/mike_mujoco_ws"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash

export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export PYTHONPATH="$WORKSPACE_DIR/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"

exec ros2 launch spot_micro_mujoco_sim mujoco_sim_launch.py \
  use_rviz:=true use_mujoco_viewer:=true
