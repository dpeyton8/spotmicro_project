#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$ROOT_DIR/mike_mujoco_ws"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash

exec ros2 launch spot_micro_motion_cmd motion_cmd_launch.py \
  run_standalone:=true run_lcd:=false publish_tf:=false
