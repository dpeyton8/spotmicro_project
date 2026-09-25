# Install and run Mike Spot Micro in MuJoCo

This is the verified Ubuntu 22.04/Jammy runbook. Do not source Foxy for this workspace.

## Quick start after installation

From the repository root, use the tracked launchers:

```bash
cd ~/Documents/spotmicro_project
./scripts/start_spotmicro_sim.sh
```

In a second terminal:

```bash
cd ~/Documents/spotmicro_project
./scripts/start_spotmicro_motion.sh
```

These start the combined native MuJoCo viewer and RViz, then the existing
stand/idle motion controller with simulation-safe TF ownership.

## Gate sequence

1. Correct the enabled ROS repository so Jammy never queries Focal packages.
2. Run an apt simulation, review dependency and disk impact, then install ROS 2 Humble.
3. Initialize rosdep and inspect its dependency resolution.
4. Install only approved ROS dependencies.
5. Create `mike_mujoco_ws/.venv` with `--system-site-packages`; install pinned official `mujoco`, `numpy`, and `Pillow` wheels only in it.

## Intended terminal setup

After installation, each clean terminal should explicitly source Humble and the workspace rather than relying on Foxy:

```bash
source /opt/ros/humble/setup.bash
source /home/davipeyton8/Documents/spotmicro_project/mike_mujoco_ws/.venv/bin/activate
source /home/davipeyton8/Documents/spotmicro_project/mike_mujoco_ws/install/setup.bash
```

## Intended validation order

1. Compile `spot_micro_sim.xml` with `mujoco.MjModel.from_xml_path` and confirm `nu == 12`.
2. Run 1000 headless `mj_step` calls and check for finite state.
3. Confirm intended packages with `colcon list`; ignore `ros-i2cpwmboard` without deleting it.
4. Build with `colcon build --symlink-install`.
5. Launch simulation without RViz using `use_rviz:=false`; verify `/joint_states` and `/tf`.
6. Launch with RViz using `use_rviz:=true`.
7. Launch motion control with `run_standalone:=true` and verify no `i2cpwm_board_node` process/node exists.
8. Launch `keyboard_command_launch.py` in an interactive terminal.
9. Perform a controlled manual `stand` test.
10. Only after the stand test is reviewed, prepare the walking test. Walking commands must never be sent automatically.

## Safety notes

- `motion_cmd_launch.py` defaults `run_standalone` to `false`; always pass `run_standalone:=true` in simulation.
- Keep `run_lcd:=false` to avoid LCD/I2C access.
- Do not use the combined SLAM launch for this setup; it starts lidar-related components.
- Video recording additionally needs Pillow and the external `ffmpeg` executable. Pillow is imported unconditionally by the simulation node even when recording is disabled.
- For the VM workflow, run the motion controller with `publish_tf:=false` so
	`robot_state_publisher` is the sole owner of the leg transforms.
- The verified MuJoCo mapping is `LF_1` on servo 12 and `LF_3` on servo 10.
- The motion controller already converts target angles to proportional units.
	The MuJoCo bridge must not apply the hardware direction multiplier again;
	double inversion offsets RF_1 and LB_1 shoulders.

To stop normally, press `Ctrl+C` in the simulation terminal and in the motion
controller terminal. The one-shot stand/idle command terminal returns to its
prompt automatically. Restart by running `start_spotmicro_sim.sh` once, then
`start_spotmicro_motion.sh` once. Do not start duplicate launches.

