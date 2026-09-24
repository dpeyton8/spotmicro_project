# Mike Spot Micro MuJoCo setup status

Audit date: 2026-07-19. Workspace: `/home/davipeyton8/Documents/spotmicro_project`.

## Current verified state — 2026-09-23

The historical table below records the pre-install audit. The current VM setup
is complete: Ubuntu 22.04.5 amd64, ROS 2 Humble, RViz 2, MuJoCo 3.10.0,
NumPy 1.26.4, Pillow, rosdep, and colcon are installed. The workspace builds
successfully, the MJCF compiles with 12 actuators, and the combined launch
opens both MuJoCo and RViz.

Use the tracked launchers from the repository root:

```bash
./scripts/start_spotmicro_sim.sh
```

In a second terminal:

```bash
./scripts/start_spotmicro_motion.sh
```

The verified fixes are included in source: ROS 2 Humble RViz plugin IDs,
`base_link`/`/robot_description`, simulation-safe `publish_tf:=false`, and
the front-left mapping `LF_1` -> servo 12 and `LF_3` -> servo 10. Sit and
stand are the existing `/idle_cmd` and `/stand_cmd` state-machine commands.

| Item | PASS/PARTIAL/FAIL | Evidence | Next action |
|---|---|---|---|
| Original Mike repository preserved | PASS | `physical/spotMicro`, origin `https://github.com/mike4192/spotMicro.git`, branch `master`, commit `2a34f5d303dff91b62180031b31ef512a672f3c3`, clean status | Keep read-only/unchanged |
| Working MuJoCo branch verified | PASS | `mike_mujoco_ws/src/spotMicro`, origin `https://github.com/cometyang/spotMicro.git`, branch `feature/mujoco-simulation`, commit `abb9eeb9d353d1643b68027d02e43a8c65ff9a5b`, clean status | Keep branch pinned while setting up |
| Submodules complete | PASS | All three submodules in each repository are initialized at recorded commits; no `-` or `+` marker | Ignore the obsolete ROS 1 `ros-i2cpwmboard` package during colcon build without deleting it |
| Host ROS state | FAIL | Ubuntu 22.04.5 amd64 has `/opt/ros/foxy`, 262 Foxy packages, no Humble packages; no ROS environment variables are currently exported | Gate 1: disable incompatible ROS source; Gate 2: review/install Humble |
| Apt-source state | FAIL | Enabled `/etc/apt/sources.list.d/ros2.list` says `focal main` on Jammy; ROS keyring exists; `ros2-apt-source` is absent | Run Gate 1 only after approval |
| Humble availability | FAIL | `/opt/ros/humble` absent and zero `ros-humble-*` packages installed | Simulate installation and review disk/dependencies at Gate 2 |
| rosdep state | FAIL | `/usr/bin/rosdep` exists, but `/etc/ros/rosdep/sources.list.d/20-default.list` and user cache are absent | Gate 3: initialize/update only after approval |
| Python environment state | PARTIAL | Python 3.10.12; workspace venv absent; system NumPy 1.21.5 and user Pillow 12.3.0 exist | Gate 5: create `.venv --system-site-packages` and install pinned packages inside it |
| MuJoCo state | FAIL | `python3 -m pip show mujoco` reports not installed; no standalone `simulate` executable | Gate 5: install official pinned Python wheel in workspace venv |
| colcon build state | PARTIAL | `/usr/bin/colcon` and extensions installed; `colcon list` discovers ROS 2 packages plus obsolete duplicate ROS 1 package; build not attempted | Add non-destructive `COLCON_IGNORE` later, then build after dependencies |
| MJCF test state | PARTIAL | XML exists and statically contains 12 position actuators; compilation cannot be tested because MuJoCo is absent | Compile and run 1000 headless steps after Gate 5 |
| Simulation launch state | PASS | Ubuntu 22.04 VM has ROS 2 Humble, MuJoCo 3.10.0, RViz, and a passing headless launch; the corrected source is rebuilt locally | Launch one combined MuJoCo/RViz stack and one motion controller |

Verified fixes:

- RViz uses ROS 2 Humble plugin IDs, `base_link`, and `/robot_description`.
- The simulation launch uses the existing RViz configuration.
- `publish_tf:=false` prevents duplicate leg transforms in simulation.
- The MuJoCo front-left mapping is `LF_1` -> servo 12 and `LF_3` -> servo 10.
- Existing `/stand_cmd` and `/idle_cmd` state-machine commands were tested in
	both poses.

