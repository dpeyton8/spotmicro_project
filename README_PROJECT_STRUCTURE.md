# Mike Spot Micro Project

## Main repositories

### `physical/spotMicro`

Original `mike4192/spotMicro` master repository.

Purpose:

- physical KDY0523 robot reference;
- hardware and servo documentation;
- original gait controller;
- original calibration information;
- original ROS 1 implementation.

Keep this repository unchanged.

### `mike_mujoco_ws/src/spotMicro`

`cometyang/spotMicro` branch:

`feature/mujoco-simulation`

Purpose:

- ROS 2 Humble working version;
- Mike-specific MuJoCo simulation;
- ROS 2 motion controller;
- keyboard controller;
- MJCF model;
- simulation-to-controller bridge.

This is the main development repository.

The checked-in source includes the verified ROS 2 Humble/RViz/MuJoCo fixes and
the reusable launchers in `scripts/start_spotmicro_sim.sh` and
`scripts/start_spotmicro_motion.sh`. The `feature/mujoco-simulation` label is
the upstream provenance of this source; use the project checkout on `main` for
the reproducible workflow documented here.

### `references`

Optional unrelated projects may be stored here later.

MindSpace is not currently included because it is not the Mike/KDY0523 digital
twin.


