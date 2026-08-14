# SpotMicro OROBOT link check

Checked link: https://orobot.io/o/program/BROKER-2/spotmicro-esp32

Result: this is related to SpotMicro, but it is not the same build/repo as this workspace.

Why:

- The OROBOT page identifies the project as `SpotMicro ESP32` and says its source is `https://github.com/michaelkubina/SpotMicroESP32`.
- The OROBOT page describes that build as an ESP32-DevKitC, no-ROS redesign.
- This workspace is organized around:
  - `physical/spotMicro`: the Mike4192 Raspberry Pi 3B + ROS Kinetic physical robot reference.
  - `mike_mujoco_ws/src/spotMicro`: the CometYang ROS 2 Humble / MuJoCo adaptation used for simulation.
- The repo README says the physical frame source is KDY0523's Thingiverse Spot Micro frame: https://www.thingiverse.com/thing:3445283.

Practical consequence:

- Use `docs/SPOTMICRO_PHYSICAL_BOM.csv` and `docs/SPOTMICRO_3D_PRINTS.csv` for this repo's physical build.
- Use the OROBOT ESP32 BOM only if you intentionally switch to the Michael Kubina ESP32/no-ROS variant.
