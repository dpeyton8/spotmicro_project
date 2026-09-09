# SpotMicro IK → ROS2 → MuJoCo/RViz presentation investigation

Date investigated: 2026-08-16

## Short conclusion

Alanah's merged PR is documentation/test evidence for the inverse-kinematics math. It does **not** directly run the MuJoCo/RViz demo at runtime. The live demo uses the sibling C++ kinematics library inside `spot_micro_motion_cmd`, and Alanah's PR shows that this C++ library matches the same DH/IK math as the documented Python implementation, with one practical runtime difference: the C++ IK clamps some domain-sensitive square-root values while the Python version raises raw `ValueError` near some invalid/singular targets.

So for a presentation:

- Alanah's PR is the math traceability/proof layer.
- `spot_micro_motion_cmd` is the ROS2 command/control layer where IK actually runs.
- `spot_micro_mujoco_sim` is the physics/visual bridge receiving already-computed joint targets.
- RViz is the URDF/TF/joint-state visualizer. RViz does not solve IK.

This is currently **analytic kinematics + hand-coded gait control**, not reinforcement learning.

## What Alanah's PR changed

Merged commit:

- `a01b7a9` — `Document and test SpotMicro IK paper traceability`
- Files changed:
  - `docs/IK_PAPER_TRACEABILITY.md`
  - `mike_mujoco_ws/src/spotMicro/spot_micro_plot/scripts/spot_micro_kinematics_python/tests/test_paper_traceability.py`

The PR added a paper-to-code traceability document and Python tests. It did not change production ROS2, MuJoCo, URDF, MJCF, or hardware-control code.

Main useful PR findings for slides:

1. The code's inverse-kinematics formula for `q1` is correct, while the paper's printed Eq.15 appears to contain a real formula error.
2. The FK/IK math is internally consistent at SpotMicro dimensions.
3. The repo has multiple IK branches; the branch choice is a sign choice in the `atan2(±sqrt(1-D^2), D)` part of `q3`.
4. Exact leg singularities happen when the knee is fully extended or fully folded (`q3 = 0` or `q3 = ±π`), proven computationally with Jacobian rank tests.
5. The Python kinematics code and C++ kinematics code use the same core math, but C++ has runtime clamp protection near invalid IK domains.
6. The URDF and MJCF dimensions match the kinematics dimensions, but the exact joint-axis reconciliation is still listed as an unresolved gap.

## Actual runtime data flow

This is the flow used by the demo where MuJoCo and RViz show the robot:

```text
ROS2 command topics
  /stand_cmd, /idle_cmd, /walk_cmd
  /angle_cmd
  /cmd_vel
        |
        v
spot_micro_motion_cmd
  finite-state machine:
    idle -> transition stand -> stand -> walk
  creates body pose + foot targets
  calls C++ SpotMicroKinematics IK
        |
        v
/servos_proportional
  i2cpwm_board/msg/ServoArray
        |
        v
spot_micro_mujoco_sim
  converts proportional servo command back into joint target angle
  writes target angle into MuJoCo position actuators
        |
        v
MuJoCo model
  simulates base/body/joint physics
        |
        v
/joint_states + world -> base_link TF
        |
        v
robot_state_publisher + RViz
  renders URDF meshes in the simulated joint pose
```

The conceptual control pipeline for the presentation is:

```text
desired body/foot motion
        -> inverse kinematics
        -> desired joint angles
        -> servo/actuator commands
        -> simulated robot motion
        -> joint states / TF visualization
```

## Where IK actually happens

Runtime IK is in the C++ library:

- `mike_mujoco_ws/src/spotMicro/spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/src/utils.cpp`
- `mike_mujoco_ws/src/spotMicro/spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/src/spot_micro_leg.cpp`
- `mike_mujoco_ws/src/spotMicro/spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/src/spot_micro_kinematics.cpp`

The key runtime function is:

```cpp
JointAngles ikine(const Point& point, const LinkLengths& link_lengths, bool is_leg_12)
```

That function:

1. Receives a desired foot point in a leg-local coordinate frame.
2. Computes `D`, the cosine-law helper term for the two-link leg section.
3. Clamps `D` into `[-1, 1]`.
4. Selects one of two knee branches using `is_leg_12`.
5. Computes `q2`.
6. Computes `q1`.
7. Returns three joint angles.

The runtime branch assignment is:

- `right_back_leg_` → `is_leg_12=true`
- `right_front_leg_` → `is_leg_12=true`
- `left_front_leg_` → `is_leg_12=false`
- `left_back_leg_` → `is_leg_12=false`

That means the live controller groups the right-side legs together and the left-side legs together for the IK branch sign choice.

## How ROS2 commands become IK targets

The ROS2 command node subscribes to:

- `/stand_cmd` — event command to rise/stand.
- `/idle_cmd` — event command to return to idle/lie-down style state.
- `/walk_cmd` — event command to enter walking state.
- `/angle_cmd` — body orientation command as `geometry_msgs/msg/Vector3`.
- `/cmd_vel` — walking velocity/yaw-rate command as `geometry_msgs/msg/Twist`.

The state machine then chooses the body and foot targets.

### Stand

When standing, the controller:

1. Uses `getNeutralStance()` to place all four feet at fixed neutral locations around the body.
2. Sets body height to `default_stand_height`.
3. Accepts `/angle_cmd` for body roll/pitch/yaw style commands.
4. Runs filters so body-angle changes are not instantaneous.
5. Calls `setServoCommandMessageData()`, which calls C++ IK and fills servo-angle commands.
6. Publishes `/servos_proportional`.

This is the cleanest presentation example because the robot is not stepping yet. It is just:

```text
body pose + four fixed foot locations -> IK -> 12 joint angles
```

### Walk

When walking, the controller:

1. Uses a phase table to decide which foot is in swing versus stance.
2. Uses `/cmd_vel` to move stance feet backward/sideways relative to the body.
3. Uses a triangular swing-height profile for the swing foot.
4. Optionally shifts the body for balance during the 8-phase gait.
5. Calls C++ IK every control tick to convert the newly desired foot positions into joint angles.
6. Publishes `/servos_proportional`.

This is more advanced than standing and is best presented after the standing IK story.

## MuJoCo's role

MuJoCo does not compute the IK in this repo's current flow.

`spot_micro_mujoco_sim` subscribes to `/servos_proportional`, then converts each servo command back into a target joint angle:

```text
cmd_kin_rad = proportional * servo_max_angle_rad * direction + center_angle_rad
joint_angle = joint_sign * cmd_kin_rad
```

Then it writes that angle into the corresponding MuJoCo position actuator target.

The MJCF model has:

- 12 actuated joints.
- 12 position actuators.
- Shoulder joints with `axis="1 0 0"`.
- Leg and foot joints with `axis="0 1 0"`.
- Front/rear and left/right body positions that match the kinematics dimensions.

MuJoCo is therefore acting like the simulated plant:

```text
joint target -> position actuator -> simulated dynamics -> actual joint state/base pose
```

That is a strong controls-course connection: MuJoCo is the plant/environment, and the ROS2 controller is sending reference commands into it.

## RViz's role

RViz does not compute physics or IK.

The launch file:

1. Processes the SpotMicro xacro into URDF.
2. Starts `robot_state_publisher`.
3. Starts RViz with the SpotMicro RViz config.

The MuJoCo node publishes:

- `/joint_states`
- TF transform `world -> base_link`

Then:

- `robot_state_publisher` uses `/joint_states` + URDF to publish link transforms.
- RViz renders the robot model from those transforms.

So RViz is a visual readout of the simulated robot state, not the source of robot motion.

## How this relates to controls

This is not reinforcement learning yet. The current system is more like:

- analytic inverse kinematics,
- finite-state-machine behavior,
- hand-coded gait generation,
- rate-limited filtering,
- simulated position actuators,
- joint-state/TF feedback for visualization.

Controls vocabulary that fits:

- Reference command: `/stand_cmd`, `/angle_cmd`, `/cmd_vel`.
- Controller: `spot_micro_motion_cmd`.
- Plant: MuJoCo SpotMicro model.
- Actuators: 12 MuJoCo position actuators / future physical servos.
- Sensors/feedback: simulated joint states, base pose, TF, future IMU/LiDAR/hardware sensors.
- Tracking: target joint angles versus simulated actual joint states.
- Stability/singularity: Alanah's PR documents IK singularities and branch behavior.

For a course/presentation angle, phrase it as:

> We are using an analytic inverse-kinematics controller to turn desired body and foot motion into joint-angle references. ROS2 transports those commands, MuJoCo acts as the simulated plant, and RViz visualizes the resulting robot state.

## Presentation slide outline

### Slide 1 — Project framing

SpotMicro is a small quadruped robotics testbed inspired by Boston Dynamics Spot. The current focus is simulation and inverse-kinematics-based control before hardware walking.

### Slide 2 — The problem

A quadruped has 12 actuated joints, but the command we care about is higher level: stand, sit, body angle, velocity, yaw rate, and foot placement.

### Slide 3 — What inverse kinematics does

IK answers:

```text
Where do I want the foot/body?
        -> what should the hip, leg, and knee angles be?
```

Each leg has 3 degrees of freedom:

- side-swing/shoulder,
- upper leg/hip,
- lower leg/knee.

### Slide 4 — Alanah's PR: math traceability

Alanah's PR mapped the 2017 quadruped IK paper to the repo's Python kinematics code and added tests. The PR proves that the repo's code is not just “magic numbers”; it follows a documented FK/IK derivation, while also recording paper typos and unresolved ambiguities honestly.

### Slide 5 — Important PR findings

Recommended points:

- Code's `q1` formula is correct; paper Eq.15 appears wrong.
- FK/IK round-trip checks pass at SpotMicro dimensions.
- Multiple IK branches exist.
- Singularities exist at fully extended/folded knee configurations.
- Python and C++ kinematics match, except C++ adds clamp protection.

### Slide 6 — Runtime architecture

Use this diagram:

```text
ROS2 command topics -> motion command node -> C++ IK -> servo commands
       -> MuJoCo position actuators -> joint states/base TF -> RViz
```

### Slide 7 — Standing demo explained

Standing is the simplest useful demo:

```text
neutral foot locations + desired body height
        -> IK
        -> 12 joint commands
        -> MuJoCo dog stands
        -> RViz shows joint state
```

### Slide 8 — Walking demo explained

Walking adds a gait generator:

- stance feet move relative to commanded velocity,
- swing feet lift and place forward,
- body shifts for balance,
- IK runs each tick to convert foot targets to joint angles.

### Slide 9 — Controls-course connection

Map the system:

- Reference: body pose, foot positions, velocity/yaw commands.
- Controller: ROS2 motion command + IK.
- Plant: MuJoCo simulated robot.
- Output: joint states, base motion, TF.
- Evaluation: tracking error, overshoot, settling, stability, joint limits.

### Slide 10 — What is next

Suggested next steps:

1. Verify standing is stable in MuJoCo.
2. Log target joint commands versus actual `/joint_states`.
3. Add plots of command tracking error.
4. Reconcile URDF/MJCF axes with DH axes more rigorously.
5. Add safe joint limits and avoid singular configurations.
6. Only after simulation is stable, move toward hardware.

## Demo commands worth showing

Terminal 1, MuJoCo + RViz:

```bash
cd /home/davipeyton8/Documents/spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch spot_micro_mujoco_sim mujoco_sim_launch.py use_rviz:=true use_mujoco_viewer:=true
```

Terminal 2, controller:

```bash
cd /home/davipeyton8/Documents/spotmicro_project/mike_mujoco_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch spot_micro_motion_cmd motion_cmd_launch.py run_standalone:=true run_lcd:=false publish_tf:=false
```

Terminal 3, stand once:

```bash
ros2 topic pub --once /stand_cmd std_msgs/msg/Bool "{data: true}"
```

Terminal 3, stand/idle loop:

```bash
while true; do
  ros2 topic pub --once /stand_cmd std_msgs/msg/Bool "{data: true}"
  sleep 10
  ros2 topic pub --once /idle_cmd std_msgs/msg/Bool "{data: true}"
  sleep 10
done
```

Terminal 3, inspect joint states:

```bash
ros2 topic echo /joint_states --once
```

## Good presentation wording

Use this:

> Alanah's PR verified the inverse-kinematics math against the source paper and against the repo's Python implementation. The live ROS2/MuJoCo demo uses the C++ version of the same kinematics inside the motion-command node. ROS2 command topics select high-level states like stand or walk; the controller converts body and foot targets into joint angles; MuJoCo simulates the resulting actuator targets; RViz visualizes the joint states and transforms.

Avoid saying:

> RViz uses the PR to control the robot.

More accurate:

> RViz visualizes the robot state produced by MuJoCo, and Alanah's PR helps us explain and trust the IK math that produces the joint commands upstream.

## Open gaps before claiming hardware readiness

These are still not fully resolved:

1. Exact mapping between paper leg numbers and physical SpotMicro corner names.
2. Physical servo mounting orientation and zero offsets.
3. Full DH-axis to URDF/MJCF-axis reconciliation.
4. Safe hardware joint limits and safe singularity avoidance.
5. Whether the current walking gait is dynamically stable beyond simple visual simulation.
6. Command tracking error between controller targets and actual MuJoCo joint states.

These are good “future work” points for the presentation.

