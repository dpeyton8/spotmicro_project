# SpotMicro one-right-leg control project

This project is a small, safe bridge between the repository's inverse-
kinematics work, a MuJoCo contact simulation, and a later physical right-leg
bench test using an ESP32, PCA9685, and three servos.

## Experiment

The simulated right leg is mounted to a constrained test carriage. The
carriage moves slowly in height and fore/aft position while the controller
tries to hold the toe at one fixed point on the floor.

```text
fixed toe target on floor
        -> toe-position error
        -> damped least-squares Jacobian controller
        -> three joint-angle references
        -> MuJoCo position actuators
        -> leg motion + floor contact
        -> measured toe position (feedback)
```

The controller records toe error, commanded/measured joint angles, contact
state, and estimated normal contact force. This makes it possible to evaluate
tracking, settling, contact loss, and sensitivity to disturbances before
moving to hardware.

This is deliberately a test-rig model, not a claim that one leg can balance a
free body. A single point foot cannot stabilize all six body degrees of
freedom. The physical leg must be attached to a rigid frame or linear guide.

## Why MuJoCo is primary

MuJoCo models dynamics, gravity, collision, friction, and floor contact. RViz
does not; it displays robot geometry and transforms. RViz can be added later
when this project is connected to ROS 2, but it is not required for this
standalone control experiment.

## Relationship to the IK paper

The paper assets in `projects/ik_paper_presentation_assets` provide three
useful ideas:

1. One leg is a three-degree-of-freedom chain (`q1`, `q2`, `q3`).
2. Forward kinematics predicts the toe position from joint angles.
3. Inverse kinematics chooses joint angles for a requested toe position.

The repository's traceability work also documents errors in the paper's
printed matrices and `q1` equation. This first simulation uses MuJoCo's
geometric Jacobian and damped least squares rather than copying the suspect
printed equation. A later milestone can replace the numerical step with the
repo-corrected analytic IK after the physical joint-axis/sign mapping is
measured.

References in this repository:

- `docs/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf`
- `docs/IK_PAPER_TRACEABILITY.md`
- `projects/ik_paper_presentation_assets/selected_pdf_crops/03_fig2_single_leg_frames_q1_q2_q3_links.png`
- `physical/spotMicro/spot_micro_plot/scripts/spot_micro_kinematics_python/tests/plotting/plot_single_leg.py`

## Files

```text
projects/one_leg/
|-- README.md
|-- requirements.txt
|-- run_stance_control.py
|-- windows_testbench.html
|-- WINDOWS_TESTBENCH.md
|-- model/
|   `-- right_leg_test_rig.xml
|-- firmware/
|   |-- README.md
|   `-- one_leg_servo_test.ino
`-- outputs/                 # generated CSV logs; ignored by Git
```

## Use now on Windows

Open `windows_testbench.html` directly in a browser. It provides an
interactive three-dimensional projection, joint and toe-target sliders,
corrected-IK versus paper-Eq.15 comparison, and a kinematic damped-Jacobian
toe-hold experiment. See `WINDOWS_TESTBENCH.md` for a controls-lab
presentation sequence.

This Windows page intentionally has no physics. Results graduate to
`run_stance_control.py` in MuJoCo once the Ubuntu VM is ready, and then to ROS
joint-state/TF visualization in RViz.

## Run

Use the repository's Ubuntu 22.04 Python environment after MuJoCo and NumPy
have been installed. Alternatively, make a project-local environment (the
requirements file includes a compatible NumPy selection for the current
Ubuntu 26.04/Python 3.14 WSL installation):

```bash
cd /mnt/c/Users/David/OneDrive\ -\ University\ of\ New\ Mexico/Documents/SpotMicro/projects/one_leg
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run_stance_control.py --viewer
```

For the repository's intended Ubuntu 22.04 workspace:

```bash
cd /home/davipeyton8/Documents/spotmicro_project
source mike_mujoco_ws/.venv/bin/activate
python projects/one_leg/run_stance_control.py --viewer
```

Run headless and save a CSV log:

```bash
python projects/one_leg/run_stance_control.py \
  --duration 12 \
  --output projects/one_leg/outputs/stance_log.csv
```

The script exits nonzero if the final report violates its broad smoke-test
limits. Those limits are not acceptance criteria for hardware.

## Controls milestones

1. **Simulation smoke test:** hold the toe on the floor with no disturbance.
2. **Tracking test:** enable slow carriage motion and log toe error.
3. **Contact test:** measure contact-loss time and normal-force variation.
4. **Model calibration:** replace estimated link masses, damping, actuator
   gains, joint limits, and friction with measured values.
5. **Servo calibration:** establish center pulse, safe minimum/maximum pulse,
   direction, and zero-angle offset for each physical joint.
6. **Hardware open-loop IK:** command small, slow trajectories with the leg
   unloaded and mounted in a fixture.
7. **Hardware feedback:** add an IMU/encoder/contact or load sensor before
   calling the physical behavior "ground stabilization."

## Physical mapping

The temporary ESP32/PCA9685 allocation is:

| Joint | Meaning | PCA9685 channel |
|---|---|---:|
| `q1` | shoulder side swing | 0 |
| `q2` | upper leg/hip pitch | 1 |
| `q3` | knee/lower-leg pitch | 2 |

The included firmware is intentionally a manual, center-first calibration
tool. It does not automatically sweep a constructed leg. Do not transfer the
simulation's radians directly to servo pulses until the offsets and directions
have been measured.

## Cometyang MuJoCo and RViz compatibility

The one-leg MJCF deliberately uses the existing Cometyang/Mike names:

```text
front_right_shoulder
front_right_leg
front_right_foot
front_right_shoulder_actuator
front_right_leg_actuator
front_right_foot_actuator
```

These are the same front-right joint and actuator names used by
`spot_micro_mujoco_sim`. They also match the joints in the existing SpotMicro
URDF consumed by `robot_state_publisher` and RViz.

The standalone experiment does not yet publish ROS messages. Its CSV output
uses measured and commanded `q1/q2/q3` values so that the controller can be
validated before adding transport. The ROS integration milestone is:

```text
one-leg simulation
  -> /joint_states using the three front_right_* names
  -> robot_state_publisher using spot_micro_rviz/urdf/spot_micro.urdf.xacro
  -> /tf
  -> RViz RobotModel
```

RViz will show the state but will not provide ground contact or stability
feedback; that remains in MuJoCo and the CSV metrics. Run
`test_integration_contract.py` after upstream Cometyang changes to detect a
joint/actuator naming mismatch.

## Hardware safety boundary

- Mount the single leg in a rigid fixture; do not hold it by hand.
- Begin with horns disconnected and one servo enabled at a time.
- Use an external current-limited servo supply and a common signal ground.
- Do not power servos from the ESP32 or PCA9685 logic `VCC`.
- For MG996R, use regulated 5--6 V servo power, not a direct 2S LiPo.
- Keep an accessible power disconnect.
- Stop immediately on binding, chatter, unexpected motion, heat, or excessive
  current.
