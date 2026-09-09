# q1 range-of-motion comparison

This project visualizes Alanah's `q1` finding across a range of shoulder side-swing angles.

It compares:

```text
repo/corrected q1:
q1 = atan2(y4, x4) + atan2(sqrt(x4² + y4² - L1²), -L1)

paper Eq.15 q1:
q1 = -atan2(-y4, x4) - atan2(sqrt(x4² + y4² - L1²), -L1)
```

The test sweeps the true/intended `q1` angle from `-30°` to `+30°`, with:

```text
q2 = 15°
q3 = -25°
```

For every point in the sweep:

1. Forward kinematics generates the desired foot target.
2. The repo formula tries to recover `q1`.
3. The paper Eq.15 formula tries to recover `q1`.
4. The script compares reconstructed foot error.
5. If MuJoCo is available, it also compares front-right toe error in the MJCF model.

## Run it

From the repo root:

```bash
cd /home/davipeyton8/Documents/spotmicro_project
./mike_mujoco_ws/.venv/bin/python projects/q1_range_of_motion_comparison/q1_range_sweep.py
```

Outputs go to:

```text
projects/q1_range_of_motion_comparison/outputs/
```

## Open the MuJoCo sweep viewer

This cycles between a repo-q1 sweep and a paper-Eq.15-q1 sweep:

```bash
cd /home/davipeyton8/Documents/spotmicro_project
./mike_mujoco_ws/.venv/bin/python projects/q1_range_of_motion_comparison/q1_range_sweep.py --viewer
```

## Best images for a presentation

Use these:

```text
01_q1_angle_recovery_sweep.png
02_leg_frame_foot_error_sweep.png
03_leg_frame_target_vs_reconstructed_path.png
04_mujoco_front_right_toe_error_sweep.png
```

The most important one is:

```text
03_leg_frame_target_vs_reconstructed_path.png
```

That plot shows the intended foot path versus what the repo formula and the paper formula reconstruct.

## Plain-English interpretation

The repo formula follows the intended range of motion. The paper Eq.15 formula does not. This means the paper's printed q1 formula is not merely labeled differently; it sends the side-swing/shoulder angle to the wrong value over a whole motion range.

