# q1 paper-vs-repo MuJoCo test

This folder tests Alanah's specific `q1` finding:

> The paper's printed Eq.15 `q1` formula gives the wrong side-swing angle, while the repo's corrected `q1` formula reconstructs the correct leg pose.

Dummy version:

```text
q1 = shoulder / side-swing angle
q2 = upper leg angle
q3 = knee angle

This test changes only q1.
q2 and q3 are kept the same so the mistake is isolated.
```

## What this test does

The script:

1. Starts with a known-good leg pose:

   ```text
   q1 = 20°
   q2 = 15°
   q3 = -25°
   ```

2. Uses forward kinematics to generate the foot target.
3. Tries to recover the original angles using:

   - the repo's corrected `q1` formula;
   - the paper's printed Eq.15 `q1` formula.

4. Checks which one puts the foot back at the original target.
5. If MuJoCo is installed, loads the SpotMicro MJCF and compares the front-right toe position.

## Run the test

From the repo root:

```bash
cd /home/davipeyton8/Documents/spotmicro_project
source mike_mujoco_ws/.venv/bin/activate
python projects/q1_paper_vs_repo_mujoco/q1_compare_mujoco.py
```

If you want to skip MuJoCo and only run the math proof:

```bash
python projects/q1_paper_vs_repo_mujoco/q1_compare_mujoco.py --skip-mujoco
```

## Open the MuJoCo visual comparison

This opens a MuJoCo viewer and cycles the front-right leg between:

1. repo-corrected `q1`
2. paper Eq.15 `q1`

```bash
cd /home/davipeyton8/Documents/spotmicro_project
source mike_mujoco_ws/.venv/bin/activate
python projects/q1_paper_vs_repo_mujoco/q1_compare_mujoco.py --viewer
```

The paper pose should look very wrong because the paper's `q1` angle is far outside the expected shoulder side-swing range.

## Expected result

You should see something like:

```text
repo_q1_formula:
  q1,q2,q3 = +20°, +15°, -25°
  error vs target = near 0 m

paper_eq15_q1_formula:
  q1,q2,q3 = about -186.69°, +15°, -25°
  error vs target = clearly not zero
```

That means:

```text
repo q1 reconstructs the target
paper Eq.15 q1 does not
```

## Why this is a fair test

This is not testing all of IK at once. It intentionally isolates `q1`.

Alanah's PR found:

```text
q1 / Eq.15: paper appears wrong, repo is correct
q2 / Eq.16: matches
q3 / Eq.17: matches, with branch-selection care
```

So this test keeps `q2` and `q3` the same and swaps only the `q1` formula.

## Presentation wording

Use this:

> To test Alanah's `q1` finding, we generated a foot target from a known leg pose, then tried to recover that pose with the repo formula and with the paper's printed Eq.15 formula. The repo formula recovered the original pose and foot position. The paper formula produced a shoulder angle around `-186.69°`, which moved the MuJoCo toe far away from the target. This shows the issue is not just angle labeling; the printed paper formula is not the inverse of the paper's own forward kinematics.

