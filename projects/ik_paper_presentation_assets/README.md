# IK paper presentation assets

These are selected image crops from:

`/home/davipeyton8/Downloads/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf`

Use these as small presentation references, not as full-page reproductions.

## Recommended images to include

### 1. `01_fig1_full_robot_leg_numbering_body_frame.png`

Use for: whole quadruped body frame and paper leg numbering.

What it shows:

```text
4 —— body —— 3
1 —— body —— 2
```

Why it matters: this explains that the paper has its own leg numbering, which is separate from our repo's named corners like `right_front`, `right_back`, `left_front`, `left_back`.

### 2. `03_fig2_single_leg_frames_q1_q2_q3_links.png`

Use for: the main IK explanation slide.

What it shows:

- `θ1` / `q1` = side-swing or shoulder angle.
- `θ2` / `q2` = upper leg angle.
- `θ3` / `q3` = knee/lower-leg angle.
- `L1`, `L2`, `L3` = the three leg segment lengths used by the IK math.
- endpoint frame containing `x4, y4, z4`.

This is the best image for explaining “one leg = 3 DOF.”

### 3. `05_eq15_eq16_paper_q1_q2_inverse_formulas.png`

Use for: Alanah's Eq.15 finding.

What it shows:

- the paper's printed `θ1` formula;
- the paper's printed `θ2` formula.

Why it matters: Alanah's PR found that the printed `θ1` formula is wrong, while `θ2` matches the repo.

Important correction:

The repo/corrected `q1` equation is:

```text
q1 = atan2(y4, x4)
     + atan2(sqrt(x4² + y4² - L1²), -L1)
```

The paper Eq.15 image shows:

```text
θ1 = -atan2(-y4, x4)
     - atan2(sqrt(x4² + y4² - L1²), -L1)
```

The key difference is the sign before the second `atan2` term:

```text
repo:  first angle + second angle
paper: first angle - second angle
```

### 4. `06_eq17_q3_and_D_reachability_branch_formula.png`

Use for: q3, branch choice, and singularity/reachability.

What it shows:

- `D`, the reachability helper.
- `θ3 = atan2(±sqrt(1-D²), D)`.
- the two branch choices for different leg groups.

Why it matters: this is where multiple IK solutions and singularities enter the conversation.

## Optional images

### `02_table1_symbols_lengths_joint_angles.png`

Use only if the audience needs a symbol dictionary.

It defines:

- `L1`, `L2`, `L3`;
- `θ1`, `θ2`, `θ3`;
- body dimensions and coordinate systems.

### `04_table2_denavit_hartenberg_leg_parameters.png`

Use only if the slide is more technical.

It shows the D-H parameters used to build the forward-kinematics chain.

### `07_table3_paper_example_angle_outputs.png`

Use only if discussing unresolved paper discrepancies.

Alanah's PR found that the repo cannot fully reproduce all of the paper's Table 3 example outputs from the printed equations. This table is not necessary for a simple first IK presentation.

## Suggested slide image names

If exporting/importing into slides, rename or caption them like this:

```text
Fig1_Paper_Leg_Numbering_And_Body_Frame
Fig2_One_Leg_3DOF_q1_q2_q3
Eq15_Paper_q1_Formula_Error
Eq17_q3_D_Branch_Reachability
Table1_Symbol_Dictionary
Table2_DH_Parameters_Optional
Table3_Paper_Examples_Optional
```

