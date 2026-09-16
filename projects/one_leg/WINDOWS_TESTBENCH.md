# Windows controls testbench

`windows_testbench.html` is a dependency-free visualization for work that can
be completed before the Ubuntu VM is ready. It runs locally in a modern web
browser and does not require Python, ROS, MuJoCo, or an extension.

## Open it

From VS Code:

1. Open `projects/one_leg/windows_testbench.html` in Explorer.
2. Right-click its tab and choose **Reveal in File Explorer**.
3. Double-click the file to open it in the default browser.

Alternatively, paste this path into a browser:

```text
C:\Users\David\OneDrive - University of New Mexico\Documents\SpotMicro\projects\one_leg\windows_testbench.html
```

The page runs entirely from the local file. A Live Server extension is not
required.

## Demonstrations

### Forward kinematics

Move `q1`, `q2`, and `q3`. The browser computes and draws the hip, upper leg,
lower leg, and resulting toe position using the same transform structure as
the repository's Python kinematics.

### Corrected inverse kinematics

Move the desired `x/y/z` toe sliders and choose **Solve corrected IK**. In the
paper/repository leg frame, `x` is fore/aft, negative `y` points downward, and
`z` is lateral. The preview ground is therefore `y = -180 mm`. The page
computes the right-leg positive-knee branch and reports reconstructed toe
error.

### Paper Eq.15 comparison

The comparison keeps corrected `q2/q3` but substitutes the paper's printed
`q1`. This isolates the sign error discussed in `docs/IK_PAPER_TRACEABILITY.md`.

### Toe-hold controls experiment

Choose an initial reachable target above the ground, solve it, and choose
**Start toe hold**. The controller first lowers the toe vertically to
`y = -180 mm`. It then moves the mount in two axes while a damped least-squares
Jacobian controller updates `q1/q2/q3` to keep the toe at the green ground
target. Adjust gain and damping and observe the tracking-error graph. If the
same fore/aft and lateral coordinates cannot reach the ground, the run is
rejected instead of silently commanding an impossible pose.

Questions suitable for a controls-lab presentation:

- What happens when gain is too low?
- Does low damping cause large or abrupt joint corrections near a singularity?
- Which target locations are unreachable?
- When do joint limits prevent zero tracking error?
- Why does the paper's printed `q1` fail to reconstruct the target?

## Presentation use

Suggested sequence:

1. Show the paper's three-DOF leg figure.
2. Manipulate joint angles to demonstrate forward kinematics.
3. choose a toe target and solve corrected inverse kinematics.
4. Apply the paper Eq.15 `q1` and show the error.
5. Run toe hold and change controller gain/damping.
6. Explain that the next VM experiment repeats the controller with MuJoCo
   gravity, contact, friction, actuator limits, and logged forces.
7. Use RViz to show ROS joint states and TF, not as the physics engine.

## Boundary

This browser testbench is kinematic. Its line labeled `ground` is a visual
reference only. It cannot predict whether the physical foot slips, how much
current a servo draws, contact force, structural flex, or whether the leg can
support a mass. Those questions belong in MuJoCo and then a constrained
physical test rig.
