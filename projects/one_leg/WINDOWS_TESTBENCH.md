# Windows controls testbench

`windows_testbench.html` is a dependency-free left-leg kinematics preview. It
matches the LF ESP32 sketch's absolute `shoulder,hip,knee` angle order and
calibration centers. It does not require Python, ROS, MuJoCo, or an extension.

## Open it

From VS Code:

1. Open `projects/one_leg/windows_testbench.html` in Explorer.
2. Right-click its tab and choose **Reveal in File Explorer**.
3. Double-click the file to open it in the default browser.

Alternatively, paste this path into a browser:

```text
C:\Users\David\OneDrive - University of New Mexico\Documents\SpotMicro\projects\one_leg\windows_testbench.html
```

If a browser blocks local files, serve the repository root and open
`http://localhost:8000/projects/one_leg/windows_testbench.html`:

```bash
python -m http.server 8000
```

## Demonstrations

### Firmware-matched joint angles

Move the shoulder/q1, hip/q2, and knee/q3 sliders. Values are absolute
calibrated angles, matching the firmware serial command order
`shoulder,hip,knee`. The displayed command can be entered in Serial Monitor,
but the page does not communicate with the ESP32. Sliders use the firmware's
configured ±82.5 degree bounds around centers -7.6, 38.6, and -82.8 degrees.

### Move the foot to a target

Set target x/z and foot height above the fixed ground plane at `y = -180 mm`,
then choose **Move foot to target**. The page solves the left-leg negative-knee
branch, updates the joint angles, and reports toe-position error. Targets that
are unreachable or require angles outside the firmware bounds are rejected.

### Lower and hold the toe

Choose **Lower toe, then hold ground**. The simulated controller lowers the
toe from the selected height to the ground at the selected x/z point, then
applies small fixture disturbances while a damped least-squares Jacobian
controller updates the three joint angles. Adjust gain and damping and watch
the tracking-error graph. Unreachable ground targets are rejected.

Questions suitable for a controls-lab presentation:

- What happens when gain is too low?
- Does low damping cause large or abrupt joint corrections near a singularity?
- Which target locations are unreachable?
- When do joint limits prevent zero tracking error?
- How do the firmware angle limits restrict reachable foot targets?

## Presentation use

Suggested sequence:

1. Show the paper's three-DOF leg figure.
2. Manipulate joint angles to demonstrate forward kinematics.
3. Set a foot height and target x/z, then move the foot to the target.
4. Run the lower-and-hold test and change controller gain/damping.
5. Use MuJoCo for gravity, contact, friction, actuator limits, and force logs.
6. Use RViz to show ROS joint states and TF, not as the physics engine.

## Boundary

This browser testbench is kinematic. Its ground line is a target reference,
not physical contact. It cannot predict foot slip, servo current, contact
force, structural flex, or support load. The firmware also has no toe or
ground sensor, so it cannot stabilize a physical foot by itself. Validate
contact behavior in MuJoCo, then add physical feedback sensing and test in a
constrained fixture.
