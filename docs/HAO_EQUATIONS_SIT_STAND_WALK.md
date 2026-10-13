# Hao et al. (2020) equations for sit, stand and walk

Source: Q. Hao, Z. Wang, J. Wang, G. Chen, "Stability-Guaranteed and High
Terrain Adaptability Static Gait for Quadruped Robots," Sensors 2020, 20, 4911
(`docs/Stability-Guaranteed and High Terrain Adaptability Static Gait for Quadruped Robots.pdf`).
Equation numbers are the paper's printed numbers (pp. 3-10). Eq. 7-18 were
transcribed from page images; Eq. 1-3 came from the extracted text, so
check them against the PDF before showing them on screen.

## How this paper fits (read first)

- Unlike Zhang, this is a **whole-body, four-leg** paper. It describes a **static
  gait**: only one leg moves at a time, and the body's centre of mass (CoM) stays
  inside the triangle formed by the three feet on the ground.
- It has **no "sit" or "stand" section**. It does give the equation that moves
  the body (position and tilt) smoothly while the feet stay planted (Eq. 9 and
  Eq. 18). Changing body height with the feet planted *is* stand/sit, so that
  is the equation to point to, but the stand/sit heights are our choice.
- Its leg is 3-DOF: hip roll (theta1), hip pitch (theta2), knee pitch (theta3),
  the same joint layout as SpotMicro. It uses generic `FK` and `IK` (Eq. 2-3)
  without printing the closed form. For the closed-form IK use Sen et al.
  (`docs/IK_PAPER_TRACEABILITY.md`).
- Its experiments are **simulation only (Webots)** on a hydraulic robot. Nothing
  here is hardware-validated for a hobby-servo robot like SpotMicro.
- One leg in MuJoCo can show the **leg's** part of every motion below (foot
  target in, joint angles out). The CoM-shift, support-triangle and stability
  parts need the full four-leg body.

## Shared building blocks

### Eq. 2-3: forward and inverse kinematics

```text
B_pFOOT = [x y z]^T = FK(q)          (Eq. 2)
q = [theta1 theta2 theta3]^T = IK(B_pFOOT)     (Eq. 3)
```

| Symbol | Meaning | Analogy |
|---|---|---|
| `B_pFOOT` | foot position in the body frame {B} | where the toe is, measured from the body centre |
| `q` | the three joint angles | how far each joint is bent |
| `FK` | joints to foot position | "given my joint bends, where is my toe?" |
| `IK` | foot position to joints | "I want my toe there, how should I bend?" |

Purpose: every planned foot or body motion below ends up as a foot position in
{B}, and `IK` turns it into servo angles.

### Eq. 1: leg Jacobian (printed at the start of section 2)

```text
J = [ 0                              l2*c2 + l3*c23            l3*c23
      l1*c1 + l2*c1*c2 + l3*c1*c23   -l2*s1*s2 - l3*s1*s23     -l3*s1*s23
      l1*s1 + l2*s1*c2 + l3*s1*c23    l2*c1*s2 + l3*c1*s23       l3*c1*s23 ]
```

`s_i = sin(theta_i)`, `c_i = cos(theta_i)`, `s_ij = sin(theta_i + theta_j)`.
`l1, l2, l3` are the hip offset, thigh and shank lengths.

Purpose and analogy: a gear ratio. Foot velocity = J * joint velocity, joint
torque = J^T * foot force. Use it to plot servo torque while standing.
Note the Jacobian here uses `theta_i + theta_j` angle sums, while Zhang uses
`theta2 - theta3`; the sign convention of the knee differs, so use whichever
matches your MuJoCo joint axes.

## Part A: Sit and stand (adapted: body height change with feet planted)

The paper never names this motion. The adaptation is: keep all four feet where
they are on the ground and move the body (CoM) vertically between a **stand**
height and a **sit** height, using Eq. 9 and Eq. 3.

### Eq. 9: smooth cycloid motion of the body

```text
q   = q0 + (qd - q0) * (phi - sin(phi)) / (2*pi)
phi = 2*pi*t / T_sp
```

with `q = [x y z alpha beta gamma]^T`.

| Variable | Meaning | Analogy |
|---|---|---|
| `q` | body position (x, y, z) and tilt (alpha roll, beta pitch, gamma yaw) | where the torso is and how it is tilted |
| `q0` | starting body pose | the pose you start in |
| `qd` | desired body pose | the pose you want to reach |
| `t` | time since the motion started | a stopwatch |
| `T_sp` | duration of the support-phase motion | how many seconds the move takes |
| `phi` | normalized time, 0 to 2*pi | a clock hand sweeping one full turn |
| `(phi - sin phi)/(2*pi)` | progress from 0 to 1 with zero speed at both ends (cycloid) | an elevator that eases away from the floor and eases into the top |

Purpose: moves the body without jerks, so speed and acceleration are zero at
the start and end. The paper uses it for the CoM shift in the support phase.

For stand and sit:

```text
q0 = [0 0 z_sit    0 0 0]   (sitting)
qd = [0 0 z_stand  0 0 0]   (standing)       swap them to sit down
```

Then each foot, fixed in the world, is re-expressed in the moving body frame
(Eq. 14-17 below), and `IK` (Eq. 3) gives the joint angles at every instant.

What is ours, not the paper's: the numbers `z_sit` and `z_stand`, and calling
this "stand/sit". Say so in the video.

### Eq. 18: moving the body with feet planted (also covers body tilt)

```text
P_i^Bd(t) = T(alpha + lam*(alpha_d - alpha))
            * T(beta  + lam*(beta_d  - beta))
            * T(gamma + lam*(gamma_d - gamma))
            * T(-gamma) * T(-beta) * T(-alpha) * P_i^B

lam = (phi - sin(phi)) / (2*pi),   phi = 2*pi*t / T
```

| Variable | Meaning | Analogy |
|---|---|---|
| `P_i^B` | foot `i` (1 to 4) in the current body frame | where leg `i`'s toe appears to the torso now |
| `P_i^Bd(t)` | foot `i` in the body frame as it moves toward the desired tilt | where the toe appears to the torso mid-move |
| `T(a)`, `T(b)`, `T(g)` | rotation matrices about X, Y, Z (Eq. 15) | turning the torso like a gimbal |
| `alpha, beta, gamma` | current roll, pitch, yaw | torso tilt now |
| `alpha_d, beta_d, gamma_d` | desired roll, pitch, yaw | torso tilt wanted |
| `lam` | cycloid progress from 0 to 1 (same as Eq. 9) | the same eased progress bar |
| `T` | duration of the attitude adjustment | seconds for the tilt |

Purpose: tilt or level the body while the feet stay put on the ground. At
`t = 0` it returns the current foot positions; at `t = T` it returns the
desired ones. Use it if the sit or stand pose should also pitch the body
(for example a "sit" that tilts the torso back). The rotation order is
Z, then Y, then X as the paper states (Eq. 14-15).

### Eq. 14-17: converting foot positions between frames

```text
P_i^B  = T(alpha) T(beta) T(gamma) P_i^P                      (Eq. 14)
P_i^P  = T(-gamma) T(-beta) T(-alpha) P_i^B                   (Eq. 16)
P_i^Bd = T(alpha_d) T(beta_d) T(gamma_d) P_i^P                (Eq. 17)
```

`{P}` is a level frame at the same origin as the body (a "horizontal" frame),
and `{Bd}` is the desired body frame. `T` in Eq. 15 is the rotation matrix
for one axis. Analogy: describing the same toe from two viewpoints, one from
the tilted torso and one from a level reference. Rotating the viewpoint
changes the coordinates but not where the toe is on the ground.

## Part B: Walking (static gait, "intermittent gait 1")

The paper compares three static gaits (Fig. 4) and builds the rest on
**intermittent gait 1**, which has the largest stability margin:

1. Move the CoM so it sits inside the triangle of the three feet that will stay
   down, then lift **one** leg, swing it forward and put it down.
2. Repeat for the next leg. One cycle is 8 steps of duration `T` (so 8T), with
   a leg step and a CoM move taking turns.
3. Rules for the order of legs: hind leg first between left or right side in a
   row; rear leg first within a fore or hind side in a column; each CoM move
   follows the walking direction as much as possible.

Cycle times and speeds (step length `S`, one action takes `T`):

| Gait | Cycle | Speed | Stability margin |
|---|---|---|---|
| Intermittent 1 | 8T | S/(8T) | highest |
| Intermittent 2 | 6T | S/(6T) | medium |
| Coordinated | 4T | S/(4T) | lowest |

### Eq. 7-8: stability margin

```text
x_CoP = (F_Az*x_A + F_Bz*x_B + F_Cz*x_C) / (m*g)
y_CoP = (F_Az*y_A + F_Bz*y_B + F_Cz*y_C) / (m*g)               (Eq. 7)

SM = min{ d1, d2, d3 }                                          (Eq. 8)
```

| Variable | Meaning | Analogy |
|---|---|---|
| `x_CoP, y_CoP` | centre of pressure on the ground | the point where the robot's weight effectively pushes down |
| `A, B, C` | the three supporting feet | three table legs |
| `F_Az, F_Bz, F_Cz` | vertical force on each supporting foot | how hard each table leg is pressed |
| `(x_A, y_A)` etc. | foot positions on the ground | where each table leg stands |
| `m, g` | robot mass and gravity | total weight |
| `d1, d2, d3` | distances from the CoM projection to each side of the support triangle | how far you are from each edge of a stool |
| `SM` | stability margin | the closest edge. Larger is safer; zero or negative means a tip-over |

Purpose: a walking check. The paper simplifies by using the CoM projection in
place of the CoP. In static walking the CoM must stay inside the support
triangle; Remark 1 says stability is highest when it is at the triangle's
inner centre (`d1 = d2 = d3`).

### Eq. 9 (again): CoM shift in the support phase

Same cycloid equation as above, with `q0` the CoM start and `qd` the CoM end
for that step. The CoM move stays inside the support triangle.

### Eq. 10: swing leg trajectory (polynomial, zero impact)

```text
f(t) = a0 + a1*t + a2*t^2 + ... + an*t^n
```

| Variable | Meaning | Analogy |
|---|---|---|
| `f(t)` | foot position along one axis during swing | the toe's height or forward position over time |
| `a0 ... an` | polynomial coefficients | the dials that shape the curve |
| `n + 1` | number of constraints | how many conditions the curve must satisfy |

The constraints (paper, "principle of zero impact"): speed and acceleration
are zero at liftoff and at touchdown (four conditions), plus start and end
position (two), plus `n - 5` custom ones (for example the lift height `H`).
You solve for the coefficients `a_k` from these. Purpose: soft landing and
soft liftoff, like Zhang's Bezier curve in Eq. 18 of that paper. The paper
gives no specific coefficient values, so the shape and the lift height `H`
are our choice.

### Eq. 9 again for the support phase, and Eq. 3 for the joints

During each leg step the other three feet are planted. The body trajectory
(Eq. 9) is converted to each planted foot's position in {B}, and Eq. 3 gives
the joint angles. For one MuJoCo leg this is the leg's stance motion.

### Optional: exploratory gait on uneven ground (section 4.1.2, Fig. 5)

The swing is split into three timed parts:

| Part | Time share | What happens |
|---|---|---|
| Swing 1 | 1/5 of `T_sw` | lift the foot by `H` |
| Swing 2 | 2/5 of `T_sw` | move forward by `S`; if a force opposes the motion in x above a threshold, retract and raise by `H/2` (and the CoM by `H/2`) and try again |
| Swing 3 | 2/5 of `T_sw` | lower the foot; if a vertical force above a threshold appears, the foot has landed; if no contact after probing `H`, keep going at speed `H/T_sw` |

`H` is the step height, `S` the step length, `T_sw` the swing time. Purpose:
stepping over obstacles with only foot force sensing. Needs a foot force sensor,
which stock SpotMicro does not have. Skip it for the basic video.

### Optional: compliance (Eq. 11, section 4.2)

```text
Z_f(s) = Delta_x_f(s) / F_e(s) = 1 / (M*s^2 + D*s + K)           (Eq. 11)
```

`M, D, K` are virtual mass, damping and stiffness; `F_e` is the contact force;
`Delta_x_f` is how much the foot target is shifted. Analogy: a car suspension.
Same idea as Zhang's Eq. 20-21 but for the whole controller. It needs force
sensing; in MuJoCo you can show it by lowering the position actuator gains.

### Optional: terrain estimation (Eq. 12-13)

```text
n_g = P1P2 x P2P4 = [x_g y_g z_g]^T                              (Eq. 12)
alpha_g = -arctan( y_g / sqrt(x_g^2 + z_g^2) ),  beta_g = arctan( x_g / z_g )   (Eq. 13)
```

`P1, P2, P4` are three foot contact points; their cross product gives the
slope's normal vector `n_g`, and Eq. 13 turns it into the slope's roll and
pitch. These become the desired body tilt `alpha_d, beta_d` in Eq. 17-18.
Four-leg only; not needed for a single-leg video.

## Quick map for the video

| Segment | Paper equations | Our own additions |
|---|---|---|
| Leg model | Eq. 2-3 (and Sen et al. for the closed-form IK) | SpotMicro link lengths and joint limits |
| Stand / sit (body height, feet planted) | Eq. 9, Eq. 14-17, Eq. 3 | `z_stand`, `z_sit`, calling it stand/sit |
| Body tilt (optional) | Eq. 18 | tilt targets |
| Walking order | Fig. 4a (intermittent gait 1), 8T cycle | which leg starts, `T` |
| Swing leg | Eq. 10 | `H`, `S`, `T_sw`, polynomial order |
| Body shift | Eq. 9 | CoM target inside the triangle |
| Walking safety check | Eq. 7-8 | a threshold for the minimum `SM` |
| Torque plot (optional) | Eq. 1 (Jacobian) | servo torque limits |
