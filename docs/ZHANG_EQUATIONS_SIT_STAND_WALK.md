# Zhang et al. (2020) equations for the one-leg MuJoCo video

Source: S. Zhang, H. Zhang, Y. Fu, "Leg Locomotion Adaption for Quadruped Robots
with Ground Compliance Estimation," Applied Bionics and Biomechanics, 2020,
Article 8854411 (`docs/Leg Locomotion Adaption for Quadruped Robot.pdf`).
Equation numbers below are the paper's printed numbers (pages 3-7).

## Slide-ready attribution block (matches "Locomotion Gait - Walk Cycle")

Use this short split when presenting, so viewers can clearly see what comes
from the repo and what comes from Zhang et al.:

### Repo - Cometyang/Mike full-robot gait

- 8-phase gait scheduler; one leg swings at a time.
- Front-left demo: LF swings in phase 8, while the other three legs support.
- Stance feet respond to translation and yaw commands from the motion stack.
- Swing clearance uses the repo triangular profile:

```text
h(s) = 2*z_c*s           , 0 <= s < 1/2
h(s) = 2*z_c*(1 - s)     , 1/2 <= s <= 1
```

Where `z_c` is swing clearance (for example 0.050 m in your slide).

### Article - Zhang et al. (2020), Eq. 18-19

- Eq. 18: Bezier swing trajectory for the foot path in air.
- Eq. 19: sinusoidal stance trajectory while the foot is on ground.
- Rescale Zhang control-point dimensions to SpotMicro geometry.
- Convert each trajectory foot target to joint angles via IK.

Speaker-note sentence:

"In this demo, gait phase orchestration and LF-only phase selection come from
the Cometyang/Mike repo, while the one-leg swing and stance trajectory shapes
reference Zhang Eq. 18-19 after SpotMicro rescaling and IK conversion."

## Read this first: what the paper does and does not cover

- The paper has **no sit or stand equations** and **no four-leg gait**. It
  gives one leg's kinematics, a swing/stance foot path, and an impedance
  controller. Sit/stand below is built from Zhang's kinematics plus a ramp that
  is **our own addition**; say so in the video.
- Zhang's leg is **3 actuated joints** (hip roll theta1, hip pitch theta2, knee
  theta3) plus an ankle angle theta4 that is mechanically tied to the knee by a
  parallelogram (theta4 = theta3). SpotMicro has no such parallelogram, so
  Eq. 8-10 do **not** transfer. For the real SpotMicro joint solution use the
  Sen et al. IK (`docs/IK_PAPER_TRACEABILITY.md`); Zhang's Eq. 13-15 are the
  same style of closed-form answer for his leg.
- Zhang's numbers (Table 1) are in mm for a ~460 mm leg. SpotMicro's leg is
  much shorter, so every length and Bezier point must be rescaled.
- Zhang's printed Eq. 17 uses lower-case `l1, l2, l3` for the link lengths
  while Eq. 1-15 use `L1..L4`. Treat them as the same hip-offset, thigh and
  shank lengths.

---

## Shared building blocks (used by both sit/stand and walking)

### Joint and link variables

| Symbol | Meaning | Analogy |
|---|---|---|
| theta1 | hip roll, swings the leg sideways | a person stepping their leg out to the side |
| theta2 | hip pitch, swings the thigh forward/back | the thigh swinging at the hip |
| theta3 | knee pitch | bending the knee |
| theta4 | ankle pitch, equal to theta3 by the parallelogram | the shin staying parallel so the foot stays level |
| L1 | sideways offset between roll axis and leg plane | how far the leg hangs off to the side of the hip |
| L2 | thigh length | thigh bone |
| L3 | shank length | shin bone |
| L4 | foot link length | the foot/toe segment |
| P = (Px, Py, Pz) | foot position in the hip frame ("BH") | where your toe is, measured from your hip |

### Eq. 1-6: chain the joint frames

```text
BH_HipR T   = Rot_x(-theta1)                                  (Eq. 1)
HipR_HipP T = Rot_y(theta2),  translate -L1 along y            (Eq. 2)
HipP_Knee T = Rot_y(-theta3), translate -L2 along z            (Eq. 3)
Knee_Ankle T= Rot_y(theta4),  translate -L3 along z            (Eq. 4)
Ankle_Foot T= translate -L4 along z                            (Eq. 5)

BH_Foot T = BH_HipR T * HipR_HipP T * HipP_Knee T * Knee_Ankle T * Ankle_Foot T   (Eq. 6)
```

- Purpose: each matrix moves you from one joint to the next, so multiplying
  them gives the toe pose from the hip.
- Analogy: a chain of coordinate "hand-offs", like giving directions "turn at
  the hip, walk the thigh, turn at the knee, walk the shin, step the foot".

### Eq. 7: forward kinematics (position column of BH_Foot T)

```text
Px = -L4*s(2-3+4) - L3*s(2-3) - L2*s2
Py = -L1*c1 - (L2*c2 + L3*c(2-3) + L4*c(2-3+4)) * s1
Pz =  L1*s1 - (L2*c2 + L3*c(2-3) + L4*c(2-3+4)) * c1
```

`c1 = cos(theta1)`, `s1 = sin(theta1)`, `c(2-3) = cos(theta2 - theta3)`,
`s(2-3+4) = sin(theta2 - theta3 + theta4)`, and so on.

- Purpose: joint angles in, toe position out. In MuJoCo this is the check that
  the simulated toe lands where the IK said it would.
- Analogy: knowing how far each joint is bent, you compute where the toe ends
  up.

### Eq. 8-10: leg length constraint and knee angle

```text
Px^2 + Py^2 + Pz^2 = L1^2 + L2^2 + L3^2 + L4^2
                     + 2*L2*L3*cos(theta3) + 2*L3*L4*cos(theta4)
                     + 2*L2*L4*cos(theta3 - theta4)                        (Eq. 8)

with theta3 = theta4 (parallelogram):
Px^2 + Py^2 + Pz^2 = L1^2 + L2^2 + L3^2 + L4^2
                     + 2*(L2*L3 + L3*L4)*cos(theta3) + 2*L2*L4              (Eq. 9)

theta3 = theta4 = arccos( (Px^2+Py^2+Pz^2 - L1^2 - L2^2 - L3^2 - L4^2 - 2*L2*L4)
                          / (2*(L2*L3 + L3*L4)) )                           (Eq. 10)
```

- Purpose: the squared distance from hip to toe depends only on how bent the
  knee is, so the knee angle falls out first.
- Analogy: the law of cosines. The farther the toe is from the hip, the
  straighter the knee must be; a toe close to the hip needs a deep bend.
- SpotMicro note: replace with the plain two-link law of cosines for
  thigh and shank (Sen et al. theta3 formula).

### Eq. 11-13: hip roll angle

```text
(Py + L1*cos(theta1)) / (Pz + L1*sin(theta1)) = sin(theta1)/cos(theta1)    (Eq. 11)

(-Pz/sqrt(Py^2+Pz^2))*sin(theta1) - (-Py/sqrt(Py^2+Pz^2))*cos(theta1)
      = -L1/sqrt(Py^2+Pz^2)                                                  (Eq. 12)

theta1 = arcsin( -L1 / sqrt(Py^2+Pz^2) ) + arctan( Py / Pz )               (Eq. 13)
```

- Purpose: finds how far the leg must tilt sideways so the toe sits at the
  requested side-to-side and height position.
- Analogy: looking at the leg from the front; the hip offset `L1` makes the
  leg's plane sit beside the hip, and this angle aims that plane at the foot.
- For a toe directly under the hip with no sideways motion, theta1 stays near
  its neutral value, which is what happens in a pure sit/stand.

### Eq. 14-15: hip pitch angle

```text
Px = -(L4 + L2 + L3*cos(theta3))*sin(theta2) + L3*sin(theta3)*cos(theta2)   (Eq. 14)

theta2 = arcsin( Px / sqrt((L4 + L2 + L3*cos(theta3))^2 + (L3*sin(theta3))^2) )
         + arctan( L3*sin(theta3) / (L4 + L2 + L3*cos(theta3)) )            (Eq. 15)
```

- Purpose: with the knee angle known, finds the thigh swing that puts the toe
  at the requested forward/back distance `Px`.
- Analogy: once the knee bend sets the leg's effective length, you swing the
  whole leg forward or back like a pendulum until the toe is at the right spot.

Eq. 10, 13 and 15 together are the paper's inverse kinematics: toe position in,
`theta1, theta2, theta3` out, one closed-form solution.

### Eq. 16-17: Jacobian (velocity and torque map)

```text
J = [ dPx/dtheta1  dPx/dtheta2  dPx/dtheta3 ]
    [ dPy/dtheta1  dPy/dtheta2  dPy/dtheta3 ]
    [ dPz/dtheta1  dPz/dtheta2  dPz/dtheta3 ]                                (Eq. 16)

J = [ 0                              l2*c2 + l3*c23         l3*c23        ]
    [ l1*s1 + l2*c1*c2 + l3*c1*c23   -s1*(l3*s23 + l2*s2)   -l3*s1*s23     ]
    [ -l1*c1 + l2*s1*c2 + l3*s1*c23   c1*(l3*s23 + l2*s2)    l3*c1*s23     ]   (Eq. 17)
```

- Purpose: foot velocity = J * joint velocity, and joint torque = J^T * foot
  force. Use it to plot required torque while the leg holds a stand or
  pushes off.
- Analogy: a gear ratio between your joints and your toe. A small joint turn
  moves the toe a lot when the leg is extended, so the same servo torque gives
  less force at the toe.

---

## Part A: Sit and stand (adapted, not in the paper)

Idea: keep the toe under the hip (`Px` about 0, `Py` about `-L1`) and move only
the vertical toe distance between a **stand height** and a **sit height**. Each
point on the ramp goes through the IK above.

1. Pick `z_stand` and `z_sit` (hip-to-toe distance) inside the leg workspace
   (paper section 2.1, Figs. 4-5). The paper's joint limits are theta1 in
   +-20 deg, theta2 in +-50 deg, theta3 in +-120 deg. Use SpotMicro's own limits.
2. Ramp the toe target smoothly (our addition):

```text
z(t) = z_stand + (z_sit - z_stand) * s(t),   s(t) = 3t^2 - 2t^3,   t in [0, 1]
```

   `s(t)` is a cubic ease curve; it is the simplest Bezier-type blend and keeps
   speed zero at both ends. You may instead use a two-endpoint Bezier from the
   same form as Eq. 18 below.
3. Convert each `(Px, Py, z)` to joint angles with Eq. 10, 13, 15 (or the Sen
   IK for the real SpotMicro leg), then command the MuJoCo position actuators.
4. Optional plot: joint torque from Eq. 16-17 while standing.

| Variable | Meaning | Analogy |
|---|---|---|
| `z_stand`, `z_sit` | hip-to-toe distance when standing, sitting | standing tall vs squatting down |
| `t` | normalized time, 0 at start, 1 at end | a progress bar |
| `s(t)` | eased progress from 0 to 1 | an elevator that accelerates, cruises, then slows to a stop |

What to point to in the video:

- Eq. 6-7 (forward kinematics) to show the model.
- Eq. 10, 13, 15 for the joint angles at each height.
- Eq. 16-17 if you show torque.
- State that the ramp `s(t)` and the stand/sit heights are the team's own.

---

## Part B: Walking (one leg stepping), taken directly from the paper

A step is two phases of one period: **swing** (foot in the air, Bezier curve)
then **stance** (foot on the ground, sine wave). Each point goes through the
same IK as above.

### Eq. 18: swing phase Bezier curve (section 2.3.1)

```text
p_sw(t) = p_sw(S_sw(t)) = sum_{k=0..n}  C(n,k) * (1 - S_sw)^(n-k) * S_sw^k * c_k
v_sw(t) = (dp_sw / dS_sw) * (dS_sw / dt) = (dp_sw / dS_sw) * (1 / T_sw)
```

with `n + 1 = 12` control points `c0..c11` from Table 1 (mm):

| Point | x | y |
|---|---|---|
| c0 | -170 | 460 |
| c1 | -280.5 | 460 |
| c2, c3, c4 | -300 | 361.1 |
| c5, c6 | 0 | 361.1 |
| c7 | 0 | 321.4 |
| c8, c9 | 303.2 | 321.4 |
| c10 | 282.6 | 460 |
| c11 | 170 | 460 |

| Variable | Meaning | Analogy |
|---|---|---|
| `p_sw` | foot position during swing | where the toe is in the air |
| `S_sw` | swing progress, 0 to 1 | how far through the step you are |
| `C(n,k)` | binomial coefficient ("n choose k") | the weight each guide point gets at a given moment |
| `c_k` | the k-th control point | magnets that pull the toe's path toward them without the toe touching all of them |
| `n` | curve order, 11 | number of magnets minus one |
| `T_sw` | swing duration | how many seconds the foot spends in the air |
| `v_sw` | swing foot velocity | toe speed |

- Purpose: lifts the foot with enough clearance and lands it softly with low
  speed and acceleration, avoiding a hard impact.
- Analogy: a flexible wire pulled toward guide points. The foot rises, swings
  forward over the "bump," and comes down gently.
- Repeated control points (c2-c4, c5-c6, c8-c9) make the path flat and slow
  near those spots, which gives the soft liftoff and touchdown.
- In Table 1 / Fig. 6, `x` is forward and `y` points down from the hip, so
  `y = 460` is the foot on the ground and `y = 321.4` is the foot lifted.
  Rescale all points to the SpotMicro leg before use.

### Eq. 19: stance phase sine path (section 2.3.2)

```text
p_x_st(t) = L_span * (1 - 2*S_st(t)) + P0x
p_y_st(t) = delta * cos( pi / (2*L_span) * p_x_st(t) ) + P0y
v_x_st(t) = -2*L_span / T_st
v_y_st(t) = (delta*pi / T_st) * sin( pi / (2*L_span) * p_x_st(t) )
```

| Variable | Meaning | Analogy |
|---|---|---|
| `S_st` | stance progress, 0 to 1 | how far through the ground contact you are |
| `L_span` | half of the stroke length | half the distance the foot scrubs backward |
| `P0x, P0y` | center of the stance stroke | where the foot rests on average |
| `delta` | height of the sine bump | how much the leg rises and sinks while pushing along the ground |
| `T_st` | stance duration | time the foot spends on the ground |
| `v_x_st, v_y_st` | foot velocity along and across the stroke | toe speed while pushing |

- Purpose: while the foot is on the ground, it slides backward at constant
  speed (`v_x_st`) along a gentle arch so the body travels forward smoothly.
- Analogy: the foot is a paddle on a treadmill; it moves back at steady speed
  while rising and falling slightly like a rolling wheel.
- Caution: as printed, the cosine uses `p_x_st`, which includes `P0x`. Use
  `p_x_st - P0x` inside the cosine so the arch is centered on the stroke.

### Full step

```text
period = T_sw + T_st
swing:  Eq. 18 -> foot point -> Eq. 10, 13, 15 -> joint angles
stance: Eq. 19 -> foot point -> Eq. 10, 13, 15 -> joint angles
```

The swing curve ends at `c11` and the stance path should start there, and the
stance end should match `c0`, so the loop is continuous. Check that the two
phases meet in position before running.

### Optional: compliant stance with impedance control (section 2.4)

```text
m*x'' + b*x' + k*x = f                                                      (Eq. 20)

f_control - f_est = m*(x_d'' - x'') + b*(x_d' - x') + k*(x_d - x)           (Eq. 21)

[ rho_virtual   ]   [ sqrt(x^2 + y^2) ]
[ theta_virtual ] = [ arctan(x / y)   ]                                     (Eq. 22)
```

| Variable | Meaning | Analogy |
|---|---|---|
| `m` | virtual mass (paper drops it in practice) | how heavy the foot feels |
| `b` | virtual damping | a shock absorber that resists speed |
| `k` | virtual stiffness | a spring that pulls the foot back to the planned path |
| `x_d, x_d', x_d''` | desired position, velocity, acceleration | where the plan says the foot should be |
| `x, x', x''` | actual position, velocity, acceleration | where the foot really is |
| `f_control` | force commanded to the leg | the push you ask for |
| `f_est` | estimated ground reaction force | what the ground is pushing back |
| `rho_virtual` | distance from hip to toe | length of an imaginary spring along the leg |
| `theta_virtual` | angle of that imaginary spring from vertical | which way the spring leans |

- Purpose: makes the leg behave like a spring and damper so landing on soft or
  hard ground does not cause a hard impact.
- Analogy: a pogo stick or car suspension. Large `k` and `b` feel stiff like a
  rigid floor; small values feel soft like a mattress.
- This needs force or torque sensing on a real robot. In MuJoCo you can
  emulate it by lowering actuator `kp`/`kv` (position stiffness and damping),
  which is how to show low versus high impedance in the video.
- Eq. 23-33 (ground stiffness identification by least squares) are **not**
  needed for the sit/stand/walk video.

---

## Quick map for the video

| Segment | Paper equations to show | Our own additions |
|---|---|---|
| Model | Eq. 1-7 | none |
| Stand | Eq. 10, 13, 15; Eq. 16-17 for torque | stand height, cubic ramp |
| Sit | Eq. 10, 13, 15; Eq. 16-17 for torque | sit height, cubic ramp |
| Walk, swing | Eq. 18 + Table 1 | rescale control points to SpotMicro |
| Walk, stance | Eq. 19 | `L_span`, `delta`, `T_sw`, `T_st` tuned for SpotMicro |
| Walk, compliance (optional) | Eq. 20-22 | emulate with lower actuator gains |
