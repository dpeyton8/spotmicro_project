# IK Paper Traceability: Şen, Bakırcıoğlu & Kalyoncu (2017) → `spot_micro_kinematics_python`

**Source paper:** Muhammed Arif Şen, Veli Bakırcıoğlu, Mete Kalyoncu, "Inverse
Kinematic Analysis Of A Quadruped Robot," *International Journal of
Scientific & Technology Research*, Vol. 6, Issue 9, September 2017, pp.
285-289.

The paper was originally hosted at `ijstr.org/final-print/sep2017/...pdf`;
that URL currently serves a maintenance page, and the ResearchGate mirror
requires a login. This document was verified against a locally-supplied
copy of the original PDF (all equation numbers, table contents, and figure
descriptions below are transcribed directly from it, not from memory or a
secondary source).

**Code under analysis:** `spot_micro_kinematics_python/` — specifically
`utilities/transformations.py`, `utilities/spot_micro_kinematics.py`, and
`spot_micro_stick_figure.py`. No equations in these files were changed to
produce this document; all findings below come from reading the paper
closely and running the existing code, both directly and through the new
tests in `tests/test_paper_traceability.py`.

---

## 1. Variable, frame, and unit conventions

| Paper symbol | Paper meaning (Table 1) | Rotation axis (Fig. 1) | Code attribute | Code meaning (docstring) | Code axis |
|---|---|---|---|---|---|
| φ (phi) | Yaw Angle of Robot | y | `self.psi` | yaw | y |
| ψ (psi) | Pitch Angle of Robot | z | `self.theta` | pitch | z |
| ω (omega) | Roll Angle of Robot | x | `self.phi` | roll | x |
| θ1 | Angle of Side Swing Joint | leg-local z0 | `q1` / `_q1` | hip joint (side-swing) | leg-local z0 |
| θ2 | Angle of Hip Joint | leg-local z1→z2 | `q2` / `_q2` | upper leg joint | same |
| θ3 | Angle of Knee Joint | leg-local z2→z3 | `q3` / `_q3` | lower leg (knee) joint | same |
| L | Length of Robot (body) | — | `body_length` | body length | — |
| W | Width of Robot (body) | — | `body_width` | body width | — |
| L1 | Length of Side Swing Joint | — | `hip_length` | hip joint length | — |
| L2 | Length of Hip Joint | — | `upper_leg_length` | upper leg length | — |
| L3 | Length of Knee Joint | — | `lower_leg_length` | lower leg length | — |
| [xm,ym,zm] | body center frame | — | `self.x, self.y, self.z` | body center position | — |
| [x4,y4,z4] | leg endpoint (foot) frame | — | `x4,y4,z4` args | foot position | — |

### The φ/θ/ψ naming trap — read this before anything else in this document

**The paper and the code use the same three symbol names (φ/phi, ψ/psi are
shared verbatim; θ/theta shares a letter) for *different* rotational
roles.** Getting this backwards is the single easiest way to misread every
other section below.

The paper defines (Table 1, Fig. 1): **φ = yaw** (about y, since the
paper's y-axis is vertical/up per Fig. 1), **ψ = pitch** (about z), **ω =
roll** (about x).

The code's `SpotMicroStickFigure` docstring defines (`spot_micro_stick_figure.py:168-170`):
**`phi` = roll** (about x), **`theta` = pitch** (about z), **`psi` = yaw**
(about y).

So `phi` (code) is **not** φ (paper) — `phi` is paper's ω. And `psi`
(code) is **not** ψ (paper) — `psi` is paper's φ. Only `theta`↔ψ share a
role (pitch) despite not sharing a letter.

This is verified, not inferred, from `self.ht_body = ... homog_rotxyz(self.phi,
self.psi, self.theta)` (`spot_micro_stick_figure.py:206`) combined with
`transformations.homog_rotxyz(x_ang,y_ang,z_ang) = rotx(x_ang) @
roty(y_ang) @ rotz(z_ang)` (`transformations.py:87-103`): the call passes
`x_ang=self.phi` (→ `rotx`, i.e. rotation about x — matches code's own
"phi: roll" docstring, and matches paper's ω since ω is also roll-about-x);
`y_ang=self.psi` (→ `roty`, rotation about y — matches code's "psi: yaw",
and matches paper's φ since φ is also yaw-about-y); `z_ang=self.theta` (→
`rotz`, rotation about z — matches code's "theta: pitch", and matches
paper's ψ since ψ is also pitch-about-z).

**Net result: the axis *roles* match exactly between paper and code (roll
is always about x, yaw always about y, pitch always about z — both use the
paper's y-up convention). Only the *symbol assigned to each role* differs.**
A reader who matches symbols by name alone (φ↔phi, ψ↔psi) will map every
rotation backwards.

### Units

The paper's Table 3 worked examples report angles in **degrees**. All code
in `spot_micro_kinematics_python` operates in **radians** throughout (see
the `d2r`/`r2d` conversion constants defined at the top of
`spot_micro_stick_figure.py` and `utilities/spot_micro_kinematics.py`).
Lengths are meters in both the paper and the code.

---

## 2. Leg numbering

The code names legs by physical position: `rightback`, `rightfront`,
`leftfront`, `leftback` (see `SpotMicroStickFigure.legs` dict,
`spot_micro_stick_figure.py:216-232`). The paper numbers legs 1-4 (Fig. 1,
Table 3) and states in Fig. 2's caption that its Denavit-Hartenberg
derivation is done for "the right front leg" specifically — but the
figure's own corner-to-number layout (which corner is 1 vs 2 vs 3 vs 4) is
only available as an image, and is not independently recoverable from the
paper's body text alone.

**This document does not assert a specific paper-leg-number ↔
code-leg-name correspondence**, because the two candidate ways of
resolving it via Table 3's own numbers disagree with each other:

- Table 3's θ1 values show **legs {1,4}** sharing one `|θ1|` magnitude and
  **legs {2,3}** sharing the other, across all three examples — consistent
  with {1,4} and {2,3} being the two left/right mirror pairs.
- The paper's own prose (Section 2, just before Eq.15) instead groups
  **legs {1,3}** and **legs {2,4}** together for the θ3 branch-selection
  ("the legs of the robot (1 and 3) and the leg of the robot (2 and 4)
  have been realized in the same kinematic structure but in different
  configurations").

These are two *different* partitions of the same four legs, both stated
or implied by the paper itself. Reconciling them requires Fig. 1's actual
corner layout, which this document does not have independent access to.
See Section 5 for how this interacts with `ikine()`'s `legs12` parameter,
and `tests/test_paper_traceability.py::TestPaperTable3Example1Unresolved`
for the concrete numeric attempt that ran into this ambiguity.

### The 180° leg orientation (a separate, hardware-level fact)

The `spot_micro_kinematics_python` README states: "Legs **1** and **3**
are rotated 180 degrees, as that's the way they are oriented on the spot
micro frame." This is a statement about how SpotMicro's *physical hips are
bolted to the chassis* — it is unrelated to the paper's IK-branch-grouping
language quoted above, even though both happen to say "legs 1 and 3." The
D-H derivation in Eq.10-14 doesn't need to know about this physical
mounting quirk at all: each leg gets an independent placement transform
(`t_rightback`/`t_rightfront`/`t_leftfront`/`t_leftback`, Eq.6-9) computed
from the body pose, and the 180° physical rotation is simply absorbed into
which corner gets which placement transform — it never appears as a
separate term in the per-leg IK math (Eq.10-17 / `ikine()`).

---

## 3. Equation-by-equation mapping (Eq. 1-17)

### Eq. 1-3 — rotation matrices ↔ `transformations.rotx/roty/rotz`

Exact structural match, verified element-by-element against the PDF.
`Rx(ω)` (Eq.1) = `rotx()` (`transformations.py:9-26`); `Ry(φ)` (Eq.2) =
`roty()` (`:29-46`); `Rz(ψ)` (Eq.3) = `rotz()` (`:49-66`). **No
discrepancy.**

### Eq. 4 — `Rxyz = Rx·Ry·Rz` ↔ `transformations.rotxyz`

`rotxyz(x_ang,y_ang,z_ang) = rotx(x_ang) @ roty(y_ang) @ rotz(z_ang)`
(`:68-84`) — same multiplication order as the paper. **No discrepancy.**

### Eq. 5 — `T_M = Rxyz · Trans(xm,ym,zm)` ↔ `SpotMicroStickFigure.__init__`

**Intentional, documented divergence.** The paper's Eq.5 rotates first,
then translates in the *already-rotated* frame — so the body ends up at
`Rxyz · [xm,ym,zm]ᵀ` in the global frame, not at `[xm,ym,zm]` itself
whenever the orientation is nonzero. The code instead does
`self.ht_body = homog_transxyz(x,y,z) @ homog_rotxyz(phi,psi,theta)`
(`spot_micro_stick_figure.py:206`) — translate in the global frame, then
rotate in place — so the body's origin lands exactly at `(x,y,z)`
regardless of orientation. The code's own comment
(`spot_micro_stick_figure.py:202-204`) explains why: *"Convention for this
class is to initialize the body pose at a x,y,z position, with a
phi,theta,psi orientation... To achieve this pose, need to apply a
homogeneous translation first, then a homogeneous rotation. If done the
other way around, a coordinate system will be rotated first, then
translated along the rotated coordinate system."* This is a deliberate
API-ergonomics choice — callers specify where the body *is*, not where a
pre-rotation reference point was — not an error.

### Eq. 6-9 — leg placement transforms ↔ `t_rightback/t_rightfront/t_leftfront/t_leftback`

Exact match. Paper: `T_rightback = T_M · [roty(π/2), (-L/2,0,W/2)]`,
`T_rightfront = T_M · [roty(π/2), (L/2,0,W/2)]`, `T_leftfront = T_M ·
[roty(-π/2), (L/2,0,-W/2)]`, `T_leftback = T_M · [roty(-π/2),
(-L/2,0,-W/2)]`. Code (`utilities/spot_micro_kinematics.py:13-87`) matches
every rotation angle and translation sign exactly; the function names even
match the paper's subscripts one-to-one. **No discrepancy.**

### Eq. 10 (`T0¹`) ↔ `t_0_to_1` — paper typo, code correct

The paper prints (transcribed directly from the PDF, page 3):

```
T0¹ = [ cos(θ1)  -sin(θ1)  0   -L1·cos(θ1)
        sin(θ1)   cos(θ1)  0   -L1·sin(θ1)
              1         0  0             0
              0         0  0             1 ]
```

Row 3 (`[1,0,0,0]`) is not a valid rotation-matrix row — for a pure
z-axis rotation the third row must be `[0,0,1,0]` (z is unchanged by a
rotation about z). This is a genuine typo in the paper. `t_0_to_1`
(`utilities/spot_micro_kinematics.py:90-118`) sidesteps it entirely by
building the block from `transformations.rotz(theta1)` (a tested, correct
3×3 rotation) plus translation `[-l1·cos(θ1), -l1·sin(θ1), 0]` — matching
the paper's rotation submatrix and translation sign exactly, just built
from a validated helper instead of a hand-typed 4×4 matrix. The code's own
comment (`:100-114`) documents this reasoning. **Paper typo; code was
already correct** (confirmed by the ground-truth FK/IK round-trip in
`tests/test_paper_traceability.py::TestEquation15AgainstGroundTruth`,
which exercises this exact matrix).

### Eq. 11 (`T1²`) ↔ `t_1_to_2` — second paper typo, code correct

The paper prints (transcribed directly from the PDF, page 3):

```
T1² = [  0   0  -1   0
        -1   0   0   0
         0   0   1   0
         0   0   0   1 ]
```

Row 3 (`[0,0,1,0]`) duplicates the `-1` already present in column 3, row
1, giving column 3 = `[-1,0,1]ᵀ` — not unit length, not orthogonal to the
other columns. Code (`utilities/spot_micro_kinematics.py:121-149`) uses:

```
t_12 = [[ 0, 0,-1, 0],
        [-1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [ 0, 0, 0, 1]]
```

moving the `1` from row 3/col 3 to row 3/col 2, which restores a valid
orthogonal (permutation) rotation matrix. The code's comment (`:130-144`)
documents this as a second suspected typo. **Paper typo; code correct.**

### Eq. 12 (`T2³`) ↔ `t_2_to_3` — exact match

Paper: `T2³ = [rotz(θ2) block, translation L2·(cos θ2, sin θ2, 0)]`.
Code (`utilities/spot_micro_kinematics.py:151-164`) matches exactly. **No
discrepancy.**

### Eq. 13 (`T3⁴`) ↔ `t_3_to_4` — exact match

Same pattern as Eq.12 with θ3, L3. Code (`:166-179`) matches exactly. **No
discrepancy.**

### Eq. 14 (`T0⁴ = T0¹T1²T2³T3⁴`) ↔ `t_0_to_4` — exact match

Same composition order in code: `t_0_to_1 @ t_1_to_2 @ t_2_to_3 @
t_3_to_4` (`:181-196`). **No discrepancy** — any numeric difference
against the paper's Table 3 element formulas (`m11`...`m44`) traces back
only to the Eq.10/Eq.11 typos above, not to this composition step.

### Eq. 15 (θ1) ↔ code's `q1` — genuine formula error in the paper, resolved empirically

The paper prints:

```
θ1 = -atan2(-y4,x4) - atan2(√(x4²+y4²-L1²), -L1)
```

Code (`utilities/spot_micro_kinematics.py:230`):

```
q1 = atan2(y4, x4) + atan2(sqrt(x4**2+y4**2-l1**2), -l1)
```

Using the standard identity `atan2(-y,x) = -atan2(y,x)`, the paper's
formula algebraically reduces to `θ1 = atan2(y4,x4) - atan2(√(...), -L1)`
— a single sign flip on the second term versus code's `+`. That alone
would be a simple, arguable convention difference. **It isn't just
that.** `tests/test_paper_traceability.py::TestEquation15AgainstGroundTruth`
settles it empirically: starting from a known `(θ1,θ2,θ3) =
(20°,15°,-25°)`, running it through the paper's own (typo-corrected, per
Eq.10/11 above) forward kinematics (`t_0_to_4`) to get `(x4,y4,z4)`, then
inverting —

- **code's `q1` formula recovers `θ1 = 20.0000°` exactly** (to float
  precision), while
- **the paper's literal Eq.15 formula gives `θ1 ≈ -186.69°`** — not a
  sign flip, not a coterminal-angle (`mod 360`) match, not explained by
  the algebraic reduction above.

**Conclusion: code's `q1` is the mathematically correct inverse of the
paper's own forward kinematics chain. The paper's printed Eq.15 has a
genuine formula error, not a valid alternate sign convention.** The code's
inline comment at `:226-229` (*"there seem to be two errors... y4 should
not have a negative sign... entire equation should be multiplied by
-1"*) describes the fix as two separate steps; the algebraic reduction
above shows the *net* effect is a single sign flip on the second `atan2`
term, so the comment's narrative doesn't line up step-for-step with the
actual diff — but the destination formula (code's `q1`) is verified
correct regardless of how the original author arrived at it.

### Eq. 16 (θ2) ↔ code's `q2` — exact match

Paper: `θ2 = atan2(z4, √(x4²+y4²-L1²)) - atan2(L3·sin θ3, L2+L3·cos θ3)`.
Code (`:224`): `q2 = atan2(z4, sqrt(x4**2+y4**2-l1**2)) -
atan2(l3*sin(q3), l2+l3*cos(q3))`. **Exact match, no discrepancy.**

### Eq. 17 (θ3, D) ↔ code's `q3`, `D` — D matches exactly; branch-grouping convention needs care

Paper: `D = (x4²+y4²-L1²+z4²-L2²-L3²)/(2·L2·L3)`. Code (`:217`): `D =
(x4**2+y4**2+z4**2-l1**2-l2**2-l3**2)/(2*l2*l3)` — algebraically identical
(terms reordered). **D formula matches exactly.**

Branch selection: paper says `θ3 = atan2(-√(1-D²), D)` for "legs 1 and 3",
`θ3 = atan2(√(1-D²), D)` for "legs 2 and 4". Code: `legs12=True → q3 =
atan2(sqrt(1-D**2), D)` (the **positive**-sqrt branch), `legs12=False → q3
= atan2(-sqrt(1-D**2), D)` (**negative**-sqrt branch) — see
`utilities/spot_micro_kinematics.py:219-222`.

Code sets `legs12=True` for `leg_rightback` and `leg_rightfront`, and
`legs12=False` for `leg_leftfront` and `leg_leftback`
(`spot_micro_stick_figure.py:218-232`) — a **right-vs-left** split. This
is a physically sensible choice on its own terms (left and right legs are
mirror images and need opposite knee-bend sign for a visually consistent
gait), and the `SpotMicroLeg` docstring even calls this out directly:
*"leg is 1 or 2 (rightback or rightfront) or 3 or 4 (leftfront or
leftback)"* (`:24-25`) — i.e. the code's own internal "1,2 vs 3,4"
shorthand is a right/left split, which is **not** the same partition as
the paper's "(1,3) vs (2,4)" wording. See Section 2 above and Section 5
below for why this correspondence could not be pinned down further.

---

## 4. Intentional SpotMicro differences

- **Eq.5 order reversal** (translate-then-rotate vs. paper's
  rotate-then-translate) — see Section 3, Eq.5. Deliberate, documented in
  code.
- **Dimensions.** SpotMicro's real hardware (`spot_micro_stick_figure.py:187-191`):
  `hip_length=0.055`, `upper_leg_length=0.1075`, `lower_leg_length=0.130`,
  `body_width=0.078`, `body_length=0.186` (all meters) — versus the
  paper's own illustrative example (`L1=0.1, L2=0.4, L3=0.4, L=1, W=0.4`,
  Table 1). These are just different numeric inputs to the *same*
  equations, not a structural or geometric difference — the D-H chain and
  IK formulas are dimension-agnostic.
- **The 180° leg-1/leg-3 physical mounting** (README) — a hardware fact
  about how hips are bolted on, absorbed into which corner gets which
  placement transform (Eq.6-9), never appearing as a separate term in the
  per-leg D-H/IK math. See Section 2.

---

## 5. Ambiguities, singularities, multiple IK branches, unreachable poses

**Leg-numbering ambiguity (Section 2, Section 3 Eq.17).** The paper's
"(1,3)"/"(2,4)" branch-grouping language and its own Table 3 numbers
(which show mirror symmetry across `{1,4}`/`{2,3}` instead) don't agree
with each other, and neither is independently verifiable against code's
right/left `legs12` split without Fig. 1's actual corner layout. Left
open; see `tests/test_paper_traceability.py::TestPaperTable3Example1Unresolved`.

**Table 3 reproduction — open discrepancy.** Attempting to reproduce
Table 3, Example 1 in full (global foot target `[0,-0.65,0]`, body at the
origin, φ=0°/ψ=-15°/ω=0°, paper's own dimensions) by composing Eq.5-9 —
each individually verified correct against the paper in Section 3 — with
`ikine()` does **not** reproduce the paper's published angles, and the gap
is not explained by the Eq.15 sign issue (which is tested and resolved
*in code's favor*, in isolation, by
`TestEquation15AgainstGroundTruth`). Concretely:

- Two of the four legs (`rightback`, `leftback` under the paper's literal
  -15° pitch sign; the *other* two legs under the opposite sign) land
  outside `ikine()`'s valid domain entirely (`D > 1`). This was checked
  under both signs of the pitch value and the infeasible pair simply
  swaps — it is not an artifact of guessing the wrong pitch sign.
- For the two legs that are within domain, the resulting θ1
  (≈26.4° or ≈-171.1°, depending on branch) matches neither of the
  paper's two `|θ1|` values for this example (7.5883° / 11.5735°), by a
  margin far larger than the already-resolved Eq.15 sign flip.

**Most likely explanation:** the paper's Table 3 was generated by the
authors' actual MATLAB program, which — like Eq.15 — may differ from the
equations as typeset in the paper's text in ways not recoverable from the
text alone. This is a known, common gap between a paper's typeset math and
its actual code, and it's consistent with the Eq.10/Eq.11/Eq.15 errors
already found directly in the typeset equations. **This is left as an
explicitly open, unresolved discrepancy** — see
`tests/test_paper_traceability.py::TestPaperTable3Example1Unresolved` for
the exact reproduction attempt and its result as an executable record.

**Singularity at `D = ±1`.** `θ3 = 0` (D=1, fully extended leg) or `θ3 =
π` (D=-1, fully folded) are boundary configurations where the elbow-up and
elbow-down branches coincide — a rank-deficient Jacobian point. `ikine()`
still returns a valid answer exactly at the boundary (see
`TestWorkspaceBoundariesAndSingularities.test_fully_extended_leg_singularity_d_equals_one`,
added in Task 4).

**Unreachable-target domain errors.** `ikine()` uses `math.sqrt`, which
raises `ValueError` (not `nan`) on a negative argument — not a caught or
translated exception, so callers see Python's raw domain error. Two
distinct conditions trigger it: `x4²+y4² < L1²` (target inside the
hip-swing offset cylinder — `q1`'s second `atan2` argument goes negative
under the sqrt) and `|D| > 1` (target beyond max/min leg reach in the
L2/L3 plane — `q3`'s `sqrt(1-D**2)` argument goes negative). Both are
exercised directly by `TestWorkspaceBoundariesAndSingularities` (Task 4).

**Multiple IK solutions.** For any reachable target, both
`legs12=True` and `legs12=False` yield a distinct, FK-consistent solution
(elbow-up vs. elbow-down) — `ikine()` takes `legs12` as a caller-supplied
choice, it does not auto-select a "best" branch. See
`TestIKBranchSelection` (Task 4).

---

## 6. Numeric tolerance notes

- **Paper-comparison tests** (`TestEquation15AgainstGroundTruth`): paper
  reports angles to 4 decimal places in degrees; the ground-truth check
  here uses a much tighter `delta=1e-9` radians since it's comparing
  against a synthetic, exactly-known angle via pure floating-point
  forward/inverse kinematics, not against the paper's rounded table
  values.
- **FK→IK→FK round trips** (`TestFKIKRoundTrip`, `TestIKBranchSelection`'s
  consistency check): `atol=1e-9` radians. This is a pure floating-point
  round trip through the same equations (not an independent solve), so
  error should be within a couple orders of magnitude of machine epsilon
  (~2.2e-16); `1e-9` leaves generous headroom while still being a
  meaningful bound.
- **Symmetry tests** (`TestLeftRightFrontBackSymmetry`): `atol=1e-9`
  radians, same reasoning — comparing two branches of the same
  floating-point computation, not independently-derived values.
- **Domain/singularity tests**
  (`TestWorkspaceBoundariesAndSingularities`): exact `ValueError`
  raising/non-raising behavior at the boundary, no floating tolerance
  needed for the raise/no-raise assertions themselves; the one numeric
  check there (`D=1` singularity) uses `delta=1e-9` for the same reason
  as the round-trip tests.

---

## 7. Test coverage

- `TestEquation15AgainstGroundTruth` — Section 3's Eq.15 discrepancy
  claim (code correct, paper's printed formula wrong).
- `TestPaperTable3Example1Unresolved` — Section 5's open Table 3
  reproduction discrepancy, as an executable, reproducible record of what
  was tried.
- `TestFKIKRoundTrip` — Section 3's "no discrepancy" claims for Eq.10-14
  (forward kinematics) taken together with Eq.15-17 (inverse kinematics),
  at SpotMicro's real hardware dimensions, across all four legs and
  nonzero roll/pitch/yaw.
- `TestLeftRightFrontBackSymmetry` — Section 2's left/right mirror-pair
  observation, reproduced directly from code's default stance.
- `TestIKBranchSelection` — Section 5's "multiple IK solutions" claim.
- `TestWorkspaceBoundariesAndSingularities` — Section 5's singularity and
  unreachable-target claims.
