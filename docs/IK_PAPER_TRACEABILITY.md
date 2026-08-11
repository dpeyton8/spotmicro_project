# IK Paper Traceability: Şen, Bakırcıoğlu & Kalyoncu (2017) → `spot_micro_kinematics_python`

**Source paper:** Muhammed Arif Şen, Veli Bakırcıoğlu, Mete Kalyoncu, "Inverse
Kinematic Analysis Of A Quadruped Robot," *International Journal of
Scientific & Technology Research*, Vol. 6, Issue 9, September 2017,
ISSN 2277-8616, pp. 285-289. Equations cited as Eq.1-Eq.17 throughout;
tables cited as Table 1-3; figures as Fig.1-2, all per the paper's own
numbering. Specific page references are given per section below.

The paper was originally hosted at `ijstr.org/final-print/sep2017/...pdf`;
that URL currently serves a maintenance page, and the ResearchGate mirror
requires a login. This document was verified against a locally-supplied
copy of the original PDF — **every page was visually inspected as a
rendered image**, not read via automated text extraction alone (an
earlier pass using text extraction garbled three of the matrices on page
3; re-rendering that page, and subsequently pages 2 and 4, as images
resolved this — see Section 3 and Section 7 for what changed).

**Code under analysis:** `spot_micro_kinematics_python/` — specifically
`utilities/transformations.py`, `utilities/spot_micro_kinematics.py`, and
`spot_micro_stick_figure.py`. No equations in these files were changed to
produce this document; all findings below come from reading the paper
closely, reading the sibling C++/URDF/MJCF implementations in this same
repository, and running the existing code and this document's paired
tests (`tests/test_paper_traceability.py`).

### How to read claims in this document

Statements below are one of four kinds, and are labeled where the
distinction matters (mainly Sections 2, 5, 6, and 9):

- **Direct** — a fact stated explicitly in the paper's text, or a value
  printed in its tables/equations, transcribed as-is.
- **Visual** — an interpretation of what a figure (in the paper or in
  this repository's assets) shows, where the figure itself doesn't carry
  an explicit caption stating the conclusion.
- **Test-demonstrated** — a claim backed by a specific, named test in
  `tests/test_paper_traceability.py` that runs as part of this repo's
  suite; re-running that test re-verifies the claim.
- **Hypothesis** — a plausible explanation that is *not* demonstrated,
  offered because leaving a discrepancy unexplained would be less useful
  than naming the most likely cause, but explicitly not proven.

---

## 1. Variable, frame, and unit conventions

*(Direct, from Table 1 and Fig. 1, p.286, plus code docstrings.)*

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

The paper defines (Table 1, Fig. 1, p.286): **φ = yaw** (about y, since the
paper's y-axis is vertical/up per Fig. 1), **ψ = pitch** (about z), **ω =
roll** (about x).

The code's `SpotMicroStickFigure` docstring defines (`spot_micro_stick_figure.py:168-170`):
**`phi` = roll** (about x), **`theta` = pitch** (about z), **`psi` = yaw**
(about y).

So `phi` (code) is **not** φ (paper) — `phi` is paper's ω. And `psi`
(code) is **not** ψ (paper) — `psi` is paper's φ. Only `theta`↔ψ share a
role (pitch) despite not sharing a letter.

This is [Test-demonstrated], not merely inferred, from `self.ht_body = ...
homog_rotxyz(self.phi, self.psi, self.theta)` (`spot_micro_stick_figure.py:206`)
combined with `transformations.homog_rotxyz(x_ang,y_ang,z_ang) =
rotx(x_ang) @ roty(y_ang) @ rotz(z_ang)` (`transformations.py:87-103`):
the call passes `x_ang=self.phi` (→ `rotx`, i.e. rotation about x —
matches code's own "phi: roll" docstring, and matches paper's ω since ω
is also roll-about-x); `y_ang=self.psi` (→ `roty`, rotation about y —
matches code's "psi: yaw", and matches paper's φ since φ is also
yaw-about-y); `z_ang=self.theta` (→ `rotz`, rotation about z — matches
code's "theta: pitch", and matches paper's ψ since ψ is also
pitch-about-z). `TestFKIKRoundTrip`'s nonzero-roll/pitch/yaw tests
exercise all three of these mappings and pass, which is consistent with
(though not, on its own, an exhaustive proof of) this reading.

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

*(Combines Direct, Visual, and Test-demonstrated evidence — see labels
inline.)*

The code names legs by physical position: `rightback`, `rightfront`,
`leftfront`, `leftback` (see `SpotMicroStickFigure.legs` dict,
`spot_micro_stick_figure.py:216-232`). The paper numbers legs 1-4 in
Fig. 1 (p.285) and Table 3 (p.288), and states in Fig. 2's caption
(p.287) that its Denavit-Hartenberg derivation is done for "the right
front leg" specifically.

### What the figure shows [Visual]

Fig. 1 (reproduced in this repository as
`spot_micro_kinematics_python/assets/robot_geometry.png`, per the
package README's note that this asset is "taken from the paper above")
draws four numbered leg positions around a horizontal body bar: **4** and
**1** at the left end of the drawing (4 above, 1 below), **3** and **2**
at the right end (3 above, 2 below). The body's `x_m` axis (red) is drawn
horizontal, separating the left-end cluster from the right-end cluster;
`y_m` (green) is vertical/up; `z_m` (blue) is the third axis, drawn as a
diagonal "into the page" arrow, separating the two legs within each
end-cluster. Per Table 1's own transform formulas (Eq.6-9, Section 3
below), `x_m` carries the body-length (front/back) offset and `z_m`
carries the body-width (left/right) offset — so **the figure's two
end-clusters, {4,1} and {3,2}, are separated along the front/back axis,
and the two legs within each cluster are separated along the left/right
axis.**

### What Table 3's own numbers show [Test-demonstrated]

Table 3's three worked examples all show leg 1 and leg 4 sharing one
`|θ1|` magnitude, and legs 2 and 3 sharing a different one, across all
three examples (e.g. Example 1: legs 1,4 both give `|θ1|=7.5883°`; legs
2,3 both give `|θ1|=11.5735°`). On its own, this numeric pattern only
establishes that legs 1↔4 and legs 2↔3 are mirror-symmetric *pairs* in
whatever coordinate system the paper's program used — a purely numeric
fact, with no coordinate frame drawn. It does **not**, by itself,
establish that these pairs correspond to Fig. 1's spatial front/back
end-clusters specifically, as opposed to some other pairing; that
spatial identification comes from the figure (previous subsection), not
from the numbers. `TestPaperTable3ExamplesUnresolved` in the test suite
reproduces these Table 3 numbers directly from the PDF and preserves
this pattern as transcribed (see Section 5 for what does and doesn't
reproduce computationally).

**Conclusion: {1,4} and {2,3} are the figure's two end-groups, per Fig.
1's spatial layout (previous subsection). Table 3's matching `|θ1|`
pairing is *consistent with* that same partition — the same two pairs of
legs, {1,4} and {2,3}, turn up as the mirror-symmetric pairs in the
numbers too — but the numeric pairing alone does not independently
establish the front/back spatial interpretation; it only confirms that
whichever pairing the figure shows is at least self-consistent with a
real mirror-symmetry in the paper's own worked examples. The paper's
separate branch-selection grouping — "the legs of the robot (1 and 3)
and the leg of the robot (2 and 4)" (p.287, prose immediately before
Eq.15) — is a *different* partition of the same four legs: it pairs one
leg from each end-cluster together, not the two legs within an
end-cluster.** These are not reconcilable into a single grouping; the
paper uses both, for different purposes (Fig. 1's spatial layout vs.
Eq.17's branch selection), and nothing in the paper claims they should
coincide.

### What remains genuinely uncertain

Confirming *which* end-cluster is "front" vs. "back", and *which* leg
within a cluster is "right" vs. "left", requires knowing the robot's
forward-facing direction relative to Fig. 1's drawing — and **neither
Fig. 1 nor Fig. 2 labels this**. Fig. 2's caption says its derivation is
for "the right front leg," but Fig. 2 itself shows only a single,
unnumbered leg-joint chain (θ1-θ3, L1-L3, frames x0-x4) with no body
context tying it back to one of Fig. 1's four numbered corners. So even
combining both figures, **this document cannot establish a definitive
mapping from paper leg numbers {1,2,3,4} to SpotMicro's named corners
(`rightback`/`rightfront`/`leftfront`/`leftback`)** — only that {1,4} and
{2,3} are the two end-groups, and within each group the two legs are
left/right mirrors. This is listed again in Section 9 as an item that
would need another source (e.g. the paper's original MATLAB code, or an
author confirmation) to resolve.

### The 180° leg orientation (a separate, hardware-level fact)

The `spot_micro_kinematics_python` README states: "Legs **1** and **3**
are rotated 180 degrees, as that's the way they are oriented on the spot
micro frame." This is a statement about how SpotMicro's *physical hips are
bolted to the chassis* — it is unrelated to the paper's IK-branch-grouping
language quoted above, even though both happen to say "legs 1 and 3" (and
this README statement is about SpotMicro's own leg numbering convention,
not necessarily the same numbering as the paper's Fig. 1 — see Section 9).
The D-H derivation in Eq.10-14 doesn't need to know about this physical
mounting quirk at all: each leg gets an independent placement transform
(`t_rightback`/`t_rightfront`/`t_leftfront`/`t_leftback`, Eq.6-9) computed
from the body pose, and the 180° physical rotation is simply absorbed into
which corner gets which placement transform — it never appears as a
separate term in the per-leg IK math (Eq.10-17 / `ikine()`).

---

## 3. Equation-by-equation mapping (Eq. 1-17)

*(Direct transcriptions from the PDF unless noted; every matrix below was
re-verified against a rendered image of its page, not text extraction —
see the provenance note at the end of this section.)*

### Eq. 1-3 — rotation matrices ↔ `transformations.rotx/roty/rotz`

Exact structural match, verified element-by-element against the PDF (p.286).
`Rx(ω)` (Eq.1) = `rotx()` (`transformations.py:9-26`); `Ry(φ)` (Eq.2) =
`roty()` (`:29-46`); `Rz(ψ)` (Eq.3) = `rotz()` (`:49-66`). **No
discrepancy.**

### Eq. 4 — `Rxyz = Rx·Ry·Rz` ↔ `transformations.rotxyz`

`rotxyz(x_ang,y_ang,z_ang) = rotx(x_ang) @ roty(y_ang) @ rotz(z_ang)`
(`:68-84`) — same multiplication order as the paper (p.286). **No discrepancy.**

### Eq. 5 — `T_M = Rxyz · Trans(xm,ym,zm)` ↔ `SpotMicroStickFigure.__init__`

**Intentional, documented divergence.** The paper's Eq.5 (p.286) rotates
first, then translates in the *already-rotated* frame — so the body ends
up at `Rxyz · [xm,ym,zm]ᵀ` in the global frame, not at `[xm,ym,zm]` itself
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

[Test-demonstrated]: `TestPaperTable3ExamplesUnresolved.test_example_3_all_four_legs_infeasible_under_both_t_m_orders`
tries both orders on the one Table 3 example with nonzero body
translation (Example 3, the only one where the order choice could
possibly matter) and finds neither order makes it reachable — so this
order difference does not explain Section 5's Table 3 discrepancy, on the
one example where it was even possible for it to.

### Eq. 6-9 — leg placement transforms ↔ `t_rightback/t_rightfront/t_leftfront/t_leftback`

Exact match (p.286). Paper: `T_rightback = T_M · [roty(π/2), (-L/2,0,W/2)]`,
`T_rightfront = T_M · [roty(π/2), (L/2,0,W/2)]`, `T_leftfront = T_M ·
[roty(-π/2), (L/2,0,-W/2)]`, `T_leftback = T_M · [roty(-π/2),
(-L/2,0,-W/2)]`. Code (`utilities/spot_micro_kinematics.py:13-87`) matches
every rotation angle and translation sign exactly; the function names even
match the paper's subscripts one-to-one. **No discrepancy.**

### Eq. 10 (`T0¹`) ↔ `t_0_to_1` — paper typo, code correct

The paper prints (p.287, transcribed directly from a rendered image of
the page — text extraction alone garbled this matrix in an earlier pass,
see the provenance note below):

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
already correct** — [Test-demonstrated] by
`TestIndependentForwardKinematicsReference`, which recomputes this exact
transform via a completely independent rotation formula (Rodrigues', not
`transformations.py`'s trig matrices) and matches production output to
machine epsilon.

### Eq. 11 (`T1²`) ↔ `t_1_to_2` — second paper typo, code correct

The paper prints (p.287, from a rendered image):

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

Paper (p.287): `T2³ = [rotz(θ2) block, translation L2·(cos θ2, sin θ2, 0)]`.
Code (`utilities/spot_micro_kinematics.py:151-164`) matches exactly. **No
discrepancy.**

### Eq. 13 (`T3⁴`) ↔ `t_3_to_4` — exact match

Same pattern as Eq.12 with θ3, L3 (p.287). Code (`:166-179`) matches
exactly. **No discrepancy.**

### Eq. 14 (`T0⁴ = T0¹T1²T2³T3⁴`) ↔ `t_0_to_4` — exact match

Same composition order in code: `t_0_to_1 @ t_1_to_2 @ t_2_to_3 @
t_3_to_4` (`:181-196`). **No discrepancy** — any numeric difference
against the paper's Table 3 element formulas (`m11`...`m44`, p.287)
traces back only to the Eq.10/Eq.11 typos above, not to this composition
step.

### Eq. 15 (θ1) ↔ code's `q1` — genuine formula error in the paper, resolved empirically

The paper prints (p.287):

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

**Conclusion [Test-demonstrated]: code's `q1` is the mathematically
correct inverse of the paper's own forward kinematics chain. The paper's
printed Eq.15 has a genuine formula error, not a valid alternate sign
convention.** The code's inline comment at `:226-229` (*"there seem to be
two errors... y4 should not have a negative sign... entire equation
should be multiplied by -1"*) describes the fix as two separate steps;
the algebraic reduction above shows the *net* effect is a single sign
flip on the second `atan2` term, so the comment's narrative doesn't line
up step-for-step with the actual diff — but the destination formula
(code's `q1`) is verified correct regardless of how the original author
arrived at it.

### Eq. 16 (θ2) ↔ code's `q2` — exact match

Paper (p.288): `θ2 = atan2(z4, √(x4²+y4²-L1²)) - atan2(L3·sin θ3,
L2+L3·cos θ3)`. Code (`:224`): `q2 = atan2(z4, sqrt(x4**2+y4**2-l1**2)) -
atan2(l3*sin(q3), l2+l3*cos(q3))`. **Exact match, no discrepancy.**

### Eq. 17 (θ3, D) ↔ code's `q3`, `D` — D matches exactly; branch-grouping convention needs care

Paper (p.288): `D = (x4²+y4²-L1²+z4²-L2²-L3²)/(2·L2·L3)`. Code (`:217`): `D =
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
document deliberately avoids calling the two `legs12` branches
"elbow-up"/"elbow-down" or any other physical label (see Section 5,
"Multiple IK solutions") — what's verifiable from the code alone is only
that it's a right-vs-left split at the naming level, and a
positive-vs-negative sign choice at the math level. The `SpotMicroLeg`
docstring's own internal shorthand — *"leg is 1 or 2 (rightback or
rightfront) or 3 or 4 (leftfront or leftback)"* (`:24-25`) — is **not**
the same partition as the paper's "(1,3) vs (2,4)" wording (Section 2
established {1,4}/{2,3} as the figure's own grouping, and the branch
grouping is a third, different partition again). Whether the paper's
"(1,3)/(2,4)" branch choice and code's right/left `legs12` choice
actually correspond to the same underlying leg positions is not
resolvable from the paper's text and figures alone — see Section 2 and
Section 9.

### Provenance note: visual re-verification of the whole paper

Every equation and table cited above (Eq.1-17, Table 1-3) was re-checked
against a page image rendered directly from the source PDF (via
`pdftoppm`), not against the initial automated text-extraction pass. The
text-extraction pass had garbled three matrices on p.287 (T0¹, T1², T2³ —
rendered as illegible placeholder characters) while leaving the rest of
that page and all of pages 1, 2, and 4 legible; re-rendering p.287, and
subsequently pages 2 and 4, as images resolved all of it, and **no
additional transcription errors were found beyond what the initial
legible portions already showed** — specifically, Table 1, Eq.1-9 (p.286)
and Eq.16-17 plus all of Table 3's numeric values (p.288) were re-checked
against page images and matched the original transcription exactly, with
no digit, sign, subscript, or heading differences found. This means the
Eq.10/Eq.11 "typos" documented above are treated as confirmed printing
errors in the paper itself (visually confirmed present in the rendered
page image, not artifacts of automated extraction), while the *initial*
difficulty reading them was a text-extraction artifact on this
document's end, now resolved and superseded by direct image inspection.

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
  IK formulas are dimension-agnostic. [Test-demonstrated]: the same
  independent-FK cross-check that validates the paper's dimensions in
  Section 3 is also run at SpotMicro's real dimensions
  (`TestIndependentForwardKinematicsReference.test_independent_reference_matches_at_spotmicro_dimensions`).
- **The 180° leg-1/leg-3 physical mounting** (README) — a hardware fact
  about how hips are bolted on, absorbed into which corner gets which
  placement transform (Eq.6-9), never appearing as a separate term in the
  per-leg D-H/IK math. See Section 2.

---

## 5. Ambiguities, singularities, multiple IK branches, unreachable poses

**Leg-numbering ambiguity (Section 2).** Resolved in part: {1,4} and
{2,3} are confirmed (by both Fig. 1's layout and Table 3's own numbers)
to be the figure's two front/back end-groups. Not resolved: which
end-group is physically front vs. back, which leg within a group is
right vs. left, and whether the paper's "(1,3)/(2,4)" branch-selection
grouping (a third, different partition) corresponds to code's right/left
`legs12` split. See Section 2 and Section 9.

**Table 3 reproduction — open discrepancy, now checked against all
three examples.** Attempting to reproduce each of Table 3's three worked
examples in full (global foot target, body pose, paper's own dimensions)
by composing Eq.5-9 — each individually verified correct against the
paper in Section 3 — with `ikine()` does **not** reproduce the paper's
published angles for *any* of the three, and the gap is not explained by
the Eq.15 sign issue (tested and resolved *in code's favor*, in
isolation, by `TestEquation15AgainstGroundTruth`). [Test-demonstrated],
via `TestPaperTable3ExamplesUnresolved`:

- **Example 1**: 2 of 4 legs land outside `ikine()`'s valid domain
  entirely (`D > 1`). This was checked under both signs of the pitch
  value and the infeasible pair simply swaps — not a sign-convention
  artifact. The 2 in-domain legs' θ1 (≈26.4° or ≈-171.1°, depending on
  branch) matches neither of the paper's two `|θ1|` values for this
  example (7.5883° / 11.5735°), by a margin far larger than the
  already-resolved Eq.15 sign flip.
- **Example 2**: also 2 of 4 legs infeasible (a different pair than
  Example 1, but the same *kind* of partial failure).
- **Example 3** — the only example with nonzero body translation *and*
  all three rotation angles nonzero: **all four legs** land outside the
  valid domain (`D` ranging from ≈2.2 to ≈4.2), under *both* candidate
  `T_M` orders from Eq.5 (ruling out the Eq.5 order ambiguity as the
  explanation, on the one example where it could have mattered).

**[Hypothesis], not demonstrated:** the paper's Table 3 was generated by
the authors' own MATLAB program, which — like the printed Eq.15 — may
differ from the equations exactly as typeset in the paper's text. This
is offered because it's consistent with the Eq.10/Eq.11/Eq.15 errors
already *confirmed* directly in the typeset equations (Section 3), and
because it is the null result of an extensive, methodical search (three
examples, both `T_M` orders, both pitch-sign conventions all ruled out
individually) — but it is not proven, and this document does not claim
otherwise. Whether Table 3 is reproducible with the authors' original
program cannot be determined without that program; see Section 9.

**Singularity at `D = ±1` — three distinct claims, kept in three
separate places.** It's easy to blur "the leg is mathematically singular
here," "the numbers get badly conditioned as you approach that point,"
and "the code throws an exception near that point" into one claim. They
are not the same claim, and this document (and its tests) keep them
separate:

1. **Exact mathematical singularity, demonstrated directly in joint
   space.** [Test-demonstrated] by `TestJacobianSingularity`: a
   finite-difference Jacobian of the foot position with respect to
   `(q1,q2,q3)`, evaluated via `t_0_to_4` at *fixed joint angles* — `θ3=0`
   exactly (fully extended, `D=+1`) and `θ3=±π` exactly (fully folded,
   `D=-1` — governed by the minimum planar reach `|L2-L3|`, not `L2+L3`),
   each at three different, arbitrary, nonsingular `(θ1,θ2)` pairs, plus
   a regular (non-boundary) configuration as a control. This
   *deliberately does not go through `ikine()` at all* — no Cartesian
   target, no boundary rounding, nothing but the forward-kinematics
   equations and calculus. Result: the regular configuration's smallest
   singular value is a substantial fraction of the largest
   (ratio ≈ 0.13); every `θ3=0` or `θ3=±π` configuration's smallest
   singular value is ten-or-more orders of magnitude below the largest
   (ratio ≈ 1e-13 to 5e-11), consistent with the true mathematical value
   being exactly zero and the measured value being finite-difference/
   floating-point noise. See the singular values recorded directly in
   `TestJacobianSingularity`'s docstring and assertion messages. **This
   is the actual proof of rank-deficiency; the other two items below are
   not.**
2. **Near-boundary numerical behavior**, i.e. what happens to the
   Jacobian or to `ikine()` at configurations *close to but not at* a
   singularity, is a genuinely different, softer question (how conditioning
   degrades as you approach) that this document does not attempt to
   characterize quantitatively — only the exact points above are analyzed.
3. **Python `ikine()`'s exact-boundary rounding failure** at the
   constructed `D=-1` Cartesian target is a separate, IK-specific
   implementation behavior — see the next paragraph — and is
   [Test-demonstrated] by `TestWorkspaceBoundariesAndSingularities`, not
   by `TestJacobianSingularity`. It shows that reaching the exact `D=-1`
   *Cartesian target* through `ikine()` fails for float-rounding reasons;
   it says nothing by itself about whether the underlying configuration
   is a mathematical singularity (item 1 answers that, independently).

`ikine()` returns a finite, FK-consistent answer exactly at the `D=+1`
joint-space singularity when reached from a Cartesian target (see
`TestWorkspaceBoundariesAndSingularities`'s extension test) — the
mapping from joint velocities to foot velocity still loses a degree of
freedom there even though `ikine()` itself doesn't error out.

**Unreachable-target domain errors and floating-point boundary behavior.**
`ikine()` uses `math.sqrt`, which raises `ValueError` (not `nan`) on a
negative argument — not a caught or translated exception, so callers see
Python's raw domain error. Two distinct conditions trigger it: `x4²+y4² <
L1²` (target inside the hip-swing offset cylinder — `q1`'s second `atan2`
argument goes negative under the sqrt) and `|D| > 1` (target beyond
max/min leg reach in the L2/L3 plane — `q3`'s `sqrt(1-D**2)` argument
goes negative). Both, plus their exact-boundary and near-boundary cases
on both sides, are [Test-demonstrated] by
`TestWorkspaceBoundariesAndSingularities`. One boundary case is worth
calling out specifically, as item 3 above: **the exact analytic `D=-1`
point (minimum planar reach, `|L2-L3|`) raises `ValueError` in practice**
when approached via `ikine()` from a constructed Cartesian target, even
though item 1 above independently establishes that the underlying
joint-space configuration is a valid, reachable (if singular)
configuration — float rounding computes `D` as very slightly less than
`-1` (about `-1 - 4.4e-16` at SpotMicro's dimensions) rather than exactly
`-1` when the target is built from the analytic formula and run through
`ikine()`. This is a concrete, reproduced instance of the "`D` slightly
outside `[-1,1]` purely from numerical rounding" scenario — an
**IK-specific rounding artifact**, not evidence about the Jacobian one
way or the other (item 1 settles the Jacobian question on its own, via a
route that never touches `ikine()` or this rounding behavior).
**Recommended follow-up work (not made in this documentation PR):**
SpotMicro's own C++ port of this same kinematics library already handles
this — see Section 6 — by clamping `D` to `[-1,1]` and the `q1`/`q2`
shared `sqrt` argument to `≥0` before taking the square root
(`spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/src/utils.cpp`,
`ikine()`). Porting that same clamping to the Python `ikine()` would make
it robust to exactly this class of float-rounding domain error, and
would be a reasonable, small, separate change — but changing
production kinematics code is out of scope for this documentation PR
per the original request, and is called out here as a recommendation
rather than made.

**Multiple IK solutions.** [Test-demonstrated] by `TestIKBranchSelection`,
now across a representative spread of five reachable targets plus one
near (but not at) the `D=1` boundary, not just a single point: for any
reachable target away from the `D=±1` singularities, `legs12=True` and
`legs12=False` yield two distinct, FK-consistent solutions.
**What `legs12` controls, precisely, and no more than this:** it is the
sign passed to `atan2()` when computing `q3` from `D` — a
positive-sqrt-branch vs. negative-sqrt-branch choice in Eq.17's formula,
nothing more. This document does not call the two branches
"elbow-up"/"elbow-down," "knee-forward"/"knee-backward," or any other
physically-labeled name — doing so would require knowing which way the
physical knee joint bends on the real chassis, which cannot be
determined from this code and figures alone (see Section 9). `ikine()`
never auto-selects between the two; the caller supplies `legs12`.

---

## 6. Cross-check against other implementations in this repository

*(All [Direct]/[Test-demonstrated] — this repository contains three other
implementations of essentially the same kinematics, which this section
compares against the Python code without modifying any of them.)*

### C++ (`spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/`)

`src/utils.cpp` and `src/spot_micro_leg.cpp` implement the same D-H chain
and `ikine()` as the Python code, function-for-function
(`ht0To1`/`ht1To2`/`ht2To3`/`ht3To4`/`ht0To4`/`ikine`, plus
`htLegRightBack`/`htLegRightFront`/`htLegLeftFront`/`htLegLeftBack`). The
leg-naming/branch convention matches exactly: `right_back_leg_` and
`right_front_leg_` are constructed with `is_leg_12=true`, `left_front_leg_`
and `left_back_leg_` with `is_leg_12=false`
(`src/spot_micro_kinematics.cpp:30-33`) — identical to Python's
`leg12=True`/`False` assignment (Section 3, Eq.17). Body/link dimensions
in `spot_micro_motion_cmd/config/spot_micro_motion_cmd.yaml` (`hip_link_length:
0.055`, `upper_leg_link_length: 0.1075`, `lower_leg_link_length: 0.130`,
`body_width: 0.078`, `body_length: 0.186`) match Python's
`spot_micro_stick_figure.py` values exactly.

**One real, concrete difference:** C++'s `ikine()`
(`src/utils.cpp`) explicitly clamps its domain-sensitive values before
taking a square root — *"Poor man's inverse kinematics reachability
protection: Limit D to a maximum value of 1... otherwise the square root
functions below... will attempt a square root of a negative number"* —
clamping `D` to `[-1,1]` and the shared `q1`/`q2` `sqrt` argument
(`x4²+y4²-l1²`) to `≥0`. Python's `ikine()` has no equivalent protection
and raises a raw `ValueError` instead (Section 5). This is the concrete
basis for the "recommended follow-up work" note in Section 5 — not
acted on in this PR.

### URDF (`spot_micro_rviz/urdf/spot_micro.urdf` / `.urdf.xacro`)

Link lengths match: the `front_left_leg` joint's child-link origin is
`xyz="0 0.055 0"` (matches `hip_length`), `front_left_foot`'s is `xyz="0 0
-0.1075"` (matches `upper_leg_length`), and the fixed toe offset is
`-0.13` (matches `lower_leg_length`). Body half-dimensions also match:
`front_left_shoulder`'s origin is `xyz="0.093 0.039 0"`, and
`0.093 = body_length/2 = 0.186/2`, `0.039 = body_width/2 = 0.078/2`,
exactly.

**Not fully cross-checked:** the URDF's joint *axes* are expressed
per-joint in each joint's own local frame with an explicit `origin
rpy=...` (e.g. `front_left_shoulder axis="1 0 0"`, `front_left_leg axis="0
1 0"`), which is a fundamentally different modeling style from the D-H
chain's convention of every joint rotating about its own frame's z-axis.
Reconciling the two would require propagating each URDF joint's `origin
rpy` through the chain and checking the net rotation axis at each
step — a nontrivial exercise this review did not complete. This is left
as a genuine gap, not asserted as either matching or mismatched.

Also present in the URDF but explicitly **not evaluated** here (per
Section 9): joint `<limit>` values (e.g. `front_left_leg` lower=-2.666,
upper=1.548 radians) encode mechanical range, but confirming what
"lower"/"upper" mean physically, and where each joint's zero position
sits on the real hardware, needs hardware/human confirmation — reading
the numbers alone does not establish this.

### MJCF (`spot_micro_mujoco_sim/models/spot_micro_sim.xml`)

Joint names, axes (`axis="1 0 0"` for shoulder, `axis="0 1 0"` for
leg/foot), positions, and `range` values all match the URDF exactly,
consistent with this MJCF file having been generated from the URDF
(matching this repository's git history, which records it as a "SpotMicro
MuJoCo export"). It does not add information independent of the URDF
comparison above.

---

## 7. Numeric tolerance notes

- **Paper-comparison tests** (`TestEquation15AgainstGroundTruth`): paper
  reports angles to 4 decimal places in degrees; the ground-truth check
  here uses a much tighter `delta=1e-9` radians since it's comparing
  against a synthetic, exactly-known angle via pure floating-point
  forward/inverse kinematics, not against the paper's rounded table
  values.
- **FK→IK→FK round trips** (`TestFKIKRoundTrip`), **IK branch
  consistency** (`TestIKBranchSelection`), **symmetry**
  (`TestLeftRightFrontBackSymmetry`), **independent FK reference**
  (`TestIndependentForwardKinematicsReference`): `atol=1e-9`. These are
  pure floating-point round trips or cross-checks through mathematically
  equivalent formulas (not independent physical measurements), so error
  should be within a few orders of magnitude of machine epsilon
  (~2.2e-16); `1e-9` leaves generous headroom while still being a
  meaningful bound. The independent-reference test was verified during
  development to match production output to exactly `0.0` or `~1.1e-16`
  across six angle combinations before this tolerance was chosen.
- **Domain/singularity tests** (`TestWorkspaceBoundariesAndSingularities`):
  exact `ValueError` raising/non-raising behavior needs no floating
  tolerance for those assertions. The `D=+1`/`D=-1` boundary angle checks
  use `delta=1e-6` rad, looser than the round-trip tests, because `q3` at
  those exact boundaries was verified to land ~1e-9 to ~2e-8 rad off the
  analytic value (not exactly on it) due to float precision in
  `sqrt(1-D**2)` — expected precision loss at a singular point, not a
  bug.
- **Jacobian rank checks** (`TestJacobianSingularity`): evaluated directly
  in joint space at `θ3=0`/`θ3=±π` (not via `ikine()` on a constructed
  Cartesian target — see Section 5's three-way singularity distinction),
  so there is no IK-boundary rounding to contend with. Rank judged by the
  smallest/largest singular-value *ratio*, not an absolute cutoff:
  `< 1e-8` counts as singular, `> 1e-2` counts as regular. Both
  thresholds sit in the wide gap between what was actually measured
  during development — singular configurations' ratios were ~1e-13 to
  5e-11, the regular configuration's ratio was ~0.13 — with several
  orders of magnitude of margin on both sides of each threshold, not a
  borderline call. Finite-difference step `h=1e-6`.
- **`TestPaperTable3ExamplesUnresolved`**: exact equality on which legs
  are infeasible (`D > 1` or the hip-cylinder domain check failing), no
  floating tolerance involved — this test checks a boolean
  feasibility classification, not a continuous quantity.

---

## 8. Test coverage

- `TestEquation15AgainstGroundTruth` — Section 3's Eq.15 discrepancy
  claim (code correct, paper's printed formula wrong).
- `TestIndependentForwardKinematicsReference` — cross-checks
  `smk.t_0_to_4` against a from-scratch Rodrigues'-formula
  reimplementation that calls no production FK code, at both the paper's
  and SpotMicro's dimensions.
- `TestJacobianSingularity` — Section 5's rank-deficient-Jacobian claim,
  evaluated directly in joint space at `θ3=0` and `θ3=±π` (the exact
  mathematical singularities, independent of `ikine()`'s boundary
  rounding) via numerical SVD, against a regular-configuration control
  case. Kept deliberately separate from
  `TestWorkspaceBoundariesAndSingularities`'s IK-rounding tests below.
- `TestPaperTable3ExamplesUnresolved` — Section 5's open Table 3
  reproduction discrepancy across all three published examples, as an
  executable, reproducible record of what was tried (including both
  `T_M` orders for Example 3).
- `TestFKIKRoundTrip` — Section 3's "no discrepancy" claims for Eq.10-14
  (forward kinematics) taken together with Eq.15-17 (inverse kinematics),
  at SpotMicro's real hardware dimensions, across all four legs and
  nonzero roll/pitch/yaw.
- `TestLeftRightFrontBackSymmetry` — Section 2's left/right mirror-pair
  observation, reproduced directly from code's default stance.
- `TestIKBranchSelection` — Section 5's "multiple IK solutions" claim,
  across a representative spread of targets plus a near-boundary case.
- `TestWorkspaceBoundariesAndSingularities` — Section 5's singularity and
  unreachable-target claims, including both `D=+1` and `D=-1`, both sides
  of each boundary, and the hip-swing-cylinder boundary.

---

## 9. Requires human or hardware-owner confirmation — explicitly unresolved

None of the following can be determined from the paper, the figures, or
static analysis of this repository's code alone. They are listed here
rather than guessed at:

- Which end of the paper's Fig. 1 figure is physically front versus rear
  on the actual robot (Section 2).
- The definitive mapping of the paper's leg numbers 1-4 to SpotMicro's
  named corners (`rightback`/`rightfront`/`leftfront`/`leftback`)
  (Section 2) — including whether the README's "legs 1 and 3" (180°
  mounting) uses the same leg-numbering convention as the paper's Fig. 1
  "legs 1 and 3," or is SpotMicro's own independent numbering that
  happens to reuse the same digits.
- How the actual servos are mounted, including the claimed 180° rotations
  of legs 1 and 3 (README) — this document repeats the README's claim but
  has not independently verified it against hardware.
- Each physical joint's positive direction, mechanical zero, calibration
  offset, and safe range — the URDF's joint `<limit>` values (Section 6)
  are numbers in a file, not a substitute for this confirmation.
- Whether the Python visualization (`spot_micro_stick_figure.py`'s
  matplotlib plotting) matches real robot motion, or only its own
  internal coordinate conventions.
- Whether this document's transcription of the paper PDF matches the
  original's typography exactly, especially Eq.10, Eq.11, Eq.15, and
  Table 3 — Section 3's provenance note describes the re-verification
  process (direct visual inspection of rendered page images) used to
  raise confidence in this, but "matches the original typography" is
  ultimately a claim about the source document that only the publisher
  or authors can fully attest to.
- Whether the reported Table 3 values can be reproduced with the authors'
  original MATLAB program. That program is not available to this review;
  **direct reproduction is impossible without it**, and Section 5's
  "authors' MATLAB program may have differed from the typeset equations"
  explanation is explicitly labeled a hypothesis, not a demonstrated fact,
  for exactly this reason.
- Whether poses near the `D=±1` singularities (Section 5) are mechanically
  safe on the real SpotMicro hardware. **These poses were not, and must
  not be, tested on physical hardware as part of this documentation PR** —
  all singularity analysis here is purely computational (Jacobian rank
  via finite differences on the FK equations), with no hardware
  involvement of any kind.
