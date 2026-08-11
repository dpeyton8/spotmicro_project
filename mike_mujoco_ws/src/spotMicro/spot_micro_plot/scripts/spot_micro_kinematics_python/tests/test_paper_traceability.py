"""Tests that reproduce and cross-check numeric claims from the source paper:
Sen, Bakircioglu & Kalyoncu (2017), "Inverse Kinematic Analysis Of A
Quadruped Robot", IJSTR Vol 6 Issue 9, pp. 285-289.

See docs/IK_PAPER_TRACEABILITY.md for the full equation-by-equation
mapping these tests are evidence for.
"""
import unittest
from math import radians, degrees, atan2, sqrt, pi
import numpy as np

from ..utilities import transformations as tf
from ..utilities import spot_micro_kinematics as smk
from ..spot_micro_stick_figure import SpotMicroStickFigure

# Paper's Table 1 dimensions (NOT SpotMicro's real hardware dimensions --
# these are the paper's own example robot).
PAPER_L1, PAPER_L2, PAPER_L3 = 0.1, 0.4, 0.4
PAPER_L, PAPER_W = 1.0, 0.4

d2r = pi / 180


def paper_t_m(xm, ym, zm, yaw_deg, pitch_deg, roll_deg):
    """Build T_M exactly as the paper's Eq.5 does: T_M = Rxyz * Trans(xm,ym,zm).
    Paper axis roles: yaw about y, pitch about z, roll about x (Fig. 1, Table 1).
    transformations.rotxyz(x_ang,y_ang,z_ang) = rotx(x)@roty(y)@rotz(z),
    so pass (roll, yaw, pitch) positionally to land roll->x, yaw->y, pitch->z.
    """
    r = tf.rotxyz(radians(roll_deg), radians(yaw_deg), radians(pitch_deg))
    r_h = np.block([[r, np.zeros((3, 1))], [np.array([0, 0, 0, 1])]])
    t_h = np.block([[np.eye(3), np.array([[xm], [ym], [zm]])],
                     [np.array([0, 0, 0, 1])]])
    return r_h @ t_h  # paper order: rotate, then translate-in-rotated-frame


class TestEquation15AgainstGroundTruth(unittest.TestCase):
    """Isolates the theta1 (Eq.15) question from all the extra unknowns in
    the full body-pose/leg-placement pipeline (Eq.5-9), by round-tripping a
    single leg through the paper's own (typo-corrected, per code comments
    and docs/IK_PAPER_TRACEABILITY.md Sec.3) forward kinematics (Eq.10-14,
    i.e. smk.t_0_to_4) and checking which of the two candidate theta1
    formulas recovers the known angle.

    This resolves the discrepancy noted in docs/IK_PAPER_TRACEABILITY.md
    Sec.3 (Eq.15): the paper's literally-printed formula
        theta1 = -atan2(-y4,x4) - atan2(sqrt(x4^2+y4^2-L1^2), -L1)
    does NOT recover the ground-truth angle used to generate (x4,y4,z4) via
    the paper's own forward kinematics -- it is off by roughly -206 degrees
    in this example, not a simple sign or periodicity difference. Code's
    formula (smk.ikine's q1) recovers it exactly (to float precision).
    """

    def test_code_q1_formula_is_correct_inverse_of_paper_fkine(self):
        l1, l2, l3 = PAPER_L1, PAPER_L2, PAPER_L3
        true_q1, true_q2, true_q3 = radians(20), radians(15), radians(-25)

        t = smk.t_0_to_4(true_q1, true_q2, true_q3, l1, l2, l3)
        x4, y4, z4 = t[0, 3], t[1, 3], t[2, 3]

        q1_code = atan2(y4, x4) + atan2(sqrt(x4**2 + y4**2 - l1**2), -l1)
        self.assertAlmostEqual(q1_code, true_q1, delta=1e-9)

    def test_paper_literal_eq15_does_not_recover_ground_truth(self):
        # Documents the discrepancy as a standing, executable fact rather
        # than a one-off finding buried in a commit message. If this ever
        # starts passing, the paper-vs-code discrepancy write-up in
        # docs/IK_PAPER_TRACEABILITY.md Sec.3 needs to be revisited.
        l1, l2, l3 = PAPER_L1, PAPER_L2, PAPER_L3
        true_q1, true_q2, true_q3 = radians(20), radians(15), radians(-25)

        t = smk.t_0_to_4(true_q1, true_q2, true_q3, l1, l2, l3)
        x4, y4, z4 = t[0, 3], t[1, 3], t[2, 3]

        q1_paper_literal = -atan2(-y4, x4) - atan2(
            sqrt(x4**2 + y4**2 - l1**2), -l1)
        self.assertGreater(abs(q1_paper_literal - true_q1), radians(30))


def rodrigues(axis, angle):
    """Rotation matrix from an axis-angle pair via Rodrigues' rotation
    formula (R = I + sin(t)K + (1-cos(t))K^2). Deliberately independent of
    transformations.py's rotx/roty/rotz (which hand-type cos/sin matrix
    entries directly): this computes the same mathematical object via the
    skew-symmetric cross-product-matrix exponential map instead, using
    only numpy -- no dependency on any code under test.
    """
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def independent_t_0_to_4(q1, q2, q3, l1, l2, l3):
    """Independent re-derivation of Eq.10-14's forward kinematics chain,
    computed via rodrigues() instead of smk.t_0_to_1/t_1_to_2/t_2_to_3/
    t_3_to_4/t_0_to_4. Does NOT call any function from
    utilities/transformations.py or utilities/spot_micro_kinematics.py.

    What IS independently computed here: the numeric composition of three
    parametrized rotations (q1, q2, q3, each about a local z-axis) with
    their associated link-length translations, via a different rotation
    formula/code path than the production matrices.

    What is NOT independently re-derived: which axis each joint rotates
    about, and the translation signs. Those come from
    docs/IK_PAPER_TRACEABILITY.md Sec.3's Eq.10-14 analysis (direct visual
    PDF verification plus the code's own typo-correction comments), not
    from this function. In particular, the fixed (non-actuated) T1->2
    reorientation is taken as the constant matrix already established
    correct there, since Rodrigues' formula adds no independence for a
    matrix with no joint-angle parameter to compute from first principles.
    """
    r1 = rodrigues([0, 0, 1], q1)
    t01 = np.eye(4)
    t01[0:3, 0:3] = r1
    t01[0:3, 3] = r1 @ np.array([-l1, 0, 0])

    t12 = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    r2 = rodrigues([0, 0, 1], q2)
    t23 = np.eye(4)
    t23[0:3, 0:3] = r2
    t23[0:3, 3] = r2 @ np.array([l2, 0, 0])

    r3 = rodrigues([0, 0, 1], q3)
    t34 = np.eye(4)
    t34[0:3, 0:3] = r3
    t34[0:3, 3] = r3 @ np.array([l3, 0, 0])

    return t01 @ t12 @ t23 @ t34


class TestIndependentForwardKinematicsReference(unittest.TestCase):
    """Cross-checks smk.t_0_to_4 against independent_t_0_to_4() above, which
    does not call any production FK routine, across several angle
    combinations including negative and large (>90deg) angles. Tolerance:
    atol=1e-9 -- verified numerically to match to machine epsilon (max
    diff 0.0 or ~1.1e-16 across the angles tried) when this was developed;
    1e-9 leaves headroom while remaining a meaningful bound.
    """
    TOL = 1e-9

    def test_independent_reference_matches_production_fkine(self):
        l1, l2, l3 = PAPER_L1, PAPER_L2, PAPER_L3
        angle_sets_deg = [
            (0, 0, 0),
            (20, 15, -25),
            (45, -30, 60),
            (-10, 5, -170),
            (170, -80, 10),
        ]
        for a1, a2, a3 in angle_sets_deg:
            q1, q2, q3 = radians(a1), radians(a2), radians(a3)
            t_indep = independent_t_0_to_4(q1, q2, q3, l1, l2, l3)
            t_prod = smk.t_0_to_4(q1, q2, q3, l1, l2, l3)
            np.testing.assert_allclose(t_indep, t_prod, atol=self.TOL)

    def test_independent_reference_matches_at_spotmicro_dimensions(self):
        l1, l2, l3 = 0.055, 0.1075, 0.130
        q1, q2, q3 = radians(12), radians(-33), radians(58)
        t_indep = independent_t_0_to_4(q1, q2, q3, l1, l2, l3)
        t_prod = smk.t_0_to_4(q1, q2, q3, l1, l2, l3)
        np.testing.assert_allclose(t_indep, t_prod, atol=self.TOL)


class TestJacobianSingularity(unittest.TestCase):
    """Supports the D=+-1 rank-deficient-Jacobian statement in
    docs/IK_PAPER_TRACEABILITY.md Sec.5 with an actual numerical check,
    rather than leaving it as an unverified "mathematically expected"
    claim. Computes a finite-difference Jacobian of the foot position
    w.r.t. (q1,q2,q3) via smk.t_0_to_4, and checks its rank via SVD at a
    regular configuration versus at D=+1 and D=-1.

    Tolerance: singular-value-based rank check with tol=1e-6 -- chosen
    because the smallest singular value at the two singularities came out
    ~1e-9 (D=1) and ~9e-7 (D=-1) in development, both several orders of
    magnitude below the other two singular values (~0.06-0.27) at the same
    configurations, and below the regular configuration's smallest singular
    value (~0.069). Finite-difference step h=1e-6 for the same reason
    used elsewhere in this file: far enough from machine epsilon to avoid
    catastrophic cancellation, far enough below the length scales involved
    (~0.05-0.13m) to approximate the true derivative well.
    """
    l1, l2, l3 = 0.055, 0.1075, 0.130

    def _foot_pos(self, q1, q2, q3):
        t = smk.t_0_to_4(q1, q2, q3, self.l1, self.l2, self.l3)
        return t[0:3, 3]

    def _numerical_jacobian(self, q1, q2, q3, h=1e-6):
        q0 = np.array([q1, q2, q3])
        j = np.zeros((3, 3))
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = h
            j[:, i] = (self._foot_pos(*(q0 + dq)) -
                       self._foot_pos(*(q0 - dq))) / (2 * h)
        return j

    def test_regular_configuration_is_full_rank(self):
        q = smk.ikine(0.05, -0.15, 0.05, self.l1, self.l2, self.l3)
        j = self._numerical_jacobian(*q)
        self.assertEqual(np.linalg.matrix_rank(j, tol=1e-6), 3)

    def test_d_equals_plus_one_is_rank_deficient(self):
        # Fully extended: r^2+z4^2 = (l2+l3)^2 with y4=z4=0
        x4 = sqrt(self.l1**2 + (self.l2 + self.l3)**2)
        q = smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)
        j = self._numerical_jacobian(*q)
        self.assertEqual(np.linalg.matrix_rank(j, tol=1e-6), 2)

    def test_d_equals_minus_one_is_rank_deficient(self):
        # Fully folded: r^2+z4^2 = (l2-l3)^2 with y4=z4=0. Nudged a hair
        # inside the domain (see TestWorkspaceBoundariesAndSingularities'
        # d_equals_minus_one test for the exact-boundary ValueError this
        # sidesteps) so ikine() doesn't raise while computing the target.
        r_folded = abs(self.l2 - self.l3)
        x4 = sqrt(self.l1**2 + r_folded**2) * (1 + 1e-9)
        q = smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)
        j = self._numerical_jacobian(*q)
        self.assertEqual(np.linalg.matrix_rank(j, tol=1e-6), 2)


class TestPaperTable3ExamplesUnresolved(unittest.TestCase):
    """Attempts to reproduce all three of Table 3's worked examples in full
    (global foot target, body pose, paper's own L1/L2/L3/L/W dimensions) by
    composing the paper's own Eq.5-9 (individually verified correct against
    the paper and against code in docs/IK_PAPER_TRACEABILITY.md Sec.3) with
    the per-leg ikine() call.

    None of the three reproduces the paper's published angles, and the
    mismatch is not explained by the Eq.15 sign issue (tested and resolved
    in isolation, in code's favor, by TestEquation15AgainstGroundTruth).
    Assumptions used throughout, stated explicitly per the review request
    that asked this not be forced to agree:
      - Eq.5's literal rotate-then-translate T_M order (paper_t_m() above).
        For Examples 1 and 2, body translation is zero, so this order
        choice cannot be the cause of infeasibility there (translating by
        zero commutes with anything). For Example 3 specifically, where
        body translation IS nonzero, both T_M orders were tried (this
        literal one and code's translate-then-rotate convention); neither
        makes Example 3 fully reachable -- see
        test_example_3_all_four_legs_infeasible_under_both_t_m_orders.
      - Paper axis roles for yaw/pitch/roll as established in Sec.1 of the
        doc (yaw about y, pitch about z, roll about x).
      - No alternate leg-name<->paper-leg-number mapping was searched for
        beyond what Section 2 of the doc already establishes (the {1,4}/
        {2,3} figure-confirmed grouping) -- a mapping search was not
        pursued further because Examples 1/2's infeasibility affects
        specific legs regardless of what they are named, and Example 3
        fails for all four legs regardless of naming.

    Findings, not forced to pass as agreement with the paper:
      - Example 1: 2 of 4 legs (rightback, leftback, under the paper's
        literal -15deg pitch sign; the other two legs under the opposite
        sign -- not a sign-convention artifact, see docstring history)
        land outside ikine()'s valid domain (D > 1). The 2 in-domain legs'
        theta1 (~26.4deg or ~-171.1deg depending on branch) matches
        neither of the paper's two |theta1| values (7.5883/11.5735deg).
      - Example 2: also 2 of 4 legs infeasible (rightfront, leftfront),
        i.e. the same kind of partial reachability failure as Example 1.
      - Example 3 (the only example with nonzero body translation AND all
        three rotation angles nonzero): ALL FOUR legs land outside the
        valid domain (D ranging ~2.2 to ~4.2), under both T_M orders. This
        is a materially worse failure than Examples 1/2, not just "more of
        the same."

    Hypothesis (not confirmed): the paper's Table 3 was generated by the
    authors' own MATLAB program, which -- like the printed Eq.15 -- may
    differ from the equations exactly as typeset. This is explicitly a
    hypothesis, not a demonstrated fact; see docs/IK_PAPER_TRACEABILITY.md
    Sec.5 and Sec.8 for the provenance labeling of this claim, and
    "Whether the reported Table 3 values can be reproduced with the
    authors' original MATLAB program" in that document's unresolved-items
    list -- direct reproduction is impossible without that program.
    """

    LEG_FNS = {
        'rightback': smk.t_rightback,
        'rightfront': smk.t_rightfront,
        'leftfront': smk.t_leftfront,
        'leftback': smk.t_leftback,
    }

    @staticmethod
    def _infeasible_legs(t_m, global_target):
        infeasible = []
        for name, fn in TestPaperTable3ExamplesUnresolved.LEG_FNS.items():
            t0 = fn(t_m, PAPER_L, PAPER_W)
            local = tf.ht_inverse(t0).dot(np.array([*global_target, 1]))
            x4, y4, z4 = local[0], local[1], local[2]
            d = ((x4**2 + y4**2 + z4**2 - PAPER_L1**2 - PAPER_L2**2 - PAPER_L3**2)
                 / (2 * PAPER_L2 * PAPER_L3))
            in_domain = x4**2 + y4**2 - PAPER_L1**2
            if abs(d) > 1 or in_domain < 0:
                infeasible.append(name)
        return sorted(infeasible)

    def test_example_1_two_of_four_legs_infeasible(self):
        t_m = paper_t_m(0, 0, 0, yaw_deg=0, pitch_deg=-15, roll_deg=0)
        infeasible = self._infeasible_legs(t_m, (0, -0.65, 0))
        self.assertEqual(infeasible, ['leftback', 'rightback'])

    def test_example_2_two_of_four_legs_infeasible(self):
        t_m = paper_t_m(0, 0, 0, yaw_deg=-45, pitch_deg=0, roll_deg=-10)
        infeasible = self._infeasible_legs(t_m, (-0.05, -0.55, 0))
        self.assertEqual(infeasible, ['leftfront', 'rightfront'])

    def test_example_3_all_four_legs_infeasible_under_both_t_m_orders(self):
        target = (-0.15, -0.7, 0.05)
        xm, ym, zm = 0.1, 0.2, -0.3
        yaw, pitch, roll = -10, -10, 15

        # Paper's literal Eq.5 order (rotate, then translate-in-rotated-frame)
        t_m_paper_order = paper_t_m(xm, ym, zm, yaw_deg=yaw, pitch_deg=pitch,
                                     roll_deg=roll)
        self.assertEqual(
            self._infeasible_legs(t_m_paper_order, target),
            ['leftback', 'leftfront', 'rightback', 'rightfront'])

        # Code's convention (translate-in-global-frame, then rotate) --
        # this is the only Table 3 example where the two orders can differ
        # at all, since it's the only one with nonzero body translation.
        r = tf.rotxyz(radians(roll), radians(yaw), radians(pitch))
        r_h = np.block([[r, np.zeros((3, 1))], [np.array([0, 0, 0, 1])]])
        t_h = np.block([[np.eye(3), np.array([[xm], [ym], [zm]])],
                         [np.array([0, 0, 0, 1])]])
        t_m_code_order = t_h @ r_h
        self.assertEqual(
            self._infeasible_legs(t_m_code_order, target),
            ['leftback', 'leftfront', 'rightback', 'rightfront'])


class TestFKIKRoundTrip(unittest.TestCase):
    """FK -> IK -> FK round trips at SpotMicro's real hardware dimensions.
    Tolerance: 1e-9 radians -- this is a pure floating-point round trip
    through the same equations, not an independent solve, so error should
    be within a couple orders of magnitude of machine epsilon (~2.2e-16).
    """
    TOL = 1e-9

    def _round_trip(self, x=0, y=0.18, z=0, phi=0, theta=0, psi=0):
        sm = SpotMicroStickFigure(x=x, y=y, z=z, phi=phi, theta=theta, psi=psi)
        orig_angles = np.array(sm.get_leg_angles())
        coords = sm.get_leg_coordinates()
        foot_coords = np.array([leg[3] for leg in coords])
        sm2 = SpotMicroStickFigure(x=x, y=y, z=z, phi=phi, theta=theta, psi=psi)
        sm2.set_absolute_foot_coordinates(foot_coords)
        new_angles = np.array(sm2.get_leg_angles())
        np.testing.assert_allclose(orig_angles, new_angles, atol=self.TOL)
        return orig_angles, new_angles

    def test_default_pose_all_four_legs(self):
        self._round_trip()

    def test_nonzero_roll(self):
        self._round_trip(phi=15 * d2r)

    def test_nonzero_pitch(self):
        self._round_trip(theta=10 * d2r)

    def test_nonzero_yaw(self):
        self._round_trip(psi=20 * d2r)

    def test_combined_roll_pitch_yaw(self):
        self._round_trip(phi=12 * d2r, theta=-8 * d2r, psi=25 * d2r)


class TestLeftRightFrontBackSymmetry(unittest.TestCase):
    """SpotMicro's default stance is bilaterally symmetric; verify the
    joint-angle magnitudes reflect that symmetry through the IK/FK code
    at zero body orientation. Tolerance: 1e-9 radians -- comparing two
    branches of the same floating-point computation, not independently
    derived values.
    """
    TOL = 1e-9

    def test_left_right_hip_angles_mirror_at_default_pose(self):
        sm = SpotMicroStickFigure()
        rb, rf, lf, lb = sm.get_leg_angles()
        # rightback/leftback and rightfront/leftfront are mirror pairs;
        # q1 (side-swing) matches, q2/q3 flip sign, per the default angle
        # assignment in the constructor (rb=[0,-30d2r,60d2r],
        # lb=[0,30d2r,-60d2r], etc -- spot_micro_stick_figure.py:209-212).
        # Confirmed against actual get_leg_angles() output before writing
        # this assertion, not assumed.
        np.testing.assert_allclose([rb[0], -rb[1], -rb[2]], lb, atol=self.TOL)
        np.testing.assert_allclose([rf[0], -rf[1], -rf[2]], lf, atol=self.TOL)

    def test_front_back_same_side_share_hip_swing_zero(self):
        sm = SpotMicroStickFigure()
        rb, rf, lf, lb = sm.get_leg_angles()
        self.assertAlmostEqual(rb[0], rf[0], delta=self.TOL)
        self.assertAlmostEqual(lf[0], lb[0], delta=self.TOL)


def angles_close_mod_2pi(a, b, tol=1e-9):
    """True if angle a and angle b represent the same rotation modulo 2*pi
    (i.e. wrapped-equivalent angles aren't reported as different). Used
    where a branch or boundary computation could legitimately land on a
    coterminal angle rather than the exact same real-number value.
    """
    diff = (a - b + pi) % (2 * pi) - pi
    return abs(diff) < tol


class TestIKBranchSelection(unittest.TestCase):
    """Both legs12=True and legs12=False are two distinct, FK-consistent
    solutions for a reachable target; ikine() does not auto-select, the
    caller must choose.

    What legs12 controls, precisely, and no more than this: it is the sign
    passed to atan2() when computing q3 from D (Eq.17) -- literally
    `atan2(sqrt(1-D**2), D)` vs `atan2(-sqrt(1-D**2), D)`,
    i.e. a positive-q3-branch vs negative-q3-branch choice (see
    utilities/spot_micro_kinematics.py:219-222). This document and these
    tests deliberately do NOT call the two branches "elbow-up"/"elbow-down"
    or assign any other physical label (knee-forward/backward,
    right/left) -- doing so requires knowing which way the physical knee
    joint bends on the real chassis, which is outside what static analysis
    of this code can determine. See docs/IK_PAPER_TRACEABILITY.md's
    unresolved-items list ("Each physical joint's positive direction,
    mechanical zero, calibration offset, and safe range").

    Tolerance: atol=1e-9 rad for the FK-consistency check (same pure
    floating-point round-trip reasoning as TestFKIKRoundTrip). Angle
    comparisons use angles_close_mod_2pi() where a branch could plausibly
    land on a coterminal angle rather than the exact same value.
    """
    TOL = 1e-9
    L1, L2, L3 = 0.055, 0.1075, 0.130

    def _check_both_branches(self, x4, y4, z4):
        q_true = smk.ikine(x4, y4, z4, self.L1, self.L2, self.L3, legs12=True)
        q_false = smk.ikine(x4, y4, z4, self.L1, self.L2, self.L3, legs12=False)
        # Different solutions (branch actually changes the answer), unless
        # exactly at a D=+-1 singularity where the branches coincide.
        differs = not angles_close_mod_2pi(q_true[2], q_false[2], tol=1e-6)
        # Both reproduce the same foot position via forward kinematics
        for q in (q_true, q_false):
            t = smk.t_0_to_4(q[0], q[1], q[2], self.L1, self.L2, self.L3)
            np.testing.assert_allclose(t[0:3, 3], [x4, y4, z4], atol=self.TOL)
        return differs

    def test_both_branches_are_fk_consistent_but_differ(self):
        self.assertTrue(self._check_both_branches(0.05, -0.15, 0.05))

    def test_both_branches_across_a_representative_set_of_targets(self):
        # A spread of reachable targets across the workspace, not just one.
        targets = [
            (0.06, -0.10, 0.02),
            (0.05, -0.20, -0.05),
            (0.08, -0.05, 0.10),
            (0.055, -0.15, 0.0),
            (0.10, -0.12, 0.03),
        ]
        any_differed = False
        for x4, y4, z4 in targets:
            if self._check_both_branches(x4, y4, z4):
                any_differed = True
        self.assertTrue(any_differed)

    def test_both_branches_near_but_not_at_extension_boundary(self):
        # Close to the D=1 boundary (see TestJacobianSingularity) but
        # comfortably inside it, where the two branches should still be
        # numerically distinguishable rather than having converged.
        x4 = sqrt(self.L1**2 + (self.L2 + self.L3 - 0.01)**2)
        self.assertTrue(self._check_both_branches(x4, 0, 0))


class TestWorkspaceBoundariesAndSingularities(unittest.TestCase):
    """Tolerance: exact ValueError checks need no floating tolerance. The
    D=+-1 singularity checks use 1e-6 rad, looser than the round-trip
    tests' 1e-9 -- verified numerically that q3 at these exact boundaries
    comes out ~1e-9 to ~2e-8 rad off the analytic value (0 or pi), not
    exactly on it, due to float precision in sqrt(1-D**2) when D is
    extremely close to +-1; this is expected precision loss right at a
    singular point, not a bug.

    NOTE on production code: ikine() (utilities/spot_micro_kinematics.py)
    performs no clamping or protection against float-rounding-induced
    domain errors -- it calls math.sqrt directly and lets ValueError
    propagate raw. The C++ port (spot_micro_motion_cmd/libs/
    spot_micro_kinematics_cpp/src/utils.cpp, ikine()) DOES clamp D to
    [-1,1] and the q1/q2 shared sqrt argument to >=0 before taking the
    square root (see docs/IK_PAPER_TRACEABILITY.md Sec.6 "Cross-check
    against other implementations"). This test file does not change
    Python's ikine() to match -- per the review scope, any such change is
    documented in the doc as recommended follow-up work, not made here.
    """
    l1, l2, l3 = 0.055, 0.1075, 0.130

    def test_target_inside_hip_swing_cylinder_raises(self):
        # x4^2 + y4^2 < l1^2 makes the q1 sqrt term's argument negative
        with self.assertRaises(ValueError):
            smk.ikine(0.01, 0.01, 0.05, self.l1, self.l2, self.l3)

    def test_target_exactly_on_hip_swing_cylinder_boundary_does_not_raise(self):
        # x4^2 + y4^2 == l1^2 exactly: the q1 sqrt argument is exactly 0,
        # which is a valid (if degenerate) input to math.sqrt -- the
        # boundary itself is reachable, only strictly inside it is not.
        # z4=0.1 is chosen (not arbitrary) to also keep D in [-1,1]: with
        # x4^2+y4^2=l1^2, D reduces to (z4^2-l2^2-l3^2)/(2*l2*l3), which
        # needs z4 in [|l2-l3|, l2+l3] = [0.0225, 0.2375] here -- verified
        # numerically before fixing this value, not assumed.
        x4, y4 = self.l1, 0.0
        q = smk.ikine(x4, y4, 0.1, self.l1, self.l2, self.l3)
        self.assertTrue(all(np.isfinite(q)))

    def test_target_beyond_max_reach_raises(self):
        # D > 1: target farther than l2+l3 (in the reduced radial/z plane)
        # from the hip-swing joint
        far = self.l1 + self.l2 + self.l3 + 0.5
        with self.assertRaises(ValueError):
            smk.ikine(far, 0, 0, self.l1, self.l2, self.l3)

    def test_target_just_beyond_max_reach_raises(self):
        # D = 1 + epsilon: a target only slightly beyond max reach still
        # raises -- there is no tolerance/clamping margin in production
        # Python code (contrast the C++ port, see class docstring).
        x4 = sqrt(self.l1**2 + (self.l2 + self.l3 + 1e-6)**2)
        with self.assertRaises(ValueError):
            smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)

    def test_target_just_inside_max_reach_does_not_raise(self):
        x4 = sqrt(self.l1**2 + (self.l2 + self.l3 - 1e-6)**2)
        q = smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)
        self.assertTrue(all(np.isfinite(q)))

    def test_fully_extended_leg_singularity_d_equals_one(self):
        # D == 1 (theta3 == 0, straight leg) is the boundary between
        # reachable and unreachable -- must not raise, and FK must
        # reproduce the target. With y4=z4=0, D=1 requires
        # x4 = sqrt(l1^2 + (l2+l3)^2), NOT x4=l1+l2+l3 (that formula is
        # only valid for a planar 2-link arm without the l1 offset).
        x4 = sqrt(self.l1**2 + (self.l2 + self.l3)**2)
        q = smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)
        self.assertAlmostEqual(q[2], 0.0, delta=1e-6)
        t = smk.t_0_to_4(q[0], q[1], q[2], self.l1, self.l2, self.l3)
        np.testing.assert_allclose(t[0:3, 3], [x4, 0, 0], atol=1e-6)

    def test_minimum_planar_reach_singularity_d_equals_minus_one_raises_at_exact_boundary(self):
        # D == -1 (theta3 == pi, fully folded knee against the upper leg)
        # is the OTHER boundary -- the minimum planar reach, governed by
        # |L2-L3| rather than L2+L3. Computed exactly with y4=z4=0:
        # r = |l2-l3|, x4 = sqrt(l1^2 + r^2). Unlike the D=+1 boundary
        # above, this exact analytic point is NOT reachable in practice:
        # float rounding computes D as very slightly LESS than -1 (about
        # -1 - 4.4e-16 in this configuration), so ikine() raises ValueError
        # here even though the true mathematical D is exactly -1. This is
        # itself a concrete instance of "D slightly outside [-1,1] purely
        # from numerical rounding" -- documented, not treated as a bug in
        # this test.
        r_folded = abs(self.l2 - self.l3)
        x4 = sqrt(self.l1**2 + r_folded**2)
        with self.assertRaises(ValueError):
            smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)

    def test_minimum_planar_reach_just_inside_boundary_does_not_raise(self):
        # Nudged fractionally inside the true boundary to sidestep the
        # float-rounding cliff documented above, confirming the boundary
        # is reachable in principle (D=-1 itself, per the Jacobian test in
        # TestJacobianSingularity, is a valid rank-deficient configuration,
        # not an inherently invalid one -- only its exact float
        # representation here happens to round the wrong way).
        r_folded = abs(self.l2 - self.l3)
        x4 = sqrt(self.l1**2 + r_folded**2) * (1 + 1e-9)
        q = smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)
        self.assertTrue(angles_close_mod_2pi(q[2], pi, tol=1e-3))

    def test_minimum_planar_reach_just_outside_boundary_raises(self):
        r_folded = abs(self.l2 - self.l3) - 1e-6
        x4 = sqrt(self.l1**2 + r_folded**2)
        with self.assertRaises(ValueError):
            smk.ikine(x4, 0, 0, self.l1, self.l2, self.l3)


if __name__ == '__main__':
    unittest.main()
