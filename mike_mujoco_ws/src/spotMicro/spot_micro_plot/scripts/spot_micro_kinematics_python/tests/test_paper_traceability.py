"""Tests that reproduce and cross-check numeric claims from the source paper:
Sen, Bakircioglu & Kalyoncu (2017), "Inverse Kinematic Analysis Of A
Quadruped Robot", IJSTR Vol 6 Issue 9, pp. 285-289.

See docs/IK_PAPER_TRACEABILITY.md for the full equation-by-equation
mapping these tests are evidence for.
"""
import unittest
from math import radians, degrees, atan2, sqrt
import numpy as np

from ..utilities import transformations as tf
from ..utilities import spot_micro_kinematics as smk

# Paper's Table 1 dimensions (NOT SpotMicro's real hardware dimensions --
# these are the paper's own example robot).
PAPER_L1, PAPER_L2, PAPER_L3 = 0.1, 0.4, 0.4
PAPER_L, PAPER_W = 1.0, 0.4


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


class TestPaperTable3Example1Unresolved(unittest.TestCase):
    """Attempts to reproduce Table 3, Example 1 in full (global foot target
    [x4,y4,z4]=[0,-0.65,0], body [xm,ym,zm]=[0,0,0], phi(yaw)=0,
    psi(pitch)=-15deg, omega(roll)=0, paper's own L1/L2/L3/L/W dimensions)
    by composing the paper's own Eq.5-9 (as verified correct against the
    paper and against code in docs/IK_PAPER_TRACEABILITY.md Sec.3) with the
    per-leg ikine() call.

    This does NOT reproduce the paper's published Table 3 angles, and the
    mismatch is not explained by the Eq.15 sign issue above (which is
    tested and resolved in isolation, in code's favor, by the test class
    above). Concretely, under this reconstruction:
      - 2 of the 4 legs (whichever pair sits at the "far" front/back offset
        for a given pitch sign) land outside ikine()'s valid domain
        entirely (D > 1, i.e. sqrt(1-D**2) of a negative number) -- this
        was checked under both signs of the -15 degree pitch value, and
        which two legs are infeasible simply swaps; it is not a sign
        artifact of the pitch convention.
      - For the 2 legs that ARE within domain, the resulting theta1 (~26.4
        degrees or ~-171.1 degrees depending on branch) does not match
        either of the paper's two distinct |theta1| values for this
        example (7.5883 or 11.5735 degrees), and the gap is far larger
        than the Eq.15 sign flip already accounted for above.

    Most likely explanation (see docs/IK_PAPER_TRACEABILITY.md Sec.5): the
    paper's Table 3 was generated by the authors' actual MATLAB program,
    which -- like Eq.15 -- may differ from the equations as typeset in the
    paper's text in ways not recoverable from the text alone. This is left
    as an explicitly open, unresolved discrepancy rather than forced to
    pass; see the referenced section for the full write-up and what was
    tried.
    """

    def test_two_of_four_legs_are_outside_ikine_domain_as_reconstructed(self):
        t_m = paper_t_m(0, 0, 0, yaw_deg=0, pitch_deg=-15, roll_deg=0)
        global_target = np.array([0, -0.65, 0, 1])
        leg_fns = {
            'rightback': smk.t_rightback,
            'rightfront': smk.t_rightfront,
            'leftfront': smk.t_leftfront,
            'leftback': smk.t_leftback,
        }
        infeasible = []
        for name, fn in leg_fns.items():
            t0 = fn(t_m, PAPER_L, PAPER_W)
            local = tf.ht_inverse(t0).dot(global_target)
            x4, y4, z4 = local[0], local[1], local[2]
            d = ((x4**2 + y4**2 + z4**2 - PAPER_L1**2 - PAPER_L2**2 - PAPER_L3**2)
                 / (2 * PAPER_L2 * PAPER_L3))
            if abs(d) > 1:
                infeasible.append(name)
        # Documents the actual, reproduced finding -- not a paper claim.
        self.assertEqual(sorted(infeasible), ['leftback', 'rightback'])


if __name__ == '__main__':
    unittest.main()
