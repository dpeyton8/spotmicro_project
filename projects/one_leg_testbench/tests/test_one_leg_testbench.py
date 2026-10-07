import math
import re
import unittest
from pathlib import Path

import numpy as np

from one_leg_testbench import leg, plan
from one_leg_testbench.sim import run, summarize
from one_leg_testbench.trajectory import SWING_POINTS_MM, Gait, bezier

try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None


class LegKinematics(unittest.TestCase):
    def test_limits_are_firmware_and_simulator_intersection(self):
        expected = [[-31.4, 31.4], [-43.9, 88.7], [-149.0, -0.3]]
        np.testing.assert_allclose(np.degrees(leg.LIMITS_RAD), expected, atol=0.06)

    def test_calibration_center_is_firmware_neutral(self):
        self.assertEqual(leg.firmware_line(leg.CENTER_RAD), "-7.6,38.6,-82.8")

    def test_inverse_round_trips_forward_inside_limits(self):
        rng = np.random.default_rng(1)
        checked = 0
        for _ in range(300):
            q = rng.uniform(leg.LIMITS_RAD[:, 0], leg.LIMITS_RAD[:, 1])
            solved = leg.inverse(leg.forward(q))
            if solved is None or leg.violated_joints(solved):
                continue  # forward() can reach points whose negative-knee solution differs
            np.testing.assert_allclose(leg.forward(solved), leg.forward(q), atol=1e-9)
            checked += 1
        self.assertGreater(checked, 100)

    def test_unreachable_target_returns_none(self):
        self.assertIsNone(leg.inverse([0.5, -0.5, 0.5]))

    def test_ros_sign_mapping_is_an_involution(self):
        q = np.array([0.1, 0.7, -1.4])
        np.testing.assert_allclose(leg.from_ros(leg.to_ros(q)), q)

    @unittest.skipIf(mujoco is None, "mujoco not installed")
    def test_forward_matches_mujoco_toe(self):
        from one_leg_testbench.sim import MODEL

        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "toe_site")
        rng = np.random.default_rng(2)
        for _ in range(50):
            q = rng.uniform(leg.LIMITS_RAD[:, 0], leg.LIMITS_RAD[:, 1])
            data.qpos[:3] = leg.to_ros(q)
            mujoco.mj_forward(model, data)
            toe = data.site_xpos[site] - data.mocap_pos[0]
            np.testing.assert_allclose(toe, leg.mujoco_from_kinematic(leg.forward(q)), atol=1e-9)


class PaperTrajectory(unittest.TestCase):
    def test_bezier_hits_first_and_last_control_points(self):
        np.testing.assert_allclose(bezier(SWING_POINTS_MM, 0.0), SWING_POINTS_MM[0])
        np.testing.assert_allclose(bezier(SWING_POINTS_MM, 1.0), SWING_POINTS_MM[-1])

    def test_stance_and_swing_join_continuously(self):
        gait = Gait()
        end, _ = gait.foot_position(gait.cycle_seconds - 1e-9)
        start, _ = gait.foot_position(0.0)
        np.testing.assert_allclose(end, start, atol=1e-6)
        stance_end, _ = gait.foot_position(gait.stance_seconds - 1e-9)
        swing_start, _ = gait.foot_position(gait.stance_seconds)
        np.testing.assert_allclose(stance_end, swing_start, atol=1e-6)

    def test_stance_moves_foot_rearward(self):
        gait = Gait()
        self.assertGreater(gait.foot_position(0.0)[0][0], gait.foot_position(gait.stance_seconds * 0.99)[0][0])

    def test_default_gait_stays_inside_limits_with_margin(self):
        gait = Gait()
        worst = math.inf
        for t in np.arange(0.0, gait.cycle_seconds, 0.01):
            q, _, _ = plan.joint_targets(gait, float(t))
            worst = min(worst, float(np.min(np.minimum(q - leg.LIMITS_RAD[:, 0], leg.LIMITS_RAD[:, 1] - q))))
        self.assertGreater(math.degrees(worst), 10.0)

    def test_oversized_path_is_rejected_with_joint_name(self):
        with self.assertRaisesRegex(plan.PathError, "front_left_leg"):
            plan.check_path(Gait(hip_height=0.14, scale=0.3))

    def test_unreachable_height_is_rejected(self):
        with self.assertRaisesRegex(plan.PathError, "unreachable"):
            plan.check_path(Gait(hip_height=0.5))


class FirmwareParity(unittest.TestCase):
    INO = Path(__file__).resolve().parents[2] / "one_leg/firmware/one_left_leg_angle_control/one_left_leg_angle_control.ino"

    @classmethod
    def setUpClass(cls):
        cls.source = cls.INO.read_text()

    def _constant(self, name):
        return float(re.search(rf"constexpr \w+ {name} = ([\d.]+)", self.source).group(1))

    def _joint_rows(self):
        pattern = r'\{"[^"]+",\s*(\d+),\s*(\d+),\s*(\d+),\s*(-?\d+),\s*(-?[\d.]+)f\}'
        return [tuple(float(v) for v in row) for row in re.findall(pattern, self.source)]

    def test_firmware_constants_match_leg_module(self):
        self.assertEqual(self._constant("MAX_JOINT_ANGLE_DEG"), leg.SERVO_WINDOW_DEG)
        self.assertEqual((self._constant("PWM_MIN"), self._constant("PWM_MAX")), (leg.PWM_MIN, leg.PWM_MAX))

    def test_joint_table_matches_leg_module(self):
        rows = self._joint_rows()
        self.assertEqual(len(rows), 3)
        for i, (channel, center_pwm, range_pwm, direction, center_deg) in enumerate(rows):
            self.assertEqual((channel, center_pwm, range_pwm, direction), tuple(float(v) for v in leg.FIRMWARE_JOINTS[i]))
            self.assertAlmostEqual(center_deg, leg.CENTER_DEG[i])

    def test_pwm_is_center_at_calibration_center(self):
        np.testing.assert_array_equal(leg.firmware_pwm(leg.CENTER_RAD), [306, 306, 306])

    def test_serial_line_is_three_comma_separated_numbers(self):
        self.assertRegex(leg.firmware_line([0.1, 0.2, -0.3]), r"^-?\d+\.\d,-?\d+\.\d,-?\d+\.\d$")

    def test_gait_never_reaches_pwm_clamp(self):
        gait = Gait()
        for t in np.arange(0.0, gait.cycle_seconds, 0.01):
            q, _, _ = plan.joint_targets(gait, float(t))
            pwm = leg.firmware_pwm(q)
            self.assertTrue(np.all(pwm > leg.PWM_MIN + 20) and np.all(pwm < leg.PWM_MAX - 20), pwm)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class MujocoSimulation(unittest.TestCase):
    def test_leg_lands_and_alternates_stance_and_swing_contact(self):
        gait = Gait(stance_press=0.004)
        rows, _, settle = run(gait, drop=0.06, lower_seconds=2.0, hold_seconds=0.5, cycles=2.0, viewer=False)
        stats = summarize(rows, settle)
        self.assertTrue(all(np.isfinite(r["tracking_error_m"]) for r in rows))
        self.assertGreater(stats["stance_contact_fraction"], 0.8)
        self.assertLess(stats["swing_contact_fraction"], 0.2)
        self.assertGreater(stats["max_toe_clearance_swing_mm"], 15.0)
        self.assertGreater(stats["peak_normal_force_n"], 1.0)


if __name__ == "__main__":
    unittest.main()
