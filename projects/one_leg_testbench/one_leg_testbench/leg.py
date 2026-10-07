"""Front-left SpotMicro leg: geometry, calibration, forward kinematics and the repo's corrected IK."""

from __future__ import annotations

import math

import numpy as np

# Link lengths (m) from spot_micro_motion_cmd.yaml.
L1, L2, L3 = 0.055, 0.1075, 0.130
TOE_RADIUS = 0.018

# LF servo center angles (deg, kinematic) from spot_micro_motion_cmd.yaml; firmware allows +/-82.5 deg about them.
CENTER_DEG = np.array([-7.6, 38.6, -82.8])
SERVO_WINDOW_DEG = 82.5
CENTER_RAD = np.radians(CENTER_DEG)

# front_left_{shoulder,leg,foot} joint ranges (rad) in spot_micro_sim.xml; ROS angle = ROS_SIGN * kinematic angle.
MODEL_RANGE_RAD = np.array([[-0.548, 0.548], [-2.666, 1.548], [-2.6, 0.1]])
ROS_SIGN = np.array([-1.0, 1.0, 1.0])
JOINT_NAMES = ("front_left_shoulder", "front_left_leg", "front_left_foot")


def _limits_rad() -> np.ndarray:
    model = np.sort(MODEL_RANGE_RAD * ROS_SIGN[:, None], axis=1)
    window = np.radians(CENTER_DEG[:, None] + SERVO_WINDOW_DEG * np.array([-1.0, 1.0]))
    return np.stack([np.maximum(model[:, 0], window[:, 0]), np.minimum(model[:, 1], window[:, 1])], axis=1)


# Intersection of the firmware window and the simulator ranges, in kinematic radians.
LIMITS_RAD = _limits_rad()

_T12 = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)


def _rot_z(angle: float, length: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0, length * c], [s, c, 0, length * s], [0, 0, 1, 0], [0, 0, 0, 1]])


def forward(q) -> np.ndarray:
    """Toe position (m) in the leg frame for kinematic angles q1, q2, q3 (rad)."""
    q1, q2, q3 = q
    c, s = math.cos(q1), math.sin(q1)
    t01 = np.array([[c, -s, 0, -L1 * c], [s, c, 0, -L1 * s], [0, 0, 1, 0], [0, 0, 0, 1]])
    return (t01 @ _T12 @ _rot_z(q2, L2) @ _rot_z(q3, L3))[:3, 3]


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def inverse(point) -> np.ndarray | None:
    """Kinematic angles reaching `point`, or None if unreachable; limits are checked separately."""
    x, y, z = point
    radial_sq = x * x + y * y - L1 * L1
    d = (x * x + y * y + z * z - L1 * L1 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if radial_sq < 0 or abs(d) > 1:
        return None
    root = math.sqrt(radial_sq)
    q3 = math.atan2(-math.sqrt(1 - d * d), d)  # left legs use the negative knee branch
    q2 = math.atan2(z, root) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    q1 = math.atan2(y, x) + math.atan2(root, -L1)
    return np.array([_wrap(q1), _wrap(q2), _wrap(q3)])


def violated_joints(q, tolerance: float = 1e-9) -> list[int]:
    q = np.asarray(q)
    return [i for i in range(3) if not LIMITS_RAD[i, 0] - tolerance <= q[i] <= LIMITS_RAD[i, 1] + tolerance]


def to_ros(q) -> np.ndarray:
    return ROS_SIGN * np.asarray(q)


def from_ros(q) -> np.ndarray:
    return ROS_SIGN * np.asarray(q)


def firmware_line(q) -> str:
    """Serial Monitor input for one_left_leg_angle_control.ino (shoulder,hip,knee in degrees)."""
    return ",".join(f"{v:.1f}" for v in np.degrees(q))


# Per-channel values from the Joint table in one_left_leg_angle_control.ino: (channel, centerPwm, rangePwm, direction).
FIRMWARE_JOINTS = ((0, 306, 389, 1), (1, 306, 397, 1), (2, 306, 387, 1))
PWM_MIN, PWM_MAX = 80, 520


def firmware_pwm(q) -> np.ndarray:
    """PWM counts the sketch sends for kinematic angles q; mirrors angleToPwm() including its clamps."""
    out = np.empty(3, dtype=int)
    for i, (_, center_pwm, range_pwm, direction) in enumerate(FIRMWARE_JOINTS):
        angle = float(np.clip(np.degrees(q[i]), CENTER_DEG[i] - SERVO_WINDOW_DEG, CENTER_DEG[i] + SERVO_WINDOW_DEG))
        pwm = center_pwm + direction * (angle - CENTER_DEG[i]) * range_pwm / (2.0 * SERVO_WINDOW_DEG)
        out[i] = int(np.clip(round(pwm), PWM_MIN, PWM_MAX))
    return out


def kinematic_from_mujoco(p) -> np.ndarray:
    """Hip-relative MuJoCo point (x fore, y left, z up) to the leg frame used by forward()."""
    return np.array([-p[1], p[2], -p[0]])


def mujoco_from_kinematic(p) -> np.ndarray:
    return np.array([-p[2], -p[0], p[1]])
