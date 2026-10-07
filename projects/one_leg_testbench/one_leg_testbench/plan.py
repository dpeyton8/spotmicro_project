"""Turns the paper's foot path into checked joint targets; run as a module to print ESP32 serial lines."""

from __future__ import annotations

import argparse

import numpy as np

from . import leg
from .trajectory import Gait


class PathError(ValueError):
    pass


def joint_targets(gait: Gait, t: float) -> tuple[np.ndarray, np.ndarray, str]:
    """Kinematic joint angles (rad), hip-relative toe target, and phase; raises PathError if infeasible."""
    point, phase = gait.foot_position(t)
    q = leg.inverse(leg.kinematic_from_mujoco(point))
    if q is None:
        raise PathError(f"t={t:.2f}s ({phase}): toe target {np.round(point, 3)} m is unreachable")
    bad = leg.violated_joints(q)
    if bad:
        names = ", ".join(f"{leg.JOINT_NAMES[i]}={np.degrees(q[i]):.1f} deg" for i in bad)
        raise PathError(f"t={t:.2f}s ({phase}): outside joint limits: {names}")
    return q, point, phase


def check_path(gait: Gait, step: float = 0.01) -> None:
    for t in np.arange(0.0, gait.cycle_seconds, step):
        joint_targets(gait, float(t))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hip-height", type=float, default=Gait.hip_height)
    parser.add_argument("--scale", type=float, default=Gait.scale)
    parser.add_argument("--points", type=int, default=25)
    args = parser.parse_args()
    gait = Gait(hip_height=args.hip_height, scale=args.scale)
    check_path(gait)
    print("t_s, phase, shoulder,hip,knee (paste into one_left_leg_angle_control.ino), predicted PWM")
    for t in np.linspace(0.0, gait.cycle_seconds, args.points, endpoint=False):
        q, _, phase = joint_targets(gait, float(t))
        print(f"{t:5.2f}, {phase:6s}, {leg.firmware_line(q)}, {','.join(map(str, leg.firmware_pwm(q)))}")


if __name__ == "__main__":
    main()
