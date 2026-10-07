"""Runs the paper's foot path on the left-leg MuJoCo rig: hold above ground, lower to the floor, then walk in place."""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from . import leg
from .plan import PathError, joint_targets
from .trajectory import Gait

MODEL = Path(__file__).resolve().parent / "model" / "left_leg_rig.xml"


def contact_force(model, data, toe_id: int, floor_id: int) -> tuple[bool, float]:
    wrench = np.zeros(6)
    total, touching = 0.0, False
    for i in range(data.ncon):
        pair = {int(data.contact[i].geom1), int(data.contact[i].geom2)}
        if pair == {toe_id, floor_id}:
            touching = True
            mujoco.mj_contactForce(model, data, i, wrench)
            total += max(0.0, float(wrench[0]))
    return touching, total


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def run(gait: Gait, drop: float, lower_seconds: float, hold_seconds: float, cycles: float, viewer: bool):
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    toe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_toe")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "toe_site")
    mount_touch = gait.hip_height + leg.TOE_RADIUS
    mount_start = mount_touch + drop
    total = hold_seconds + lower_seconds + cycles * gait.cycle_seconds

    q0, _, _ = joint_targets(gait, 0.0)
    data.qpos[:3] = leg.to_ros(q0)
    data.ctrl[:3] = leg.to_ros(q0)
    data.mocap_pos[0] = [0, 0, mount_start]
    mujoco.mj_forward(model, data)

    handle = None
    if viewer:
        handle = mujoco.viewer.launch_passive(model, data)
    rows = []
    wall0 = time.perf_counter()
    try:
        while data.time < total:
            if handle is not None and not handle.is_running():
                break
            gait_t = max(0.0, data.time - hold_seconds - lower_seconds)
            mount_z = mount_start - drop * smoothstep((data.time - hold_seconds) / lower_seconds)
            data.mocap_pos[0] = [0, 0, mount_z]
            q, point, phase = joint_targets(gait, gait_t)
            data.ctrl[:3] = leg.to_ros(q)
            mujoco.mj_step(model, data)

            toe_rel = data.site_xpos[site] - data.mocap_pos[0]
            touching, force = contact_force(model, data, toe, floor)
            rows.append(
                dict(
                    time_s=data.time, phase=phase if data.time > hold_seconds + lower_seconds else "lowering",
                    mount_z_m=mount_z, target_x_m=point[0], target_y_m=point[1], target_z_m=point[2],
                    toe_x_m=toe_rel[0], toe_y_m=toe_rel[1], toe_z_m=toe_rel[2],
                    tracking_error_m=float(np.linalg.norm(toe_rel - point)),
                    toe_height_m=float(data.site_xpos[site][2]) - leg.TOE_RADIUS,
                    in_contact=int(touching), normal_force_n=force,
                    **{f"q{i + 1}_cmd_rad": float(q[i]) for i in range(3)},
                    **{f"q{i + 1}_rad": float(leg.from_ros(data.qpos[:3])[i]) for i in range(3)},
                )
            )
            if handle is not None:
                handle.sync()
                time.sleep(max(0.0, wall0 + data.time - time.perf_counter()))
        # Closing programmatically races the render thread (GLXBadDrawable), so wait for the user to close the window.
        while handle is not None and handle.is_running():
            time.sleep(0.05)
    finally:
        if handle is not None and handle.is_running():
            handle.close()
    return rows, gait.cycle_seconds, hold_seconds + lower_seconds


def summarize(rows, settle_seconds: float) -> dict:
    gait_rows = [r for r in rows if r["time_s"] >= settle_seconds]
    err = np.array([r["tracking_error_m"] for r in gait_rows])
    stance = [r for r in gait_rows if r["phase"] == "stance"]
    swing = [r for r in gait_rows if r["phase"] == "swing"]
    return dict(
        rms_tracking_error_mm=float(np.sqrt(np.mean(err**2)) * 1000),
        max_tracking_error_mm=float(err.max() * 1000),
        stance_contact_fraction=float(np.mean([r["in_contact"] for r in stance])) if stance else float("nan"),
        swing_contact_fraction=float(np.mean([r["in_contact"] for r in swing])) if swing else float("nan"),
        peak_normal_force_n=float(max(r["normal_force_n"] for r in gait_rows)),
        min_toe_clearance_swing_mm=float(min(r["toe_height_m"] for r in swing) * 1000) if swing else float("nan"),
        max_toe_clearance_swing_mm=float(max(r["toe_height_m"] for r in swing) * 1000) if swing else float("nan"),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hip-height", type=float, default=Gait.hip_height, help="hip to toe center at stance, m")
    parser.add_argument("--scale", type=float, default=Gait.scale, help="fraction of the paper's leg size")
    parser.add_argument("--press", type=float, default=0.004, help="extra downward toe target at mid-stance, m")
    parser.add_argument("--drop", type=float, default=0.06, help="start height of the toe above the floor, m")
    parser.add_argument("--hold", type=float, default=0.5, help="seconds held at the start height")
    parser.add_argument("--lower", type=float, default=2.0, help="seconds to lower to the floor")
    parser.add_argument("--cycles", type=float, default=3.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--output", type=Path, help="write the per-step log as CSV")
    args = parser.parse_args()

    gait = Gait(hip_height=args.hip_height, scale=args.scale, stance_press=args.press)
    try:
        rows, _, settle = run(gait, args.drop, args.lower, args.hold, args.cycles, args.viewer)
    except PathError as error:
        print(f"Infeasible path: {error}", file=sys.stderr)
        return 2
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    for key, value in summarize(rows, settle).items():
        print(f"{key:28s} {value:10.3f}")
    if args.viewer:
        # GLFW teardown can hang or segfault after the window closes; the results are already printed.
        sys.stdout.flush()
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
