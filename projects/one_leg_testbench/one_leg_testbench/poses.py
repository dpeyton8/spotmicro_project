"""Replays serial-sketch poses on the free-hanging left-leg rig: default servo position -> pose -> default."""

from __future__ import annotations

import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from . import leg
from .sim import MODEL

# IK of spot_micro_motion_cmd.yaml poses: stand = 0.155 m below hip, 0.015 m forward; idle = lie_down 0.083 m below, 0.065 m forward.
POSES = {
    "stand": (0.0, 50.1, -98.7),
    "sit": (0.0, 37.2, -128.4),
}
STEP_SECONDS = 0.02
MOUNT_Z = 0.35


def firmware_steps(target_deg, start_deg=None):
    """Joint angles after each 1-degree step of moveJoint() in the sketch, joints in order, from start (default position if omitted)."""
    current = np.degrees(leg.CENTER_RAD).astype(float) if start_deg is None else np.array(start_deg, dtype=float)
    steps = []
    for joint in range(3):
        goal = target_deg[joint]
        direction = 1.0 if goal >= current[joint] else -1.0
        while (goal - current[joint]) * direction > 1.0:
            current[joint] += direction
            steps.append(current.copy())
        current[joint] = goal
        steps.append(current.copy())
    return steps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose", nargs="?", choices=[*POSES, "both", "stand-sit"], default="both")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hold", type=float, default=1.5, help="seconds held at each pose")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    neutral = np.degrees(leg.CENTER_RAD)
    data.qpos[:3] = leg.to_ros(leg.CENTER_RAD)
    data.ctrl[:3] = leg.to_ros(leg.CENTER_RAD)
    data.mocap_pos[0] = [0, 0, MOUNT_Z]
    mujoco.mj_forward(model, data)

    handle = mujoco.viewer.launch_passive(model, data) if args.viewer else None
    substeps = max(1, round(STEP_SECONDS / model.opt.timestep))

    def command(angles_deg, seconds, label):
        q = np.radians(angles_deg)
        data.ctrl[:3] = leg.to_ros(q)
        if label:
            print(f"{label:22s} {leg.firmware_line(q)}   PWM {','.join(map(str, leg.firmware_pwm(q)))}")
        end = data.time + seconds
        while data.time < end:
            if handle is not None and not handle.is_running():
                return False
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            if handle is not None:
                handle.sync()
                time.sleep(STEP_SECONDS)
        return True

    names = list(POSES) if args.pose == "both" else [args.pose]
    if args.pose == "stand-sit":
        stand, sit = np.array(POSES["stand"]), np.array(POSES["sit"])
        ok = command(neutral, args.hold, "default position")
        for step in firmware_steps(stand):
            ok = ok and command(step, STEP_SECONDS, "")
        ok = ok and command(stand, args.hold, "stand reached")
        for step in firmware_steps(sit, stand):
            ok = ok and command(step, STEP_SECONDS, "")
        ok = ok and command(sit, args.hold, "sit reached")
        print(f"simulated joints: {','.join(f'{v:.1f}' for v in np.degrees(leg.from_ros(data.qpos[:3])))}")
        names = []
    for name in names:
        target = np.array(POSES[name])
        bad = leg.violated_joints(np.radians(target))
        if len(bad):
            print(f"warning: {name} is outside the testbench limits on joint index {list(bad)}")
        ok = command(neutral, args.hold, "default position")
        for step in firmware_steps(target):
            ok = ok and command(step, STEP_SECONDS, "")
        ok = ok and command(target, args.hold, f"{name} reached")
        actual = np.degrees(leg.from_ros(data.qpos[:3]))
        print(f"{name} simulated joints: {','.join(f'{v:.1f}' for v in actual)}")
        if not ok:
            break
    if handle is not None:
        while handle.is_running():
            time.sleep(0.05)
        sys.stdout.flush()
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
