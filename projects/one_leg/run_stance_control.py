#!/usr/bin/env python3
"""Hold a simulated SpotMicro right toe at a fixed ground location.

The model represents a single physical leg mounted in a constrained test rig.
MuJoCo provides joint dynamics and floor contact. A damped least-squares
Jacobian controller updates three position-servo references to reject slow
motion of the mounting carriage.

No ROS installation is required.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
DEFAULT_MODEL = HERE / "model" / "right_leg_test_rig.xml"
JOINT_NAMES = ("front_right_shoulder", "front_right_leg", "front_right_foot")
ACTUATOR_NAMES = (
    "front_right_shoulder_actuator",
    "front_right_leg_actuator",
    "front_right_foot_actuator",
)
TOE_GEOM = "right_toe"
FLOOR_GEOM = "floor"
TOE_RADIUS_M = 0.018


@dataclass
class Sample:
    time_s: float
    target_x_m: float
    target_y_m: float
    target_z_m: float
    toe_x_m: float
    toe_y_m: float
    toe_z_m: float
    error_m: float
    in_contact: int
    normal_force_n: float
    q1_rad: float
    q2_rad: float
    q3_rad: float
    q1_cmd_rad: float
    q2_cmd_rad: float
    q3_cmd_rad: float


def object_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, object_type, name)
    if result < 0:
        raise RuntimeError(f"MuJoCo object not found: {name}")
    return result


def joint_addresses(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    joint_ids = np.array(
        [object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES],
        dtype=int,
    )
    qpos = np.array([model.jnt_qposadr[joint_id] for joint_id in joint_ids], dtype=int)
    dofs = np.array([model.jnt_dofadr[joint_id] for joint_id in joint_ids], dtype=int)
    return qpos, dofs


def actuator_ids(model: mujoco.MjModel) -> np.ndarray:
    return np.array(
        [object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATOR_NAMES],
        dtype=int,
    )


def clamp_controls(model: mujoco.MjModel, ids: np.ndarray, values: np.ndarray) -> np.ndarray:
    result = values.copy()
    for index, actuator_id in enumerate(ids):
        if model.actuator_ctrllimited[actuator_id]:
            low, high = model.actuator_ctrlrange[actuator_id]
            result[index] = np.clip(result[index], low, high)
    return result


def toe_contact_force(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    toe_geom_id: int,
    floor_geom_id: int,
) -> tuple[bool, float]:
    in_contact = False
    normal_force = 0.0
    contact_force = np.zeros(6, dtype=float)
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        pair = {int(contact.geom1), int(contact.geom2)}
        if pair != {toe_geom_id, floor_geom_id}:
            continue
        in_contact = True
        mujoco.mj_contactForce(model, data, contact_index, contact_force)
        normal_force += max(0.0, float(contact_force[0]))
    return in_contact, normal_force


def carriage_position(initial: np.ndarray, sim_time: float, disturb: bool) -> np.ndarray:
    if not disturb or sim_time < 2.0:
        return initial.copy()
    elapsed = sim_time - 2.0
    result = initial.copy()
    result[0] += 0.010 * math.sin(2.0 * math.pi * 0.20 * elapsed)
    result[2] += 0.012 * math.sin(2.0 * math.pi * 0.30 * elapsed)
    return result


def controller_step(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    toe_geom_id: int,
    joint_dofs: np.ndarray,
    actuator_id_array: np.ndarray,
    target: np.ndarray,
    command: np.ndarray,
    gain: float,
    damping: float,
    max_step_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    jacobian_position = np.zeros((3, model.nv), dtype=float)
    jacobian_rotation = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacGeom(
        model,
        data,
        jacobian_position,
        jacobian_rotation,
        toe_geom_id,
    )
    jacobian = jacobian_position[:, joint_dofs]
    toe = np.array(data.geom_xpos[toe_geom_id], dtype=float)
    error = target - toe

    regularized = jacobian @ jacobian.T + (damping * damping) * np.eye(3)
    delta_q = gain * jacobian.T @ np.linalg.solve(regularized, error)
    delta_q = np.clip(delta_q, -max_step_rad, max_step_rad)
    command = clamp_controls(model, actuator_id_array, command + delta_q)
    data.ctrl[actuator_id_array] = command
    return command, error


def write_csv(path: Path, samples: Iterable[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(samples)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(Sample.__dataclass_fields__))
        writer.writeheader()
        for sample in rows:
            writer.writerow(sample.__dict__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--no-disturbance", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--control-hz", type=float, default=100.0)
    parser.add_argument("--gain", type=float, default=0.35)
    parser.add_argument("--damping", type=float, default=0.025)
    parser.add_argument("--max-step-deg", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0 or args.control_hz <= 0:
        raise SystemExit("duration and control-hz must be positive")

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    stance_key = object_id(model, mujoco.mjtObj.mjOBJ_KEY, "stance")
    mujoco.mj_resetDataKeyframe(model, data, stance_key)

    toe_id = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, TOE_GEOM)
    floor_id = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_GEOM)
    joint_qpos, joint_dofs = joint_addresses(model)
    actuators = actuator_ids(model)
    initial_carriage = np.array(model.body_pos[object_id(model, mujoco.mjtObj.mjOBJ_BODY, "test_carriage")])
    data.mocap_pos[0] = initial_carriage
    mujoco.mj_forward(model, data)

    # Anchor x/y at the initial toe location and put the sphere tangent to floor.
    toe_initial = np.array(data.geom_xpos[toe_id], dtype=float)
    target = np.array([toe_initial[0], toe_initial[1], TOE_RADIUS_M], dtype=float)
    command = np.array(data.ctrl[actuators], dtype=float)
    control_period = 1.0 / args.control_hz
    next_control_time = 0.0
    max_step_rad = math.radians(args.max_step_deg)
    samples: list[Sample] = []

    viewer_context = None
    if args.viewer:
        import mujoco.viewer

        viewer_context = mujoco.viewer.launch_passive(model, data)
        viewer_context.cam.lookat[:] = [0.0, -0.03, 0.12]
        viewer_context.cam.distance = 0.75
        viewer_context.cam.azimuth = 145
        viewer_context.cam.elevation = -18

    wall_start = time.perf_counter()
    try:
        while data.time < args.duration:
            if viewer_context is not None and not viewer_context.is_running():
                break

            data.mocap_pos[0] = carriage_position(
                initial_carriage, data.time, not args.no_disturbance
            )
            if data.time + 1e-12 >= next_control_time:
                command, _ = controller_step(
                    model,
                    data,
                    toe_id,
                    joint_dofs,
                    actuators,
                    target,
                    command,
                    args.gain,
                    args.damping,
                    max_step_rad,
                )
                next_control_time += control_period

            mujoco.mj_step(model, data)
            toe = np.array(data.geom_xpos[toe_id], dtype=float)
            error = float(np.linalg.norm(target - toe))
            contact, normal_force = toe_contact_force(model, data, toe_id, floor_id)
            q = np.array(data.qpos[joint_qpos], dtype=float)
            samples.append(
                Sample(
                    time_s=float(data.time),
                    target_x_m=float(target[0]),
                    target_y_m=float(target[1]),
                    target_z_m=float(target[2]),
                    toe_x_m=float(toe[0]),
                    toe_y_m=float(toe[1]),
                    toe_z_m=float(toe[2]),
                    error_m=error,
                    in_contact=int(contact),
                    normal_force_n=normal_force,
                    q1_rad=float(q[0]),
                    q2_rad=float(q[1]),
                    q3_rad=float(q[2]),
                    q1_cmd_rad=float(command[0]),
                    q2_cmd_rad=float(command[1]),
                    q3_cmd_rad=float(command[2]),
                )
            )

            if viewer_context is not None:
                viewer_context.sync()
                target_wall = wall_start + data.time
                remaining = target_wall - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        if viewer_context is not None:
            viewer_context.close()

    if args.output:
        write_csv(args.output, samples)

    if not samples:
        print("No samples produced", file=sys.stderr)
        return 2

    errors = np.array([sample.error_m for sample in samples])
    contacts = np.array([sample.in_contact for sample in samples], dtype=float)
    forces = np.array([sample.normal_force_n for sample in samples])
    report_start = min(len(samples) - 1, int(2.0 / model.opt.timestep))
    evaluated_errors = errors[report_start:]
    evaluated_contacts = contacts[report_start:]
    evaluated_forces = forces[report_start:]

    print("SpotMicro one-right-leg stance report")
    print(f"  simulated time:        {samples[-1].time_s:.3f} s")
    print(f"  RMS toe error:         {np.sqrt(np.mean(evaluated_errors ** 2)) * 1000:.2f} mm")
    print(f"  maximum toe error:     {np.max(evaluated_errors) * 1000:.2f} mm")
    print(f"  floor-contact fraction:{np.mean(evaluated_contacts) * 100:6.2f} %")
    print(f"  peak normal force:     {np.max(evaluated_forces):.2f} N")
    if args.output:
        print(f"  CSV log:               {args.output.resolve()}")

    # Broad smoke-test limits catch divergence, not controller certification.
    stable = bool(
        np.isfinite(errors).all()
        and np.max(evaluated_errors) < 0.050
        and np.mean(evaluated_contacts) > 0.70
    )
    print(f"  smoke test:            {'PASS' if stable else 'FAIL'}")
    return 0 if stable else 1


if __name__ == "__main__":
    raise SystemExit(main())
