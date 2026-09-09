#!/usr/bin/env python3
"""Compare the paper Eq.15 q1 formula against the repo q1 formula.

This is a small presentation/demo test for the SpotMicro project.

It does two checks:

1. Analytic FK/IK check:
   - Start from a known leg pose: q1, q2, q3.
   - Use the repo's forward kinematics to compute the foot target.
   - Recover the pose with:
       a) the repo q1 formula
       b) the paper's printed Eq.15 q1 formula
   - Compare which one puts the foot back at the original target.

2. Optional MuJoCo check:
   - Load the SpotMicro MJCF model.
   - Put the front-right leg into the true/repo/paper joint poses.
   - Compare the front-right toe geom position.

No ROS nodes need to be running for this script.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
KINEMATICS_PARENT = (
    PROJECT_ROOT
    / "mike_mujoco_ws"
    / "src"
    / "spotMicro"
    / "spot_micro_plot"
    / "scripts"
)
MUJOCO_MODEL = (
    PROJECT_ROOT
    / "mike_mujoco_ws"
    / "src"
    / "spotMicro"
    / "spot_micro_mujoco_sim"
    / "models"
    / "spot_micro_sim.xml"
)

sys.path.insert(0, str(KINEMATICS_PARENT))

from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk  # noqa: E402


@dataclass
class AngleSet:
    q1_deg: float
    q2_deg: float
    q3_deg: float

    @classmethod
    def from_rad(cls, q: Iterable[float]) -> "AngleSet":
        q1, q2, q3 = q
        return cls(math.degrees(q1), math.degrees(q2), math.degrees(q3))

    def as_rad_tuple(self) -> Tuple[float, float, float]:
        return (
            math.radians(self.q1_deg),
            math.radians(self.q2_deg),
            math.radians(self.q3_deg),
        )


@dataclass
class FormulaCase:
    name: str
    angles_deg: AngleSet
    analytic_foot_xyz_m: List[float]
    analytic_error_m: float
    mujoco_front_right_toe_xyz_m: Optional[List[float]] = None
    mujoco_error_vs_true_m: Optional[float] = None
    q1_within_mujoco_joint_range: Optional[bool] = None


def fmt_vec(v: Iterable[float]) -> str:
    return "[" + ", ".join(f"{x:+.6f}" for x in v) + "]"


def wrap_deg(angle_deg: float) -> float:
    """Wrap an angle to [-180, 180) degrees for human display."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def foot_from_fk(q_rad: Tuple[float, float, float], links: Tuple[float, float, float]) -> np.ndarray:
    t = smk.t_0_to_4(q_rad[0], q_rad[1], q_rad[2], links[0], links[1], links[2])
    return np.array(t[0:3, 3], dtype=float)


def repo_q1_formula(x4: float, y4: float, l1: float) -> float:
    return math.atan2(y4, x4) + math.atan2(math.sqrt(x4 * x4 + y4 * y4 - l1 * l1), -l1)


def paper_literal_q1_formula(x4: float, y4: float, l1: float) -> float:
    """The paper's printed Eq.15 formula as written in Alanah's traceability doc."""
    return -math.atan2(-y4, x4) - math.atan2(math.sqrt(x4 * x4 + y4 * y4 - l1 * l1), -l1)


def inverse_with_repo_q2_q3(
    foot_xyz: np.ndarray,
    links: Tuple[float, float, float],
    use_negative_q3_branch: bool,
) -> Tuple[float, float, float]:
    """Use the repo IK for q2/q3, and repo q1.

    In the SpotMicro code, legs12=True uses the positive-q3 branch and
    legs12=False uses the negative-q3 branch.
    """
    return smk.ikine(
        float(foot_xyz[0]),
        float(foot_xyz[1]),
        float(foot_xyz[2]),
        links[0],
        links[1],
        links[2],
        legs12=not use_negative_q3_branch,
    )


def inverse_with_paper_q1_repo_q2_q3(
    foot_xyz: np.ndarray,
    links: Tuple[float, float, float],
    use_negative_q3_branch: bool,
) -> Tuple[float, float, float]:
    """Use paper Eq.15 for q1, but keep q2/q3 from the repo.

    Alanah's PR found q2 and q3 match the paper; the contested part is q1.
    Keeping q2/q3 identical isolates the q1 formula difference.
    """
    repo_q = inverse_with_repo_q2_q3(foot_xyz, links, use_negative_q3_branch)
    q1 = paper_literal_q1_formula(float(foot_xyz[0]), float(foot_xyz[1]), links[0])
    return q1, repo_q[1], repo_q[2]


def maybe_load_mujoco():
    try:
        import mujoco  # type: ignore

        return mujoco
    except Exception:
        return None


def set_mujoco_front_right_pose(mujoco, model, data, q_rad: Tuple[float, float, float]) -> np.ndarray:
    """Set only the front-right leg pose and return the front-right toe position.

    The sign mapping matches spot_micro_mujoco_sim/mujoco_sim_node.py:
      right side shoulder = +q1
      right side leg      = -q2
      right side foot     = -q3
    """
    mujoco.mj_resetData(model, data)

    # Free joint qpos: [x, y, z, qw, qx, qy, qz]
    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    joint_values = {
        "front_right_shoulder": q_rad[0],
        "front_right_leg": -q_rad[1],
        "front_right_foot": -q_rad[2],
    }
    for joint_name, value in joint_values.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo joint not found: {joint_name}")
        qpos_addr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_addr] = value

    mujoco.mj_forward(model, data)

    toe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "front_right_toe")
    if toe_id < 0:
        raise RuntimeError("MuJoCo geom not found: front_right_toe")
    return np.array(data.geom_xpos[toe_id], dtype=float)


def get_mujoco_joint_range(model, mujoco, joint_name: str) -> Optional[Tuple[float, float]]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None
    if not bool(model.jnt_limited[joint_id]):
        return None
    lo, hi = model.jnt_range[joint_id]
    return float(lo), float(hi)


def build_cases(
    true_angles_deg: AngleSet,
    links: Tuple[float, float, float],
    use_negative_q3_branch: bool,
    run_mujoco: bool,
) -> Dict:
    true_q = true_angles_deg.as_rad_tuple()
    target_foot = foot_from_fk(true_q, links)

    repo_q = inverse_with_repo_q2_q3(target_foot, links, use_negative_q3_branch)
    paper_q = inverse_with_paper_q1_repo_q2_q3(target_foot, links, use_negative_q3_branch)

    q_cases = {
        "true_original_pose": true_q,
        "repo_q1_formula": repo_q,
        "paper_eq15_q1_formula": paper_q,
    }

    cases: Dict[str, FormulaCase] = {}
    for name, q in q_cases.items():
        foot = foot_from_fk(q, links)
        cases[name] = FormulaCase(
            name=name,
            angles_deg=AngleSet.from_rad(q),
            analytic_foot_xyz_m=[float(x) for x in foot],
            analytic_error_m=float(np.linalg.norm(foot - target_foot)),
        )

    mujoco_note = "not requested"
    if run_mujoco:
        mujoco = maybe_load_mujoco()
        if mujoco is None:
            mujoco_note = "mujoco Python package not importable; skipped"
        elif not MUJOCO_MODEL.exists():
            mujoco_note = f"MJCF model not found: {MUJOCO_MODEL}; skipped"
        else:
            model = mujoco.MjModel.from_xml_path(str(MUJOCO_MODEL))
            data = mujoco.MjData(model)
            true_toe = set_mujoco_front_right_pose(mujoco, model, data, true_q)
            shoulder_range = get_mujoco_joint_range(model, mujoco, "front_right_shoulder")

            for name, q in q_cases.items():
                toe = set_mujoco_front_right_pose(mujoco, model, data, q)
                case = cases[name]
                case.mujoco_front_right_toe_xyz_m = [float(x) for x in toe]
                case.mujoco_error_vs_true_m = float(np.linalg.norm(toe - true_toe))
                if shoulder_range is not None:
                    case.q1_within_mujoco_joint_range = bool(
                        shoulder_range[0] <= q[0] <= shoulder_range[1]
                    )
            mujoco_note = f"loaded {MUJOCO_MODEL}"

    return {
        "links_m": {
            "l1_hip_side_swing": links[0],
            "l2_upper_leg": links[1],
            "l3_lower_leg": links[2],
        },
        "true_angles_deg": asdict(true_angles_deg),
        "q3_branch": "negative" if use_negative_q3_branch else "positive",
        "target_foot_xyz_m": [float(x) for x in target_foot],
        "cases": {name: asdict(case) for name, case in cases.items()},
        "mujoco_note": mujoco_note,
    }


def print_report(results: Dict) -> None:
    print("\n=== SpotMicro q1 formula comparison ===\n")
    print("Plain-English point:")
    print("  q1 is the side-swing / shoulder angle.")
    print("  Alanah's PR says the paper's printed q1 equation is not the inverse")
    print("  of the paper's own forward kinematics, while the repo q1 equation is.\n")

    print("Input / known-good pose:")
    t = results["true_angles_deg"]
    print(f"  true q1,q2,q3 = {t['q1_deg']:+.4f}°, {t['q2_deg']:+.4f}°, {t['q3_deg']:+.4f}°")
    print(f"  generated foot target xyz = {fmt_vec(results['target_foot_xyz_m'])} m")
    print(f"  q3 branch used for inverse test = {results['q3_branch']}\n")

    print("Analytic FK check:")
    for key in ["true_original_pose", "repo_q1_formula", "paper_eq15_q1_formula"]:
        case = results["cases"][key]
        a = case["angles_deg"]
        print(f"  {key}:")
        print(
            "    q1,q2,q3 = "
            f"{a['q1_deg']:+.4f}°, {a['q2_deg']:+.4f}°, {a['q3_deg']:+.4f}°"
        )
        if key == "paper_eq15_q1_formula":
            print(f"    q1 wrapped to [-180,180) = {wrap_deg(a['q1_deg']):+.4f}°")
        print(f"    foot xyz = {fmt_vec(case['analytic_foot_xyz_m'])} m")
        print(f"    error vs target = {case['analytic_error_m']:.9f} m")
    print()

    print("MuJoCo check:")
    print(f"  {results['mujoco_note']}")
    any_mujoco = any(
        case.get("mujoco_front_right_toe_xyz_m") is not None
        for case in results["cases"].values()
    )
    if any_mujoco:
        for key in ["true_original_pose", "repo_q1_formula", "paper_eq15_q1_formula"]:
            case = results["cases"][key]
            print(f"  {key}:")
            print(f"    front_right_toe xyz = {fmt_vec(case['mujoco_front_right_toe_xyz_m'])} m")
            print(f"    toe error vs true pose = {case['mujoco_error_vs_true_m']:.9f} m")
            in_range = case.get("q1_within_mujoco_joint_range")
            if in_range is not None:
                print(f"    q1 inside MuJoCo shoulder range = {in_range}")
    print()

    paper = results["cases"]["paper_eq15_q1_formula"]
    repo = results["cases"]["repo_q1_formula"]
    print("Conclusion:")
    if repo["analytic_error_m"] < 1e-9 and paper["analytic_error_m"] > 1e-4:
        print("  PASS: repo q1 reconstructs the target; paper Eq.15 q1 does not.")
    else:
        print("  CHECK RESULTS: expected repo error near zero and paper error clearly nonzero.")
    print()


def run_viewer(results: Dict, cycle_seconds: float) -> None:
    """Open a MuJoCo viewer and alternate front-right leg poses."""
    mujoco = maybe_load_mujoco()
    if mujoco is None:
        raise RuntimeError("mujoco Python package is not importable")

    import mujoco.viewer  # type: ignore

    model = mujoco.MjModel.from_xml_path(str(MUJOCO_MODEL))
    data = mujoco.MjData(model)

    sequence = ["repo_q1_formula", "paper_eq15_q1_formula"]
    q_sequence = []
    for key in sequence:
        a = results["cases"][key]["angles_deg"]
        q_sequence.append(
            (
                key,
                (
                    math.radians(a["q1_deg"]),
                    math.radians(a["q2_deg"]),
                    math.radians(a["q3_deg"]),
                ),
            )
        )

    print("Opening MuJoCo viewer.")
    print("It will alternate the front-right leg between repo q1 and paper Eq.15 q1.")
    print("Close the viewer window or press Ctrl+C in this terminal to stop.")

    idx = 0
    last_switch = 0.0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            now = time.monotonic()
            if now - last_switch >= cycle_seconds:
                name, q = q_sequence[idx % len(q_sequence)]
                set_mujoco_front_right_pose(mujoco, model, data, q)
                print(f"viewer pose: {name}")
                idx += 1
                last_switch = now
            viewer.sync()
            time.sleep(0.02)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare paper Eq.15 q1 against repo q1, optionally in MuJoCo."
    )
    parser.add_argument(
        "--true-angles-deg",
        nargs=3,
        type=float,
        metavar=("Q1", "Q2", "Q3"),
        default=(20.0, 15.0, -25.0),
        help="Known-good q1 q2 q3 leg angles in degrees.",
    )
    parser.add_argument(
        "--links-m",
        nargs=3,
        type=float,
        metavar=("L1", "L2", "L3"),
        default=(0.055, 0.1075, 0.130),
        help="SpotMicro leg link lengths in meters.",
    )
    parser.add_argument(
        "--positive-q3-branch",
        action="store_true",
        help="Use positive q3 IK branch. Default uses negative branch, matching q3=-25 deg.",
    )
    parser.add_argument(
        "--skip-mujoco",
        action="store_true",
        help="Only run the analytic FK/IK check; do not load MuJoCo.",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open a MuJoCo viewer cycling between repo q1 and paper Eq.15 q1.",
    )
    parser.add_argument(
        "--cycle-seconds",
        type=float,
        default=2.0,
        help="Viewer cycle time between repo and paper poses.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        help="Optional path to write machine-readable JSON results.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    true_angles = AngleSet(*args.true_angles_deg)
    links = tuple(args.links_m)

    results = build_cases(
        true_angles_deg=true_angles,
        links=links,  # type: ignore[arg-type]
        use_negative_q3_branch=not args.positive_q3_branch,
        run_mujoco=not args.skip_mujoco or args.viewer,
    )
    print_report(results)

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(results, indent=2) + "\n")
        print(f"Wrote JSON results: {args.json_out}")

    if args.viewer:
        run_viewer(results, args.cycle_seconds)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

