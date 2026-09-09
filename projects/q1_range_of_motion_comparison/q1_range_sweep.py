#!/usr/bin/env python3
"""Range-of-motion comparison for repo q1 vs paper Eq.15 q1.

This builds on the single-pose q1 test, but sweeps q1 across a shoulder range.
It shows whether each equation can recover a whole family of intended side-swing
angles, not just one example.

No ROS nodes are required.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
import textwrap
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont


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


Color = Tuple[int, int, int]
Callout = Tuple[float, float, str, Color, int, int]

BLACK: Color = (20, 20, 20)
GRAY: Color = (130, 130, 130)
LIGHT_GRAY: Color = (224, 224, 224)
RED: Color = (190, 30, 45)
TEAL: Color = (0, 115, 125)
BLUE: Color = (35, 95, 180)
ORANGE: Color = (220, 125, 25)
GREEN: Color = (20, 140, 75)
WHITE: Color = (255, 255, 255)


def load_font(size: int = 16):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size)
        except Exception:
            pass
    return ImageFont.load_default()


def wrap_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def foot_from_fk(q_rad: Tuple[float, float, float], links: Tuple[float, float, float]) -> np.ndarray:
    t = smk.t_0_to_4(q_rad[0], q_rad[1], q_rad[2], links[0], links[1], links[2])
    return np.array(t[0:3, 3], dtype=float)


def repo_q1_formula(x4: float, y4: float, l1: float) -> float:
    return math.atan2(y4, x4) + math.atan2(math.sqrt(x4 * x4 + y4 * y4 - l1 * l1), -l1)


def paper_literal_q1_formula(x4: float, y4: float, l1: float) -> float:
    return -math.atan2(-y4, x4) - math.atan2(math.sqrt(x4 * x4 + y4 * y4 - l1 * l1), -l1)


def maybe_load_mujoco():
    try:
        import mujoco  # type: ignore

        return mujoco
    except Exception:
        return None


def set_mujoco_front_right_pose(mujoco, model, data, q_rad: Tuple[float, float, float]) -> np.ndarray:
    """Set front-right qpos and return front-right toe world xyz.

    Sign mapping follows spot_micro_mujoco_sim/mujoco_sim_node.py:
      front_right_shoulder = q1
      front_right_leg      = -q2
      front_right_foot     = -q3
    """
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0.0, 0.0, 0.25]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    joint_values = {
        "front_right_shoulder": q_rad[0],
        "front_right_leg": -q_rad[1],
        "front_right_foot": -q_rad[2],
    }
    for joint_name, value in joint_values.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_adr = model.jnt_qposadr[jid]
        data.qpos[qpos_adr] = value

    mujoco.mj_forward(model, data)
    toe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "front_right_toe")
    return np.array(data.geom_xpos[toe_id], dtype=float)


def get_front_right_shoulder_range(mujoco, model) -> Optional[Tuple[float, float]]:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "front_right_shoulder")
    if jid < 0 or not bool(model.jnt_limited[jid]):
        return None
    lo, hi = model.jnt_range[jid]
    return float(lo), float(hi)


def compute_rows(
    q1_min_deg: float,
    q1_max_deg: float,
    samples: int,
    q2_deg: float,
    q3_deg: float,
    links: Tuple[float, float, float],
    include_mujoco: bool,
) -> Tuple[List[Dict], Dict]:
    rows: List[Dict] = []

    mujoco_note = "not requested"
    mujoco = None
    model = None
    data = None
    shoulder_range = None
    if include_mujoco:
        mujoco = maybe_load_mujoco()
        if mujoco is None:
            mujoco_note = "mujoco Python package not importable; skipped"
        elif not MUJOCO_MODEL.exists():
            mujoco_note = f"MuJoCo model not found: {MUJOCO_MODEL}; skipped"
        else:
            model = mujoco.MjModel.from_xml_path(str(MUJOCO_MODEL))
            data = mujoco.MjData(model)
            shoulder_range = get_front_right_shoulder_range(mujoco, model)
            mujoco_note = f"loaded {MUJOCO_MODEL}"

    for q1_deg in np.linspace(q1_min_deg, q1_max_deg, samples):
        true_q = (math.radians(float(q1_deg)), math.radians(q2_deg), math.radians(q3_deg))
        target = foot_from_fk(true_q, links)

        # q2/q3 are recovered by the repo IK; only q1 is swapped.
        repo_q = smk.ikine(
            float(target[0]), float(target[1]), float(target[2]),
            links[0], links[1], links[2],
            legs12=False,  # negative q3 branch, matching q3_deg=-25 default
        )
        paper_q = (
            paper_literal_q1_formula(float(target[0]), float(target[1]), links[0]),
            repo_q[1],
            repo_q[2],
        )

        repo_foot = foot_from_fk(repo_q, links)
        paper_foot = foot_from_fk(paper_q, links)

        row = {
            "true_q1_deg": float(q1_deg),
            "true_q2_deg": q2_deg,
            "true_q3_deg": q3_deg,
            "target_x_m": float(target[0]),
            "target_y_m": float(target[1]),
            "target_z_m": float(target[2]),
            "repo_q1_deg": math.degrees(repo_q[0]),
            "repo_q1_wrapped_deg": wrap_deg(math.degrees(repo_q[0])),
            "repo_q2_deg": math.degrees(repo_q[1]),
            "repo_q3_deg": math.degrees(repo_q[2]),
            "repo_foot_x_m": float(repo_foot[0]),
            "repo_foot_y_m": float(repo_foot[1]),
            "repo_foot_z_m": float(repo_foot[2]),
            "repo_foot_error_m": float(np.linalg.norm(repo_foot - target)),
            "paper_q1_deg": math.degrees(paper_q[0]),
            "paper_q1_wrapped_deg": wrap_deg(math.degrees(paper_q[0])),
            "paper_q2_deg": math.degrees(paper_q[1]),
            "paper_q3_deg": math.degrees(paper_q[2]),
            "paper_foot_x_m": float(paper_foot[0]),
            "paper_foot_y_m": float(paper_foot[1]),
            "paper_foot_z_m": float(paper_foot[2]),
            "paper_foot_error_m": float(np.linalg.norm(paper_foot - target)),
        }

        if mujoco is not None and model is not None and data is not None:
            true_toe = set_mujoco_front_right_pose(mujoco, model, data, true_q)
            repo_toe = set_mujoco_front_right_pose(mujoco, model, data, repo_q)
            paper_toe = set_mujoco_front_right_pose(mujoco, model, data, paper_q)
            row.update({
                "repo_mujoco_toe_error_m": float(np.linalg.norm(repo_toe - true_toe)),
                "paper_mujoco_toe_error_m": float(np.linalg.norm(paper_toe - true_toe)),
                "repo_mujoco_toe_x_m": float(repo_toe[0]),
                "repo_mujoco_toe_y_m": float(repo_toe[1]),
                "repo_mujoco_toe_z_m": float(repo_toe[2]),
                "paper_mujoco_toe_x_m": float(paper_toe[0]),
                "paper_mujoco_toe_y_m": float(paper_toe[1]),
                "paper_mujoco_toe_z_m": float(paper_toe[2]),
            })
            if shoulder_range is not None:
                row["repo_q1_inside_mujoco_range"] = shoulder_range[0] <= repo_q[0] <= shoulder_range[1]
                row["paper_q1_inside_mujoco_range"] = shoulder_range[0] <= paper_q[0] <= shoulder_range[1]

        rows.append(row)

    meta = {
        "links_m": {"l1": links[0], "l2": links[1], "l3": links[2]},
        "q1_sweep_deg": [q1_min_deg, q1_max_deg],
        "q2_deg": q2_deg,
        "q3_deg": q3_deg,
        "samples": samples,
        "mujoco_note": mujoco_note,
        "mujoco_front_right_shoulder_range_deg": (
            [math.degrees(shoulder_range[0]), math.degrees(shoulder_range[1])]
            if shoulder_range is not None else None
        ),
    }
    return rows, meta


def data_range(series: Sequence[Sequence[float]], pad: float = 0.08) -> Tuple[float, float]:
    vals = [v for s in series for v in s if math.isfinite(v)]
    lo, hi = min(vals), max(vals)
    if abs(hi - lo) < 1e-12:
        lo -= 1.0
        hi += 1.0
    extra = (hi - lo) * pad
    return lo - extra, hi + extra


def text_size(draw: ImageDraw.ImageDraw, text: str, font) -> Tuple[int, int]:
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0], bbox[3] - bbox[1]


def draw_centered_text(
    draw: ImageDraw.ImageDraw,
    center_x: int,
    y: int,
    text: str,
    fill: Color,
    font,
) -> None:
    w, _ = text_size(draw, text, font)
    draw.text((center_x - w // 2, y), text, fill=fill, font=font)


def format_tick(value: float) -> str:
    if abs(value) >= 10 or abs(value - round(value)) < 1e-9:
        return f"{value:.0f}"
    return f"{value:.2f}"


def draw_callout(
    draw: ImageDraw.ImageDraw,
    px: int,
    py: int,
    text: str,
    color: Color,
    dx: int,
    dy: int,
    font,
) -> None:
    tx, ty = px + dx, py + dy
    tw, th = text_size(draw, text, font)
    box = (tx - 8, ty - 6, tx + tw + 8, ty + th + 8)
    draw.line((px, py, tx, ty), fill=color, width=2)
    draw.rectangle(box, fill=WHITE, outline=color, width=2)
    draw.text((tx, ty), text, fill=BLACK, font=font)


def draw_line_plot(
    path: Path,
    title: str,
    x_label: str,
    y_label: str,
    x_values: List[float],
    series: List[Tuple[str, List[float], Color]],
    x_bounds: Optional[Tuple[float, float]] = None,
    y_bounds: Optional[Tuple[float, float]] = None,
    x_ticks: Optional[List[float]] = None,
    y_ticks: Optional[List[float]] = None,
    reference_lines: Optional[List[Tuple[float, Color, str]]] = None,
    callouts: Optional[List[Callout]] = None,
) -> None:
    width, height = 1400, 900
    margin_left, margin_right = 140, 70
    margin_top, margin_bottom = 140, 210
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    if x_bounds:
        x_min, x_max = x_bounds
    else:
        x_min, x_max = data_range([x_values], pad=0.03)
    if y_bounds:
        y_min, y_max = y_bounds
    else:
        y_min, y_max = data_range([s[1] for s in series], pad=0.10)

    def sx(x: float) -> int:
        return int(margin_left + (x - x_min) / (x_max - x_min) * plot_w)

    def sy(y: float) -> int:
        return int(margin_top + (y_max - y) / (y_max - y_min) * plot_h)

    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    font_title = load_font(30)
    font = load_font(18)
    small = load_font(15)

    title_lines = textwrap.wrap(title, width=78)
    for i, line in enumerate(title_lines[:2]):
        draw.text((margin_left, 25 + i * 34), line, fill=BLACK, font=font_title)

    # Grid and axes.
    x_tick_values = x_ticks if x_ticks is not None else [
        x_min + i * (x_max - x_min) / 5 for i in range(6)
    ]
    y_tick_values = y_ticks if y_ticks is not None else [
        y_max - i * (y_max - y_min) / 5 for i in range(6)
    ]

    for xv in x_tick_values:
        if not x_min <= xv <= x_max:
            continue
        x = sx(xv)
        draw.line((x, margin_top, x, margin_top + plot_h), fill=LIGHT_GRAY, width=1)
        draw_centered_text(draw, x, margin_top + plot_h + 14, format_tick(xv), BLACK, small)
    for yv in y_tick_values:
        if not y_min <= yv <= y_max:
            continue
        y = sy(yv)
        draw.line((margin_left, y, margin_left + plot_w, y), fill=LIGHT_GRAY, width=1)
        tick_text = format_tick(yv)
        tw, _ = text_size(draw, tick_text, small)
        draw.text((margin_left - tw - 18, y - 9), tick_text, fill=BLACK, font=small)

    draw.rectangle((margin_left, margin_top, margin_left + plot_w, margin_top + plot_h), outline=BLACK, width=2)
    draw_centered_text(draw, margin_left + plot_w // 2, margin_top + plot_h + 58, x_label, BLACK, font)
    draw.text((margin_left, margin_top - 34), y_label, fill=BLACK, font=font)

    if reference_lines:
        for y_ref, color, label in reference_lines:
            if y_min <= y_ref <= y_max:
                y = sy(y_ref)
                draw.line((margin_left, y, margin_left + plot_w, y), fill=color, width=2)
                draw.text((margin_left + plot_w - 230, y - 24), label, fill=color, font=small)

    legend_x = margin_left
    legend_y = margin_top + plot_h + 120
    for name, vals, color in series:
        points = [(sx(x), sy(y)) for x, y in zip(x_values, vals)]
        if len(points) > 1:
            draw.line(points, fill=color, width=4)
        draw.line((legend_x, legend_y, legend_x + 35, legend_y), fill=color, width=5)
        draw.text((legend_x + 45, legend_y - 10), name, fill=BLACK, font=small)
        legend_x += 355

    if callouts:
        for x_data, y_data, text, color, dx, dy in callouts:
            if x_min <= x_data <= x_max and y_min <= y_data <= y_max:
                draw_callout(draw, sx(x_data), sy(y_data), text, color, dx, dy, small)

    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)


def draw_xy_plot(
    path: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: List[Tuple[str, List[Tuple[float, float]], Color]],
    callouts: Optional[List[Callout]] = None,
) -> None:
    width, height = 1050, 980
    margin_left, margin_right = 120, 70
    margin_top, margin_bottom = 130, 190
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    xs = [[p[0] for p in pts] for _, pts, _ in series]
    ys = [[p[1] for p in pts] for _, pts, _ in series]
    x_min, x_max = data_range(xs, pad=0.14)
    y_min, y_max = data_range(ys, pad=0.14)

    # Equal-ish scale for a less misleading geometry view.
    x_mid, y_mid = (x_min + x_max) / 2, (y_min + y_max) / 2
    span = max(x_max - x_min, y_max - y_min)
    x_min, x_max = x_mid - span / 2, x_mid + span / 2
    y_min, y_max = y_mid - span / 2, y_mid + span / 2

    def sx(x: float) -> int:
        return int(margin_left + (x - x_min) / (x_max - x_min) * plot_w)

    def sy(y: float) -> int:
        return int(margin_top + (y_max - y) / (y_max - y_min) * plot_h)

    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    font_title = load_font(28)
    font = load_font(17)
    small = load_font(14)

    title_lines = textwrap.wrap(title, width=70)
    for i, line in enumerate(title_lines[:2]):
        draw.text((margin_left, 25 + i * 32), line, fill=BLACK, font=font_title)
    for i in range(6):
        x = margin_left + int(i * plot_w / 5)
        y = margin_top + int(i * plot_h / 5)
        draw.line((x, margin_top, x, margin_top + plot_h), fill=LIGHT_GRAY, width=1)
        draw.line((margin_left, y, margin_left + plot_w, y), fill=LIGHT_GRAY, width=1)
        xv = x_min + i * (x_max - x_min) / 5
        yv = y_max - i * (y_max - y_min) / 5
        draw_centered_text(draw, x, margin_top + plot_h + 10, f"{xv:.2f}", BLACK, small)
        y_text = f"{yv:.2f}"
        tw, _ = text_size(draw, y_text, small)
        draw.text((margin_left - tw - 18, y - 8), y_text, fill=BLACK, font=small)

    draw.rectangle((margin_left, margin_top, margin_left + plot_w, margin_top + plot_h), outline=BLACK, width=2)
    draw_centered_text(draw, margin_left + plot_w // 2, margin_top + plot_h + 58, x_label, BLACK, font)
    draw.text((margin_left, margin_top - 32), y_label, fill=BLACK, font=font)

    legend_x = margin_left
    legend_y = margin_top + plot_h + 118
    for name, pts, color in series:
        pixel_points = [(sx(x), sy(y)) for x, y in pts]
        if len(pixel_points) > 1:
            draw.line(pixel_points, fill=color, width=4)
        for px, py in pixel_points[::max(1, len(pixel_points)//20)]:
            draw.ellipse((px - 3, py - 3, px + 3, py + 3), fill=color)
        draw.line((legend_x, legend_y, legend_x + 35, legend_y), fill=color, width=5)
        draw.text((legend_x + 45, legend_y - 10), name, fill=BLACK, font=small)
        legend_x += 310

    if callouts:
        for x_data, y_data, text, color, dx, dy in callouts:
            if x_min <= x_data <= x_max and y_min <= y_data <= y_max:
                draw_callout(draw, sx(x_data), sy(y_data), text, color, dx, dy, small)

    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)


def write_outputs(rows: List[Dict], meta: Dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "q1_range_sweep_results.csv"
    json_path = out_dir / "q1_range_sweep_summary.json"

    fieldnames = list(rows[0].keys())
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    repo_max_error = max(r["repo_foot_error_m"] for r in rows)
    paper_min_error = min(r["paper_foot_error_m"] for r in rows)
    paper_max_error = max(r["paper_foot_error_m"] for r in rows)
    paper_mean_error = sum(r["paper_foot_error_m"] for r in rows) / len(rows)
    paper_q1_in_range_count = sum(1 for r in rows if r.get("paper_q1_inside_mujoco_range") is True)
    repo_q1_in_range_count = sum(1 for r in rows if r.get("repo_q1_inside_mujoco_range") is True)

    summary = {
        **meta,
        "repo_max_analytic_foot_error_m": repo_max_error,
        "paper_min_analytic_foot_error_m": paper_min_error,
        "paper_max_analytic_foot_error_m": paper_max_error,
        "paper_mean_analytic_foot_error_m": paper_mean_error,
        "repo_q1_inside_mujoco_range_count": repo_q1_in_range_count,
        "paper_q1_inside_mujoco_range_count": paper_q1_in_range_count,
        "sample_count": len(rows),
    }
    if "paper_mujoco_toe_error_m" in rows[0]:
        summary["repo_max_mujoco_toe_error_m"] = max(r["repo_mujoco_toe_error_m"] for r in rows)
        summary["paper_min_mujoco_toe_error_m"] = min(r["paper_mujoco_toe_error_m"] for r in rows)
        summary["paper_max_mujoco_toe_error_m"] = max(r["paper_mujoco_toe_error_m"] for r in rows)
        summary["paper_mean_mujoco_toe_error_m"] = (
            sum(r["paper_mujoco_toe_error_m"] for r in rows) / len(rows)
        )

    json_path.write_text(json.dumps(summary, indent=2) + "\n")

    x = [r["true_q1_deg"] for r in rows]
    x_bounds = (min(x), max(x))
    x_ticks = [min(x), min(x) / 2.0, 0.0, max(x) / 2.0, max(x)]
    paper_error = sum(r["paper_foot_error_m"] for r in rows) / len(rows)
    draw_line_plot(
        out_dir / "01_q1_angle_recovery_sweep.png",
        "q1 sweep: repo formula tracks intended shoulder angle; paper Eq.15 does not",
        "true q1 used to generate foot target (deg)",
        "recovered q1 (deg)",
        x,
        [
            ("ideal y=x", x, GRAY),
            ("repo corrected q1", [r["repo_q1_deg"] for r in rows], TEAL),
            ("paper Eq.15 q1", [r["paper_q1_deg"] for r in rows], RED),
        ],
        x_bounds=x_bounds,
        y_bounds=(-250.0, 50.0),
        x_ticks=x_ticks,
        y_ticks=[-250.0, -200.0, -150.0, -100.0, -50.0, 0.0, 50.0],
        callouts=[
            (7.0, 7.0, "repo sits exactly on ideal line", TEAL, 70, -55),
            (7.0, -200.0, "paper Eq.15 lands on wrong q1 branch", RED, 70, 18),
        ],
    )

    draw_line_plot(
        out_dir / "02_leg_frame_foot_error_sweep.png",
        "Leg-frame FK error after swapping only q1 formula",
        "true q1 used to generate foot target (deg)",
        "foot-position error (m)",
        x,
        [
            ("repo corrected q1", [r["repo_foot_error_m"] for r in rows], TEAL),
            ("paper Eq.15 q1", [r["paper_foot_error_m"] for r in rows], RED),
        ],
        x_bounds=x_bounds,
        y_bounds=(-0.02, max(r["paper_foot_error_m"] for r in rows) * 1.15),
        x_ticks=x_ticks,
        y_ticks=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        reference_lines=[(0.0, GRAY, "zero error")],
        callouts=[
            (2.0, 0.0, "repo returns the target foot point", TEAL, 70, -40),
            (2.0, paper_error, f"paper stays ~{paper_error:.3f} m away", RED, 70, -48),
        ],
    )

    draw_xy_plot(
        out_dir / "03_leg_frame_target_vs_reconstructed_path.png",
        "Leg-frame foot path from q1 sweep",
        "x4 in leg frame (m)",
        "y4 in leg frame (m)",
        [
            ("target path", [(r["target_x_m"], r["target_y_m"]) for r in rows], BLACK),
            ("repo reconstructed", [(r["repo_foot_x_m"], r["repo_foot_y_m"]) for r in rows], TEAL),
            ("paper reconstructed", [(r["paper_foot_x_m"], r["paper_foot_y_m"]) for r in rows], RED),
        ],
        callouts=[
            (-0.04, -0.235, "repo path overlaps target", TEAL, 70, -55),
            (0.15, 0.19, "paper path is mirrored/far away", RED, 65, 28),
        ],
    )

    if "paper_mujoco_toe_error_m" in rows[0]:
        draw_line_plot(
            out_dir / "04_mujoco_front_right_toe_error_sweep.png",
            "MuJoCo front-right toe error after swapping only q1 formula",
            "true q1 used to generate foot target (deg)",
            "front-right toe error (m)",
            x,
            [
                ("repo corrected q1", [r["repo_mujoco_toe_error_m"] for r in rows], TEAL),
                ("paper Eq.15 q1", [r["paper_mujoco_toe_error_m"] for r in rows], RED),
            ],
            x_bounds=x_bounds,
            y_bounds=(-0.02, max(r["paper_mujoco_toe_error_m"] for r in rows) * 1.15),
            x_ticks=x_ticks,
            y_ticks=[0.0, 0.1, 0.2, 0.3, 0.4],
            reference_lines=[(0.0, GRAY, "zero error")],
            callouts=[
                (2.0, 0.0, "repo toe matches MuJoCo baseline", TEAL, 70, -40),
                (
                    2.0,
                    sum(r["paper_mujoco_toe_error_m"] for r in rows) / len(rows),
                    "paper toe is far from target",
                    RED,
                    70,
                    -48,
                ),
            ],
        )

    # Contact sheet. Use generous tile sizes so axis labels and legends remain
    # readable in preview and do not overlap after thumbnailing.
    image_paths = [
        out_dir / "01_q1_angle_recovery_sweep.png",
        out_dir / "02_leg_frame_foot_error_sweep.png",
        out_dir / "03_leg_frame_target_vs_reconstructed_path.png",
    ]
    if (out_dir / "04_mujoco_front_right_toe_error_sweep.png").exists():
        image_paths.append(out_dir / "04_mujoco_front_right_toe_error_sweep.png")

    thumbs = []
    for image_path in image_paths:
        im = Image.open(image_path).convert("RGB")
        im.thumbnail((820, 520))
        tile = Image.new("RGB", (880, 610), "white")
        d = ImageDraw.Draw(tile)
        tile.paste(im, ((880 - im.width) // 2, 10))
        d.text((18, 555), image_path.name, fill=BLACK, font=load_font(20))
        thumbs.append(tile)

    cols = 2
    rows_n = math.ceil(len(thumbs) / cols)
    sheet = Image.new("RGB", (cols * 880, rows_n * 610), (238, 238, 238))
    for i, tile in enumerate(thumbs):
        sheet.paste(tile, ((i % cols) * 880, (i // cols) * 610))
    sheet.save(out_dir / "q1_range_sweep_contact_sheet.png")


def run_viewer(rows: List[Dict], cycle_seconds: float) -> None:
    mujoco = maybe_load_mujoco()
    if mujoco is None:
        raise RuntimeError("mujoco Python package not importable")
    if not MUJOCO_MODEL.exists():
        raise RuntimeError(f"MuJoCo model not found: {MUJOCO_MODEL}")

    import mujoco.viewer  # type: ignore

    model = mujoco.MjModel.from_xml_path(str(MUJOCO_MODEL))
    data = mujoco.MjData(model)

    idx = 0
    mode = "repo"
    last_mode_switch = time.monotonic()
    print("Viewer mode cycles between repo q1 sweep and paper Eq.15 q1 sweep.")
    print("Close viewer or Ctrl+C to stop.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            now = time.monotonic()
            if now - last_mode_switch > cycle_seconds:
                mode = "paper" if mode == "repo" else "repo"
                idx = 0
                last_mode_switch = now
                print(f"viewer mode: {mode}")

            row = rows[idx % len(rows)]
            if mode == "repo":
                q = (
                    math.radians(row["repo_q1_deg"]),
                    math.radians(row["repo_q2_deg"]),
                    math.radians(row["repo_q3_deg"]),
                )
            else:
                q = (
                    math.radians(row["paper_q1_deg"]),
                    math.radians(row["paper_q2_deg"]),
                    math.radians(row["paper_q3_deg"]),
                )
            set_mujoco_front_right_pose(mujoco, model, data, q)
            viewer.sync()
            idx += 1
            time.sleep(0.035)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep q1 and compare repo q1 vs paper Eq.15 q1.")
    parser.add_argument("--q1-min-deg", type=float, default=-30.0)
    parser.add_argument("--q1-max-deg", type=float, default=30.0)
    parser.add_argument("--samples", type=int, default=121)
    parser.add_argument("--q2-deg", type=float, default=15.0)
    parser.add_argument("--q3-deg", type=float, default=-25.0)
    parser.add_argument("--links-m", nargs=3, type=float, default=(0.055, 0.1075, 0.130))
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "outputs")
    parser.add_argument("--skip-mujoco", action="store_true", help="Skip MuJoCo toe-error checks.")
    parser.add_argument("--viewer", action="store_true", help="Open interactive MuJoCo q1 sweep viewer.")
    parser.add_argument("--cycle-seconds", type=float, default=4.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows, meta = compute_rows(
        q1_min_deg=args.q1_min_deg,
        q1_max_deg=args.q1_max_deg,
        samples=args.samples,
        q2_deg=args.q2_deg,
        q3_deg=args.q3_deg,
        links=tuple(args.links_m),
        include_mujoco=not args.skip_mujoco or args.viewer,
    )
    write_outputs(rows, meta, args.out_dir)

    print("=== q1 range-of-motion comparison ===")
    print(f"Output folder: {args.out_dir}")
    print(f"MuJoCo: {meta['mujoco_note']}")
    print(f"q1 sweep: {args.q1_min_deg}° to {args.q1_max_deg}°")
    print(f"fixed q2/q3: {args.q2_deg}°, {args.q3_deg}°")
    print(f"repo max analytic foot error: {max(r['repo_foot_error_m'] for r in rows):.9f} m")
    print(f"paper mean analytic foot error: {sum(r['paper_foot_error_m'] for r in rows)/len(rows):.9f} m")
    print(f"paper min/max analytic foot error: {min(r['paper_foot_error_m'] for r in rows):.9f} / {max(r['paper_foot_error_m'] for r in rows):.9f} m")
    if "paper_mujoco_toe_error_m" in rows[0]:
        print(f"repo max MuJoCo toe error: {max(r['repo_mujoco_toe_error_m'] for r in rows):.9f} m")
        print(f"paper mean MuJoCo toe error: {sum(r['paper_mujoco_toe_error_m'] for r in rows)/len(rows):.9f} m")
        print(f"paper min/max MuJoCo toe error: {min(r['paper_mujoco_toe_error_m'] for r in rows):.9f} / {max(r['paper_mujoco_toe_error_m'] for r in rows):.9f} m")
        print(f"paper q1 inside MuJoCo shoulder range count: {sum(1 for r in rows if r.get('paper_q1_inside_mujoco_range') is True)} / {len(rows)}")
    print("Generated:")
    for p in sorted(args.out_dir.glob("*.png")):
        print(f"  {p}")
    print(f"  {args.out_dir / 'q1_range_sweep_results.csv'}")
    print(f"  {args.out_dir / 'q1_range_sweep_summary.json'}")

    if args.viewer:
        run_viewer(rows, args.cycle_seconds)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
