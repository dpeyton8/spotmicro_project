"""Run the two-track comparison and write CSV data plus presentation-ready plots."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import numpy as np

from .sim import (
    COMETYANG_SIT_MOUNT,
    COMETYANG_STAND_MOUNT,
    COMETYANG_TRANSIT_DT,
    HAO_TRANSITION_SECONDS,
    rate_limited_transition_step,
    run,
    summarize,
)
from .trajectory import Gait
from .tracks import hao_mount, walk_gait


def sitstand_samples(duration: float = HAO_TRANSITION_SECONDS):
    times = np.arange(0.0, duration + COMETYANG_TRANSIT_DT / 2, COMETYANG_TRANSIT_DT)
    repo = COMETYANG_STAND_MOUNT.copy()
    rows = []
    for elapsed in times:
        if elapsed > 0.0:
            repo = rate_limited_transition_step(repo, COMETYANG_SIT_MOUNT)
        hao = hao_mount(COMETYANG_STAND_MOUNT, COMETYANG_SIT_MOUNT, elapsed, duration)
        rows.append(dict(time_s=elapsed, repo_x_m=repo[0], repo_z_m=repo[2],
                         hao_x_m=hao[0], hao_z_m=hao[2]))
    return rows


def write_rows(path: Path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/two_track_comparison"))
    parser.add_argument("--cycles", type=float, default=2.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    article = Gait(stance_press=0.004)
    all_walk_rows = []
    results = {}
    for method in ("repo", "zhang"):
        gait = walk_gait(method, article)
        rows, _, settle = run(gait, 0.06, 2.0, 0.5, args.cycles, False)
        for row in rows:
            row["comparison_method"] = method
        all_walk_rows.extend(rows)
        results[method] = summarize(rows, settle)
    write_rows(args.output_dir / "walk_comparison.csv", all_walk_rows)

    sit_rows = sitstand_samples()
    write_rows(args.output_dir / "sitstand_comparison.csv", sit_rows)

    os.environ.setdefault("MPLCONFIGDIR", "/tmp/spotmicro-matplotlib")
    import matplotlib.pyplot as plt

    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot([r["time_s"] for r in sit_rows], [r["repo_z_m"] for r in sit_rows], label="Cometyang repo filter")
    axis.plot([r["time_s"] for r in sit_rows], [r["hao_z_m"] for r in sit_rows], label="Hao Eq. 9 cycloid")
    axis.set(xlabel="Time (s)", ylabel="Mount height z (m)", title="Sit transition: same endpoints, different mathematics")
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "sitstand_mount_height.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for method in ("repo", "zhang"):
        rows = [r for r in all_walk_rows if r["comparison_method"] == method and r["phase"] != "lowering"]
        t0 = rows[0]["time_s"]
        axes[0].plot([r["time_s"] - t0 for r in rows], [1000 * r["target_x_m"] for r in rows], label=method)
        axes[1].plot([r["time_s"] - t0 for r in rows], [1000 * r["toe_height_m"] for r in rows], label=method)
    axes[0].set(ylabel="Desired fore/aft x (mm)", title="Walk comparison on the same left-leg MuJoCo model")
    axes[1].set(xlabel="Time after lowering (s)", ylabel="Simulated toe clearance (mm)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "walk_trajectory_comparison.png", dpi=180)
    plt.close(fig)

    with (args.output_dir / "summary.txt").open("w") as stream:
        for method, stats in results.items():
            stream.write(f"{method}\n")
            for key, value in stats.items():
                stream.write(f"  {key}: {value:.3f}\n")
    print(args.output_dir)
    for method, stats in results.items():
        print(method, {key: round(value, 3) for key, value in stats.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
