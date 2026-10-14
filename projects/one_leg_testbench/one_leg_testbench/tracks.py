"""Comparable repository and article trajectories for the left-leg testbench."""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, sin

import numpy as np

from .leg import L1
from .trajectory import Gait


def hao_progress(elapsed: float, duration: float) -> float:
    """Hao et al. Eq. 9 cycloidal progress, including endpoint clamps."""
    if duration <= 0.0:
        return 1.0
    u = float(np.clip(elapsed / duration, 0.0, 1.0))
    phi = 2.0 * pi * u
    return (phi - sin(phi)) / (2.0 * pi)


def hao_mount(start, target, elapsed: float, duration: float) -> np.ndarray:
    """Interpolate a mount/body pose with Hao Eq. 9."""
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    return start + (target - start) * hao_progress(elapsed, duration)


@dataclass(frozen=True)
class RepoGait:
    """Cometyang/Mike LF trajectory shape, normalized for a fair one-leg test.

    The repository controller advances a stance foot opposite commanded body
    velocity and uses a triangular clearance profile during the LF swing
    phase.  The full repository scheduler has eight phases (seven LF stance,
    one LF swing); this class isolates that LF path on the suspended rig.
    """

    hip_height: float = Gait.hip_height
    half_stroke: float = Gait.scale * 0.170
    lateral: float = L1
    stance_seconds: float = Gait.stance_seconds
    swing_seconds: float = Gait.swing_seconds
    clearance: float = 0.050

    @property
    def cycle_seconds(self) -> float:
        return self.stance_seconds + self.swing_seconds

    def foot_position(self, t: float) -> tuple[np.ndarray, str]:
        phase_t = t % self.cycle_seconds
        if phase_t < self.stance_seconds:
            s = phase_t / self.stance_seconds
            fore = self.half_stroke * (1.0 - 2.0 * s)
            lift = 0.0
            phase = "stance"
        else:
            s = (phase_t - self.stance_seconds) / self.swing_seconds
            fore = -self.half_stroke + 2.0 * self.half_stroke * s
            lift = self.clearance * (2.0 * s if s < 0.5 else 2.0 * (1.0 - s))
            phase = "swing"
        return np.array([fore, self.lateral, -self.hip_height + lift]), phase


def walk_gait(method: str, article_gait: Gait) -> Gait | RepoGait:
    """Return either repo triangular/linear gait or Zhang Eq. 18-19 gait."""
    if method == "repo":
        return RepoGait(
            hip_height=article_gait.hip_height,
            half_stroke=article_gait.scale * 0.170,
            lateral=article_gait.lateral,
            stance_seconds=article_gait.stance_seconds,
            swing_seconds=article_gait.swing_seconds,
        )
    if method == "zhang":
        return article_gait
    raise ValueError(f"walk method must be repo or zhang, got {method!r}")
