"""Foot-end trajectories from Zhang et al. 2020: Bezier swing (Table 1) and sinusoidal stance (Eq. 19)."""

from __future__ import annotations

from dataclasses import dataclass
from math import comb, cos, pi

import numpy as np

from .leg import L1

# Table 1 control points in mm: x forward, y down from the hip, ground at y = 460.
SWING_POINTS_MM = np.array(
    [
        [-170.0, 460.0], [-280.5, 460.0], [-300.0, 361.1], [-300.0, 361.1],
        [-300.0, 361.1], [0.0, 361.1], [0.0, 361.1], [0.0, 321.4],
        [303.2, 321.4], [303.2, 321.4], [282.6, 460.0], [170.0, 460.0],
    ]
)
GROUND_Y_MM = 460.0
HALF_STROKE_MM = 170.0


def bezier(points: np.ndarray, s: float) -> np.ndarray:
    n = len(points) - 1
    weights = np.array([comb(n, k) * (1 - s) ** (n - k) * s**k for k in range(n + 1)])
    return weights @ points


@dataclass(frozen=True)
class Gait:
    hip_height: float = 0.17  # m, hip to toe center; 12.8 deg minimum joint-limit margin at scale 0.25
    scale: float = 0.25  # fraction of the paper's leg size
    fore_offset: float = 0.0  # m, shifts the whole path fore/aft
    lateral: float = L1  # m, left of the shoulder axis; L1 puts the foot under the pitch joints
    stance_seconds: float = 1.5
    swing_seconds: float = 1.0
    stance_press: float = 0.0  # m, extra downward travel at mid-stance

    @property
    def cycle_seconds(self) -> float:
        return self.stance_seconds + self.swing_seconds

    def foot_position(self, t: float) -> tuple[np.ndarray, str]:
        """Hip-relative toe target in the MuJoCo frame (x fore, y left, z up) and the gait phase."""
        phase_t = t % self.cycle_seconds
        if phase_t < self.stance_seconds:
            x_mm = HALF_STROKE_MM * (1 - 2 * phase_t / self.stance_seconds)
            depth = self.hip_height + self.stance_press * cos(pi * x_mm / (2 * HALF_STROKE_MM))
            phase = "stance"
        else:
            s = (phase_t - self.stance_seconds) / self.swing_seconds
            x_mm, y_mm = bezier(SWING_POINTS_MM, s)
            depth = self.hip_height + (y_mm - GROUND_Y_MM) * 1e-3 * self.scale
            phase = "swing"
        fore = self.fore_offset + x_mm * 1e-3 * self.scale
        return np.array([fore, self.lateral, -depth]), phase
