"""Pure vial geometry: the calculation of sample_scraping/scripts/vial_estimation.py without its node.

The legacy tool records three rim points, fits a circle in XY and takes the lowest sampled Z as the
bottom estimate. That model is kept and its limits are explicit:

  * XY circle in a base-aligned frame. The vial axis is assumed parallel to base Z. There is no
    arbitrary 3D circle fit.
  * The legacy bottom is min(rim Z), which is only a bottom if a sampled point touched the bottom.
    A separately touched bottom point is preferred when one is given.

All values are metres, in one frame, supplied by the caller (the GUI's frame/TCP system).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

MAX_PLAUSIBLE_COORD_M = 5.0        # bigger than any UR reach: these are millimetres, not metres
MAX_PLAUSIBLE_RADIUS_M = 0.5
MIN_POINT_SEPARATION_M = 0.001
MIN_TRIANGLE_HEIGHT_M = 0.0005     # rim points closer to a line than this do not define a circle
MAX_RIM_Z_SPREAD_M = 0.005         # more than this contradicts the base-aligned model


class VialGeometryError(ValueError):
    pass


@dataclass
class VialGeometry:
    center_xy: np.ndarray
    radius: float
    bottom_z: float
    bottom_source: str              # "touched" | "min_rim_z"
    rim_z_spread: float
    model: str = "xy_circle_base_aligned"

    def to_meta(self) -> dict:
        return {"method": "vial_xy_circle", "model": self.model, "radius_m": float(self.radius),
                "bottom_source": self.bottom_source, "rim_z_spread_m": float(self.rim_z_spread)}


def _validated(points: Sequence[Sequence[float]], count: Optional[int] = None) -> np.ndarray:
    try:
        p = np.asarray(points, dtype=float)
    except (TypeError, ValueError):
        raise VialGeometryError("points must be numeric [x, y, z] in metres") from None
    if p.ndim != 2 or p.shape[1] != 3 or (count is not None and p.shape[0] != count):
        raise VialGeometryError(f"expected {count or 'N'} points of [x, y, z], got shape {p.shape}")
    if not np.all(np.isfinite(p)):
        raise VialGeometryError("points must be finite")
    if np.max(np.abs(p)) > MAX_PLAUSIBLE_COORD_M:
        raise VialGeometryError("coordinates are larger than the robot's reach: points must be in metres, not mm")
    return p


def circle_from_three_points(p1: Sequence[float], p2: Sequence[float], p3: Sequence[float]):
    """Centre and radius of the circle through three XY points. Rejects collinear points."""
    a, b, c = (np.asarray(p, dtype=float)[:2] for p in (p1, p2, p3))
    ab, ac, bc = b - a, c - a, c - b
    if min(np.linalg.norm(ab), np.linalg.norm(ac), np.linalg.norm(bc)) < MIN_POINT_SEPARATION_M:
        raise VialGeometryError("rim points are too close together")
    cross = ab[0] * ac[1] - ab[1] * ac[0]
    if abs(cross) / max(np.linalg.norm(ab), np.linalg.norm(ac), np.linalg.norm(bc)) < MIN_TRIANGLE_HEIGHT_M:
        raise VialGeometryError("rim points are collinear and do not define a circle")
    d = 2.0 * cross
    u = np.array([ac[1] * (ab @ ab) - ab[1] * (ac @ ac), ab[0] * (ac @ ac) - ac[0] * (ab @ ab)]) / d
    return a + u, float(np.linalg.norm(u))


def estimate_vial(rim_points: Sequence[Sequence[float]], bottom_point: Optional[Sequence[float]] = None) -> VialGeometry:
    """Three rim points [x, y, z] (m) and an optional touched bottom point. See the module limits."""
    rim = _validated(rim_points, 3)
    center, radius = circle_from_three_points(*rim)
    if radius > MAX_PLAUSIBLE_RADIUS_M:
        raise VialGeometryError(f"fitted radius {radius:.3f} m is implausible for a vial")
    spread = float(np.ptp(rim[:, 2]))
    if spread > MAX_RIM_Z_SPREAD_M:
        raise VialGeometryError(f"rim Z varies by {spread * 1000:.1f} mm: the vial is not aligned with base Z, "
                                "which is outside this XY-circle model")
    if bottom_point is None:
        return VialGeometry(center, radius, float(np.min(rim[:, 2])), "min_rim_z", spread)
    bottom = _validated([bottom_point], 1)[0]
    if bottom[2] > np.min(rim[:, 2]):
        raise VialGeometryError("the bottom point is above the rim")
    return VialGeometry(center, radius, float(bottom[2]), "touched", spread)
