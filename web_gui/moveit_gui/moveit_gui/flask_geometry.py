"""Pure numpy geometry for a round-bottom flask touched with a ball tip.

Points and dimensions are in metres.  Touch points are ball-tip centres in
``base_link``.  An outside touch therefore measures a radius enlarged by the
tip radius.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np


class FlaskGeometryError(ValueError):
    pass


@dataclass(frozen=True)
class StandardFlaskSize:
    volume_ml: int
    bulb_od_m: float
    neck_inner_diameter_m: float


# Typical catalogue dimensions, not a dimensional standard.
COMMON_FLASK_SIZES = (
    StandardFlaskSize(50, 0.051, 0.018),
    StandardFlaskSize(100, 0.065, 0.020),
    StandardFlaskSize(250, 0.085, 0.024),
    StandardFlaskSize(500, 0.105, 0.029),
    StandardFlaskSize(1000, 0.131, 0.034),
)

_MAX_COORD_M = 5.0
_MIN_RADIUS_M = 0.005
_MAX_RADIUS_M = 0.5


def _points(values: Sequence[Sequence[float]], name: str, minimum: int) -> np.ndarray:
    try:
        out = np.asarray(values, dtype=float)
    except (TypeError, ValueError):
        raise FlaskGeometryError(f"{name} must be numeric [x, y, z] points") from None
    if out.ndim != 2 or out.shape[1] != 3:
        raise FlaskGeometryError(f"{name} must have shape (N, 3), got {out.shape}")
    if len(out) < minimum:
        raise FlaskGeometryError(f"need at least {minimum} {name}")
    if not np.all(np.isfinite(out)):
        raise FlaskGeometryError(f"{name} must be finite")
    if float(np.max(np.abs(out))) > _MAX_COORD_M:
        raise FlaskGeometryError(f"{name} coordinates are implausible; use metres, not millimetres")
    return out


def _positive(value: float, name: str, allow_zero: bool = False) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise FlaskGeometryError(f"{name} must be a number in metres") from None
    if not math.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
        word = "non-negative" if allow_zero else "positive"
        raise FlaskGeometryError(f"{name} must be {word}")
    return value


def fit_circle_3d(points: Sequence[Sequence[float]]) -> tuple[np.ndarray, float, np.ndarray, float]:
    """Least-squares 3D circle as ``(centre, radius, normal, radial_rms)``."""
    p = _points(points, "rim points", 3)
    mean = p.mean(axis=0)
    _, singular, vh = np.linalg.svd(p - mean, full_matrices=False)
    scale = max(float(singular[0]), 1e-15)
    if len(singular) < 2 or float(singular[1]) / scale < 1e-4:
        raise FlaskGeometryError("rim points are collinear and do not define a circle")
    normal = vh[-1]
    e1 = vh[0]
    e2 = np.cross(normal, e1)
    e2 /= np.linalg.norm(e2)
    xy = np.column_stack(((p - mean) @ e1, (p - mean) @ e2))
    A = np.column_stack((2.0 * xy, np.ones(len(xy))))
    b = np.sum(xy * xy, axis=1)
    if np.linalg.matrix_rank(A) < 3:
        raise FlaskGeometryError("rim points are collinear and do not define a circle")
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c2 = sol[:2]
    r2 = float(sol[2] + c2 @ c2)
    if r2 <= 0.0:
        raise FlaskGeometryError("rim fit produced a nonsense radius")
    radius = math.sqrt(r2)
    # A few geometric least-squares iterations remove the algebraic-fit bias.
    for _ in range(8):
        delta = xy - c2
        dist = np.linalg.norm(delta, axis=1)
        if np.any(dist < 1e-12):
            break
        residual = dist - radius
        J = np.column_stack((-delta / dist[:, None], -np.ones(len(xy))))
        change, *_ = np.linalg.lstsq(J, -residual, rcond=None)
        c2 += change[:2]
        radius += float(change[2])
        if np.linalg.norm(change) < 1e-12:
            break
    centre = mean + e1 * c2[0] + e2 * c2[1]
    radial = np.linalg.norm(xy - c2, axis=1) - radius
    plane = (p - centre) @ normal
    rms = math.sqrt(float(np.mean(radial * radial + plane * plane)))
    return centre, float(radius), normal, rms


def fit_sphere(points: Sequence[Sequence[float]]) -> tuple[np.ndarray, float, float]:
    """Geometric least-squares sphere as ``(centre, radius, radial_rms)``."""
    p = _points(points, "bulb points", 4)
    A = np.column_stack((2.0 * p, np.ones(len(p))))
    if np.linalg.matrix_rank(A) < 4:
        raise FlaskGeometryError("bulb points are coplanar and do not define a sphere")
    b = np.sum(p * p, axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    centre = sol[:3]
    r2 = float(sol[3] + centre @ centre)
    if r2 <= 0.0:
        raise FlaskGeometryError("sphere fit produced a nonsense radius")
    radius = math.sqrt(r2)
    for _ in range(10):
        delta = p - centre
        dist = np.linalg.norm(delta, axis=1)
        if np.any(dist < 1e-12):
            break
        residual = dist - radius
        J = np.column_stack((-delta / dist[:, None], -np.ones(len(p))))
        change, *_ = np.linalg.lstsq(J, -residual, rcond=None)
        centre += change[:3]
        radius += float(change[3])
        if np.linalg.norm(change) < 1e-12:
            break
    residual = np.linalg.norm(p - centre, axis=1) - radius
    return centre, float(radius), math.sqrt(float(np.mean(residual * residual)))


def closest_standard_flask(outer_diameter_m: float,
                           neck_inner_diameter_m: Optional[float] = None) -> StandardFlaskSize:
    """Return the closest typical size, using relative bulb and neck errors."""
    od = _positive(outer_diameter_m, "outer diameter")
    neck = None if neck_inner_diameter_m is None else _positive(neck_inner_diameter_m, "neck inner diameter")

    def score(size: StandardFlaskSize) -> float:
        terms = [(od - size.bulb_od_m) / size.bulb_od_m]
        if neck is not None:
            terms.append((neck - size.neck_inner_diameter_m) / size.neck_inner_diameter_m)
        return math.sqrt(sum(v * v for v in terms) / len(terms))

    return min(COMMON_FLASK_SIZES, key=score)


def closest_standard_size(outer_diameter_m: float,
                          neck_inner_diameter_m: Optional[float] = None) -> str:
    """Name the closest typical flask size, for example ``"250 ml"``."""
    return f"{closest_standard_flask(outer_diameter_m, neck_inner_diameter_m).volume_ml} ml"


@dataclass
class FlaskGeometry:
    sphere_center: np.ndarray
    R_outer: float
    R_inner: float
    neck_axis: np.ndarray
    rim_center: np.ndarray
    neck_inner_radius: float
    neck_length: float
    rim_rms: float
    sphere_rms: float
    warnings: list[str] = field(default_factory=list)
    neck_outer_radius: float = 0.0
    neck_rms: float = 0.0
    wall_thickness: float = 0.0015
    tip_radius: float = 0.0
    base_link: str = "base_link"

    @property
    def outer_radius(self) -> float:
        return self.R_outer

    @property
    def inner_radius(self) -> float:
        return self.R_inner

    @property
    def standard_size(self) -> str:
        return closest_standard_size(2.0 * self.R_outer, 2.0 * self.neck_inner_radius)

    @property
    def axis_offset(self) -> float:
        d = self.sphere_center - self.rim_center
        return float(np.linalg.norm(d - self.neck_axis * (d @ self.neck_axis)))

    @property
    def rim_height(self) -> float:
        return float((self.rim_center - self.sphere_center) @ self.neck_axis)

    @property
    def neck_join_height(self) -> float:
        radial = min(self.neck_outer_radius, self.R_outer * (1.0 - 1e-12))
        return math.sqrt(max(0.0, self.R_outer * self.R_outer - radial * radial))

    def T_base_flask(self) -> np.ndarray:
        return T_base_flask(self)

    def to_dict(self) -> dict:
        return {
            "sphere_center": [float(v) for v in self.sphere_center],
            "R_outer": float(self.R_outer),
            "R_inner": float(self.R_inner),
            "neck_axis": [float(v) for v in self.neck_axis],
            "rim_center": [float(v) for v in self.rim_center],
            "neck_inner_radius": float(self.neck_inner_radius),
            "neck_length": float(self.neck_length),
            "rim_rms": float(self.rim_rms),
            "sphere_rms": float(self.sphere_rms),
            "neck_outer_radius": float(self.neck_outer_radius),
            "neck_rms": float(self.neck_rms),
            "wall_thickness": float(self.wall_thickness),
            "tip_radius": float(self.tip_radius),
            "base_link": self.base_link,
            "warnings": list(self.warnings),
        }

    @classmethod
    def from_dict(cls, value: dict) -> "FlaskGeometry":
        if not isinstance(value, dict):
            raise FlaskGeometryError("saved flask geometry must be a mapping")
        try:
            out = cls(
                sphere_center=np.asarray(value["sphere_center"], dtype=float),
                R_outer=float(value["R_outer"]),
                R_inner=float(value["R_inner"]),
                neck_axis=np.asarray(value["neck_axis"], dtype=float),
                rim_center=np.asarray(value["rim_center"], dtype=float),
                neck_inner_radius=float(value["neck_inner_radius"]),
                neck_length=float(value["neck_length"]),
                rim_rms=float(value["rim_rms"]),
                sphere_rms=float(value["sphere_rms"]),
                warnings=[str(v) for v in value.get("warnings", [])],
                neck_outer_radius=float(value.get("neck_outer_radius", 0.0)),
                neck_rms=float(value.get("neck_rms", 0.0)),
                wall_thickness=float(value.get("wall_thickness", 0.0015)),
                tip_radius=float(value.get("tip_radius", 0.0)),
                base_link=str(value.get("base_link", "base_link")),
            )
        except (KeyError, TypeError, ValueError):
            raise FlaskGeometryError("saved flask geometry has missing or invalid values") from None
        _check_result(out)
        return out


def _check_result(g: FlaskGeometry) -> None:
    if g.sphere_center.shape != (3,) or g.rim_center.shape != (3,) or g.neck_axis.shape != (3,):
        raise FlaskGeometryError("flask centres and neck axis must have three values")
    values = np.r_[g.sphere_center, g.rim_center, g.neck_axis,
                   g.R_outer, g.R_inner, g.neck_inner_radius, g.neck_length]
    if not np.all(np.isfinite(values)):
        raise FlaskGeometryError("flask geometry must be finite")
    n = float(np.linalg.norm(g.neck_axis))
    if abs(n - 1.0) > 1e-5:
        raise FlaskGeometryError("neck axis must be a unit vector")
    if not (_MIN_RADIUS_M <= g.R_outer <= _MAX_RADIUS_M):
        raise FlaskGeometryError("fitted bulb size is nonsense")
    if not 0.0 < g.R_inner < g.R_outer:
        raise FlaskGeometryError("wall thickness leaves no inner bulb")
    if not 0.0 < g.neck_inner_radius < g.R_outer:
        raise FlaskGeometryError("neck size is nonsense for the bulb")
    if g.neck_outer_radius <= 0.0 or g.neck_outer_radius >= g.R_outer:
        raise FlaskGeometryError("outer neck size is nonsense for the bulb")
    if g.neck_inner_radius >= g.neck_outer_radius + 1e-9:
        raise FlaskGeometryError("inner neck must be smaller than the outer neck")
    if g.neck_length <= 0.0 or g.neck_length > 1.0:
        raise FlaskGeometryError("neck length is nonsense")


def fit_flask(rim_points: Sequence[Sequence[float]], bulb_points: Sequence[Sequence[float]],
              neck_points: Optional[Sequence[Sequence[float]]] = None, *, r_tip: float = 0.0,
              wall_thickness: float = 0.0015, neck_inner_diameter: Optional[float] = None,
              base_link: str = "base_link") -> FlaskGeometry:
    """Fit a spherical bulb and cylindrical neck from outside touch points."""
    tip = _positive(r_tip, "tip radius", allow_zero=True)
    wall = _positive(wall_thickness, "wall thickness")
    if tip > 0.05 or wall > 0.02:
        raise FlaskGeometryError("tip radius or wall thickness is implausibly large")
    rim_center, rim_measured, axis, rim_rms = fit_circle_3d(rim_points)
    sphere_center, sphere_measured, sphere_rms = fit_sphere(bulb_points)
    if axis[2] < 0.0:
        axis = -axis
    rim_outer = rim_measured - tip
    R_outer = sphere_measured - tip
    R_inner = R_outer - wall
    if neck_points is not None and len(neck_points):
        side = _points(neck_points, "neck side points", 1)
        delta = side - rim_center
        radial = np.linalg.norm(delta - np.outer(delta @ axis, axis), axis=1) - tip
        neck_outer = float(np.mean(radial))
        neck_rms = math.sqrt(float(np.mean((radial - neck_outer) ** 2)))
    else:
        neck_outer, neck_rms = rim_outer, 0.0
    if neck_inner_diameter is None:
        neck_inner = neck_outer - wall
    else:
        neck_inner = _positive(neck_inner_diameter, "neck inner diameter") / 2.0
    if R_outer <= 0.0 or rim_outer <= 0.0 or neck_outer <= 0.0:
        raise FlaskGeometryError("tip compensation leaves a nonsense flask size")
    if neck_outer >= R_outer:
        raise FlaskGeometryError("neck is not smaller than the bulb")
    join_height = math.sqrt(max(0.0, R_outer * R_outer - neck_outer * neck_outer))
    rim_height = float((rim_center - sphere_center) @ axis)
    neck_length = rim_height - join_height
    warnings: list[str] = []
    offset_vec = sphere_center - rim_center
    offset = float(np.linalg.norm(offset_vec - axis * (offset_vec @ axis)))
    tilt = math.degrees(math.acos(float(np.clip(axis[2], -1.0, 1.0))))
    if offset > 0.002:
        warnings.append("The sphere centre is more than 2 mm off the neck axis.")
    if tilt > 3.0:
        warnings.append("The neck axis is tilted more than 3 degrees from vertical.")
    if max(rim_rms, sphere_rms, neck_rms) > 0.001:
        warnings.append("A fitted surface has more than 1 mm RMS residual.")
    typical = closest_standard_flask(2.0 * R_outer, 2.0 * neck_inner)
    bulb_error = abs(2.0 * R_outer - typical.bulb_od_m) / typical.bulb_od_m
    neck_error = abs(2.0 * neck_inner - typical.neck_inner_diameter_m) / typical.neck_inner_diameter_m
    if bulb_error > 0.25 or neck_error > 0.35:
        warnings.append("The fitted sizes are far from a common round-bottom flask size.")
    out = FlaskGeometry(
        sphere_center=sphere_center,
        R_outer=R_outer,
        R_inner=R_inner,
        neck_axis=axis,
        rim_center=rim_center,
        neck_inner_radius=neck_inner,
        neck_length=neck_length,
        rim_rms=rim_rms,
        sphere_rms=sphere_rms,
        warnings=warnings,
        neck_outer_radius=neck_outer,
        neck_rms=neck_rms,
        wall_thickness=wall,
        tip_radius=tip,
        base_link=str(base_link),
    )
    _check_result(out)
    return out


estimate_flask = fit_flask


def T_base_flask(geometry: FlaskGeometry) -> np.ndarray:
    """Fixture ``flask``: sphere centre, Z up the neck, X close to base X."""
    z = np.asarray(geometry.neck_axis, dtype=float)
    z = z / np.linalg.norm(z)
    base_x = np.array([1.0, 0.0, 0.0])
    x = base_x - z * float(base_x @ z)
    if np.linalg.norm(x) < 1e-6:
        base_y = np.array([0.0, 1.0, 0.0])
        x = base_y - z * float(base_y @ z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    T = np.eye(4)
    T[:3, :3] = np.column_stack((x, y, z))
    T[:3, 3] = geometry.sphere_center
    return T
