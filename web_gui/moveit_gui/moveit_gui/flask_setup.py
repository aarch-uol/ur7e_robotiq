"""SETUP > Flask Geometry without the web: touch points, the next-touch hint, flask profiles, the tool factory and
the reach summary of a flask scrape. The maths is in flask_geometry.py (fit) and flask_scrape.py (poses, reach).

Touch points are TCP (ball tip) centres in the base frame, metres, as the vial's. A saved flask is an ordinary
fixture named "flask" (FLASK_FIXTURE: flask_scrape writes its poses in that frame) whose meta holds the fitted
geometry, so the scrape generator reads the same flask the fixture was made from.

Flask profiles (several named flasks, one of them put to work as the fixture) live in
config_dir()/flasks/<robot profile>.yaml, like the vial profiles.
"""
from __future__ import annotations

import math
import re
import time
from collections import Counter
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np
import yaml

from . import flask_scrape as fs
from .flask_geometry import FlaskGeometry, FlaskGeometryError, T_base_flask

FLASK_FIXTURE = "flask"
KINDS = ("rim", "bulb", "neck")
MIN_POINTS = {"rim": 3, "bulb": 4, "neck": 0}
MAX_POINTS = 40                      # per kind: more says nothing new
NAME = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_ .-]{0,39}$")


class FlaskSetupError(ValueError):
    pass


# ------------------------------------------------------------------ fit inputs
def fit_settings(tip_mm=None, wall_mm=None, neck_id_mm=None, old: Optional[dict] = None) -> dict:
    """The fit's numbers from the page (mm), checked: {tip_mm, wall_mm, neck_id_mm or None}. Missing = old/default."""
    out = dict(old or {"tip_mm": 0.0, "wall_mm": 1.5, "neck_id_mm": None})

    def num(v, what, lo, hi):
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise FlaskSetupError(f"{what} must be a number of mm") from None
        if not (math.isfinite(v) and lo <= v <= hi):
            raise FlaskSetupError(f"{what} must be {lo:g} to {hi:g} mm")
        return v
    if tip_mm not in (None, ""):
        out["tip_mm"] = num(tip_mm, "tip radius", 0.0, 50.0)
    if wall_mm not in (None, ""):
        out["wall_mm"] = num(wall_mm, "wall thickness", 0.1, 20.0)
    if neck_id_mm == "" or neck_id_mm == 0:
        out["neck_id_mm"] = None
    elif neck_id_mm is not None:
        out["neck_id_mm"] = num(neck_id_mm, "neck inner diameter", 1.0, 200.0)
    return out


def fit(points: Dict[str, List[List[float]]], settings: dict, base_link: str = "base_link") -> FlaskGeometry:
    from .flask_geometry import fit_flask
    nid = settings.get("neck_id_mm")
    return fit_flask(points.get("rim") or [], points.get("bulb") or [], points.get("neck") or None,
                     r_tip=float(settings.get("tip_mm") or 0.0) / 1000.0,
                     wall_thickness=float(settings.get("wall_mm") or 1.5) / 1000.0,
                     neck_inner_diameter=None if nid in (None, "") else float(nid) / 1000.0, base_link=base_link)


def geometry_view(g: FlaskGeometry) -> dict:
    """What the page shows of a fit (mm) plus what the 3D view draws (base frame, mm)."""
    T = T_base_flask(g)
    return {"od_mm": 2000.0 * g.R_outer, "r_inner_mm": 1000.0 * g.R_inner, "neck_id_mm": 2000.0 * g.neck_inner_radius,
            "neck_od_mm": 2000.0 * g.neck_outer_radius, "neck_length_mm": 1000.0 * g.neck_length,
            "size": g.standard_size, "rim_rms_mm": 1000.0 * g.rim_rms, "bulb_rms_mm": 1000.0 * g.sphere_rms,
            "neck_rms_mm": 1000.0 * g.neck_rms, "axis_offset_mm": 1000.0 * g.axis_offset,
            "tilt_deg": math.degrees(math.acos(float(np.clip(g.neck_axis[2], -1.0, 1.0)))),
            "warnings": list(g.warnings),
            "draw": draw_shapes(g, T)}


def draw_shapes(g: FlaskGeometry, T_base_flask_: np.ndarray, pivot_flask: Optional[np.ndarray] = None) -> dict:
    """Sphere (outer bulb) and neck cylinder in the base frame, mm, for the 3D view."""
    R = T_base_flask_[:3, :3]
    c = T_base_flask_[:3, 3]
    axis = R[:, 2]
    rim_flask = T_base_flask(g)[:3, :3].T @ (np.asarray(g.rim_center, float) - g.sphere_center)
    rim = c + R @ rim_flask                    # in the fixture as it is (it may have been moved since the fit)
    join = rim - axis * g.neck_length
    if pivot_flask is None:
        pivot_flask = fs.pivot_point(g)
    pivot = c + R @ np.asarray(pivot_flask, float)
    mm = lambda v: [round(1000.0 * float(x), 3) for x in v]          # noqa: E731
    return {"sphere": {"c": mm(c), "r": round(1000.0 * g.R_outer, 3)},
            "neck": {"a": mm(join), "b": mm(rim), "r": round(1000.0 * g.neck_outer_radius, 3)},
            "pivot": mm(pivot)}


# -------------------------------------------------------------- next touch
def _azimuth_gap(az: Sequence[float]) -> tuple[float, float]:
    """The middle of the largest gap between azimuths (deg) and the gap's size."""
    a = sorted(x % 360.0 for x in az)
    if not a:
        return 0.0, 360.0
    if len(a) == 1:
        return (a[0] + 180.0) % 360.0, 360.0
    gaps = [((a[(i + 1) % len(a)] - a[i]) % 360.0 or 360.0, a[i]) for i in range(len(a))]
    size, start = max(gaps)
    return (start + size / 2.0) % 360.0, size


def _toward(az_deg: float) -> str:
    """'+X', '-X +Y', … of the base frame for an azimuth."""
    names = ["+X", "+X +Y", "+Y", "-X +Y", "-X", "-X -Y", "-Y", "+X -Y"]
    return names[int(((az_deg % 360.0) + 22.5) // 45.0) % 8]


def next_touch(points: Dict[str, List[List[float]]], g: Optional[FlaskGeometry] = None) -> dict:
    """Where to touch next: {text, kind, xyz (base, m) or None}. Rim first (3+), then the bulb (4+, spread around
    and down to the bottom), then optional neck-side points. Azimuths are about the vertical through the flask."""
    rim, bulb = points.get("rim") or [], points.get("bulb") or []
    if rim:
        cxy = np.mean(np.asarray(rim, float)[:, :2], axis=0)
    elif bulb:
        cxy = np.mean(np.asarray(bulb, float)[:, :2], axis=0)
    else:
        return {"kind": "rim", "text": "Touch the outside of the rim: 3 or more points spread around it.", "xyz": None}
    if g is not None:
        cxy = np.asarray(g.sphere_center[:2], float)
    if len(rim) < 3:
        az = [math.degrees(math.atan2(p[1] - cxy[1], p[0] - cxy[0])) for p in rim]
        mid, _ = _azimuth_gap(az)
        return {"kind": "rim", "xyz": None,
                "text": f"Rim: {3 - len(rim)} more. Next on the {_toward(mid)} side ({mid:.0f}°)."}
    # bulb: azimuth gap and height band, about the fitted sphere when there is one
    if g is not None:
        c, r = np.asarray(g.sphere_center, float), g.R_outer + g.tip_radius
    else:
        c, r = None, None
    bands = [("bottom", 0.0, 35.0), ("lower side", 35.0, 75.0), ("equator", 75.0, 105.0)]
    count = {b[0]: 0 for b in bands}
    az = []
    for p in bulb:
        p = np.asarray(p, float)
        az.append(math.degrees(math.atan2(p[1] - cxy[1], p[0] - cxy[0])))
        if c is not None:
            d = (p - c) / max(np.linalg.norm(p - c), 1e-9)
            polar = math.degrees(math.acos(float(np.clip(-d[2], -1.0, 1.0))))      # 0 = the bottom
            for name, lo, hi in bands:
                if lo <= polar < hi:
                    count[name] += 1
    mid, gap = _azimuth_gap(az)
    if c is None:
        band = "lower side"
    else:
        band = min(bands, key=lambda b: (count[b[0]], b[0] != "lower side"))[0]
    polar = {"bottom": 15.0, "lower side": 55.0, "equator": 90.0}[band]
    xyz = None
    if c is not None:
        th, ph = math.radians(polar), math.radians(mid)
        xyz = [float(v) for v in c + r * np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph),
                                                   -math.cos(th)])]
    if len(bulb) < 4:
        lead = f"Bulb: {4 - len(bulb)} more (6–10 is better)."
    elif gap > 120.0 or (c is not None and min(count.values()) == 0):
        lead = "Bulb: spread it more."
    else:
        lead = "Enough to fit. More points spread out help."
    where = "the bottom" if band == "bottom" else f"the {band}, {_toward(mid)} side ({mid:.0f}°)"
    return {"kind": "bulb", "xyz": xyz, "text": f"{lead} Next: {where}."}


# ------------------------------------------------------------ synthetic flask
def synthetic_touches(center, od_mm: float = 85.0, neck_od_mm: float = 27.0, neck_length_mm: float = 50.0,
                      tip_mm: float = 2.0, rim=4, bulb=8, neck=3, noise_mm: float = 0.0, seed: int = 1) -> dict:
    """Touches (tip centres, base frame, m) on an upright flask whose sphere centre is `center` (m): what a perfect
    operator would capture. For tests and the fake robot's "Fake touches"."""
    rng = np.random.default_rng(seed)
    c = np.asarray(center, float)
    R, rn, tip = od_mm / 2000.0, neck_od_mm / 2000.0, tip_mm / 1000.0
    join = math.sqrt(R * R - rn * rn)
    rim_z = join + neck_length_mm / 1000.0
    jitter = lambda: rng.normal(0.0, noise_mm / 1000.0, 3) if noise_mm else np.zeros(3)   # noqa: E731
    out = {"rim": [], "bulb": [], "neck": []}
    for k in range(rim):
        a = 2 * math.pi * k / rim + 0.3
        out["rim"].append(c + [(rn + tip) * math.cos(a), (rn + tip) * math.sin(a), rim_z] + jitter())
    polars = [15.0, 55.0, 90.0, 40.0, 75.0, 25.0, 65.0, 100.0]
    for k in range(bulb):
        th, ph = math.radians(polars[k % len(polars)]), 2 * math.pi * k / bulb * 1.618 + 0.5
        d = np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), -math.cos(th)])
        out["bulb"].append(c + (R + tip) * d + jitter())
    for k in range(neck):
        a = 2 * math.pi * k / max(neck, 1) + 1.1
        z = join + (0.3 + 0.4 * k / max(neck - 1, 1)) * (rim_z - join)
        out["neck"].append(c + [(rn + tip) * math.cos(a), (rn + tip) * math.sin(a), z] + jitter())
    return {k: [[float(x) for x in p] for p in v] for k, v in out.items()}


# ---------------------------------------------------------------- the tool
def _straight(d: dict, tcp: str):
    return fs.StraightTool(tcp=tcp, shaft_radius=_mm(d, "shaft_radius_mm") / 1000.0,
                           shaft_length=_mm(d, "shaft_length_mm") / 1000.0)


def _bent(d: dict, tcp: str):
    try:
        angle = float(d["bend_angle_deg"])
    except KeyError:
        raise FlaskSetupError("the bent tool needs a bend angle (°)") from None
    except (TypeError, ValueError):
        raise FlaskSetupError("bend angle must be a number of degrees") from None
    tip = d.get("tip_radius_mm")
    return fs.BentTool(tcp=tcp, shaft_radius=_mm(d, "shaft_radius_mm") / 1000.0,
                       shaft_length=_mm(d, "shaft_length_mm") / 1000.0,
                       bend_length=_mm(d, "bend_length_mm") / 1000.0, bend_angle_deg=angle,
                       tip_radius=0.0 if tip in (None, "", 0) else _mm(d, "tip_radius_mm") / 1000.0)


# "kind" -> factory(tool dict, tcp name) -> a tool object for flask_scrape.generate_scrape_poses / build_program
TOOLS: Dict[str, Callable[[dict, str], object]] = {"straight": _straight, "bent": _bent}


def _mm(d: dict, key: str) -> float:
    try:
        v = float(d[key])
    except KeyError:
        raise FlaskSetupError(f"the tool needs {key.replace('_mm', '').replace('_', ' ')} (mm)") from None
    except (TypeError, ValueError):
        raise FlaskSetupError(f"{key.replace('_mm', '').replace('_', ' ')} must be a number of mm") from None
    if not (math.isfinite(v) and v > 0):
        raise FlaskSetupError(f"{key.replace('_mm', '').replace('_', ' ')} must be positive")
    return v


def make_tool(d: Optional[dict], tcp: str):
    """The one place a tool dict from the page becomes a tool object: {"kind": "straight", "shaft_radius_mm",
    "shaft_length_mm"} or {"kind": "bent", ... + "bend_length_mm", "bend_angle_deg" (, "tip_radius_mm")}.
    A new kind is a new TOOLS entry."""
    d = dict(d or {})
    kind = str(d.pop("kind", "straight") or "straight")
    f = TOOLS.get(kind)
    if f is None:
        raise FlaskSetupError(f"unknown tool '{kind}' (one of {', '.join(TOOLS)})")
    try:
        return f(d, tcp)
    except fs.FlaskScrapeError as e:
        raise FlaskSetupError(str(e)) from None


# ------------------------------------------------------- reach and program
def scrape_args(theta_min_deg=0.0, theta_max_deg=15.0, rings=3, step_deg=15.0, press_mm=0.5, margin_mm=1.0) -> dict:
    try:
        out = {"theta_min_deg": float(theta_min_deg), "theta_max_deg": float(theta_max_deg),
               "rings": int(rings), "step_deg": float(step_deg), "press_depth_mm": float(press_mm),
               "margin_m": float(margin_mm) / 1000.0}
    except (TypeError, ValueError):
        raise FlaskSetupError("the scrape needs numbers: angles, rings, step, press depth") from None
    if out["rings"] > 50 or out["step_deg"] < 1.0:
        raise FlaskSetupError("at most 50 rings and a step of 1° or more")
    return out


def reach_view(plan, band, T_base_flask_: np.ndarray, shaft_mm: float = 60.0) -> dict:
    """Counts, the reasons grouped (most first) and every pose for the 3D view: tip and a short shaft line, base mm."""
    reasons = Counter(u.reason for u in plan.unreachable)
    poses = []
    for ok, items in ((True, plan.reachable), (False, plan.unreachable)):
        for p in items:
            if p.pose is None:
                continue
            T = T_base_flask_ @ p.pose.T()
            pts = [T[:3, 3]]
            if isinstance(plan.tool, getattr(fs, "BentTool", ())):     # tip -> elbow -> back up the shaft
                a = math.radians(plan.tool.bend_angle_deg)
                pts.append(pts[0] - plan.tool.bend_length * T[:3, 2])
                pts.append(pts[1] - shaft_mm / 1000.0 * (T[:3, :3] @ [-math.sin(a), 0.0, math.cos(a)]))
            else:
                pts.append(pts[0] - T[:3, 2] * shaft_mm / 1000.0)
            mm = [[round(1000.0 * float(v), 2) for v in q] for q in pts]
            poses.append({"ok": ok, "tip": mm[0], "tail": mm[-1], "pts": mm,
                          "theta": round(p.theta_deg, 2), "az": round(p.azimuth_deg, 2)})
    return {"band": list(band) if band else None, "reachable": len(plan.reachable),
            "unreachable": len(plan.unreachable), "reasons": [[r, n] for r, n in reasons.most_common()],
            "poses": poses, "draw": draw_shapes(plan.geometry, T_base_flask_, plan.pivot)}


def add_speed(program, speed_mm_s: float) -> int:
    """speed_mm_s on every linear move of a built program (cart_speed.py); returns how many steps got it."""
    n = 0
    for s in program.steps:
        if s.type == "move_pose" and s.params.get("cartesian"):
            s.params["speed_mm_s"] = float(speed_mm_s)
            n += 1
    return n


# ---------------------------------------------------------------- profiles
class FlaskLibrary:
    """Named flasks (fitted geometry in the base frame), kept per robot in config_dir()/flasks/<profile>.yaml."""

    def __init__(self, path: Path, base_link: str = "base_link"):
        self.path = Path(path)
        self.base_link = base_link
        self.saved: Dict[str, dict] = self._load()

    def _load(self) -> Dict[str, dict]:
        try:
            d = yaml.safe_load(self.path.read_text()) or {}
        except Exception:                      # missing or unreadable: no saved flasks
            return {}
        return {str(k): v for k, v in (d.get("flasks") or {}).items() if isinstance(v, dict) and "geometry" in v}

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("# MoveIt GUI flask profiles: fitted round-bottom flasks in the robot base frame (m).\n" +
                       yaml.safe_dump({"version": 1, "flasks": self.saved}, sort_keys=False))
        tmp.replace(self.path)

    def get(self, name: str) -> FlaskGeometry:
        e = self.saved.get(str(name))
        if e is None:
            raise FlaskSetupError(f"no flask profile '{name}'")
        try:
            return FlaskGeometry.from_dict(e["geometry"])
        except FlaskGeometryError as ex:
            raise FlaskSetupError(f"flask profile '{name}': {ex}") from None

    def save(self, name: str, g: FlaskGeometry, source: str = "touched") -> str:
        name = str(name).strip()
        if not NAME.match(name):
            raise FlaskSetupError("a flask name is 1-40 letters, digits, space, _ . -")
        self.saved[name] = {"geometry": g.to_dict(), "size": g.standard_size, "source": str(source),
                            "saved": time.strftime("%Y-%m-%d %H:%M")}
        self._write()
        return name

    def delete(self, name: str):
        if self.saved.pop(str(name), None) is None:
            raise FlaskSetupError(f"no flask profile '{name}'")
        self._write()

    def view(self, fixtures: dict) -> List[dict]:
        """Every profile, with the fixtures it is in use as (its name in the meta, the same sphere centre)."""
        out = []
        for n, e in self.saved.items():
            g = e["geometry"]
            used = [f for f, fx in fixtures.items() if fx.meta.get("flask_profile") == n and fx.parent == self.base_link
                    and np.allclose(fx.T_parent_fixture[:3, 3], g["sphere_center"], atol=5e-5)]
            out.append({"name": n, "size": e.get("size", ""), "od_mm": round(2000.0 * float(g["R_outer"]), 2),
                        "neck_id_mm": round(2000.0 * float(g["neck_inner_radius"]), 2),
                        "centre_mm": [round(1000.0 * float(v), 1) for v in g["sphere_center"]],
                        "source": e.get("source", ""), "saved": e.get("saved", ""),
                        "warnings": len(g.get("warnings") or []), "in_use_as": used})
        return out


def suggestion_view(sg) -> dict:
    """flask_scrape.BendSuggestion in the page's units (mm, °)."""
    return {"length_mm": 1000.0 * sg.recommended_length_m, "angle_deg": sg.recommended_angle_deg,
            "length_range_mm": [1000.0 * v for v in sg.bend_length_range_m],
            "angle_range_deg": list(sg.bend_angle_range_deg),
            "band": list(sg.reachable_band_deg) if sg.reachable_band_deg else None,
            "target": list(sg.target_band_deg), "meets": bool(sg.meets_target)}
