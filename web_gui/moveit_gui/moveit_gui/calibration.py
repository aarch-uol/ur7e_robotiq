"""User calibration: virtual TCPs, fixture frames, vision baselines.

Kept separate from (and never written into) the package robot profiles.
Stored per profile in  $XDG_CONFIG_HOME/moveit_gui/calibrations/<profile>.yaml
(default ~/.config/moveit_gui/...). Loaded with yaml.safe_load only.

The calibration hierarchy, each kept separate:
  robot kinematics   -> UR factory calibration (ur7e_calibration.yaml), not handled here
  TCPs               -> `tcps`      (model links come from the profile, virtual ones from here)
  fixtures           -> `fixtures`  static workcell frames, taught or measured
  vision baselines   -> `vision`    correction of a dynamic pose source
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import transforms as tf
from .profile import RobotProfile

CALIB_VERSION = 1
NAME_OK = set("abcdefghijklmnopqrstuvwxyz0123456789_")


class CalibrationError(ValueError):
    pass


def config_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or str(Path.home() / ".config")
    return Path(base) / "moveit_gui"


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def check_name(name: str, what: str) -> str:
    if not name or not set(name) <= NAME_OK or name[0].isdigit():
        raise CalibrationError(f"{what} name '{name}' must be lower_snake_case (a-z, 0-9, _)")
    return name


def _T_to_yaml(T: np.ndarray) -> dict:
    p, q = tf.to_pos_quat(T)
    return {"position": [round(v, 7) for v in p], "quat_xyzw": [round(v, 9) for v in q]}


def _T_from_yaml(d: dict, what: str) -> np.ndarray:
    try:
        return tf.make(d["position"], tf.check_unit_quat(d["quat_xyzw"]))
    except (KeyError, TypeError) as e:
        raise CalibrationError(f"{what}: needs position [x,y,z] and quat_xyzw [x,y,z,w] ({e})") from None
    except tf.TransformError as e:
        raise CalibrationError(f"{what}: {e}") from None


# ===================================================================== data
@dataclass
class TcpDef:
    """A tool centre point.

    model_link: a link that exists in TF / the robot description (from the profile, read-only).
    virtual:    T_parent_tcp relative to another TCP or link. Pose semantics only: does NOT
                change MoveIt collision geometry.
    """
    name: str
    type: str                       # "model_link" | "virtual"
    link: str = ""                  # model_link
    moveit_target: bool = False     # model_link: MoveIt may take pose goals for this link directly
    parent: str = ""                # virtual: name of a TCP or a TF link
    T_parent_tcp: Optional[np.ndarray] = None
    meta: dict = field(default_factory=dict)   # method, sample_count, residual, created, modified

    @property
    def builtin(self) -> bool:
        return self.type == "model_link"

    def to_yaml(self) -> dict:
        if self.type == "model_link":
            return {"type": "model_link", "link": self.link}
        return {"type": "virtual", "parent": self.parent, **_T_to_yaml(self.T_parent_tcp), **self.meta}


@dataclass
class FixtureDef:
    """Static workcell frame: T_parent_fixture."""
    name: str
    parent: str
    T_parent_fixture: np.ndarray
    meta: dict = field(default_factory=dict)

    def to_yaml(self) -> dict:
        return {"parent": self.parent, **_T_to_yaml(self.T_parent_fixture), **self.meta}


CORRECTION_SIDES = ("reference", "object")


@dataclass
class VisionBaseline:
    """Correction of a pose source, all in `reference_frame` R.

    correction_side = "reference" (default): the error lives in the reference/camera frame,
    e.g. camera extrinsic error, T_R_det = T_err @ T_R_true:
        T_correction = T_R_obj_expected @ inv(T_R_obj_detected_baseline)
        T_R_obj_corrected = T_correction @ T_R_obj_detected
    correction_side = "object": the error lives in the object frame, e.g. the detector's object
    model has an offset origin, T_R_det = T_R_true @ T_err:
        T_correction = inv(T_R_obj_detected_baseline) @ T_R_obj_expected
        T_R_obj_corrected = T_R_obj_detected @ T_correction
    Both are exact at the baseline pose; they differ once the object moves or rotates, so pick
    the one matching the error source (or re-baseline near where the object is used).
    """
    name: str                        # object frame name programs use
    source_type: str                 # "tf" | "pose_topic"
    source: str                      # TF child frame or PoseStamped topic
    reference_frame: str
    T_correction: np.ndarray
    meta: dict = field(default_factory=dict)
    correction_side: str = "reference"

    def to_yaml(self) -> dict:
        return {"source_type": self.source_type, "source": self.source,
                "reference_frame": self.reference_frame, "correction_side": self.correction_side,
                "correction": _T_to_yaml(self.T_correction), **self.meta}

    def corrected(self, T_R_obj_detected: np.ndarray) -> np.ndarray:
        return apply_vision_correction(self.T_correction, T_R_obj_detected, self.correction_side)


# ==================================================================== store
class CalibrationStore:
    def __init__(self, profile: RobotProfile, path: Optional[Path] = None):
        self.profile = profile
        self.path = Path(path) if path else config_dir() / "calibrations" / f"{profile.name}.yaml"
        self.tcps: Dict[str, TcpDef] = {}
        self.fixtures: Dict[str, FixtureDef] = {}
        self.vision: Dict[str, VisionBaseline] = {}
        self.active_tcp: str = profile.default_tcp
        self.notes: List[str] = []
        self._load_builtin()
        if self.path.exists():
            self.load_file(self.path, replace=True)

    def _load_builtin(self):
        for name, t in self.profile.model_tcps.items():
            self.tcps[name] = TcpDef(name=name, type="model_link", link=t["link"],
                                     moveit_target=t["moveit_target"])

    # -------------------------------------------------------------- parsing
    def parse(self, data, source: str) -> Tuple[Dict[str, TcpDef], Dict[str, FixtureDef],
                                                Dict[str, VisionBaseline], Optional[str]]:
        """Validate a calibration document without changing the store."""
        if not isinstance(data, dict):
            raise CalibrationError(f"{source}: not a calibration document")
        if int(data.get("version", 0)) != CALIB_VERSION:
            raise CalibrationError(f"{source}: unsupported calibration version {data.get('version')}")
        robot = data.get("robot")
        if robot and robot != self.profile.name:
            raise CalibrationError(f"{source}: calibration is for robot '{robot}', "
                                   f"current profile is '{self.profile.name}'")
        tcps: Dict[str, TcpDef] = {}
        for name, d in (data.get("tcps") or {}).items():
            check_name(name, "TCP")
            if not isinstance(d, dict):
                raise CalibrationError(f"TCP {name}: must be a mapping")
            if d.get("type") == "model_link":
                if name not in self.profile.model_tcps or self.profile.model_tcps[name]["link"] != d.get("link"):
                    raise CalibrationError(f"TCP {name}: model links come from the robot profile "
                                           "and cannot be defined in a calibration file")
                continue
            if d.get("type") != "virtual":
                raise CalibrationError(f"TCP {name}: type must be 'virtual' (got {d.get('type')})")
            if name in self.profile.model_tcps:
                raise CalibrationError(f"TCP {name}: name clashes with a model-link TCP")
            meta = {k: v for k, v in d.items() if k not in ("type", "parent", "position", "quat_xyzw")}
            tcps[name] = TcpDef(name=name, type="virtual", parent=str(d.get("parent", "")),
                                T_parent_tcp=_T_from_yaml(d, f"TCP {name}"), meta=meta)
        fixtures: Dict[str, FixtureDef] = {}
        for name, d in (data.get("fixtures") or {}).items():
            check_name(name, "fixture")
            if not isinstance(d, dict) or not d.get("parent"):
                raise CalibrationError(f"fixture {name}: needs a parent frame")
            meta = {k: v for k, v in d.items() if k not in ("parent", "position", "quat_xyzw")}
            fixtures[name] = FixtureDef(name, str(d["parent"]), _T_from_yaml(d, f"fixture {name}"), meta)
        vision: Dict[str, VisionBaseline] = {}
        for name, d in (data.get("vision") or {}).items():
            check_name(name, "vision object")
            if not isinstance(d, dict):
                raise CalibrationError(f"vision {name}: must be a mapping")
            if d.get("source_type") not in ("tf", "pose_topic"):
                raise CalibrationError(f"vision {name}: source_type must be tf or pose_topic")
            for k in ("source", "reference_frame", "correction"):
                if not d.get(k):
                    raise CalibrationError(f"vision {name}: missing {k}")
            side = d.get("correction_side", "reference")
            if side not in CORRECTION_SIDES:
                raise CalibrationError(f"vision {name}: correction_side must be reference or object")
            meta = {k: v for k, v in d.items()
                    if k not in ("source_type", "source", "reference_frame", "correction", "correction_side")}
            vision[name] = VisionBaseline(name, d["source_type"], str(d["source"]), str(d["reference_frame"]),
                                          _T_from_yaml(d["correction"], f"vision {name} correction"), meta,
                                          side)
        names = list(tcps) + list(fixtures) + list(vision)
        dup = {n for n in names if names.count(n) > 1}
        if dup:
            raise CalibrationError(f"{source}: names used twice: {', '.join(sorted(dup))}")
        active = data.get("active_tcp")
        return tcps, fixtures, vision, active

    def _check_graph(self, tcps: Dict[str, TcpDef], fixtures: Dict[str, FixtureDef]):
        """Every virtual TCP must chain to a model link without cycles; fixture chains acyclic."""
        all_tcps = {**{n: t for n, t in self.tcps.items() if t.builtin}, **tcps}
        for t in tcps.values():
            seen, cur = set(), t
            while cur.type == "virtual":
                if cur.name in seen:
                    raise CalibrationError(f"TCP {t.name}: parent chain has a cycle")
                seen.add(cur.name)
                parent = all_tcps.get(cur.parent)
                if parent is None:
                    if cur.parent in [m["link"] for m in self.profile.model_tcps.values()]:
                        break
                    raise CalibrationError(f"TCP {t.name}: parent '{cur.parent}' is not a known TCP")
                cur = parent
        known = {self.profile.base_link, *self.profile.tf_frames, *fixtures, *self.vision}
        for f in fixtures.values():
            if f.parent not in known:
                raise CalibrationError(f"fixture {f.name}: parent frame '{f.parent}' is not base, a fixture, "
                                       "a vision object or a profile tf_frame")
        for f in fixtures.values():
            seen, cur = set(), f
            while cur is not None:
                if cur.name in seen:
                    raise CalibrationError(f"fixture {f.name}: parent chain has a cycle")
                seen.add(cur.name)
                cur = fixtures.get(cur.parent)

    # ------------------------------------------------------------- load/save
    def load_file(self, path: Path, replace: bool):
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        tcps, fixtures, vision, active = self.parse(data, str(path))
        if not replace:
            clash = [n for n in list(tcps) + list(fixtures) + list(vision)
                     if n in self.tcps or n in self.fixtures or n in self.vision]
            if clash:
                raise CalibrationError("import would overwrite: " + ", ".join(clash)
                                       + ". Rename or delete those first.")
            tcps = {**{n: t for n, t in self.tcps.items() if not t.builtin}, **tcps}
            fixtures = {**self.fixtures, **fixtures}
            vision = {**self.vision, **vision}
        self._check_graph(tcps, fixtures)
        self.tcps = {n: t for n, t in self.tcps.items() if t.builtin}
        self.tcps.update(tcps)
        self.fixtures, self.vision = fixtures, vision
        if replace and active:
            if active in self.tcps:
                self.active_tcp = active
            else:
                self.notes.append(f"active TCP '{active}' from {path.name} no longer exists; "
                                  f"using '{self.profile.default_tcp}'")

    def to_yaml(self) -> dict:
        return {
            "version": CALIB_VERSION,
            "robot": self.profile.name,
            "ur_type": self.profile.ur_type,
            "robot_ip": self.profile.robot_ip,
            "saved": now_iso(),
            "active_tcp": self.active_tcp,
            "tcps": {n: t.to_yaml() for n, t in self.tcps.items() if not t.builtin},
            "fixtures": {n: f.to_yaml() for n, f in self.fixtures.items()},
            "vision": {n: v.to_yaml() for n, v in self.vision.items()},
        }

    def save(self, path: Optional[Path] = None):
        path = Path(path) if path else self.path
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".yaml.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            f.write("# MoveIt GUI calibration. Lengths in metres, quaternions xyzw.\n")
            yaml.safe_dump(self.to_yaml(), f, sort_keys=False, default_flow_style=None)
        os.replace(tmp, path)

    def export(self, path: Path):
        self.save(path)

    def import_file(self, path: Path):
        self.load_file(Path(path), replace=False)
        self.save()

    # -------------------------------------------------------------- editing
    def _name_free(self, name: str, kind: str):
        check_name(name, kind)
        if name in self.tcps or name in self.fixtures or name in self.vision:
            raise CalibrationError(f"name '{name}' is already used")
        if name in (self.profile.base_link, self.profile.planning_link) or name in self.profile.tf_frames:
            raise CalibrationError(f"name '{name}' is a robot frame")

    def put_tcp(self, tcp: TcpDef, replace: bool = False):
        if tcp.type != "virtual":
            raise CalibrationError("only virtual TCPs can be created or edited")
        if not tf.is_valid(tcp.T_parent_tcp):
            raise CalibrationError("invalid TCP transform")
        if replace:
            if tcp.name not in self.tcps or self.tcps[tcp.name].builtin:
                raise CalibrationError(f"no editable TCP '{tcp.name}'")
            tcp.meta.setdefault("created", self.tcps[tcp.name].meta.get("created", now_iso()))
        else:
            self._name_free(tcp.name, "TCP")
            tcp.meta.setdefault("created", now_iso())
        tcp.meta["modified"] = now_iso()
        tcp.meta.setdefault("robot", self.profile.name)
        new = {n: t for n, t in self.tcps.items() if not t.builtin}
        new[tcp.name] = tcp
        self._check_graph(new, self.fixtures)
        self.tcps[tcp.name] = tcp
        self.save()

    def delete_tcp(self, name: str):
        t = self.tcps.get(name)
        if t is None:
            raise CalibrationError(f"no TCP '{name}'")
        if t.builtin:
            raise CalibrationError(f"'{name}' is a model link from the robot profile and cannot be deleted")
        children = [c.name for c in self.tcps.values() if c.type == "virtual" and c.parent == name]
        if children:
            raise CalibrationError(f"'{name}' is the parent of: {', '.join(children)}")
        del self.tcps[name]
        if self.active_tcp == name:
            self.active_tcp = self.profile.default_tcp
        self.save()

    def set_active_tcp(self, name: str):
        if name not in self.tcps:
            raise CalibrationError(f"no TCP '{name}'")
        self.active_tcp = name
        self.save()

    def put_fixture(self, fx: FixtureDef, replace: bool = False):
        if not tf.is_valid(fx.T_parent_fixture):
            raise CalibrationError("invalid fixture transform")
        if not replace:
            self._name_free(fx.name, "fixture")
            fx.meta.setdefault("created", now_iso())
        fx.meta["modified"] = now_iso()
        fx.meta.setdefault("robot", self.profile.name)
        new = dict(self.fixtures)
        new[fx.name] = fx
        self._check_graph({n: t for n, t in self.tcps.items() if not t.builtin}, new)
        self.fixtures[fx.name] = fx
        self.save()

    def delete_fixture(self, name: str):
        children = [f.name for f in self.fixtures.values() if f.parent == name]
        if children:
            raise CalibrationError(f"'{name}' is the parent of: {', '.join(children)}")
        self.fixtures.pop(name, None)
        self.save()

    def put_vision(self, vb: VisionBaseline, replace: bool = False):
        if not tf.is_valid(vb.T_correction):
            raise CalibrationError("invalid correction transform")
        if not replace:
            self._name_free(vb.name, "vision object")
            vb.meta.setdefault("created", now_iso())
        vb.meta["modified"] = now_iso()
        vb.meta.setdefault("robot", self.profile.name)
        self.vision[vb.name] = vb
        self.save()

    def delete_vision(self, name: str):
        self.vision.pop(name, None)
        self.save()

    def frame_names(self) -> List[str]:
        out = [self.profile.base_link] + list(self.fixtures) + list(self.vision)
        for f in self.profile.tf_frames:
            if f not in out:
                out.append(f)
        return out


# ============================================================ pivot calib
@dataclass
class PivotResult:
    p_parent_tcp: np.ndarray      # TCP position in the parent link frame [m]
    p_base_pivot: np.ndarray      # pivot point in the base frame [m]
    residuals_m: np.ndarray       # per-sample distance from the common pivot
    rms_m: float
    max_m: float
    quality: str                  # insufficient | poor | acceptable | good
    quality_reason: str
    angular_span_deg: float
    sample_count: int


PIVOT_MIN_SAMPLES = 4


def pivot_quality(T_base_parent_samples: Sequence[np.ndarray]) -> Tuple[str, str, float]:
    """Orientation diversity check. The problem is only solvable when the
    rotations span at least two independent axes; we use the largest pairwise
    rotation angle and the conditioning of the stacked system."""
    n = len(T_base_parent_samples)
    if n < PIVOT_MIN_SAMPLES:
        return "insufficient", f"{n}/{PIVOT_MIN_SAMPLES} samples", 0.0
    Rs = [T[:3, :3] for T in T_base_parent_samples]
    span = max(tf.rotation_angle(a.T @ b) for i, a in enumerate(Rs) for b in Rs[i + 1:])
    A = np.vstack([np.hstack([R, -np.eye(3)]) for R in Rs])
    s = np.linalg.svd(A, compute_uv=False)
    cond = s[0] / s[-1] if s[-1] > 1e-12 else math.inf
    span_deg = math.degrees(span)
    if not math.isfinite(cond) or cond > 1e3 or span_deg < 10:
        return "poor", (f"orientations too similar (span {span_deg:.0f}°, cond {cond:.0f}); "
                        "tilt the tool about two different axes"), span_deg
    if span_deg < 30 or cond > 50:
        return "acceptable", f"span {span_deg:.0f}°, cond {cond:.1f}; more tilt improves accuracy", span_deg
    return "good", f"span {span_deg:.0f}°, cond {cond:.1f}", span_deg


def solve_pivot(T_base_parent_samples: Sequence[np.ndarray]) -> PivotResult:
    """Least squares for  R_i p_tcp + t_i = p_pivot  (all i).

    T_base_parent_samples: poses of the parent link (e.g. tool0 or robotiq_tcp) in the base frame,
    captured while the tool tip stays on one fixed point. Gives the tip POSITION in the parent
    frame only; orientation is not observable from a pivot and is not calibrated here.
    """
    quality, reason, span = pivot_quality(T_base_parent_samples)
    if quality in ("insufficient", "poor"):
        raise CalibrationError(f"pivot data {quality}: {reason}")
    A = np.vstack([np.hstack([T[:3, :3], -np.eye(3)]) for T in T_base_parent_samples])
    b = np.concatenate([-T[:3, 3] for T in T_base_parent_samples])
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    p_tcp, p_pivot = x[:3], x[3:]
    res = np.array([np.linalg.norm(T[:3, :3] @ p_tcp + T[:3, 3] - p_pivot) for T in T_base_parent_samples])
    return PivotResult(p_tcp, p_pivot, res, float(math.sqrt(float(np.mean(res ** 2)))), float(res.max()),
                       quality, reason, span, len(T_base_parent_samples))


# ========================================================= fixture teach
FIXTURE_MIN_SEPARATION_M = 0.01
FIXTURE_MIN_ANGLE_DEG = 15.0


def frame_from_three_points(p_origin, p_x, p_y) -> np.ndarray:
    """Taught frame: origin, a point on +X, a point in the XY half-plane with +Y.

    X = normalize(p_x - origin);  Z = normalize(X x (p_y - origin));  Y = Z x X.
    Points are in the parent frame (normally base_link); returns T_parent_fixture.
    """
    o, px, py = (np.asarray(v, dtype=float) for v in (p_origin, p_x, p_y))
    vx, vy = px - o, py - o
    for name, v in (("+X point", vx), ("+Y point", vy)):
        if np.linalg.norm(v) < FIXTURE_MIN_SEPARATION_M:
            raise CalibrationError(f"{name} is closer than {FIXTURE_MIN_SEPARATION_M * 1000:.0f} mm to the origin")
    if np.linalg.norm(px - py) < FIXTURE_MIN_SEPARATION_M:
        raise CalibrationError("+X and +Y points are too close together")
    x = vx / np.linalg.norm(vx)
    sin_angle = np.linalg.norm(np.cross(x, vy / np.linalg.norm(vy)))
    if sin_angle < math.sin(math.radians(FIXTURE_MIN_ANGLE_DEG)):
        raise CalibrationError(f"points are almost collinear (angle < {FIXTURE_MIN_ANGLE_DEG:.0f}°)")
    z = np.cross(x, vy)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return tf.from_rotation(np.column_stack([x, y, z]), o)


# ========================================================= vision baseline
def apply_vision_correction(T_correction: np.ndarray, T_R_obj_detected: np.ndarray,
                            side: str = "reference") -> np.ndarray:
    if side == "reference":
        return T_correction @ T_R_obj_detected
    if side == "object":
        return T_R_obj_detected @ T_correction
    raise CalibrationError(f"unknown correction side '{side}'")


def solve_vision_baseline(T_R_obj_expected: np.ndarray, T_R_obj_detected_samples: Sequence[np.ndarray],
                          side: str = "reference", orientation: bool = False) -> Tuple[np.ndarray, dict]:
    """Correction from a ground truth and averaged detector samples (all in frame R).

    side "reference": T_correction = T_expected @ inv(mean(T_detected))
    side "object":    T_correction = inv(mean(T_detected)) @ T_expected

    orientation=False (default): position-only correction. The rotation of the correction is
    identity and the ground-truth orientation is ignored, because a probe touch measures a
    POSITION; it says nothing reliable about the object's orientation. Use orientation=True only
    with a deliberate orientation source (fixture frame, aligned tool, explicit entry).
    """
    if side not in CORRECTION_SIDES:
        raise CalibrationError(f"unknown correction side '{side}'")
    if len(T_R_obj_detected_samples) < 3:
        raise CalibrationError("need at least 3 detector samples")
    T_det_mean, stats = tf.average_transforms(T_R_obj_detected_samples)
    if orientation:
        T_target = T_R_obj_expected
    else:   # keep the detected orientation, correct only the position
        T_target = tf.from_rotation(T_det_mean[:3, :3], T_R_obj_expected[:3, 3])
    if side == "reference":
        T_corr = T_target @ tf.inv(T_det_mean)
        if not orientation:      # pure translation, no rotation about the reference origin
            T_corr = tf.make(T_R_obj_expected[:3, 3] - T_det_mean[:3, 3])
    else:
        T_corr = tf.inv(T_det_mean) @ T_target
    stats["components"] = ["position", "orientation"] if orientation else ["position"]
    stats["correction_translation_m"] = float(np.linalg.norm(T_corr[:3, 3]))
    stats["correction_angle_deg"] = math.degrees(tf.rotation_angle(T_corr[:3, :3]))
    return T_corr, stats
