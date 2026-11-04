"""Keep-out zones: named boxes that MoveIt must plan around (MoveIt planning-scene collision objects).

Like MoveIt Pro's Planning Scene Editor, for a table, a vial holder, a camera mount: a box is
{name, frame, size_mm, xyz_mm, rpy_deg, enabled}, with xyz/rpy the box CENTRE in `frame`.
`frame` is the base frame or a calibration fixture (e.g. the `vial` frame from SETUP > Vial Geometry), so a
box on a fixture follows that fixture when it is re-taught.

Stored per profile next to the calibration file:
    $XDG_CONFIG_HOME/moveit_gui/calibrations/<profile>.keepout.yaml
(yaml.safe_load only). Kept out of the calibration document so calibration import/export stays unchanged.

Applied through the bridge (bridge.scene_set_box / scene_remove / scene_objects) as world collision objects
named `gui_keepout/<name>`, always expressed in the base frame. Only objects with that prefix are ever touched:
other planning-scene objects (yours, MoveIt Pro's, a perception pipeline's) are left alone.

This changes what MoveIt PLANS; it never commands motion. A box around the robot's current TCP / tool flange /
base would put MoveIt's start state in collision and make every plan fail, so such a box is refused on edit and
skipped (with a warning) when it is applied.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import yaml

from . import transforms as tf
from .calibration import CalibrationError, check_name, now_iso

KEEPOUT_VERSION = 1
PREFIX = "gui_keepout/"
MAX_SIZE_MM = 3000.0            # a dimension over 3 m is surely a typo (units are mm)
MIN_SIZE_MM = 1.0
MAX_OFFSET_MM = 3000.0          # box centre in its frame
INSIDE_TOL_M = 0.001            # a point counts as inside when it is deeper than 1 mm
NEAR_M = 0.03                   # warn when the TCP is closer than this to a box
VERIFY_TRIES = 4                # planning-scene read-backs after publishing
VERIFY_WAIT_S = 0.5
RETRY_S = 3.0                   # retry an incomplete apply (robot pose unknown, box skipped, not verified)
POLL_S = 1.0                    # how often poll() re-checks the fixture frames
CHECK_S = 8.0                   # while idle: re-read the scene; a move_group that restarted fast has lost the boxes


class KeepoutError(CalibrationError):
    """Invalid keep-out box or edit (a CalibrationError, so front ends report it like one)."""


@dataclass
class KeepoutBox:
    name: str
    frame: str
    size_mm: List[float]
    xyz_mm: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    enabled: bool = True
    meta: dict = field(default_factory=dict)       # created, modified

    @property
    def object_id(self) -> str:
        return PREFIX + self.name

    def size_m(self) -> List[float]:
        return [v / 1000.0 for v in self.size_mm]

    def T_frame_box(self) -> np.ndarray:
        return tf.mm_deg_to_T(self.xyz_mm, self.rpy_deg)

    def to_yaml(self) -> dict:
        return {"frame": self.frame, "size_mm": [round(v, 3) for v in self.size_mm],
                "xyz_mm": [round(v, 3) for v in self.xyz_mm], "rpy_deg": [round(v, 4) for v in self.rpy_deg],
                "enabled": bool(self.enabled), **self.meta}

    def view(self) -> dict:
        return {"name": self.name, "frame": self.frame, "size_mm": list(self.size_mm), "xyz_mm": list(self.xyz_mm),
                "rpy_deg": list(self.rpy_deg), "enabled": bool(self.enabled), "id": self.object_id}


def _vec3(v, what: str) -> List[float]:
    try:
        out = [float(x) for x in v]
    except (TypeError, ValueError):
        raise KeepoutError(f"{what} must be three numbers") from None
    if len(out) != 3 or not all(math.isfinite(x) for x in out):
        raise KeepoutError(f"{what} must be three finite numbers")
    return out


def make_box(name, frame, size_mm, xyz_mm, rpy_deg, enabled=True, meta=None) -> KeepoutBox:
    """Validated box from loose values (web form, YAML). Frame existence is checked by the store."""
    try:
        name = check_name(str(name or "").strip(), "keep-out box")
    except CalibrationError as e:
        raise KeepoutError(str(e)) from None
    if not frame or not isinstance(frame, str):
        raise KeepoutError(f"keep-out box {name}: needs a frame")
    size = _vec3(size_mm, f"keep-out box {name}: size")
    if any(s < MIN_SIZE_MM for s in size):
        raise KeepoutError(f"keep-out box {name}: every size must be at least {MIN_SIZE_MM:g} mm")
    if any(s > MAX_SIZE_MM for s in size):
        raise KeepoutError(f"keep-out box {name}: a side over {MAX_SIZE_MM / 1000:g} m is surely a typo (units are mm)")
    xyz = _vec3(xyz_mm, f"keep-out box {name}: position")
    if any(abs(v) > MAX_OFFSET_MM for v in xyz):
        raise KeepoutError(f"keep-out box {name}: the centre is more than {MAX_OFFSET_MM / 1000:g} m from {frame}")
    rpy = _vec3(rpy_deg, f"keep-out box {name}: rotation")
    if any(abs(v) > 360.0 for v in rpy):
        raise KeepoutError(f"keep-out box {name}: angles must be within ±360°")
    if not isinstance(enabled, bool):
        raise KeepoutError(f"keep-out box {name}: enabled must be true or false")
    return KeepoutBox(name, frame, size, xyz, rpy, enabled, dict(meta or {}))


def point_box_distance(p_base, T_base_box: np.ndarray, size_m) -> Tuple[float, float]:
    """(distance outside the box, depth inside the box) of a point, in metres. One of them is 0."""
    p = tf.inv(T_base_box) @ np.array([*map(float, p_base), 1.0])
    half = np.asarray(size_m, float) / 2.0
    out = np.maximum(np.abs(p[:3]) - half, 0.0)
    outside = float(np.linalg.norm(out))
    inside = float(np.min(half - np.abs(p[:3]))) if outside == 0.0 else 0.0
    return outside, inside


def point_in_box(p_base, T_base_box: np.ndarray, size_m, tol: float = INSIDE_TOL_M) -> bool:
    outside, inside = point_box_distance(p_base, T_base_box, size_m)
    return outside == 0.0 and inside > tol


class KeepoutStore:
    """The persisted list of boxes (insertion order = display order)."""

    def __init__(self, calib_path: Path, profile_name: str, frame_ok: Callable[[str], bool]):
        self.path = Path(calib_path).with_name(f"{profile_name}.keepout.yaml")
        self.profile_name = profile_name
        self.frame_ok = frame_ok
        self.boxes: Dict[str, KeepoutBox] = {}
        self.notes: List[str] = []
        if self.path.exists():
            try:
                self.boxes = self.parse(yaml.safe_load(self.path.read_text(encoding="utf-8")), str(self.path))
            except (OSError, yaml.YAMLError, CalibrationError) as e:
                # Never start with a half-read list silently: report it, keep the file untouched.
                self.notes.append(f"keep-out zones NOT loaded from {self.path}: {e}")

    def parse(self, data, source: str) -> Dict[str, KeepoutBox]:
        if data is None:
            return {}
        if not isinstance(data, dict):
            raise KeepoutError(f"{source}: not a keep-out document")
        if int(data.get("version", 0)) != KEEPOUT_VERSION:
            raise KeepoutError(f"{source}: unsupported keep-out version {data.get('version')}")
        robot = data.get("robot")
        if robot and robot != self.profile_name:
            raise KeepoutError(f"{source}: keep-out zones are for robot '{robot}', not '{self.profile_name}'")
        out: Dict[str, KeepoutBox] = {}
        boxes = data.get("boxes") or {}
        if not isinstance(boxes, dict):
            raise KeepoutError(f"{source}: boxes must be a mapping name -> box")
        for name, d in boxes.items():
            if not isinstance(d, dict):
                raise KeepoutError(f"keep-out box {name}: must be a mapping")
            meta = {k: v for k, v in d.items() if k not in ("frame", "size_mm", "xyz_mm", "rpy_deg", "enabled")}
            b = make_box(name, d.get("frame"), d.get("size_mm"), d.get("xyz_mm", [0, 0, 0]),
                         d.get("rpy_deg", [0, 0, 0]), d.get("enabled", True), meta)
            out[b.name] = b       # unknown frames are allowed on load (a fixture may come back): shown as an error
        return out

    def to_yaml(self) -> dict:
        return {"version": KEEPOUT_VERSION, "robot": self.profile_name, "saved": now_iso(),
                "boxes": {n: b.to_yaml() for n, b in self.boxes.items()}}

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".yaml.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            f.write("# MoveIt GUI keep-out zones (MoveIt collision boxes). Box centre pose in `frame`; mm, deg.\n")
            yaml.safe_dump(self.to_yaml(), f, sort_keys=False, default_flow_style=None)
        os.replace(tmp, self.path)

    def put(self, box: KeepoutBox, replace: Optional[str] = None):
        """Add a box, or (replace=old name) change one, possibly renaming it."""
        if not self.frame_ok(box.frame):
            raise KeepoutError(f"keep-out box {box.name}: frame '{box.frame}' is not the base frame or a fixture")
        if replace is not None:
            if replace not in self.boxes:
                raise KeepoutError(f"no keep-out box '{replace}'")
            if box.name != replace and box.name in self.boxes:
                raise KeepoutError(f"a keep-out box named '{box.name}' already exists")
            box.meta.setdefault("created", self.boxes[replace].meta.get("created", now_iso()))
        else:
            if box.name in self.boxes:
                raise KeepoutError(f"a keep-out box named '{box.name}' already exists")
            box.meta.setdefault("created", now_iso())
        box.meta["modified"] = now_iso()
        new = {}
        for n, b in self.boxes.items():                    # keep the position in the list when editing
            if n == replace:
                new[box.name] = box
            else:
                new[n] = b
        if replace is None:
            new[box.name] = box
        self.boxes = new
        self.save()

    def delete(self, name: str):
        if name not in self.boxes:
            raise KeepoutError(f"no keep-out box '{name}'")
        del self.boxes[name]
        self.save()


class KeepoutScene:
    """Keeps MoveIt's planning scene in step with the store.

    apply() is blocking (publish, then read the planning scene back) and serialised; poll() is called from the
    front end's update loop and starts apply() in a thread when MoveIt (re)appears, when a fixture a box hangs
    on has moved, or to retry an incomplete apply. Automatic applies wait until the robot is idle.
    """

    def __init__(self, store: KeepoutStore, geo, bridge, active_tcp: Callable[[], str], log: Callable[[str], None]):
        self.store, self.geo, self.bridge = store, geo, bridge
        self.active_tcp = active_tcp
        self.log = log
        self._lock = threading.Lock()
        self._running = False
        self._need: Optional[str] = None
        self._moveit_up = False
        self._last_poll = 0.0
        self._last_try = 0.0
        self._last_check = time.monotonic()
        self._last_inside = 0.0
        self.inside: List[str] = []        # enabled boxes the robot is inside now (MoveIt will refuse every plan)
        self._applied_sig = None
        self._applied_ids: set = set()
        self.status: dict = {"state": "pending", "ok": None, "msg": "not applied yet (waiting for MoveIt)",
                             "count": 0, "skipped": [], "problems": [], "verified": False, "t": None,
                             "reason": "", "retry": False}
        self.supported = all(hasattr(bridge, m) for m in ("scene_set_box", "scene_remove", "scene_objects"))
        if not self.supported:
            self.status.update(state="unsupported", msg="this bridge cannot change the planning scene")

    # ------------------------------------------------------------ geometry
    def T_base_box(self, box: KeepoutBox) -> np.ndarray:
        """GeometryError if the frame cannot be resolved (e.g. a deleted fixture)."""
        if not (box.frame == self.geo.base or box.frame in self.geo.store.fixtures):
            from .geometry import GeometryError
            raise GeometryError(f"frame '{box.frame}' is not the base frame or a fixture")
        return self.geo.T_base_frame(box.frame) @ box.T_frame_box()

    def resolved(self) -> Tuple[Dict[str, Tuple[KeepoutBox, np.ndarray]], List[str]]:
        """Enabled boxes in the base frame, and the problems of those that cannot be placed."""
        out, problems = {}, []
        for b in list(self.store.boxes.values()):
            if not b.enabled:
                continue
            try:
                out[b.object_id] = (b, self.T_base_box(b))
            except Exception as e:  # noqa: BLE001 - GeometryError and malformed fixtures alike
                problems.append(f"{b.name}: {e}")
        return out, problems

    def signature(self):
        desired, problems = self.resolved()
        return (tuple(sorted((i, tuple(np.round(T, 5).ravel()), tuple(b.size_mm)) for i, (b, T) in desired.items())),
                tuple(problems))

    def robot_points(self) -> Tuple[Optional[List[Tuple[str, np.ndarray]]], str]:
        """Points of the robot that must never be inside a box: active TCP, tool flange, base origin."""
        tcp = self.active_tcp()
        pts = [(f"robot base ({self.geo.base} origin)", np.zeros(3))]
        try:
            pts.append((f"current TCP '{tcp}'", self.geo.current_T_base_tcp(tcp)[:3, 3]))
            link = self.geo.profile.planning_link
            pts.append((f"tool flange ({link})", self.geo.T_base_frame(link)[:3, 3]))
        except Exception as e:  # noqa: BLE001
            return None, f"the robot's TCP pose is unknown ({e})"
        return pts, ""

    def conflicts(self, T_base_box: np.ndarray, size_m) -> Tuple[Optional[List[str]], List[str], str]:
        """(robot parts inside the box or None if unknown, warnings for parts close to it, why unknown)."""
        pts, why = self.robot_points()
        if pts is None:
            return None, [], why
        inside, near = [], []
        for what, p in pts:
            outside, _ = point_box_distance(p, T_base_box, size_m)
            if point_in_box(p, T_base_box, size_m):
                inside.append(what)
            elif outside < NEAR_M and not what.startswith("robot base"):
                near.append(f"{what} is {outside * 1000:.0f} mm from it")
        return inside, near, ""

    def robot_inside(self) -> List[str]:
        """Enabled boxes that contain the TCP, flange or base now, e.g. after freedrive or a fixture re-teach."""
        try:
            desired, _ = self.resolved()
            out = []
            for _oid, (b, T) in desired.items():
                inside, _near, _why = self.conflicts(T, b.size_m())
                if inside:
                    out.append(f"{b.name} ({', '.join(inside)})")
            return out
        except Exception:  # noqa: BLE001 - a geometry hiccup must not stop the poll
            return []

    def check_edit(self, box: KeepoutBox) -> List[str]:
        """Refuse a box that would contain the robot. Returns warnings (robot close to the box)."""
        if not box.enabled:
            return []
        try:
            T = self.T_base_box(box)
        except Exception as e:  # noqa: BLE001
            raise KeepoutError(f"keep-out box {box.name}: {e}") from None
        inside, near, why = self.conflicts(T, box.size_m())
        if inside is None:
            raise KeepoutError(f"keep-out box {box.name}: cannot check it against the robot: {why}. "
                               "Save it disabled, or wait until the robot pose is known")
        if inside:
            raise KeepoutError(
                f"keep-out box {box.name} would contain the {', '.join(inside)}: MoveIt would start in collision and "
                "every plan would fail. Move the robot out first, or change the box")
        return near

    # --------------------------------------------------------------- apply
    def request(self, reason: str):
        self._need = reason

    def poll(self, moveit_up: bool, idle: bool):
        if not self.supported:
            return
        if moveit_up and not self._moveit_up:
            self._need = "MoveIt is available" if self.status["t"] is None else "MoveIt came back"
        self._moveit_up = moveit_up
        now = time.monotonic()
        if now - self._last_inside >= POLL_S:
            self._last_inside = now
            self.inside = self.robot_inside()
        if not moveit_up or not idle or self._running:
            return
        if self._need is None:
            if now - self._last_poll < POLL_S:
                return
            self._last_poll = now
            if self.signature() != self._applied_sig:
                self._need = "a fixture frame changed"
            elif self.status.get("retry") and now - self._last_try > RETRY_S:
                self._need = "retry"
            elif self._applied_ids and now - self._last_check > CHECK_S:
                # A move_group that crashes and respawns faster than DDS notices never looks "down", so the
                # zones would be gone with a green status: check that MoveIt still has them.
                self._last_check = now
                self._running = True
                threading.Thread(target=self._check, daemon=True, name="keepout-check").start()
                return
            else:
                return
        reason, self._need = self._need, None
        self._running = True
        threading.Thread(target=self._run, args=(reason,), daemon=True, name="keepout-apply").start()

    def _check(self):
        try:
            scene = self.bridge.scene_objects()
            gone = sorted(self._applied_ids - set(scene)) if scene is not None else []
            if gone:
                self.log(f"keep-out zones: MoveIt's planning scene lost {', '.join(gone)} "
                         "(move_group restarted?): applying again")
                self._set(state="unverified", ok=False, verified=False,
                          msg=f"MoveIt lost {len(gone)} box(es): applying again")
                self._need = "MoveIt lost the boxes"
        except Exception as e:  # noqa: BLE001
            self.log(f"keep-out zones: scene check failed: {e}")
        finally:
            self._running = False

    def _run(self, reason: str):
        try:
            self.apply(reason)
        except Exception as e:  # noqa: BLE001 - a background apply must never die silently
            self._set(state="error", ok=False, msg=f"apply failed: {type(e).__name__}: {e}", retry=True)
            self.log(f"keep-out zones: apply failed: {e}")
        finally:
            self._running = False

    def _set(self, **kw):
        st = dict(self.status)
        st.update(kw, t=time.time())
        self.status = st

    def apply(self, reason: str = "") -> Tuple[bool, str]:
        """Blocking: make MoveIt's `gui_keepout/*` objects match the enabled boxes. Returns (ok, message)."""
        if not self.supported:
            return False, self.status["msg"]
        with self._lock:
            self._last_try = time.monotonic()
            sig = self.signature()
            desired, problems = self.resolved()
            skipped: List[str] = []
            if desired:
                pts, why = self.robot_points()
                if pts is None:
                    msg = f"not applied: {why}"
                    self._applied_sig = sig
                    if self.status.get("msg") != msg:
                        self.log(f"keep-out zones {msg}")
                    self._set(state="waiting", ok=False, msg=msg, retry=True, reason=reason)
                    return False, msg
                for oid, (b, T) in list(desired.items()):
                    inside, _, _ = self.conflicts(T, b.size_m())
                    if inside:
                        skipped.append(f"{b.name} (contains the {', '.join(inside)})")
                        del desired[oid]
            b = self.bridge
            known = b.scene_objects()
            ours = {i for i in (known or {}) if i.startswith(PREFIX)}
            to_remove = sorted((ours | self._applied_ids) - set(desired))
            errors: List[str] = []
            for oid in to_remove:
                ok, m = b.scene_remove(oid)
                if not ok:
                    errors.append(f"remove {oid}: {m}")
            for oid, (box, T) in desired.items():
                ok, m = b.scene_set_box(oid, box.size_m(), T)
                if not ok:
                    errors.append(f"{box.name}: {m}")
            verified, missing = False, []
            if not errors:
                for attempt in range(VERIFY_TRIES):
                    if attempt:
                        time.sleep(VERIFY_WAIT_S)
                    scene = b.scene_objects()
                    if scene is None:
                        missing = ["(planning scene could not be read back)"]
                        continue
                    missing = [oid for oid, (box, T) in desired.items() if not _matches(scene.get(oid), box, T)]
                    lingering = [oid for oid in to_remove if oid in scene]
                    if not missing and not lingering:
                        verified = True
                        break
                    # A publish can be lost while move_group's subscription is still being matched: send again.
                    for oid in missing:
                        if oid in desired:
                            b.scene_set_box(oid, desired[oid][0].size_m(), desired[oid][1])
                    for oid in lingering:
                        b.scene_remove(oid)
                    missing = missing + [f"{oid} (still present)" for oid in lingering]
            self._applied_ids = set(desired)
            self._applied_sig = sig
            names = [box.name for box, _ in desired.values()]
            notes = problems + [f"{s}: NOT applied" for s in skipped]
            if errors:
                ok, state = False, "error"
                msg = "failed: " + "; ".join(errors)
            elif not verified:
                ok, state = False, "unverified"
                msg = (f"sent {len(desired)} box(es), but MoveIt's planning scene does not show them: "
                       + ", ".join(missing))
            else:
                ok, state = not notes, ("ok" if not notes else "warning")
                msg = (f"{len(desired)} box(es) in MoveIt's planning scene" + (f": {', '.join(names)}" if names else "")
                       + (f"; removed {len(to_remove)}" if to_remove else ""))
            if notes:
                msg += ". " + "; ".join(notes)
            repeat = reason == "retry" and msg == self.status.get("msg")       # no log line every RETRY_S
            self._set(state=state, ok=ok, msg=msg, count=len(desired) if verified else 0, skipped=skipped,
                      problems=problems, verified=verified, reason=reason,
                      retry=bool(errors or not verified or skipped))
            if not repeat:
                self.log(f"keep-out zones ({reason or 'apply'}): {msg}")
            return ok, msg

    def view(self) -> dict:
        """Boxes with their base-frame pose (for the 3D view) and the apply status."""
        boxes = []
        for bx in list(self.store.boxes.values()):
            row = bx.view()
            try:
                xyz, rpy = tf.T_to_mm_deg(self.T_base_box(bx))
                row["base"] = {"xyz_mm": [round(v, 2) for v in xyz], "rpy_deg": [round(v, 3) for v in rpy]}
            except Exception as e:  # noqa: BLE001
                row["error"] = str(e) or type(e).__name__
            boxes.append(row)
        st = dict(self.status)
        st["applied"] = sorted(self._applied_ids) if st.get("verified") else []
        return {"boxes": boxes, "status": st, "prefix": PREFIX, "inside": list(self.inside)}


def _matches(obj: Optional[dict], box: KeepoutBox, T_base_box: np.ndarray) -> bool:
    """A box read back from the planning scene matches what we sent (size 1 mm, position 1 mm, if reported)."""
    if obj is None:
        return False
    size = obj.get("size")
    if size is not None and not np.allclose(size, box.size_m(), atol=1e-3):
        return False
    T = obj.get("T")
    if T is not None and np.linalg.norm(np.asarray(T)[:3, 3] - T_base_box[:3, 3]) > 1e-3:
        return False
    return True
