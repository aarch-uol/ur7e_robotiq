"""Frames and TCPs: the one place where poses are converted.

A Cartesian pose in this GUI always means T_frame_tcp:
the pose of TCP `tcp` expressed in reference frame `frame`.

Frame resolution (T_base_frame), in this order:
  base_link                  -> identity
  fixture (calibration)      -> T_base_parent @ T_parent_fixture   (recursive)
  vision object (calibration)-> T_base_R @ T_correction @ T_R_obj_detected (live source)
  anything else              -> TF lookup (must be fresh, see bridge.lookup_transform)
A frame that cannot be resolved raises GeometryError. There is no fallback to base.

TCP handling:
  model link with moveit_target: true  -> MoveIt gets the pose for that link directly
  anything else (incl. virtual TCPs)   -> converted to the planning link:
        T_base_planning = T_base_tcp @ inv(T_planning_tcp)
Virtual TCPs change pose semantics only, not MoveIt collision geometry.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np

from . import transforms as tf
from .calibration import CalibrationStore, TcpDef


class GeometryError(RuntimeError):
    pass


# a pose whose tcp is this is reached by the TCP active at run time (UR style), not a named one
ACTIVE_TCP = "active"


@dataclass
class PoseSpec:
    """T_frame_tcp, as stored in programs (schema v2)."""
    frame: str
    tcp: str
    position: List[float]           # metres
    quat_xyzw: List[float]

    @classmethod
    def from_dict(cls, d: dict) -> "PoseSpec":
        if not isinstance(d, dict):
            raise GeometryError("pose must be a mapping with frame, tcp, position, quat_xyzw")
        missing = [k for k in ("frame", "tcp", "position", "quat_xyzw") if k not in d]
        if missing:
            raise GeometryError(f"pose is missing {', '.join(missing)}")
        return cls(str(d["frame"]), str(d["tcp"]), [float(v) for v in d["position"]],
                   [float(v) for v in d["quat_xyzw"]])

    def to_dict(self) -> dict:
        return {"frame": self.frame, "tcp": self.tcp,
                "position": [round(v, 6) + 0.0 for v in self.position],      # + 0.0: no "-0.0" in files
                "quat_xyzw": [round(v, 6) + 0.0 for v in self.quat_xyzw]}

    def T(self) -> np.ndarray:
        """T_frame_tcp. Raises TransformError for malformed numbers / non-unit quaternions."""
        return tf.make(self.position, tf.check_unit_quat(self.quat_xyzw))

    @classmethod
    def from_T(cls, frame: str, tcp: str, T_frame_tcp: np.ndarray) -> "PoseSpec":
        p, q = tf.to_pos_quat(T_frame_tcp)
        return cls(frame, tcp, p, q)


@dataclass
class PlanningTarget:
    link: str                       # link whose pose MoveIt should reach
    T_base_link: np.ndarray         # goal pose of that link in the base frame


# lookup(parent, child) -> T_parent_child, or None if unavailable / stale
Lookup = Callable[[str, str], Optional[np.ndarray]]
# read_pose_source(source_type, source, reference_frame) -> T_R_obj_detected or None
PoseSourceReader = Callable[[str, str, str], Optional[np.ndarray]]


class Geometry:
    def __init__(self, store: CalibrationStore, lookup: Lookup,
                 read_pose_source: Optional[PoseSourceReader] = None):
        self.store = store
        self.profile = store.profile
        self.lookup = lookup
        self.read_pose_source = read_pose_source

    @property
    def base(self) -> str:
        return self.profile.base_link

    # ------------------------------------------------------------- frames
    def frame_kind(self, frame: str) -> str:
        if frame == self.base:
            return "base"
        if frame in self.store.fixtures:
            return "fixture"
        if frame in self.store.vision:
            return "vision"
        return "tf"

    def _tf(self, parent: str, child: str) -> np.ndarray:
        T = self.lookup(parent, child)
        if T is None:
            raise GeometryError(f"transform {parent} -> {child} is unavailable or stale")
        return T

    def T_base_frame(self, frame: str, _seen: Optional[set] = None) -> np.ndarray:
        kind = self.frame_kind(frame)
        if kind == "base":
            return np.eye(4)
        if kind == "fixture":
            seen = _seen or set()
            if frame in seen:
                raise GeometryError(f"fixture '{frame}' has a cyclic parent chain")
            seen.add(frame)
            fx = self.store.fixtures[frame]
            return self.T_base_frame(fx.parent, seen) @ fx.T_parent_fixture
        if kind == "vision":
            vb = self.store.vision[frame]
            if self.read_pose_source is None:
                raise GeometryError(f"no pose source reader for vision object '{frame}'")
            T_R_det = self.read_pose_source(vb.source_type, vb.source, vb.reference_frame)
            if T_R_det is None:
                raise GeometryError(f"vision object '{frame}': no fresh detection from {vb.source}")
            return self.T_base_frame(vb.reference_frame) @ vb.corrected(T_R_det)
        return self._tf(self.base, frame)

    # --------------------------------------------------------------- TCPs
    def resolve_tcp(self, name: str) -> str:
        """ACTIVE_TCP ("active") in a pose means whichever TCP is active when the move runs."""
        return self.store.active_tcp if name == ACTIVE_TCP else name

    def tcp(self, name: str) -> TcpDef:
        name = self.resolve_tcp(name)
        t = self.store.tcps.get(name)
        if t is None:
            raise GeometryError(f"unknown TCP '{name}'")
        return t

    def tcp_anchor(self, name: str) -> Tuple[str, bool, np.ndarray]:
        """Walk virtual TCPs up to a real link. Returns (link, moveit_target, T_link_tcp)."""
        name = self.resolve_tcp(name)
        T_link_tcp = np.eye(4)
        cur, seen = self.tcp(name), set()
        while cur.type == "virtual":
            if cur.name in seen:
                raise GeometryError(f"TCP '{name}' has a cyclic parent chain")
            seen.add(cur.name)
            T_link_tcp = cur.T_parent_tcp @ T_link_tcp
            parent = self.store.tcps.get(cur.parent)
            if parent is None:
                return cur.parent, False, T_link_tcp       # parent is a plain TF link
            cur = parent
        return cur.link, cur.moveit_target, T_link_tcp

    def T_planning_tcp(self, name: str) -> np.ndarray:
        link, _, T_link_tcp = self.tcp_anchor(name)
        planning = self.profile.planning_link
        T_planning_link = np.eye(4) if link == planning else self._tf(planning, link)
        return T_planning_link @ T_link_tcp

    def current_T_base_tcp(self, name: str) -> np.ndarray:
        link, _, T_link_tcp = self.tcp_anchor(name)
        return self._tf(self.base, link) @ T_link_tcp

    # ------------------------------------------------------- public API
    def current_pose(self, frame: str, tcp: str) -> PoseSpec:
        """What 'Record pose' saves: T_frame_tcp right now."""
        T_frame_tcp = tf.inv(self.T_base_frame(frame)) @ self.current_T_base_tcp(tcp)
        return PoseSpec.from_T(frame, tcp, T_frame_tcp)

    def planning_target(self, pose: PoseSpec) -> PlanningTarget:
        """Convert a desired T_frame_tcp into a MoveIt goal (link + base-frame pose)."""
        try:
            T_frame_tcp = pose.T()
        except tf.TransformError as e:
            raise GeometryError(f"malformed pose: {e}") from None
        T_base_tcp = self.T_base_frame(pose.frame) @ T_frame_tcp
        link, moveit_ok, T_link_tcp = self.tcp_anchor(pose.tcp)
        if moveit_ok and np.allclose(T_link_tcp, np.eye(4)):
            return PlanningTarget(link, T_base_tcp)
        T_base_planning = T_base_tcp @ tf.inv(self.T_planning_tcp(pose.tcp))
        return PlanningTarget(self.profile.planning_link, T_base_planning)

    def static_problems(self, frame: str, tcp: str) -> List[str]:
        """Checks that need no robot: names exist. Used by program preflight."""
        out = []
        tcp = self.resolve_tcp(tcp)
        if tcp not in self.store.tcps:
            out.append(f"unknown TCP '{tcp}'")
        else:
            try:
                self.tcp_anchor(tcp)
            except GeometryError as e:
                out.append(str(e))
        if self.frame_kind(frame) == "tf" and frame not in self.profile.tf_frames:
            out.append(f"unknown frame '{frame}' (not base, a fixture, a vision object, or a profile tf_frame)")
        return out

    def live_problems(self, frame: str, tcp: str) -> List[str]:
        """Checks that the frame and TCP resolve right now (TF fresh, detection present)."""
        out = []
        try:
            self.T_base_frame(frame)
        except GeometryError as e:
            out.append(str(e))
        try:
            self.T_planning_tcp(tcp)
        except GeometryError as e:
            out.append(str(e))
        return out
