"""Robot profile: every robot-specific name, loaded from a YAML file.

Nothing in this module imports ROS, so it can be unit-tested anywhere.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import yaml


class ProfileError(ValueError):
    pass


class WaypointDict(dict):
    """The profile's named poses. "home" was renamed "up": either name finds the pose saved under the other
    (so old programs that say "home" keep working)."""
    _ALIAS = {"home": "up", "up": "home"}

    def _key(self, k):
        return k if (dict.__contains__(self, k) or k not in self._ALIAS) else self._ALIAS[k]

    def __getitem__(self, k):
        return dict.__getitem__(self, self._key(k))

    def __contains__(self, k):
        return dict.__contains__(self, self._key(k))

    def get(self, k, default=None):
        return dict.get(self, self._key(k), default)


@dataclass
class ControlMode:
    name: str
    label: str
    activate: List[str]
    supports_moveit: bool = False
    experimental: bool = False
    hidden: bool = False
    requires_ft: bool = False
    allow_activation: bool = True
    disabled_reason: str = ""
    command_interface: str = "position"       # position | velocity | effort | none
    command_source: str = "trajectory"        # trajectory | cartesian_reference | manual
    moveit_controller: str = ""
    direct_torque: bool = False
    chain: List[str] = field(default_factory=list)
    note: str = ""
    reference_check: dict = field(default_factory=dict)   # direct torque: how the reference is verified
    # ros2_control hardware components the mode's controllers read from (e.g. the ATI topic bridge). A
    # component that ros2_control deactivated after a read error is brought back before entering the mode.
    hardware: List[str] = field(default_factory=list)
    # Cartesian force target of this mode's controller (OPERATE > Force): topic, frame (the frame the vector is
    # sent in), point (the link the wrench acts about), params_node (optional, read for selected_axes).
    wrench: dict = field(default_factory=dict)
    # Chained position modes (admittance): start every MoveIt trajectory at the trajectory controller's REFERENCE,
    # not the measured joints. {state_topic: .../controller_state, max_shift_rad}. See ros_bridge.
    reference_start: dict = field(default_factory=dict)

    @property
    def domain(self) -> str:
        """'direct_torque' or 'position' (trajectory/position domain). Switching across domains
        crosses the hardware command-interface boundary."""
        return "direct_torque" if self.direct_torque else "position"


@dataclass
class Waypoint:
    name: str
    joints: Optional[List[float]] = None
    position: Optional[List[float]] = None      # x, y, z  [m] in moveit.base_link
    quat_xyzw: Optional[List[float]] = None

    def to_dict(self) -> dict:
        d: dict = {}
        if self.joints is not None:
            d["joints"] = [round(float(v), 6) for v in self.joints]
        if self.position is not None:
            d["position"] = [round(float(v), 6) for v in self.position]
            d["quat_xyzw"] = [round(float(v), 6) for v in (self.quat_xyzw or [0, 0, 0, 1])]
        return d


@dataclass
class RobotProfile:
    name: str
    display_name: str
    ur_type: str
    robot_ip: str
    joint_names: List[str]
    moveit: dict
    topics: dict
    services: dict
    modes: Dict[str, ControlMode]
    default_mode: str
    managed_controllers: List[str]
    stationary_joint_velocity: float
    freedrive: dict
    gripper: dict
    waypoints: Dict[str, Waypoint] = field(default_factory=dict)
    model_tcps: Dict[str, dict] = field(default_factory=dict)   # name -> {link, moveit_target}
    default_tcp: str = ""
    tf_frames: List[str] = field(default_factory=list)
    joint_state_timeout_s: float = 0.5
    ft_calibration: dict = field(default_factory=dict)
    example_fixtures: dict = field(default_factory=dict)   # name -> {parent, xyz_mm, rpy_deg, source}: previews / examples
    servo: dict = field(default_factory=dict)
    jog: dict = field(default_factory=dict)          # step sizes and hold-to-jog speeds (jog.DEFAULTS)
    force: dict = field(default_factory=dict)        # limits and ramps of OPERATE > Force (force.DEFAULTS)
    tf_timeout_s: float = 0.5
    cameras: List[dict] = field(default_factory=list)   # [{name, topic, info?}] expected image topics
    gains: dict = field(default_factory=dict)        # mode -> tunable controller gains (gains.py)
    ft_bias: dict = field(default_factory=dict)      # F/T software bias on the broadcaster (ft_bias.py)
    source_path: Optional[Path] = None

    @property
    def base_link(self) -> str:
        return self.moveit.get("base_link", "base_link")

    @property
    def planning_link(self) -> str:
        return self.moveit["planning_link"]

    @property
    def max_cart_speed_mm_s(self) -> float:
        """Upper limit of a linear move's TCP speed (moveit.max_cartesian_speed_mm_s, default 250)."""
        from .cart_speed import max_mm_s
        return max_mm_s(self)

    # ------------------------------------------------------------------ load
    @classmethod
    def load(cls, path: str | Path) -> "RobotProfile":
        path = Path(path)
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        prof = cls.from_dict(data)
        prof.source_path = path
        return prof

    @classmethod
    def from_dict(cls, d: dict) -> "RobotProfile":
        try:
            prefix = d.get("joint_prefix", "") or ""
            joints = [prefix + j for j in d["joint_names"]]
            cm = d.get("control_modes", {}) or {}
            modes: Dict[str, ControlMode] = {}
            for name, m in (cm.get("modes", {}) or {}).items():
                modes[name] = ControlMode(
                    name=name,
                    label=m.get("label", name.capitalize()),
                    activate=list(m.get("activate", [])),
                    supports_moveit=bool(m.get("supports_moveit", False)),
                    experimental=bool(m.get("experimental", False)),
                    hidden=bool(m.get("hidden", False)),
                    requires_ft=bool(m.get("requires_ft", False)),
                    allow_activation=bool(m.get("allow_activation", True)),
                    disabled_reason=str(m.get("disabled_reason", "")),
                    command_interface=str(m.get("command_interface", "position")),
                    command_source=str(m.get("command_source", "trajectory")),
                    moveit_controller=str(m.get("moveit_controller", "")),
                    direct_torque=bool(m.get("direct_torque", False)),
                    chain=list(m.get("chain", [])),
                    note=str(m.get("note", "")),
                    reference_check=dict(m.get("reference_check", {}) or {}),
                    hardware=[str(h) for h in (m.get("hardware") or [])],
                    wrench=dict(m.get("wrench") or {}),
                    reference_start=dict(m.get("reference_start") or {}),
                )
            managed = list(cm.get("managed", []))
            # Every controller a mode activates is implicitly managed.
            for m in modes.values():
                if m.direct_torque and m.supports_moveit:
                    raise ProfileError(f"mode '{m.name}': direct-torque modes cannot take MoveIt trajectories")
                if m.direct_torque != (m.command_interface == "effort"):
                    raise ProfileError(f"mode '{m.name}': direct_torque must match command_interface: effort")
                if m.direct_torque and not m.reference_check:
                    raise ProfileError(f"mode '{m.name}': a direct-torque mode needs reference_check, or torque "
                                       "could be generated from a stale pose error")
                if m.supports_moveit and m.moveit_controller and m.moveit_controller not in m.activate:
                    raise ProfileError(f"mode '{m.name}': moveit_controller is not one of its controllers")
                for c in m.activate:
                    if c not in managed:
                        managed.append(c)
            default_mode = cm.get("default", next(iter(modes), "position"))
            if modes and default_mode not in modes:
                raise ProfileError(f"default control mode '{default_mode}' is not defined")
            wps = WaypointDict()
            for wname, w in (d.get("waypoints", {}) or {}).items():
                wps[wname] = Waypoint(
                    name=wname,
                    joints=w.get("joints"),
                    position=w.get("position"),
                    quat_xyzw=w.get("quat_xyzw"),
                )
            moveit = dict(d.get("moveit", {}) or {})
            if "planning_link" not in moveit:          # v2.0 profiles called it end_effector
                moveit["planning_link"] = moveit.pop("end_effector", "tool0")
            moveit.pop("end_effector", None)
            tcps = d.get("tcps", {}) or {}
            model_tcps = {}
            for tname, t in (tcps.get("model_links", {}) or {}).items():
                model_tcps[tname] = {"link": t.get("link", tname),
                                     "moveit_target": bool(t.get("moveit_target", False))}
            planning = moveit["planning_link"]
            if not any(t["link"] == planning for t in model_tcps.values()):
                model_tcps[planning] = {"link": planning, "moveit_target": True}
            default_tcp = tcps.get("default", planning)
            if default_tcp not in model_tcps:
                raise ProfileError(f"tcps.default '{default_tcp}' is not one of tcps.model_links")
            st = d.get("state", {}) or {}
            prof = cls(
                name=d["name"],
                display_name=d.get("display_name", d["name"]),
                ur_type=d.get("ur_type", ""),
                robot_ip=str(d.get("robot_ip", "")),
                joint_names=joints,
                moveit=moveit,
                topics=d.get("topics", {}) or {},
                services=d.get("services", {}) or {},
                modes=modes,
                default_mode=default_mode,
                managed_controllers=managed,
                stationary_joint_velocity=float(cm.get("stationary_joint_velocity", 0.01)),
                freedrive=d.get("freedrive", {}) or {},
                gripper=d.get("gripper", {}) or {},
                waypoints=wps,
                model_tcps=model_tcps,
                default_tcp=default_tcp,
                tf_frames=list(d.get("tf_frames", []) or []),
                joint_state_timeout_s=float(st.get("joint_state_timeout_s", 0.5)),
                tf_timeout_s=float(st.get("tf_timeout_s", 0.5)),
                ft_calibration=d.get("ft_calibration", {}) or {},
                example_fixtures=d.get("example_fixtures", {}) or {},
                servo=d.get("servo", {}) or {},
                jog=d.get("jog", {}) or {},
                force=d.get("force", {}) or {},
                cameras=[c for c in (d.get("cameras") or []) if isinstance(c, dict) and c.get("topic")],
                gains=d.get("gains", {}) or {},
                ft_bias=d.get("ft_bias", {}) or {},
            )
        except KeyError as e:
            raise ProfileError(f"profile is missing required key {e}") from None
        for w in prof.waypoints.values():
            if w.joints is not None and len(w.joints) != len(prof.joint_names):
                raise ProfileError(
                    f"waypoint '{w.name}' has {len(w.joints)} joints, robot has {len(prof.joint_names)}")
        return prof

    def visible_modes(self) -> List[ControlMode]:
        return [m for m in self.modes.values() if not m.hidden]


def list_profiles(directory: str | Path) -> List[Path]:
    return sorted(Path(directory).glob("*.yaml"))


# ---------------------------------------------------------------- joint maps
def reorder_joints(msg_names: Sequence[str], msg_values: Sequence[float],
                   wanted: Sequence[str]) -> Optional[List[float]]:
    """Pick `wanted` joints out of a JointState-like message BY NAME.

    The UR driver does not publish joints in shoulder->wrist order and the
    gripper joint shares the same message, so indexing position[0..5] (what the
    old GUI did) records scrambled waypoints. Returns None if any wanted joint
    is missing or the message is malformed.
    """
    if len(msg_values) < len(msg_names):
        return None
    index = {n: i for i, n in enumerate(msg_names)}
    out = []
    for j in wanted:
        i = index.get(j)
        if i is None:
            return None
        v = float(msg_values[i])
        if not math.isfinite(v):
            return None
        out.append(v)
    return out
