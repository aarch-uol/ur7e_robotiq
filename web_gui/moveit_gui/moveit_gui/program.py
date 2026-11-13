"""Human-readable YAML robot programs (schema v2) and the runner that executes them.

Schema v2 (written by this version):

    version: 2
    name: scrape_vial
    robot: ur7e_lab
    waypoints:
      above_vial:
        pose: {frame: vial_rack, tcp: spatula_tip, position: [0.0, 0.0, 0.05], quat_xyzw: [1, 0, 0, 0]}
      park:
        joints: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    steps:
      - {type: controller, mode: position}
      - {type: movej, target: ready, velocity: 0.2}          # profile or program joint waypoint
      - {type: move_pose, target: above_vial, cartesian: false}
      - type: move_pose
        pose: {frame: base_link, tcp: robotiq_tcp, position: [0.42, 0.12, 0.31], quat_xyzw: [1, 0, 0, 0]}
      - {type: wait, seconds: 0.5}
      - type: loop                                           # like UR "Loop N times"; folder groups steps
        times: 3
        steps:
          - {type: gripper, command: close}
          - {type: comment, text: "anything"}
      # force work (Admittance / Impedance): a force stays applied across the following steps until force_off
      - {type: zero_ft}                                      # software F/T bias at the current load (not in Admittance)
      - {type: move_until_contact, direction: [0, 0, 1], frame: tool, distance_mm: 20, force_n: 3, velocity: 0.02}
      - {type: force, wrench: [0, 0, 5, 0, 0, 0], frame: tool}     # N / Nm the tool exerts, ramped in
      - {type: move_relative, offset_mm: [0, 0, -15], frame: vial, cartesian: true, velocity: 0.02}
      # speed_mm_s (optional, linear moves: move_pose with cartesian, move_relative, move_until_contact): the TCP
      # moves at this constant speed (mm/s, up to the profile's moveit.max_cartesian_speed_mm_s); velocity still applies
      - {type: move_relative, offset_mm: [0, 0, 5], frame: tool, speed_mm_s: 10}
      - {type: force_off}                                    # ramped out; also at the end, STOP or a failed step
      # recording and photos (nothing moves): a recording a program starts also ends with it
      - {type: record_start, preset: all, note: "trial 3"}   # preset and note are optional; no preset = saved selection
      - {type: photo, topic: /camera/camera/color/image_rect_raw, label: before}   # one JPEG, timestamp + label
      - {type: record_stop}
      # record and replay (Teach by hand, Path, Insert as Replay; replay.py): the joint motion as it was taught.
      # A planned MoveJ (velocity) to the first point, then ONE trajectory with the recorded timing / speed
      - {type: replay, name: "Taught path 14:02", speed: 1.0, velocity: 0.1, points: [[0.0, q1, .., q6], [0.012, ...], ...]}

Every Cartesian pose carries its reference frame and TCP (T_frame_tcp).
Lengths are metres, joint values radians, quaternions xyzw.

Schema v1 files still load. v1 poses had no frame/TCP; v1 *executed* them as the
pose of the MoveIt end-effector link in the base frame, so they are migrated as
frame = profile base_link, tcp = profile planning_link (not the active TCP,
which would move the robot somewhere else). The file on disk is not changed
until it is saved, which writes v2. Program.notes lists what was migrated.
"""
from __future__ import annotations

import enum
import math
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterator, List, Optional, Tuple

import yaml

from .cart_speed import DEFAULT_MAX_MM_S, SPEED_STEPS, check_mm_s
from .geometry import Geometry, GeometryError, PoseSpec
from .profile import RobotProfile

PROGRAM_VERSION = 2
STEP_TYPES = ("movej", "move_pose", "move_relative", "move_until_contact", "force", "force_off", "zero_ft",
              "gripper", "wait", "controller", "policy", "set_io", "speed", "loop", "folder", "comment",
              "photo", "record_start", "record_stop", "replay")
CONTAINER_STEPS = ("loop", "folder")
MOTION_STEPS = ("movej", "move_pose", "move_relative", "move_until_contact", "replay")
MAX_LOOP_TIMES = 10000
TITLES = {"movej": "MoveJ", "gripper": "Gripper", "wait": "Wait", "controller": "Control mode", "policy": "Policy",
          "set_io": "Set output", "speed": "Speed", "loop": "Loop", "folder": "Folder", "comment": "Comment",
          "move_relative": "Move relative", "move_until_contact": "Move until contact", "force": "Force",
          "force_off": "Force off", "zero_ft": "Zero F/T", "photo": "Photo",
          "record_start": "Record start", "record_stop": "Record stop", "replay": "Replay"}
AXIS_NAMES = ("X", "Y", "Z")
MAX_RELATIVE_MM = 300.0
MAX_CONTACT_MM = 200.0


class ProgramError(ValueError):
    pass


@dataclass
class ProgramWaypoint:
    name: str
    joints: Optional[List[float]] = None
    pose: Optional[PoseSpec] = None

    @classmethod
    def from_dict(cls, name: str, d: dict) -> "ProgramWaypoint":
        if not isinstance(d, dict):
            raise ProgramError(f"waypoint '{name}' must be a mapping")
        if ("joints" in d) == ("pose" in d):
            raise ProgramError(f"waypoint '{name}' needs exactly one of 'joints' or 'pose'")
        if "joints" in d:
            return cls(name, joints=[float(v) for v in d["joints"]])
        try:
            return cls(name, pose=PoseSpec.from_dict(d["pose"]))
        except GeometryError as e:
            raise ProgramError(f"waypoint '{name}': {e}") from None

    def to_dict(self) -> dict:
        if self.joints is not None:
            return {"joints": [round(float(v), 6) for v in self.joints]}
        return {"pose": self.pose.to_dict()}


@dataclass
class Step:
    type: str
    params: dict = field(default_factory=dict)
    children: List["Step"] = field(default_factory=list)     # loop / folder only

    @classmethod
    def from_dict(cls, d: dict) -> "Step":
        if not isinstance(d, dict):
            raise ProgramError("each step must be a mapping")
        d = dict(d)
        t = d.pop("type", None)
        if t not in STEP_TYPES:
            raise ProgramError(f"unknown step type '{t}' (allowed: {', '.join(STEP_TYPES)})")
        children = d.pop("steps", None) if t in CONTAINER_STEPS else None
        if children is not None and not isinstance(children, list):
            raise ProgramError(f"{t}: 'steps' must be a list")
        return cls(type=t, params=d, children=[Step.from_dict(c) for c in (children or [])])

    def to_dict(self) -> dict:
        d = {"type": self.type, **self.params}
        if self.container:
            d["steps"] = [c.to_dict() for c in self.children]
        return d

    @property
    def container(self) -> bool:
        return self.type in CONTAINER_STEPS

    @property
    def linear(self) -> bool:
        """A straight-line (MoveL) motion step: the ones speed_mm_s applies to."""
        if self.type == "move_pose":
            return bool(self.params.get("cartesian", False))
        if self.type == "move_relative":
            return bool(self.params.get("cartesian", True))
        return self.type == "move_until_contact"

    def speed_text(self) -> str:
        """'  20 mm/s' for a linear step with speed_mm_s, else ''."""
        v = self.params.get("speed_mm_s")
        if not self.linear or isinstance(v, bool) or not isinstance(v, (int, float)):
            return ""
        return f"  {float(v):g} mm/s"

    @property
    def title(self) -> str:
        if self.type == "move_pose":
            return "MoveL" if self.params.get("cartesian") else "MoveJ to pose"
        return TITLES.get(self.type, self.type)

    def summary(self) -> str:
        p = self.params
        v = f"  v={p['velocity']}" if "velocity" in p else ""
        if self.type == "movej":
            return f"{p.get('target') or 'joints'}{v}"
        if self.type == "move_pose":
            lin = " (linear)" if p.get("cartesian") else ""
            if "pose" in p and isinstance(p["pose"], dict):
                pose = p["pose"]
                return f"{pose.get('tcp')} @ {pose.get('frame')}{lin}{v}{self.speed_text()}"
            return f"{p.get('target', '?')}{lin}{v}{self.speed_text()}"
        if self.type == "gripper":
            what = p.get("command") or f"pos {p.get('position')}"
            eff = f"  {p['effort']} N" if "effort" in p else ""
            return f"{what}{eff}"
        if self.type == "wait":
            return f"{p.get('seconds', 0)} s"
        if self.type == "controller":
            return str(p.get("mode", "?")).upper()
        if self.type == "policy":
            d = f"  {p['duration']} s" if "duration" in p else ""
            return f"{p.get('policy', '?')}{d}"
        if self.type == "set_io":
            return f"DO{p.get('pin')} = {'ON' if p.get('state') else 'OFF'}"
        if self.type == "speed":
            return f"{int(float(p.get('fraction', 1.0)) * 100)} %"
        if self.type == "loop":
            return f"{p.get('times', 1)} times · {len(self.children)} step{'s' * (len(self.children) != 1)}"
        if self.type == "folder":
            return f"{p.get('name', '')} · {len(self.children)} step{'s' * (len(self.children) != 1)}"
        if self.type == "comment":
            return str(p.get("text", ""))
        if self.type == "move_relative":
            off = ", ".join(f"{float(x):g}" for x in p.get("offset_mm", [0, 0, 0]))
            rot = p.get("rotate_deg")
            r = f" rot [{', '.join(f'{float(x):g}' for x in rot)}]°" if rot and any(float(x) for x in rot) else ""
            if r and p.get("pivot") == "frame" and p.get("frame", "tool") != "tool":
                r += " about its origin"
            return (f"[{off}] mm{r} in {p.get('frame', 'tool')}{' (linear)' if p.get('cartesian', True) else ''}{v}"
                    f"{self.speed_text()}")
        if self.type == "move_until_contact":
            d = p.get("direction", [0, 0, 1])
            return (f"along [{', '.join(f'{float(x):g}' for x in d)}] {p.get('frame', 'tool')}, "
                    f"≤ {p.get('distance_mm', 0)} mm until {p.get('force_n', 0)} N{v}{self.speed_text()}")
        if self.type == "force":
            w = p.get("wrench", [0] * 6)
            tq = f" T [{', '.join(f'{float(x):g}' for x in w[3:])}] Nm" if any(float(x) for x in w[3:]) else ""
            nw = " · no wait" if p.get("wait", True) is False else ""
            return f"F [{', '.join(f'{float(x):g}' for x in w[:3])}] N{tq} · {p.get('frame', 'tool')}{nw}"
        if self.type == "force_off":
            return "ramp to zero"
        if self.type == "zero_ft":
            return "software bias at the current load"
        if self.type == "photo":
            lab = f" · {p['label']}" if p.get("label") else ""
            return f"{p.get('topic') or '?'}{lab}"
        if self.type == "record_start":
            return " · ".join(x for x in (p.get("preset") or "selected topics", p.get("note")) if x)
        if self.type == "record_stop":
            return "finish the recording"
        if self.type == "replay":
            from .replay import summary
            return summary(p)
        return ""


Path_ = List[int]


def walk(steps: List[Step], prefix: Tuple[int, ...] = ()) -> Iterator[Tuple[Tuple[int, ...], Step]]:
    """Every step in display order (pre-order) with its path."""
    for i, st in enumerate(steps):
        path = prefix + (i,)
        yield path, st
        if st.container:
            yield from walk(st.children, path)


def path_label(path) -> str:
    return ".".join(str(i + 1) for i in path)


def unroll(steps: List[Step], prefix: Tuple[int, ...] = ()) -> List[Tuple[Tuple[int, ...], Step]]:
    """The executable sequence: loops repeated, folders inlined, comments dropped."""
    return [(path, st) for path, st, _ in unroll_iters(steps, prefix)]


def unroll_iters(steps: List[Step], prefix: Tuple[int, ...] = (), iters: tuple = ()) -> list:
    """unroll(), each entry with the loops around it: ((loop path, iteration 1.., times), ...), outermost first."""
    out: list = []
    for i, st in enumerate(steps):
        path = prefix + (i,)
        if st.type == "comment":
            continue
        if st.type == "loop":
            times = max(0, min(int(st.params.get("times", 1)), MAX_LOOP_TIMES))
            for k in range(times):
                out.extend(unroll_iters(st.children, path, iters + ((path, k + 1, times),)))
        elif st.container:
            out.extend(unroll_iters(st.children, path, iters))
        else:
            out.append((path, st, iters))
    return out


def _migrate_v1(d: dict, profile: RobotProfile, notes: List[str]) -> dict:
    """v1 -> v2 in memory. v1 poses meant: planning link in the base frame."""
    base, link = profile.base_link, profile.planning_link
    out = {k: v for k, v in d.items() if k not in ("waypoints", "steps", "version")}
    wps = {}
    for name, w in (d.get("waypoints") or {}).items():
        if isinstance(w, dict) and "position" in w:
            wps[name] = {"pose": {"frame": base, "tcp": link, "position": w["position"],
                                  "quat_xyzw": w.get("quat_xyzw", [0, 0, 0, 1])}}
            notes.append(f"waypoint '{name}': v1 pose interpreted as {link} in {base}")
        else:
            wps[name] = w
    steps = []
    for i, st in enumerate(d.get("steps") or [], 1):
        st = dict(st) if isinstance(st, dict) else st
        if isinstance(st, dict) and st.get("type") == "movep":
            st["type"] = "move_pose"
            if "position" in st:
                st["pose"] = {"frame": base, "tcp": link, "position": st.pop("position"),
                              "quat_xyzw": st.pop("quat_xyzw", [0, 0, 0, 1])}
                notes.append(f"step {i}: v1 inline pose interpreted as {link} in {base}")
        steps.append(st)
    out["waypoints"], out["steps"] = wps, steps
    if notes:
        notes.insert(0, "loaded a v1 program; it will be saved as v2")
    return out


@dataclass
class Program:
    name: str = "untitled"
    robot: str = ""
    steps: List[Step] = field(default_factory=list)
    waypoints: Dict[str, ProgramWaypoint] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)     # migration notices, not saved

    @classmethod
    def from_dict(cls, d: dict, profile: RobotProfile) -> "Program":
        if not isinstance(d, dict):
            raise ProgramError("program file must be a mapping")
        version = int(d.get("version", 1))
        notes: List[str] = []
        if version == 1:
            d = _migrate_v1(d, profile, notes)
        elif version != PROGRAM_VERSION:
            raise ProgramError(f"unsupported program version {version}")
        wps = {n: ProgramWaypoint.from_dict(n, w) for n, w in (d.get("waypoints") or {}).items()}
        steps = [Step.from_dict(s) for s in (d.get("steps") or [])]
        return cls(name=d.get("name", "untitled"), robot=d.get("robot", ""), steps=steps,
                   waypoints=wps, notes=notes)

    @classmethod
    def load(cls, path: str | Path, profile: RobotProfile) -> "Program":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(yaml.safe_load(f) or {}, profile)

    def to_dict(self) -> dict:
        d: dict = {"version": PROGRAM_VERSION, "name": self.name, "robot": self.robot}
        if self.waypoints:
            d["waypoints"] = {n: w.to_dict() for n, w in self.waypoints.items()}
        d["steps"] = [s.to_dict() for s in self.steps]
        return d

    def save(self, path: str | Path) -> None:
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False, default_flow_style=None)
        self.notes = []

    # ------------------------------------------------------------------ tree
    def siblings(self, path) -> List[Step]:
        """The list that holds the step at `path`."""
        path = list(path)
        if not path:
            raise ProgramError("empty step path")
        lst = self.steps
        for i in path[:-1]:
            if not 0 <= i < len(lst) or not lst[i].container:
                raise ProgramError(f"no container at step {path_label(path[:-1])}")
            lst = lst[i].children
        return lst

    def step_at(self, path) -> Step:
        lst = self.siblings(path)
        i = list(path)[-1]
        if not 0 <= i < len(lst):
            raise ProgramError(f"no step {path_label(path)}")
        return lst[i]

    def all_steps(self) -> List[Step]:
        return [st for _, st in walk(self.steps)]

    # ------------------------------------------------------------ resolution
    def joint_target(self, step: Step, profile: RobotProfile) -> List[float]:
        p = step.params
        if "target" in p:
            name = p["target"]
            if name in self.waypoints:
                w = self.waypoints[name]
                if w.joints is None:
                    raise ProgramError(f"waypoint '{name}' is a pose, movej needs joints")
                return w.joints
            if name in profile.waypoints and profile.waypoints[name].joints is not None:
                return profile.waypoints[name].joints
            raise ProgramError(f"unknown joint waypoint '{name}'")
        if "joints" in p:
            return [float(v) for v in p["joints"]]
        raise ProgramError("movej needs 'target' or 'joints'")

    def pose_target(self, step: Step) -> PoseSpec:
        p = step.params
        try:
            if "pose" in p:
                return PoseSpec.from_dict(p["pose"])
        except GeometryError as e:
            raise ProgramError(str(e)) from None
        if "target" in p:
            w = self.waypoints.get(p["target"])
            if w is None or w.pose is None:
                raise ProgramError(f"unknown pose waypoint '{p['target']}'")
            return w.pose
        raise ProgramError("move_pose needs 'pose' or 'target'")

    # ------------------------------------------------------------ validation
    def validate(self, profile: RobotProfile, geometry: Optional[Geometry] = None,
                 policies=None, live: bool = False, bridge=None) -> List[str]:
        """Problems that would stop the program, as sentences (empty = OK).

        Static checks always; with live=True also checks that frames resolve now,
        the gripper is up, and policies pass their preflight (files exist).
        """
        problems = []
        n = len(profile.joint_names)
        if self.robot and self.robot != profile.name:
            problems.append(f"program was written for robot '{self.robot}', current profile is '{profile.name}'")
        snap = bridge.snapshot() if (live and bridge is not None) else None
        for path, s in walk(self.steps):
            i = path_label(path)
            p = s.params
            if p.get("on_fail", "stop") not in ("stop", "continue") or ("on_fail" in p and s.type not in MOTION_STEPS):
                problems.append(f"step {i}: on_fail is stop or continue, on motion steps only")
            try:
                if s.type == "movej":
                    q = self.joint_target(s, profile)
                    if len(q) != n:
                        problems.append(f"step {i}: movej needs {n} joint values, got {len(q)}")
                elif s.type == "move_pose":
                    pose = self.pose_target(s)
                    try:
                        pose.T()
                    except Exception as e:
                        problems.append(f"step {i}: malformed pose ({e})")
                    if geometry is not None:
                        for msg in geometry.static_problems(pose.frame, pose.tcp):
                            problems.append(f"step {i}: {msg}")
                        if live:
                            for msg in geometry.live_problems(pose.frame, pose.tcp):
                                problems.append(f"step {i}: {msg}")
                elif s.type == "controller":
                    m = profile.modes.get(p.get("mode"))
                    if m is None:
                        problems.append(f"step {i}: unknown control mode '{p.get('mode')}'")
                    elif not m.allow_activation:
                        problems.append(f"step {i}: {m.label} is disabled ({m.disabled_reason})")
                elif s.type == "gripper":
                    if not profile.gripper.get("enabled", False):
                        problems.append(f"step {i}: this robot profile has no gripper")
                    elif snap is not None and not snap.connected.get("Gripper"):
                        problems.append(f"step {i}: gripper action server is not available")
                    if p.get("command") not in ("open", "close") and "position" not in p:
                        problems.append(f"step {i}: gripper needs command open/close or position")
                elif s.type == "wait":
                    if float(p.get("seconds", -1)) < 0:
                        problems.append(f"step {i}: wait needs seconds >= 0")
                elif s.type == "policy":
                    if policies is not None:
                        m = policies.manifests.get(p.get("policy"))
                        if m is None:
                            problems.append(f"step {i}: unknown policy '{p.get('policy')}'")
                        else:
                            # Mode is checked when the step runs: an earlier controller step may set it.
                            check = getattr(policies, "program_problems", None)
                            for msg in (check(m) if check and live else m.preflight(profile.modes)):
                                problems.append(f"step {i}: policy {m.name}: {msg}")
                elif s.type == "speed":
                    if not 0.0 < float(p.get("fraction", 0)) <= 1.0:
                        problems.append(f"step {i}: speed fraction must be in (0, 1]")
                elif s.type == "set_io":
                    if "pin" not in p or "state" not in p:
                        problems.append(f"step {i}: set_io needs pin and state")
                elif s.type == "move_relative":
                    off = [float(x) for x in p.get("offset_mm", [0, 0, 0])]
                    rot = [float(x) for x in p.get("rotate_deg", [0, 0, 0])]
                    if len(off) != 3 or len(rot) != 3:
                        problems.append(f"step {i}: move_relative needs offset_mm [x, y, z] and rotate_deg [rx, ry, rz]")
                    elif sum(x * x for x in off) ** 0.5 > MAX_RELATIVE_MM:
                        problems.append(f"step {i}: move_relative offset over {MAX_RELATIVE_MM:g} mm")
                    elif not any(off) and not any(rot):
                        problems.append(f"step {i}: move_relative with no offset")
                    if p.get("pivot", "tcp") not in ("tcp", "frame"):
                        problems.append(f"step {i}: move_relative pivot is tcp (turn in place) or frame (about its origin)")
                    problems.extend(f"step {i}: {m}" for m in _frame_problems(p.get("frame", "tool"), geometry, live))
                elif s.type == "move_until_contact":
                    d = [float(x) for x in p.get("direction", [])]
                    if len(d) != 3 or sum(x * x for x in d) < 1e-9:
                        problems.append(f"step {i}: move_until_contact needs a direction [x, y, z]")
                    if not 0 < float(p.get("distance_mm", 0)) <= MAX_CONTACT_MM:
                        problems.append(f"step {i}: distance_mm must be in (0, {MAX_CONTACT_MM:g}]")
                    if not 0 < float(p.get("force_n", 0)) <= 50:
                        problems.append(f"step {i}: force_n (the contact threshold) must be in (0, 50] N")
                    if float(p.get("velocity", 0.02)) > 0.1:
                        problems.append(f"step {i}: move_until_contact velocity is at most 0.1 (it has to stop on contact)")
                    problems.extend(f"step {i}: {m}" for m in _frame_problems(p.get("frame", "tool"), geometry, live))
                elif s.type == "force":
                    w = [float(x) for x in p.get("wrench", [])]
                    if len(w) != 6:
                        problems.append(f"step {i}: force needs wrench [Fx, Fy, Fz, Tx, Ty, Tz]")
                    if p.get("frame", "tool") not in ("tool", "base"):
                        problems.append(f"step {i}: force frame is tool or base")
                    if not any(m.wrench.get("topic") for m in profile.modes.values()):
                        problems.append(f"step {i}: no control mode of this robot takes a force")
                    lim = profile.force or {}
                    if len(w) == 6:
                        fn, tn = sum(x * x for x in w[:3]) ** 0.5, sum(x * x for x in w[3:]) ** 0.5
                        if fn > float(lim.get("max_force_n", 20.0)) + 1e-9:
                            problems.append(f"step {i}: force {fn:.1f} N over the profile limit "
                                            f"{float(lim.get('max_force_n', 20.0)):g} N")
                        if tn > float(lim.get("max_torque_nm", 2.0)) + 1e-9:
                            problems.append(f"step {i}: torque {tn:.2f} Nm over the profile limit "
                                            f"{float(lim.get('max_torque_nm', 2.0)):g} Nm")
                elif s.type == "photo":
                    topic = p.get("topic")
                    if not isinstance(topic, str) or not topic.startswith("/"):
                        problems.append(f"step {i}: photo needs a camera topic (starting with /)")
                    elif live and bridge is not None and hasattr(bridge, "image_topics"):
                        try:
                            if topic not in {t for t, _ in bridge.image_topics()}:
                                problems.append(f"step {i}: no image is published on {topic}")
                        except Exception:  # noqa: BLE001 - a graph query that failed: the step reports it
                            pass
                elif s.type == "record_start":
                    from .recorder import PRESETS
                    if p.get("preset") and p["preset"] not in PRESETS:
                        problems.append(f"step {i}: unknown recording preset '{p['preset']}'")
                    if len(str(p.get("note", ""))) > 200:
                        problems.append(f"step {i}: the note is limited to 200 characters")
                elif s.type == "replay":
                    from .replay import ReplayError, parse, speed_of
                    try:
                        parse(p, n)
                        speed_of(p)
                    except ReplayError as e:
                        problems.append(f"step {i}: {e}")
                elif s.type == "loop":
                    t = p.get("times", 1)
                    if isinstance(t, bool) or int(t) != float(t) or not 1 <= int(t) <= MAX_LOOP_TIMES:
                        problems.append(f"step {i}: loop times must be a whole number 1..{MAX_LOOP_TIMES}")
                    if not s.children:
                        problems.append(f"step {i}: empty loop")
                for k in ("velocity", "acceleration"):
                    if k in p and not 0.0 < float(p[k]) <= 1.0:
                        problems.append(f"step {i}: {k} scaling must be in (0, 1]")
                if "speed_mm_s" in p:
                    if s.type not in SPEED_STEPS:
                        problems.append(f"step {i}: speed_mm_s is for linear moves (MoveL, move relative, until contact)")
                    else:
                        try:
                            check_mm_s(p["speed_mm_s"], getattr(profile, "max_cart_speed_mm_s", DEFAULT_MAX_MM_S))
                        except ValueError as e:
                            problems.append(f"step {i}: {e}")
            except ProgramError as e:
                problems.append(f"step {i}: {e}")
            except (TypeError, ValueError) as e:
                problems.append(f"step {i}: bad value ({e})")
        return _group_missing_frames(problems, profile)


def _group_missing_frames(problems: List[str], profile: RobotProfile) -> List[str]:
    """A fixture that is not taught yet fails every step that uses it, twice: say it once, with what to do."""
    missing = list(dict.fromkeys(re.findall(r"unknown frame '([^']+)'", " ".join(problems))))
    if not missing:
        return problems
    out, steps = [], {f: [] for f in missing}
    for msg in problems:
        m = re.match(r"step ([^:]+): ", msg)
        f = next((f for f in missing if f"'{f}'" in msg or f"-> {f} " in msg), None)
        if m and f:
            if m.group(1) not in steps[f]:
                steps[f].append(m.group(1))
        else:
            out.append(msg)
    examples = getattr(profile, "example_fixtures", None) or {}
    for f in missing:
        if (examples.get(f) or {}).get("kind") == "flask" or f == "flask":
            how = "SETUP > Flask: touch your flask and Save"
        else:
            how = "SETUP > Vial Geometry: teach it and Save" + (
                ", or pick one in Vial profiles (baseline = the example position, only if yours is really there)"
                if f in examples else "")
        out.insert(0, f"fixture '{f}' is not taught yet (used by step {', '.join(steps[f])}): {how}")
    return out


def _frame_problems(frame: str, geometry: Optional[Geometry], live: bool) -> List[str]:
    """A relative / contact move's axes: tool, base or a frame the GUI can resolve now (live)."""
    if frame in ("tool", "base") or geometry is None or not live:
        return []
    try:
        geometry.T_base_frame(frame)
        return []
    except Exception as e:
        return [f"frame '{frame}' is not available now ({e})"]


class ProgramHooks:
    """What the runner needs from the GUI core for force work and relative moves. The default does nothing
    useful: a program with force steps fails clearly when no core is attached (tests, tools)."""

    def active_tcp(self) -> str:
        raise ProgramError("no TCP available")

    def wrench(self) -> Optional[List[float]]:
        return None

    def force_apply(self, wrench: List[float], frame: str) -> Tuple[bool, str]:
        return False, "force steps need the GUI core"

    def force_progress(self) -> Optional[Tuple[List[float], List[float]]]:
        """(current, target) of the applied force, None if none is applied."""
        return None

    def force_release(self, wait_s: float) -> Tuple[bool, str]:
        return True, "no force applied"

    def force_stop(self, reason: str):
        pass

    def zero_ft(self, stopped: Callable[[], bool]) -> Tuple[bool, str]:
        return False, "zero_ft needs the GUI core"

    def record_start(self, presets, name: str, note: str) -> Tuple[bool, str]:
        return False, "recording needs the GUI core"

    def record_stop(self, reason: str, wait: bool = False) -> Tuple[bool, str]:
        """Finish the recording; wait=True returns when the bag is closed."""
        return True, "not recording"

    def recording(self) -> Optional[str]:
        """The folder of the recording running now, else None."""
        return None

    def photo(self, topic: str, label: str) -> Tuple[bool, str]:
        return False, "photo steps need the GUI core"

    def replay_check(self, names, Q):
        """(problems, joint velocity limits, joint acceleration limits) of a replay (replay.check)."""
        return ["replay steps need the GUI core (robot model, keep-out zones)"], None, None


# =================================================================== runner
class RunState(enum.Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPING = "stopping"


class ProgramRunner:
    """Executes a Program step by step in its own thread.

    pause(): motion steps are cancelled and repeated on resume (going to a
             target is idempotent); waits keep their remaining time; gripper,
             I/O and policy steps finish and the pause takes effect after them.
    stop():  cancels motion and any running policy, then ends the run.
    step mode: run one step, then pause.
    """

    def __init__(self, bridge, profile: RobotProfile, modes, geometry: Geometry, policies=None,
                 on_step: Callable[[int], None] = lambda i: None,
                 on_state: Callable[[RunState], None] = lambda s: None,
                 on_log: Callable[[str], None] = print,
                 on_done: Callable[[bool, str], None] = lambda ok, msg: None):
        self.bridge, self.profile, self.modes, self.geometry = bridge, profile, modes, geometry
        self.policies = policies
        self.on_step, self.on_state, self.on_log, self.on_done = on_step, on_state, on_log, on_done
        self.speed_override: Callable[[], float] = lambda: 1.0     # the GUI speed override (core sets it)
        self.robot_stopped: Callable[[], Optional[str]] = lambda: None  # why the robot takes no motion (core sets it)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._resume = threading.Event()
        self._resume.set()
        self._state = RunState.IDLE
        self._step_mode = False
        self._current_type: Optional[str] = None
        self.current_path: Optional[Tuple[int, ...]] = None   # the step executing now (nested programs)
        self.hooks: ProgramHooks = ProgramHooks()               # force / F/T / TCP access (core sets it)
        self.last_contact: Optional[dict] = None                # what the last move_until_contact found
        self._force_on = False                                  # a force step applied a force; force_off ends it
        self._step_memo: dict = {}                              # targets of the step now: kept over a pause
        self._force_params = None                               # (wrench, frame) of the force applied now
        self._rec_owned = False                                 # a record_start step of THIS run began the recording
        self._was_paused = False                                # paused since the last resume check
        self._fail_msg = ""
        self.skipped: List[str] = []
        self.loop_progress: list = []                           # [(loop label, iteration, times)] of the step now
        self.pause_reason = ""                                  # why the run paused by itself ("" = by the user)

    # ---------------------------------------------------------------- control
    @property
    def state(self) -> RunState:
        return self._state

    def _set_state(self, s: RunState):
        self._state = s
        self.on_state(s)

    def start(self, program: Program, start_index=0, loop: bool = False,
              step_mode: bool = False) -> bool:
        """start_index: a top-level index or a step path (list); the run starts at that step."""
        if self._thread and self._thread.is_alive():
            if self._state != RunState.IDLE:
                return False
            self._thread.join(2.0)                 # the last run reported its end and is just finishing
            if self._thread.is_alive():
                return False
        self.steps_total = len(program.steps)
        problems = self.preflight(program, start_index)
        if problems:
            self.program_name, self.started_at = program.name, None
            self.on_done(False, "preflight failed:\n  " + "\n  ".join(problems))
            return False
        self.program_name, self.started_at = program.name, time.time()
        self._stop.clear()
        self._resume.set()
        self._step_mode = step_mode
        self._thread = threading.Thread(target=self._run, args=(program, start_index, loop),
                                        daemon=True, name="program-runner")
        self._thread.start()
        return True

    @staticmethod
    def _start_path(start) -> Tuple[int, ...]:
        return tuple(int(i) for i in start) if isinstance(start, (list, tuple)) else (int(start),)

    def preflight(self, program: Program, start_index=0) -> List[str]:
        """Full check before the first step moves anything."""
        top = self._start_path(start_index)[0]
        sub = Program(program.name, program.robot, program.steps[top:], program.waypoints)
        problems = sub.validate(self.profile, self.geometry, self.policies, live=True, bridge=self.bridge)
        if not problems:
            from .replay import ReplayError, parse
            for path, st in walk(sub.steps):
                if st.type == "replay":          # joint limits and keep-out zones: refused before anything moves
                    try:
                        _t, Q = parse(st.params, len(self.profile.joint_names))
                    except ReplayError:
                        continue
                    why, _v, _a = self.hooks.replay_check(list(self.profile.joint_names), Q)
                    problems += [f"step {path_label((path[0] + top,) + path[1:])}: {w}" for w in why]
        if self.policies is not None and self.policies.running:
            problems.append("a policy is already running")
        return problems

    def pause(self, reason: str = ""):
        if self._state != RunState.RUNNING:
            return
        self.pause_reason = reason
        self._was_paused = True
        self._resume.clear()
        self._set_state(RunState.PAUSED)
        if self._current_type in MOTION_STEPS:
            self.bridge.cancel_motion()

    def resume(self, step_mode: bool = False):
        if self._state == RunState.PAUSED:
            self.pause_reason = ""
            self._step_mode = step_mode
            self._set_state(RunState.RUNNING)
            self._resume.set()

    def stop(self):
        if self._state == RunState.IDLE:
            return
        self._set_state(RunState.STOPPING)
        self._stop.set()
        self._resume.set()
        self.bridge.cancel_motion()
        if self.policies:
            self.policies.stop()

    def join(self, timeout: Optional[float] = None):
        if self._thread:
            self._thread.join(timeout)

    def alive(self) -> bool:
        """The run thread is still there (it has reported its end once it is gone)."""
        return bool(self._thread and self._thread.is_alive())

    # ------------------------------------------------------------------- loop
    def _wait_if_paused(self) -> bool:
        """Block while paused. Returns False if stopped (or the force could not be re-applied: _fail_msg)."""
        while not self._resume.wait(0.05):
            if self._stop.is_set():
                return False
        if self._stop.is_set():
            return False
        if self._was_paused:
            self._was_paused = False
            if self._force_on and self._force_params and self.hooks.force_progress() is None:
                # the robot stopped while pressing and the force was zeroed: press again before going on
                ok_f, msg_f = self.hooks.force_apply(*self._force_params)
                self.on_log(f"force re-applied after the pause: {msg_f}")
                if not ok_f or not self._wait_force_ramp():
                    self._fail_msg = f"the force could not be re-applied after the pause: {msg_f}"
                    return False
        return True

    def _run(self, program: Program, start, loop: bool):
        self._force_on = False
        self._rec_owned = False
        self._step_memo, self.pause_reason, self._force_params = {}, "", None
        self._was_paused, self._fail_msg = False, ""
        self.skipped: List[str] = []           # steps that failed with on_fail: continue
        self._set_state(RunState.RUNNING)
        ok, msg = True, "finished"
        try:
            seq_it = unroll_iters(program.steps)
            seq = [(path, st) for path, st, _ in seq_it]
            first = self._start_path(start)
            # the first executable step at or after `first` (a container or comment starts at its first step)
            i = next((k for k, (path, _) in enumerate(seq) if path >= first), len(seq))
            while True:
                while i < len(seq):
                    if not self._wait_if_paused():
                        ok, msg = False, self._fail_msg or "stopped"
                        return
                    path, step = seq[i]
                    if self._step_memo.get("path") != path:
                        self._step_memo = {"path": path}   # a repeat after a pause keeps the step's targets
                    self.current_path = path
                    # e.g. "loop 9: 3 of 37" (outermost first): which scraping angle the run is on
                    self.loop_progress = [(path_label(lp), k, n) for lp, k, n in seq_it[i][2]]
                    self.on_step(path[0])
                    self._current_type = step.type
                    done = self._execute(program, step)
                    self._current_type = None
                    if done and self._force_on and self.hooks.force_progress() is None and not self._was_paused:
                        # e.g. the controller stopped or the mode changed: never carry on as if still pressing
                        ok, msg = False, (f"the force ended during step {path_label(path)} ({step.title}) "
                                          "without a force_off step")
                        return
                    if self._stop.is_set():
                        ok, msg = False, "stopped"
                        return
                    if not done and self._state == RunState.RUNNING and (step.type in MOTION_STEPS
                                                                         or step.type == "force"):
                        why = self.robot_stopped()
                        if why:               # the pendant paused or stopped the program: pause, do not fail
                            self.on_log(f"program paused at step {path_label(path)}: {why}")
                            self.pause(why)
                    if not done:
                        if self._state == RunState.PAUSED or self._was_paused:
                            continue          # paused mid-motion (maybe resumed already): repeat this step
                        if step.params.get("on_fail") == "continue" and step.type in MOTION_STEPS:
                            # e.g. one scraping angle out of reach: note it and go on (the Franka baseline did)
                            self.skipped.append(path_label(path))
                            self.on_log(f"step {path_label(path)} ({step.title}) failed: continuing (on failure: "
                                        f"continue); {len(self.skipped)} skipped so far")
                            i += 1
                            self._step_memo = {}
                            continue
                        ok, msg = False, (f"step {path_label(path)} ({step.title}) failed"
                                          + (f": {self._fail_msg}" if self._fail_msg else ""))
                        return
                    i += 1
                    self._step_memo = {}
                    if self._step_mode and i < len(seq):
                        self._resume.clear()
                        self._set_state(RunState.PAUSED)
                if not loop:
                    return
                i = 0
        except ProgramError as e:
            ok, msg = False, f"error: {e}"
        except Exception as e:      # never let the runner thread die silently
            ok, msg = False, f"error: {e}"
        finally:
            self.current_path = None
            self.loop_progress = []
            # a force never outlives the run: ramped out after a normal end, zeroed at once otherwise
            if self.hooks.force_progress() is not None:
                if ok:
                    rel_ok, rel_msg = self.hooks.force_release(5.0)
                    self.on_log(f"program ended with a force applied: {rel_msg}")
                else:
                    self.hooks.force_stop(f"program {msg}: zeroed at once")
            if self._rec_owned:                       # a recording this run started never outlives it
                self._rec_owned = False
                _ok, rec_msg = self.hooks.record_stop(f"the program ended ({'finished' if ok else (msg.splitlines() or [''])[0]})")
                self.on_log(f"program ended while recording: {rec_msg}")
            if ok and self.skipped:
                msg = (f"finished, {len(self.skipped)} step(s) failed and were skipped (on failure: continue): "
                       + ", ".join(self.skipped[:10]) + (" …" if len(self.skipped) > 10 else ""))
            self._set_state(RunState.IDLE)
            self.on_step(-1)
            self.on_done(ok, msg)

    # -------------------------------------------------------------- execution
    def _require_moveit_mode(self) -> bool:
        mode = self.modes.current_mode()
        if not self.modes.mode_supports_moveit(mode):
            self.on_log(f"motion step refused: control mode '{mode}' does not accept MoveIt trajectories")
            return False
        return True

    def _execute(self, program: Program, step: Step) -> bool:
        p = step.params
        t = step.type
        moveit = self.profile.moveit
        f = float(self.speed_override())
        vel = max(0.01, f * float(p.get("velocity", moveit.get("default_velocity_scaling", 0.1))))
        acc = max(0.01, f * float(p.get("acceleration", moveit.get("default_acceleration_scaling", 0.1))))

        if t == "movej":
            if not self._require_moveit_mode():
                return False
            return self.bridge.move_joints(list(program.joint_target(step, self.profile)), vel, acc)

        if t == "move_pose":
            if not self._require_moveit_mode():
                return False
            pose = program.pose_target(step)
            try:
                target = self.geometry.planning_target(pose)   # frame/TCP -> MoveIt link goal
            except GeometryError as e:
                self.on_log(f"move_pose refused: {e}")
                return False
            return self.bridge.move_pose(target.link, target.T_base_link, bool(p.get("cartesian", False)),
                                         vel, acc, cart_speed=self._cart_speed(step),
                                         speed_point=target.T_link_tcp[:3, 3])

        if t == "gripper":
            g = self.profile.gripper
            if p.get("command") == "open":
                pos = float(g.get("open_position", 0.0))
            elif p.get("command") == "close":
                pos = float(g.get("closed_position", 0.8))
            else:
                pos = float(p["position"])
            return self.bridge.gripper_move(pos, float(p.get("effort", g.get("max_effort", 100.0))))

        if t == "wait":
            remaining = float(p.get("seconds", 0.0))
            while remaining > 0:
                if self._stop.is_set():
                    return False
                if not self._resume.is_set():
                    if not self._wait_if_paused():
                        return False
                dt = min(0.05, remaining)
                time.sleep(dt)
                remaining -= dt
            return True

        if t == "controller":
            ok, msg = self.modes.switch_to(p["mode"], from_program=True)
            self.on_log(msg)
            return ok

        if t == "policy":
            if not self.policies:
                self.on_log("no policy library configured")
                return False
            # self.policies is the same PolicyExecutor AUTOMATION uses: one runtime path.
            return self.policies.run(p["policy"], self.modes, duration=p.get("duration"),
                                     stop_event=self._stop, log=self.on_log)

        if t == "move_relative":
            if not self._require_moveit_mode():
                return False
            return self._move_relative(p, vel, acc, self._cart_speed(step))

        if t == "move_until_contact":
            if not self._require_moveit_mode():
                return False
            return self._move_until_contact(p, vel, acc, self._cart_speed(step))

        if t == "replay":
            if not self._require_moveit_mode():
                return False
            return self._replay(p, vel, acc)

        if t == "force":
            ok, msg = self.hooks.force_apply([float(x) for x in p["wrench"]], p.get("frame", "tool"))
            self.on_log(f"force: {msg}")
            self._force_on = self._force_on or ok
            if ok:
                self._force_params = ([float(x) for x in p["wrench"]], p.get("frame", "tool"))
            # wait: false = set the target and go on while it ramps (a preload during the next move, as the
            # Franka baseline publishes its wrench and moves on); a Wait step after it gives it time
            return ok and (p.get("wait", True) is False or self._wait_force_ramp())

        if t == "force_off":
            ok, msg = self.hooks.force_release(float(p.get("timeout_s", 10.0)))
            self.on_log(f"force off: {msg}")
            self._force_on, self._force_params = False, None
            return ok

        if t == "zero_ft":
            ok, msg = self.hooks.zero_ft(self._stop.is_set)
            if not ok:                      # a success is logged by the F/T bias itself
                self.on_log(msg)
            return ok

        if t == "photo":
            label = str(p.get("label") or "") or "step" + path_label(self.current_path or (0,))
            ok, msg = self.hooks.photo(p["topic"], label)
            self.on_log(msg)
            return ok

        if t == "record_start":
            if self.hooks.recording():
                self.on_log("record_start: already recording, carrying on with that recording")
                return True
            ok, msg = self.hooks.record_start([p["preset"]] if p.get("preset") else None,
                                              str(program.name or ""), str(p.get("note") or ""))
            self.on_log(msg)
            self._rec_owned = self._rec_owned or ok
            return ok

        if t == "record_stop":
            if not self.hooks.recording():
                return True
            ok, msg = self.hooks.record_stop("stopped by the program", wait=True)
            self._rec_owned = False
            self.on_log(msg)
            return ok

        if t == "set_io":
            return self.bridge.set_digital_out(int(p["pin"]), bool(p["state"]))

        if t == "speed":
            return self.bridge.set_speed_slider(float(p["fraction"]))

        self.on_log(f"unhandled step type {t}")
        return False

    # ------------------------------------------------------------ replay
    def _replay(self, p: dict, vel: float, acc: float) -> bool:
        """A taught joint trajectory (replay.py): MoveJ to its first point, then one trajectory with the recorded
        timing / speed, slowed down if it would exceed the joint limits. After a pause: from where it stopped."""
        import numpy as np
        from . import replay as rp
        names = list(self.profile.joint_names)
        try:
            t, Q = rp.parse(p, len(names))
            speed = rp.speed_of(p) * max(0.01, float(self.speed_override()))
        except rp.ReplayError as e:
            self.on_log(f"replay refused: {e}")
            return False
        why, vlim, alim = self.hooks.replay_check(names, Q)
        if why:
            for w in why:
                self.on_log(f"replay refused: {w}")
            self._fail_msg = why[0]
            return False
        t_run, V, k = rp.fit_limits(t / speed, Q, np.asarray(vlim, float), np.asarray(alim, float))
        if k > 1.0:
            self.on_log(f"replay slowed x{k:.2f}: the taught motion at {speed * 100:.0f} % would exceed the joint "
                        "velocity / acceleration limits")
        k0 = 0
        snap = self.bridge.snapshot()
        if not snap.joints:
            self.on_log("replay refused: no joint state")
            return False
        memo = self._step_memo
        if "replay_elapsed" in memo:               # resumed after a pause: go on from about where it stopped
            k0 = rp.resume_index(memo["replay_t"], Q, memo["replay_k0"], memo["replay_elapsed"], snap.joints)
            if k0 >= len(Q) - 1:
                k0 = len(Q) - 1
            self.on_log(f"replay: going on from point {k0 + 1} of {len(Q)}")
        dev = float(np.max(np.abs(np.asarray(snap.joints, float) - Q[k0])))
        if dev > rp.START_TOL_RAD:
            self.on_log(f"replay: MoveJ to {'its start' if k0 == 0 else f'point {k0 + 1}'} "
                        f"({math.degrees(dev):.1f}° away)")
            if not self.bridge.move_joints([float(x) for x in Q[k0]], vel, acc):
                return False
            if self._stop.is_set() or self._state == RunState.PAUSED:
                return False
            q = self.bridge.snapshot().joints or []
            dev = float(np.max(np.abs(np.asarray(q, float) - Q[k0]))) if q else 1.0
            if dev > rp.START_TOL_RAD:
                self._fail_msg = f"the arm did not reach the replay's start ({math.degrees(dev):.2f}° off)"
                self.on_log(f"replay refused: {self._fail_msg}")
                return False
        if k0 >= len(Q) - 1:
            return True
        tt = t_run[k0:] - t_run[k0]
        VV = V[k0:].copy()
        VV[0] = 0.0                                 # from rest (a resumed replay too)
        self.on_log(f"replay: {len(tt)} points, {tt[-1]:.1f} s")
        memo.update(replay_t=t_run, replay_k0=k0)
        t_start = time.monotonic()
        ok = self.bridge.execute_joint_path(names, [float(x) for x in tt], Q[k0:].tolist(), VV.tolist())
        memo["replay_elapsed"] = time.monotonic() - t_start
        return bool(ok)

    # ------------------------------------------------------------ force work
    def _target_from(self, frame: str, offset_m, rot_deg, pivot: str = "tcp") -> PoseSpec:
        """The active TCP moved by offset (m) and rotated (deg) along `frame`'s axes: tool = the TCP's own axes,
        base = the robot base, anything else = that frame (e.g. a fixture). pivot "tcp" turns the tool in place;
        "frame" turns it ABOUT the frame's origin: with a vial fixture, rotate Z 10° steps around the vial axis
        to the next scraping angle, the tool facing the wall the same way (an orbit)."""
        import numpy as np
        from . import transforms as tf
        tcp = self.hooks.active_tcp()
        ref = self.profile.base_link if frame in ("tool", "base") else frame
        T = self.geometry.current_pose(ref, tcp).T()
        R_off = tf.mm_deg_to_T([0, 0, 0], [float(a) for a in rot_deg])[:3, :3]
        d = np.asarray(offset_m, float)
        out = T.copy()
        if frame == "tool":
            out[:3, 3] = T[:3, 3] + T[:3, :3] @ d
            out[:3, :3] = T[:3, :3] @ R_off
        elif pivot == "frame":
            out[:3, 3] = R_off @ T[:3, 3] + d
            out[:3, :3] = R_off @ T[:3, :3]
        else:
            out[:3, 3] = T[:3, 3] + d
            out[:3, :3] = R_off @ T[:3, :3]
        return PoseSpec.from_T(ref, tcp, out)

    def _cart_speed(self, step: Step) -> float:
        """The step's speed_mm_s as the bridge's cart_speed: m/s after the GUI speed override, 0.0 = none.
        Validated before the run; clamped here too, so a bad value never moves faster than the profile allows."""
        v = step.params.get("speed_mm_s")
        if not step.linear or isinstance(v, bool) or not isinstance(v, (int, float)) or not float(v) > 0:
            return 0.0
        return min(float(v), self.profile.max_cart_speed_mm_s) / 1000.0 * float(self.speed_override())

    def _move_to(self, pose: PoseSpec, cartesian: bool, vel: float, acc: float, cart_speed: float = 0.0) -> bool:
        try:
            target = self.geometry.planning_target(pose)
        except GeometryError as e:
            self.on_log(f"move refused: {e}")
            return False
        return self.bridge.move_pose(target.link, target.T_base_link, cartesian, vel, acc,
                                     cart_speed=cart_speed if cartesian else 0.0,
                                     speed_point=target.T_link_tcp[:3, 3])

    def _move_relative(self, p: dict, vel: float, acc: float, cart_speed: float = 0.0) -> bool:
        import numpy as np
        off = [float(x) / 1000.0 for x in p.get("offset_mm", [0, 0, 0])]
        if "pose" not in self._step_memo:      # from where the step STARTED: a resume after a pause finishes it
            pose = self._target_from(p.get("frame", "tool"), off, p.get("rotate_deg", [0, 0, 0]), p.get("pivot", "tcp"))
            # an orbit about a far origin travels far for a small angle: the same limit as a typed offset
            here = self._target_from(p.get("frame", "tool"), [0, 0, 0], [0, 0, 0])
            travel = float(np.linalg.norm(np.asarray(pose.position) - np.asarray(here.position))) * 1000.0
            if travel > MAX_RELATIVE_MM:
                self.on_log(f"move relative refused: it would move the TCP {travel:.0f} mm (limit {MAX_RELATIVE_MM:g} "
                            "mm). Rotating about a frame origin far from the tool? Use a fixture at the part")
                return False
            self._step_memo["pose"] = pose
        pose = self._step_memo["pose"]
        return self._move_to(pose, bool(p.get("cartesian", True)), vel, acc, cart_speed)

    def _move_until_contact(self, p: dict, vel: float, acc: float, cart_speed: float = 0.0) -> bool:
        """Linear move along `direction` for at most distance_mm; stops as soon as the measured force changes by
        force_n from its value at the start (|ΔF|). No contact within the distance fails the step unless
        on_no_contact: continue."""
        import numpy as np
        d = np.asarray([float(x) for x in p["direction"]], float)
        d = d / np.linalg.norm(d)
        dist = float(p["distance_mm"]) / 1000.0
        thr = float(p["force_n"])
        vel = min(vel, float(p.get("velocity", 0.02)) * float(self.speed_override()))
        if "f0" not in self._step_memo:        # baseline and end point from where the search STARTED
            base = []
            for _ in range(10):
                w = self.hooks.wrench()
                if w is None:
                    self.on_log("move until contact refused: no fresh F/T data (none, or older than 0.2 s)")
                    return False
                base.append(w[:3])
                time.sleep(0.01)
            self._step_memo["f0"] = np.mean(base, axis=0)
            self._step_memo["pose"] = self._target_from(p.get("frame", "tool"), d * dist, [0, 0, 0])
        f0, pose = self._step_memo["f0"], self._step_memo["pose"]
        hit = threading.Event()
        peak = [0.0]
        done = threading.Event()

        stale = threading.Event()

        def watch():
            t_ok = time.monotonic()
            while not done.is_set():
                w = self.hooks.wrench()          # None when there is no reading or it is older than 0.2 s
                if w is None and time.monotonic() - t_ok > 0.2:
                    stale.set()                  # a frozen sensor never shows the contact: stop driving
                    self.bridge.cancel_motion()
                    return
                if w is not None:
                    t_ok = time.monotonic()
                    df = float(np.linalg.norm(np.asarray(w[:3]) - f0))
                    peak[0] = max(peak[0], df)
                    if df >= thr:
                        hit.set()
                        self.bridge.cancel_motion()
                        return
                time.sleep(0.004)
        th = threading.Thread(target=watch, daemon=True, name="contact-watch")
        th.start()
        try:
            reached = self._move_to(pose, True, vel, acc, cart_speed)
        finally:
            done.set()
            th.join(1.0)
        self.last_contact = {"contact": hit.is_set(), "peak_n": round(peak[0], 2)}
        if stale.is_set() and not hit.is_set():
            self.on_log("move until contact aborted: the F/T reading stopped updating")
            return False
        if self._stop.is_set() or self._state == RunState.PAUSED:
            return False
        if hit.is_set():
            self.on_log(f"contact: |ΔF| reached {thr:g} N, stopped there")
            return True
        if reached and p.get("on_no_contact", "fail") == "continue":
            self.on_log(f"no contact within {p['distance_mm']} mm (peak |ΔF| {peak[0]:.1f} N): continuing")
            return True
        self.on_log(f"no contact within {p['distance_mm']} mm (peak |ΔF| {peak[0]:.1f} N)" if reached
                    else "move until contact: the move failed")
        return False

    def _wait_force_ramp(self, tol_n: float = 0.2) -> bool:
        """Block until the applied force reached its target (it ramps), so the next step starts loaded."""
        import numpy as np
        t_end = time.monotonic() + 30.0
        while time.monotonic() < t_end:
            if self._stop.is_set():
                return False
            pr = self.hooks.force_progress()
            if pr is None:
                self.on_log("force ended before it was reached")
                return False
            cur, goal = np.asarray(pr[0], float), np.asarray(pr[1], float)
            if np.linalg.norm(cur[:3] - goal[:3]) <= tol_n and np.linalg.norm(cur[3:] - goal[3:]) <= tol_n / 10:
                return True
            time.sleep(0.02)
        self.on_log("force did not reach its target within 30 s")
        return False
