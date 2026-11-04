"""PROGRAMS 3D preview: where the TCP goes, step by step, without moving the robot.

The waypoints alone do not show a program that works with relative moves: a scraping loop is one taught pose and
37 x (scrape down, up, orbit 10 deg). This walks the program from the robot's current joints the way the runner
would (loops repeated, folders inlined) and returns the TCP positions in the base frame, for a line in the 3D view.

Kinematics only: forward kinematics from the URDF for joint targets, the planned end pose for everything else.
It does not plan (a MoveJ draws a straight line between its ends, not the joint-space arc), and contact, forces
and failed moves are not simulated. Move until contact stops where it meets the wall or the floor of a vial of
known size (a fixture with a radius: taught by touch, from a vial profile, the example vial; or the profile's
example vial while it is assumed); anywhere
else it is drawn to its full distance.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from . import transforms as tf
from .program import MOTION_STEPS, Program, path_label, unroll, unroll_iters

MAX_POINTS = 4000


def _rot(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / (np.linalg.norm(a) or 1.0)
    x, y, z = a
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _origin(j) -> np.ndarray:
    T = tf.mm_deg_to_T([v * 1000.0 for v in j.get("xyz", [0, 0, 0])], [math.degrees(v) for v in j.get("rpy", [0, 0, 0])])
    return T


def fk(model: dict, q: Dict[str, float], base: str, link: str) -> Optional[np.ndarray]:
    """T_base_link from the parsed URDF (robot_model.parse_urdf) and joint values (rad / m) by name."""
    by_child = {j["child"]: j for j in model.get("joints", [])}
    chain, cur = [], link
    while cur != base:
        j = by_child.get(cur)
        if j is None:
            return None                       # link not below base
        chain.append(j)
        cur = j["parent"]
    T = np.eye(4)
    for j in reversed(chain):
        T = T @ _origin(j)
        v = q.get(j["name"], 0.0)
        if j.get("mimic"):
            m = j["mimic"]
            v = q.get(m["joint"], 0.0) * m["multiplier"] + m["offset"]
        M = np.eye(4)
        if j["type"] in ("revolute", "continuous"):
            M[:3, :3] = _rot(j["axis"], v)
        elif j["type"] == "prismatic":
            M[:3, 3] = np.asarray(j["axis"], float) * v
        T = T @ M
    return T


class _AssumedFrames:
    """The robot's geometry plus preview-only frames for fixtures that are not taught yet."""
    def __init__(self, geo, extra: Dict[str, np.ndarray]):
        self._geo, self.extra = geo, extra

    def T_base_frame(self, frame, *a, **k):
        return self.extra[frame] if frame in self.extra else self._geo.T_base_frame(frame, *a, **k)

    def __getattr__(self, n):
        return getattr(self._geo, n)


def _assume_missing(program: Program, geometry, T_tcp: np.ndarray, examples: Optional[dict] = None
                    ) -> Dict[str, np.ndarray]:
    """Untaught frames of the program's pose waypoints: the profile's example fixture (example_fixtures, in its
    parent frame) if there is one, else placed (base axes) so that the first waypoint in it is where the TCP is
    now. Preview only, never saved."""
    extra: Dict[str, np.ndarray] = {}
    for name, e in (examples or {}).items():
        try:
            geometry.T_base_frame(name)
            continue                                   # taught: the real one wins
        except Exception:
            pass
        try:
            T_par = geometry.T_base_frame(e.get("parent", "base_link")) if e.get("parent") not in (None, "base_link") \
                else np.eye(4)
            extra[name] = T_par @ tf.mm_deg_to_T(list(e.get("xyz_mm", [0, 0, 0])), list(e.get("rpy_deg", [0, 0, 0])))
        except Exception:
            continue
    for _, st in unroll(program.steps):
        if st.type != "move_pose":
            continue
        try:
            pose = program.pose_target(st)
            geometry.T_base_frame(pose.frame)
            continue
        except Exception:
            pass
        try:
            pose = program.pose_target(st)
        except Exception:
            continue
        if pose.frame in extra:
            continue
        T = np.eye(4)
        T[:3, 3] = T_tcp[:3, 3] - pose.T()[:3, 3]
        extra[pose.frame] = T
    return extra


def _contact_t(p: np.ndarray, d: np.ndarray, radius: float, height: Optional[float]) -> Optional[float]:
    """Distance along the unit direction d (vial frame) from p to the vial's inside (wall or floor), or None if p is
    not inside the vial or the ray leaves through the open top first."""
    r0 = math.hypot(p[0], p[1])
    if r0 >= radius or p[2] < 0.0 or (height is not None and p[2] > height + 0.01):
        return None
    hits = []
    a = d[0] ** 2 + d[1] ** 2
    if a > 1e-12:                                      # |p_xy + t d_xy| = radius, the positive root
        b = p[0] * d[0] + p[1] * d[1]
        t = (-b + math.sqrt(b * b - a * (r0 * r0 - radius * radius))) / a
        if height is None or p[2] + t * d[2] <= height + 0.01:
            hits.append(t)
    if d[2] < -1e-9:                                   # the floor (z = 0)
        hits.append(-p[2] / d[2])
    return min(hits) if hits else None


def _tcp_to_tcp(geometry, a: str, b: str) -> np.ndarray:
    """T_a_b between two TCPs on the same rigid tool (through the planning link)."""
    try:
        return tf.inv(geometry.T_planning_tcp(a)) @ geometry.T_planning_tcp(b)
    except Exception:
        return np.eye(4)                       # unknown TCP: the program check says so


def _pose7(T) -> List[float]:
    q = tf.matrix_to_quat(T[:3, :3])
    return [round(float(v) * 1000.0, 2) for v in T[:3, 3]] + [round(float(v), 6) for v in q]


def simulate(program: Program, profile, geometry, model: dict, joints: List[float], tcp: str,
             max_points: int = MAX_POINTS, assume_missing: bool = False,
             vials: Optional[Dict[str, tuple]] = None) -> dict:
    """{points: [[x, y, z] mm in base], poses: [[x, y, z mm, qx, qy, qz, qw]], joints: [q rad | None] (known for
    MoveJ targets), steps: [path label per point], what: [step title + summary], loops: [[[loop label, iteration, times]]], assumed: {frame: [x, y, z] mm}, note} from the current joints.
    vials: {fixture: (radius_m, height_m | None)}, where a Move until contact meets a wall (the example vial's
    size is added while it is assumed)."""
    names = list(profile.joint_names)
    link, _, T_link_tcp = geometry.tcp_anchor(tcp)
    base = profile.base_link

    def tcp_of(q) -> Optional[np.ndarray]:
        T = fk(model, dict(zip(names, q)), base, link)
        return None if T is None else T @ T_link_tcp

    T = tcp_of(joints)
    if T is None:
        return {"points": [], "steps": [], "note": f"no kinematic chain {base} -> {link} in the robot model"}
    assumed, examples = {}, set()
    if assume_missing:
        extra = _assume_missing(program, geometry, T, getattr(profile, "example_fixtures", None))
        examples = set(getattr(profile, "example_fixtures", None) or {}) & set(extra)
        if extra:
            geometry = _AssumedFrames(geometry, extra)
            vials = dict(vials or {})
            for f in examples:
                e = profile.example_fixtures[f]
                if e.get("radius_mm") and f not in vials:
                    vials[f] = (float(e["radius_mm"]) / 1000.0,
                                float(e["height_mm"]) / 1000.0 if e.get("height_mm") else None)
            assumed = {f: [round(float(v) * 1000.0, 1) for v in E[:3, 3]] for f, E in extra.items()}
    pts, labels, note = [T[:3, 3].tolist()], [""], ""
    hit = None

    def vf_txt(h):
        return f"{h[1]} wall / floor after {h[0] * 1000.0:.1f} mm"
    poses, qs, linear = [_pose7(T)], [list(joints)], [False]
    what, loops = ["start: where the robot is now"], [[]]     # per point, for stepping through the preview
    for path, st, iters in unroll_iters(program.steps):
        if st.type not in MOTION_STEPS:
            continue
        p = st.params
        try:
            if st.type == "movej":
                q = program.joint_target(st, profile)
                T2 = tcp_of(q)
            elif st.type == "move_pose":
                pose = program.pose_target(st)
                T2 = geometry.T_base_frame(pose.frame) @ pose.T()
                if pose.tcp != tcp:                    # the runner puts the pose's TCP there; draw the active one
                    T2 = T2 @ _tcp_to_tcp(geometry, pose.tcp, tcp)
            elif st.type == "move_relative":
                frame = p.get("frame", "tool")
                ref = np.eye(4) if frame in ("tool", "base") else geometry.T_base_frame(frame)
                Tr = np.linalg.inv(ref) @ T
                R_off = tf.mm_deg_to_T([0, 0, 0], [float(a) for a in p.get("rotate_deg", [0, 0, 0])])[:3, :3]
                d = np.asarray([float(x) / 1000.0 for x in p.get("offset_mm", [0, 0, 0])])
                out = Tr.copy()
                if frame == "tool":
                    out[:3, 3] = Tr[:3, 3] + Tr[:3, :3] @ d
                    out[:3, :3] = Tr[:3, :3] @ R_off
                elif p.get("pivot") == "frame":
                    out[:3, 3] = R_off @ Tr[:3, 3] + d
                    out[:3, :3] = R_off @ Tr[:3, :3]
                else:
                    out[:3, 3] = Tr[:3, 3] + d
                    out[:3, :3] = R_off @ Tr[:3, :3]
                T2 = ref @ out
            else:                              # move_until_contact: to a known vial's wall, else its full distance
                frame = p.get("frame", "tool")
                dvec = np.asarray([float(x) for x in p.get("direction", [0, 0, 1])], float)
                dvec = dvec / (np.linalg.norm(dvec) or 1.0)
                ref = np.eye(4) if frame in ("tool", "base") else geometry.T_base_frame(frame)
                u = T[:3, :3] @ dvec if frame == "tool" else ref[:3, :3] @ dvec          # unit, base frame
                dist = float(p.get("distance_mm", 0)) / 1000.0
                hit = None
                for vf, (rad, hgt) in (vials or {}).items():
                    try:
                        Tv = geometry.T_base_frame(vf)
                    except Exception:
                        continue
                    pv = (tf.inv(Tv) @ np.r_[T[:3, 3], 1.0])[:3]
                    t = _contact_t(pv, Tv[:3, :3].T @ u, float(rad), hgt)
                    if t is not None and t <= dist and (hit is None or t < hit[0]):
                        hit = (t, vf)
                T2 = T.copy()
                T2[:3, 3] = T[:3, 3] + u * (hit[0] if hit else dist)
        except Exception as e:                 # an unknown frame or waypoint: the program check says so
            note = f"preview stops at step {path_label(path)}: {e}"
            break
        if T2 is None:
            note = f"preview stops at step {path_label(path)}: no forward kinematics"
            break
        T = T2
        pts.append(T[:3, 3].tolist())
        poses.append(_pose7(T))
        qs.append(list(q) if st.type == "movej" else None)
        # how the arm gets there: MoveJ and a non-linear MovePose are planned in joint space, the rest are straight
        linear.append(st.type in ("move_relative", "move_until_contact") or
                      (st.type == "move_pose" and bool(p.get("cartesian", False))))
        labels.append(path_label(path))
        extra_what = ""
        if st.type == "move_until_contact":
            extra_what = (f"  (contact: {vf_txt(hit)})" if hit else "  (no known wall: drawn to its full distance)")
        what.append(f"{st.title}  {st.summary()}{extra_what}".strip())
        loops.append([[path_label(lp), k, n] for lp, k, n in iters])
        if len(pts) >= max_points:
            note = f"preview shows the first {max_points} moves"
            break
    if assumed and not note:
        ex = [f for f in assumed if f in examples]
        under = [f for f in assumed if f not in examples]
        note = "; ".join(x for x in (
            ("not taught yet, previewed at the profile's example position: " + ", ".join(
                f"{f} {assumed[f]} mm" for f in ex)) if ex else "",
            ("not taught yet, previewed as if under the tool now: " + ", ".join(under)) if under else "")
            if x) + " (SETUP > Vial Geometry to teach it)"
    zs = [p[2] for p in pts[1:]]
    if zs and min(zs) < 0.0:                   # below the robot base: on a table-mounted arm, into the table
        k = 1 + int(np.argmin(zs))
        below = (f"WARNING: the TCP goes below the robot base (z {min(zs) * 1000.0:.0f} mm at step {labels[k]}): "
                 f"check the TCP of the poses (a named TCP plus a longer active one ends up lower) and the fixture")
        note = below + ("; " + note if note else "")
    return {"points": [[round(v * 1000.0, 2) for v in p] for p in pts], "poses": poses, "joints": qs, "linear": linear,
            "steps": labels, "what": what, "loops": loops, "assumed": assumed, "note": note}
