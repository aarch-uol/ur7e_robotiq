"""Parts: objects bolted to the robot or the cell (a spatula holder, a camera bracket, a table), without editing URDF.

A part is a simple shape (box, cylinder, sphere, a mesh (.stl / .obj / .dae), or none = a frame only) on a parent: a link of the
robot model or another part, at an offset in mm / deg. Parts stack: by default a shape sits on its frame and grows
along +Z (anchor "base"), and a part put on another part starts at that part's far face, so a tool is built like
the real one, piece on piece.

Where a part shows up:
  live (no restart)      TF frame parent -> part, a MoveIt collision object (attached to the arm link it rides
                          on, so plans carry it; or a world object for parts of the cell) and the 3D views
  after a stack restart  the generated URDF / SRDF (config_dir()/parts/generated/), included by the robot's xacro
                          when the launcher passes UR7E_USER_PARTS_URDF / _SRDF: robot_description, RViz,
                          recordings and MoveIt's self-collision model all have it

Once the running robot_description has a part (its joint gui_part_<name>_joint), the live copy is dropped: the
robot model wins. A part changed or deleted after that takes effect at the next restart; the GUI says which.

Saved in config_dir()/parts/<profile>.yaml (+ meshes/). Loaded with yaml.safe_load only.
"""
from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple
from xml.sax.saxutils import quoteattr

import numpy as np
import yaml

from . import meshes as M
from . import transforms as tf

NAME = re.compile(r"^[a-z][a-z0-9_]{0,39}$")
SHAPES = ("box", "cylinder", "sphere", "mesh", "none")
KINDS = ("part", "camera")
JOINT_PREFIX, JOINT_SUFFIX = "gui_part_", "_joint"
OBJECT_PREFIX = "gui_part/"
MESH_MAX_BYTES = M.MESH_MAX_BYTES
MESH_FILE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]*\.(stl|obj|dae)$", re.I)
COLLISIONS = ("visual", "box", "hull", "none")     # visual: the shape itself; box: its bounding box;
                                                   # hull: a mesh's convex hull (light for MoveIt); none: show only
GENERATED_URDF, GENERATED_SRDF = "robot_parts.urdf.xacro", "robot_parts.srdf.xacro"
POS_TOL_M, ANG_TOL_RAD = 1e-5, 1e-4


class PartError(ValueError):
    pass


def joint_name(part: str) -> str:
    return f"{JOINT_PREFIX}{part}{JOINT_SUFFIX}"


def part_of_joint(joint: str) -> Optional[str]:
    if joint.startswith(JOINT_PREFIX) and joint.endswith(JOINT_SUFFIX):
        return joint[len(JOINT_PREFIX):-len(JOINT_SUFFIX)]
    return None


# ===================================================================== meshes
def read_stl(path: Path) -> np.ndarray:
    """Triangles of any supported mesh file (kept under its old name). Raises PartError."""
    return read_mesh(path)


def read_mesh(path: Path) -> np.ndarray:
    """Triangles (n, 3, 3) of an .stl / .obj / .dae (a .dae in metres, the others in file units). Raises PartError."""
    try:
        return M.read_mesh(path)
    except M.MeshError as e:
        raise PartError(str(e)) from None


# ===================================================================== data
@dataclass
class Part:
    name: str
    parent: str                                     # robot link or another part
    xyz_mm: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    shape: dict = field(default_factory=lambda: {"type": "none"})
    anchor: str = "base"                            # base: the shape starts at the frame and grows along +Z
    color: str = "#9aa0a6"
    collision: str = "visual"                      # one of COLLISIONS
    kind: str = "part"                              # camera: a camera's frame (hand-eye calibration result)
    note: str = ""
    meta: dict = field(default_factory=dict)        # saved, source (typed / handeye / preset)

    # ---- geometry, metres
    def T_parent_part(self) -> np.ndarray:
        return tf.mm_deg_to_T(self.xyz_mm, self.rpy_deg)

    def extent_m(self, mesh_bounds: Optional[Tuple[np.ndarray, np.ndarray]] = None) -> Tuple[float, float]:
        """(z_min, z_max) of the shape in the part frame."""
        s, t = self.shape, self.shape.get("type")
        if t == "box":
            h = s["size_mm"][2] / 1000.0
        elif t == "cylinder":
            h = s["length_mm"] / 1000.0
        elif t == "sphere":
            h = 2 * s["radius_mm"] / 1000.0
        elif t == "mesh" and mesh_bounds is not None:
            lo, hi = mesh_bounds
            z = s.get("offset_mm", [0, 0, 0])[2] / 1000.0
            return float(lo[2]) * s.get("scale", 1.0) + z, float(hi[2]) * s.get("scale", 1.0) + z
        else:
            return 0.0, 0.0
        return (0.0, h) if self.anchor == "base" else (-h / 2, h / 2)

    def T_part_geom(self) -> np.ndarray:
        """Where the primitive's centre is (URDF primitives are centred); a mesh's origin, moved by its offset_mm."""
        T = np.eye(4)
        if self.shape.get("type") in ("box", "cylinder", "sphere"):
            lo, hi = self.extent_m()
            T[2, 3] = (lo + hi) / 2
        elif self.shape.get("type") == "mesh":
            T[:3, 3] = [v / 1000.0 for v in self.shape.get("offset_mm", [0, 0, 0])]
        return T

    def to_yaml(self) -> dict:
        d = {"parent": self.parent, "xyz_mm": [round(float(v), 3) for v in self.xyz_mm],
             "rpy_deg": [round(float(v), 4) for v in self.rpy_deg], "shape": self.shape, "anchor": self.anchor,
             "color": self.color, "collision": self.collision, "kind": self.kind}
        if self.note:
            d["note"] = self.note
        if self.meta:
            d["meta"] = self.meta
        return d


def _num(v, what, lo=-1e9, hi=1e9) -> float:
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise PartError(f"{what} must be a number") from None
    if not math.isfinite(x) or not lo <= x <= hi:
        raise PartError(f"{what} must be between {lo:g} and {hi:g}")
    return x


def collision_mode(collision, shape_type: str) -> str:
    """visual / box / hull / none; True / False (older files) are visual / none. A frame has none; only a mesh has
    a hull (for a primitive, hull is the shape itself)."""
    if collision is None:
        c = "hull" if shape_type == "mesh" else "visual"
    elif isinstance(collision, bool):
        c = "visual" if collision else "none"
    else:
        c = str(collision).strip().lower()
    if c not in COLLISIONS:
        raise PartError(f"collision is one of {', '.join(COLLISIONS)}")
    if shape_type == "none":
        return "none"
    if c == "hull" and shape_type != "mesh":
        return "visual"
    return c


def make_part(name, parent, xyz_mm=None, rpy_deg=None, shape=None, anchor="base", color="#9aa0a6",
              collision=None, kind="part", note="", meta=None) -> Part:
    name = str(name).strip()
    if not NAME.match(name):
        raise PartError("a part name is lower_snake_case: a-z, 0-9, _ (up to 40, starting with a letter)")
    xyz = [_num(v, "offset", -3000, 3000) for v in (xyz_mm or [0, 0, 0])]
    rpy = [_num(v, "rotation", -360, 360) for v in (rpy_deg or [0, 0, 0])]
    if len(xyz) != 3 or len(rpy) != 3:
        raise PartError("give x, y, z (mm) and roll, pitch, yaw (deg)")
    s = dict(shape or {"type": "none"})
    t = s.get("type")
    if t not in SHAPES:
        raise PartError(f"shape must be one of {', '.join(SHAPES)}")
    if t == "box":
        size = [_num(v, "box size", 0.1, 3000) for v in s.get("size_mm", [])]
        if len(size) != 3:
            raise PartError("a box needs x, y, z sizes in mm")
        s = {"type": "box", "size_mm": size}
    elif t == "cylinder":
        s = {"type": "cylinder", "radius_mm": _num(s.get("radius_mm"), "radius", 0.1, 1500),
             "length_mm": _num(s.get("length_mm"), "length", 0.1, 3000)}
    elif t == "sphere":
        s = {"type": "sphere", "radius_mm": _num(s.get("radius_mm"), "radius", 0.1, 1500)}
    elif t == "mesh":
        f = str(s.get("file", ""))
        if not MESH_FILE.match(f):
            raise PartError("a mesh part needs an uploaded .stl, .obj or .dae file")
        off = [_num(v, "mesh offset", -3000, 3000) for v in (s.get("offset_mm") or [0, 0, 0])]
        if len(off) != 3:
            raise PartError("a mesh offset is x, y, z in mm")
        dflt = 1.0 if M.unit_metres(f) else 0.001                  # a .dae carries its unit; STL / OBJ: CAD mm
        s = {"type": "mesh", "file": f, "scale": _num(s.get("scale", dflt), "mesh scale", 1e-6, 10)}
        if any(off):
            s["offset_mm"] = off
    else:
        s = {"type": "none"}
    if anchor not in ("base", "center"):
        raise PartError("anchor is base or center")
    if not re.match(r"^#[0-9a-fA-F]{6}$", str(color)):
        raise PartError("colour is #rrggbb")
    if kind not in KINDS:
        raise PartError(f"kind is one of {', '.join(KINDS)}")
    return Part(name, str(parent).strip(), xyz, rpy, s, anchor, str(color), collision_mode(collision, s["type"]),
                kind, str(note or "")[:200], dict(meta or {}))


# ===================================================================== store
class PartStore:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.mesh_dir = self.path.parent / "meshes"
        self.generated_dir = self.path.parent / "generated"
        self.parts: Dict[str, Part] = {}
        self.error = ""
        self._bounds: Dict[str, tuple] = {}            # file -> ((file, mtime), (lo, hi))
        self.load()

    def load(self):
        self.parts, self.error = {}, ""
        try:
            d = yaml.safe_load(self.path.read_text()) or {}
        except FileNotFoundError:
            return
        except Exception as e:
            self.error = f"{self.path}: {e}"
            return
        for n, p in (d.get("parts") or {}).items():
            try:
                self.parts[n] = make_part(n, **{k: v for k, v in (p or {}).items()
                                                if k in ("parent", "xyz_mm", "rpy_deg", "shape", "anchor", "color",
                                                         "collision", "kind", "note", "meta")})
            except (PartError, TypeError) as e:
                self.error = f"part {n}: {e}"

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("# MoveIt GUI parts (SETUP > Parts): shapes on robot links, mm / deg.\n" +
                       yaml.safe_dump({"version": 1, "parts": {n: p.to_yaml() for n, p in self.ordered().items()}},
                                      sort_keys=False))
        tmp.replace(self.path)

    def ordered(self) -> Dict[str, Part]:
        """Parents before children (robot links count as placed)."""
        out: Dict[str, Part] = {}
        left = dict(self.parts)
        while left:
            ready = [n for n, p in left.items() if p.parent not in left]
            if not ready:                                   # a cycle: put() refuses those, keep the rest stable
                out.update(left)
                break
            for n in ready:
                out[n] = left.pop(n)
        return out

    def check(self, part: Part, robot_links: Set[str], replace: Optional[str] = None):
        others = {n: p for n, p in self.parts.items() if n != replace}
        if part.name in robot_links:
            raise PartError(f"'{part.name}' is a link of the robot model: pick another name")
        if part.name in others:
            raise PartError(f"a part '{part.name}' exists")
        if part.parent == part.name:
            raise PartError("a part cannot sit on itself")
        if part.parent not in robot_links and part.parent not in others:
            raise PartError(f"unknown parent '{part.parent}' (a robot link or another part)")
        cur, seen = part.parent, set()
        while True:                                          # the parent chain must not come back to this part
            if cur in (part.name, replace) or cur in seen:
                raise PartError("that parent is on this part (a part cannot carry its own parent)")
            if cur not in others:
                break
            seen.add(cur)
            cur = others[cur].parent
        if part.shape.get("type") == "mesh" and not (self.mesh_dir / part.shape["file"]).is_file():
            raise PartError(f"mesh {part.shape['file']} is not uploaded")

    def put(self, part: Part, robot_links: Set[str], replace: Optional[str] = None):
        self.check(part, robot_links, replace)
        if replace and replace != part.name:
            if replace not in self.parts:
                raise PartError(f"no part '{replace}'")
            for c in self.parts.values():                    # renamed: children follow
                if c.parent == replace:
                    c.parent = part.name
            del self.parts[replace]
        part.meta = {**part.meta, "saved": time.strftime("%Y-%m-%d %H:%M")}
        self.parts[part.name] = part
        self.save()

    def delete(self, name: str):
        if name not in self.parts:
            raise PartError(f"no part '{name}'")
        kids = [n for n, p in self.parts.items() if p.parent == name]
        if kids:
            raise PartError(f"{', '.join(kids)} sit(s) on '{name}': move or delete them first")
        del self.parts[name]
        self.save()

    # ---- meshes
    def add_mesh(self, filename: str, data: bytes) -> str:
        base = re.sub(r"[^A-Za-z0-9_.-]", "_", Path(str(filename)).name).lstrip(".-") or "mesh"
        if not MESH_FILE.match(base):
            raise PartError("upload an .stl, .obj or .dae file (export it from CAD as one of these)")
        if len(data) > MESH_MAX_BYTES:
            raise PartError(f"mesh over {MESH_MAX_BYTES // 2**20} MB")
        try:
            M.read_mesh_bytes(data, base)
        except M.MeshError as e:
            raise PartError(str(e)) from None
        self.mesh_dir.mkdir(parents=True, exist_ok=True)
        tmp = self.mesh_dir / (base + ".upload")
        tmp.write_bytes(data)
        tmp.replace(self.mesh_dir / base)
        self._bounds.pop(base, None)
        return base

    def meshes(self) -> List[str]:
        if not self.mesh_dir.is_dir():
            return []
        return sorted(p.name for p in self.mesh_dir.iterdir() if p.is_file() and MESH_FILE.match(p.name))

    def mesh_info(self, f: str) -> dict:
        """Size of a mesh as read (a .dae in m, others in file units) and a guess of the unit, for the upload."""
        b = self.mesh_bounds(f)
        if b is None:
            return {"file": f, "error": "cannot read it"}
        size = [float(v) for v in b[1] - b[0]]
        metres = M.unit_metres(f)
        hint = "m" if metres else M.unit_hint(size)
        return {"file": f, "min": [float(v) for v in b[0]], "max": [float(v) for v in b[1]], "size": size,
                "unit": "m" if metres else "file", "hint": hint,
                "scale": 1.0 if metres or hint == "m" else 0.001}

    def mesh_bounds(self, f: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        p = self.mesh_dir / f
        try:
            key = (f, p.stat().st_mtime_ns)
        except OSError:
            return None
        if self._bounds.get(f, (None,))[0] != key:
            try:
                tri = read_mesh(p).reshape(-1, 3)
            except (OSError, PartError):
                return None
            self._bounds[f] = (key, (tri.min(axis=0), tri.max(axis=0)))
        return self._bounds[f][1]

    def hull_file(self, f: str) -> Path:
        """The convex hull of mesh f as an STL (same units as f reads), made once per mesh version."""
        src = self.mesh_dir / f
        out = self.generated_dir / "hulls" / (f + ".hull.stl")
        if out.is_file() and out.stat().st_mtime_ns >= src.stat().st_mtime_ns:
            return out
        try:
            tri = M.convex_hull(read_mesh(src))
        except M.MeshError as e:
            raise PartError(f"{f}: no hull ({e})") from None
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(".tmp")
        M.write_stl(tmp, tri)
        tmp.replace(out)
        return out

    def extent_m(self, part: Part) -> Tuple[float, float]:
        mb = self.mesh_bounds(part.shape["file"]) if part.shape.get("type") == "mesh" else None
        return part.extent_m(mb)

    def stack_offset_mm(self, parent: str) -> List[float]:
        """Default offset for a new part on `parent`: on its far face (0 for robot links)."""
        p = self.parts.get(parent)
        return [0.0, 0.0, round(self.extent_m(p)[1] * 1000.0, 3) if p else 0.0]


# ===================================================================== robot model helpers
def _joint_T(j) -> np.ndarray:
    return tf.mm_deg_to_T([v * 1000.0 for v in j.get("xyz", [0, 0, 0])], [math.degrees(v) for v in j.get("rpy", [0, 0, 0])])


def rigid_group(model: dict, link: str) -> Set[str]:
    """Links joined to `link` through fixed joints only (they never move relative to it)."""
    adj: Dict[str, List[str]] = {}
    for j in model.get("joints", []):
        if j["type"] == "fixed":
            adj.setdefault(j["parent"], []).append(j["child"])
            adj.setdefault(j["child"], []).append(j["parent"])
    out, todo = {link}, [link]
    while todo:
        for n in adj.get(todo.pop(), []):
            if n not in out:
                out.add(n)
                todo.append(n)
    return out


def neighbourhood(model: dict, link: str) -> Set[str]:
    """The rigid group of `link` plus every link one joint away from it: what a part on `link` may touch
    (MoveIt's 'Adjacent' plus the rigid stack it is bolted to)."""
    g = rigid_group(model, link)
    out = set(g)
    for j in model.get("joints", []):
        if j["parent"] in g:
            out.add(j["child"])
        if j["child"] in g:
            out.add(j["parent"])
    return out


def rides_on_arm(model: dict, link: str) -> bool:
    """True when the link moves with the robot (not in the root's rigid group)."""
    root = model.get("root")
    return bool(root) and root not in rigid_group(model, link)


def fixed_T(model: dict, a: str, b: str) -> Optional[np.ndarray]:
    """T_a_b for two links of one rigid group (joint values do not matter), else None."""
    if b not in rigid_group(model, a):
        return None
    edges: Dict[str, List[Tuple[str, np.ndarray]]] = {}
    for j in model.get("joints", []):
        if j["type"] == "fixed":
            T = _joint_T(j)
            edges.setdefault(j["parent"], []).append((j["child"], T))
            edges.setdefault(j["child"], []).append((j["parent"], tf.inv(T)))
    seen, todo = {a}, [(a, np.eye(4))]
    while todo:
        n, T = todo.pop()
        if n == b:
            return T
        for m, E in edges.get(n, []):
            if m not in seen:
                seen.add(m)
                todo.append((m, T @ E))
    return None


def model_parts(model: dict) -> Dict[str, dict]:
    """Parts the robot model has (from a generated URDF): {name: joint}."""
    return {p: j for j in model.get("joints", []) if (p := part_of_joint(j["name"])) is not None}


# ===================================================================== resolved view
def _same_T(a: np.ndarray, b: np.ndarray) -> bool:
    if np.abs(a[:3, 3] - b[:3, 3]).max() > POS_TOL_M:
        return False
    c = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
    return math.acos(max(-1.0, min(1.0, c))) < ANG_TOL_RAD


def _geom_m(store: PartStore, part: Part) -> Optional[dict]:
    s, t = part.shape, part.shape.get("type")
    if t == "box":
        return {"type": "box", "size": [v / 1000.0 for v in s["size_mm"]]}
    if t == "cylinder":
        return {"type": "cylinder", "radius": s["radius_mm"] / 1000.0, "length": s["length_mm"] / 1000.0}
    if t == "sphere":
        return {"type": "sphere", "radius": s["radius_mm"] / 1000.0}
    if t == "mesh":
        return {"type": "mesh", "uri": mesh_uri(store, s["file"]), "scale": [s["scale"]] * 3}
    return None


def mesh_uri(store: PartStore, f: str) -> str:
    return (store.mesh_dir / f).resolve().as_uri()


def _rgba(color: str) -> List[float]:
    return [int(color[i:i + 2], 16) / 255.0 for i in (1, 3, 5)] + [1.0]


def _keeps_colours(p: Part) -> bool:
    """A .dae shows its own colours (RViz and the 3D view): the part colour is not put over it."""
    return p.shape.get("type") == "mesh" and M.unit_metres(p.shape["file"])


def _visual(store: PartStore, p: Part) -> Optional[dict]:
    g = _geom_m(store, p)
    if g is None:
        return None
    T = p.T_part_geom()
    return {"geometry": g, "xyz": [float(v) for v in T[:3, 3]], "rpy": [0.0, 0.0, 0.0],
            "rgba": None if _keeps_colours(p) else _rgba(p.color)}


def _collision(store: PartStore, p: Part) -> Optional[Tuple[dict, np.ndarray]]:
    """(geometry in metres, T_part_geom) MoveIt and the URDF collide with, None for collision none."""
    g = _geom_m(store, p)
    if g is None or p.collision == "none":
        return None
    T = p.T_part_geom()
    if p.collision == "box":
        if g["type"] == "mesh":
            b = store.mesh_bounds(p.shape["file"])
            if b is None:
                return None
            lo, hi = b[0] * p.shape["scale"], b[1] * p.shape["scale"]
            C = np.eye(4)
            C[:3, 3] = (lo + hi) / 2
            return {"type": "box", "size": [max(float(v), 1e-4) for v in hi - lo]}, T @ C
        if g["type"] == "cylinder":
            return {"type": "box", "size": [2 * g["radius"], 2 * g["radius"], g["length"]]}, T
        if g["type"] == "sphere":
            return {"type": "box", "size": [2 * g["radius"]] * 3}, T
        return g, T
    if p.collision == "hull" and g["type"] == "mesh":
        try:
            return {**g, "uri": store.hull_file(p.shape["file"]).resolve().as_uri()}, T
        except (OSError, PartError):
            return g, T                                        # no hull (flat mesh): the mesh itself
    return g, T


def collision_geom(store: PartStore, p: Part) -> Optional[dict]:
    """The collision shape as a visual-like dict {geometry, xyz, rpy} (for the 3D views), None for none."""
    c = _collision(store, p)
    if c is None:
        return None
    g, T = c
    return {"geometry": g, "xyz": [float(v) for v in T[:3, 3]],
            "rpy": list(tf.quat_to_rpy(tf.matrix_to_quat(T[:3, :3]))), "rgba": None}


def _same_geom(a: Optional[dict], b: Optional[dict]) -> bool:
    if a is None or b is None:
        return a is None and b is None
    ga, gb = a["geometry"], b["geometry"]
    if ga.get("type") != gb.get("type") or not np.allclose(a.get("xyz", [0, 0, 0]), b["xyz"], atol=POS_TOL_M):
        return False
    if ga.get("type") == "mesh" and Path(str(ga.get("uri", ""))).name != Path(str(gb.get("uri", ""))).name:
        return False
    return all(np.allclose(ga.get(k), gb[k], atol=1e-6) for k in ("size", "radius", "length", "scale") if k in gb)


def status(store: PartStore, model: dict) -> Dict[str, str]:
    """Per part: live (only in the GUI's live scene / TF), in_model (the robot model has it, as saved),
    changed (the robot model has an older version), plus removed parts the model still has."""
    have = model_parts(model)
    links = model.get("links", {})
    out: Dict[str, str] = {}
    for n, p in store.parts.items():
        j = have.get(n)
        if j is None:
            out[n] = "live"
            continue
        same = j["parent"] == p.parent and _same_T(_joint_T(j), p.T_parent_part())
        vis, mine = links.get(n) or [], _visual(store, p)
        same = same and len(vis) == (1 if mine else 0) and (not mine or _same_geom(vis[0], mine))
        if same and "collisions" in model:                   # the model knows its collision shapes: compare those
            col, cmine = model["collisions"].get(n) or [], collision_geom(store, p)
            same = len(col) == (1 if cmine else 0) and (not cmine or _same_geom(col[0], cmine))
        out[n] = "in_model" if same else "changed"
    for n in have:
        if n not in store.parts:
            out[n] = "removed"
    return out


def merged_model(store: PartStore, model: dict) -> dict:
    """The robot model plus the live parts (for the 3D views and FK). Parts already in the model stay as they are."""
    st = status(store, model)
    links = dict(model.get("links", {}))
    cols = dict(model.get("collisions", {}))
    joints = list(model.get("joints", []))
    for n, p in store.ordered().items():
        if st.get(n) != "live" or (p.parent not in links):
            continue
        v = _visual(store, p)
        links[n] = [v] if v else []
        c = collision_geom(store, p)
        cols[n] = [c] if c else []
        T = p.T_parent_part()
        joints.append({"name": joint_name(n), "type": "fixed", "parent": p.parent, "child": n,
                       "xyz": [float(x) for x in T[:3, 3]], "rpy": list(tf.quat_to_rpy(tf.matrix_to_quat(T[:3, :3]))),
                       "axis": [1.0, 0, 0], "limit": None, "mimic": None})
    return {**model, "links": links, "collisions": cols, "joints": joints}


def anchor(store: PartStore, model: dict, name: str) -> Tuple[str, np.ndarray]:
    """(robot link the part is bolted to through other parts, T_link_part)."""
    T = np.eye(4)
    cur = name
    links = model.get("links", {})
    hops = 0
    while cur in store.parts and cur not in model_parts(model):
        hops += 1
        if hops > len(store.parts):
            raise PartError(f"'{name}': its parents form a loop")
        p = store.parts[cur]
        T = p.T_parent_part() @ T
        cur = p.parent
    if cur not in links:
        raise PartError(f"'{cur}' is not in the robot model")
    return cur, T


def live_plan(store: PartStore, model: dict) -> Tuple[List[Tuple[str, str, np.ndarray]], List[dict]]:
    """What to publish now: static TF [(parent, child, T)] and collision objects
    [{id, link, T_link_obj, geom (metres), attach, touch_links}] for the parts the robot model does not have yet."""
    st = status(store, model)
    links = set(model.get("links", {}))
    frames, objects = [], []
    for n, p in store.ordered().items():
        if st.get(n) != "live":
            continue
        try:
            link, T_link_part = anchor(store, model, n)
        except PartError:
            continue                                         # parent missing from this model: shown as a problem
        if p.parent in links or p.parent in store.parts:
            frames.append((p.parent, n, p.T_parent_part()))
        c = _collision(store, p)
        if c is None:
            continue
        objects.append({"id": OBJECT_PREFIX + n, "link": link, "T_link_obj": T_link_part @ c[1], "geom": c[0], "attach": rides_on_arm(model, link),
                        "touch_links": sorted(neighbourhood(model, link) | {
                            q for q in store.parts if q != n and _anchor_link(store, model, q) == link})})
    return frames, objects


def _anchor_link(store, model, name) -> Optional[str]:
    try:
        return anchor(store, model, name)[0]
    except PartError:
        return None


def problems(store: PartStore, model: dict) -> List[str]:
    out = []
    links = set(model.get("links", {}))
    for n, p in store.parts.items():
        if p.parent not in links and p.parent not in store.parts:
            out.append(f"{n}: parent '{p.parent}' is not in the robot model")
        if p.shape.get("type") == "mesh" and not (store.mesh_dir / p.shape["file"]).is_file():
            out.append(f"{n}: mesh {p.shape['file']} is missing")
    return out


# ===================================================================== generated URDF / SRDF
def _f(v: float) -> str:
    s = f"{float(v):.9f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _origin_xml(T: np.ndarray) -> str:
    r, p, y = tf.quat_to_rpy(tf.matrix_to_quat(T[:3, :3]))
    return f'<origin xyz="{" ".join(_f(v) for v in T[:3, 3])}" rpy="{_f(r)} {_f(p)} {_f(y)}"/>'


def _geometry_xml(g: dict) -> str:
    if g["type"] == "box":
        return f'<box size="{" ".join(_f(v) for v in g["size"])}"/>'
    if g["type"] == "cylinder":
        return f'<cylinder radius="{_f(g["radius"])}" length="{_f(g["length"])}"/>'
    if g["type"] == "sphere":
        return f'<sphere radius="{_f(g["radius"])}"/>'
    return f'<mesh filename={quoteattr(g["uri"])} scale="{" ".join(_f(v) for v in g["scale"])}"/>'


def _base_only(model: dict) -> dict:
    """The robot model without generated parts (what the robot's own xacro makes)."""
    return {**model, "joints": [j for j in model.get("joints", []) if part_of_joint(j["name"]) is None],
            "links": {k: v for k, v in model.get("links", {}).items() if k not in model_parts(model)}}


def placeable(store: PartStore, model: Optional[dict]) -> Dict[str, Part]:
    """Parts whose parent chain ends on a link of the robot's own model (others would break the URDF)."""
    if not model:
        return store.ordered()
    base = _base_only(model)
    return {n: p for n, p in store.ordered().items() if _anchor_link(store, base, n) is not None}


def generate_urdf(store: PartStore, model: Optional[dict] = None) -> str:
    """Fixed links for the parts. With the robot model given, parts that do not hang on it are left out (listed
    by problems()), so the robot stack never gets a URDF with a missing parent link."""
    lines = ['<?xml version="1.0"?>',
             '<!-- GENERATED by MoveIt GUI (SETUP > Parts). Do not edit: the GUI rewrites it. -->',
             '<robot xmlns:xacro="http://wiki.ros.org/xacro">']
    for n, p in placeable(store, model).items():
        g = _geom_m(store, p)
        lines.append(f'  <link name="{n}">')
        if g is not None:
            o = _origin_xml(p.T_part_geom())
            rgba = " ".join(_f(v) for v in _rgba(p.color))
            mat = "" if _keeps_colours(p) else \
                f'<material name="gui_part_{n}_colour"><color rgba="{rgba}"/></material>'
            lines.append(f'    <visual>{o}<geometry>{_geometry_xml(g)}</geometry>{mat}</visual>')
            c = _collision(store, p)
            if c is not None:
                lines.append(f'    <collision>{_origin_xml(c[1])}<geometry>{_geometry_xml(c[0])}</geometry>'
                             '</collision>')
        lines.append('  </link>')
        lines.append(f'  <joint name="{joint_name(n)}" type="fixed"><parent link="{p.parent}"/><child link="{n}"/>'
                     f'{_origin_xml(p.T_parent_part())}</joint>')
    lines.append('</robot>')
    return "\n".join(lines) + "\n"


def generate_srdf(store: PartStore, model: dict) -> str:
    """disable_collisions for each part with collision against what it is bolted to (its anchor link's
    neighbourhood) and against the other parts on the same anchor."""
    base_links = {k for k in model.get("links", {}) if k not in model_parts(model)}
    lines = ['<?xml version="1.0"?>',
             '<!-- GENERATED by MoveIt GUI (SETUP > Parts). Do not edit: the GUI rewrites it. -->',
             '<robot xmlns:xacro="http://wiki.ros.org/xacro">']
    pairs: Set[Tuple[str, str]] = set()
    base = _base_only(model)
    anchors = {n: _anchor_link(store, base, n) for n in store.parts}
    for n, p in placeable(store, model).items():
        a = anchors.get(n)
        if a is None:
            continue
        for other in sorted(neighbourhood(model, a) & base_links):
            pairs.add(tuple(sorted((n, other))))
        for m in store.parts:
            if m != n and anchors.get(m) == a:
                pairs.add(tuple(sorted((n, m))))
        if p.parent in store.parts:
            pairs.add(tuple(sorted((n, p.parent))))
    for a, b in sorted(pairs):
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>')
    lines.append('</robot>')
    return "\n".join(lines) + "\n"


def write_generated(store: PartStore, model: dict) -> Tuple[Path, Path]:
    """Write the URDF / SRDF includes the launcher hands to the robot's xacro at the next start."""
    d = store.generated_dir
    d.mkdir(parents=True, exist_ok=True)
    out = []
    for name, text in ((GENERATED_URDF, generate_urdf(store, model)), (GENERATED_SRDF, generate_srdf(store, model))):
        tmp = d / (name + ".tmp")
        tmp.write_text(text)
        tmp.replace(d / name)
        out.append(d / name)
    return out[0], out[1]


# ===================================================================== presets
PRESETS = {
    "spatula on the gripper": dict(parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 150]},
                                   xyz_mm=[0, 0, -20], color="#c0c4c8"),
    "wrist camera bracket": dict(parent="tool0", shape={"type": "box", "size_mm": [40, 90, 25]},
                                 xyz_mm=[0, -70, 0], color="#555b62"),
    "RealSense D435": dict(parent="tool0", shape={"type": "box", "size_mm": [25, 90, 25]}, kind="camera",
                           xyz_mm=[0, -80, 30], color="#2b2f33"),
    "table": dict(parent="base_link", shape={"type": "box", "size_mm": [1200, 800, 20]}, anchor="center",
                  xyz_mm=[0, 300, -11], color="#8a7356"),
    "frame only": dict(parent="tool0", shape={"type": "none"}),
}
