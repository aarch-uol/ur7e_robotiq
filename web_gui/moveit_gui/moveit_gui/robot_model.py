"""The robot model for the web 3D view: the URDF reduced to what a browser needs to draw it.

The browser does forward kinematics itself from the joint states (arm joints and the gripper joint, mimic joints
follow), so only the tree, the joint axes/limits and the visuals go over the wire, once.

Meshes stay package:// URIs; the web server serves them from the package's share directory (mesh_path), never
from anywhere else.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Callable, Dict, List, Optional

MESH_SUFFIXES = {".dae", ".stl", ".obj", ".png", ".jpg", ".jpeg", ".tga"}


class ModelError(ValueError):
    pass


def _floats(text: Optional[str], n: int, default: float = 0.0) -> List[float]:
    if not text:
        return [default] * n
    v = [float(x) for x in text.split()]
    if len(v) != n:
        raise ModelError(f"expected {n} numbers, got '{text}'")
    return v


def _origin(el) -> dict:
    o = el.find("origin") if el is not None else None
    return {"xyz": _floats(o.get("xyz") if o is not None else None, 3),
            "rpy": _floats(o.get("rpy") if o is not None else None, 3)}


def _geometry(g) -> Optional[dict]:
    if g is None:
        return None
    for kind in ("mesh", "box", "cylinder", "sphere"):
        e = g.find(kind)
        if e is None:
            continue
        if kind == "mesh":
            return {"type": "mesh", "uri": e.get("filename", ""), "scale": _floats(e.get("scale"), 3, 1.0)}
        if kind == "box":
            return {"type": "box", "size": _floats(e.get("size"), 3)}
        if kind == "cylinder":
            return {"type": "cylinder", "radius": float(e.get("radius", 0)), "length": float(e.get("length", 0))}
        return {"type": "sphere", "radius": float(e.get("radius", 0))}
    return None


def parse_urdf(xml: str) -> dict:
    """{root, links: {name: [visual]}, collisions: {name: [collision]}, joints: [{name, type, parent, child, xyz,
    rpy, axis, mimic}]}"""
    try:
        robot = ET.fromstring(xml)
    except ET.ParseError as e:
        raise ModelError(f"robot_description is not valid XML: {e}") from None
    materials = {}
    for m in robot.findall("material"):
        c = m.find("color")
        if c is not None and m.get("name"):
            materials[m.get("name")] = _floats(c.get("rgba"), 4, 1.0)
    links: Dict[str, list] = {}
    collisions: Dict[str, list] = {}
    for link in robot.findall("link"):
        cols = []
        for c in link.findall("collision"):
            geo = _geometry(c.find("geometry"))
            if geo is not None:
                cols.append({"geometry": geo, **_origin(c), "rgba": None})
        collisions[link.get("name")] = cols
        vis = []
        for v in link.findall("visual"):
            geo = _geometry(v.find("geometry"))
            if geo is None:
                continue
            rgba = None
            mat = v.find("material")
            if mat is not None:
                c = mat.find("color")
                rgba = _floats(c.get("rgba"), 4, 1.0) if c is not None else materials.get(mat.get("name"))
            vis.append({"geometry": geo, **_origin(v), "rgba": rgba})
        links[link.get("name")] = vis
    joints, children = [], set()
    for j in robot.findall("joint"):
        parent, child = j.find("parent"), j.find("child")
        if parent is None or child is None:
            continue
        mimic = j.find("mimic")
        axis = j.find("axis")
        lim = j.find("limit")
        limit = None
        if lim is not None and lim.get("lower") is not None and lim.get("upper") is not None:
            limit = {"lower": float(lim.get("lower")), "upper": float(lim.get("upper"))}
        joints.append({
            "name": j.get("name"), "type": j.get("type", "fixed"),
            "parent": parent.get("link"), "child": child.get("link"), **_origin(j),
            "axis": _floats(axis.get("xyz") if axis is not None else None, 3) if axis is not None else [1.0, 0, 0],
            "limit": limit,
            # rad/s (m/s): the constant-speed retime of linear moves keeps under it (cart_speed.py)
            "max_velocity": float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None,
            "mimic": None if mimic is None else {"joint": mimic.get("joint"),
                                                 "multiplier": float(mimic.get("multiplier", 1.0)),
                                                 "offset": float(mimic.get("offset", 0.0))},
        })
        children.add(child.get("link"))
    roots = [n for n in links if n not in children]
    if not roots:
        raise ModelError("the URDF has no root link")
    return {"root": roots[0], "links": links, "collisions": collisions, "joints": joints}


def default_share_lookup(package: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except Exception:                                   # not found, or no ROS in this environment
        return None


def mesh_path(package: str, rel: str, share_lookup: Callable[[str], Optional[Path]] = default_share_lookup
              ) -> Optional[Path]:
    """The file for package://<package>/<rel>, only inside that package's share directory and only for mesh and
    texture file types. None otherwise.

    The request is checked lexically (no '..', no absolute path); symlinks the package installed are then
    followed (colcon --symlink-install links every file back to the source tree)."""
    if not package or "/" in package or package.startswith("."):
        return None
    parts = Path(rel).parts
    if not parts or Path(rel).is_absolute() or any(x in ("..", ".") for x in parts) or "\\" in rel:
        return None
    share = share_lookup(package)
    if share is None:
        return None
    p = share.joinpath(*parts)
    if p.suffix.lower() not in MESH_SUFFIXES or not p.is_file():
        return None
    return p


def model_or_error(xml: Optional[str]) -> dict:
    if not xml:
        return {"ok": False, "msg": "Robot model not loaded yet: is the robot stack running? (no robot_description from robot_state_publisher)"}
    try:
        return {"ok": True, **parse_urdf(xml)}
    except (ModelError, ValueError) as e:
        return {"ok": False, "msg": str(e)}

