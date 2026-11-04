"""Mesh files for SETUP > Parts: STL (binary / ASCII), OBJ and COLLADA (.dae), the formats URDF, RViz and MoveIt read.

read_mesh() returns triangles (n, 3, 3) in the file's own units, except COLLADA, whose <unit meter> is applied (as
RViz / MoveIt and the 3D view do), so a .dae comes back in metres. Node transforms of a .dae's visual scene are
applied; its up_axis is not (RViz does not either). Textures and materials are not read: the 3D view shows a .dae
with its own colours, collision only needs the triangles.
"""
from __future__ import annotations

import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

MESH_MAX_BYTES = 20 * 1024 * 1024
MESH_MAX_TRIANGLES = 200_000
SUFFIXES = (".stl", ".obj", ".dae")


class MeshError(ValueError):
    pass


def _check(tri: np.ndarray, what: str) -> np.ndarray:
    if tri.size == 0:
        raise MeshError(f"{what}: no triangles")
    if len(tri) > MESH_MAX_TRIANGLES:
        raise MeshError(f"{what}: {len(tri)} triangles (at most {MESH_MAX_TRIANGLES}): simplify it")
    if not np.isfinite(tri).all():
        raise MeshError(f"{what}: non-finite vertices")
    return tri


def read_stl_bytes(data: bytes) -> np.ndarray:
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            if n > MESH_MAX_TRIANGLES:
                raise MeshError(f"STL: {n} triangles (at most {MESH_MAX_TRIANGLES}): simplify it")
            rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
            return _check(np.frombuffer(data, rec, n, 84)["v"].astype(float), "STL")
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        raise MeshError("not an STL file (binary size does not match, and it is not ASCII)") from None
    if not text.lstrip().lower().startswith("solid"):
        raise MeshError("not an STL file")
    v = [[float(x) for x in m.groups()] for m in re.finditer(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", text)]
    if not v or len(v) % 3:
        raise MeshError("STL without complete triangles")
    return _check(np.asarray(v, float).reshape(-1, 3, 3), "STL")


def read_obj_bytes(data: bytes) -> np.ndarray:
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception:
        raise MeshError("OBJ: not text") from None
    verts: List[List[float]] = []
    tris: List[List[int]] = []
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "v" and len(p) >= 4:
            try:
                verts.append([float(p[1]), float(p[2]), float(p[3])])
            except ValueError:
                raise MeshError(f"OBJ: bad vertex line '{line[:40]}'") from None
        elif p[0] == "f" and len(p) >= 4:
            idx = []
            for tok in p[1:]:
                try:
                    k = int(tok.split("/")[0])
                except ValueError:
                    raise MeshError(f"OBJ: bad face line '{line[:40]}'") from None
                idx.append(k - 1 if k > 0 else len(verts) + k)       # 1-based, negative = from the end
            for i in range(1, len(idx) - 1):                          # polygon -> triangle fan
                tris.append([idx[0], idx[i], idx[i + 1]])
            if len(tris) > MESH_MAX_TRIANGLES:
                raise MeshError(f"OBJ: over {MESH_MAX_TRIANGLES} triangles: simplify it")
    if not verts or not tris:
        raise MeshError("OBJ: no faces (an .obj needs v and f lines)")
    V, F = np.asarray(verts, float), np.asarray(tris, int)
    if F.min() < 0 or F.max() >= len(V):
        raise MeshError("OBJ: a face refers to a vertex that does not exist")
    return _check(V[F], "OBJ")


def _floats(text: Optional[str]) -> List[float]:
    return [float(x) for x in (text or "").split()]


def _node_T(node, ns) -> np.ndarray:
    """A COLLADA <node>'s transform from its matrix / translate / rotate / scale children, in order."""
    T = np.eye(4)
    for el in node:
        tag = el.tag.split("}")[-1]
        v = _floats(el.text)
        M = np.eye(4)
        if tag == "matrix" and len(v) == 16:
            M = np.asarray(v).reshape(4, 4)
        elif tag == "translate" and len(v) == 3:
            M[:3, 3] = v
        elif tag == "scale" and len(v) == 3:
            M[:3, :3] = np.diag(v)
        elif tag == "rotate" and len(v) == 4:
            ax = np.asarray(v[:3]) / (np.linalg.norm(v[:3]) or 1.0)
            a = np.radians(v[3])
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            M[:3, :3] = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
        else:
            continue
        T = T @ M
    return T


def read_dae_bytes(data: bytes) -> np.ndarray:
    """Triangles of a COLLADA file in metres (<unit meter> applied), visual scene node transforms applied."""
    try:
        root = ET.fromstring(data)
    except ET.ParseError as e:
        raise MeshError(f"DAE: not valid XML ({e})") from None
    m = re.match(r"\{.*\}", root.tag)
    ns = m.group(0) if m else ""
    unit = root.find(f"{ns}asset/{ns}unit")
    meter = float(unit.get("meter", 1.0)) if unit is not None else 1.0
    geoms: Dict[str, np.ndarray] = {}
    for g in root.iter(f"{ns}geometry"):
        mesh = g.find(f"{ns}mesh")
        if mesh is None:
            continue
        sources = {s.get("id"): s for s in mesh.findall(f"{ns}source")}
        vert_src = {}
        for v in mesh.findall(f"{ns}vertices"):
            inp = next((i for i in v.findall(f"{ns}input") if i.get("semantic") == "POSITION"), None)
            if inp is not None:
                vert_src[v.get("id")] = inp.get("source", "").lstrip("#")
        out = []
        for prim in list(mesh.findall(f"{ns}triangles")) + list(mesh.findall(f"{ns}polylist")) + \
                list(mesh.findall(f"{ns}polygons")):
            inputs = prim.findall(f"{ns}input")
            stride = max(int(i.get("offset", 0)) for i in inputs) + 1 if inputs else 1
            vin = next((i for i in inputs if i.get("semantic") == "VERTEX"), None)
            if vin is None:
                continue
            sid = vert_src.get(vin.get("source", "").lstrip("#"), vin.get("source", "").lstrip("#"))
            src = sources.get(sid)
            if src is None:
                continue
            fa = src.find(f"{ns}float_array")
            P = np.asarray(_floats(fa.text if fa is not None else ""), float).reshape(-1, 3)
            off = int(vin.get("offset", 0))
            pels = prim.findall(f"{ns}p")
            idx = np.asarray([int(x) for p in pels for x in (p.text or "").split()], int)
            vi = idx[off::stride] if idx.size else idx
            tag = prim.tag.split("}")[-1]
            if tag == "triangles":
                F = vi.reshape(-1, 3)
            else:
                counts = _floats(prim.findtext(f"{ns}vcount")) if tag == "polylist" else \
                    [len((p.text or "").split()) // stride for p in pels]
                F, k = [], 0
                for c in map(int, counts):
                    poly = vi[k:k + c]
                    k += c
                    F += [[poly[0], poly[i], poly[i + 1]] for i in range(1, c - 1)]
                F = np.asarray(F, int).reshape(-1, 3)
            if F.size and (F.min() < 0 or F.max() >= len(P)):
                raise MeshError("DAE: a triangle refers to a vertex that does not exist")
            if F.size:
                out.append(P[F])
        if out:
            geoms[g.get("id")] = np.concatenate(out)
    if not geoms:
        raise MeshError("DAE: no triangle meshes in the file")
    tris = []

    def walk(node, T):
        T = T @ _node_T(node, ns)
        for ig in node.findall(f"{ns}instance_geometry"):
            g = geoms.get(ig.get("url", "").lstrip("#"))
            if g is not None:
                tris.append(np.einsum("ij,nkj->nki", T[:3, :3], g) + T[:3, 3])
        for child in node.findall(f"{ns}node"):
            walk(child, T)
    for vs in root.iter(f"{ns}visual_scene"):
        for n in vs.findall(f"{ns}node"):
            walk(n, np.eye(4))
    if not tris:                                          # no scene: the geometry as it is
        tris = list(geoms.values())
    return _check(np.concatenate(tris) * meter, "DAE")


def read_mesh_bytes(data: bytes, name: str) -> np.ndarray:
    if len(data) > MESH_MAX_BYTES:
        raise MeshError(f"mesh over {MESH_MAX_BYTES // 2**20} MB")
    s = Path(name).suffix.lower()
    if s == ".stl":
        return read_stl_bytes(data)
    if s == ".obj":
        return read_obj_bytes(data)
    if s == ".dae":
        return read_dae_bytes(data)
    raise MeshError(f"{name}: use .stl, .obj or .dae (export it from CAD as one of these)")


def read_mesh(path) -> np.ndarray:
    p = Path(path)
    return read_mesh_bytes(p.read_bytes(), p.name)


def unit_metres(name: str) -> bool:
    """A .dae carries its unit (read_mesh returns metres); .stl / .obj are in whatever unit the CAD used."""
    return Path(name).suffix.lower() == ".dae"


def unit_hint(size_file_units) -> str:
    """Guess the unit of an STL / OBJ from its size (most tool parts are 5 mm .. 1 m)."""
    m = max(float(x) for x in size_file_units)
    if m <= 0:
        return ""
    if m > 5.0:
        return "mm"                    # > 5 'metres' is no tool: millimetres
    if m < 0.002:
        return "?"                     # < 2 mm either way: suspicious
    if m < 1.5:
        return "m"
    return ""


def write_stl(path, tri: np.ndarray):
    tri = np.asarray(tri, dtype="<f4").reshape(-1, 3, 3)
    rec = np.zeros(len(tri), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["v"] = tri
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    n = np.cross(e1, e2)
    rec["n"] = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    Path(path).write_bytes(b"MoveIt GUI generated".ljust(80, b" ") + struct.pack("<I", len(tri)) + rec.tobytes())


def convex_hull(tri: np.ndarray) -> np.ndarray:
    """Triangles of the convex hull of a mesh (for a light collision shape), outward facing."""
    from scipy.spatial import ConvexHull
    pts = np.unique(np.asarray(tri, float).reshape(-1, 3).round(9), axis=0)
    if len(pts) < 4:
        raise MeshError("too few distinct vertices for a hull")
    h = ConvexHull(pts)
    out = pts[h.simplices]
    c = pts.mean(axis=0)
    flip = np.einsum("ij,ij->i", np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0]), out.mean(axis=1) - c) < 0
    out[flip] = out[flip][:, ::-1]
    return out
