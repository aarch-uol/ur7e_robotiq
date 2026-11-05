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
MESH_MAX_TRIANGLES = 400_000
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


# ===================================================================== automatic light collision
AUTO_MAX_PIECES = 8
AUTO_MAX_TRIANGLES = 64
AUTO_TOLERANCE_MM = 0.5
AUTO_CONCAVITY = 1.35
AUTO_COLLISION_VERSION = 1
AUTO_SEARCH_TRIANGLES = 1200
AUTO_EXACT_VOLUME_TRIANGLES = 10_000


def _points(tri: np.ndarray) -> np.ndarray:
    """Distinct mesh vertices, stable across repeated reads of the same file."""
    p = np.asarray(tri, float).reshape(-1, 3)
    return np.unique(p.round(12), axis=0)


def _stable_hull(points: np.ndarray):
    """(outward triangles, scipy hull), with deterministic triangle/vertex ordering."""
    from scipy.spatial import ConvexHull
    p = np.unique(np.asarray(points, float).reshape(-1, 3).round(12), axis=0)
    if len(p) < 4:
        raise MeshError("too few distinct vertices for a hull")
    h = ConvexHull(p)
    out = p[h.simplices].copy()
    c = p.mean(axis=0)
    n = np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0])
    flip = np.einsum("ij,ij->i", n, out.mean(axis=1) - c) < 0
    out[flip] = out[flip][:, ::-1]
    for i in range(len(out)):
        keys = [tuple(out[i, k]) for k in range(3)]
        k = min(range(3), key=lambda q: keys[q])
        out[i] = np.roll(out[i], -k, axis=0)
    order = sorted(range(len(out)), key=lambda i: tuple(out[i].reshape(-1)))
    return out[order], h


def hull_volume(points: np.ndarray) -> float:
    """Convex hull volume in the input coordinate unit cubed."""
    from scipy.spatial import ConvexHull
    p = np.unique(np.asarray(points, float).reshape(-1, 3).round(12), axis=0)
    if len(p) < 4:
        raise MeshError("too few distinct vertices for a hull")
    return float(ConvexHull(p).volume)


def _watertight_volume(tri: np.ndarray):
    """(volume, watertight). Signed-tetrahedron volume is exact for a consistently oriented closed mesh."""
    t = np.asarray(tri, float).reshape(-1, 3, 3)
    _, inv = np.unique(t.reshape(-1, 3).round(12), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    directed = np.concatenate((f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]))
    e = np.sort(directed, axis=1)
    _, groups, counts = np.unique(e, axis=0, return_inverse=True, return_counts=True)
    signs = np.where(directed[:, 0] < directed[:, 1], 1, -1)
    balanced = np.bincount(groups, weights=signs, minlength=len(counts))
    watertight = bool(len(counts) and np.all(counts == 2) and np.all(balanced == 0))
    v = abs(float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum())) / 6.0
    return v, watertight


def _voxel_volume(tri: np.ndarray, cells: int = 40) -> float:
    """Coarse deterministic fallback for open/non-watertight meshes.

    It rasterises a bounded quasi-random sample of surface triangles, seals one-voxel cracks and fills enclosed
    voxels. Sampling triangle interiors (not just vertices/centres) avoids badly underestimating large CAD facets.
    This is only the concavity heuristic; generated collision coverage is checked against every real vertex.
    """
    from scipy.ndimage import binary_dilation, binary_erosion, binary_fill_holes
    t = np.asarray(tri, float).reshape(-1, 3, 3)
    pts = t.reshape(-1, 3)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = hi - lo
    longest = float(span.max())
    if longest <= 0:
        return 0.0
    step = longest / max(int(cells), 8)
    dims = np.maximum(3, np.ceil(span / step).astype(int) + 3)
    surf = np.zeros(tuple(int(x) for x in dims), dtype=bool)
    if len(t) > 20_000:
        # Golden-ratio sequence is deterministic but avoids periodic aliasing with CAD triangle ordering.
        q = (np.arange(20_000) * 0.6180339887498949) % 1.0
        t = t[np.unique(np.floor(q * len(t)).astype(int))]
    bary = {}
    for face in t:
        edge = np.linalg.norm(face[[1, 2, 0]] - face[[0, 1, 2]], axis=1)
        n = max(1, min(80, int(np.ceil(float(edge.max()) / step * 1.4))))
        if n not in bary:
            bary[n] = np.asarray([(i / n, j / n, 1 - (i + j) / n)
                                  for i in range(n + 1) for j in range(n + 1 - i)], float)
        q = bary[n] @ face
        ijk = np.floor((q - lo) / step).astype(int) + 1
        ijk = np.clip(ijk, 0, np.asarray(surf.shape) - 1)
        surf[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
    # One dilation closes discretisation cracks; one erosion after filling roughly removes that added shell.
    filled = binary_fill_holes(binary_dilation(surf, iterations=1))
    filled = binary_erosion(filled, iterations=1, border_value=0)
    return float(filled.sum()) * step ** 3


def mesh_volume(tri: np.ndarray):
    """(estimated mesh volume, watertight). Uses exact signed volume when closed, coarse voxels otherwise."""
    t = np.asarray(tri, float).reshape(-1, 3, 3)
    if len(t) > AUTO_EXACT_VOLUME_TRIANGLES:
        span = float(np.ptp(t.reshape(-1, 3), axis=0).max())
        eps = max(span ** 3 * 1e-12, 1e-18)
        return max(_voxel_volume(t), eps), None               # exact edge pairing is too slow at CAD scale
    v, closed = _watertight_volume(tri)
    span = float(np.ptp(np.asarray(tri, float).reshape(-1, 3), axis=0).max())
    eps = max(span ** 3 * 1e-12, 1e-18)
    if closed and v > eps:
        return v, True
    return max(_voxel_volume(tri), eps), False


def _normal_order(hull, limit: int = 40) -> np.ndarray:
    """Source-hull support normals, largest surface facets first after six bounding planes."""
    fixed = np.array([[1., 0, 0], [-1., 0, 0], [0, 1., 0], [0, -1., 0], [0, 0, 1.], [0, 0, -1.]])
    n = np.asarray(hull.equations[:, :3], float)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-15)
    tri = hull.points[hull.simplices]
    area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) / 2
    key = n.round(6)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    weight = np.bincount(inv, weights=area)
    order = sorted(range(len(uniq)), key=lambda i: (-round(float(weight[i]), 12), tuple(uniq[i])))
    selected = [v for v in fixed]
    for i in order:
        if len(selected) >= max(6, int(limit)):
            break
        v = uniq[i] / (np.linalg.norm(uniq[i]) or 1.0)
        if max(float(v @ q) for q in selected) < 1 - 1e-7:
            selected.append(v)
    return np.asarray(selected)


def _support_values(points: np.ndarray, normals: np.ndarray, tolerance: float) -> np.ndarray:
    p = np.asarray(points, float).reshape(-1, 3)
    b = np.full(len(normals), -np.inf)
    for i in range(0, len(p), 50_000):
        b = np.maximum(b, np.max(p[i:i + 50_000] @ normals.T, axis=0))
    return b + max(float(tolerance), 0.0)


def _polytope_from_support(normals: np.ndarray, support: np.ndarray, interior: np.ndarray, epsilon: float = 0.0):
    from scipy.spatial import HalfspaceIntersection
    hs = np.column_stack((normals, -support))
    if epsilon:
        hs[:, 3] -= epsilon
    try:
        inter = HalfspaceIntersection(hs, interior).intersections
        return _stable_hull(inter)[0]
    except Exception as e:
        raise MeshError(f"cannot simplify collision hull ({e})") from None


def _polytope(points: np.ndarray, normals: np.ndarray, tolerance: float):
    """Convex polytope from conservative support planes n.x <= max(n.P)+tolerance."""
    p = np.asarray(points, float).reshape(-1, 3)
    b = _support_values(p, normals, tolerance)
    eps = max(float(np.ptp(p, axis=0).max()), 1.0) * 1e-10 if tolerance <= 0 else 0.0
    return _polytope_from_support(normals, b, p.mean(axis=0), eps)


def simplify_hull(points: np.ndarray, max_triangles: int = AUTO_MAX_TRIANGLES,
                  tolerance: float = 0.0) -> np.ndarray:
    """Conservative low-poly hull: selected support planes are pushed out to the furthest source vertex."""
    max_triangles = max(12, int(max_triangles))
    p = np.asarray(points, float).reshape(-1, 3)
    if len(p) <= 30_000:
        exact, h = _stable_hull(p)
        if len(exact) <= max_triangles:
            return exact
    else:
        idx = np.linspace(0, len(p) - 1, 12_000, dtype=int)
        ext = np.r_[np.argmin(p, axis=0), np.argmax(p, axis=0)]
        sample = p[np.unique(np.r_[idx, ext])]
        _, h = _stable_hull(sample)
    interior = p.mean(axis=0)
    eps = max(float(np.ptp(p, axis=0).max()), 1.0) * 1e-10 if tolerance <= 0 else 0.0
    cap = max(10, min(72, max_triangles // 2 + 8))
    normals = _normal_order(h, cap)
    support = _support_values(p, normals, tolerance)
    cap = min(len(normals), cap)
    counts = list(range(cap, 5, -2))
    if 6 not in counts:
        counts.append(6)
    for k in counts:
        q = _polytope_from_support(normals[:k], support[:k], interior, eps)
        if len(q) <= max_triangles:
            return q
    return _polytope_from_support(normals[:6], support[:6], interior, eps)


def _sample_tris(tri: np.ndarray, limit: int = AUTO_SEARCH_TRIANGLES) -> np.ndarray:
    if len(tri) <= limit:
        return tri
    t = np.asarray(tri)
    idx = np.linspace(0, len(t) - 1, limit, dtype=int)
    p = t.reshape(-1, 3)
    ext = np.r_[np.argmin(p, axis=0), np.argmax(p, axis=0)] // 3
    return t[np.unique(np.r_[idx, ext])]


def _search_hull_volume(tri: np.ndarray) -> float:
    t = np.asarray(tri, float).reshape(-1, 3, 3)
    q = t if len(t) <= AUTO_EXACT_VOLUME_TRIANGLES else _sample_tris(t)
    return hull_volume(q.reshape(-1, 3))


def _axes(tri: np.ndarray) -> List[np.ndarray]:
    """World + principal axes for deterministic split search."""
    p = _points(_sample_tris(tri))
    out = [np.eye(3)[i] for i in range(3)]
    if len(p) >= 4:
        _, vec = np.linalg.eigh(np.cov(p.T))
        for a in vec.T:
            a = a / (np.linalg.norm(a) or 1.0)
            j = int(np.argmax(np.abs(a)))
            if a[j] < 0:
                a = -a
            if max(abs(float(a @ q)) for q in out) < 1 - 1e-7:
                out.append(a)
    return out


def _best_split(tri: np.ndarray):
    """Best deterministic centroid cut by summed child hull volume; every source triangle stays whole."""
    t = np.asarray(tri, float).reshape(-1, 3, 3)
    if len(t) < 8:
        return None
    search = _sample_tris(t)
    c = search.mean(axis=1)
    best = None
    for ai, axis in enumerate(_axes(search)):
        z = c @ axis
        for qi, q in enumerate((0.25, 1 / 3, 0.5, 2 / 3, 0.75)):
            cut = float(np.quantile(z, q))
            left = z <= cut
            right = ~left
            if left.sum() < 4 or right.sum() < 4:
                continue
            try:
                score = hull_volume(search[left].reshape(-1, 3)) + hull_volume(search[right].reshape(-1, 3))
            except Exception:
                continue
            key = (round(score, 12), ai, qi)
            if best is None or key < best[0]:
                best = (key, axis, cut)
    if best is None:
        return None
    axis, cut = best[1], best[2]
    left = t.mean(axis=1) @ axis <= cut
    right = ~left
    if left.sum() < 4 or right.sum() < 4:
        return None
    return t[left], t[right]


def collision_coverage(vertices: np.ndarray, pieces: List[np.ndarray]) -> float:
    """Maximum outside distance of any vertex from the union of convex pieces (0 means covered)."""
    from scipy.spatial import ConvexHull
    p = np.asarray(vertices, float).reshape(-1, 3)
    outside = np.full(len(p), np.inf)
    for piece in pieces:
        h = ConvexHull(_points(piece))
        n, d = h.equations[:, :3], h.equations[:, 3]
        for i in range(0, len(p), 50_000):
            outside[i:i + 50_000] = np.minimum(outside[i:i + 50_000],
                np.maximum(0.0, (p[i:i + 50_000] @ n.T + d).max(axis=1)))
    return float(outside.max(initial=0.0))


def _piece_coverage(vertices: np.ndarray, piece: np.ndarray) -> float:
    """Outside distance for vertices assigned to one collision piece."""
    from scipy.spatial import ConvexHull
    p = np.asarray(vertices, float).reshape(-1, 3)
    h = ConvexHull(_points(piece))
    n, d = h.equations[:, :3], h.equations[:, 3]
    worst = 0.0
    for i in range(0, len(p), 50_000):
        outside = np.maximum(0.0, (p[i:i + 50_000] @ n.T + d).max(axis=1))
        worst = max(worst, float(outside.max(initial=0.0)))
    return worst


def auto_collision(tri: np.ndarray, max_pieces: int = AUTO_MAX_PIECES,
                   max_triangles: int = AUTO_MAX_TRIANGLES, tolerance: float = 0.0,
                   concavity: float = AUTO_CONCAVITY):
    """Deterministic conservative convex decomposition using NumPy + SciPy only."""
    t = np.asarray(tri, float).reshape(-1, 3, 3)
    max_pieces = min(16, max(1, int(max_pieces)))
    max_triangles = min(256, max(16, int(max_triangles)))
    concavity = max(1.0, float(concavity))
    mesh_v, watertight = mesh_volume(t)
    whole_v = _search_hull_volume(t)
    if watertight is not True:
        mesh_v = min(mesh_v, whole_v)                         # a mesh cannot exceed its convex hull physically
    clusters = [t]
    exact_volumes = [whole_v]
    while len(clusters) < max_pieces and sum(exact_volumes) / mesh_v > concavity:
        options = []
        for i, c in enumerate(clusters):
            split = _best_split(c)
            if split is None:
                continue
            try:
                va = _search_hull_volume(split[0])
                vb = _search_hull_volume(split[1])
            except MeshError:
                continue
            improvement = exact_volumes[i] - (va + vb)
            options.append((round(-improvement, 12), i, split, va, vb))
        if not options:
            break
        _, i, split, va, vb = min(options, key=lambda x: (x[0], x[1]))
        if exact_volumes[i] - (va + vb) <= max(exact_volumes[i] * 1e-4, 1e-15):
            break
        clusters[i:i + 1] = [split[0], split[1]]
        exact_volumes[i:i + 1] = [va, vb]
    pieces = [simplify_hull(c.reshape(-1, 3), max_triangles, tolerance) for c in clusters]
    piece_v = [hull_volume(p.reshape(-1, 3)) for p in pieces]
    cov = max((_piece_coverage(c.reshape(-1, 3), p) for c, p in zip(clusters, pieces)), default=0.0)
    return pieces, {
        "pieces": len(pieces), "triangles": int(sum(len(p) for p in pieces)),
        "triangles_each": [int(len(p)) for p in pieces],
        "mesh_volume": float(mesh_v), "hull_volume": float(whole_v),
        "collision_volume": float(sum(piece_v)), "hull_ratio": float(whole_v / mesh_v),
        "volume_ratio": float(sum(piece_v) / mesh_v), "watertight": watertight,
        "coverage": cov,
    }
