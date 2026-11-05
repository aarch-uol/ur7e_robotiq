"""Automatic conservative collision meshes: NumPy/SciPy only, deterministic and light."""
from pathlib import Path

import numpy as np

from moveit_gui import meshes as M
from moveit_gui import parts as P
from moveit_gui.robot_model import parse_urdf


def box(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                  [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                  [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    return v[f]


def voxel_shape(cells, size=10.0):
    cells = set(cells)
    faces = [
        ((-1, 0, 0), [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)]),
        ((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)]),
        ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)]),
        ((0, 1, 0), [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)]),
        ((0, 0, -1), [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)]),
        ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]),
    ]
    out = []
    for cell in sorted(cells):
        for delta, corners in faces:
            neighbour = tuple(cell[i] + delta[i] for i in range(3))
            if neighbour in cells:
                continue
            p = size * np.array([[cell[i] + q[i] for i in range(3)] for q in corners], float)
            out += [p[[0, 1, 2]], p[[0, 2, 3]]]
    return np.asarray(out)


def cylinder(n=128, radius=20.0, height=50.0):
    # Four triangles per segment (two side + one per cap): n=128 -> exactly 512 triangles.
    a = np.arange(n) * 2 * np.pi / n
    bottom = np.c_[radius * np.cos(a), radius * np.sin(a), np.zeros(n)]
    top = bottom.copy()
    top[:, 2] = height
    out = []
    for i in range(n):
        j = (i + 1) % n
        out += [np.array([bottom[i], bottom[j], top[j]]), np.array([bottom[i], top[j], top[i]]),
                np.array([[0, 0, 0], bottom[j], bottom[i]]),
                np.array([[0, 0, height], top[i], top[j]])]
    return np.asarray(out)


def assert_pieces(source, pieces, info, max_triangles=64, tolerance=0.5):
    assert pieces
    assert all(len(p) <= max_triangles for p in pieces)
    assert sum(len(p) for p in pieces) <= 8 * max_triangles
    assert M.collision_coverage(source.reshape(-1, 3), pieces) <= tolerance + 1e-7
    for piece in pieces:
        centre = piece.reshape(-1, 3).mean(axis=0)
        normal = np.cross(piece[:, 1] - piece[:, 0], piece[:, 2] - piece[:, 0])
        assert (np.einsum("ij,ij->i", normal, piece.mean(axis=1) - centre) > 0).all()
    assert info["coverage"] <= tolerance + 1e-7


def deterministic(source, tmp_path: Path):
    a, _ = M.auto_collision(source, tolerance=0.5)
    b, _ = M.auto_collision(source, tolerance=0.5)
    assert len(a) == len(b)
    for i, (pa, pb) in enumerate(zip(a, b)):
        fa, fb = tmp_path / f"a{i}.stl", tmp_path / f"b{i}.stl"
        M.write_stl(fa, pa)
        M.write_stl(fb, pb)
        assert fa.read_bytes() == fb.read_bytes()


def test_cube_is_one_exact_light_piece_and_deterministic(tmp_path):
    src = box([0, 0, 0], [20, 20, 20])
    pieces, info = M.auto_collision(src, tolerance=0.5)
    assert len(pieces) == 1 and len(pieces[0]) <= 12
    assert info["volume_ratio"] == 1.0
    assert_pieces(src, pieces, info)
    deterministic(src, tmp_path)


def test_512_triangle_cylinder_is_one_conservative_piece(tmp_path):
    src = cylinder()
    assert len(src) == 512
    pieces, info = M.auto_collision(src, tolerance=0.5)
    assert len(pieces) == 1 and len(pieces[0]) <= 64
    assert_pieces(src, pieces, info)
    deterministic(src, tmp_path)


def test_l_bracket_splits_and_is_much_tighter_than_one_hull(tmp_path):
    src = voxel_shape([(x, 0, 0) for x in range(4)] + [(0, y, 0) for y in range(1, 4)])
    pieces, info = M.auto_collision(src, tolerance=0.5)
    assert len(pieces) >= 2
    assert info["volume_ratio"] < info["hull_ratio"] * 0.8
    assert_pieces(src, pieces, info)
    deterministic(src, tmp_path)


def test_u_shape_needs_at_least_three_pieces(tmp_path):
    src = voxel_shape([(x, 0, 0) for x in range(5)] +
                      [(0, y, 0) for y in range(1, 5)] + [(4, y, 0) for y in range(1, 5)])
    pieces, info = M.auto_collision(src, tolerance=0.5)
    assert len(pieces) >= 3
    assert info["volume_ratio"] < info["hull_ratio"] * 0.8
    assert_pieces(src, pieces, info)
    deterministic(src, tmp_path)


def test_auto_part_uses_generated_pieces_in_live_model_and_urdf(tmp_path):
    src = voxel_shape([(x, 0, 0) for x in range(4)] + [(0, y, 0) for y in range(1, 4)])
    raw = tmp_path / "source.stl"
    M.write_stl(raw, src)
    store = P.PartStore(tmp_path / "parts" / "p.yaml")
    assert store.add_mesh("bracket.stl", raw.read_bytes()) == "bracket.stl"
    part = P.make_part("bracket", "tool0", shape={"type": "mesh", "file": "bracket.stl", "scale": 0.001},
                       collision="auto")
    files, info = store.auto_collision_files(part)
    assert len(files) >= 2
    assert all(p.parent.name == "collision" and p.name.startswith("bracket.stl.") and p.suffix == ".stl"
               for p in files)
    store.put(part, {"base_link", "tool0"})

    model = parse_urdf("""<robot name="t"><link name="base_link"/><link name="tool0"/>
      <joint name="j" type="revolute"><parent link="base_link"/><child link="tool0"/>
      <axis xyz="0 0 1"/></joint></robot>""")
    _, objects = P.live_plan(store, model)
    assert len(objects) == 1 and len(objects[0]["geoms"]) == len(files)
    assert all(x["geom"]["type"] == "mesh" for x in objects[0]["geoms"])
    merged = P.merged_model(store, model)
    assert len(merged["collisions"]["bracket"]) == len(files)

    urdf = P.generate_urdf(store, model)
    assert urdf.count("<collision>") == len(files)
    for f in files:
        assert f.name in urdf
    inner = urdf.split('<robot xmlns:xacro="http://wiki.ros.org/xacro">')[1].rsplit("</robot>", 1)[0]
    parsed = parse_urdf('<robot name="t"><link name="base_link"/><link name="tool0"/>' +
                        '<joint name="j" type="fixed"><parent link="base_link"/><child link="tool0"/></joint>' +
                        inner + "</robot>")
    assert len(parsed["collisions"]["bracket"]) == len(files)
    assert store.auto_collision_info(part)["triangles"] == info["triangles"]

    # Backward compatibility: a saved/constructed mesh with no collision field still means the old one-hull mode.
    legacy = P.make_part("legacy", "tool0", shape={"type": "mesh", "file": "bracket.stl", "scale": 0.001})
    assert legacy.collision == "hull"


def test_auto_cache_reuses_same_settings_and_invalidates_on_settings_or_mesh_change(tmp_path):
    src = voxel_shape([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    raw = tmp_path / "source.stl"
    M.write_stl(raw, src)
    store = P.PartStore(tmp_path / "parts" / "p.yaml")
    store.add_mesh("shape.stl", raw.read_bytes())
    part = P.make_part("shape", "tool0", shape={"type": "mesh", "file": "shape.stl", "scale": 0.001},
                       collision="auto")
    files, _ = store.auto_collision_files(part)
    first = [p.read_bytes() for p in files]
    assert store.auto_collision_files(part)[0] == files

    changed = P.make_part("shape", "tool0", shape=part.shape, collision="auto",
                          collision_settings={"tolerance_mm": 0.7})
    files2, _ = store.auto_collision_files(changed)
    assert files2[0].name != files[0].name                 # settings hash changes the generated URI

    M.write_stl(raw, src * 1.1)
    store.add_mesh("shape.stl", raw.read_bytes())
    assert store.auto_collision_cached(part) is None       # same name/settings, newer source must regenerate
    files3, _ = store.auto_collision_files(part)
    assert files3 == files and [p.read_bytes() for p in files3] != first
