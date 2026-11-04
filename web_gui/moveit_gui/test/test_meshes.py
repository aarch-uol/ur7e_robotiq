"""Mesh files (STL / OBJ / DAE) and the collision shape of a part (visual / box / hull / none)."""
import struct

import numpy as np
import pytest

from moveit_gui import meshes as M
from moveit_gui import parts as P
from moveit_gui.robot_model import parse_urdf

CUBE_OBJ = b"""# a 2 cm cube, quads, one negative index
v 0 0 0
v 2 0 0
v 2 2 0
v 0 2 0
v 0 0 2
v 2 0 2
v 2 2 2
v 0 2 2
f 1 4 3 2
f 5 6 7 8
f 1 2 6 5
f 2/1 3/1 7/1 6/1
f 3//1 4//1 8//1 7//1
f 4 1 5 -1
"""

DAE = b"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="millimeter" meter="0.001"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="g"><mesh>
      <source id="pos"><float_array id="pa" count="12">0 0 0 100 0 0 0 100 0 0 0 100</float_array></source>
      <vertices id="v"><input semantic="POSITION" source="#pos"/></vertices>
      <triangles count="2"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#n" offset="1"/>
        <p>0 0 1 0 2 0  0 0 2 0 3 0</p></triangles>
      <polylist count="1"><input semantic="VERTEX" source="#v" offset="0"/><vcount>4</vcount><p>0 1 2 3</p></polylist>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="s">
    <node id="n"><translate>0 0 50</translate><instance_geometry url="#g"/></node>
  </visual_scene></library_visual_scenes>
</COLLADA>"""


def stl_bytes(tris) -> bytes:
    out = b"\0" * 80 + struct.pack("<I", len(tris))
    for t in tris:
        out += struct.pack("<12fH", 0, 0, 0, *[c for v in t for c in v], 0)
    return out


def test_obj_quads_negative_indices_and_bad_faces():
    t = M.read_mesh_bytes(CUBE_OBJ, "cube.obj")
    assert t.shape == (12, 3, 3)
    P_ = t.reshape(-1, 3)
    assert np.allclose(P_.min(0), 0) and np.allclose(P_.max(0), 2)
    with pytest.raises(M.MeshError, match="does not exist"):
        M.read_mesh_bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n", "x.obj")
    with pytest.raises(M.MeshError, match="no faces"):
        M.read_mesh_bytes(b"v 0 0 0\n", "x.obj")


def test_dae_unit_node_transform_and_polylist():
    t = M.read_mesh_bytes(DAE, "x.dae")
    assert t.shape == (4, 3, 3)                       # 2 triangles + a quad as 2
    P_ = t.reshape(-1, 3)
    assert np.allclose(P_.min(0), [0, 0, 0.05]) and np.allclose(P_.max(0), [0.1, 0.1, 0.15])   # mm -> m, +50 mm
    with pytest.raises(M.MeshError, match="XML"):
        M.read_mesh_bytes(b"<COLLADA", "x.dae")


def test_stl_ascii_and_suffixes():
    a = b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid\n"
    assert M.read_mesh_bytes(a, "a.STL").shape == (1, 3, 3)
    with pytest.raises(M.MeshError, match="use .stl"):
        M.read_mesh_bytes(a, "a.ply")
    assert M.unit_hint([120, 40, 30]) == "mm" and M.unit_hint([0.12, 0.04, 0.03]) == "m"


def test_hull_is_closed_and_outward(tmp_path):
    t = M.read_mesh_bytes(CUBE_OBJ, "cube.obj")
    h = M.convex_hull(np.concatenate([t, t * 0.5 + 0.5]))   # interior points do not change the hull
    c = h.reshape(-1, 3).mean(0)
    n = np.cross(h[:, 1] - h[:, 0], h[:, 2] - h[:, 0])
    assert (np.einsum("ij,ij->i", n, h.mean(1) - c) > 0).all()
    vol = abs(np.einsum("ij,ij->i", h[:, 0], np.cross(h[:, 1], h[:, 2])).sum()) / 6
    assert vol == pytest.approx(8.0)
    M.write_stl(tmp_path / "h.stl", h)
    assert M.read_mesh(tmp_path / "h.stl").shape == h.shape


MODEL = parse_urdf("""<robot name="t"><link name="base_link"/><link name="tool0"/>
  <joint name="j" type="revolute"><parent link="base_link"/><child link="tool0"/><axis xyz="0 0 1"/>
  <limit lower="-1" upper="1"/></joint></robot>""")


def store(tmp_path):
    s = P.PartStore(tmp_path / "parts" / "p.yaml")
    s.add_mesh("cube.obj", CUBE_OBJ)
    s.add_mesh("thing.dae", DAE)
    return s


def test_collision_modes_in_urdf_live_plan_and_status(tmp_path):
    s = store(tmp_path)
    links = set(MODEL["links"])
    assert s.meshes() == ["cube.obj", "thing.dae"]
    assert s.mesh_info("cube.obj")["hint"] == "" and s.mesh_info("thing.dae")["unit"] == "m"
    # defaults: a mesh collides with its hull, a primitive with itself, a frame with nothing; bools of old files
    assert P.make_part("a", "tool0", shape={"type": "mesh", "file": "cube.obj"}).collision == "hull"
    assert P.make_part("a", "tool0", shape={"type": "box", "size_mm": [1, 1, 1]}).collision == "visual"
    assert P.make_part("a", "tool0", shape={"type": "box", "size_mm": [1, 1, 1]}, collision=False).collision == "none"
    assert P.make_part("a", "tool0", collision="box").collision == "none"
    assert P.make_part("a", "tool0", shape={"type": "sphere", "radius_mm": 5}, collision="hull").collision == "visual"
    assert P.make_part("a", "tool0", shape={"type": "mesh", "file": "thing.dae"}).shape["scale"] == 1.0
    with pytest.raises(P.PartError, match="collision"):
        P.make_part("a", "tool0", collision="maybe")

    s.put(P.make_part("hulled", "tool0", shape={"type": "mesh", "file": "cube.obj", "scale": 0.01,
                                                 "offset_mm": [0, 0, 5]}), links)
    s.put(P.make_part("boxed", "tool0", shape={"type": "mesh", "file": "cube.obj", "scale": 0.01},
                      collision="box"), links)
    s.put(P.make_part("ghost", "tool0", shape={"type": "mesh", "file": "thing.dae"}, collision="none"), links)
    assert s.extent_m(s.parts["hulled"]) == pytest.approx((0.005, 0.025))      # offset + 2 cm cube
    frames, objs = P.live_plan(s, MODEL)
    by = {o["id"]: o for o in objs}
    assert set(by) == {"gui_part/hulled", "gui_part/boxed"}                      # none: shown, not collided with
    assert by["gui_part/hulled"]["geom"]["uri"].endswith("hulls/cube.obj.hull.stl")
    assert np.allclose(by["gui_part/hulled"]["T_link_obj"][:3, 3], [0, 0, 0.005])
    assert by["gui_part/boxed"]["geom"] == {"type": "box", "size": pytest.approx([0.02, 0.02, 0.02])}
    assert np.allclose(by["gui_part/boxed"]["T_link_obj"][:3, 3], [0.01, 0.01, 0.01])  # box centred on the mesh
    hull = s.generated_dir / "hulls" / "cube.obj.hull.stl"
    assert M.read_mesh(hull).shape == (12, 3, 3)

    urdf = P.generate_urdf(s, MODEL)
    assert "thing.dae" in urdf and 'gui_part_ghost_colour' not in urdf          # a .dae keeps its colours
    assert urdf.count("<collision>") == 2 and "cube.obj.hull.stl" in urdf and '<box size="0.02 0.02 0.02"/>' in urdf
    # the stack restarted with the generated URDF: in_model; a changed collision mode shows as changed
    inner = urdf.split('<robot xmlns:xacro="http://wiki.ros.org/xacro">')[1].rsplit("</robot>", 1)[0]
    m2 = parse_urdf('<robot name="t"><link name="base_link"/><link name="tool0"/><joint name="j" type="revolute">'
                    '<parent link="base_link"/><child link="tool0"/><axis xyz="0 0 1"/></joint>' + inner + "</robot>")
    assert set(P.status(s, m2).values()) == {"in_model"}
    s.put(P.make_part("boxed", "tool0", shape={"type": "mesh", "file": "cube.obj", "scale": 0.01},
                      collision="hull"), links, replace="boxed")
    assert P.status(s, m2)["boxed"] == "changed"
    # the 3D views get the collision shapes of live parts
    mm = P.merged_model(s, MODEL)
    assert mm["collisions"]["hulled"][0]["geometry"]["type"] == "mesh" and mm["collisions"]["ghost"] == []


def test_mesh_upload_rules(tmp_path):
    s = P.PartStore(tmp_path / "parts" / "p.yaml")
    with pytest.raises(P.PartError, match=".stl, .obj or .dae"):
        s.add_mesh("x.step", b"ISO-10303-21;")
    with pytest.raises(P.PartError, match="OBJ"):
        s.add_mesh("x.obj", b"hello")
    assert s.meshes() == []                                     # a rejected upload leaves nothing behind
    assert s.add_mesh("../../evil name.obj", CUBE_OBJ) == "evil_name.obj"
    old = P.make_part("p", "tool0", shape={"type": "box", "size_mm": [1, 1, 1]}, collision=True)
    assert old.to_yaml()["collision"] == "visual"
