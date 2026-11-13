"""SETUP > Parts, Camera calibration, Setups and restart: through the web service on the fake robot."""
import base64
import json
import os
import struct
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from moveit_gui import fake_bridge as fb
from moveit_gui import handeye as he
from moveit_gui import parts as P
from moveit_gui import transforms as tf
from moveit_gui.core import RobotCore
from moveit_gui.fake_bridge import FakeBridge
from moveit_gui.profile import RobotProfile
from moveit_gui.robot_model import parse_urdf
from moveit_gui.web.server import Server
from moveit_gui.web.service import CommandError, WebService
from moveit_gui.web.workcell import RESTART_GUI, RESTART_STACK

ROOT = os.path.join(os.path.dirname(__file__), "..")

# base_link -(revolute)- wrist_3_link - tool0 - robotiq_85_base_link - robotiq_tcp; a finger on a revolute joint
URDF = """<robot name="t">
  <link name="world"/><link name="base_link"/><link name="wrist_3_link"/><link name="tool0"/>
  <link name="robotiq_85_base_link"/><link name="robotiq_tcp"/><link name="finger"/>
  <joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/></joint>
  <joint name="arm" type="revolute"><parent link="base_link"/><child link="wrist_3_link"/>
    <origin xyz="0.4 0 0.3"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="flange" type="fixed"><parent link="wrist_3_link"/><child link="tool0"/></joint>
  <joint name="grip" type="fixed"><parent link="tool0"/><child link="robotiq_85_base_link"/>
    <origin xyz="0 0 0.061"/></joint>
  <joint name="tcp" type="fixed"><parent link="robotiq_85_base_link"/><child link="robotiq_tcp"/>
    <origin xyz="0 0 0.1571"/></joint>
  <joint name="knuckle" type="revolute"><parent link="robotiq_85_base_link"/><child link="finger"/>
    <axis xyz="0 1 0"/><limit lower="0" upper="0.8"/></joint>
</robot>"""


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    bridge.fake_robot_description = URDF
    (tmp_path / "progs").mkdir()
    (tmp_path / "pol").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    s = WebService(core)
    s.take_lease("me")
    s.tick()
    yield s, bridge, tmp_path
    s.shutdown()
    bridge.shutdown()


def run(s, cmd, **args):
    return s.command("me", cmd, args)


def wait(cond, timeout=20.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def synced(s):
    s.tick()
    return wait(lambda: (s.tick() or True) and not s._parts_busy and s.parts_sync["state"] == "ok", 5)


def stl(tris) -> bytes:
    out = b"\0" * 80 + struct.pack("<I", len(tris))
    for t in tris:
        out += struct.pack("<12fH", 0, 0, 0, *[c for v in t for c in v], 0)
    return out


# ================================================================== parts
def test_parts_go_live_in_tf_moveit_and_the_3d_model(env):
    s, b, _ = env
    run(s, "parts_save", name="bracket", parent="tool0", shape={"type": "box", "size_mm": [40, 90, 25]},
        xyz_mm=[0, -70, 0])
    # a child on the bracket starts on its far face by default (stacking)
    assert s.parts.stack_offset_mm("bracket") == [0.0, 0.0, 25.0]
    run(s, "parts_save", name="cam", parent="bracket", xyz_mm=s.parts.stack_offset_mm("bracket"),
        shape={"type": "cylinder", "radius_mm": 10, "length_mm": 30})
    run(s, "parts_save", name="table", parent="base_link", shape={"type": "box", "size_mm": [1000, 800, 20]},
        anchor="center", xyz_mm=[0, 0, -11])
    assert synced(s), s.parts_sync
    # TF: tool0 -> bracket -> cam
    T = b.lookup_transform("tool0", "cam")
    assert np.allclose(T[:3, 3], [0, -0.070, 0.025], atol=1e-9)
    objs = b.part_objects
    assert objs["gui_part/bracket"]["attach"] and objs["gui_part/bracket"]["link"] == "tool0"
    assert objs["gui_part/cam"]["link"] == "tool0"                      # carried by the link it is bolted to
    assert np.allclose(objs["gui_part/cam"]["T_link_obj"][:3, 3], [0, -0.070, 0.040])   # cylinder centred
    assert not objs["gui_part/table"]["attach"] and objs["gui_part/table"]["link"] == "base_link"
    assert "wrist_3_link" in objs["gui_part/bracket"]["touch_links"]
    m = s.robot_model()
    assert "cam" in m["links"] and any(j["child"] == "cam" for j in m["joints"])
    v = s.parts_view()
    assert {p["name"]: p["status"] for p in v["parts"]} == {"bracket": "live", "cam": "live", "table": "live"}
    assert set(v["restart_for"]) == {"bracket", "cam", "table"}


def test_parts_rules(env):
    s, _, _ = env
    with pytest.raises(CommandError, match="robot model"):
        run(s, "parts_save", name="tool0", parent="base_link")
    with pytest.raises(CommandError, match="unknown parent"):
        run(s, "parts_save", name="a", parent="nowhere")
    run(s, "parts_save", name="a", parent="tool0")
    run(s, "parts_save", name="b", parent="a")
    with pytest.raises(CommandError, match="carry its own parent"):
        run(s, "parts_save", name="a", parent="b", replace="a")
    with pytest.raises(CommandError, match="sit"):
        run(s, "parts_delete", name="a")
    run(s, "parts_save", name="a2", parent="tool0", replace="a")         # rename: children follow
    assert s.parts.parts["b"].parent == "a2"
    with pytest.raises(CommandError, match="snake"):
        run(s, "parts_save", name="Bad Name", parent="tool0")


def test_generated_urdf_srdf_and_the_restart_states(env):
    s, b, tmp = env
    run(s, "parts_save", name="spatula", parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 150]})
    gen = tmp / "cfg" / "moveit_gui" / "parts" / "generated"
    urdf = (gen / P.GENERATED_URDF).read_text()
    srdf = (gen / P.GENERATED_SRDF).read_text()
    assert '<joint name="gui_part_spatula_joint" type="fixed"><parent link="robotiq_tcp"/>' in urdf
    assert 'link1="robotiq_85_base_link" link2="spatula"' in srdf          # bolted to the gripper stack
    assert 'link2="spatula"' in srdf and "finger" in srdf                   # the finger is one joint away
    # what the stack would publish after a restart: the robot's URDF with the include
    inner = urdf.split('<robot xmlns:xacro="http://wiki.ros.org/xacro">')[1].rsplit("</robot>", 1)[0]
    b.fake_robot_description = URDF.replace("</robot>", inner + "</robot>")
    s.tick()
    assert s.parts_view()["parts"][0]["status"] == "in_model"
    assert synced(s)
    assert "gui_part/spatula" not in b.part_objects                         # the model has it: no live copy
    run(s, "parts_save", name="spatula", parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 160]})
    assert s.parts_view()["parts"][0]["status"] == "changed"
    run(s, "parts_delete", name="spatula")
    assert s.parts_view()["removed"] == ["spatula"]
    # a part whose parent link is not in the robot model is left out of the generated URDF (it would break it)
    s.parts.parts["ghost"] = P.make_part("ghost", parent="no_such_link")
    s.parts.save()
    run(s, "parts_save", name="ok_part", parent="tool0")
    urdf = (gen / P.GENERATED_URDF).read_text()
    assert "ok_part" in urdf and "ghost" not in urdf
    assert any("no_such_link" in x for x in s.parts_view()["problems"])


def test_mesh_upload_and_mesh_part(env):
    s, b, _ = env
    tri = [[(0, 0, 0), (10, 0, 0), (0, 10, 0)], [(0, 0, 0), (0, 10, 0), (0, 0, 40)]]
    r = run(s, "parts_mesh_upload", filename="holder v2.STL", data_b64=base64.b64encode(stl(tri)).decode())
    assert r["file"] == "holder_v2.STL" and r["size_file_units"] == [10, 10, 40]
    run(s, "parts_save", name="holder", parent="tool0", shape={"type": "mesh", "file": "holder_v2.STL", "scale": 0.001})
    assert s.parts.stack_offset_mm("holder") == [0, 0, 40.0]
    assert synced(s)
    assert b.part_objects["gui_part/holder"]["geom"]["type"] == "mesh"
    with pytest.raises(CommandError, match="STL"):
        run(s, "parts_mesh_upload", filename="x.stl", data_b64=base64.b64encode(b"hello").decode())
    assert P.read_stl(s.part_mesh_file("holder_v2.STL")).shape == (2, 3, 3)
    assert s.part_mesh_file("../calibrations/x.stl") is None


def test_auto_mesh_preview_and_async_save(env):
    s, b, tmp = env
    v = [(0, 0, 0), (20, 0, 0), (20, 20, 0), (0, 20, 0),
         (0, 0, 20), (20, 0, 20), (20, 20, 20), (0, 20, 20)]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    data = stl([[v[a], v[b], v[c]] for a, b, c in f])
    run(s, "parts_mesh_upload", filename="cube.stl", data_b64=base64.b64encode(data).decode())
    preview = run(s, "parts_collision_preview", file="cube.stl", scale=0.001,
                  collision_settings={"max_pieces": 8, "max_triangles": 64, "tolerance_mm": 0.5})
    assert preview["generating"] and "generating collision" in preview["msg"]
    assert wait(lambda: s._collision_jobs.get(preview["collision_job"], {}).get("state") != "generating", 5)
    preview = run(s, "parts_collision_preview_status", job=preview["collision_job"])
    assert preview["info"]["pieces"] == 1 and preview["info"]["triangles"] <= 12
    assert len(preview["preview"]) == 1
    rel = "collision/" + Path(preview["preview"][0]["uri"]).name
    assert s.part_mesh_file(rel).is_file()

    r = run(s, "parts_save", name="cube", parent="tool0",
            shape={"type": "mesh", "file": "cube.stl", "scale": 0.001}, collision="auto",
            collision_settings={"max_pieces": 8, "max_triangles": 64, "tolerance_mm": 0.5})
    assert not r.get("generating")                         # preview cache: save does not pretend to regenerate
    assert s.parts.parts["cube"].collision == "auto"
    assert s.parts_view()["parts"][0]["collision_info"]["pieces"] == 1
    assert synced(s)
    assert len(b.part_objects["gui_part/cube"]["geoms"]) == 1
    urdf = (tmp / "cfg" / "moveit_gui" / "parts" / "generated" / P.GENERATED_URDF).read_text()
    assert urdf.count("<collision>") == 1 and "/generated/collision/" in urdf

    # A cache miss is generated away from the request thread and the part appears only after it is ready.
    r = run(s, "parts_save", name="cube_async", parent="tool0",
            shape={"type": "mesh", "file": "cube.stl", "scale": 0.001}, collision="auto",
            collision_settings={"max_pieces": 8, "max_triangles": 64, "tolerance_mm": 0.6})
    assert r["generating"] and "generating collision" in r["msg"]
    assert "cube_async" not in s.parts.parts
    assert wait(lambda: s._collision_jobs.get("cube_async", {}).get("state") != "generating", 5)
    assert s._collision_jobs["cube_async"]["state"] == "done" and "cube_async" in s.parts.parts


def test_tcp_from_a_part_follows_the_part(env):
    s, _, _ = env
    run(s, "parts_save", name="spatula", parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 150]},
        xyz_mm=[0, 0, -20])
    run(s, "parts_make_tcp", name="spatula", tcp="spatula_tip")
    t = s.core.store.tcps["spatula_tip"]
    assert t.parent == "tool0" and np.allclose(t.T_parent_tcp[:3, 3], [0, 0, 0.061 + 0.1571 - 0.020 + 0.150])
    run(s, "parts_save", name="spatula", parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 170]},
        xyz_mm=[0, 0, -20])
    assert np.isclose(s.core.store.tcps["spatula_tip"].T_parent_tcp[2, 3], 0.061 + 0.1571 - 0.020 + 0.170)
    run(s, "parts_save", name="pad", parent="finger", shape={"type": "box", "size_mm": [5, 5, 5]})
    with pytest.raises(CommandError, match="rigidly"):
        run(s, "parts_make_tcp", name="pad", tcp="pad_tcp")


def test_parts_wait_while_the_arm_moves(env):
    s, b, _ = env
    s.core.busy = True
    run(s, "parts_save", name="a", parent="tool0", shape={"type": "sphere", "radius_mm": 10})
    s.tick()
    assert s.parts_sync["state"] == "pending" and "gui_part/a" not in b.part_objects
    s.core.busy = False
    assert synced(s) and "gui_part/a" in b.part_objects


# ================================================================== hand-eye
def test_handeye_eye_in_hand_on_the_fake_wrist_camera(env):
    s, b, _ = env
    b.time_scale = 20.0                                                 # fast fake moves
    with pytest.raises(CommandError, match="CameraInfo"):
        run(s, "handeye_start", topic="/zed/zed_node/rgb/color/rect/image")
    run(s, "handeye_start", topic=fb.FAKE_CAM_TOPIC)                   # info topic found next to the image
    look = run(s, "handeye_look")
    assert look["detection"]["ok"] and look["jpeg"]
    run(s, "handeye_plan", tilt_deg=20)
    r = run(s, "handeye_collect")
    assert wait(lambda: not s.he["progress"]["running"], 60)
    res = s.results[-1]
    assert res["ok"], res
    assert len(s.he["samples"]) >= 10 and s.he["result"]["verdict"] == "good"
    truth = fb.FAKE_T_TOOL0_CAMLINK @ fb.FAKE_T_LINK_OPTICAL
    E = tf.inv(s.he["result"]["X"]) @ truth
    assert np.linalg.norm(E[:3, 3]) < 0.003 and np.degrees(tf.rotation_angle(E[:3, :3])) < 0.3
    out = run(s, "handeye_save", name="wrist_rs")
    # the part is the driver's camera_link (parent of the optical frame), placed so the optical frame is right
    p = s.parts.parts["camera_link"]
    assert p.parent == "tool0" and p.kind == "camera" and p.meta["source"] == "handeye"
    E2 = tf.inv(p.T_parent_part()) @ fb.FAKE_T_TOOL0_CAMLINK
    assert np.linalg.norm(E2[:3, 3]) < 0.003, out
    assert synced(s)
    T = b.lookup_transform("tool0", fb.FAKE_CAM_OPTICAL)                # the camera tree hangs on the robot now
    assert T is not None and np.linalg.norm(T[:3, 3] - truth[:3, 3]) < 0.003
    saved = s.cams.view()
    assert saved[0]["name"] == "wrist_rs" and saved[0]["samples"] == len(s.he["samples"])
    # editing the calibrated offset drops the calibration mark
    run(s, "parts_save", name="camera_link", parent="tool0", xyz_mm=[0, 0, 0], rpy_deg=[0, 0, 0], kind="camera",
        replace="camera_link")
    assert "source" not in s.parts.parts["camera_link"].meta


def test_handeye_needs_varied_poses(env):
    s, _, _ = env
    run(s, "handeye_start", topic=fb.FAKE_CAM_TOPIC)
    for _ in range(5):
        run(s, "handeye_capture")
    with pytest.raises(CommandError, match="tilt the tool more"):
        run(s, "handeye_solve")


def test_board_svg_and_rules():
    svg = he.board_svg(he.DEFAULT_BOARD)
    assert svg.startswith("<svg") and 'width="265.0mm"' in svg
    with pytest.raises(he.HandEyeError, match="odd"):
        he.check_board({"squares_y": 6})


# ================================================================== setups + restart
def test_setups_save_load_restore_and_restart(env):
    s, _, tmp = env
    restarts = []
    s.on_restart = lambda: restarts.append(s.restart_code)
    run(s, "parts_save", name="spatula", parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 150]})
    run(s, "tcp_save", name="tip_a", parent="tool0", xyz_mm=[0, 0, 200], rpy_deg=[0, 0, 0])
    r = run(s, "setup_save", name="scraping", note="spatula")
    assert "parts" in s.setups.list()[0]["holds"]
    with pytest.raises(CommandError, match="exists"):
        run(s, "setup_save", name="scraping")
    run(s, "parts_delete", name="spatula")
    run(s, "tcp_delete", name="tip_a")
    s.core.busy = True
    with pytest.raises(CommandError, match="busy"):
        run(s, "setup_load", name="scraping")
    s.core.busy = False
    r = run(s, "setup_load", name="scraping")
    assert r["restart"] == "stack"                                    # its part is not in the running model
    assert wait(lambda: restarts == [RESTART_STACK], 3)
    cfg = tmp / "cfg" / "moveit_gui"
    assert "spatula" in (cfg / "parts" / "ur7e_lab.yaml").read_text()
    assert "tip_a" in (cfg / "calibrations" / "ur7e_lab.yaml").read_text()
    autos = [x for x in s.setups.list() if x["auto"]]
    assert len(autos) == 1 and "spatula" not in (s.setups.dir / autos[0]["name"] / "files" / "parts" /
                                                 "ur7e_lab.yaml").read_text()


def test_setup_current_changes_and_preview(env):
    s, _, _ = env
    assert run(s, "setup_list")["current"]["name"] == ""
    run(s, "parts_save", name="spatula", parent="robotiq_tcp", shape={"type": "box", "size_mm": [20, 3, 150]})
    run(s, "tcp_save", name="tip_a", parent="tool0", xyz_mm=[0, 0, 200], rpy_deg=[0, 0, 0])
    cur = run(s, "setup_save", name="scraping")["current"]
    assert cur["name"] == "scraping" and cur["changed"] == [] and "parts" in cur["present"]
    assert s.workcell_state()["setup"]["name"] == "scraping"
    run(s, "tcp_delete", name="tip_a")
    assert run(s, "setup_list")["current"]["changed"] == ["TCPs, fixtures, vision"]
    pv = run(s, "setup_preview", name="scraping")
    assert pv["changes"] == ["TCPs, fixtures, vision"] and pv["base"] is None
    assert pv["restart"] == "stack"                       # the spatula is not in the running robot model yet
    import json
    fb = s.setups.cfg / "ft_bias" / "ur7e_lab.json"           # the GUI's own "restored at" stamp is not a change
    fb.parent.mkdir(parents=True, exist_ok=True)
    fb.write_text(json.dumps({"presets": {}, "last": {"bias": [0.1] * 6, "at": "x"}}))
    run(s, "setup_save", name="scraping", replace=True)
    fb.write_text(json.dumps({"presets": {}, "last": {"bias": [0.1] * 6, "at": "x", "restored": "now"}}))
    assert run(s, "setup_list")["current"]["changed"] == []
    run(s, "setup_save", name="other")
    run(s, "setup_delete", name="other")
    assert run(s, "setup_list")["current"]["name"] == ""   # the one in use was deleted


def test_restart_rules(env):
    s, _, _ = env
    with pytest.raises(CommandError, match="launcher"):
        run(s, "restart", what="gui")
    got = []
    s.on_restart = lambda: got.append(1)
    s.core.busy = True
    with pytest.raises(CommandError, match="busy"):
        run(s, "restart", what="stack")
    s.core.busy = False
    run(s, "restart", what="gui")
    assert s.restart_code == RESTART_GUI and wait(lambda: got == [1], 3)


def test_http_upload_and_mesh_route(env):
    s, _, _ = env
    srv = Server(s, "127.0.0.1", 0).start()
    try:
        base = f"http://127.0.0.1:{srv.port}"
        data = stl([[(0, 0, 0), (1, 0, 0), (0, 1, 0)]])
        body = {"token": s.lease.token, "filename": "m.stl", "data_b64": base64.b64encode(data).decode()}
        req = urllib.request.Request(base + "/api/upload", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json", "Origin": base})
        with urllib.request.urlopen(req, timeout=5) as r:
            assert json.loads(r.read())["file"] == "m.stl"
        with urllib.request.urlopen(base + "/mesh/_parts/m.stl", timeout=5) as r:
            assert r.read() == data
    finally:
        srv.stop()


# ================================================================== end-effector bases
def test_base_from_model_gates_modes_and_gripper(env, monkeypatch):
    s, b, tmp = env
    monkeypatch.delenv("UR7E_EE_CONFIG", raising=False)
    assert s.running_base() == "gripper"                     # the test URDF: Robotiq, no ATI
    assert "F/T" in s.base_lacks(mode="admittance")
    assert s.base_lacks(mode="impedance") is None            # robotiq_tcp is there
    info = s.info()
    assert info["base"]["name"] == "gripper"
    assert not next(m for m in info["modes"] if m["name"] == "admittance")["allowed"]
    with pytest.raises(CommandError, match="F/T"):
        run(s, "switch_mode", mode="admittance")
    monkeypatch.setenv("UR7E_EE_CONFIG", "ft")               # the launcher's word wins over the model
    assert s.running_base() == "ft"
    with pytest.raises(CommandError, match="no gripper"):
        run(s, "gripper", percent=0)
    assert not s.info()["gripper"]["enabled"]


def test_base_switch_and_setups_remember_their_base(env, monkeypatch):
    s, b, tmp = env
    monkeypatch.setenv("UR7E_EE_CONFIG", "ft_gripper")
    cfg = tmp / "cfg" / "moveit_gui"
    with pytest.raises(CommandError, match="launcher"):
        run(s, "base_switch", base="gripper", confirm=True)
    restarts = []
    s.on_restart = lambda: restarts.append(s.restart_code)
    with pytest.raises(CommandError, match="one of"):
        run(s, "base_switch", base="robot_with_hat", confirm=True)
    with pytest.raises(CommandError, match="confirm"):
        run(s, "base_switch", base="gripper")
    assert "already" in run(s, "base_switch", base="ft_gripper")["msg"]
    run(s, "setup_save", name="full")
    assert s.setups.list()[0]["base"] == "ft_gripper"
    r = run(s, "base_switch", base="gripper", confirm=True)
    assert r["restart"] == "stack" and (cfg / "base").read_text().strip() == "gripper"
    assert wait(lambda: restarts == [RESTART_STACK], 3)
    # ... the stack came back on the gripper base; loading "full" puts the F/T + gripper base back
    monkeypatch.setenv("UR7E_EE_CONFIG", "gripper")
    s.restart_code = None
    v = run(s, "setup_list")
    assert v["base"]["running"] == "gripper" and v["base"]["next"] == "gripper"
    r = run(s, "setup_load", name="full")
    assert r["restart"] == "stack" and "F/T sensor + gripper" in r["msg"]
    assert (cfg / "base").read_text().strip() == "ft_gripper"
    auto = next(x for x in s.setups.list() if x["auto"])
    assert auto["base"] == "gripper"                          # the backup remembers what ran before


def test_active_tcp_falls_back_when_its_link_is_gone(env, monkeypatch):
    s, b, _ = env
    monkeypatch.setenv("UR7E_EE_CONFIG", "none")
    run(s, "tcp_save", name="tip", parent="robotiq_tcp", xyz_mm=[0, 0, 20], rpy_deg=[0, 0, 0])
    s.core.set_active_tcp("tip")
    s.tick()
    assert s.core.store.active_tcp == "tip"                   # robotiq_tcp is in this model: kept
    b.fake_robot_description = URDF.replace('<link name="robotiq_85_base_link"/><link name="robotiq_tcp"/><link name="finger"/>', "") \
        .split('<joint name="grip"')[0] + "</robot>"
    assert wait(lambda: (s.tick() or True) and s.core.store.active_tcp == "tool0", 5)


def test_a_setup_brings_its_cameras_back(env):
    s, _, _ = env
    s.on_restart = lambda: None
    run(s, "drivers_save", service={"name": "d405", "label": "D405", "type": "realsense", "params": {}, "autostart": True})
    run(s, "drivers_save", service={"name": "zed", "label": "ZED X Nano", "type": "zed", "params": {}})
    run(s, "parts_save", name="holder_a", parent="tool0", shape={"type": "box", "size_mm": [20, 20, 20]})
    g = run(s, "setup_list")["guide"]
    assert g["parts"] == ["holder_a"] and g["on"] == ["d405"] and g["calibrations"] == []
    assert {c["label"] for c in g["cameras"]} == {"D405", "ZED X Nano"}
    run(s, "setup_save", name="alice")
    assert s.setups.list()[0]["cameras"] == ["d405"]
    run(s, "drivers_autostart", name="d405", on=False)            # the colleague's camera instead
    run(s, "drivers_autostart", name="zed", on=True)
    run(s, "setup_save", name="bob")
    run(s, "setup_load", name="alice")
    auto = {x["name"]: x["autostart"] for x in s.drivers.view()["services"]}
    assert auto == {"d405": True, "zed": False}
    backup = next(x for x in s.setups.list() if x["auto"])
    assert backup["cameras"] == ["zed"]                             # what ran before the load


def test_a_late_tf_takes_the_tcp_pose_from_the_joints(env, monkeypatch):
    """Camera drivers crowd /tf: the TCP pose comes from the joints and the robot model, not blank or stale."""
    s, _, _ = env
    assert s.state()["tcp"]
    T = s._tcp_from_joints()
    assert T is not None

    def late(*a, **k):
        raise RuntimeError("TF is stale")
    monkeypatch.setattr(s.core.geo, "current_pose", late)
    s._tcp_last = (0.0, None, None)                    # nothing to hold: only the joints can give it
    got = s.state()["tcp"]
    assert got and "stale" not in got
    xyz, _ = tf.T_to_mm_deg(T)                         # (this test's URDF is not the fake arm's kinematics)
    assert max(abs(a - b) for a, b in zip(got["xyz_mm"], xyz)) < 0.01
    s.core.last_snap.joint_state_age = 5.0             # old joints: not used; the last pose is held, marked stale
    assert s.state()["tcp"]["stale"] is True
