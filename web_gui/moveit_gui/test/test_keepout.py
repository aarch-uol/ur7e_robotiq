"""Keep-out zones: validation, persistence, planning-scene calls through the bridge, fixture frames, and the
refusal of a box around the robot's current TCP."""
import os
import time
from unittest import mock

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.keepout import (PREFIX, KeepoutError, KeepoutStore, make_box, point_box_distance,  # noqa: E402
                                point_in_box)
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


def make_service(tmp_path, bridge=None):
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = bridge or FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    (tmp_path / "progs").mkdir(exist_ok=True)
    (tmp_path / "pol").mkdir(exist_ok=True)
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a")
    return svc


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    svc = make_service(tmp_path)
    assert wait(lambda: svc.core.keepout.status["t"] is not None)     # startup apply (no boxes yet)
    yield svc
    svc.shutdown()
    svc.core.bridge.shutdown()


APPLYING = {"keepout_save", "keepout_enable", "keepout_delete", "keepout_apply"}


def cmd(s, _command, **args):
    out = s.command("a", _command, args)
    if _command in APPLYING:
        s.ko_jobs = getattr(s, "ko_jobs", 0) + 1         # each of these starts one apply job
    return out


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def scene(s):
    return {k: v for k, v in s.core.bridge.scene.items() if k.startswith(PREFIX)}


def settled(s):
    """Wait until every apply job started by cmd() has reported back."""
    return wait(lambda: sum(r["cmd"] == "keep-out zones" for r in s.results) >= getattr(s, "ko_jobs", 0)
                and not s.core.keepout._lock.locked())


TABLE = dict(name="table", frame="base_link", size_mm=[1200, 800, 20], xyz_mm=[300, 0, -15], rpy_deg=[0, 0, 0])


def tcp_mm(s):
    return np.asarray(s.state()["tcp"]["xyz_mm"])


# ------------------------------------------------------------------ geometry
def test_point_in_box_respects_rotation_and_size():
    T = tf.mm_deg_to_T([1000, 0, 0], [0, 0, 90])          # 400 x 100 x 100 mm box turned 90° about z
    size = [0.4, 0.1, 0.1]
    assert point_in_box([1.0, 0.15, 0.0], T, size)        # along the box's long axis (now base y)
    assert not point_in_box([1.15, 0.0, 0.0], T, size)    # base x is the box's short axis
    out, inside = point_box_distance([1.0, 0.0, 0.08], T, size)
    assert out == pytest.approx(0.03) and inside == 0.0
    assert not point_in_box([1.0, 0.0, 0.05], T, size)    # on the face: not inside


# ---------------------------------------------------------------- validation
@pytest.mark.parametrize("kw,match", [
    (dict(size_mm=[0, 10, 10]), "at least"),
    (dict(size_mm=[-5, 10, 10]), "at least"),
    (dict(size_mm=[3500, 10, 10]), "typo"),
    (dict(size_mm=[10, 10]), "three"),
    (dict(xyz_mm=[0, 0, 4000]), "more than 3 m"),
    (dict(rpy_deg=[0, float("nan"), 0]), "finite"),
    (dict(name="Table 1"), "lower_snake_case"),
    (dict(name=""), "lower_snake_case"),
])
def test_invalid_boxes_are_refused(kw, match):
    with pytest.raises(KeepoutError, match=match):
        make_box(**{**TABLE, **kw})


def test_unknown_frame_and_duplicate_names_are_refused(s):
    with pytest.raises(CommandError, match="not the base frame or a fixture"):
        cmd(s, "keepout_save", **{**TABLE, "frame": "tool0"})
    cmd(s, "keepout_save", **TABLE)
    with pytest.raises(CommandError, match="already exists"):
        cmd(s, "keepout_save", **TABLE)
    cmd(s, "keepout_save", **{**TABLE, "name": "holder", "xyz_mm": [500, 500, 100], "size_mm": [50, 50, 50]})
    with pytest.raises(CommandError, match="already exists"):    # renaming onto another box
        cmd(s, "keepout_save", **{**TABLE, "name": "holder", "replace": "table"})


def test_edits_need_the_lease_and_an_idle_robot(s):
    with pytest.raises(CommandError, match="take control"):
        s.command("other", "keepout_save", TABLE)
    s.core.busy = True
    try:
        with pytest.raises(CommandError, match="busy"):
            cmd(s, "keepout_save", **TABLE)
    finally:
        s.core.busy = False


# -------------------------------------------------------- persistence + scene
def test_add_edit_disable_delete_reach_the_planning_scene(s):
    b = s.core.bridge
    r = cmd(s, "keepout_save", **TABLE)
    assert r["ok"]
    assert wait(lambda: PREFIX + "table" in b.scene)
    assert settled(s)
    obj = b.scene[PREFIX + "table"]
    assert obj["size"] == pytest.approx([1.2, 0.8, 0.02])
    assert obj["T"][:3, 3] == pytest.approx([0.3, 0.0, -0.015])
    st = s.state()["keepout"]["status"]
    assert st["ok"] and st["verified"] and st["count"] == 1 and st["applied"] == [PREFIX + "table"]

    cmd(s, "keepout_save", **{**TABLE, "xyz_mm": [300, 0, -25]}, replace="table")      # edit
    assert wait(lambda: abs(b.scene[PREFIX + "table"]["T"][2, 3] + 0.025) < 1e-9)

    cmd(s, "keepout_save", **{**TABLE, "name": "bench", "xyz_mm": [300, 0, -25]}, replace="table")   # rename
    assert wait(lambda: PREFIX + "bench" in b.scene and PREFIX + "table" not in b.scene)

    cmd(s, "keepout_enable", name="bench", enabled=False)
    assert wait(lambda: PREFIX + "bench" not in b.scene)
    assert s.core.keepout_store.boxes["bench"].enabled is False
    cmd(s, "keepout_enable", name="bench", enabled=True)
    assert wait(lambda: PREFIX + "bench" in b.scene)

    cmd(s, "keepout_delete", name="bench")
    assert wait(lambda: not scene(s))
    assert "user_table" in b.scene                    # objects that are not ours are never touched
    assert not any(c[0].startswith("scene") and c[1] == "user_table" for c in b.calls)
    logs = "\n".join(b.drain_logs())
    assert "keep-out box table created" in logs and "keep-out box bench deleted" in logs


def test_boxes_survive_a_restart_and_are_applied_at_startup(s, tmp_path):
    cmd(s, "keepout_save", **TABLE)
    cmd(s, "keepout_save", name="holder", frame="base_link", size_mm=[80, 80, 120], xyz_mm=[600, 300, 60],
        rpy_deg=[0, 0, 30], enabled=False)
    path = s.core.keepout_store.path
    assert path == tmp_path / "cfg" / "moveit_gui" / "calibrations" / "ur7e_lab.keepout.yaml"
    assert path.exists()

    s2 = make_service(tmp_path)                        # a new GUI process; move_group still holds a stale box
    try:
        b2 = s2.core.bridge
        b2.scene[PREFIX + "old_box"] = {"frame": "base_link", "size": [0.1] * 3, "T": np.eye(4)}
        st = s2.core.keepout_store
        assert list(st.boxes) == ["table", "holder"]
        h = st.boxes["holder"]
        assert (h.size_mm, h.xyz_mm, h.rpy_deg, h.enabled) == ([80, 80, 120], [600, 300, 60], [0, 0, 30], False)
        s2.core.keepout.request("test start")
        # keep ticking: the startup apply may still be running at the first tick (then the request waits)
        assert wait(lambda: (s2.tick() or True) and PREFIX + "table" in b2.scene and PREFIX + "old_box" not in b2.scene)
        assert PREFIX + "holder" not in b2.scene      # disabled
    finally:
        s2.shutdown()
        s2.core.bridge.shutdown()


def test_a_broken_file_is_reported_not_silently_dropped(tmp_path):
    p = tmp_path / "ur7e_lab.yaml"
    (tmp_path / "ur7e_lab.keepout.yaml").write_text("version: 1\nboxes:\n  t: {frame: base_link, size_mm: [0, 1, 1]}\n")
    st = KeepoutStore(p, "ur7e_lab", lambda f: True)
    assert st.boxes == {} and "NOT loaded" in st.notes[0]


def test_startup_waits_for_moveit_then_applies(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    s = make_service(tmp_path)
    cmd(s, "keepout_save", **TABLE)
    s.shutdown()
    s.core.bridge.shutdown()

    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    b = FakeBridge(profile)
    b.scene.clear()                                   # move_group restarted: empty scene
    real = b.snapshot
    up = {"v": False}

    def snap():
        out = real()
        out.connected["MoveIt"] = up["v"]
        return out
    b.snapshot = snap
    s = make_service(tmp_path, b)
    try:
        for _ in range(3):
            s.tick()
        time.sleep(0.1)
        assert not scene(s) and s.core.keepout.status["state"] == "pending"
        up["v"] = True
        s.tick()
        assert wait(lambda: PREFIX + "table" in b.scene)
        assert wait(lambda: s.core.keepout.status["verified"])
        # move_group restarts (scene lost): applied again when it comes back
        up["v"] = False
        s.tick()
        b.scene.clear()
        up["v"] = True
        s.tick()
        assert wait(lambda: PREFIX + "table" in b.scene)
    finally:
        s.shutdown()
        b.shutdown()


def test_lost_publish_is_detected_and_sent_again(s):
    s.core.bridge.scene_drop = 1
    cmd(s, "keepout_save", **TABLE)
    assert wait(lambda: PREFIX + "table" in s.core.bridge.scene)
    assert settled(s)
    assert s.core.keepout.status["verified"]
    assert sum(1 for c in s.core.bridge.calls if c[0] == "scene_set_box") >= 2



def test_a_fast_move_group_restart_is_noticed_and_the_boxes_come_back(s):
    """move_group crashed and respawned before DDS noticed: MoveIt never looked down, but the scene is empty."""
    b = s.core.bridge
    cmd(s, "keepout_save", **TABLE)
    assert wait(lambda: PREFIX + "table" in b.scene) and settled(s)
    b.scene.pop(PREFIX + "table")                         # the respawned move_group has an empty scene
    s.core.keepout._last_check -= 60                      # the periodic check is due
    assert wait(lambda: (s.tick() or True) and PREFIX + "table" in b.scene, 10)
    assert wait(lambda: (s.tick() or True) and s.core.keepout.status["verified"], 10)
    assert any("lost" in line for _, line in s.logs)

def test_unreadable_scene_is_reported_as_unverified(s):
    s.core.bridge.scene_readable = False
    with mock.patch("moveit_gui.keepout.VERIFY_WAIT_S", 0.01):
        cmd(s, "keepout_save", **TABLE)
        assert settled(s)
    st = s.state()["keepout"]["status"]
    assert st["state"] == "unverified" and not st["ok"] and st["retry"]
    assert "does not show them" in st["msg"]


# ------------------------------------------------------------- fixture frames
def test_box_on_a_fixture_is_sent_in_the_base_frame_and_follows_the_fixture(s):
    store = s.core.store
    T_vial = tf.mm_deg_to_T([500, -200, 50], [0, 0, 90])
    store.put_fixture(FixtureDef("vial", "base_link", T_vial))
    cmd(s, "keepout_save", name="holder", frame="vial", size_mm=[100, 60, 40], xyz_mm=[10, 0, 20], rpy_deg=[0, 0, 0])
    b = s.core.bridge
    assert wait(lambda: PREFIX + "holder" in b.scene)
    T = b.scene[PREFIX + "holder"]["T"]
    assert T[:3, 3] == pytest.approx([0.5, -0.19, 0.07])          # 10 mm along the vial's x = base y
    assert np.allclose(T[:3, :3], T_vial[:3, :3])
    view = {x["name"]: x for x in s.state()["keepout"]["boxes"]}
    assert view["holder"]["base"]["xyz_mm"] == pytest.approx([500, -190, 70])

    # the vial is re-taught 30 mm further in x: poll() notices and re-applies, nothing else needed
    store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([530, -200, 50], [0, 0, 90])), replace=True)
    s.core.keepout._last_poll = 0
    assert wait(lambda: (s.tick() or True) and abs(b.scene[PREFIX + "holder"]["T"][0, 3] - 0.53) < 1e-9)


def test_box_whose_fixture_is_deleted_is_removed_and_reported(s):
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([500, -200, 50], [0, 0, 0])))
    cmd(s, "keepout_save", name="holder", frame="vial", size_mm=[100, 60, 40], xyz_mm=[0, 0, 0], rpy_deg=[0, 0, 0])
    assert wait(lambda: PREFIX + "holder" in s.core.bridge.scene)
    assert settled(s)
    s.core.store.delete_fixture("vial")
    s.core.keepout._last_poll = 0
    assert wait(lambda: (s.tick() or True) and PREFIX + "holder" not in s.core.bridge.scene)
    assert wait(lambda: "holder" in s.core.keepout.status["msg"])
    st = s.state()["keepout"]
    assert st["status"]["state"] == "warning" and st["boxes"][0]["error"]


# -------------------------------------------------------------- robot inside
def test_box_around_the_current_tcp_is_refused(s):
    p = tcp_mm(s)
    with pytest.raises(CommandError, match="would contain the current TCP"):
        cmd(s, "keepout_save", name="oops", frame="base_link", size_mm=[100, 100, 100], xyz_mm=list(p),
            rpy_deg=[0, 0, 0])
    assert "oops" not in s.core.keepout_store.boxes
    assert not scene(s)
    # saved disabled it is accepted, but enabling it is refused
    cmd(s, "keepout_save", name="oops", frame="base_link", size_mm=[100, 100, 100], xyz_mm=list(p),
        rpy_deg=[0, 0, 0], enabled=False)
    with pytest.raises(CommandError, match="current TCP"):
        cmd(s, "keepout_enable", name="oops", enabled=True)


def test_box_around_the_robot_base_is_refused(s):
    with pytest.raises(CommandError, match="robot base"):
        cmd(s, "keepout_save", name="floor", frame="base_link", size_mm=[400, 400, 100], xyz_mm=[0, 0, 0],
            rpy_deg=[0, 0, 0])


def test_box_close_to_the_tcp_is_saved_with_a_warning(s):
    p = tcp_mm(s)
    r = cmd(s, "keepout_save", name="near", frame="base_link", size_mm=[100, 100, 100],
            xyz_mm=list(p - [0, 0, 60]), rpy_deg=[0, 0, 0])
    assert r["ok"] and "10 mm from it" in r["msg"]


def test_box_the_robot_moved_into_is_skipped_at_apply(s):
    """A stored box that contains the TCP at apply time (moved there in freedrive) is not sent to MoveIt."""
    p = tcp_mm(s)
    cmd(s, "keepout_save", name="later", frame="base_link", size_mm=[100, 100, 100],
        xyz_mm=list(p + [0, 0, 300]), rpy_deg=[0, 0, 0])
    assert wait(lambda: PREFIX + "later" in s.core.bridge.scene)
    assert settled(s)
    box = s.core.keepout_store.boxes["later"]
    box.xyz_mm = list(p)                               # as if the robot were now inside it
    ok, msg = s.core.keepout.apply("test")
    assert not ok and "later" in msg and "NOT applied" in msg
    assert PREFIX + "later" not in s.core.bridge.scene
    assert s.core.keepout.status["retry"]



def test_robot_inside_a_box_is_reported_live_and_in_failures(s):
    p = tcp_mm(s)
    cmd(s, "keepout_save", name="cell", frame="base_link", size_mm=[100, 100, 100],
        xyz_mm=list(p + [0, 0, 300]), rpy_deg=[0, 0, 0])
    assert wait(lambda: PREFIX + "cell" in s.core.bridge.scene) and settled(s)
    s.tick()
    assert s.state()["keepout"]["inside"] == []
    s.core.keepout_store.boxes["cell"].xyz_mm = list(p)       # the robot is now inside (freedrive, fixture re-taught)
    assert wait(lambda: (s.tick() or True) and s.state()["keepout"]["inside"], 5)
    assert "cell" in s.state()["keepout"]["inside"][0]
    assert "keep-out zone cell" in s.core.keepout_hint()
    ok, msg = s.core.run_motion(lambda: False)
    assert not ok and "inside keep-out zone cell" in msg

def test_from_tcp_puts_the_box_top_under_the_tcp(s):
    p = tcp_mm(s)
    r = cmd(s, "keepout_from_tcp", frame="base_link", size_mm=[200, 200, 40])
    assert r["xyz_mm"] == pytest.approx([p[0], p[1], p[2] - 25], abs=0.11)
    r2 = cmd(s, "keepout_save", name="pad", frame="base_link", size_mm=[200, 200, 40], xyz_mm=r["xyz_mm"],
             rpy_deg=r["rpy_deg"])
    assert r2["ok"] and r2["warnings"]                 # 5 mm from the TCP: saved, with a warning


def test_state_and_diagnostics_view(s):
    cmd(s, "keepout_save", **TABLE)
    assert settled(s)
    k = s.state()["keepout"]
    assert k["frames"][0] == "base_link"
    assert k["boxes"][0]["base"]["xyz_mm"] == pytest.approx([300, 0, -15])
    assert k["status"]["count"] == 1 and "table" in k["status"]["msg"]
    cmd(s, "keepout_apply")
    assert wait(lambda: s.core.keepout.status["reason"] == "re-apply")


# ------------------------------------------------------------ ros bridge path
def test_ros_bridge_scene_calls_use_pymoveit2_in_the_base_frame():
    import importlib
    import sys
    from test_ros_bridge_static import STUBS, _bare_bridge
    with mock.patch.dict(sys.modules, {m: mock.MagicMock() for m in STUBS}):
        sys.modules.pop("moveit_gui.ros_bridge", None)
        rb = importlib.import_module("moveit_gui.ros_bridge")
        try:
            b = _bare_bridge(rb)
            b.moveit = mock.MagicMock()
            T = tf.mm_deg_to_T([100, 200, 300], [0, 0, 90])
            assert b.scene_set_box("gui_keepout/t", [0.1, 0.2, 0.3], T) == (True, "published")
            kw = b.moveit.add_collision_box.call_args.kwargs
            assert kw["id"] == "gui_keepout/t" and kw["frame_id"] == "base_link"
            assert kw["size"] == (0.1, 0.2, 0.3) and kw["position"] == pytest.approx([0.1, 0.2, 0.3])
            assert kw["quat_xyzw"] == pytest.approx([0, 0, 2 ** -0.5, 2 ** -0.5])
            b.scene_remove("gui_keepout/t")
            b.moveit.remove_collision_object.assert_called_once_with("gui_keepout/t")

            # read back: a box in another frame is expressed in base_link
            def pose(x, y, z, q=(0, 0, 0, 1)):
                return mock.Mock(position=mock.Mock(x=x, y=y, z=z), orientation=mock.Mock(x=q[0], y=q[1], z=q[2], w=q[3]))
            co = mock.Mock(id="gui_keepout/t", primitives=[mock.Mock(type=1, dimensions=[0.1, 0.2, 0.3])],
                           primitive_poses=[pose(0, 0, 0)], pose=pose(1.0, 0, 0))
            co.header.frame_id = "world"
            b.moveit.update_planning_scene.return_value = True
            b.moveit.planning_scene.world.collision_objects = [co]
            b.lookup_transform = lambda parent, child: tf.make([0, 0, 0.5]) if (parent, child) == ("base_link", "world") else None
            out = b.scene_objects()
            assert out["gui_keepout/t"]["size"] == [0.1, 0.2, 0.3]
            assert out["gui_keepout/t"]["T"][:3, 3] == pytest.approx([1.0, 0, 0.5])
            b.moveit.update_planning_scene.return_value = False
            assert b.scene_objects() is None
            b.moveit = None
            assert b.scene_set_box("x", [1, 1, 1], np.eye(4))[0] is False
        finally:
            sys.modules.pop("moveit_gui.ros_bridge", None)
