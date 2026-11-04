"""Session recording (rosbag2/MCAP): topic selection, limits, auto-record with runs, and the web commands."""
import json
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.recorder import FakeBagProcess, Recorder, preset_topics  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
TOPICS = [("/joint_states", "sensor_msgs/msg/JointState"), ("/tf", "tf2_msgs/msg/TFMessage"),
          ("/ft_data", "geometry_msgs/msg/WrenchStamped"), ("/cam/image_raw", "sensor_msgs/msg/Image"),
          ("/cam/image_raw/compressed", "sensor_msgs/msg/CompressedImage"), ("/rosout", "rcl_interfaces/msg/Log"),
          ("/policy_runner/status", "std_msgs/msg/String")]


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def recorder(tmp_path, free=1e6, **kw):
    logs = []
    r = Recorder(tmp_path / "rec", "test", lambda: TOPICS, logs.append, spawn=FakeBagProcess,
                 free_mb=lambda p: free if not callable(free) else free(), settings_path=tmp_path / "rec.json", **kw)
    r.logs = logs
    return r


def test_presets_resolve_against_what_is_published():
    assert preset_topics("robot", TOPICS) == ["/joint_states", "/tf"]
    assert preset_topics("ft", TOPICS) == ["/ft_data"]
    assert preset_topics("camera_compressed", TOPICS) == ["/cam/image_raw/compressed"]
    assert preset_topics("camera_raw", TOPICS) == ["/cam/image_raw"]
    # a policy-ish name on an image stream is still a camera topic, not a "small" policy topic
    vis = TOPICS + [("/scraping_vision/annotated_image", "sensor_msgs/msg/Image"),
                    ("/scraping_vision/world_midpoints", "geometry_msgs/msg/PoseArray")]
    assert preset_topics("policy", vis) == ["/policy_runner/status", "/scraping_vision/world_midpoints"]
    assert "/scraping_vision/annotated_image" in preset_topics("camera_raw", vis)


def test_opt_in_selection_is_presets_plus_topics_minus_exclusions_and_is_saved(tmp_path):
    r = recorder(tmp_path)
    assert r.selected_topics() == ["/ft_data", "/joint_states", "/policy_runner/status", "/tf"]   # defaults
    ok, _ = r.configure(presets=["robot", "camera_compressed"], topics=["/rosout"], exclude=["/tf"])
    assert ok
    assert r.selected_topics() == ["/cam/image_raw/compressed", "/joint_states", "/rosout"]
    r2 = recorder(tmp_path)                                         # a restarted GUI keeps the choice
    assert r2.selected_topics() == r.selected_topics()
    assert not r.configure(presets=["nope"])[0]
    assert not r.configure(topics=["no_slash"])[0]
    assert not r.configure(max_s=0)[0]
    cat = r.catalog()
    raw = next(t for t in cat["topics"] if t["topic"] == "/cam/image_raw")
    assert raw["presets"] == ["camera_raw"] and not raw["selected"]


def test_start_stop_writes_a_bag_folder_with_metadata(tmp_path):
    r = recorder(tmp_path)
    ok, msg = r.start(name="first try!", source="manual")
    assert ok, msg
    live = r.view()["recording"]
    assert live["name"].endswith("_first_try") and live["topics"] == 4
    assert not r.start()[0]                                          # one at a time
    time.sleep(0.3)
    r.stop("done")
    assert wait(lambda: not r.recording)
    d = tmp_path / "rec" / live["name"]
    assert (d / "bag" / "metadata.yaml").exists()
    meta = json.loads((d / "gui_meta.json").read_text())
    assert meta["ok"] and meta["reason"] == "done" and meta["topics"][0] == "/ft_data" and meta["size_mb"] > 0
    assert r.recent()[0]["name"] == live["name"]


def test_limits_and_disk_space_end_a_recording(tmp_path):
    r = recorder(tmp_path)
    r.configure(max_s=1)
    r.start()
    assert wait(lambda: (r.poll() or True) and not r.recording, 5)
    assert "time limit" in r.last["reason"]

    r.configure(max_s=600, max_mb=10)
    r.start()                                                       # the fake writes 0.5 MB/s: lower the bar
    r._cur["limits"]["max_mb"] = 0.2
    assert wait(lambda: (r.poll() or True) and not r.recording, 5)
    assert "size limit" in r.last["reason"]

    free = [5000.0]
    r2 = recorder(tmp_path, free=lambda: free[0])
    r2.start()
    free[0] = 500.0
    r2._cur["_size_t"] = 0
    assert wait(lambda: (r2.poll() or True) and not r2.recording, 5)
    assert "disk almost full" in r2.last["reason"]
    assert not recorder(tmp_path, free=100).start()[0]              # refused when the disk is nearly full


def test_nothing_published_is_refused(tmp_path):
    r = Recorder(tmp_path / "rec", "t", lambda: [], print, spawn=FakeBagProcess, free_mb=lambda p: 1e6,
                 settings_path=tmp_path / "s.json")
    ok, msg = r.start()
    assert not ok and "nothing to record" in msg


# ------------------------------------------------------------------ web service + auto-record
@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    (tmp_path / "progs").mkdir()
    (tmp_path / "pol").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    core.recorder.free_mb = lambda p: 1e6
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def cmd(s, _command, client="a", **args):
    return s.command(client, _command, args)


def test_web_commands_and_state(s, tmp_path):
    cat = cmd(s, "record_catalog", client="viewer")                 # read-only: no control needed
    assert any(t["topic"] == "/joint_states" and t["selected"] for t in cat["topics"])
    assert cat["dir"] == str(tmp_path / "recordings")
    with pytest.raises(CommandError, match="take control"):
        cmd(s, "record_start", client="viewer")
    cmd(s, "record_configure", presets=["robot"], topics=["/ft_data"])
    assert cmd(s, "record_start", name="calib")["ok"]
    st = s.state()["record"]
    assert st["recording"]["topics"] == 5                            # joint_states, tf, tf_static, speed_scaling, ft
    assert cmd(s, "record_stop", client="viewer")["ok"]              # anyone may stop, like STOP
    assert wait(lambda: s.state()["record"]["recording"] is None)
    assert s.state()["record"]["last"]["ok"]
    with pytest.raises(CommandError, match="60 characters"):
        cmd(s, "record_start", name="x" * 61)


def test_stop_does_not_end_a_recording(s):
    cmd(s, "record_start")
    s.stop("test")
    assert s.state()["record"]["recording"] is not None
    cmd(s, "record_stop")


def test_auto_record_follows_a_program_run_and_is_linked_in_history(s):
    cmd(s, "record_configure", auto=True)
    cmd(s, "program_new")
    cmd(s, "program_rename", name="pick")
    cmd(s, "program_insert", type="wait", params={"seconds": 0.6})
    cmd(s, "program_run")
    assert wait(lambda: (s.tick() or True) and s.state()["record"]["recording"] is not None)
    rec = s.state()["record"]["recording"]
    assert rec["source"] == "auto" and rec["run"]["kind"] == "program" and "program_pick" in rec["name"]
    assert wait(lambda: (s.tick() or True) and s.state()["record"]["recording"] is None)
    run = s.core.history.recent(1)[0]
    assert run["kind"] == "program" and run["recording"] == rec["dir"]
    assert s.state()["record"]["last"]["reason"] == "the run ended"


def test_auto_record_leaves_a_manual_recording_alone(s):
    cmd(s, "record_configure", auto=True)
    cmd(s, "record_start", name="manual")
    cmd(s, "program_new")
    cmd(s, "program_insert", type="wait", params={"seconds": 0.3})
    cmd(s, "program_run")
    assert wait(lambda: (s.tick() or True) and s.core.runner.state.name != "IDLE")
    assert wait(lambda: (s.tick() or True) and s.core.runner.state.name == "IDLE")
    for _ in range(3):
        s.tick()
    live = s.state()["record"]["recording"]
    assert live is not None and live["name"].endswith("_manual")     # still recording after the run
    cmd(s, "record_stop")


def test_closing_the_gui_finishes_the_recording(s):
    cmd(s, "record_start")
    s.shutdown()
    assert s.state()["record"]["recording"] is None and s.core.recorder.last["reason"] == "GUI closed"
