"""The one-button recording (the "all" preset, meta.json) and the photo / record_start / record_stop program steps."""
import json
import os
import time
from pathlib import Path

import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.program import Program, Step  # noqa: E402
from moveit_gui.recorder import preset_topics  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
CAM = "/camera/camera/color/image_rect_raw"


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=60.0, move_time_scale=5.0)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a", name="Ada")
    yield svc
    svc.core.stop_all("test end")
    svc.core.recorder.stop_and_wait("test end")
    svc.shutdown()
    bridge.shutdown()


def cmd(s, _name, **args):
    return s.command("a", _name, args)


def tick_until(s, cond, timeout=20.0):
    return wait(lambda: (s.heartbeat("a") or True) and (s.tick() or True) and cond(), timeout)


def load(s, steps):
    d = {"version": 2, "name": "rec_t", "robot": "ur7e_lab", "steps": steps}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))


def run(s, steps, timeout=30.0):
    load(s, steps)
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None, timeout)
    return s.core.program_result


def recording(s):
    return s.core.recorder.view()["recording"]


# ---------------------------------------------------------------- the "all" preset and meta.json
def test_all_preset_picks_everything_useful_and_only_compressed_images(s):
    topics = s.core.bridge.topic_types()
    got = preset_topics("all", topics)
    for t in ("/joint_states", "/tf", "/ft_data", f"{CAM}/compressed"):
        assert t in got
    assert any("gripper" in t for t in got)
    assert CAM not in got and not any(t.endswith("depth/image_rect_raw") for t in got)   # raw images stay out
    assert "/rosout" not in got


def test_record_all_writes_meta_json(s):
    r = cmd(s, "record_start", name="trial", note="first try", preset="all")
    assert r["ok"]
    live = recording(s)
    assert f"{CAM}/compressed" in s.core.recorder._cur["topics"]
    d = Path(live["dir"])
    assert tick_until(s, lambda: d.joinpath("bag").exists())
    cmd(s, "record_stop")
    assert tick_until(s, lambda: recording(s) is None)
    meta = json.loads((d / "meta.json").read_text())
    assert meta["operator"] == "Ada" and meta["note"] == "first try" and meta["ok"] is True
    assert meta["started"] and meta["stopped"] and meta["presets"] == ["all"] and meta["mode"]
    assert meta["program"] == ""
    assert (d / "gui_meta.json").exists()


def test_record_start_rejects_unknown_preset_and_long_note(s):
    with pytest.raises(CommandError):
        cmd(s, "record_start", preset="nope")
    with pytest.raises(CommandError):
        cmd(s, "record_start", note="x" * 201)


# ---------------------------------------------------------------- schema / validate / summary
def test_new_steps_schema_summary_and_old_programs_load(s):
    prog = Program.from_dict({"version": 2, "name": "x", "robot": "ur7e_lab", "steps": [
        {"type": "record_start", "preset": "all", "note": "n"},
        {"type": "photo", "topic": CAM, "label": "before"},
        {"type": "record_stop"}]}, s.core.profile)
    assert [st.title for st in prog.steps] == ["Record start", "Photo", "Record stop"]
    assert prog.steps[1].summary() == f"{CAM} · before"
    assert prog.steps[0].summary() == "all · n"
    assert Program.from_dict(prog.to_dict(), s.core.profile).to_dict() == prog.to_dict()
    old = Program.from_dict({"version": 2, "name": "old", "steps": [{"type": "wait", "seconds": 1}]}, s.core.profile)
    assert old.steps[0].type == "wait"


def test_validate_photo_and_record_start(s):
    prof = s.core.profile
    bad = Program("x", "ur7e_lab", [Step("photo", {}), Step("photo", {"topic": "cam"}),
                                    Step("record_start", {"preset": "nope"})])
    problems = bad.validate(prof)
    assert len(problems) == 3
    ok = Program("x", "ur7e_lab", [Step("photo", {"topic": CAM}), Step("record_start", {"preset": "all"}),
                                   Step("record_stop")])
    assert ok.validate(prof) == []
    live = Program("x", "ur7e_lab", [Step("photo", {"topic": "/nothing/image"})])
    assert any("no image is published" in p for p in live.validate(prof, live=True, bridge=s.core.bridge))


def test_step_editor_inserts_and_edits_the_new_steps(s):
    for t in ("photo", "record_start", "record_stop"):
        assert cmd(s, "program_insert", type=t)["ok"]
    steps = s.program.steps
    assert steps[0].params["topic"] == CAM and steps[1].params["preset"] == "all"
    cmd(s, "program_update", path=[0], params={"label": "top view"})
    assert s.program.steps[0].params["label"] == "top view"
    with pytest.raises(CommandError):
        cmd(s, "program_update", path=[0], params={"topic": "no-slash"})


def test_simulate_ignores_the_new_steps(s):
    from moveit_gui import program_path
    assert "simulate" in dir(program_path)
    prog = Program("x", "ur7e_lab", [Step("record_start", {}), Step("photo", {"topic": CAM}), Step("record_stop")])
    assert [st for st in prog.steps if st.type in ("movej", "move_pose")] == []


# ---------------------------------------------------------------- the runner
def test_photo_step_saves_a_jpeg_in_the_run_folder_while_recording(s):
    ok, msg = run(s, [{"type": "record_start", "preset": "all", "note": "demo"},
                      {"type": "photo", "topic": CAM, "label": "before pick"},
                      {"type": "record_stop"}])
    assert ok, msg
    assert recording(s) is None
    run_dir = Path(s.core.recorder.last["dir"])
    jpgs = list((run_dir / "photos").glob("*_before_pick.jpg"))
    assert len(jpgs) == 1 and jpgs[0].read_bytes()[:3] == b"\xff\xd8\xff"
    meta = json.loads((run_dir / "meta.json").read_text())
    assert meta["program"] == "rec_t" and meta["note"] == "demo" and meta["operator"] == "Ada"
    assert s.core.history.recent(1)[0]["recording"] == str(run_dir)


def test_photo_step_without_recording_goes_to_the_photos_folder(s, tmp_path):
    ok, msg = run(s, [{"type": "photo", "topic": CAM}])
    assert ok, msg
    files = list((tmp_path / "photos").glob("*_step1.jpg"))
    assert len(files) == 1


def test_photo_step_fails_clearly_on_a_dead_topic(s):
    load(s, [{"type": "photo", "topic": "/nothing/image"}])
    with pytest.raises(CommandError, match="no image is published"):      # refused before the first step
        cmd(s, "program_run")
    # a topic that is listed but gives no frame fails the step with the reason
    s.core.bridge.fake_published.add("/silent/image")
    s.core.bridge.image_topics = lambda: [("/silent/image", "sensor_msgs/msg/Image")]
    load(s, [{"type": "photo", "topic": "/silent/image"}])
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None and not s.core.program_result[0], 5)
    assert "Photo" in s.core.program_result[1]


def test_a_program_that_ends_stops_the_recording_it_started(s):
    ok, msg = run(s, [{"type": "record_start"}, {"type": "wait", "seconds": 0.3}])
    assert ok, msg
    assert tick_until(s, lambda: recording(s) is None and s.core.recorder.last, 15)
    assert s.core.recorder.last["ok"] and "program ended" in s.core.recorder.last["reason"]


def test_stopping_a_program_stops_its_recording(s):
    load(s, [{"type": "record_start", "preset": "all"}, {"type": "wait", "seconds": 30}])
    cmd(s, "program_run")
    assert tick_until(s, lambda: recording(s) is not None, 5)
    time.sleep(0.3)
    s.core.runner.stop()
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE", 10)
    assert tick_until(s, lambda: recording(s) is None and s.core.recorder.last, 15)
    assert "program ended" in s.core.recorder.last["reason"]


def test_a_recording_the_operator_started_is_left_alone(s):
    assert cmd(s, "record_start", preset="all")["ok"]
    ok, msg = run(s, [{"type": "record_start"}, {"type": "wait", "seconds": 0.2}])
    assert ok, msg
    s.tick()
    assert recording(s) is not None and not recording(s)["stopping"]
    # ...but a record_stop step ends any recording
    ok, msg = run(s, [{"type": "record_stop"}])
    assert ok, msg
    assert recording(s) is None


def test_record_stop_then_start_again_makes_two_recordings(s):
    ok, msg = run(s, [{"type": "record_start", "note": "a"}, {"type": "wait", "seconds": 0.2}, {"type": "record_stop"},
                      {"type": "record_start", "note": "b"}, {"type": "wait", "seconds": 0.2}])
    assert ok, msg
    assert tick_until(s, lambda: recording(s) is None, 15)
    notes = sorted(json.loads(p.read_text())["note"] for p in s.core.recorder.root.glob("*/meta.json"))
    assert notes == ["a", "b"]
