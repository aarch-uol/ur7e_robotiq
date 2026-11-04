"""UR-style program editing: nested loops/folders, insert/edit/set-waypoint/move-here, and running nested programs."""
import math
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.program import Program, Step, unroll  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    (tmp_path / "progs").mkdir()
    (tmp_path / "pol").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def cmd(s, _command, **args):
    return s.command("a", _command, args)


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_build_a_program_like_on_the_pendant(s):
    cmd(s, "program_new", name="pick")
    r = cmd(s, "program_insert", type="movej")                              # waypoint = where the arm is now
    assert r["selected"] == [0] and r["program"]["rows"][0]["waypoint"]["kind"] == "joints"
    r = cmd(s, "program_insert", type="loop", path=[0])
    assert r["selected"] == [1] and r["program"]["rows"][1]["container"]
    assert r["program"]["rows"][1]["problems"]                               # empty loop is flagged
    r = cmd(s, "program_insert", type="move_pose", path=[1], where="into", params={"cartesian": True})
    assert r["selected"] == [1, 0]
    row = r["program"]["rows"][2]
    assert row["title"] == "MoveL" and row["depth"] == 1 and row["label"] == "2.1" and row["waypoint"]["kind"] == "pose"
    r = cmd(s, "program_insert", type="wait", path=[1, 0])
    assert r["selected"] == [1, 1]
    r = cmd(s, "program_update", path=[1, 1], params={"seconds": 0.05})
    assert r["program"]["rows"][3]["params"]["seconds"] == 0.05
    cmd(s, "program_update", path=[1], params={"times": 3})
    r = cmd(s, "program_insert", type="gripper", path=[1], params={"command": "close"})
    rows = r["program"]["rows"]
    assert [x["label"] for x in rows] == ["1", "2", "2.1", "2.2", "3"]
    assert not any(x["problems"] for x in rows)
    with pytest.raises(CommandError, match="seconds"):
        cmd(s, "program_update", path=[1, 1], params={"seconds": -1})
    assert s.program.step_at((1, 1)).params["seconds"] == 0.05             # a refused edit changes nothing
    with pytest.raises(CommandError, match="cannot contain"):
        cmd(s, "program_insert", type="wait", path=[0], where="into")

    # the saved file is plain nested YAML and loads back the same
    cmd(s, "program_save", name="pick")
    p = Program.load(s.core.programs_dir / "pick.yaml", s.core.profile)
    assert p.steps[1].type == "loop" and [c.type for c in p.steps[1].children] == ["move_pose", "wait"]
    assert [path for path, _ in unroll(p.steps)] == [(0,), (1, 0), (1, 1), (1, 0), (1, 1), (1, 0), (1, 1), (2,)]


def test_running_a_loop_executes_its_body_n_times(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="loop", params={"times": 3})
    cmd(s, "program_insert", type="set_io", path=[0], where="into", params={"pin": 1})
    cmd(s, "program_insert", type="wait", path=[0, 0], params={"seconds": 0.05})
    cmd(s, "program_insert", type="comment", path=[0], params={"text": "done"})
    bridge = s.core.bridge
    n0 = len([c for c in bridge.calls if c[0] == "io"])
    seen = set()
    cmd(s, "program_run")
    assert wait(lambda: (seen.add(tuple(s.state()["runner"]["path"] or ())) or s.core.runner.state.name == "IDLE")
                and len(seen) > 1)
    assert len([c for c in bridge.calls if c[0] == "io"]) - n0 == 3
    assert seen & {(0, 0), (0, 1)}                                          # nested path reported
    assert s.core.program_result[0], s.core.program_result


def test_run_from_a_nested_step(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="wait", params={"seconds": 0.01})
    cmd(s, "program_insert", type="folder", path=[0])
    cmd(s, "program_insert", type="set_io", path=[1], where="into", params={"pin": 1})
    cmd(s, "program_insert", type="set_io", path=[1, 0], params={"pin": 2})
    calls = s.core.bridge.calls
    n0 = len(calls)
    cmd(s, "program_run", start=[1, 1])
    assert wait(lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None)
    pins = [c[1] for c in calls[n0:] if c[0] == "io"]
    assert pins == [2]


def test_set_waypoint_and_move_here(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="movej")
    name = s.program.steps[0].params["target"]
    q_taught = list(s.program.waypoints[name].joints)
    q_other = [q + 0.2 for q in q_taught]
    assert s.core.move_joints(q_other, 1.0, 1.0)[0]
    s.tick()
    r = cmd(s, "program_set_waypoint", path=[0])
    assert r["selected"] == [0]
    assert s.program.waypoints[name].joints == pytest.approx(q_other, abs=1e-3)
    assert s.core.move_joints(q_taught, 1.0, 1.0)[0]
    s.tick()
    r = cmd(s, "program_move_here", path=[0], vel=1.0, acc=1.0)
    assert r["ok"] and "job" in r
    assert wait(lambda: s.core.idle() and max(abs(a - b) for a, b in zip(s.core.bridge.snapshot().joints, q_other)) < 1e-3)
    with pytest.raises(CommandError, match="move steps"):
        cmd(s, "program_insert", type="wait", path=[0])
        cmd(s, "program_set_waypoint", path=[1])


def test_move_duplicate_delete_stay_within_their_parent(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="loop")
    cmd(s, "program_insert", type="wait", path=[0], where="into", params={"seconds": 1})
    cmd(s, "program_insert", type="wait", path=[0, 0], params={"seconds": 2})
    r = cmd(s, "program_move_step", path=[0, 1], delta=-1)
    assert r["selected"] == [0, 0] and s.program.step_at((0, 0)).params["seconds"] == 2
    with pytest.raises(CommandError, match="further"):
        cmd(s, "program_move_step", path=[0, 0], delta=-1)
    r = cmd(s, "program_duplicate", path=[0])
    assert r["selected"] == [1] and len(s.program.steps[1].children) == 2
    s.program.steps[1].children[0].params["seconds"] = 9                     # a real copy, not shared
    assert s.program.steps[0].children[0].params["seconds"] == 2
    r = cmd(s, "program_delete_step", path=[0, 1])
    assert r["selected"] == [0, 0] and len(s.program.steps[0].children) == 1
    cmd(s, "program_delete_step", index=1)                                    # old flat form still works
    assert len(s.program.steps) == 1


def test_loop_validation():
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    p = Program(steps=[Step("loop", {"times": 0}, [Step("wait", {"seconds": 1})]), Step("loop", {"times": 2})])
    problems = p.validate(profile)
    assert any("1: loop times" in x for x in problems) and any("2: empty loop" in x for x in problems)
    assert math.isfinite(len(unroll(p.steps)))
