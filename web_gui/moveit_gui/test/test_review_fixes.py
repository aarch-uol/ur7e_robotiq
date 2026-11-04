"""Fixes from the product review: manual commands while busy, and the rest of the "Now" list."""
import math
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
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


def test_gripper_is_refused_while_a_program_runs(s):
    assert s.state()["manual_refusal"] is None
    cmd(s, "program_new")
    cmd(s, "program_insert", type="wait", params={"seconds": 2.0})
    cmd(s, "program_run")
    assert wait(lambda: s.core.runner.state.name == "RUNNING")
    st = s.state()
    assert "program" in st["manual_refusal"]
    with pytest.raises(CommandError, match="program is running"):
        cmd(s, "gripper", percent=100)
    cmd(s, "program_stop")
    assert wait(lambda: s.core.runner.state.name == "IDLE")
    assert cmd(s, "gripper", percent=0)["ok"]


def test_undo_redo_and_unsaved_flag(s):
    r = cmd(s, "program_new", name="p")
    assert r["program"]["dirty"] is False                                # (New itself can be undone)
    cmd(s, "program_insert", type="wait", params={"seconds": 1.0})
    r = cmd(s, "program_insert", type="wait", path=[0], params={"seconds": 2.0})
    assert r["program"]["dirty"] and r["program"]["can_undo"] and len(r["program"]["rows"]) == 2
    assert s.state()["runner"]["dirty"]
    r = cmd(s, "program_update", path=[1], params={"seconds": 3.0})
    r = cmd(s, "program_undo")
    assert s.program.step_at((1,)).params["seconds"] == 2.0 and r["program"]["can_redo"]
    cmd(s, "program_undo")
    assert len(s.program.steps) == 1
    r = cmd(s, "program_redo")
    assert len(s.program.steps) == 2 and s.program.steps[1].params["seconds"] == 2.0
    cmd(s, "program_delete_step", path=[0])
    assert len(s.program.steps) == 1
    r = cmd(s, "program_undo")                                           # a deleted step comes back
    assert len(s.program.steps) == 2 and r["program"]["can_redo"]
    r = cmd(s, "program_save", name="p")
    assert r["program"]["dirty"] is False
    r = cmd(s, "program_update", path=[0], params={"seconds": 5.0})
    assert r["program"]["dirty"]
    with pytest.raises(CommandError, match="seconds"):                  # a refused edit is not an undo entry
        cmd(s, "program_update", path=[0], params={"seconds": -1})
    cmd(s, "program_undo")
    assert s.program.steps[0].params["seconds"] == 1.0 and not s.state()["runner"]["dirty"]
    cmd(s, "program_new")
    with pytest.raises(CommandError, match="nothing to redo"):
        cmd(s, "program_redo")


def test_operator_names_and_takeover_message(s):
    s.take_lease("b", "Ayse @ 100.1.2.3")
    assert s.state()["lease"]["name"] == "Ayse @ 100.1.2.3"
    assert not s.heartbeat("a")
    s.tick()
    assert any("took over control" in line for _, line in s.logs)


def test_failed_readiness_checks_come_with_a_fix(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    pol = tmp_path / "pol"
    (pol / "f").mkdir(parents=True)
    for f in ("model.zip", "vec_normalize.pkl", "config.yaml"):
        (pol / "f" / f).write_text("")
    (pol / "p.yaml").write_text(f"""name: p
description: test
algorithm: PPO
backend: subprocess
required_mode: admittance
required_tcp: robotiq_tcp
control_hz: 5.0
model: {pol}/f/model.zip
normalization: {pol}/f/vec_normalize.pkl
training_config: {pol}/f/config.yaml
entrypoint: run_policy
duration: 8.0
""")
    (tmp_path / "progs").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", pol, fake=True)
    s = WebService(core)
    try:
        s.tick()
        s.take_lease("a")
        reqs = {r["label"]: r for r in cmd(s, "policy_requirements", name="p")["requirements"]}
        mode = profile.modes["admittance"].label
        assert reqs[mode]["state"] == "fail"
        assert reqs[mode]["fix"]["cmd"] == "switch_mode" and reqs[mode]["fix"]["args"] == {"mode": "admittance"}
        assert all(r["fix"] is None for r in reqs.values() if r["state"] == "ok")
        if reqs["TCP robotiq_tcp"]["state"] == "fail":
            assert reqs["TCP robotiq_tcp"]["fix"]["cmd"] == "set_tcp"
    finally:
        s.shutdown()
        bridge.shutdown()


def test_speed_override_scales_gui_motion_and_programs(s, monkeypatch):
    seen = []
    real = s.core.bridge.move_joints
    monkeypatch.setattr(s.core.bridge, "move_joints", lambda q, v, a: (seen.append((v, a)), real(q, v, a))[1])
    cmd(s, "speed_override", percent=25)
    assert s.state()["speed_override"] == 0.25
    q = [math.degrees(v) for v in s.core.bridge.snapshot().joints]
    q[0] += 2
    cmd(s, "move_joints", q_deg=q, vel=0.4, acc=0.2)
    assert wait(lambda: seen and s.core.idle())
    assert seen[-1] == pytest.approx((0.1, 0.05))
    cmd(s, "program_new")
    cmd(s, "program_insert", type="movej")
    cmd(s, "program_update", path=[0], params={"velocity": 0.2})
    cmd(s, "program_run")
    assert wait(lambda: len(seen) >= 2 and s.core.runner.state.name == "IDLE")
    assert seen[-1][0] == pytest.approx(0.05)
    with pytest.raises(CommandError, match="override"):
        cmd(s, "speed_override", percent=0)


def test_rename_waypoint_updates_every_step(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="movej")
    cmd(s, "program_duplicate", path=[0])
    old = s.program.steps[0].params["target"]
    r = cmd(s, "program_rename_waypoint", old=old, new="above_vial")
    assert [st.params["target"] for st in s.program.steps] == ["above_vial", "above_vial"]
    wp = r["program"]["rows"][0]["waypoint"]
    assert wp["name"] == "above_vial" and wp["users"] == 2 and len(wp["joints_deg"]) == 6
    with pytest.raises(CommandError, match="letters"):
        cmd(s, "program_rename_waypoint", old="above_vial", new="bad name")
    cmd(s, "program_undo")
    assert s.program.steps[0].params["target"] == old


def test_virtual_tcp_from_the_browser(s):
    r = cmd(s, "tcp_save", name="spatula_tip", parent="robotiq_tcp", xyz_mm=[0, 0, 120], rpy_deg=[0, 0, 0])
    assert r["ok"]
    t = next(x for x in s.state()["calibration"]["tcps"] if x["name"] == "spatula_tip")
    assert t["parent"] == "robotiq_tcp" and t["xyz_mm"] == pytest.approx([0, 0, 120])
    with pytest.raises(CommandError, match="already used"):
        cmd(s, "tcp_save", name="spatula_tip", parent="robotiq_tcp", xyz_mm=[0, 0, 1], rpy_deg=[0, 0, 0])
    cmd(s, "tcp_save", name="spatula_tip", parent="robotiq_tcp", xyz_mm=[0, 5, 130], rpy_deg=[0, 0, 90], replace=True)
    t = next(x for x in s.state()["calibration"]["tcps"] if x["name"] == "spatula_tip")
    assert t["xyz_mm"] == pytest.approx([0, 5, 130]) and t["rpy_deg"][2] == pytest.approx(90)
    with pytest.raises(CommandError, match="2 m"):
        cmd(s, "tcp_save", name="x", parent="robotiq_tcp", xyz_mm=[0, 0, 5000], rpy_deg=[0, 0, 0])
    with pytest.raises(CommandError, match="parent"):
        cmd(s, "tcp_save", name="x", parent="nope", xyz_mm=[0, 0, 1], rpy_deg=[0, 0, 0])
    with pytest.raises(CommandError, match="cannot be deleted"):
        cmd(s, "tcp_delete", name="robotiq_tcp")
    cmd(s, "tcp_delete", name="spatula_tip")
    assert all(x["name"] != "spatula_tip" for x in s.state()["calibration"]["tcps"])


def test_run_history_records_programs(s):
    cmd(s, "program_new", name="hist")
    cmd(s, "program_insert", type="wait", params={"seconds": 0.05})
    cmd(s, "program_run")
    assert wait(lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None)
    runs = cmd(s, "history")["runs"]
    assert runs[0]["kind"] == "program" and runs[0]["name"] == "hist" and runs[0]["ok"]
    assert runs[0]["duration_s"] >= 0.04
    assert s.core.history.path.exists()


def test_an_executed_plan_disappears(s):
    q = [math.degrees(v) for v in s.core.bridge.snapshot().joints]
    q[0] += 3
    assert cmd(s, "plan_joints", q_deg=q, vel=0.5, acc=0.5)["ok"]
    assert wait(lambda: s.state()["plan"].get("what"))
    pid = s.state()["plan"]["id"]
    cmd(s, "plan_execute")
    assert s.state()["plan"]["id"] != pid                              # browsers drop the preview at once
    assert wait(lambda: not s.state()["plan"].get("what"))            # and the core forgets it (job thread)
    assert wait(lambda: s.core.idle())


def test_hold_to_run_stops_without_renewals(s):
    t = [0.0]
    s.clock = lambda: t[0]
    cmd(s, "program_new")
    cmd(s, "program_insert", type="movej")
    q = [math.degrees(v) for v in s.core.bridge.snapshot().joints]
    q[0] += 40                                                           # a long move (slow fake: 40 deg/s)
    s.core.move_joints([math.radians(v) for v in q], 1.0, 1.0)
    r = cmd(s, "program_move_here", path=[0], vel=0.1, acc=0.1, hold=True)
    assert r["ok"]
    assert wait(lambda: not s.core.idle())
    t[0] += 0.2
    assert cmd(s, "hold_renew")["holding"]
    s.tick()
    assert not s.core.idle()                                             # renewed: keeps moving
    t[0] += 1.0                                                          # the browser went quiet (tap, lost link)
    s.tick()
    assert wait(lambda: s.core.idle(), 5)
    assert any("hold-to-run button released" in line for _, line in s.logs) or \
        any("hold-to-run" in x for x in s.core.bridge.drain_logs())


def test_stale_program_commands_are_refused(s):
    r = cmd(s, "program_new")
    rev = r["program"]["rev"]
    r = cmd(s, "program_insert", type="wait", rev=rev)
    assert r["program"]["rev"] == rev + 1 and s.state()["runner"]["rev"] == rev + 1
    with pytest.raises(CommandError, match="program was changed"):
        cmd(s, "program_delete_step", path=[0], rev=rev)                # sent by a page showing the old program
    assert len(s.program.steps) == 1
    cmd(s, "program_undo", rev=rev + 1)
    assert s.state()["runner"]["rev"] == rev + 2                         # undo is a change too
    cmd(s, "program_get", rev=0)                                         # reads are never refused


def test_tcp_change_refused_while_a_program_runs(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="wait", params={"seconds": 2.0})
    cmd(s, "program_run")
    assert wait(lambda: s.core.runner.state.name == "RUNNING")
    with pytest.raises(CommandError, match="busy"):
        cmd(s, "set_tcp", name="tool0")
    cmd(s, "program_stop")


# ---- review 3: a STOP while MoveIt is still planning (or while the job is queued) must not be lost
def _joints_deg(s):
    return [math.degrees(v) for v in s.core.bridge.snapshot().joints]


def _result(s, job, timeout=10.0):
    assert wait(lambda: any(r["id"] == job for r in s.results), timeout)
    return next(r for r in s.results if r["id"] == job)


def test_stop_during_planning_cancels_the_move(s):
    s.core.bridge.plan_delay = 0.5
    q0 = _joints_deg(s)
    q = list(q0)
    q[0] += 30
    job = cmd(s, "move_joints", q_deg=q, vel=1.0, acc=1.0)["job"]
    time.sleep(0.15)                                                   # still "planning"
    s.stop("test")
    r = _result(s, job)
    assert not r["ok"]
    assert abs(_joints_deg(s)[0] - q0[0]) < 0.5                         # the arm never moved


def test_a_queued_move_does_not_start_after_stop(s):
    q = _joints_deg(s)
    q0 = q[1]
    q[1] += 20
    busy = [s._submit("slow job", lambda: (time.sleep(0.4), (True, "done"))[1])["job"] for _ in range(4)]
    first = busy[0]                                                    # all 4 job threads are busy
    second = cmd(s, "move_joints", q_deg=q, vel=1.0, acc=1.0)["job"]   # so this one waits in the queue
    time.sleep(0.1)
    s.stop("test")
    assert _result(s, first)["ok"]
    r = _result(s, second)
    assert not r["ok"] and "STOP" in r["msg"]
    assert abs(_joints_deg(s)[1] - q0) < 0.5


def test_hold_tap_during_planning_does_not_run_the_move(s):
    t = [0.0]
    s.clock = lambda: t[0]
    cmd(s, "program_new")
    cmd(s, "program_insert", type="movej")                              # waypoint = here
    q = _joints_deg(s)
    q[0] += 30
    s.core.move_joints([math.radians(v) for v in q], 1.0, 1.0)
    s.core.bridge.plan_delay = 0.5
    job = cmd(s, "program_move_here", path=[0], vel=1.0, acc=1.0, hold=True)["job"]
    time.sleep(0.1)
    t[0] += 1.0                                                         # released after a tap: no renewals
    s.tick()
    assert not _result(s, job)["ok"]
    assert abs(_joints_deg(s)[0] - q[0]) < 0.5                          # stayed where it was


def test_program_stop_during_planning_stops_the_program(s):
    cmd(s, "program_new")
    cmd(s, "program_insert", type="movej")
    q = _joints_deg(s)
    q[0] += 30
    s.core.move_joints([math.radians(v) for v in q], 1.0, 1.0)
    s.core.bridge.plan_delay = 0.5
    cmd(s, "program_run")
    time.sleep(0.15)
    s.stop("test")
    assert wait(lambda: s.core.runner.state.name == "IDLE")
    assert abs(_joints_deg(s)[0] - q[0]) < 0.5
