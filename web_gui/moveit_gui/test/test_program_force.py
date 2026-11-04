"""PROGRAMS force work: move_relative, move_until_contact, force / force_off, zero_ft and the scripted scrape
baseline, run end to end on the fake robot."""
import os
import time

import numpy as np
import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.program import Program  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
ADM_TOPIC = "/admittance_controller/wrench_reference"


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
    bridge.ft_noise = 0.002
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    core.ft_bias.cfg["zero_s"] = 0.2
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a")
    yield svc
    svc.core.stop_all("test end")
    svc.shutdown()
    bridge.shutdown()


def cmd(s, _command, **args):
    return s.command("a", _command, args)


def tick_until(s, cond, timeout=20.0):
    return wait(lambda: (s.heartbeat("a") or True) and (s.tick() or True) and cond(), timeout)


def run(s, steps, waypoints=None, timeout=30.0):
    """Load these steps as the open program, run it to the end, return (ok, msg)."""
    d = {"version": 2, "name": "t", "robot": "ur7e_lab", "steps": steps}
    if waypoints:
        d["waypoints"] = waypoints
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.runner.state.name != "IDLE" or s.core.program_result is not None, 5)
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None, timeout)
    return s.core.program_result


def logs(s):
    """Every log line of this service so far (it drains the bridge's log on each tick; s.logs is a window)."""
    s.tick()
    seen = s.__dict__.setdefault("_test_logs", {})
    seen.update({seq: line for seq, line in s.logs})
    return [seen[k] for k in sorted(seen)]


def down_tool_axis(s):
    """The TCP axis that points down (base -Z) in the fake's start pose, as a direction list."""
    R = tcp_base(s)[:3, :3]
    return [round(float(v), 6) for v in R.T @ [0, 0, -1]]


def tcp_base(s):
    return s.core.geo.current_T_base_tcp(s.core.store.active_tcp)


def published(s):
    return [w for t, _, w in s.core.bridge.wrenches if t == ADM_TOPIC]


def test_move_relative_in_tool_base_and_fixture_axes(s):
    T0 = tcp_base(s)
    ok, msg = run(s, [{"type": "move_relative", "offset_mm": [0, 0, 20], "frame": "tool", "velocity": 0.5}])
    assert ok, msg
    T1 = tcp_base(s)
    assert np.allclose(T1[:3, 3], T0[:3, 3] + T0[:3, :3] @ [0, 0, 0.02], atol=1e-4)      # along the TCP's own Z
    ok, msg = run(s, [{"type": "move_relative", "offset_mm": [10, 0, 0], "frame": "base", "velocity": 0.5}])
    assert ok, msg
    assert np.allclose(tcp_base(s)[:3, 3], T1[:3, 3] + [0.01, 0, 0], atol=1e-4)
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([400, 0, 0], [0, 0, 90])))
    T2 = tcp_base(s)
    ok, msg = run(s, [{"type": "move_relative", "offset_mm": [5, 0, 0], "frame": "vial", "velocity": 0.5}])
    assert ok, msg
    assert np.allclose(tcp_base(s)[:3, 3], T2[:3, 3] + [0, 0.005, 0], atol=1e-4)          # vial X = base Y


def test_relative_moves_are_validated(s):
    p = Program.from_dict({"version": 2, "steps": [
        {"type": "move_relative", "offset_mm": [400, 0, 0]},
        {"type": "move_relative", "offset_mm": [0, 0, 0]},
        {"type": "move_until_contact", "direction": [0, 0, 0], "distance_mm": 500, "force_n": 0},
        {"type": "move_until_contact", "direction": [0, 0, 1], "distance_mm": 10, "force_n": 3, "velocity": 0.5},
        {"type": "force", "wrench": [0, 0, 50, 0, 0, 0]},
        {"type": "force", "wrench": [0, 0, 5, 0, 0, 0], "frame": "world"}]}, s.core.profile)
    text = "\n".join(p.validate(s.core.profile))
    for why in ("over 300 mm", "with no offset", "needs a direction", "distance_mm", "force_n", "at most 0.1",
                "50.0 N over the profile limit", "tool or base"):
        assert why in text, why
    with pytest.raises(CommandError, match="force_n"):
        cmd(s, "program_new")
        cmd(s, "program_insert", type="move_until_contact", params={"force_n": 0})


def test_move_until_contact_stops_at_the_surface(s):
    z_tcp = tcp_base(s)[2, 3]
    s.core.bridge.fake_surface = {"z_m": z_tcp - 0.010, "k": 3000.0}      # 10 mm below the TCP
    ok, msg = run(s, [{"type": "move_until_contact", "direction": down_tool_axis(s), "frame": "tool", "distance_mm": 30,
                       "force_n": 3, "velocity": 0.05}])
    assert ok, msg
    z = tcp_base(s)[2, 3]
    assert z_tcp - 0.0125 < z < z_tcp - 0.0095                  # stopped about 1 mm in (3 N at 3000 N/m), not 30 mm
    assert s.core.runner.last_contact["contact"]
    assert any("contact: |ΔF| reached 3 N" in line for line in logs(s))


def test_no_contact_fails_unless_told_to_continue(s):
    step = {"type": "move_until_contact", "direction": [0, 0, 1], "frame": "base", "distance_mm": 5,
            "force_n": 3, "velocity": 0.1}
    ok, msg = run(s, [step])
    assert not ok and "failed" in msg
    assert any("no contact within 5" in line for line in logs(s))
    ok, msg = run(s, [dict(step, on_no_contact="continue"), {"type": "wait", "seconds": 0.05}])
    assert ok, msg


def test_force_needs_a_force_mode(s):
    ok, msg = run(s, [{"type": "force", "wrench": [0, 0, 5, 0, 0, 0]}])
    assert not ok
    assert any("applying a force needs Admittance" in line for line in logs(s))
    assert not published(s)


def test_force_stays_applied_across_steps_and_force_off_ramps_it_out(s):
    ok, msg = run(s, [
        {"type": "controller", "mode": "admittance"},
        {"type": "force", "wrench": [0, 0, 4, 0, 0, 0], "frame": "tool"},
        {"type": "wait", "seconds": 0.3},
        {"type": "move_relative", "offset_mm": [0, 0, 2], "frame": "tool", "velocity": 0.5},
        {"type": "force_off"},
        {"type": "wait", "seconds": 0.1}], timeout=40)
    assert ok, msg
    w = published(s)
    norms = [np.linalg.norm(x[:3]) for x in w]
    assert max(norms) == pytest.approx(4.0, abs=0.05)            # reached the target (ramped, then held)
    k_max = int(np.argmax(norms))
    assert norms[1] < 1.0                                         # ramped in, not a step
    assert norms[-1] == 0.0 and any(0.3 < n < 3.7 for n in norms[k_max:])   # ramped out
    assert not s.core.force_active


def test_a_force_never_outlives_the_program(s):
    ok, msg = run(s, [{"type": "controller", "mode": "admittance"},
                      {"type": "force", "wrench": [0, 0, 3, 0, 0, 0]}], timeout=40)
    assert ok, msg
    assert not s.core.force_active and published(s)[-1] == [0.0] * 6
    assert any("program ended with a force applied: force ramped out" in line for line in logs(s))
    # STOP: zeroed at once
    d = {"version": 2, "name": "t", "steps": [{"type": "controller", "mode": "admittance"},
                                              {"type": "force", "wrench": [0, 0, 3, 0, 0, 0]},
                                              {"type": "wait", "seconds": 5}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.runner.current_path == (2,), 30)
    s.stop("test")
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE", 5)
    assert not s.core.force_active and published(s)[-1] == [0.0] * 6


def test_zero_ft_runs_in_free_air_and_is_refused_in_admittance(s):
    s.core.bridge.ft_raw_offset = np.array([0.5, 0.2, 1.0, 0.0, 0.0, 0.0])
    ok, msg = run(s, [{"type": "zero_ft"}])
    assert ok, msg
    assert np.linalg.norm(np.mean([s.core.bridge.snapshot().wrench for _ in range(30)], axis=0)[:3]) < 0.05
    ok, msg = run(s, [{"type": "controller", "mode": "admittance"}, {"type": "zero_ft"}], timeout=40)
    assert not ok
    assert any("Admittance uses the F/T reading" in line for line in logs(s))


def test_scrape_baseline_is_a_plain_program_that_runs(s):
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([300, 300, 0], [0, 0, 0])))
    with pytest.raises(CommandError, match="pressing force"):
        cmd(s, "program_scrape_baseline", frame="vial", force_n=50)
    d = down_tool_axis(s)
    k = int(np.argmax(np.abs(d)))
    press = ("+" if d[k] > 0 else "-") + "xyz"[k]                 # the tool axis pointing at the surface
    r = cmd(s, "program_scrape_baseline", frame="vial", press_axis=press, force_n=4, contact_n=2, strokes=1,
            stroke_mm=5, count=2, step_deg=10, standoff_mm=2, search_mm=6)
    types = [st["type"] for st in r["program"]["steps"]]
    assert types[:5] == ["comment", "controller", "move_pose", "zero_ft", "controller"]
    assert types.count("folder") == 2 and types[-1] == "controller"
    folder = s.program.steps[5]
    assert [c.type for c in folder.children] == ["move_pose", "move_until_contact", "force", "loop", "force_off",
                                                 "move_pose"]
    assert np.linalg.norm(folder.children[2].params["wrench"][:3]) == pytest.approx(4.0)
    assert not s.program.validate(s.core.profile, s.core.geo)
    # the taught pose touches a surface 0.5 mm further on
    s.core.bridge.fake_surface = {"z_m": tcp_base(s)[2, 3] - 0.0005, "k": 3000.0}
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None, 90)
    ok, msg = s.core.program_result
    assert ok, (msg, [line for line in logs(s)][-15:])
    assert max(np.linalg.norm(w[:3]) for w in published(s)) == pytest.approx(4.0, abs=0.05)
    assert not s.core.force_active


def test_a_force_that_ends_by_itself_fails_the_program(s):
    d = {"version": 2, "name": "t", "steps": [{"type": "controller", "mode": "admittance"},
                                              {"type": "force", "wrench": [0, 0, 3, 0, 0, 0]},
                                              {"type": "wait", "seconds": 1.0},
                                              {"type": "move_relative", "offset_mm": [0, 0, 5], "frame": "base"}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.runner.current_path == (2,), 30)
    s.core.force_stop("the controller stopped")                   # e.g. the driver dropped the controller
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None, 10)
    ok, msg = s.core.program_result
    assert not ok and "the force ended during step 3" in msg
    assert not [c for c in s.core.bridge.calls if c[0] == "move_pose"]  # the stroke never ran unloaded
