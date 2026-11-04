"""OPERATE fixes: 100 Hz F/T samples for the chart, a plan never outlives another motion, a quick re-run."""
import os
import time

import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.bridge_api import WrenchHistory  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


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
    svc.take_lease("a")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def test_wrench_history_decimates_to_100_hz_and_keeps_two_seconds():
    h = WrenchHistory()
    for k in range(1500):                                    # 3 s of a 500 Hz sensor
        h.add(1000.0 + k * 0.002, [k, 0, 0, 0, 0, 0])
    got = h.since(0)
    assert 195 <= len(got) <= 205                            # 2 s at 100 Hz
    assert all(b[0] - a[0] >= 0.009 for a, b in zip(got, got[1:]))
    assert h.since(got[-2][0]) == [got[-1]]


def test_each_state_carries_the_samples_since_the_last_one(s):
    s.state()
    time.sleep(0.1)
    a = s.state()["ft_samples"]
    time.sleep(0.1)
    b = s.state()["ft_samples"]
    assert 5 <= len(a) <= 15 and 5 <= len(b) <= 15          # ~100 Hz, not one value per state
    assert b[0][0] > a[-1][0]                                # nothing repeated
    assert len(a[0]) == 7                                    # t, Fx Fy Fz Tx Ty Tz


def test_a_plan_is_dropped_by_another_motion_and_by_the_arm_moving(s):
    c = s.core
    q = list(c.bridge.snapshot().joints)
    assert c.plan_joints([q[0] + 0.3] + q[1:], 0.5, 0.5)[0]
    assert s.state()["plan"].get("what")
    assert c.move_joints([q[0] - 0.1] + q[1:], 1.0, 1.0)[0]          # MoveJ instead of Execute
    assert not s.state()["plan"].get("what")
    ok, why = c.execute_plan()
    assert not ok and "moved since planning" in why
    # a plan, then the arm moves some other way (pendant, program, jog): the preview goes away by itself
    q = list(c.bridge.snapshot().joints)
    assert c.plan_joints([q[0] + 0.3] + q[1:], 0.5, 0.5)[0]
    c.bridge.move_joints([q[0] + 0.1] + q[1:], 1.0, 1.0)              # not through the GUI's motion claim
    s.tick()
    assert not s.state()["plan"].get("what")


def test_run_again_right_after_a_run_ends_is_not_silently_ignored(s):
    d = {"version": 2, "name": "t", "steps": [{"type": "wait", "seconds": 0.05}]}
    s.command("a", "program_set_yaml", {"text": yaml.safe_dump(d)})
    for _ in range(5):
        s.command("a", "program_run", {})
        assert wait(lambda: s.core.program_result is not None, 5)
        assert s.core.program_result == (True, "finished")
