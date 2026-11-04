"""Stale sensor / status data must never count as OK: F/T freshness, joint-state and controller-list liveness."""
import os
import threading
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.bridge_api import Snapshot, wrench_fresh  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from test_ros_bridge_static import ros_bridge_module  # noqa: E402,F401
from moveit_gui.program import Program  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture
def core(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=50.0, move_time_scale=10.0)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    c = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    c.ft_bias.cfg["zero_s"] = 0.2
    yield c
    c.stop_all("test end")
    bridge.shutdown()


def test_wrench_fresh():
    assert wrench_fresh(Snapshot(wrench=[1] * 6, wrench_age=0.05)) == [1] * 6
    assert wrench_fresh(Snapshot(wrench=[1] * 6, wrench_age=0.5)) is None
    assert wrench_fresh(Snapshot(wrench=[1] * 6, wrench_age=None)) is None
    assert wrench_fresh(Snapshot(wrench=None, wrench_age=0.0)) is None


def test_fake_snapshot_has_ages(core):
    s = core.bridge.snapshot()
    assert s.wrench_age is not None and s.controllers_age is not None and s.spin_age is not None


def test_zero_ft_rejects_frozen_sensor(core):
    core.bridge.fake_wrench_age = 5.0
    ok, msg = core.program_zero_ft(lambda: False)
    assert not ok and "no F/T data" in msg


def test_program_hooks_wrench_none_when_stale(core):
    from moveit_gui.core import _CoreProgramHooks as H
    h = H(core)
    assert h.wrench() is not None
    core.bridge.fake_wrench_age = 5.0
    assert h.wrench() is None


def test_robot_refusal_on_stale_joint_states(core):
    assert core.robot_refusal() is None
    core.bridge.fake_joint_state_age = 5.0
    assert "joint states" in core.robot_refusal()
    core.bridge.fake_joint_state_age = None
    assert "joint states" in core.robot_refusal()


def test_robot_refusal_on_stale_controller_list(core, monkeypatch):
    real = core.bridge.snapshot

    def old():
        s = real()
        s.controllers_age = 10.0
        return s
    monkeypatch.setattr(core.bridge, "snapshot", old)
    assert "controller list" in core.robot_refusal()


def test_watchdog_stops_a_motion_when_joint_states_die(core):
    core.update()
    assert core.begin_motion("test") is None
    core.bridge.fake_joint_state_age = 5.0
    core.update()
    assert core.last_refusal and "joint states" in core.last_refusal
    assert core.command_source == "idle"


def test_force_start_refused_while_program_owns_force(core):
    class Owned:
        alive, releasing, owner = True, False, "program"
    core._force = Owned()
    from moveit_gui import force
    ok, msg = core.force_start(force.WrenchTarget([0, 0, 1, 0, 0, 0], "tool"))
    assert not ok and "program" in msg
    core._force = None


def _run_contact(core, freeze_after=None):
    steps = [{"type": "move_until_contact", "direction": [0, 0, 1], "frame": "base", "distance_mm": 200,
              "force_n": 40, "velocity": 0.02}]
    prog = Program.from_dict({"version": 2, "name": "t", "robot": "ur7e_lab", "steps": steps}, core.profile)
    logs = []
    core.runner.on_log = logs.append
    if freeze_after is None:
        core.bridge.fake_wrench_age = 5.0
    else:
        threading.Timer(freeze_after, lambda: setattr(core.bridge, "fake_wrench_age", 5.0)).start()
    core.runner.start(prog)
    t0 = time.monotonic()
    time.sleep(0.2)
    while core.runner.state.name != "IDLE" and time.monotonic() - t0 < 15:
        core.update()
        time.sleep(0.05)
    assert core.runner.state.name == "IDLE"
    return logs


def test_move_until_contact_refused_when_ft_stale(core):
    logs = _run_contact(core)
    assert any("no fresh F/T data" in m for m in logs)


def test_move_until_contact_aborts_when_ft_freezes_midway(core):
    logs = _run_contact(core, freeze_after=0.3)
    assert any("stopped updating" in m for m in logs)


def test_ros_bridge_ages_and_timeouts(ros_bridge_module):
    from types import SimpleNamespace as NS
    from test_ros_bridge_static import _bare_bridge
    b = _bare_bridge(ros_bridge_module)
    now = time.monotonic()
    b._t_wrench, b._t_controllers, b._t_spin = now - 0.5, now - 10.0, now
    b._snap.connected = {}
    b._snap.controllers = {"x": "active"}
    s = b.snapshot()
    assert s.wrench_age >= 0.5 and s.controllers_age >= 10.0 and s.spin_age < 1.0
    assert s.controllers == {}                      # a 10 s old list is not reported as current
    assert s.connected["ROS"] is True
    b._t_spin = now - 30.0
    assert b.snapshot().connected["ROS"] is False   # dead executor thread
    traj = NS(points=[NS(time_from_start=NS(sec=40, nanosec=0))])
    b._snap.speed_scaling = 0.5
    assert b._execution_timeout(traj) > 80.0         # 40 s at half speed is not cancelled after 60 s
    assert b._plan_timeout() >= 10.0
