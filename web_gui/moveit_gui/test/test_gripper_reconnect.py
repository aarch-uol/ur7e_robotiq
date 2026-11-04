"""Reconnect gripper: the stack started while the robot was powered off, so the Robotiq hardware failed to configure
and its controllers never started. OPERATE brings them up again once the robot has power."""
import os
import time

import pytest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.bridge_api import GRIPPER_NO_ANSWER, reconnect_gripper  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
CTRLS = ["jsb", "grip", "act", "status"]


def ops(hw="unconfigured", ctrls=None, answers=True):
    calls = []
    st = {"hw": {"g": hw}, "c": dict(ctrls or {})}

    def rec(name, ok=True):
        def f(*a):
            calls.append((name,) + a)
            return ok
        return f

    def hw_activate(n):
        calls.append(("hw_activate", n))
        if answers:
            st["hw"][n] = "active"
        return answers, st["hw"][n]
    o = SimpleNamespace(hw_states=lambda: dict(st["hw"]), hw_activate=hw_activate, controllers=lambda: dict(st["c"]),
                        set_params_file=lambda c, f: (calls.append(("params", c, f)) or True, "ok"),
                        load=rec("load"), configure=rec("configure"), activate=rec("activate"))
    return o, calls


def test_after_a_powered_off_start_everything_is_brought_up_in_launch_order():
    o, calls = ops(ctrls={"jsb": "unconfigured"})
    ok, msg = reconnect_gripper(o, "g", CTRLS, "/p.yaml")
    assert ok and "started jsb, grip, act, status" in msg
    assert calls[0] == ("hw_activate", "g")
    assert ("load", "jsb") not in calls and ("configure", "jsb") in calls          # loaded already: configure only
    assert ("params", "grip", "/p.yaml") in calls and ("load", "grip") in calls
    assert calls[-1] == ("activate", CTRLS)                                       # one switch for all


def test_no_tool_power_says_so_and_touches_no_controller():
    o, calls = ops(answers=False)
    ok, msg = reconnect_gripper(o, "g", CTRLS, None)
    assert not ok and msg.startswith(GRIPPER_NO_ANSWER)
    assert [c[0] for c in calls] == ["hw_activate"]


def test_pressing_it_when_all_is_up_changes_nothing():
    o, calls = ops(hw="active", ctrls={c: "active" for c in CTRLS})
    assert reconnect_gripper(o, "g", CTRLS, None) == (True, "gripper already connected")
    assert calls == []


def test_finalized_hardware_needs_a_restart():
    o, _ = ops(hw="finalized")
    ok, msg = reconnect_gripper(o, "g", CTRLS, None)
    assert not ok and "restart" in msg


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




def test_operate_reconnects_the_gripper_once_the_robot_has_power(s):
    b = s.core.bridge
    b.fake_gripper_start_failed()
    s.tick()
    st = s.state()
    assert st["connected"]["Gripper"] is False

    b.robot_mode = "POWER_OFF"
    s.tick()
    with pytest.raises(CommandError, match="Power the robot on first"):
        s.command("a", "gripper_reconnect", {})

    b.robot_mode = "RUNNING"
    s.tick()
    s.command("a", "gripper_reconnect", {})
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5 and not b.snapshot().connected["Gripper"]:
        time.sleep(0.05)
    assert b.snapshot().connected["Gripper"]
    assert set(b.gripper_controllers.values()) == {"active"} and len(b.gripper_controllers) == 4
    assert ("params_file", "robotiq_gripper_controller",
            s.core.profile.gripper["params_file"]) in b.calls
