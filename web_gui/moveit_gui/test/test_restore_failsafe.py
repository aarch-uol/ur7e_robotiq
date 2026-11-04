"""Automatic restores (gains, F/T bias) act only when it is positively known to be safe."""
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.ft_bias import PARAMS  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
IMP = "/cartesian_impedance_controller"
FT = "/ft/force_torque_sensor_broadcaster"


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def core(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    c = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    c.ft_bias.cfg["zero_s"] = 0.3
    yield c
    bridge.shutdown()


def poll_gains(g):
    g._last_poll = 0
    g.poll(True)
    assert wait(lambda: not g._polling)


def poll_bias(f):
    f._last_poll = 0.0
    f.poll(True)
    assert wait(lambda: not f._polling)


def gains_setup(core):
    core.gains.remember("impedance", {"stiffness.translation.x": 321.0})
    return core.bridge.node_params[IMP]


def test_gains_not_restored_when_the_controller_list_is_unavailable(core):
    p = gains_setup(core)
    core.bridge.list_controllers = lambda: None
    poll_gains(core.gains)
    assert p["stiffness.translation.x"] != 321.0
    poll_gains(core.gains)
    assert sum("restore waits" in m for m in core.bridge._logs) == 1            # logged once
    del core.bridge.list_controllers                                            # list is back, controller inactive
    poll_gains(core.gains)
    assert p["stiffness.translation.x"] == 321.0


def test_gains_not_restored_when_the_controller_is_not_confirmed_inactive(core):
    p = gains_setup(core)
    core.bridge.controllers["cartesian_impedance_controller"] = "active"
    core.gains.current_mode = lambda: "unknown"                                 # mode detection says nothing useful
    poll_gains(core.gains)
    assert p["stiffness.translation.x"] != 321.0
    ok, msg = core.gains.restore("impedance", "test")
    assert not ok and "not confirmed inactive" in msg


def test_bias_not_restored_when_state_is_unknown_or_an_ft_mode_is_active(core):
    ft = core.ft_bias
    assert core.ft_bias.set([0.5, 0, 0, 0, 0, 0], "typed")[0]
    core.bridge.node_params[FT].update({p: 0.0 for p in PARAMS})                # broadcaster restarted
    core.bridge.list_controllers = lambda: None
    poll_bias(ft)
    assert ft.lost() and core.bridge.node_params[FT]["offset.force.x"] == 0.0
    poll_bias(ft)
    assert sum("restore waits" in m for m in core.bridge._logs) == 1
    del core.bridge.list_controllers
    core.bridge.controllers["admittance_controller"] = "active"                 # F/T mode active, mode "unknown"
    core.bridge.controllers["admittance_trajectory_controller"] = "active"
    ok, msg = ft.restore()
    assert not ok and "uses the F/T reading" in msg
    assert core.bridge.node_params[FT]["offset.force.x"] == 0.0
    core.bridge.controllers["admittance_controller"] = "inactive"
    core.bridge.controllers["admittance_trajectory_controller"] = "inactive"
    poll_bias(ft)
    assert not ft.lost()
