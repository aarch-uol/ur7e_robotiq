"""Every shutdown path leaves a zero wrench behind (admittance controllers keep the last target for ever)."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import force  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


def make(tmp_path):
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=50.0, move_time_scale=10.0)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    return profile, bridge, RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)


def test_fake_shutdown_publishes_zero_on_every_wrench_topic(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile, bridge, _core = make(tmp_path)
    topics = {m.wrench["topic"] for m in profile.modes.values() if m.wrench.get("topic")}
    assert topics
    bridge.wrenches.clear()
    bridge.shutdown()
    assert {t for t, _f, _w in bridge.wrenches} == topics
    assert all(not any(w) for _t, _f, w in bridge.wrenches)


def test_stop_all_zeroes_a_running_force(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile, bridge, core = make(tmp_path)
    name = next(n for n, m in profile.modes.items() if m.wrench.get("topic") and m.supports_moveit)
    ok, _ = core.modes.switch_to(name)
    assert ok
    core.update()
    ok, msg = core.force_start(force.WrenchTarget([0, 0, 3, 0, 0, 0], "tool"))
    assert ok, msg
    core.stop_all("GUI closed")
    assert not core.force_active
    assert bridge.wrenches and not any(bridge.wrenches[-1][2])
    bridge.shutdown()


def test_ros_bridge_shutdown_zeroes_all_publishers():
    import sys
    from pathlib import Path
    from unittest import mock
    sys.path.insert(0, str(Path(__file__).parent))
    import test_ros_bridge_static as st
    with mock.patch.dict(sys.modules, {m: mock.MagicMock() for m in st.STUBS}):
        sys.modules.pop("moveit_gui.ros_bridge", None)
        import moveit_gui.ros_bridge as rb
        b = st._bare_bridge(rb)
        b._wrench_pubs = {"/a/wrench": mock.MagicMock(), "/b/wrench": mock.MagicMock()}
        b._fd_pub = None
        b._logs = __import__("collections").deque()
        b.node = mock.MagicMock()
        b._executor = mock.MagicMock()
        b.moveit = None
        b.cancel_motion = mock.MagicMock()
        sent = []
        b.publish_wrench = lambda topic, frame, w: sent.append((topic, list(w)))
        b.shutdown()
        sys.modules.pop("moveit_gui.ros_bridge", None)
    assert {t for t, _ in sent} == {"/a/wrench", "/b/wrench"}
    assert all(not any(w) for _, w in sent)
    assert sent.count(("/a/wrench", [0.0] * 6)) >= 2
    b.cancel_motion.assert_called()
    b._executor.shutdown.assert_called()
    b.node.destroy_node.assert_called()
