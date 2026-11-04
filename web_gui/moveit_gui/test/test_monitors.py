"""Joint limits for the OPERATE joint monitor: parsed from the URDF, exposed in INFO; velocities in the state."""
import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.robot_model import parse_urdf  # noqa: E402
from moveit_gui.web.service import WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture
def svc(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    (tmp_path / "progs").mkdir()
    (tmp_path / "pol").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    s = WebService(core)
    yield s
    s.shutdown()
    bridge.shutdown()


def test_parse_urdf_limits():
    m = parse_urdf('<robot name="r"><link name="a"/><link name="b"/><link name="c"/>'
                   '<joint name="j1" type="revolute"><parent link="a"/><child link="b"/>'
                   '<limit lower="-1.5" upper="2.5" effort="1" velocity="1"/></joint>'
                   '<joint name="j2" type="continuous"><parent link="b"/><child link="c"/></joint></robot>')
    j = {x["name"]: x for x in m["joints"]}
    assert j["j1"]["limit"] == {"lower": -1.5, "upper": 2.5}
    assert j["j2"]["limit"] is None


def test_info_joint_limits(svc):
    names = svc.core.profile.joint_names
    assert svc.info()["joint_limits_deg"] == [None] * len(names)       # no robot_description yet
    joints = "".join(f'<link name="l{i}"/><joint name="{n}" type="revolute"><parent link="l{i}"/>'
                     f'<child link="l{i + 1}"/><limit lower="-{math.pi}" upper="{math.pi / 2}" effort="1" velocity="1"/></joint>'
                     for i, n in enumerate(names))
    svc.core.bridge.fake_robot_description = f'<robot name="r">{joints}<link name="l{len(names)}"/></robot>'
    assert svc.info()["joint_limits_deg"] == [[-180.0, 90.0]] * len(names)


def test_state_has_joint_velocity_key(svc):
    svc.tick()
    assert "joint_vel_dps" in svc.state()
