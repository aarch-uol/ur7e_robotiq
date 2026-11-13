"""Cartesian speed limit (mm/s) of linear moves: retiming, schema / validation, fake bridge, core, web, RosBridge."""
import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from moveit_gui import cart_speed  # noqa: E402
from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import TcpDef  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.geometry import PoseSpec  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.program import Program, Step  # noqa: E402
from moveit_gui.robot_model import parse_urdf  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

PROFILE = ROOT / "profiles" / "ur7e_lab.yaml"

# base_link -j1 (about z)-> link1 -fixed 0.5 m along x-> tool0: tool0 moves on a circle of 0.5 m
URDF = """<robot name="arm">
  <link name="base_link"/><link name="link1"/><link name="tool0"/>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="fix" type="fixed"><parent link="link1"/><child link="tool0"/><origin xyz="0.5 0 0" rpy="0 0 0"/></joint>
</robot>"""


def _dur(t):
    return SimpleNamespace(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def _traj(n=11, q_end=0.2, t_end=1.0):
    """j1 0 -> q_end in t_end at constant speed: tool0 at 0.5 * q_end / t_end m/s."""
    qd = q_end / t_end
    pts = [SimpleNamespace(positions=[q_end * i / (n - 1)], velocities=[qd], accelerations=[0.5],
                           time_from_start=_dur(t_end * i / (n - 1))) for i in range(n)]
    return SimpleNamespace(joint_names=["j1"], points=pts)


def _secs(d):
    return d.sec + d.nanosec * 1e-9


# ---------------------------------------------------------------- retiming
def test_values_and_limits():
    assert cart_speed.check_mm_s(20, 250) == 20.0
    for bad in (0, -1, 251, float("nan"), "20", True, None):
        with pytest.raises(ValueError):
            cart_speed.check_mm_s(bad, 250)
    assert cart_speed.optional_mm_s(None, 250) == 0.0
    assert cart_speed.optional_mm_s("", 250) == 0.0
    assert cart_speed.optional_mm_s(0, 250) == 0.0
    assert cart_speed.optional_mm_s("15", 250) == 15.0
    with pytest.raises(ValueError):
        cart_speed.optional_mm_s(300, 250)
    prof = RobotProfile.load(PROFILE)
    assert prof.max_cart_speed_mm_s == 250.0
    prof.moveit.pop("max_cartesian_speed_mm_s")
    assert prof.max_cart_speed_mm_s == cart_speed.DEFAULT_MAX_MM_S       # an older profile without the key


def test_limit_speed_scales_the_whole_trajectory_uniformly():
    point_at = cart_speed.point_fk(parse_urdf(URDF), "base_link", "tool0")
    traj = _traj()                               # 0.5 m * 0.2 rad/s = 0.1 m/s
    assert cart_speed.peak_speed(traj, point_at) == pytest.approx(0.1, rel=0.01)
    k = cart_speed.limit_speed(traj, point_at, 0.025)
    assert k == pytest.approx(4.0, rel=0.01)
    assert _secs(traj.points[-1].time_from_start) == pytest.approx(4.0, rel=0.01)
    assert traj.points[3].velocities[0] == pytest.approx(0.2 / k)
    assert traj.points[3].accelerations[0] == pytest.approx(0.5 / k / k)
    assert [p.positions for p in traj.points] == [p.positions for p in _traj().points]   # same path
    assert cart_speed.peak_speed(traj, point_at) <= 0.025 * 1.001
    assert cart_speed.limit_speed(traj, point_at, 0.05) == 1.0           # slow enough already: unchanged


def test_the_limited_point_is_the_tcp_not_the_link():
    model = parse_urdf(URDF)
    tip = cart_speed.point_fk(model, "base_link", "tool0", [0.5, 0, 0])   # a virtual TCP 0.5 m further out
    assert cart_speed.peak_speed(_traj(), tip) == pytest.approx(0.2, rel=0.01)
    traj = _traj()
    cart_speed.limit_speed(traj, tip, 0.05)
    assert _secs(traj.points[-1].time_from_start) == pytest.approx(4.0, rel=0.01)
    assert cart_speed.point_fk(model, "base_link", "no_such_link") is None


# ------------------------------------------------------------------ schema
def test_program_field_validation_summary_and_old_files(tmp_path):
    prof = RobotProfile.load(PROFILE)
    old = {"version": 2, "name": "old", "steps": [
        {"type": "move_relative", "offset_mm": [0, 0, 5], "frame": "tool", "velocity": 0.1},
        {"type": "move_until_contact", "direction": [0, 0, 1], "distance_mm": 10, "force_n": 3}]}
    assert Program.from_dict(old, prof).validate(prof) == []          # no speed_mm_s: as before

    def problems(step):
        return Program(steps=[Step.from_dict(step)]).validate(prof)

    rel = {"type": "move_relative", "offset_mm": [0, 0, 5], "frame": "tool"}
    assert problems({**rel, "speed_mm_s": 20}) == []
    assert problems({**rel, "speed_mm_s": 250}) == []
    for bad in (0, -5, 251, "fast", True):
        assert any("mm/s" in p for p in problems({**rel, "speed_mm_s": bad})), bad
    assert any("linear moves" in p for p in problems({"type": "movej", "joints": [0] * 6, "speed_mm_s": 20}))
    contact = {"type": "move_until_contact", "direction": [0, 0, 1], "distance_mm": 10, "force_n": 3, "speed_mm_s": 5}
    assert problems(contact) == []

    assert "20 mm/s" in Step.from_dict({**rel, "speed_mm_s": 20}).summary()
    assert "mm/s" not in Step.from_dict({**rel, "cartesian": False, "speed_mm_s": 20}).summary()   # not linear
    movel = Step("move_pose", {"target": "a", "cartesian": True, "speed_mm_s": 12.5})
    assert "12.5 mm/s" in movel.summary()
    assert "mm/s" not in Step("move_pose", {"target": "a", "speed_mm_s": 12.5}).summary()
    assert "5 mm/s" in Step.from_dict(contact).summary()

    p = Program(name="t", steps=[Step.from_dict({**rel, "speed_mm_s": 20})])
    p.save(tmp_path / "t.yaml")
    back = Program.load(tmp_path / "t.yaml", prof)
    assert back.steps[0].params["speed_mm_s"] == 20


# ------------------------------------------------------------- fake bridge
@pytest.fixture
def core(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(PROFILE)
    bridge = FakeBridge(profile, joint_speed=60.0, move_time_scale=10.0)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    c = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    c.update()
    yield c
    c.stop_all("test end")
    bridge.shutdown()


def test_fake_bridge_takes_path_length_over_speed_plus_the_ramps(core):
    b = core.bridge
    T0 = b.T_base_tool0.copy()
    T1 = T0.copy()
    T1[:3, 3] += [0.0, 0.0, 0.1]                         # 100 mm, Accel 50 %: ramps at 250 mm/s^2
    assert b.move_pose("tool0", T1, True, 0.5, 0.5, cart_speed=0.02)
    assert b.last_move_s == pytest.approx(0.1 / 0.02 + 0.02 / 0.25, rel=1e-6)   # L / v + v / a
    assert np.allclose(b.T_base_tool0[:3, 3], T1[:3, 3])
    assert b.move_pose("tool0", T0, True, 0.5, 0.5)
    assert b.last_move_s == pytest.approx(0.4)            # no limit: the old timing (0.2 / vel)
    assert b.move_pose("tool0", T1, True, 0.5, 0.5, cart_speed=1.0)
    assert b.last_move_s == pytest.approx(2 * math.sqrt(0.1 / 0.25))   # too short to reach 1 m/s: a triangle
    assert b.move_pose("tool0", T0, False, 0.5, 0.5, cart_speed=0.02)
    assert b.last_move_s == pytest.approx(0.4)            # not linear: no limit
    # a pure rotation about tool0 still moves an offset TCP point: 90 deg about z, point 0.1 m along x
    R = T0.copy()
    R[:3, :3] = T0[:3, :3] @ tf.mm_deg_to_T([0, 0, 0], [0, 0, 90])[:3, :3]
    assert b.move_pose("tool0", R, True, 0.5, 0.5, cart_speed=0.01, speed_point=[0.1, 0, 0])
    assert b.last_move_s == pytest.approx(math.sqrt(2) * 0.1 / 0.01 + 0.01 / 0.25, rel=1e-6)


def test_core_passes_mm_s_as_m_s_with_the_gui_speed_and_the_tcp_point(core):
    b = core.bridge
    here = core.geo.current_pose(core.profile.base_link, "tool0")
    T = here.T()
    T[:3, 3] += [0, 0, 0.02]
    goal = PoseSpec.from_T(core.profile.base_link, "tool0", T)
    ok, msg = core.move_pose(goal, True, 0.5, 0.5, speed_mm_s=40)
    assert ok, msg
    assert b.last_cart_speed == pytest.approx(0.04)
    core.set_speed_override(0.5)
    ok, msg = core.move_pose(PoseSpec.from_T(core.profile.base_link, "tool0", here.T()), True, 0.5, 0.5, speed_mm_s=40)
    assert ok, msg
    assert b.last_cart_speed == pytest.approx(0.02)       # GUI SPEED applies on top, never faster
    ok, msg = core.move_pose(goal, False, 0.5, 0.5, speed_mm_s=40)
    assert ok and b.last_cart_speed == 0.0                # planned (not linear) moves ignore it
    ok, msg = core.move_pose(goal, True, 0.5, 0.5, speed_mm_s=400)
    assert not ok and "mm/s" in msg
    # a virtual TCP: the bridge gets the planning link and the TCP point in it
    core.store.put_tcp(TcpDef("probe_tip", "virtual", parent="tool0", T_parent_tcp=tf.make([0, 0, 0.2])))
    target = core.geo.planning_target(PoseSpec.from_T(core.profile.base_link, "probe_tip", T))
    assert target.link == "tool0" and np.allclose(target.T_link_tcp[:3, 3], [0, 0, 0.2])
    with mock.patch.object(b, "move_pose", return_value=True) as mp:
        ok, _ = core.move_pose(PoseSpec.from_T(core.profile.base_link, "probe_tip", T), True, 0.5, 0.5, speed_mm_s=10)
    assert ok
    kw = mp.call_args.kwargs
    assert kw["cart_speed"] == pytest.approx(0.005) and np.allclose(kw["speed_point"], [0, 0, 0.2])


# --------------------------------------------------------------- web / runner
@pytest.fixture
def web(core):
    s = WebService(core)
    s.tick()
    s.take_lease("a")
    yield s
    s.shutdown()


def _wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_operate_move_pose_takes_mm_s(web):
    b = web.core.bridge
    tcp = web.core.store.active_tcp
    xyz, rpy = tf.T_to_mm_deg(web.core.geo.current_pose(web.core.profile.base_link, tcp).T())
    args = {"frame": web.core.profile.base_link, "xyz_mm": [xyz[0], xyz[1], xyz[2] + 10], "rpy_deg": list(rpy),
            "linear": True, "vel": 0.5, "acc": 0.5}
    with pytest.raises(CommandError, match="mm/s"):
        web.command("a", "move_pose", {**args, "speed_mm_s": 999})
    r = web.command("a", "move_pose", {**args, "speed_mm_s": 25})
    assert r["ok"]
    assert _wait(lambda: any(x["id"] == r["job"] for x in web.results))
    assert b.last_cart_speed == pytest.approx(0.025)
    r = web.command("a", "move_pose", {**args, "xyz_mm": list(xyz), "speed_mm_s": ""})     # empty = old behaviour
    assert _wait(lambda: any(x["id"] == r["job"] for x in web.results))
    assert b.last_cart_speed == 0.0
    assert web.info()["max_cart_speed_mm_s"] == 250.0


def test_program_editor_and_runner_use_speed_mm_s(web):
    b = web.core.bridge
    d = {"version": 2, "name": "t", "steps": [
        {"type": "move_relative", "offset_mm": [0, 0, 10], "frame": "base", "velocity": 0.5}]}
    web.command("a", "program_set_yaml", {"text": yaml.safe_dump(d)})
    web.command("a", "program_update", {"path": [0], "params": {"speed_mm_s": 20}})
    assert web.program.steps[0].params["speed_mm_s"] == 20.0
    rows = web.program_view()["program"]["rows"]
    assert "20 mm/s" in rows[0]["detail"]
    with pytest.raises(CommandError, match="mm/s"):
        web.command("a", "program_update", {"path": [0], "params": {"speed_mm_s": 300}})
    with mock.patch.object(b, "move_pose", wraps=b.move_pose) as mp:
        web.command("a", "program_run", {})
        assert _wait(lambda: web.core.program_result is not None, 20)
    assert web.core.program_result[0], web.core.program_result
    assert mp.call_args.kwargs["cart_speed"] == pytest.approx(0.02)
    assert b.last_move_s == pytest.approx(0.5 + 0.02 / 0.05, rel=0.01)  # 10 mm at 20 mm/s + ramps at 50 mm/s^2
    web.command("a", "program_update", {"path": [0], "params": {"speed_mm_s": 0}})      # 0 = off, not saved
    assert "speed_mm_s" not in web.program.steps[0].params


# --------------------------------------------------------------- RosBridge
STUBS = ["rclpy", "rclpy.action", "rclpy.callback_groups", "rclpy.executors", "rclpy.node", "rclpy.qos", "rclpy.serialization",
         "rclpy.time", "builtin_interfaces", "builtin_interfaces.msg", "controller_manager_msgs",
         "controller_manager_msgs.srv", "geometry_msgs", "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
         "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv", "lifecycle_msgs", "lifecycle_msgs.msg",
         "trajectory_msgs", "trajectory_msgs.msg", "action_msgs", "action_msgs.srv"]


@pytest.fixture
def rb():
    with mock.patch.dict(sys.modules, {m: mock.MagicMock() for m in STUBS}):
        sys.modules.pop("moveit_gui.ros_bridge", None)
        import moveit_gui.ros_bridge as mod
        yield mod
        sys.modules.pop("moveit_gui.ros_bridge", None)


def _ros_bridge(rb, plan_result):
    b = object.__new__(rb.RosBridge)
    b.profile = RobotProfile.load(PROFILE)
    b._lock = threading.Lock()
    b.moveit = mock.MagicMock()
    b.moveit.plan.return_value = plan_result
    b.lines = []
    b.log = b.lines.append
    b.robot_description = lambda: URDF
    return b


def test_ros_bridge_retimes_a_linear_plan_for_the_tcp_point(rb):
    traj = _traj()
    b = _ros_bridge(rb, traj)
    out = b.plan_pose("tool0", np.eye(4), True, 0.1, 0.1, cart_speed=0.05, speed_point=[0.5, 0, 0])
    assert out is traj
    assert b.moveit.plan.call_args.kwargs["cartesian"] is True
    # TCP 1 m from the axis turns 0.2 rad: 0.2 m at a constant 0.05 m/s + ramps at 500 x 10 % = 50 mm/s^2
    assert _secs(traj.points[-1].time_from_start) == pytest.approx(0.2 / 0.05 + 0.05 / 0.05, rel=0.01)
    assert len(traj.points) > 150 and traj.points[0].velocities == [0.0] and traj.points[-1].velocities == [0.0]
    assert any("constant 50.0 mm/s" in x for x in b.lines), b.lines

    traj = _traj()
    b = _ros_bridge(rb, traj)
    b.plan_pose("tool0", np.eye(4), True, 0.1, 0.1)                                   # no limit: untouched
    assert _secs(traj.points[-1].time_from_start) == pytest.approx(1.0)
    b.plan_pose("tool0", np.eye(4), False, 0.1, 0.1, cart_speed=0.01)                # not linear: untouched
    assert _secs(traj.points[-1].time_from_start) == pytest.approx(1.0)


def test_ros_bridge_refuses_a_limit_it_cannot_compute(rb):
    b = _ros_bridge(rb, _traj())
    b.robot_description = lambda: None
    assert b.plan_pose("tool0", np.eye(4), True, 0.1, 0.1, cart_speed=0.05) is None
    b.moveit.plan.assert_not_called()                     # never planned, so never sent unlimited
    b = _ros_bridge(rb, _traj())
    assert b.plan_pose("not_a_link", np.eye(4), True, 0.1, 0.1, cart_speed=0.05) is None


def test_ros_bridge_move_pose_passes_the_limit_to_plan_pose(rb):
    b = _ros_bridge(rb, _traj())
    b.stop_epoch = 0
    b.plan_pose = mock.MagicMock(return_value="traj")
    b._execute_on_active_controller = mock.MagicMock(return_value=True)
    assert b.move_pose("tool0", np.eye(4), True, 0.2, 0.3, cart_speed=0.03, speed_point=[0, 0, 0.1])
    args = b.plan_pose.call_args.args
    assert args[0] == "tool0" and args[2:] == (True, 0.2, 0.3, 0.03, [0, 0, 0.1])


# ------------------------------------------------------------ constant speed
L1, L2 = 0.4, 0.3
PLANAR = f"""<robot name="planar">
  <link name="base_link"/><link name="a"/><link name="b"/><link name="tool0"/>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="a"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="3.0"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="{L1} 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="3.0"/></joint>
  <joint name="t" type="fixed"><parent link="b"/><child link="tool0"/><origin xyz="{L2} 0 0"/></joint>
</robot>"""


def _ik(x, y):
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    q2 = math.acos(c2)
    return [math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2)), q2]


def _line(n, p0=(0.35, -0.05), p1=(0.35, 0.05), t_end=0.7):
    """A MoveIt-like Cartesian plan: n IK points on a straight line, timed anyhow (here: evenly)."""
    pts = []
    for i in range(n):
        u = i / (n - 1)
        q = _ik(p0[0] + u * (p1[0] - p0[0]), p0[1] + u * (p1[1] - p0[1]))
        pts.append(SimpleNamespace(positions=q, velocities=[0.0, 0.0], accelerations=[0.0, 0.0],
                                   time_from_start=_dur(t_end * u)))
    return SimpleNamespace(joint_names=["j1", "j2"], points=pts)


@pytest.fixture
def planar():
    model = parse_urdf(PLANAR)
    return model, cart_speed.point_fk(model, "base_link", "tool0")


def _speeds(traj, point_at):
    P = np.array([point_at(dict(zip(traj.joint_names, p.positions))) for p in traj.points])
    t = np.array([_secs(p.time_from_start) for p in traj.points])
    return P, t, np.linalg.norm(np.diff(P, axis=0), axis=1) / np.diff(t)


def test_constant_speed_cruises_at_v_between_exact_endpoints(planar):
    model, at = planar
    traj = _line(8)                                       # sparse: 8 points for 100 mm, as move_group returned
    q_first, q_last = list(traj.points[0].positions), list(traj.points[-1].positions)
    v, a = 0.02, 0.5
    big = {"j1": 10.0, "j2": 10.0}
    info = cart_speed.constant_speed(traj, at, v, a, big, big)
    P, t, seg = _speeds(traj, at)
    assert info["slowed"] == 1.0 and info["points"] > 90  # densified to <= 1 mm steps
    assert traj.points[0].positions == pytest.approx(q_first) and traj.points[-1].positions == pytest.approx(q_last)
    assert traj.points[0].velocities == [0.0, 0.0] and traj.points[-1].velocities == [0.0, 0.0]
    assert t[0] == 0.0 and np.all(np.diff(t) > 0)
    assert all(p.accelerations == [] for p in traj.points)   # positions + velocities: cubic splines
    L = 0.1
    assert t[-1] == pytest.approx(cart_speed.trapezoid_duration(L, v, a), rel=0.01)
    ramp = v * v / (2 * a)                                # 0.4 mm
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    cruise = (s[1:] > ramp + 0.002) & (s[:-1] < L - ramp - 0.002)
    assert cruise.sum() > 80
    assert np.all(np.abs(seg[cruise] - v) <= 0.02 * v)   # constant within 2 %
    assert cart_speed.peak_speed(traj, at) <= v * 1.02
    line = np.array([0.35, 0, 0])                         # every point on the straight line x = 0.35 (< 0.1 mm)
    assert np.max(np.abs(P[:, 0] - line[0])) < 1e-4


def test_constant_speed_ramps_and_a_short_move_is_a_triangle(planar):
    _, at = planar
    traj = _line(3, (0.35, 0.0), (0.35, 0.002))           # 2 mm: 20 mm/s at 50 mm/s^2 needs 8 mm to get there
    big = {"j1": 10.0, "j2": 10.0}
    cart_speed.constant_speed(traj, at, 0.02, 0.05, big, big)
    _, t, seg = _speeds(traj, at)
    assert t[-1] == pytest.approx(2 * math.sqrt(0.002 / 0.05), rel=0.01)
    assert seg.max() <= math.sqrt(0.05 * 0.002) * 1.02    # peak of the triangle, under 20 mm/s


def test_constant_speed_keeps_the_joint_limits_by_slowing_the_whole_move(planar):
    _, at = planar
    traj = _line(8)
    vlim, alim = {"j1": 0.05, "j2": 0.05}, {"j1": 0.2, "j2": 0.2}
    info = cart_speed.constant_speed(traj, at, 0.1, 1.0, vlim, alim)
    assert info["slowed"] > 1.0 and info["v"] < 0.1
    qd = np.array([p.velocities for p in traj.points])
    t = np.array([_secs(p.time_from_start) for p in traj.points])
    assert np.max(np.abs(qd)) <= 0.05 + 1e-9
    assert np.max(np.abs(np.gradient(qd, t, axis=0))) <= 0.2 * 1.001
    _, _, seg = _speeds(traj, at)
    assert seg.max() <= info["v"] * 1.02                  # still a constant (slower) cruise


def test_joint_limits_come_from_the_urdf_and_the_profile(planar):
    model, _ = planar
    prof = RobotProfile.load(PROFILE)
    vel, acc = cart_speed.joint_limits(model, prof, ["j1", "j2", "nolimit"], 0.5, 0.1)
    assert vel["j1"] == pytest.approx(1.5) and vel["nolimit"] == pytest.approx(math.pi * 0.5)
    assert acc["j1"] == pytest.approx(5.0 * 0.1)
    assert cart_speed.cart_accel_m_s2(prof, 0.1) == pytest.approx(0.05)


def test_retime_linear_falls_back_to_the_uniform_slowdown(planar):
    model = parse_urdf(URDF)
    at = cart_speed.point_fk(model, "base_link", "tool0", [-0.5, 0, 0])   # the TCP sits on the joint axis
    traj = _traj()
    lines = []
    big = {"j1": 10.0}
    cart_speed.retime_linear(traj, at, 0.01, 0.5, big, big, lines.append)
    assert any("constant speed not possible" in x for x in lines)
    assert _secs(traj.points[-1].time_from_start) == pytest.approx(1.0)   # nothing moves: left as planned
    _, at2 = planar
    traj = _line(8)
    lines = []
    cart_speed.retime_linear(traj, at2, 0.02, 0.5, {"j1": 10.0, "j2": 10.0}, {"j1": 10.0, "j2": 10.0}, lines.append)
    assert any("constant 20.0 mm/s" in x for x in lines)
    assert not any("slowed" in x for x in lines)
    assert cart_speed.peak_speed(traj, at2) <= 0.02 * 1.02


def test_the_joint_limits_are_kept_between_the_points_of_a_short_move():
    """Two points, 0.5 mm of TCP travel but 0.1 rad of joint motion (a point 5 mm from the axis): both ends are at
    rest, so only the controller's cubic between them shows the joint speed. The retimed move keeps the limits."""
    traj = SimpleNamespace(joint_names=["j1"], points=[
        SimpleNamespace(positions=[0.0], velocities=[], accelerations=[], time_from_start=_dur(0.0)),
        SimpleNamespace(positions=[0.1], velocities=[], accelerations=[], time_from_start=_dur(1.0))])
    point_at = lambda q: np.array([0.005 * math.cos(q["j1"]), 0.005 * math.sin(q["j1"]), 0.0])   # noqa: E731
    info = cart_speed.constant_speed(traj, point_at, 0.010, 0.5, {"j1": 1.0}, {"j1": 5.0})
    t = [_secs(p.time_from_start) for p in traj.points]
    Q = [p.positions for p in traj.points]
    Qd = [p.velocities for p in traj.points]
    vpk, apk = cart_speed.spline_extrema(t, Q, Qd)
    assert vpk[0] <= 1.0 + 1e-6 and apk[0] <= 5.0 + 1e-6
    assert info["slowed"] > 1.0
    # dense check of the same spline: the closed form found the real peaks
    ts = np.linspace(t[0], t[-1], 2001)
    from scipy.interpolate import CubicHermiteSpline
    sp = CubicHermiteSpline(t, np.asarray(Q)[:, 0], np.asarray(Qd)[:, 0])
    assert np.max(np.abs(sp(ts, 1))) == pytest.approx(vpk[0], rel=1e-3)
    assert np.max(np.abs(sp(ts, 2))) == pytest.approx(apk[0], rel=1e-3)


def test_peak_speed_sees_a_tcp_peak_between_the_points():
    traj = SimpleNamespace(joint_names=["j1"], points=[
        SimpleNamespace(positions=[0.0], velocities=[0.0], accelerations=[], time_from_start=_dur(0.0)),
        SimpleNamespace(positions=[0.1], velocities=[0.0], accelerations=[], time_from_start=_dur(0.1))])
    point_at = lambda q: np.array([0.5 * math.cos(q["j1"]), 0.5 * math.sin(q["j1"]), 0.0])   # noqa: E731
    # mean 0.5 m/s; a cubic from rest to rest peaks at 1.5x the mean in the middle
    assert cart_speed.peak_speed(traj, point_at) == pytest.approx(0.75, rel=0.03)
