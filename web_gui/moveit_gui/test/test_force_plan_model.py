"""OPERATE > Force (wrench targets for admittance / impedance), Plan → preview → Execute, and the 3D view's model."""
import math
import time

import numpy as np
import pytest

from moveit_gui import force
from moveit_gui import transforms as tf
from moveit_gui.force import ForceError, ForceStream, WrenchTarget
from moveit_gui.robot_model import mesh_path, model_or_error, parse_urdf
from moveit_gui.web.service import CommandError, WebService

from test_jog import core  # noqa: F401  (fixture)

CFG = dict(force.DEFAULTS)


def _wait(cond, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def _result(s, jid):
    assert _wait(lambda: any(r["id"] == jid for r in s.results))
    return next(r for r in s.results if r["id"] == jid)


# -------------------------------------------------------------- force: maths
def test_wrench_limits_are_enforced():
    WrenchTarget.parse([0, 0, -20, 0, 0, 0], "tool", CFG)
    with pytest.raises(ForceError, match="over the limit"):
        WrenchTarget.parse([15, 15, 0, 0, 0, 0], "tool", CFG)          # |F| = 21.2 N
    with pytest.raises(ForceError, match="torque"):
        WrenchTarget.parse([0, 0, 0, 0, 0, 2.5], "base", CFG)
    with pytest.raises(ForceError, match="six"):
        WrenchTarget.parse([0, 0, float("nan"), 0, 0, 0], "base", CFG)
    with pytest.raises(ForceError, match="frame"):
        WrenchTarget.parse([0] * 6, "world", CFG)


def test_wrench_is_moved_to_the_controller_point_and_frame():
    # TCP 0.2 m below the sensor along base z, tool frame = base rotated 90° about z.
    R_tool = tf.quat_to_matrix(tf.rpy_to_quat(0, 0, math.pi / 2))
    T_tcp = tf.from_rotation(R_tool, [0.5, 0.0, 0.1])
    T_sensor = tf.from_rotation(np.eye(3), [0.5, 0.0, 0.3])
    # 10 N along tool x = base +y, at the TCP
    out = force.wrench_for_controller(WrenchTarget([10, 0, 0, 0, 0, 0], "tool"), T_tcp, T_sensor, T_sensor)
    assert np.allclose(out[:3], [0, 10, 0], atol=1e-9)
    # lever arm: (p_tcp - p_sensor) x F = (0,0,-0.2) x (0,10,0) = (2, 0, 0)
    assert np.allclose(out[3:], [2.0, 0, 0], atol=1e-9)
    # base-frame target, controller frame = sensor rotated: the vector is re-expressed
    T_rot = tf.from_rotation(R_tool, [0.5, 0.0, 0.1])
    out = force.wrench_for_controller(WrenchTarget([0, 0, -5, 0, 0, 0], "base"), T_tcp, T_rot, T_tcp)
    assert np.allclose(out, [0, 0, -5, 0, 0, 0], atol=1e-9)


def test_ramp_limits_the_rate_of_both_halves():
    cur = np.zeros(6)
    goal = np.array([10.0, 0, 0, 0, 0, 1.0])
    nxt = force.ramp_toward(cur, goal, 0.1, rate_f=10, rate_t=1)
    assert nxt[0] == pytest.approx(1.0) and nxt[5] == pytest.approx(0.1)
    assert np.allclose(force.ramp_toward(goal - 0.01, goal, 0.1, 10, 1), goal)


class _Clock:
    t = 100.0

    def __call__(self):
        return self.t


def _stream(cfg=None, target=(0, 0, -4, 0, 0, 0)):
    sent, ended = [], []
    clock = _Clock()
    s = ForceStream(WrenchTarget(list(target), "tool"), sent.append, ended.append, {**CFG, **(cfg or {})},
                    rate_hz=200.0, clock=clock)
    return s, sent, ended, clock


def test_release_ramps_down_and_ends_on_zero():
    s, sent, ended, clock = _stream({"ramp_force_nps": 1000.0})
    s.start()
    for _ in range(20):
        clock.t += 0.01
        s.hold()
        time.sleep(0.006)
    assert _wait(lambda: sent and sent[-1][2] == pytest.approx(-4.0))
    s.release()
    for _ in range(40):
        clock.t += 0.01
        time.sleep(0.006)
    assert _wait(lambda: ended)
    assert ended == ["released"] and sent[-1] == [0.0] * 6


def test_a_missed_renewal_zeroes_at_once():
    s, sent, ended, clock = _stream()
    s.start()
    time.sleep(0.05)
    clock.t += 1.0                     # no hold() for 1 s
    assert _wait(lambda: ended)
    assert "not renewed" in ended[0] and sent[-1] == [0.0] * 6


def test_max_duration_releases():
    s, sent, ended, clock = _stream({"max_duration_s": 0.5, "ramp_force_nps": 1000.0})
    s.start()
    for _ in range(100):
        clock.t += 0.02
        s.hold()
        time.sleep(0.004)
        if ended:
            break
    assert _wait(lambda: ended) and "maximum duration" in ended[0]


# ----------------------------------------------------------- force: through the core
def _enter(core, mode):
    ok, msg = core.switch_mode(mode)
    assert ok, msg
    core.update()


def test_force_needs_a_mode_with_a_wrench_target(core):
    assert "needs Admittance or Impedance" in core.force_refusal()
    ok, why = core.force_start(WrenchTarget([0, 0, -5, 0, 0, 0], "tool"))
    assert not ok and "Admittance" in why


def test_admittance_force_is_streamed_in_the_sensor_frame_then_zeroed(core):
    _enter(core, "admittance")
    b = core.bridge
    ok, msg = core.force_start(WrenchTarget([0, 0, 5, 0, 0, 0], "base"))
    assert ok, msg
    assert core.busy and core.force_active
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:          # 5 N at 10 N/s: 0.5 s ramp
        core.force_hold()
        time.sleep(0.05)
    topic, frame, w = b.wrenches[-1]
    assert topic == "/admittance_controller/wrench_reference" and frame == "ft_fts_link"
    assert np.linalg.norm(w[:3]) == pytest.approx(5.0, abs=0.05)
    ok, _ = core.move_joints(b.snapshot().joints, 0.1, 0.1)       # no MoveIt motion while pressing
    assert not ok
    core.force_stop("test")
    assert b.wrenches[-1][2] == [0.0] * 6 and not core.busy


def test_impedance_force_goes_to_the_root_frame_and_stop_all_zeroes(core):
    core.bridge.servo_available = False
    _enter(core, "impedance")
    ok, msg = core.force_start(WrenchTarget([0, 0, -3, 0, 0, 0], "base"))
    assert ok, msg
    assert _wait(lambda: len(core.bridge.wrenches) > 3)
    topic, frame, _ = core.bridge.wrenches[-1]
    assert topic == "/cartesian_impedance_controller/set_cartesian_wrench" and frame == "world"
    core.stop_all()
    assert core.bridge.wrenches[-1][2] == [0.0] * 6 and not core.force_active


def test_a_mode_change_stops_the_force(core):
    _enter(core, "admittance")
    assert core.force_start(WrenchTarget([0, 0, 2, 0, 0, 0], "tool"))[0]
    core.bridge.controllers["admittance_controller"] = "inactive"       # e.g. the driver stopped it
    core.bridge.controllers["admittance_trajectory_controller"] = "inactive"
    t0 = time.monotonic()
    while core.force_active and time.monotonic() - t0 < 2:
        core.update()
        core.force_hold()
        time.sleep(0.05)
    assert not core.force_active and "mode changed" in core.force_last_end
    assert core.bridge.wrenches[-1][2] == [0.0] * 6


def test_web_force_needs_the_lease_but_release_and_stop_do_not(core):
    s = WebService(core)
    try:
        _enter(core, "admittance")
        with pytest.raises(CommandError, match="take control"):
            s.command("a", "force_apply", {"wrench": [0, 0, 1, 0, 0, 0]})
        s.take_lease("a")
        with pytest.raises(CommandError, match="over the limit"):
            s.command("a", "force_apply", {"wrench": [0, 0, 100, 0, 0, 0]})
        assert s.command("a", "force_apply", {"wrench": [0, 0, 1, 0, 0, 0], "frame": "base"})["ok"]
        assert s.command("a", "force_hold", {})["active"]
        assert s.command("a", "force_apply", {"wrench": [0, 0, 2, 0, 0, 0], "frame": "base"})["msg"] == \
            "force target changed"
        st = s.state()["force"]
        assert st["active"] and st["target"] == [0, 0, 2, 0, 0, 0]
        assert s.command("other", "force_stop", {})["ok"]                  # anyone may zero it
        assert not core.force_active
        assert s.info()["force_modes"] == ["admittance", "impedance"]
    finally:
        s.shutdown()


# -------------------------------------------------------------- plan / execute
def test_plan_previews_without_moving_then_executes_once(core):
    s = WebService(core)
    try:
        s.take_lease("a")
        q0 = list(core.bridge.snapshot().joints)
        goal = [math.degrees(v) for v in q0]
        goal[0] += 10
        r = _result(s, s.command("a", "plan_joints", {"q_deg": goal, "vel": 0.5})["job"])
        assert r["ok"] and "planned" in r["msg"]
        assert core.bridge.snapshot().joints == q0                       # planning moves nothing
        p = s.command("a", "plan_get", {})
        assert p["id"] == 1 and p["plan"]["points"][-1]["q"][0] == pytest.approx(q0[0] + math.radians(10), abs=1e-4)
        assert s.state()["plan"]["id"] == 1
        r = _result(s, s.command("a", "plan_execute", {})["job"])
        assert r["ok"], r["msg"]
        assert core.bridge.snapshot().joints[0] == pytest.approx(q0[0] + math.radians(10), abs=1e-3)
        assert core.plan_view() is None                                   # used once
        r = _result(s, s.command("a", "plan_execute", {})["job"])
        assert not r["ok"] and "Plan first" in r["msg"]
    finally:
        s.shutdown()


def test_a_plan_is_refused_once_the_arm_has_moved(core):
    q0 = list(core.bridge.snapshot().joints)
    assert core.plan_joints([q0[0] + 0.2] + q0[1:], 0.5, 0.5)[0]
    assert core.move_joints([q0[0] - 0.1] + q0[1:], 1.0, 1.0)[0]
    ok, why = core.execute_plan()
    assert not ok and "moved since planning" in why


# ------------------------------------------------------------------ 3D model
URDF = """<robot name="r">
  <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
  <link name="world"/>
  <link name="base"><visual><origin xyz="0 0 0.1"/><geometry><mesh filename="package://pkg/meshes/base.dae"
    scale="0.001 0.001 0.001"/></geometry><material name="grey"/></visual></link>
  <link name="tool"><visual><geometry><cylinder radius="0.03" length="0.02"/></geometry></visual></link>
  <link name="finger"/>
  <joint name="fix" type="fixed"><parent link="world"/><child link="base"/></joint>
  <joint name="j1" type="revolute"><parent link="base"/><child link="tool"/><origin xyz="0 0 0.3" rpy="0 0 1.57"/>
    <axis xyz="0 0 1"/></joint>
  <joint name="j2" type="revolute"><parent link="tool"/><child link="finger"/><axis xyz="1 0 0"/>
    <mimic joint="j1" multiplier="-1"/></joint>
</robot>"""


def test_urdf_is_reduced_for_the_browser():
    m = parse_urdf(URDF)
    assert m["root"] == "world"
    v = m["links"]["base"][0]
    assert v["geometry"] == {"type": "mesh", "uri": "package://pkg/meshes/base.dae", "scale": [0.001] * 3}
    assert v["xyz"] == [0, 0, 0.1] and v["rgba"] == [0.5, 0.5, 0.5, 1.0]
    assert m["links"]["tool"][0]["geometry"]["type"] == "cylinder"
    j2 = next(j for j in m["joints"] if j["name"] == "j2")
    assert j2["mimic"] == {"joint": "j1", "multiplier": -1.0, "offset": 0.0}
    assert not model_or_error(None)["ok"] and not model_or_error("<robot")["ok"]


def test_meshes_are_served_only_from_the_package_share(tmp_path):
    share = tmp_path / "pkg"
    (share / "meshes").mkdir(parents=True)
    (share / "meshes" / "base.dae").write_text("<COLLADA/>")
    (share / "meshes" / "notes.txt").write_text("x")
    (tmp_path / "secret.dae").write_text("x")
    lookup = {"pkg": share}.get
    assert mesh_path("pkg", "meshes/base.dae", lookup) == share / "meshes" / "base.dae"
    (share / "meshes" / "link.dae").symlink_to(tmp_path / "secret.dae")   # --symlink-install: the package's own
    assert mesh_path("pkg", "meshes/link.dae", lookup) is not None
    assert mesh_path("pkg", "/etc/x.dae", lookup) is None
    assert mesh_path("pkg", "meshes/notes.txt", lookup) is None          # not a mesh type
    assert mesh_path("pkg", "../secret.dae", lookup) is None             # outside the share
    assert mesh_path("other", "meshes/base.dae", lookup) is None
    assert mesh_path("..", "secret.dae", lookup) is None


def test_web_serves_the_model(core):
    s = WebService(core)
    try:
        assert not s.robot_model()["ok"]
        core.bridge.fake_robot_description = URDF
        m = s.robot_model()
        assert m["ok"] and m["root"] == "world"
        assert s.state()["gripper_joint"] == core.profile.gripper["joint_name"]
    finally:
        s.shutdown()
