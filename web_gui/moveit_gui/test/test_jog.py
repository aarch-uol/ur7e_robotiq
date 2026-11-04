"""Jogging (step and hold-to-jog through Servo) and bringing back a hardware component before a mode switch."""
import math
import os
import re
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import jog  # noqa: E402
from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.jog import JogCommand, JogError, JogVelocity  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web import service as svc_mod  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

from test_gui_v23 import app, pump, win  # noqa: E402,F401  (fixtures)

ROOT = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture
def core(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    c = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    c.update()
    yield c
    c.jog_stop()
    bridge.shutdown()


def _admittance_with_servo(core):
    core.bridge.servo_available = True
    ok, msg = core.switch_mode("admittance")
    assert ok, msg
    core.update()


def _hold(core, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(0.05)
        core.jog_hold()


def _angle(Ta, Tb):
    return math.degrees(tf.angle_between(Ta, Tb))


# -------------------------------------------------------------------- geometry
def test_steps_translate_along_the_chosen_frame_and_turn_about_the_tcp():
    T = tf.from_rotation(tf.quat_to_matrix(tf.rpy_to_quat(math.pi, 0, math.pi / 2)), [0.4, 0.1, 0.3])
    up = jog.cartesian_step(T, JogCommand.parse("cartesian", "z", 1, "base"), 0.005, 0.0)
    assert np.allclose(up[:3, 3] - T[:3, 3], [0, 0, 0.005]) and np.allclose(up[:3, :3], T[:3, :3])
    tool_z = jog.cartesian_step(T, JogCommand.parse("cartesian", "z", 1, "tool"), 0.005, 0.0)
    assert np.allclose(tool_z[:3, 3] - T[:3, 3], T[:3, 2] * 0.005)          # tool Z points down here
    turned = jog.cartesian_step(T, JogCommand.parse("cartesian", "rx", -1, "tool"), 0.0, math.radians(10))
    assert np.allclose(turned[:3, 3], T[:3, 3]) and _angle(T, turned) == pytest.approx(10.0)
    assert jog.joint_step([0.0] * 6, JogCommand.parse("joint", 2, -1), 0.1) == [0, 0, -0.1, 0, 0, 0]


def test_servo_twist_carries_the_lever_arm_so_the_tcp_stays_put():
    T_tcp = tf.make([0.5, 0.0, 0.2])
    T_tip = tf.make([0.5, 0.0, 0.35])                    # tool0 150 mm above the TCP
    v, w = jog.servo_twist(JogCommand.parse("cartesian", "rx", 1, "base"), T_tcp, T_tip, 0.05, 0.2)
    assert w == pytest.approx([0.2, 0, 0])
    assert v == pytest.approx(np.cross([0.2, 0, 0], [0, 0, 0.15]).tolist())   # tip velocity, TCP velocity 0
    v, w = jog.servo_twist(JogCommand.parse("cartesian", "y", -1, "base"), T_tcp, T_tip, 0.05, 0.2)
    assert v == pytest.approx([0, -0.05, 0]) and w == [0, 0, 0]


def test_bad_jog_requests_are_refused():
    for args in (("cartesian", "q", 1), ("cartesian", "x", 2), ("joint", 6, 1), ("dance", 0, 1),
                 ("cartesian", "x", 1, "world")):
        with pytest.raises(JogError):
            JogCommand.parse(*args)


# ------------------------------------------------------------------------ core
def test_step_jog_moves_one_step_through_moveit(core):
    tcp = core.store.active_tcp
    T0 = core.geo.current_T_base_tcp(tcp)
    ok, msg = core.jog_step(JogCommand.parse("cartesian", "x", -1, "base"), 5.0, 0.5, 0.5)
    assert ok, msg
    assert core.bridge.calls[-1][0] == "move_pose" and core.bridge.calls[-1][3] is True      # linear
    assert np.allclose(core.geo.current_T_base_tcp(tcp)[:3, 3] - T0[:3, 3], [-0.005, 0, 0], atol=1e-6)
    ok, _ = core.jog_step(JogCommand.parse("joint", 0, 1), 1.0, 0.5, 0.5)
    assert ok and core.bridge.calls[-1][0] == "movej"
    ok, msg = core.jog_step(JogCommand.parse("joint", 0, 1), 45.0, 0.5, 0.5)
    assert not ok and "step" in msg                                                  # no huge "steps"


def test_hold_to_jog_needs_servo_in_both_position_and_admittance(core):
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "x", 1), 0.5)
    assert not ok and "Servo is not ready" in msg
    core.bridge.servo_available = True
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "x", 1), 0.5)
    assert ok, msg
    core.jog_stop()
    core.bridge.servo_available = False
    core.switch_mode("admittance")
    core.update()
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "x", 1), 0.5)
    assert not ok and "Servo is not ready" in msg
    assert not core.busy


def test_slider_velocity_can_change_while_held_in_both_modes(core):
    core.bridge.servo_available = True
    for mode in ("position", "admittance"):
        if core.mode != mode:
            assert core.switch_mode(mode)[0]
            core.update()
        v = JogVelocity.parse("joint", [0.4, 0, 0, 0, 0, 0])
        ok, msg = core.jog_start_velocity(v, 0.7)
        assert ok, msg
        assert ("servo_type", mode, "joint") in core.bridge.calls
        if mode == "position":
            assert core.mode == "servo_position"
            assert core.bridge.controllers["joint_trajectory_controller"] == "active"
            assert core.bridge.controllers["scaled_joint_trajectory_controller"] == "inactive"
        else:
            assert core.mode == "admittance"
        assert core.jog_hold([0, -0.7, 0, 0, 0, 0])
        time.sleep(0.06)
        assert any(call[0] == "joint_jog" and call[-1] == mode for call in core.bridge.calls)
        core.jog_stop()
        assert not core.jogging and not core.busy
        if mode == "position":
            assert core.mode == "position"
            assert core.bridge.controllers["scaled_joint_trajectory_controller"] == "active"
            assert core.bridge.controllers["joint_trajectory_controller"] == "inactive"


def test_hold_to_jog_moves_while_held_and_stops_on_release(core):
    _admittance_with_servo(core)
    tcp = core.store.active_tcp
    T0 = core.geo.current_T_base_tcp(tcp)
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "x", 1, "base"), 1.0)
    assert ok, msg
    assert core.busy and core.command_source.startswith("jog") and \
        ("servo_type", "admittance", "twist") in core.bridge.calls
    assert core.motion_refusal() == "robot is busy"                # nothing else moves the arm meanwhile
    _hold(core, 0.6)
    core.jog_stop()
    T1 = core.geo.current_T_base_tcp(tcp)
    dx = (T1[:3, 3] - T0[:3, 3])[0]
    assert 0.02 < dx < 0.035                                       # ~50 mm/s for ~0.6 s
    assert not core.busy and not core.jogging
    time.sleep(0.2)
    assert np.allclose(core.geo.current_T_base_tcp(tcp), T1)      # stopped means stopped


def test_a_jog_logs_what_servo_sent_and_how_far_the_arm_moved(core):
    _admittance_with_servo(core)
    core.bridge.drain_logs()
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "z", 1, "base"), 0.5)
    assert ok, msg
    _hold(core, 0.4)
    core.jog_stop()
    line = next(x for x in core.bridge.drain_logs() if "Servo jog summary" in x)
    sent = int(re.search(r"joint_trajectory: (\d+);", line).group(1))
    assert sent > 10 and "0 No warnings" in line and "TCP" in line
    assert float(re.search(r"TCP ([\d.]+) mm", line).group(1)) > 5.0         # ~25 mm/s for ~0.4 s


def test_hold_without_renewal_stops_by_itself(core):
    _admittance_with_servo(core)
    ok, _ = core.jog_start(JogCommand.parse("joint", 5, -1), 0.5)
    assert ok and ("servo_type", "admittance", "joint") in core.bridge.calls
    time.sleep(jog.HOLD_TIMEOUT_S + 0.3)                           # the UI froze / the network dropped
    assert not core.jogging and not core.busy
    assert any("hold not renewed" in line for line in core.bridge.drain_logs())


def test_held_rotation_turns_about_the_tcp(core):
    _admittance_with_servo(core)
    tcp = core.store.active_tcp
    T0 = core.geo.current_T_base_tcp(tcp)
    core.jog_start(JogCommand.parse("cartesian", "rz", 1, "tool"), 1.0)
    _hold(core, 0.5)
    core.jog_stop()
    T1 = core.geo.current_T_base_tcp(tcp)
    assert _angle(T0, T1) > 4.0
    assert np.linalg.norm(T1[:3, 3] - T0[:3, 3]) < 1e-3


def test_stop_all_ends_a_hold(core):
    _admittance_with_servo(core)
    core.jog_start(JogCommand.parse("cartesian", "z", 1), 0.5)
    core.stop_all()
    assert not core.jogging and not core.busy


def test_position_servo_switch_failure_never_starts_stream(core):
    core.bridge.servo_available = True
    core.bridge.fail_next_switch = True
    ok, msg = core.jog_start(JogCommand.parse("joint", 0, 1), 0.2)
    assert not ok and "Servo controller switch" in msg
    core.update()
    assert core.mode == "position" and not core.jogging and not core.busy
    assert not any(c[0] == "joint_jog" for c in core.bridge.calls)


def test_position_servo_stop_restores_scaled_trajectory_controller(core):
    core.bridge.servo_available = True
    ok, msg = core.jog_start(JogCommand.parse("joint", 0, 1), 0.2)
    assert ok, msg
    assert core.mode == "servo_position"
    core.stop_all()
    assert core.mode == "position"
    assert core.bridge.controllers["scaled_joint_trajectory_controller"] == "active"
    assert core.bridge.controllers["joint_trajectory_controller"] == "inactive"


# ---------------------------------------------------------------- web service
def test_web_jog_needs_the_lease_but_jog_stop_does_not(core):
    s = WebService(core)
    try:
        _admittance_with_servo(core)
        with pytest.raises(CommandError, match="take control"):
            s.command("a", "jog_start", {"kind": "cartesian", "axis": "x", "sign": 1})
        s.take_lease("a")
        r = s.command("a", "jog_start", {"kind": "cartesian", "axis": "x", "sign": 1, "speed": 0.2})
        assert r["ok"] and s.state()["jog"]["active"]
        assert s.command("a", "jog_hold", {})["jogging"] is True
        assert s.command("someone-else", "jog_stop", {})["ok"]               # anyone may stop
        assert not core.jogging
        assert s.command("a", "jog_hold", {})["jogging"] is False           # the page stops renewing
        with pytest.raises(CommandError, match="axis"):
            s.command("a", "jog_step", {"kind": "cartesian", "axis": "w", "sign": 1, "step": 1})
    finally:
        s.shutdown()


def test_web_servo_slider_mapping_is_linear(core):
    s = WebService(core)
    try:
        core.bridge.servo_available = True
        s.take_lease("a")
        r = s.command("a", "jog_start", {
            "kind": "joint", "values": [0.5, 0, 0, 0, 0, 0], "speed": 0.4,
        })
        assert r["ok"]
        time.sleep(0.04)
        calls = [c for c in core.bridge.calls if c[0] == "joint_jog" and abs(c[1][0]) > 1e-9]
        assert calls
        # 0.5 slider * 0.4 Velocity cap * configured 15 deg/s; no quadratic shaping.
        assert calls[-1][1][0] == pytest.approx(math.radians(15.0) * 0.5 * 0.4, rel=1e-3)
        s.command("a", "jog_stop", {})
    finally:
        s.shutdown()


def test_web_lease_loss_stops_a_hold(core):
    class Clock:
        t = 1000.0

        def __call__(self):
            return self.t
    clock = Clock()
    s = WebService(core, clock=clock)
    try:
        _admittance_with_servo(core)
        s.take_lease("a")
        s.command("a", "jog_start", {"kind": "joint", "axis": 0, "sign": 1, "speed": 0.2})
        clock.t += svc_mod.LEASE_TIMEOUT_S + 0.1
        s.tick()
        assert not core.jogging and s.lease.client is None
    finally:
        s.shutdown()


# ------------------------------------------------------------- hardware bridge
def test_entering_admittance_brings_back_a_stopped_ft_bridge(core):
    core.bridge.hardware["TopicSensorBridge"] = "unconfigured"       # ros2_control took it down (stale data)
    ok, msg = core.switch_mode("admittance")
    assert ok, msg
    calls = core.bridge.calls
    i_hw = calls.index(("activate_hardware", "TopicSensorBridge"))
    i_sw = next(i for i, c in enumerate(calls) if c[0] == "switch")
    assert i_hw < i_sw                                                 # before the controllers are switched
    assert core.bridge.hardware["TopicSensorBridge"] == "active"


def test_a_bridge_that_stops_again_blocks_admittance(core):
    core.bridge.hardware["TopicSensorBridge"] = "unconfigured"
    core.bridge.hardware_fails_again.add("TopicSensorBridge")
    ok, msg = core.switch_mode("admittance")
    assert not ok and "stopped again" in msg
    assert not any(c[0] == "switch" for c in core.bridge.calls)       # nothing was switched
    core.update()
    assert core.mode == "position"


def test_missing_hardware_component_is_refused(core):
    del core.bridge.hardware["TopicSensorBridge"]
    ok, msg = core.switch_mode("admittance")
    assert not ok and "not loaded" in msg


def test_position_does_not_touch_hardware(core):
    core.bridge.hardware["TopicSensorBridge"] = "unconfigured"
    core.switch_mode("admittance")
    core.bridge.calls.clear()
    core.switch_mode("position")
    assert not any(c[0] == "activate_hardware" for c in core.bridge.calls)


# --------------------------------------------------------------------- desktop
def test_desktop_jog_tab_steps_and_holds(win):
    op = win.pages["OPERATE"]
    pump(0.3)
    op.jog_type.setCurrentIndex(op.jog_type.findData("joint"))
    op.jog_step.setCurrentIndex(1)                                   # smallest step
    pump(0.2)
    q0 = list(win.last_snap.joints)
    plus_j1 = op.jog_buttons[1]
    assert plus_j1.isEnabled() and op.jog_labels[0].text().startswith("J1")
    plus_j1.pressed.emit()
    plus_j1.released.emit()
    pump(1.5)
    assert win.last_snap.joints[0] == pytest.approx(q0[0] + math.radians(0.5), abs=1e-4)

    op.jog_step.setCurrentIndex(0)                                   # hold
    pump(0.3)
    assert not plus_j1.isEnabled() and "Servo is not ready" in op.jog_note.text()
    win.bridge.servo_available = True
    pump(0.5)
    assert plus_j1.isEnabled()                                      # Position uses direct forward-position Servo
    plus_j1.setDown(True)
    plus_j1.pressed.emit()
    pump(0.6)
    assert win.core.jogging
    plus_j1.setDown(False)
    plus_j1.released.emit()
    pump(0.5)
    assert not win.core.jogging and not win.core.busy
    assert win.core.mode == "position"
    assert win.bridge.controllers["scaled_joint_trajectory_controller"] == "active"
    assert win.bridge.controllers["joint_trajectory_controller"] == "inactive"


def test_servo_warning_reaches_the_jog_view(core):
    _admittance_with_servo(core)
    s = WebService(core)
    try:
        s.take_lease("a")
        core.bridge.fake_servo_status = (4, "Close to a collision, decelerating")
        s.command("a", "jog_start", {"kind": "cartesian", "axis": "z", "sign": -1, "speed": 0.1})
        assert s.state()["jog"]["servo_warning"] == "Close to a collision, decelerating"
        s.command("a", "jog_stop", {})
        assert s.state()["jog"]["servo_warning"] == ""                    # only shown while jogging
    finally:
        s.shutdown()
