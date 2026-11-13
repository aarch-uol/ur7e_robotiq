"""Robot status the operator relies on: gripper position and Robotiq status, and refusing motion while the
robot cannot take it (pendant program stopped, protective stop)."""
import os
import time
from types import SimpleNamespace

import pytest

from moveit_gui.bridge_api import gripper_status_dict
from moveit_gui.web.service import CommandError, WebService

from test_jog import core  # noqa: F401  (fixture)
from test_ros_bridge_static import _bare_bridge, ros_bridge_module  # noqa: F401  (fixture)


def _status(obj=3, current=0.0, fault=0, severity=0):
    return SimpleNamespace(object_detection=obj, motor_current=current, gripper_fault=fault,
                           gripper_fault_severity=severity)


def _wait_result(s, jid, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        for r in s.results:
            if r["id"] == jid:
                return r
        time.sleep(0.02)
    raise AssertionError("no result")


# ------------------------------------------------------------------ decoding
def test_gripper_status_is_decoded_for_the_operator():
    held = gripper_status_dict(_status(obj=2, current=0.35))
    assert held["holding"] and held["object"] == "object held" and held["current_a"] == 0.35
    assert not held["needs_reactivate"]
    idle = gripper_status_dict(_status(obj=3))
    assert not idle["holding"] and idle["fault"] == "none"
    fault = gripper_status_dict(_status(fault=7, severity=3))
    assert fault["fault"] == "activation required" and fault["needs_reactivate"]
    odd = gripper_status_dict(_status(fault=4, severity=255))     # a code the manual does not list
    assert odd["fault"].startswith("undefined") and odd["needs_reactivate"]


# ------------------------------------------------------------ ROS bridge
def _joint_msg(names, pos):
    return SimpleNamespace(name=names, position=pos, velocity=[0.0] * len(pos), effort=[0.0] * len(pos))


def test_a_gripper_only_joint_state_still_updates_the_gripper(ros_bridge_module):
    b = _bare_bridge(ros_bridge_module)
    b._on_joints(_joint_msg(["robotiq_85_left_knuckle_joint"], [0.42]))
    assert b._snap.gripper_position == pytest.approx(0.42)
    assert b._snap.joints is None                        # the arm is untouched by the gripper message
    arm = b.profile.joint_names
    b._on_joints(_joint_msg(list(arm), [0.1] * 6))
    assert b._snap.joints == [0.1] * 6 and b._snap.gripper_position == pytest.approx(0.42)


def test_the_ur_status_topics_are_subscribed_latched():
    src = open(os.path.join(os.path.dirname(__file__), "..", "moveit_gui", "ros_bridge.py")).read()
    for cb in ("self._on_prog", "self._on_robot_mode", "self._on_safety_mode", "self._on_gripper_status"):
        line = next(ln for ln in src.splitlines() if cb + "," in ln)
        assert "LATCHED" in line, f"{cb} must use the latched QoS (the driver publishes on change only)"


# ---------------------------------------------------------- motion refusal
def test_motion_is_refused_while_the_pendant_program_is_stopped(core):
    assert core.motion_refusal() is None
    core.bridge.program_running = False
    assert "External Control program is not running" in core.motion_refusal()
    ok, why = core.move_joints(core.bridge.snapshot().joints, 0.1, 0.1)
    assert not ok and "Play program" in why
    # robot_program_running is published only on change: never heard while the robot is RUNNING = never ran
    core.bridge.program_running = None
    core.bridge.robot_mode = "RUNNING"
    assert "Play program" in core.motion_refusal()
    assert "Play program" in core.jog_hold_refusal()            # hold-to-jog checks the robot too
    core.bridge.robot_mode = "?"                               # no robot status at all: unknown, not refused
    assert core.motion_refusal() is None
    core.bridge.robot_mode, core.bridge.program_running = "RUNNING", True
    assert core.motion_refusal() is None


def test_motion_is_refused_in_a_protective_stop(core):
    core.bridge.safety_mode = "PROTECTIVE_STOP"
    assert "PROTECTIVE_STOP" in core.motion_refusal()
    core.bridge.safety_mode = "NORMAL"
    core.bridge.robot_mode = "IDLE"
    assert "robot mode is IDLE" in core.motion_refusal()


# ---------------------------------------------------------------- web gripper
def test_web_gripper_by_percent_and_force(core):
    s = WebService(core)
    try:
        s.take_lease("a")
        g = core.profile.gripper
        r = s.command("a", "gripper", {"percent": 50, "effort": 20})
        res = _wait_result(s, r["job"])
        assert res["ok"] and "at target" in res["msg"]
        assert core.bridge.gripper_pos == pytest.approx((g["open_position"] + g["closed_position"]) / 2)
        with pytest.raises(CommandError, match="force"):
            s.command("a", "gripper", {"percent": 100, "effort": g["effort_limit"] + 1})
        with pytest.raises(CommandError, match="position or a percent"):
            s.command("a", "gripper", {})
        core.update()
        st = s.state()
        assert st["gripper_status"]["object"] == "at target, no object"
        assert st["gripper_controller"] == "active"
        assert s.info()["gripper"]["can_reactivate"]
    finally:
        s.shutdown()


def test_web_gripper_reactivate_clears_a_fault(core):
    s = WebService(core)
    try:
        s.take_lease("a")
        core.bridge.gripper_fault = 7
        core.update()
        assert s.state()["gripper_status"]["needs_reactivate"]
        r = s.command("a", "gripper_reactivate", {})
        assert _wait_result(s, r["job"])["ok"]
        core.update()
        assert s.state()["gripper_status"]["fault"] == "none"
    finally:
        s.shutdown()


def test_a_program_playing_without_external_control_for_this_pc_is_refused(core, monkeypatch):
    """The pendant plays a program, but its External Control host is another PC: the arm would not move."""
    import dataclasses
    snap = core.bridge.snapshot
    monkeypatch.setattr(core.bridge, "snapshot", lambda: dataclasses.replace(
        snap(), program_running=False, program_state="PLAYING"))
    assert "Host IP" in core.motion_refusal()
