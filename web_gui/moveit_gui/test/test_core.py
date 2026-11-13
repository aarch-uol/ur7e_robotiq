import sys
import threading
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from moveit_gui.calibration import CalibrationStore  # noqa: E402
from moveit_gui.geometry import Geometry  # noqa: E402
from moveit_gui.control_modes import (ControlModeManager, ModeSwitchError, NONE, UNKNOWN,  # noqa: E402
                                      detect_mode, plan_switch)
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.policies import PolicyLibrary  # noqa: E402
from moveit_gui.profile import ProfileError, RobotProfile, reorder_joints  # noqa: E402
from moveit_gui.program import Program, ProgramRunner, RunState, Step  # noqa: E402

PROFILE = ROOT / "profiles" / "ur7e_lab.yaml"


@pytest.fixture
def profile():
    return RobotProfile.load(PROFILE)


@pytest.fixture(autouse=True)
def _isolated_config(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))


@pytest.fixture
def bridge(profile):
    return FakeBridge(profile, joint_speed=50.0, move_time_scale=10.0)


def _geometry(bridge, profile):
    return Geometry(CalibrationStore(profile), bridge.lookup_transform, bridge.read_pose_source)


def _allow_impedance(profile):
    profile.modes["impedance"].allow_activation = True


# ------------------------------------------------------------------ profile
def test_all_profiles_load():
    for p in (ROOT / "profiles").glob("*.yaml"):
        prof = RobotProfile.load(p)
        assert len(prof.joint_names) == (7 if prof.name == "fr3" else 6)
        assert prof.default_mode in prof.modes


def test_profile_rejects_bad_waypoint(profile):
    d = yaml.safe_load(PROFILE.read_text())
    d["waypoints"]["bad"] = {"joints": [0, 0, 0]}
    with pytest.raises(ProfileError):
        RobotProfile.from_dict(d)


def test_joint_prefix_applied():
    d = yaml.safe_load(PROFILE.read_text())
    d["joint_prefix"] = "ur_"
    prof = RobotProfile.from_dict(d)
    assert prof.joint_names[0] == "ur_shoulder_pan_joint"


# ----------------------------------------------------------- joint ordering
def test_reorder_joints_by_name(profile):
    # The UR driver's actual order, with the gripper joint mixed in.
    names = ["elbow_joint", "robotiq_85_left_knuckle_joint", "shoulder_lift_joint",
             "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    vals = [3.0, 0.5, 2.0, 1.0, 4.0, 5.0, 6.0]
    assert reorder_joints(names, vals, profile.joint_names) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_reorder_joints_missing_or_bad(profile):
    assert reorder_joints(["elbow_joint"], [1.0], profile.joint_names) is None
    names = list(profile.joint_names)
    assert reorder_joints(names, [0, 0, float("nan"), 0, 0, 0], profile.joint_names) is None
    assert reorder_joints(names, [0, 0], profile.joint_names) is None


# ------------------------------------------------------------- mode switch
def test_detect_mode(profile, bridge):
    assert detect_mode(profile, bridge.list_controllers()) == "position"
    st = bridge.list_controllers()
    st["admittance_controller"] = "active"
    assert detect_mode(profile, st) == UNKNOWN
    st = {k: "inactive" for k in st}
    assert detect_mode(profile, st) == NONE


def test_plan_switch_to_admittance(profile, bridge):
    plan = plan_switch(profile, bridge.list_controllers(), "admittance")
    assert set(plan.activate) == {"admittance_controller", "admittance_trajectory_controller"}
    assert plan.deactivate == ["scaled_joint_trajectory_controller"]
    # broadcasters are never touched
    assert "joint_state_broadcaster" not in plan.deactivate


def test_plan_switch_missing_controller(profile, bridge):
    st = bridge.list_controllers()
    del st["cartesian_impedance_controller"]
    with pytest.raises(ModeSwitchError, match="not loaded"):
        plan_switch(profile, st, "impedance")


def test_mode_manager_round_trip(profile, bridge):
    _allow_impedance(profile)
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    for target in ("admittance", "impedance", "position"):
        ok, msg = mm.switch_to(target)
        assert ok, msg
        assert mm.current_mode() == target
    # motion was cancelled before each switch
    assert sum(1 for c in bridge.calls if c[0] == "cancel") == 3


def test_mode_manager_reports_rejection(profile, bridge):
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    bridge.fail_next_switch = True
    ok, msg = mm.switch_to("admittance")
    assert not ok and "rejected" in msg
    assert mm.current_mode() == "position"


def test_mode_manager_refuses_while_moving(profile, bridge):
    mm = ControlModeManager(bridge, profile, log=lambda m: None, stationary_timeout_s=0.2)
    bridge.is_stationary = lambda: False
    ok, msg = mm.switch_to("admittance")
    assert not ok and "rest" in msg and msg.startswith("Cannot enter Admittance")
    assert mm.current_mode() == "position"


# ---------------------------------------------------------------- programs
def test_example_programs_valid(profile, bridge):
    lib = PolicyLibrary(ROOT / "policies")
    geo = _geometry(bridge, profile)
    # example_franka_scraping_baseline works in the vial fixture the operator teaches (SETUP > Vial Geometry)
    from moveit_gui.calibration import FixtureDef
    from moveit_gui import transforms as tf
    geo.store.put_fixture(FixtureDef("vial", profile.base_link, tf.mm_deg_to_T([400, 150, 200], [0, 0, 0])),
                          replace="vial" in geo.store.fixtures)
    # example_rbf_scrape_speed works in the flask fixture SETUP > Flask saves
    geo.store.put_fixture(FixtureDef("flask", profile.base_link, tf.mm_deg_to_T([500, 0, 100], [0, 0, 0])),
                          replace="flask" in geo.store.fixtures)
    for p in (ROOT / "programs").glob("*.yaml"):
        prog = Program.load(p, profile)
        assert prog.notes == [], p.name          # shipped examples are already v2
        assert prog.validate(profile, geo, lib, live=True, bridge=bridge) == [], p.name


def test_program_round_trip(tmp_path, profile):
    prog = Program(name="t", robot=profile.name,
                   steps=[Step("movej", {"target": "home"}), Step("wait", {"seconds": 0.1}),
                          Step("gripper", {"command": "close", "effort": 20})])
    f = tmp_path / "p.yaml"
    prog.save(f)
    again = Program.load(f, profile)
    assert [s.to_dict() for s in again.steps] == [s.to_dict() for s in prog.steps]


def test_program_validation_catches_errors(profile):
    prog = Program.from_dict({"version": 2, "steps": [
        {"type": "movej", "target": "nowhere"},
        {"type": "controller", "mode": "turbo"},
        {"type": "movej", "joints": [0, 0]},
        {"type": "speed", "fraction": 2.0},
    ]}, profile)
    problems = prog.validate(profile)
    assert len(problems) == 4


def _runner(bridge, profile, **kw):
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    geo = _geometry(bridge, profile)
    done = {}
    ev = threading.Event()
    steps = []

    def on_done(ok, msg):
        done["ok"], done["msg"] = ok, msg
        ev.set()
    r = ProgramRunner(bridge, profile, mm, geo, on_done=on_done, on_log=lambda m: None,
                      on_step=steps.append, **kw)
    return r, done, ev, steps


def test_runner_executes_in_order(profile, bridge):
    prog = Program.from_dict({"version": 2, "steps": [
        {"type": "movej", "target": "ready"},
        {"type": "gripper", "command": "close"},
        {"type": "controller", "mode": "admittance"},
        {"type": "movej", "target": "home"},
        {"type": "controller", "mode": "position"},
    ]}, profile)
    r, done, ev, steps = _runner(bridge, profile)
    assert r.start(prog)
    assert ev.wait(10)
    assert done["ok"], done["msg"]
    kinds = [c[0] for c in bridge.calls if c[0] in ("movej", "gripper", "switch")]
    assert kinds == ["movej", "gripper", "switch", "movej", "switch"]
    assert steps[:5] == [0, 1, 2, 3, 4] and steps[-1] == -1


def test_runner_refuses_motion_in_impedance(profile, bridge):
    _allow_impedance(profile)
    prog = Program.from_dict({"version": 2, "steps": [
        {"type": "controller", "mode": "impedance"},
        {"type": "movej", "target": "home"},
    ]}, profile)
    r, done, ev, _ = _runner(bridge, profile)
    r.start(prog)
    assert ev.wait(5)
    assert not done["ok"] and "step 2" in done["msg"]
    assert not any(c[0] == "movej" for c in bridge.calls)


def test_runner_stop_cancels_motion(profile):
    slow = FakeBridge(profile, joint_speed=0.2)
    prog = Program.from_dict({"version": 2, "steps": [{"type": "movej", "target": "ready", "velocity": 0.1}]}, profile)
    r, done, ev, _ = _runner(slow, profile)
    r.start(prog)
    time.sleep(0.2)
    r.stop()
    assert ev.wait(3)
    assert done == {"ok": False, "msg": "stopped"}
    assert r.state == RunState.IDLE


def test_runner_pause_repeats_motion_then_finishes(profile):
    b = FakeBridge(profile, joint_speed=2.0)
    prog = Program.from_dict({"version": 2, "steps": [{"type": "movej", "target": "ready", "velocity": 1.0}]}, profile)
    r, done, ev, _ = _runner(b, profile)
    r.start(prog)
    time.sleep(0.15)
    r.pause()
    time.sleep(0.2)
    assert r.state == RunState.PAUSED and not ev.is_set()
    r.resume()
    assert ev.wait(10)
    assert done["ok"], done["msg"]
    assert sum(1 for c in b.calls if c[0] == "movej") == 2   # cancelled once, re-issued
    assert b.snapshot().joints == pytest.approx(profile.waypoints["ready"].joints)


def test_runner_step_mode(profile, bridge):
    prog = Program.from_dict({"version": 2, "steps": [{"type": "wait", "seconds": 0.01}] * 3}, profile)
    r, done, ev, steps = _runner(bridge, profile)
    r.start(prog, step_mode=True)
    time.sleep(0.2)
    assert r.state == RunState.PAUSED and steps == [0]
    r.resume(step_mode=True)
    time.sleep(0.2)
    assert steps == [0, 1]
    r.resume()
    assert ev.wait(3) and done["ok"]


def test_runner_rejects_invalid_program(profile, bridge):
    r, done, ev, _ = _runner(bridge, profile)
    assert not r.start(Program.from_dict({"version": 2, "steps": [{"type": "movej", "target": "nowhere"}]}, profile))
    assert "preflight failed" in done["msg"]


# ---------------------------------------------------------------- policies
def _write_policy(d: Path, name: str, cmd, mode="impedance", duration=None):
    m = {"name": name, "required_mode": mode, "command": cmd}
    if duration:
        m["duration"] = duration
    (d / f"{name}.yaml").write_text(yaml.safe_dump(m))


def test_policy_requires_mode_and_runs(tmp_path, profile, bridge):
    _allow_impedance(profile)
    _write_policy(tmp_path, "ok", [sys.executable, "-c", "import time; time.sleep(30)"], duration=0.3)
    lib = PolicyLibrary(tmp_path)
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    logs = []
    assert not lib.run("ok", mm, log=logs.append)          # wrong mode: refused, no silent switch
    assert "needs control mode" in logs[-1]
    mm.switch_to("impedance")
    t0 = time.monotonic()
    assert lib.run("ok", mm, log=logs.append)              # killed at duration -> success
    assert time.monotonic() - t0 < 5
    assert not lib.running


def test_policy_stop_event(tmp_path, profile, bridge):
    _allow_impedance(profile)
    _write_policy(tmp_path, "long", [sys.executable, "-c", "import time; time.sleep(30)"])
    lib = PolicyLibrary(tmp_path)
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    mm.switch_to("impedance")
    stop = threading.Event()
    threading.Timer(0.3, stop.set).start()
    assert not lib.run("long", mm, stop_event=stop, log=lambda m: None)


def test_policy_rejects_shell_string(tmp_path):
    (tmp_path / "bad.yaml").write_text(yaml.safe_dump(
        {"name": "bad", "required_mode": "impedance", "command": "rm -rf /"}))
    lib = PolicyLibrary(tmp_path)
    assert "bad" not in lib.manifests and "bad.yaml" in lib.errors
