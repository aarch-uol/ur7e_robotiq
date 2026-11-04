"""v2.3: transactional switching, the policy runtime path, vial geometry, payload workflow, the real UR stack."""
import os
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.control_modes import (CONTROL_STATE_UNKNOWN, ControlModeManager, detect_mode,  # noqa: E402
                                      moveit_controller_for)
from moveit_gui.fake_bridge import FAKE_PAYLOAD_COG, FAKE_PAYLOAD_MASS, FakeBridge  # noqa: E402
from moveit_gui.ft_payload import PayloadCalibration, PayloadCalibrationError, load_calibration_math  # noqa: E402
from moveit_gui.geometry_calibration import VialGeometryError, circle_from_three_points, estimate_vial  # noqa: E402
from moveit_gui.policies import PolicyLibrary, PolicyManifest, RecordingOptions  # noqa: E402
from moveit_gui.policy_action import MockPolicyRunner, RunPolicyGoal, SubprocessPolicyRunner  # noqa: E402
from moveit_gui.policy_executor import FAIL, RUNNER, PolicyExecutor  # noqa: E402
from moveit_gui.profile import ProfileError, RobotProfile  # noqa: E402
from moveit_gui.program import Program, ProgramRunner  # noqa: E402

UR_REPO = Path.home() / "ur7e_robotiq"
FRANKA = Path.home() / "franka_ros2"
RUNNER_DIR = FRANKA / "mujoco_scraping/src/mujoco_multiagent_rl/policy_runner/ros2"
needs_ur = pytest.mark.skipif(not UR_REPO.is_dir(), reason="~/ur7e_robotiq not available")
needs_runner = pytest.mark.skipif(not RUNNER_DIR.is_dir(), reason="~/franka_ros2 not available")


@pytest.fixture
def profile():
    return RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml")


@pytest.fixture(autouse=True)
def _isolated_config(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))


@pytest.fixture
def bridge(profile):
    return FakeBridge(profile, move_time_scale=50.0)


def _mm(bridge, profile, **kw):
    return ControlModeManager(bridge, profile, log=lambda m: None, stationary_timeout_s=0.3, **kw)


def _mode(bridge, profile):
    return detect_mode(profile, bridge.controllers)


# ================================================================= control
def test_position_to_admittance_and_back(profile, bridge):
    mm = _mm(bridge, profile)
    ok, msg = mm.switch_to("admittance")
    assert ok and "POSITION interface verified" in msg and _mode(bridge, profile) == "admittance"
    ok, _ = mm.switch_to("position")
    assert ok and _mode(bridge, profile) == "position"


def test_failed_interface_verification_rolls_back(profile, bridge):
    mm = _mm(bridge, profile)
    real = bridge.controller_claims

    def claims():                        # wrong claims while the target is active, correct after rollback
        return {"admittance_controller": []} if _mode(bridge, profile) == "admittance" else real()
    bridge.controller_claims = claims
    ok, msg = mm.switch_to("admittance")
    assert not ok and "interface claims are wrong" in msg and "Rolled back to Position" in msg
    assert _mode(bridge, profile) == "position" and not mm.control_state_unknown


def test_unreadable_claims_roll_back_position_domain_too(profile, bridge):
    mm = _mm(bridge, profile)
    bridge.fake_claims_available = False
    ok, msg = mm.switch_to("admittance")
    assert not ok and "could not be verified" in msg
    assert mm.control_state_unknown           # the previous mode cannot be verified either
    assert _mode(bridge, profile) == "position"


def test_rollback_failure_enters_unknown_and_disables_motion(profile, bridge):
    mm = _mm(bridge, profile)
    bridge.fake_action_ready = False           # post-switch check fails
    bridge.fail_rollback = True                # and the way back is rejected
    ok, msg = mm.switch_to("admittance")
    assert not ok and "Rollback failed" in msg and "UNKNOWN" in msg
    assert mm.control_state_unknown and mm.current_mode() == CONTROL_STATE_UNKNOWN
    assert mm.mode_supports_moveit() is False
    ok, msg = mm.switch_to("position")
    assert not ok and "recover manually" in msg


def test_program_motion_refused_in_unknown(profile, bridge, tmp_path):
    mm = _mm(bridge, profile)
    mm.unknown_reason = "test"
    from moveit_gui.calibration import CalibrationStore
    from moveit_gui.geometry import Geometry
    geo = Geometry(CalibrationStore(profile), bridge.lookup_transform, bridge.read_pose_source)
    runner = ProgramRunner(bridge, profile, mm, geo, None, on_log=lambda m: None)
    prog = Program.from_dict({"version": 2, "name": "p", "robot": profile.name,
                              "steps": [{"type": "movej", "target": "home"}]}, profile)
    assert runner._require_moveit_mode() is False
    assert not any(c[0] == "movej" for c in bridge.calls)
    assert prog.steps


def test_recovery_clears_unknown_only_when_verified(profile, bridge):
    mm = _mm(bridge, profile)
    mm.unknown_reason = "test"
    bridge.fake_action_ready = False
    ok, _ = mm.verify_recovery()
    assert not ok and mm.control_state_unknown
    bridge.fake_action_ready = True
    ok, _ = mm.verify_recovery()
    assert ok and not mm.control_state_unknown and mm.current_mode() == "position"


def test_direct_torque_requires_reference_check_in_profile():
    raw = yaml.safe_load((ROOT / "profiles" / "ur7e_lab.yaml").read_text())
    del raw["control_modes"]["modes"]["impedance"]["reference_check"]
    with pytest.raises(ProfileError, match="reference_check"):
        RobotProfile.from_dict(raw)


def test_real_profile_allows_impedance_with_strict_preflight(profile):
    m = profile.modes["impedance"]
    assert m.allow_activation is True and m.experimental and m.direct_torque and not m.supports_moveit
    assert profile.modes["impedance"].reference_check["state_topic"].endswith("/controller_state")


def test_direct_torque_verifies_reference(profile, bridge):
    profile.modes["impedance"].allow_activation = True
    ok, _ = _mm(bridge, profile).switch_to("impedance")
    assert ok and ("reference_check", profile.modes["impedance"].reference_check["state_topic"]) in bridge.calls


def test_direct_torque_reference_failure_rolls_back(profile, bridge):
    profile.modes["impedance"].allow_activation = True
    bridge.fake_reference_result = (False, "reference is 90.0 mm from the measured pose")
    ok, msg = _mm(bridge, profile).switch_to("impedance")
    assert not ok and "reference initialisation failed" in msg and "Rolled back to Position" in msg
    assert _mode(bridge, profile) == "position"
    assert bridge.controllers["cartesian_impedance_controller"] == "inactive"


def test_rejected_strict_switch_needs_no_rollback(profile, bridge):
    bridge.fail_next_switch = True
    ok, msg = _mm(bridge, profile).switch_to("admittance")
    assert not ok and "rejected the STRICT switch" in msg and "still in Position" in msg
    assert len([c for c in bridge.calls if c[0] == "switch"]) == 1


def test_moveit_routing_follows_active_mode(profile, bridge):
    assert moveit_controller_for(profile, bridge.controllers) == "scaled_joint_trajectory_controller"
    assert bridge.move_joints(list(bridge.snapshot().joints), 1.0)
    assert bridge.last_route == "scaled_joint_trajectory_controller"
    assert _mm(bridge, profile).switch_to("admittance")[0]
    assert bridge.move_joints(list(bridge.snapshot().joints), 1.0)
    assert bridge.last_route == "admittance_trajectory_controller"


def test_moveit_routing_refuses_without_trajectory_mode(profile, bridge):
    profile.modes["impedance"].allow_activation = True
    assert _mm(bridge, profile).switch_to("impedance")[0]
    assert moveit_controller_for(profile, bridge.controllers) is None
    assert bridge.move_joints(list(bridge.snapshot().joints), 1.0) is False


# ================================================================== policy
def _deployment(tmp_path, ready=True, ee="spatula_tip"):
    cfg = tmp_path / "ur.yaml"
    cfg.write_text(yaml.safe_dump({"servo_admittance": {"base_frame": "base_link", "ee_frame": ee}}))
    (tmp_path / "ur.commissioning.yaml").write_text(yaml.safe_dump({"ready": ready, "unavailable": ["mass"]}))
    return cfg


def _manifest(tmp_path, backend="servo_admittance", mode="admittance", **extra):
    for n in ("model.zip", "vec.pkl", "config.yaml"):
        (tmp_path / n).write_text("x")
    d = {"name": "p", "backend": backend, "required_mode": mode, "required_tcp": "spatula_tip",
         "requires_ft": True, "model": str(tmp_path / "model.zip"), "normalization": str(tmp_path / "vec.pkl"),
         "training_config": str(tmp_path / "config.yaml"), "robot_config": str(_deployment(tmp_path)),
         "duration": 0.2, "control_hz": 20.0}
    d.update(extra)
    (tmp_path / "p.yaml").write_text(yaml.safe_dump(d))


def _executor(tmp_path, profile, bridge, backend=None, tcp="spatula_tip"):
    bridge.servo_available = True
    bridge.extra_frames["spatula_tip"] = tf.make([0, 0, 0.2])
    mm = _mm(bridge, profile)
    assert mm.switch_to("admittance")[0]
    lib = PolicyLibrary(tmp_path)
    return PolicyExecutor(lib, mm, bridge, backend or MockPolicyRunner(baseline_total=3, baseline_hz=200),
                          active_tcp=lambda: tcp)


def test_generated_command_is_the_runner_cli(tmp_path):
    _manifest(tmp_path)
    m = PolicyLibrary(tmp_path).manifests["p"]
    assert m.build_command() == [
        "ros2", "run", "my_robot_policy", "run_policy.py",
        "--model", str(tmp_path / "model.zip"), "--normalization", str(tmp_path / "vec.pkl"),
        "--config", str(tmp_path / "config.yaml"), "--robot-config", str(tmp_path / "ur.yaml"),
        "--robot-controller", "servo_admittance"]


@needs_runner
@pytest.mark.parametrize("entry,source", [("run_policy", "node.py"), ("run_scraping_policy", "scraping_policy.py")])
def test_every_generated_flag_exists_in_the_real_runner(tmp_path, entry, source):
    _manifest(tmp_path, entrypoint=entry)
    m = PolicyLibrary(tmp_path).manifests["p"]
    real = set(re.findall(r"""['"](--[a-z-]+)['"]""", (RUNNER_DIR / source).read_text()))
    cmd = m.build_command(RecordingOptions(record_obs=False, record_video=True, video_fps=30, output_root="/tmp/o"),
                          duration=8.0)
    used = {c for c in cmd if c.startswith("--")}
    assert used and used <= real, used - real
    assert entry + ".py" in (FRANKA / "my_robot_policy/CMakeLists.txt").read_text()


@needs_runner
def test_valid_backends_match_the_runner():
    from moveit_gui.policies import BACKEND_MODE
    text = (RUNNER_DIR / "node.py").read_text()
    assert set(BACKEND_MODE) == set(re.search(r"VALID_ROBOT_CONTROLLERS = frozenset\(\{(.+?)\}\)", text).group(1)
                                    .replace('"', "").replace(" ", "").split(","))


def test_fake_ros_parameters_are_rejected(tmp_path):
    (tmp_path / "bad.yaml").write_text(yaml.safe_dump({
        "name": "bad", "required_mode": "impedance", "backend": "impedance",
        "command": ["ros2", "run", "my_robot_policy", "run_policy", "--ros-args", "-p", "model_path:=x"]}))
    problems = PolicyLibrary(tmp_path).manifests["bad"].contract_problems()
    assert any("model_path:=" in p for p in problems)
    assert any("--robot-controller impedance" in p for p in problems)


def test_manifest_without_model_is_refused(tmp_path):
    (tmp_path / "x.yaml").write_text(yaml.safe_dump({"name": "x", "required_mode": "admittance"}))
    assert "x.yaml" in PolicyLibrary(tmp_path).errors


def test_backend_must_match_required_mode(tmp_path):
    _manifest(tmp_path, backend="impedance", mode="admittance")
    assert any("executes through impedance" in p for p in PolicyLibrary(tmp_path).manifests["p"].contract_problems())


def test_required_tcp_mismatch_fails(tmp_path, profile, bridge):
    _manifest(tmp_path)
    ex = _executor(tmp_path, profile, bridge, tcp="robotiq_tcp")
    assert any("active TCP is robotiq_tcp" in p for p in ex.preflight("p"))


def test_servo_unavailable_is_a_hard_failure(tmp_path, profile, bridge):
    _manifest(tmp_path)
    ex = _executor(tmp_path, profile, bridge)
    assert ex.preflight("p") == []
    bridge.servo_available = False
    assert any("MoveIt Servo" in p for p in ex.preflight("p"))
    assert ex.start(ex.goal("p")) is False
    assert "MoveIt Servo" in {r.label: r for r in ex.requirements("p")} and \
        {r.label: r for r in ex.requirements("p")}["MoveIt Servo"].state == FAIL


def test_impedance_backend_does_not_need_servo(tmp_path, profile, bridge):
    _manifest(tmp_path, backend="impedance", mode="impedance")
    ex = _executor(tmp_path, profile, bridge)
    bridge.servo_available = False
    assert not any("Servo" in p for p in ex.preflight("p"))


def test_observation_contract_is_left_to_the_runner(tmp_path, profile, bridge):
    _manifest(tmp_path)
    ex = _executor(tmp_path, profile, bridge)
    assert {r.label: r.state for r in ex.requirements("p")}["Observation contract"] == RUNNER


def test_uncommissioned_or_fr3_deployment_is_refused(tmp_path, profile, bridge):
    _manifest(tmp_path)
    ex = _executor(tmp_path, profile, bridge)
    (tmp_path / "ur.commissioning.yaml").write_text("ready: false\nunavailable: [mass]\n")
    assert any("not commissioned: mass" in p for p in ex.preflight("p"))
    (tmp_path / "ur.yaml").write_text(yaml.safe_dump({"servo_admittance": {"base_frame": "fr3_link0",
                                                                           "ee_frame": "fr3_hand_tcp"}}))
    assert any("FR3" in p for p in ex.preflight("p"))


def test_runner_frame_must_be_required_tcp_and_in_tf(tmp_path, profile, bridge):
    _manifest(tmp_path)
    ex = _executor(tmp_path, profile, bridge)
    bridge.extra_frames.clear()
    assert any("not available to the runner" in p for p in ex.preflight("p"))
    _deployment(tmp_path, ee="robotiq_tcp")
    assert any("tracks 'robotiq_tcp'" in p for p in ex.preflight("p"))


def test_recording_args_map_to_runner_arguments():
    assert RecordingOptions().to_args() == []
    assert RecordingOptions(record_obs=True, record_video=False).to_args() == ["--record-obs", "--no-record-video"]
    assert RecordingOptions(record_video=True, video_fps=15, output_root="/data/r").to_args() == [
        "--record-video", "--video-fps", "15", "--output-root", "/data/r"]


def test_recording_reaches_the_subprocess_command(tmp_path, profile, bridge):
    _manifest(tmp_path)
    m = PolicyLibrary(tmp_path).manifests["p"]
    cmd = m.build_command(RecordingOptions(record_obs=False))
    assert cmd[-1] == "--no-record-obs"


def test_scraping_entrypoint_passes_duration(tmp_path):
    _manifest(tmp_path, entrypoint="run_scraping_policy")
    cmd = PolicyLibrary(tmp_path).manifests["p"].build_command(duration=6.0)
    assert cmd[cmd.index("--policy-duration") + 1] == "6"


def test_program_step_and_automation_share_the_executor(tmp_path, profile, bridge):
    _manifest(tmp_path)
    ex = _executor(tmp_path, profile, bridge)
    seen = []
    ex.feedback_listeners.append(lambda fb: seen.append(fb.state))
    from moveit_gui.calibration import CalibrationStore
    from moveit_gui.geometry import Geometry
    geo = Geometry(CalibrationStore(profile), bridge.lookup_transform, bridge.read_pose_source)
    runner = ProgramRunner(bridge, profile, ex.modes, geo, ex, on_log=lambda m: None)
    prog = Program.from_dict({"version": 2, "name": "x", "robot": profile.name,
                              "steps": [{"type": "policy", "policy": "p", "duration": 0.1}]}, profile)
    assert runner.start(prog)
    runner.join(5.0)
    assert "running" in seen                  # feedback arrives on the shared listeners


def test_gui_wires_one_executor(tmp_path, profile, bridge):
    pytest.importorskip("PyQt5")
    from PyQt5 import QtWidgets
    from moveit_gui.gui.main_window import MainWindow
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = MainWindow(bridge, profile, ROOT / "profiles", tmp_path, tmp_path, fake=True)
    try:
        assert win.runner.policies is win.policies
        assert isinstance(win.policies, PolicyExecutor)
        assert win.pages["AUTOMATION"].win.policies is win.policies
    finally:
        win.close()
    assert app


def test_cancel_semantics_are_shared(tmp_path, profile, bridge):
    _manifest(tmp_path, duration=30.0)
    ex = _executor(tmp_path, profile, bridge)
    done = []
    assert ex.start(ex.goal("p"), done.append)
    time.sleep(0.05)
    ex.cancel()
    deadline = time.monotonic() + 2.0
    while not done and time.monotonic() < deadline:
        time.sleep(0.01)
    assert done and not done[0].success and "stopped" in done[0].stop_reason
    stop = threading.Event()
    result = []
    t = threading.Thread(target=lambda: result.append(ex.run("p", stop_event=stop, log=lambda m: None)))
    t.start()
    time.sleep(0.05)
    stop.set()
    t.join(2.0)
    assert result == [False]


def test_subprocess_backend_runs_generated_goal(tmp_path, profile, bridge):
    (tmp_path / "s.yaml").write_text(yaml.safe_dump({
        "name": "s", "required_mode": "admittance", "command": [sys.executable, "-c", "import time; time.sleep(30)"],
        "duration": 0.1}))
    bridge.servo_available = True
    mm = _mm(bridge, profile)
    assert mm.switch_to("admittance")[0]
    lib = PolicyLibrary(tmp_path)
    ex = PolicyExecutor(lib, mm, bridge, SubprocessPolicyRunner(lib, mm), active_tcp=lambda: "tool0")
    assert ex.supports_feedback is False
    assert ex.run("s", log=lambda m: None) is True


def test_policy_goal_recording_defaults_keep_runner_config():
    assert RunPolicyGoal("p").recording.to_args() == []


def test_no_vision_dependency_in_manifest(tmp_path):
    _manifest(tmp_path)
    assert PolicyLibrary(tmp_path).manifests["p"].required_frames == []


# ==================================================================== vial
def _rim(cx, cy, r, z=0.05, angles=(0.1, 2.0, 4.0)):
    return [[cx + r * np.cos(a), cy + r * np.sin(a), z] for a in angles]


def test_three_points_recover_known_circle():
    g = estimate_vial(_rim(0.512, -0.083, 0.017))
    assert g.center_xy == pytest.approx([0.512, -0.083], abs=1e-9) and g.radius == pytest.approx(0.017)
    assert g.bottom_source == "min_rim_z" and g.model == "xy_circle_base_aligned"


@needs_runner
def test_matches_legacy_vial_estimation_formula():
    pts = _rim(0.3, 0.2, 0.02)
    A = np.array([[p[0], p[1], 1] for p in pts])
    sq = [-(p[0] ** 2 + p[1] ** 2) for p in pts]
    Dx = np.array([[sq[i], pts[i][1], 1] for i in range(3)])
    Dy = np.array([[pts[i][0], sq[i], 1] for i in range(3)])
    legacy = (-np.linalg.det(Dx) / (2 * np.linalg.det(A)), -np.linalg.det(Dy) / (2 * np.linalg.det(A)))
    center, _ = circle_from_three_points(*pts)
    assert center == pytest.approx(legacy)


def test_collinear_points_rejected():
    with pytest.raises(VialGeometryError, match="collinear"):
        estimate_vial([[0.1, 0.1, 0.05], [0.12, 0.12, 0.05], [0.14, 0.14, 0.05]])


def test_duplicate_points_rejected():
    with pytest.raises(VialGeometryError, match="too close"):
        estimate_vial([[0.1, 0.1, 0.05], [0.1, 0.1, 0.05], [0.14, 0.1, 0.05]])


def test_units_validated():
    with pytest.raises(VialGeometryError, match="not mm"):
        estimate_vial(_rim(512.0, -83.0, 17.0))
    with pytest.raises(VialGeometryError, match="implausible"):
        estimate_vial(_rim(0.0, 0.0, 1.5))


def test_tilted_rim_is_outside_the_model():
    pts = _rim(0.3, 0.2, 0.02)
    pts[1][2] += 0.02
    with pytest.raises(VialGeometryError, match="base Z"):
        estimate_vial(pts)


def test_touched_bottom_preferred_and_checked():
    g = estimate_vial(_rim(0.3, 0.2, 0.02), bottom_point=[0.3, 0.2, 0.01])
    assert g.bottom_z == pytest.approx(0.01) and g.bottom_source == "touched"
    with pytest.raises(VialGeometryError, match="above the rim"):
        estimate_vial(_rim(0.3, 0.2, 0.02), bottom_point=[0.3, 0.2, 0.09])


# ================================================================= payload
ft_available = (FRANKA / "ft_sensor_calibration/ft_sensor_calibration/calibration_math.py").exists()
needs_ft = pytest.mark.skipif(not ft_available, reason="~/franka_ros2 calibration_math not available")


def _calibration(bridge, profile):
    return PayloadCalibration(bridge, profile, sleep=lambda _s: None)


def test_ur_poses_have_six_joints_not_the_fr3_seven(profile):
    assert all(len(p["joints"]) == 6 for p in profile.ft_calibration["poses"])
    raw = dict(profile.ft_calibration)
    raw["poses"] = [{"name": "fr3", "joints": [0.0] * 7}]
    profile.ft_calibration = raw
    with pytest.raises(PayloadCalibrationError, match="6 joints"):
        PayloadCalibration(FakeBridge(profile), profile)


def _ur7e_tool_rotation(q):
    """Rotation of tool0 from the published UR e-Series DH alphas (rotation only depends on these)."""
    R = np.eye(3)
    for th, al in zip(q, (np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0)):
        c, s, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        R = R @ np.array([[c, -s * ca, s * sa], [s, c * ca, -c * sa], [0, sa, ca]])
    return R


@needs_ft
def test_ur_poses_pass_observability_on_real_ur_kinematics(profile):
    cm = load_calibration_math()
    quats = [tf.matrix_to_quat(_ur7e_tool_rotation(p["joints"])) for p in profile.ft_calibration["poses"]]
    obs = cm.compute_calibration_observability(quats)
    assert obs["rank"] == 10 and obs["condition_number"] < 100.0, obs


@needs_ft
def test_workflow_uses_real_math_and_recovers_fake_payload(profile, bridge, tmp_path):
    cal = _calibration(bridge, profile)
    with pytest.raises(PayloadCalibrationError, match="at least 3 poses"):
        cal.evaluate()
    for i in range(len(cal.poses)):
        cal.run_pose(i)
    res = cal.evaluate()
    assert res.mass == pytest.approx(FAKE_PAYLOAD_MASS, abs=0.02)
    assert res.cog == pytest.approx(list(FAKE_PAYLOAD_COG), abs=0.01)
    assert "observability_rank" in res.quality["metrics"]
    assert load_calibration_math().__file__.endswith("ft_sensor_calibration/calibration_math.py")


@needs_ft
def test_saving_never_touches_controller_yaml(profile, bridge, tmp_path):
    cal = _calibration(bridge, profile)
    for i in range(len(cal.poses)):
        cal.run_pose(i)
    cal.evaluate()
    path = cal.save(tmp_path / "out")
    data = __import__("json").loads(path.read_text())
    assert data["applied_to_controller"] is False and data["robot"] == "ur7e_lab"
    assert [p.name for p in (tmp_path / "out").iterdir()] == [path.name]


@needs_ft
def test_apply_sets_admittance_gravity_compensation_and_reads_it_back(profile, bridge, tmp_path):
    from moveit_gui.ft_payload import G
    cal = _calibration(bridge, profile)
    for i in range(len(cal.poses)):
        cal.run_pose(i)
    res = cal.evaluate()
    assert res.passed
    p = cal.admittance_params()
    # Jazzy admittance_controller: CoG.force is the weight mass*g in N, CoG.pos is in frame.id = the sensor frame
    assert p["gravity_compensation.frame.id"] == "ft_fts_link"
    assert p["gravity_compensation.CoG.force"] == pytest.approx(res.mass * G, abs=1e-3)
    assert p["gravity_compensation.CoG.pos"] == pytest.approx(res.cog, abs=1e-6)
    assert "force:" in cal.admittance_yaml() and "ft_fts_link" in cal.admittance_yaml()

    ok, msg = cal.apply_to_admittance("admittance")
    assert not ok and "switch out of admittance" in msg and not any(c[0] == "set_parameters" for c in bridge.calls)
    path = cal.save(tmp_path / "out")
    ok, msg = cal.apply_to_admittance("position")
    assert ok, msg
    assert {k: bridge.node_params["/admittance_controller"][k] for k in p} == p
    assert __import__("json").loads(path.read_text())["applied_to_controller"]["node"] == "/admittance_controller"


@needs_ft
def test_apply_fails_when_the_controller_does_not_take_the_values(profile, bridge):
    cal = _calibration(bridge, profile)
    for i in range(len(cal.poses)):
        cal.run_pose(i)
    cal.evaluate()
    bridge.get_node_parameters = lambda node, names: {n: None for n in names}
    ok, msg = cal.apply_to_admittance("position")
    assert not ok and "did not take" in msg
    bridge.set_node_parameters = lambda node, params: (False, "/admittance_controller/set_parameters not available")
    ok, msg = cal.apply_to_admittance("position")
    assert not ok and "not available" in msg


def test_workflow_fails_on_motion_ft_or_tf_loss(profile, bridge):
    cal = _calibration(bridge, profile)
    bridge.ft_live = False
    with pytest.raises(PayloadCalibrationError, match="ATI stopped"):
        cal.run_pose(0)
    bridge.ft_live = True
    bridge.stale_frames.add("ft_fts_link")
    with pytest.raises(PayloadCalibrationError, match="no transform"):
        cal.run_pose(0)


def test_workflow_requires_verified_standstill(profile, bridge):
    cal = _calibration(bridge, profile)
    bridge.fake_velocities_available = False
    with pytest.raises(PayloadCalibrationError, match="stationary"):
        cal.run_pose(0)


# ========================================== real UR stack, read from its files
@needs_ur
def test_profile_matches_real_ur_controllers(profile):
    cfg = yaml.safe_load((UR_REPO / "ur7e_robotiq_bringup/config/ur_controllers_gui.yaml").read_text())
    loaded = cfg["controller_manager"]["ros__parameters"]
    for mode in ("position", "servo_position", "admittance", "impedance", "freedrive"):
        for c in profile.modes[mode].activate:
            assert c in loaded, c
    assert loaded["cartesian_impedance_controller"]["type"].startswith("cartesian_impedance_controller/")
    chain = cfg["admittance_trajectory_controller"]["ros__parameters"]["command_joints"]
    assert all(j.startswith("admittance_controller/") for j in chain)
    assert cfg["admittance_controller"]["ros__parameters"]["ft_sensor"]["name"] == "ati_fts_sensor"


@needs_ur
def test_gui_controller_file_tracks_the_commissioning_files():
    d = UR_REPO / "ur7e_robotiq_bringup/config"
    gui = yaml.safe_load((d / "ur_controllers_gui.yaml").read_text())
    adm = yaml.safe_load((d / "ur_controllers_admittance.yaml").read_text())
    imp = yaml.safe_load((d / "ur_controllers_cartesian_impedance.yaml").read_text())
    for c in ("admittance_controller", "admittance_trajectory_controller", "scaled_joint_trajectory_controller"):
        assert gui[c] == adm[c], c
    assert gui["cartesian_impedance_controller"] == imp["cartesian_impedance_controller"]


@needs_ur
def test_moveit_knows_both_arm_controllers_with_one_default():
    cfg = yaml.safe_load((UR_REPO / "ur7e_robotiq_moveit_config/config/moveit_controllers_gui.yaml").read_text())
    sm = cfg["moveit_simple_controller_manager"]
    assert sm["scaled_joint_trajectory_controller"]["default"] is True
    assert sm["admittance_trajectory_controller"]["default"] is False
    assert sm["scaled_joint_trajectory_controller"]["joints"] == sm["admittance_trajectory_controller"]["joints"]


@needs_ur
def test_gripper_values_match_the_repo(profile):
    g = profile.gripper
    script = (UR_REPO / "ur7e_robotiq_bringup/scripts/gripper.sh").read_text()
    assert f"pos={g['closed_position']}" in script and g["action"] in script and g["joint_name"] in script
    srdf = (UR_REPO / "ur7e_robotiq_moveit_config/config/ur7e_robotiq.srdf.xacro").read_text()
    assert re.search(rf'group_state name="closed".*?value="{g["closed_position"]}"', srdf, re.S)


@needs_ur
def test_servo_output_goes_to_the_admittance_chain():
    cfg = yaml.safe_load((UR_REPO / "ur7e_robotiq_moveit_config/config/ur7e_servo.yaml").read_text())
    assert cfg["command_out_topic"] == "/admittance_trajectory_controller/joint_trajectory"
    assert cfg["command_out_type"] == "trajectory_msgs/JointTrajectory" and cfg["move_group_name"] == "ur_manipulator"


@needs_ur
def test_position_servo_streams_trajectories_to_its_own_jtc(profile):
    # MoveIt 2.12 Servo starts each Float64MultiArray step from the measured state: on the real (lagging) UR the
    # arm barely moved. Trajectory output continues from Servo's own commands.
    cfg = yaml.safe_load((UR_REPO / "ur7e_robotiq_moveit_config/config/ur7e_servo_position.yaml").read_text())
    assert cfg["command_out_topic"] == "/joint_trajectory_controller/joint_trajectory"
    assert cfg["command_out_type"] == "trajectory_msgs/JointTrajectory"
    assert cfg["joint_topic"] == "/joint_states"
    ctl = yaml.safe_load((UR_REPO / "ur7e_robotiq_bringup/config/ur_controllers_gui.yaml").read_text())
    jtc = ctl["joint_trajectory_controller"]["ros__parameters"]
    assert jtc["allow_nonzero_velocity_at_trajectory_end"] is True     # Servo's last point is still moving
    assert profile.modes["servo_position"].activate == ["joint_trajectory_controller"]
    launch = (UR_REPO / "ur7e_robotiq_moveit_config/launch/servo.launch.py").read_text()
    assert 'name="servo_node"' in launch and 'name="position_servo_node"' in launch


@needs_ur
@needs_runner
def test_ur_deployment_config_loads_in_the_real_runner_and_copies_no_fr3_values():
    path = UR_REPO / "ur7e_robotiq_bringup/config/ur7e_policy_deployment.yaml"
    raw = path.read_text()
    assert "fr3" not in raw.lower()
    cfg = yaml.safe_load(raw)
    assert cfg["sensors"]["ft_compensation_enabled"] is False
    assert not {"ft_compensation_mass_kg", "ft_compensation_cog_m"} & set(cfg["sensors"])
    assert "observations" not in cfg and "scraping" not in cfg
    assert yaml.safe_load((path.with_name("ur7e_policy_deployment.commissioning.yaml")).read_text())["ready"] is False
    sys.path.insert(0, str(FRANKA / "mujoco_scraping/src"))
    rc = pytest.importorskip("mujoco_multiagent_rl.policy_runner.ros2.runtime_config")
    res = rc.load_robot_runtime_config(str(path))
    assert res.overrides["robot_controller"] == "servo_admittance"
    assert res.overrides["servo_admittance_require_servo"] is True


def test_policy_status_topic_is_not_used_as_feedback():
    if not RUNNER_DIR.is_dir():
        pytest.skip("~/franka_ros2 not available")
    assert "status_pub.publish" not in (RUNNER_DIR / "node.py").read_text()
    assert "/policy_runner/status" not in (ROOT / "moveit_gui" / "policy_executor.py").read_text()


def test_manifest_load_real_template(tmp_path):
    text = (ROOT / "policies" / "_template.yaml.example").read_text()
    (tmp_path / "t.yaml").write_text(text)
    m = PolicyManifest.load(tmp_path / "t.yaml")
    assert m.backend == "servo_admittance" and m.required_tcp == "spatula_tip" and not m.command
    assert "--robot-controller" in m.build_command()
