"""Transforms, TCPs, frames, calibration solvers, program v2 migration, fail-closed modes."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import (CalibrationError, CalibrationStore, FixtureDef, TcpDef,  # noqa: E402
                                    VisionBaseline, frame_from_three_points, pivot_quality,
                                    solve_pivot, solve_vision_baseline)
from moveit_gui.control_modes import ControlModeManager  # noqa: E402
from moveit_gui.fake_bridge import (FAKE_DETECTOR_FRAME, FAKE_OBJECT_TRUE, FAKE_T_TOOL0_ROBOTIQ,  # noqa: E402
                                    FakeBridge)
from moveit_gui.geometry import Geometry, GeometryError, PoseSpec  # noqa: E402
from moveit_gui.policies import PolicyLibrary  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.program import Program, ProgramRunner  # noqa: E402

PROFILE = ROOT / "profiles" / "ur7e_lab.yaml"
RNG = np.random.default_rng(42)


def rand_T(scale=0.5):
    q = RNG.normal(size=4)
    return tf.make(RNG.uniform(-scale, scale, 3), q / np.linalg.norm(q))


@pytest.fixture(autouse=True)
def _isolated_config(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))


@pytest.fixture
def profile():
    return RobotProfile.load(PROFILE)


@pytest.fixture
def bridge(profile):
    return FakeBridge(profile, joint_speed=50.0, move_time_scale=10.0, detector_noise=0.0)


@pytest.fixture
def store(profile):
    return CalibrationStore(profile)


@pytest.fixture
def geo(store, bridge):
    return Geometry(store, bridge.lookup_transform, bridge.read_pose_source)


def add_spatula(store, offset=(0.0, 0.01, 0.12), rpy_deg=(0, 0, 0)):
    store.put_tcp(TcpDef("spatula_tip", "virtual", parent="robotiq_tcp",
                         T_parent_tcp=tf.mm_deg_to_T([v * 1000 for v in offset], rpy_deg)))


# =============================================================== transforms
def test_compose_and_inverse():
    for _ in range(20):
        A, B = rand_T(), rand_T()
        assert np.allclose(tf.inv(A) @ A, np.eye(4), atol=1e-12)
        assert np.allclose(tf.inv(A @ B), tf.inv(B) @ tf.inv(A), atol=1e-12)
        assert tf.is_valid(A @ B)


def test_transform_direction_convention():
    # T_a_b maps b-coordinates into a: tool 10 cm along tool0 z, tool0 rotated 90 deg about x
    T_base_tool0 = tf.make([1, 0, 0], tf.rpy_to_quat(math.pi / 2, 0, 0))
    p_tool0 = np.array([0, 0, 0.1, 1])
    p_base = T_base_tool0 @ p_tool0
    assert np.allclose(p_base[:3], [1, -0.1, 0], atol=1e-12)    # +z of tool0 is -y of base


def test_quat_matrix_round_trip_and_sign():
    for _ in range(50):
        q = tf.normalize_quat(RNG.normal(size=4))
        q2 = tf.matrix_to_quat(tf.quat_to_matrix(q))
        assert abs(abs(np.dot(q, q2)) - 1) < 1e-9 and q2[3] >= 0


def test_rpy_round_trip_and_ui_units():
    for rpy in ([0, 0, 0], [math.pi, 0, 0], [0.3, -0.4, 1.2], [-1.0, 0.5, -2.5]):
        q1 = tf.rpy_to_quat(*rpy)
        q2 = tf.rpy_to_quat(*tf.quat_to_rpy(q1))
        assert abs(np.dot(q1, q2)) == pytest.approx(1.0, abs=1e-9)
    T = tf.mm_deg_to_T([100, -20, 5], [10, 20, 30])
    assert np.allclose(T[:3, 3], [0.1, -0.02, 0.005])
    xyz, rpy = tf.T_to_mm_deg(T)
    assert np.allclose(xyz, [100, -20, 5]) and np.allclose(rpy, [10, 20, 30])


def test_quaternion_validation():
    with pytest.raises(tf.TransformError):
        tf.check_unit_quat([0, 0, 0, 2])
    with pytest.raises(tf.TransformError):
        tf.make([0, 0, float("nan")])
    with pytest.raises(tf.TransformError):
        tf.normalize_quat([0, 0, 0, 0])


def test_quaternion_average_handles_sign_flip():
    q = tf.normalize_quat([0.1, 0.2, 0.3, 0.9])
    samples = [q, -q, q, -q]                    # same rotation, opposite signs
    avg = tf.average_quaternions(samples)
    assert abs(np.dot(avg, q)) == pytest.approx(1.0, abs=1e-12)
    naive = np.mean(samples, axis=0)
    assert np.linalg.norm(naive) < 1e-12        # what an element-wise mean would give


# ===================================================================== TCPs
def test_model_tcps_come_from_profile(store, profile):
    assert {"tool0", "robotiq_tcp"} <= set(store.tcps)
    assert store.active_tcp == profile.default_tcp == "robotiq_tcp"
    with pytest.raises(CalibrationError, match="model link"):
        store.delete_tcp("robotiq_tcp")


def test_virtual_tcp_planning_target_math(store, geo):
    add_spatula(store, rpy_deg=(0, 30, 0))
    T_base_tcp_desired = tf.mm_deg_to_T([450, 50, 120], [180, 0, 45])
    target = geo.planning_target(PoseSpec.from_T("base_link", "spatula_tip", T_base_tcp_desired))
    assert target.link == "tool0"
    T_tool0_tcp = FAKE_T_TOOL0_ROBOTIQ @ store.tcps["spatula_tip"].T_parent_tcp
    # T_base_tool0_desired = T_base_tcp_desired * inv(T_tool0_tcp)
    assert np.allclose(target.T_base_link, T_base_tcp_desired @ tf.inv(T_tool0_tcp), atol=1e-12)
    # and moving tool0 there really puts the spatula tip at the desired pose
    assert np.allclose(target.T_base_link @ T_tool0_tcp, T_base_tcp_desired, atol=1e-12)


def test_model_link_tcp_resolution(geo):
    T = tf.mm_deg_to_T([400, 0, 300], [180, 0, 0])
    direct = geo.planning_target(PoseSpec.from_T("base_link", "tool0", T))
    assert direct.link == "tool0" and np.allclose(direct.T_base_link, T)
    # robotiq_tcp has moveit_target: false -> converted to the planning link through TF
    conv = geo.planning_target(PoseSpec.from_T("base_link", "robotiq_tcp", T))
    assert conv.link == "tool0"
    assert np.allclose(conv.T_base_link @ FAKE_T_TOOL0_ROBOTIQ, T, atol=1e-12)


def test_record_then_move_is_identity(store, geo, bridge):
    """Recording T_frame_tcp and moving back to it must reproduce the same tool0 pose."""
    add_spatula(store, offset=(0.003, -0.002, 0.11), rpy_deg=(5, -10, 20))
    store.put_fixture(FixtureDef("vial_rack", "base_link", tf.mm_deg_to_T([600, -150, 10], [0, 0, 30])))
    bridge.set_fake_tool0(tf.mm_deg_to_T([420, -80, 260], [170, 10, -40]))
    T_before = bridge.lookup_transform("base_link", "tool0")
    pose = geo.current_pose("vial_rack", "spatula_tip")
    assert pose.frame == "vial_rack" and pose.tcp == "spatula_tip"
    target = geo.planning_target(pose)
    assert target.link == "tool0" and np.allclose(target.T_base_link, T_before, atol=1e-9)


def test_recorded_pose_is_relative_to_fixture(store, geo, bridge):
    T_base_rack = tf.mm_deg_to_T([600, -150, 10], [0, 0, 90])
    store.put_fixture(FixtureDef("vial_rack", "base_link", T_base_rack))
    bridge.set_fake_tool0(tf.mm_deg_to_T([600, -150, 10], [180, 0, 0]) @ tf.inv(FAKE_T_TOOL0_ROBOTIQ))
    pose = geo.current_pose("vial_rack", "robotiq_tcp")
    assert np.allclose(pose.position, [0, 0, 0], atol=1e-9)        # TCP sits on the rack origin
    pose_base = geo.current_pose("base_link", "robotiq_tcp")
    assert np.allclose(pose_base.position, [0.6, -0.15, 0.01], atol=1e-9)


def test_unknown_or_stale_frame_fails(store, geo, bridge):
    with pytest.raises(GeometryError):
        geo.planning_target(PoseSpec("nowhere", "robotiq_tcp", [0, 0, 0], [0, 0, 0, 1]))
    bridge.stale_frames.add("tool0")
    with pytest.raises(GeometryError, match="stale"):
        geo.current_pose("base_link", "robotiq_tcp")
    with pytest.raises(GeometryError, match="unknown TCP"):
        geo.planning_target(PoseSpec("base_link", "probe", [0, 0, 0], [0, 0, 0, 1]))


def test_virtual_tcp_chain(store, geo):
    add_spatula(store)
    store.put_tcp(TcpDef("spatula_edge", "virtual", parent="spatula_tip",
                         T_parent_tcp=tf.make([0.01, 0, 0])))
    link, ok, T_link_tcp = geo.tcp_anchor("spatula_edge")
    assert link == "robotiq_tcp" and not ok
    assert np.allclose(T_link_tcp, store.tcps["spatula_tip"].T_parent_tcp @ tf.make([0.01, 0, 0]))
    with pytest.raises(CalibrationError, match="parent of"):
        store.delete_tcp("spatula_tip")


# ============================================================ persistence
def test_store_round_trip_and_xdg_path(profile, store, tmp_path):
    add_spatula(store)
    store.put_fixture(FixtureDef("balance", "base_link", tf.make([0.3, 0.2, 0.0])))
    store.set_active_tcp("spatula_tip")
    assert store.path == tmp_path / "xdg" / "moveit_gui" / "calibrations" / "ur7e_lab.yaml"
    again = CalibrationStore(profile)
    assert again.active_tcp == "spatula_tip"
    assert np.allclose(again.tcps["spatula_tip"].T_parent_tcp, store.tcps["spatula_tip"].T_parent_tcp)
    assert again.tcps["spatula_tip"].meta["robot"] == "ur7e_lab" and "created" in again.tcps["spatula_tip"].meta
    assert "balance" in again.fixtures


def test_import_rejects_unsafe_and_wrong_robot(profile, store, tmp_path):
    bad = tmp_path / "evil.yaml"
    bad.write_text("!!python/object/apply:os.system ['echo pwned']\n")
    with pytest.raises(yaml.YAMLError):
        store.import_file(bad)
    other = tmp_path / "other.yaml"
    other.write_text(yaml.safe_dump({"version": 1, "robot": "ur5e", "tcps": {}}))
    with pytest.raises(CalibrationError, match="ur5e"):
        store.import_file(other)
    nonunit = tmp_path / "nonunit.yaml"
    nonunit.write_text(yaml.safe_dump({"version": 1, "robot": "ur7e_lab", "tcps": {"probe": {
        "type": "virtual", "parent": "tool0", "position": [0, 0, 0.1], "quat_xyzw": [0, 0, 0, 3]}}}))
    with pytest.raises(CalibrationError, match="unit"):
        store.import_file(nonunit)


def test_export_import_into_fresh_store(profile, store, tmp_path, monkeypatch):
    add_spatula(store)
    out = tmp_path / "export.yaml"
    store.export(out)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg2"))
    fresh = CalibrationStore(profile)
    fresh.import_file(out)
    assert "spatula_tip" in fresh.tcps
    with pytest.raises(CalibrationError, match="overwrite"):
        fresh.import_file(out)


# =================================================================== pivot
def synthetic_pivot(p_tcp, p_pivot, n, max_tilt_deg=35.0, noise_m=0.0):
    out = []
    for i in range(n):
        axis = np.array([math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n), 0.0])
        ang = math.radians(max_tilt_deg) * (0.5 + 0.5 * (i % 2))
        q = np.concatenate([axis * math.sin(ang / 2), [math.cos(ang / 2)]])
        R = tf.quat_to_matrix(q) @ tf.quat_to_matrix(tf.rpy_to_quat(math.pi, 0, 0.3 * i))
        t = np.asarray(p_pivot) - R @ np.asarray(p_tcp) + RNG.normal(0, noise_m, 3)
        out.append(tf.from_rotation(R, t))
    return out


def test_pivot_recovers_known_tcp():
    p_tcp, p_pivot = np.array([0.004, -0.012, 0.183]), np.array([0.55, 0.1, 0.05])
    res = solve_pivot(synthetic_pivot(p_tcp, p_pivot, 10))
    assert np.allclose(res.p_parent_tcp, p_tcp, atol=1e-9)
    assert np.allclose(res.p_base_pivot, p_pivot, atol=1e-9)
    assert res.rms_m < 1e-9 and res.quality == "good" and res.sample_count == 10


def test_pivot_with_noise_reports_residual():
    p_tcp = np.array([0.0, 0.0, 0.15])
    res = solve_pivot(synthetic_pivot(p_tcp, [0.5, 0, 0.1], 12, noise_m=0.0003))
    assert np.linalg.norm(res.p_parent_tcp - p_tcp) < 0.002
    assert 0.0001 < res.rms_m < 0.001 and res.max_m >= res.rms_m


def test_pivot_rejects_degenerate():
    R = tf.quat_to_matrix(tf.rpy_to_quat(math.pi, 0, 0))
    same = [tf.from_rotation(R, [0.5, 0, 0.3 + 1e-4 * i]) for i in range(8)]
    assert pivot_quality(same)[0] == "poor"
    with pytest.raises(CalibrationError, match="poor"):
        solve_pivot(same)
    with pytest.raises(CalibrationError, match="insufficient"):
        solve_pivot(synthetic_pivot([0, 0, 0.1], [0.5, 0, 0], 3))
    # rotations about a single axis only: the component along that axis is unobservable
    one_axis = [tf.from_rotation(tf.quat_to_matrix(tf.rpy_to_quat(0, 0, a)), [0.5, 0, 0.3])
                for a in np.linspace(0, 1.5, 8)]
    assert pivot_quality(one_axis)[0] == "poor"


# ================================================================ fixtures
def test_three_point_frame():
    T_true = tf.mm_deg_to_T([600, -150, 12], [2, -3, 35])
    o = T_true[:3, 3]
    px = (T_true @ [0.20, 0, 0, 1])[:3]
    py = (T_true @ [0.05, 0.15, 0, 1])[:3]          # anywhere in the +Y half-plane
    T = frame_from_three_points(o, px, py)
    assert np.allclose(T, T_true, atol=1e-12)


def test_three_point_rejects_bad_points():
    with pytest.raises(CalibrationError, match="collinear"):
        frame_from_three_points([0, 0, 0], [0.1, 0, 0], [0.2, 0.01, 0])
    with pytest.raises(CalibrationError, match="closer"):
        frame_from_three_points([0, 0, 0], [0.005, 0, 0], [0, 0.1, 0])


def test_nested_fixture_frames(store, geo):
    store.put_fixture(FixtureDef("rack", "base_link", tf.make([0.5, 0, 0])))
    store.put_fixture(FixtureDef("vial_7", "rack", tf.make([0.02, 0.04, 0.01])))
    assert np.allclose(geo.T_base_frame("vial_7")[:3, 3], [0.52, 0.04, 0.01])
    with pytest.raises(CalibrationError, match="parent of"):
        store.delete_fixture("rack")


# ========================================================= vision baseline
def test_vision_correction_reference_side():
    """Error in the reference/camera frame (e.g. extrinsics): T_det = T_err @ T_true."""
    T_expected = tf.mm_deg_to_T([500, 100, 20], [0, 0, 20])
    T_err = tf.mm_deg_to_T([6, -4, 3], [1, -2, 5])
    samples = [T_err @ T_expected for _ in range(5)]
    T_corr, stats = solve_vision_baseline(T_expected, samples, "reference", orientation=True)
    # T_correction = T_expected * inv(T_detected_baseline);  corrected = T_correction * T_detected
    assert np.allclose(T_corr, T_expected @ tf.inv(samples[0]), atol=1e-12)
    assert np.allclose(T_corr @ samples[0], T_expected, atol=1e-12)
    T_true_later = tf.mm_deg_to_T([300, -200, 50], [0, 10, -60])          # object moved and rotated
    assert np.allclose(T_corr @ (T_err @ T_true_later), T_true_later, atol=1e-12)
    assert stats["sample_count"] == 5 and stats["position_rms_m"] < 1e-12


def test_vision_correction_object_side():
    """Error in the object model frame (offset model origin): T_det = T_true @ T_err."""
    T_expected = tf.mm_deg_to_T([500, 100, 20], [0, 0, 20])
    T_err = tf.mm_deg_to_T([6, -4, 3], [1, -2, 5])
    T_corr, _ = solve_vision_baseline(T_expected, [T_expected @ T_err] * 4, "object", orientation=True)
    vb = VisionBaseline("obj", "tf", "x", "base_link", T_corr, correction_side="object")
    T_true_later = tf.mm_deg_to_T([300, -200, 50], [0, 10, -60])
    assert np.allclose(vb.corrected(T_true_later @ T_err), T_true_later, atol=1e-12)
    # the reference-side correction is exact only at the baseline pose for this kind of error
    T_corr_ref, _ = solve_vision_baseline(T_expected, [T_expected @ T_err] * 4, "reference", orientation=True)
    off = (T_corr_ref @ (T_true_later @ T_err))[:3, 3] - T_true_later[:3, 3]
    assert np.linalg.norm(off) > 0.001


def test_vision_noise_stats_and_min_samples():
    T_expected = tf.make([0.5, 0, 0])
    noisy = [T_expected @ tf.mm_deg_to_T(RNG.normal(0, 1, 3), RNG.normal(0, 0.5, 3)) for _ in range(30)]
    T_corr, stats = solve_vision_baseline(T_expected, noisy)
    assert stats["position_rms_m"] < 0.003 and stats["angle_rms_deg"] < 1.5
    assert stats["correction_translation_m"] < 0.002
    with pytest.raises(CalibrationError):
        solve_vision_baseline(T_expected, noisy[:2])


def test_robot_assisted_vision_baseline_end_to_end(store, geo, bridge):
    """Probe on the object origin gives ground truth; detector bias is removed afterwards."""
    store.put_tcp(TcpDef("probe_tip", "virtual", parent="tool0", T_parent_tcp=tf.make([0, 0, 0.2])))
    bridge.set_fake_tool0(FAKE_OBJECT_TRUE @ tf.inv(tf.make([0, 0, 0.2])))
    T_base_obj_expected = geo.current_T_base_tcp("probe_tip")
    assert np.allclose(T_base_obj_expected, FAKE_OBJECT_TRUE, atol=1e-12)
    samples = [bridge.read_pose_source("tf", FAKE_DETECTOR_FRAME, "base_link") for _ in range(5)]
    # default: position only -- the probe touch does not define the object's orientation
    T_corr, stats = solve_vision_baseline(T_base_obj_expected, samples, "object")
    assert stats["components"] == ["position"]
    store.put_vision(VisionBaseline("vial_7", "tf", FAKE_DETECTOR_FRAME, "base_link", T_corr,
                                    correction_side="object"))
    again = CalibrationStore(store.profile)                   # persisted with its side
    assert again.vision["vial_7"].correction_side == "object"
    T_obj = geo.T_base_frame("vial_7")
    assert np.allclose(T_obj[:3, 3], FAKE_OBJECT_TRUE[:3, 3], atol=1e-9)
    assert np.allclose(T_obj[:3, :3], samples[0][:3, :3], atol=1e-9)      # detector orientation kept
    bridge.stale_frames.add(FAKE_DETECTOR_FRAME)
    with pytest.raises(GeometryError, match="no fresh detection"):
        geo.T_base_frame("vial_7")


# =============================================================== programs
V1_PROGRAM = {
    "version": 1, "name": "old", "robot": "ur7e_lab",
    "waypoints": {"above": {"position": [0.4, 0.0, 0.3], "quat_xyzw": [1, 0, 0, 0]}},
    "steps": [{"type": "movej", "target": "home"},
              {"type": "movep", "target": "above"},
              {"type": "movep", "position": [0.4, 0.1, 0.25], "quat_xyzw": [1, 0, 0, 0], "cartesian": True}],
}


def test_v1_program_migrates_to_planning_link(profile, tmp_path):
    f = tmp_path / "old.yaml"
    f.write_text(yaml.safe_dump(V1_PROGRAM))
    original = f.read_text()
    prog = Program.load(f, profile)
    assert prog.notes and "v1" in prog.notes[0]
    assert f.read_text() == original                         # loading never rewrites the file
    wp = prog.waypoints["above"].pose
    assert (wp.frame, wp.tcp) == ("base_link", "tool0")      # what v1 actually executed
    st = prog.steps[2]
    assert st.type == "move_pose" and st.params["pose"]["tcp"] == "tool0" and st.params["cartesian"]
    out = tmp_path / "new.yaml"
    prog.save(out)
    d = yaml.safe_load(out.read_text())
    assert d["version"] == 2 and d["waypoints"]["above"]["pose"]["frame"] == "base_link"
    assert Program.load(out, profile).notes == []


def test_v1_migration_executes_the_same_tool0_pose(profile, store, geo):
    prog = Program.from_dict(V1_PROGRAM, profile)
    target = geo.planning_target(prog.pose_target(prog.steps[1]))
    assert target.link == "tool0" and np.allclose(target.T_base_link[:3, 3], [0.4, 0.0, 0.3])


def test_v2_pose_serialization_round_trip(profile, tmp_path):
    pose = {"frame": "vial_rack", "tcp": "spatula_tip", "position": [0.01, 0.02, 0.03],
            "quat_xyzw": [0.0, 0.0, 0.0, 1.0]}
    prog = Program.from_dict({"version": 2, "steps": [{"type": "move_pose", "pose": pose}]}, profile)
    prog.save(tmp_path / "p.yaml")
    again = Program.load(tmp_path / "p.yaml", profile)
    assert again.steps[0].params["pose"] == pose


def test_program_preflight_catches_everything_before_moving(profile, store, geo, bridge):
    prog = Program.from_dict({"version": 2, "steps": [
        {"type": "movej", "target": "home"},
        {"type": "move_pose", "pose": {"frame": "base_link", "tcp": "probe_tip",
                                       "position": [0.4, 0, 0.3], "quat_xyzw": [1, 0, 0, 0]}},
        {"type": "move_pose", "pose": {"frame": "vial_rack", "tcp": "robotiq_tcp",
                                       "position": [0, 0, 0.05], "quat_xyzw": [1, 0, 0, 0]}},
        {"type": "move_pose", "pose": {"frame": "base_link", "tcp": "tool0",
                                       "position": [0.4, 0, 0.3], "quat_xyzw": [0, 0, 0, 2]}},
        {"type": "controller", "mode": "impedance"},
        {"type": "policy", "policy": "nope"},
    ]}, profile)
    profile.modes["impedance"].allow_activation = False      # a profile that locks direct torque out
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    runner = ProgramRunner(bridge, profile, mm, geo, PolicyLibrary(ROOT / "policies"), on_log=lambda m: None)
    problems = runner.preflight(prog)
    text = "\n".join(problems)
    assert "step 2: unknown TCP 'probe_tip'" in text
    assert "fixture 'vial_rack' is not taught yet (used by step 3)" in text
    assert "step 4: malformed pose" in text
    assert "step 5: Impedance is disabled" in text
    assert "step 6: unknown policy 'nope'" in text
    assert not runner.start(prog)
    assert not any(c[0] in ("movej", "move_pose") for c in bridge.calls)   # step 1 never ran


def test_runner_moves_virtual_tcp_relative_to_fixture(profile, store, geo, bridge):
    add_spatula(store)
    store.put_fixture(FixtureDef("vial_rack", "base_link", tf.mm_deg_to_T([500, 0, 0], [0, 0, 90])))
    pose = {"frame": "vial_rack", "tcp": "spatula_tip", "position": [0.02, 0.0, 0.05],
            "quat_xyzw": [1.0, 0.0, 0.0, 0.0]}
    prog = Program.from_dict({"version": 2, "steps": [{"type": "move_pose", "pose": pose}]}, profile)
    import threading
    done = threading.Event()
    mm = ControlModeManager(bridge, profile, log=lambda m: None)
    runner = ProgramRunner(bridge, profile, mm, geo, on_log=lambda m: None, on_done=lambda ok, m: done.set())
    assert runner.start(prog)
    assert done.wait(10)
    now = geo.current_pose("vial_rack", "spatula_tip")
    assert np.allclose(now.position, pose["position"], atol=1e-6)
    assert abs(np.dot(now.quat_xyzw, pose["quat_xyzw"])) == pytest.approx(1, abs=1e-6)


# ===================================================== fail-closed switching
def _mm(bridge, profile, **kw):
    return ControlModeManager(bridge, profile, log=lambda m: None, stationary_timeout_s=0.2, **kw)


def test_switch_refused_without_velocities(profile, bridge):
    bridge.fake_velocities_available = False
    ok, msg = _mm(bridge, profile).switch_to("admittance")
    assert not ok and msg.startswith("Cannot enter Admittance") and "velocity" in msg
    assert not any(c[0] == "switch" for c in bridge.calls)


def test_switch_refused_when_state_stale(profile, bridge):
    bridge.fake_joint_state_age = 3.0
    ok, msg = _mm(bridge, profile).switch_to("admittance")
    assert not ok and "stale" in msg
    bridge.fake_joint_state_age = None
    ok, msg = _mm(bridge, profile).switch_to("admittance")
    assert not ok and "no joint state" in msg


def test_admittance_requires_ft(profile, bridge):
    bridge.ft_live = False
    ok, msg = _mm(bridge, profile).switch_to("admittance")
    assert not ok and "F/T" in msg


def test_impedance_disabled_by_profile_and_strict_preflight(profile, bridge):
    profile.modes["impedance"].allow_activation = False
    ok, msg = _mm(bridge, profile).switch_to("impedance")
    assert not ok and "locked out" in msg
    profile.modes["impedance"].allow_activation = True
    bridge.program_running = False
    bridge.safety_mode = "PROTECTIVE_STOP"
    ok, msg = _mm(bridge, profile).switch_to("impedance")
    assert not ok and "External Control" in msg and "PROTECTIVE_STOP" in msg
    bridge.program_running, bridge.safety_mode = True, "NORMAL"
    ok, msg = _mm(bridge, profile).switch_to("impedance")
    assert ok, msg


def test_switch_refused_while_policy_or_program_running(profile, bridge):
    ok, msg = _mm(bridge, profile, policy_running=lambda: True).switch_to("admittance")
    assert not ok and "policy is running" in msg
    mm = _mm(bridge, profile, program_running=lambda: True)
    ok, msg = mm.switch_to("admittance")
    assert not ok and "program is running" in msg
    ok, msg = mm.switch_to("admittance", from_program=True)     # the program's own controller step
    assert ok, msg


def test_switch_success_path_verifies(profile, bridge):
    mm = _mm(bridge, profile)
    ok, msg = mm.switch_to("admittance")
    assert ok and mm.current_mode() == "admittance"
    ok, msg = mm.switch_to("admittance")
    assert ok and "already" in msg


def test_policy_live_preflight(tmp_path, profile, bridge):
    (tmp_path / "p.yaml").write_text(yaml.safe_dump({
        "name": "p", "required_mode": "admittance", "requires_ft": True, "command": ["true"],
        "model": str(tmp_path / "missing.zip")}))
    lib = PolicyLibrary(tmp_path)
    mm = _mm(bridge, profile)
    bridge.ft_live = False
    problems = lib.manifests["p"].live_preflight(mm, bridge)
    text = "; ".join(problems)
    assert "missing.zip not found" in text and "needs control mode 'admittance'" in text and "F/T" in text


def test_position_only_baseline_ignores_probe_orientation():
    T_det = tf.mm_deg_to_T([500, 100, 20], [0, 0, 20])
    # ground truth from a probe that happens to be tilted 30 deg: its orientation must not leak in
    T_probe = tf.mm_deg_to_T([505, 96, 23], [30, 0, -70])
    for side in ("reference", "object"):
        T_corr, stats = solve_vision_baseline(T_probe, [T_det] * 4, side)
        vb = VisionBaseline("o", "tf", "x", "base_link", T_corr, correction_side=side)
        out = vb.corrected(T_det)
        assert np.allclose(out[:3, 3], T_probe[:3, 3], atol=1e-12)
        assert np.allclose(out[:3, :3], T_det[:3, :3], atol=1e-12)
        assert stats["correction_angle_deg"] == pytest.approx(0.0, abs=1e-9)


def test_fixture_import_rejects_unknown_parent(store, tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump({"version": 1, "robot": "ur7e_lab", "fixtures": {
        "rack": {"parent": "base_lnk", "position": [0, 0, 0], "quat_xyzw": [0, 0, 0, 1]}}}))
    with pytest.raises(CalibrationError, match="base_lnk"):
        store.import_file(bad)
    with pytest.raises(CalibrationError, match="parent frame"):
        store.put_fixture(FixtureDef("rack", "typo_frame", tf.make([0, 0, 0])))


# ================================================ control-stack semantics
def test_profile_control_stack_semantics(profile):
    pos, adm, imp = (profile.modes[m] for m in ("position", "admittance", "impedance"))
    assert (pos.command_interface, adm.command_interface, imp.command_interface) == ("position", "position", "effort")
    assert pos.domain == adm.domain == "position" and imp.domain == "direct_torque"
    assert adm.moveit_controller == "admittance_trajectory_controller" and adm.requires_ft
    assert imp.command_source == "cartesian_reference" and not imp.supports_moveit


def test_profile_rejects_moveit_through_direct_torque():
    from moveit_gui.profile import ProfileError
    d = yaml.safe_load(PROFILE.read_text())
    d["control_modes"]["modes"]["impedance"]["supports_moveit"] = True
    with pytest.raises(ProfileError, match="direct-torque"):
        RobotProfile.from_dict(d)


def test_transition_steps_mark_torque_boundary(profile):
    from moveit_gui.control_modes import crosses_torque_boundary, transition_steps
    assert not crosses_torque_boundary(profile, "position", "admittance")
    assert crosses_torque_boundary(profile, "admittance", "impedance")
    assert crosses_torque_boundary(profile, "unknown", "position")
    inside = " | ".join(transition_steps(profile, "position", "admittance"))
    cross = " | ".join(transition_steps(profile, "position", "impedance"))
    assert "one STRICT request" in inside and "admittance_trajectory_controller" in inside
    assert "interfaces are released" in cross and "EFFORT interface ownership" in cross
    assert "reference at the current pose" in cross


def test_switch_verifies_interface_claims(profile, bridge):
    from moveit_gui.control_modes import claim_problems
    mm = _mm(bridge, profile)
    ok, msg = mm.switch_to("admittance")
    assert ok and "POSITION interface verified" in msg
    claims = bridge.controller_claims()
    assert claims["admittance_trajectory_controller"][0].startswith("admittance_controller/")
    assert claim_problems(profile, "admittance", claims) == []
    assert claim_problems(profile, "impedance", claims)          # position claims are not effort


def test_direct_torque_requires_verified_effort_claims(profile, bridge):
    profile.modes["impedance"].allow_activation = True
    bridge.fake_claims_available = False
    ok, msg = _mm(bridge, profile).switch_to("impedance")
    assert not ok and "could not be verified" in msg
