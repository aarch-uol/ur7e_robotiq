"""What building the Franka scraping baseline from scratch needed: typed waypoints (a pose in the vial frame),
an orbit about the vial axis, skip-and-continue on a failed move, a force set without waiting, loop progress."""
import math
import os

import numpy as np
import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from test_program_force import cmd, logs, run, s, tcp_base, tick_until  # noqa: E402,F401
from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.program import Program  # noqa: E402
from moveit_gui.web.service import CommandError  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


def vial_at_tcp(s, radius_mm=17.0):
    """A vial fixture whose axis is radius_mm beside the TCP (base axes, origin 40 mm below)."""
    p = tcp_base(s)[:3, 3] * 1000.0
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([p[0] - radius_mm, p[1], p[2] - 40], [0, 0, 0])))
    return p


def test_orbit_about_the_vial_axis_keeps_the_radius_and_turns_the_tool(s):
    p = vial_at_tcp(s)
    R0 = tcp_base(s)[:3, :3]
    ok, msg = run(s, [{"type": "move_relative", "offset_mm": [0, 0, 0], "rotate_deg": [0, 0, 90], "frame": "vial",
                       "pivot": "frame", "velocity": 0.5}])
    assert ok, msg
    q = tcp_base(s)[:3, 3] * 1000.0
    assert np.allclose(q, [p[0] - 17, p[1] + 17, p[2]], atol=0.2)            # a quarter turn around the axis
    assert np.allclose(tcp_base(s)[:3, :3], tf.mm_deg_to_T([0, 0, 0], [0, 0, 90])[:3, :3] @ R0, atol=1e-3)
    # about a far origin a small angle travels far: refused, not a 300 mm swing
    ok, msg = run(s, [{"type": "move_relative", "rotate_deg": [0, 0, 60], "frame": "base", "pivot": "frame",
                       "offset_mm": [0, 0, 0], "velocity": 0.5}])
    assert not ok and any("would move the TCP" in line for line in logs(s))


def test_a_failed_move_can_be_skipped_and_the_run_says_so(s):
    steps = [{"type": "move_relative", "offset_mm": [0, 0, 299], "rotate_deg": [0, 0, 0], "frame": "base",
              "pivot": "frame", "on_fail": "continue", "velocity": 0.5},
             {"type": "wait", "seconds": 0.05}]
    orig = s.core.bridge.move_pose
    s.core.bridge.move_pose = lambda *a, **k: False                          # e.g. out of reach
    try:
        ok, msg = run(s, steps)
    finally:
        s.core.bridge.move_pose = orig
    assert ok and "1 step(s) failed and were skipped" in msg and "1" in msg
    bad = Program.from_dict({"version": 2, "steps": [{"type": "wait", "seconds": 1, "on_fail": "continue"}]},
                            s.core.profile)
    assert any("on_fail" in x for x in bad.validate(s.core.profile))      # motion steps only


def test_typed_waypoints_in_a_fixture_frame_and_joints(s):
    vial_at_tcp(s)
    cmd(s, "program_new")
    r = cmd(s, "program_insert", type="move_pose", params={"frame": "vial"})
    name = r["program"]["rows"][0]["waypoint"]["name"]
    cmd(s, "program_type_waypoint", name=name, frame="vial", xyz_mm=[0, 0, 90], rpy_deg=[180, 0, 0])
    wp = cmd(s, "program_get")["program"]["rows"][0]["waypoint"]
    assert wp["frame"] == "vial" and wp["xyz_mm"] == [0, 0, 90] and abs(abs(wp["rpy_deg"][0]) - 180) < 0.1
    with pytest.raises(CommandError, match="unknown frame"):
        cmd(s, "program_type_waypoint", name=name, frame="nowhere", xyz_mm=[0, 0, 0], rpy_deg=[0, 0, 0])
    r = cmd(s, "program_insert", type="movej", path=[0])
    jn = r["program"]["rows"][1]["waypoint"]["name"]
    cmd(s, "program_type_waypoint", name=jn, joints_deg=[10, -90, 0, -90, 0, 0])
    assert cmd(s, "program_get")["program"]["rows"][1]["waypoint"]["joints_deg"][0] == 10
    with pytest.raises(CommandError, match="6 joint angles"):
        cmd(s, "program_type_waypoint", name=jn, joints_deg=[1, 2])


def test_force_without_waiting_and_loop_progress(s):
    d = {"version": 2, "name": "t", "steps": [
        {"type": "controller", "mode": "admittance"},
        {"type": "loop", "times": 3, "steps": [
            {"type": "force", "wrench": [0, 0, 3, 0, 0, 0], "frame": "tool", "wait": False},
            {"type": "wait", "seconds": 0.3}]},
        {"type": "force_off"}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    cmd(s, "program_run")
    seen = set()
    assert tick_until(s, lambda: (seen.update(tuple(x) for x in s.state()["runner"]["loops"]) or True)
                      and s.core.program_result is not None, 40)
    assert s.core.program_result[0], s.core.program_result
    assert {("2", 1, 3), ("2", 2, 3), ("2", 3, 3)} <= seen
    assert any("force: applying" in line for line in logs(s))


def test_the_franka_baseline_example_is_valid():
    prof = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    with open(os.path.join(ROOT, "programs", "example_franka_scraping_baseline.yaml")) as f:
        p = Program.from_dict(yaml.safe_load(f), prof)
    assert p.validate(prof) == []
    loop = [st for st in p.steps if st.type == "loop"][0]
    assert loop.params["times"] == 37
    assert any(c.params.get("pivot") == "frame" and c.params["rotate_deg"][2] == 10 for c in loop.children)


def _chain_urdf(names):
    """base_link -> 6 revolute joints about Z, 0.1 m apart up Z -> tool0 -> robotiq_tcp (fixed, 0.157 m)."""
    links = ["base_link"] + [f"l{i}" for i in range(len(names))]
    j = "".join(f'<joint name="{n}" type="revolute"><parent link="{links[i]}"/><child link="{links[i + 1]}"/>'
                f'<origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-6" upper="6"/></joint>'
                for i, n in enumerate(names))
    j += f'<joint name="t" type="fixed"><parent link="{links[-1]}"/><child link="tool0"/><origin xyz="0.3 0 0"/></joint>'
    j += '<joint name="r" type="fixed"><parent link="tool0"/><child link="robotiq_tcp"/><origin xyz="0 0 -0.157"/></joint>'
    return '<robot name="r">' + "".join(f'<link name="{x}"/>' for x in links + ["tool0", "robotiq_tcp"]) + j + "</robot>"


def test_the_preview_path_follows_relative_moves_and_the_orbit(s):
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    q = [0.0] * 6
    s.core.bridge._q = list(q)                                  # FK start: TCP at (0.3, 0, 0.6 - 0.157)
    s.tick()
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([280, 0, 400], [0, 0, 0])),
                             replace="vial" in s.core.store.fixtures)
    d = {"version": 2, "name": "t", "steps": [
        {"type": "move_relative", "offset_mm": [0, 0, -20], "frame": "base"},
        {"type": "loop", "times": 4, "steps": [
            {"type": "move_relative", "offset_mm": [0, 0, 0], "rotate_deg": [0, 0, 90], "frame": "vial", "pivot": "frame"}]}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    r = cmd(s, "program_preview_path")
    pts = np.asarray(r["points"])
    assert len(pts) == 6 and np.allclose(pts[0], [300, 0, 443], atol=0.5)
    assert np.allclose(pts[1], [300, 0, 423], atol=0.5)                       # the relative move
    assert np.allclose(pts[2], [280, 20, 423], atol=0.5)                      # a quarter turn about the vial axis
    assert np.allclose(pts[5], pts[1], atol=0.5)                              # four quarter turns: back again


def test_the_baseline_previews_without_a_taught_vial(s):
    """No vial fixture yet: the preview places one (preview only) so the first pose is where the tool is now."""
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.core.bridge._q = [0.0] * 6
    s.tick()
    assert "vial" not in s.core.store.fixtures
    with open(os.path.join(ROOT, "programs", "example_franka_scraping_baseline.yaml")) as f:
        cmd(s, "program_set_yaml", text=f.read())
    r = cmd(s, "program_preview_path")
    assert set(r["assumed"]) == {"vial"} and "not taught yet" in r["note"]
    assert "vial" not in s.core.store.fixtures                                # nothing saved
    assert len(r["points"]) >= 37 * 2 and len(r["poses"]) == len(r["points"]) == len(r["joints"])
    assert r["joints"][0] is not None and len(r["poses"][0]) == 7
    r2 = cmd(s, "program_preview_path", assume_missing=False)                 # the strict view still says why
    assert not r2["assumed"] and "preview stops" in r2["note"]


def test_the_example_vial_is_scraping_py_target_and_can_be_saved(s):
    """robot_demo's scraping.py: target_xyz [0.525, 0.016, 0.040]. The preview uses it until a vial is taught;
    Use example vial saves it as the real fixture on request."""
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.tick()
    with open(os.path.join(ROOT, "programs", "example_franka_scraping_baseline.yaml")) as f:
        cmd(s, "program_set_yaml", text=f.read())
    r = cmd(s, "program_preview_path")
    assert r["assumed"] == {"vial": [525.0, 16.0, 40.0]} and "example position" in r["note"]
    assert np.allclose(r["points"][2], [525, 16, 40 + 90], atol=1.0)          # approach above the centre
    rim = np.asarray(r["points"][3])                                          # then the rim at -110 deg
    assert np.allclose(rim[:2], [525 + 17 * np.cos(np.radians(-110)), 16 + 17 * np.sin(np.radians(-110))], atol=1.0)
    assert "vial" not in s.core.store.fixtures
    assert cmd(s, "vial_use_example")["ok"]
    T = s.core.geo.T_base_frame("vial")
    assert np.allclose(T[:3, 3], [0.525, 0.016, 0.040])
    r = cmd(s, "program_preview_path")
    assert not r["assumed"]                                                   # the taught one now


def test_the_preview_follows_the_active_tcp_and_says_which_move_is_which(s):
    """A taught pose puts its own TCP there (as the runner does); the line is the active TCP, so a TCP change or
    edit moves it, and the state's tcp_sig tells the page to redraw. Every move carries its step and loop turn."""
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.core.bridge._q = [0.0] * 6
    s.tick()
    d = {"version": 2, "name": "t", "steps": [
        {"type": "move_pose", "pose": {"frame": "base_link", "tcp": "robotiq_tcp", "position": [0.3, 0.0, 0.40],
                                       "quat_xyzw": [0, 0, 0, 1]}},
        {"type": "loop", "times": 3, "steps": [{"type": "move_relative", "offset_mm": [0, 0, 10], "frame": "base"}]}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    r = cmd(s, "program_preview_path")
    assert np.allclose(r["points"][1], [300, 0, 400], atol=0.5)
    assert len(r["what"]) == len(r["loops"]) == len(r["points"]) == 5
    assert r["loops"][1] == [] and r["loops"][3] == [["2", 2, 3]] and "relative" in r["what"][3].lower()
    sig = s.state()["tcp_sig"]
    cmd(s, "tcp_save", name="tip", parent="robotiq_tcp", xyz_mm=[0, 0, 100], rpy_deg=[0, 0, 0])
    cmd(s, "tcp_set_active", name="tip") if hasattr(s, "cmd_tcp_set_active") else s.core.set_active_tcp("tip")
    assert s.state()["tcp_sig"] != sig
    off = s.state()["tcp_off"]                                                # what the 3D view draws paths with
    assert off["link"] == "robotiq_tcp" and np.allclose(off["xyz_mm"], [0, 0, 100]) and np.allclose(off["rpy_deg"], 0)
    r = cmd(s, "program_preview_path")
    assert np.allclose(r["points"][1], [300, 0, 500], atol=0.5)               # robotiq_tcp at the pose, tip 100 mm on
    assert np.allclose(r["points"][4], [300, 0, 530], atol=0.5)
    sig = s.state()["tcp_sig"]
    cmd(s, "tcp_save", name="tip", parent="robotiq_tcp", xyz_mm=[0, 0, 50], rpy_deg=[0, 0, 0], replace=True)
    assert s.state()["tcp_sig"] != sig                                        # an edit of the active TCP too
    assert np.allclose(cmd(s, "program_preview_path")["points"][1], [300, 0, 450], atol=0.5)


def test_poses_with_the_active_tcp_follow_a_tcp_switch(s):
    """The user's case: the baseline with spatula_tip (150 mm on from robotiq_tcp along its Z) selected. Its poses
    say tcp: active, so the tip goes to the vial positions with either TCP and the path does not drop 150 mm."""
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.tick()
    with open(os.path.join(ROOT, "programs", "example_franka_scraping_baseline.yaml")) as f:
        cmd(s, "program_set_yaml", text=f.read())
    a = cmd(s, "program_preview_path")["points"]
    cmd(s, "tcp_save", name="spatula_tip", parent="robotiq_tcp", xyz_mm=[0, 0, 150], rpy_deg=[0, 0, 0])
    s.core.set_active_tcp("spatula_tip")
    b = cmd(s, "program_preview_path")["points"]
    d = [i for i in range(len(a)) if not np.allclose(a[i], b[i], atol=0.5)]
    assert d == [0, 1, len(a) - 1], d                                         # only the joint-space start / ready moves differ
    assert np.allclose(b[2], [525, 16, 130], atol=1.0)
    # the runner's MoveIt goal: the planning link is placed so that the spatula tip is at the pose
    assert cmd(s, "vial_use_example")["ok"]
    pose = s.program.pose_target(s.program.steps[3])
    assert pose.tcp == "active"
    goal = s.core.geo.planning_target(pose)
    want = s.core.geo.T_base_frame("vial") @ pose.T()
    assert np.allclose(goal.T_base_link @ s.core.geo.T_planning_tcp("spatula_tip"), want, atol=1e-6)
    s.core.set_active_tcp("robotiq_tcp")
    goal2 = s.core.geo.planning_target(pose)
    assert np.allclose(goal2.T_base_link @ s.core.geo.T_planning_tcp("robotiq_tcp"), want, atol=1e-6)
    assert not np.allclose(goal.T_base_link, goal2.T_base_link)               # the arm stands 150 mm back
    s.core.set_active_tcp("spatula_tip")
    with pytest.raises(CommandError, match="reserved"):
        cmd(s, "tcp_save", name="active", parent="robotiq_tcp", xyz_mm=[0, 0, 1], rpy_deg=[0, 0, 0])
    # a named TCP still means that one: then the tip is 150 mm lower, as the robot would do
    xyz, rpy = tf.T_to_mm_deg(s.program.waypoints["approach"].pose.T())
    cmd(s, "program_type_waypoint", name="approach", tcp="robotiq_tcp", xyz_mm=list(xyz), rpy_deg=list(rpy))
    assert s.program.waypoints["approach"].pose.tcp == "robotiq_tcp"
    c = cmd(s, "program_preview_path")["points"]
    assert abs(c[2][2] - (130 - 150)) < 3.0                                  # tool tilted 10.3 deg: ~148 mm lower


def test_a_path_below_the_robot_base_is_flagged(s):
    """The user's old copy: poses for robotiq_tcp, spatula_tip (+150 mm) active -> the tip dips to about -108 mm."""
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.tick()
    with open(os.path.join(ROOT, "programs", "example_franka_scraping_baseline.yaml")) as f:
        text = f.read()
    cmd(s, "program_set_yaml", text=text)
    cmd(s, "tcp_save", name="spatula_tip", parent="robotiq_tcp", xyz_mm=[0, 0, 150], rpy_deg=[0, 0, 0])
    s.core.set_active_tcp("spatula_tip")
    r = cmd(s, "program_preview_path")
    assert min(p[2] for p in r["points"][1:]) > 0 and "WARNING" not in r["note"]
    cmd(s, "program_set_yaml", text=text.replace("tcp: active", "tcp: robotiq_tcp"))
    r = cmd(s, "program_preview_path")
    assert r["note"].startswith("WARNING: the TCP goes below the robot base"), r["note"]


def test_an_untaught_vial_is_one_clear_preflight_line(s):
    with open(os.path.join(ROOT, "programs", "example_franka_scraping_baseline.yaml")) as f:
        cmd(s, "program_set_yaml", text=f.read())
    probs = s.program.validate(s.core.profile, s.core.geo, live=True, bridge=s.core.bridge)
    assert len(probs) == 1, probs
    assert probs[0].startswith("fixture 'vial' is not taught yet (used by step 4, 7, 8.5, 8.9, 10)")
    assert "Vial profiles" in probs[0]
    assert cmd(s, "vial_use_example")["ok"]
    assert s.program.validate(s.core.profile, s.core.geo, live=True, bridge=s.core.bridge) == []


def test_the_preview_stops_a_contact_search_at_a_known_vial_wall(s):
    """Move until contact is drawn to the wall of a vial of known size, not its full distance (no spiral)."""
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.core.bridge._q = [0.0] * 6
    s.tick()
    d = {"version": 2, "name": "t", "waypoints": {"in": {"pose": {
            "frame": "vial", "tcp": "active", "position": [-0.014, 0.0, 0.03], "quat_xyzw": [1, 0, 0, 0]}}},
         "steps": [{"type": "move_pose", "target": "in", "cartesian": True},
                   {"type": "loop", "times": 3, "steps": [
                       {"type": "move_until_contact", "direction": [-1, 0, 0], "frame": "vial", "distance_mm": 8,
                        "force_n": 2},
                       {"type": "move_relative", "offset_mm": [3, 0, 0], "frame": "vial"},
                       {"type": "move_relative", "offset_mm": [0, 0, 0], "rotate_deg": [0, 0, 10], "frame": "vial",
                        "pivot": "frame"}]},
                   {"type": "move_until_contact", "direction": [0, 0, -1], "frame": "vial", "distance_mm": 50,
                    "force_n": 2}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))

    def radii(r, vial_xyz):
        P = np.asarray(r["points"]) - vial_xyz
        return [round(float(np.hypot(*P[k][:2])), 1) for k, w in enumerate(r["what"]) if "Move until contact" in w], P

    ex = np.asarray(s.core.profile.example_fixtures["vial"]["xyz_mm"])
    r = cmd(s, "program_preview_path")                                   # example vial (17 mm) while assumed
    rs, P = radii(r, ex)
    assert rs[:3] == [17.0, 17.0, 17.0] and "contact: vial wall" in r["what"][2]
    assert P[-1][2] == pytest.approx(0.0, abs=0.01)                      # the downward search stops on the floor
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T(list(ex), [0, 0, 0])))
    r = cmd(s, "program_preview_path")                                   # taught without a size: full distance
    rs = radii(r, ex)[0][:3]
    assert rs[0] == 22.0 and rs[0] < rs[1] < rs[2] and "no known wall" in r["what"][2]   # 8 mm out, 3 back: grows
    cmd(s, "vial_profile_save", name="wide", xyz_mm=list(ex), radius_mm=20, height_mm=40, source="typed", use=True)
    assert radii(cmd(s, "program_preview_path"), ex)[0][:3] == [20.0, 20.0, 20.0]
