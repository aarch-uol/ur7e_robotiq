import math
import os

import numpy as np
import pytest

from moveit_gui.flask_geometry import fit_flask
from moveit_gui.flask_scrape import (
    BentTool,
    FlaskScrapeError,
    StraightTool,
    build_program,
    clearance_reason,
    generate_scrape_poses,
    insertion_clearance_reason,
    reachable_band,
    suggest_bend,
)
from moveit_gui.profile import RobotProfile


ROOT = os.path.join(os.path.dirname(__file__), "..")


def _geometry():
    radius, neck_outer, tip = 0.0425, 0.0135, 0.002
    join = math.sqrt(radius * radius - neck_outer * neck_outer)
    rim_z = join + 0.060
    angles = np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False)
    rim = [[(neck_outer + tip) * math.cos(a), (neck_outer + tip) * math.sin(a), rim_z] for a in angles]
    rng = np.random.default_rng(8)
    directions = rng.normal(size=(60, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    bulb = directions * (radius + tip)
    return fit_flask(rim, bulb, r_tip=tip)


def _tool():
    return StraightTool("spatula_tip", shaft_radius=0.0015, shaft_length=0.25)


def _sized_geometry(od, bore, neck_length=0.050):
    radius, wall, tip = od / 2.0, 0.0015, 0.002
    neck_outer = bore / 2.0 + wall
    join = math.sqrt(radius * radius - neck_outer * neck_outer)
    rim_z = join + neck_length
    angles = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
    rim = [[(neck_outer + tip) * math.cos(a), (neck_outer + tip) * math.sin(a), rim_z]
           for a in angles]
    rng = np.random.default_rng(12)
    directions = rng.normal(size=(80, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    bulb = directions * (radius + tip)
    return fit_flask(rim, bulb, r_tip=tip, neck_inner_diameter=bore)


def test_generated_poses_are_on_sphere_through_pivot_clear_and_smooth():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0.0, 12.0, 4, 30.0, pattern="rings")
    assert plan.reachable and not plan.unreachable
    previous = None
    for target in plan.reachable:
        T = target.pose.T()
        tip, axis = T[:3, 3], T[:3, 2]
        assert np.linalg.norm(tip) == pytest.approx(g.R_inner, abs=0.0001)
        to_tip = tip - plan.pivot
        assert np.linalg.norm(np.cross(axis, to_tip)) < 1e-8
        assert axis @ to_tip > 0.0
        assert clearance_reason(T, g, tool, plan.pivot) is None
        if previous is not None:
            assert np.linalg.det(previous.T @ T[:3, :3]) == pytest.approx(1.0)
        previous = T[:3, :3]
    ring_az = [[p.azimuth_deg for p in plan.reachable if p.ring == k] for k in range(4)]
    assert ring_az[0][0] == 0.0 and ring_az[1][0] > ring_az[1][-1]


def test_unreachable_targets_have_reasons_and_band_agrees():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0.0, 65.0, 6, 30.0)
    assert plan.reachable and plan.unreachable
    assert all(u.reason and len(u.reason) < 100 for u in plan.unreachable)
    assert any("neck bore" in u.reason or "bulb wall" in u.reason for u in plan.unreachable)
    band = reachable_band(g, tool, resolution_deg=1.0, azimuth_step_deg=30.0)
    assert band is not None and band[0] == pytest.approx(0.0)
    assert 30.0 < band[1] < 60.0          # the shaft through the middle of the neck: well short of the equator


def test_short_and_wide_tools_are_rejected_with_specific_reasons():
    g = _geometry()
    short = generate_scrape_poses(g, StraightTool("tip", 0.001, 0.05), 0, 0, 1, 90)
    assert not short.reachable and all("too short" in u.reason for u in short.unreachable)
    wide = generate_scrape_poses(g, StraightTool("tip", 0.012, 0.25), 0, 0, 1, 90)
    assert not wide.reachable and all("too wide" in u.reason for u in wide.unreachable)


def test_press_depth_and_input_errors():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0, 5, 2, 90, press_depth_mm=0.5)
    assert all(np.linalg.norm(p.pose.position) == pytest.approx(g.R_inner + 0.0005, abs=1e-7)
               for p in plan.reachable)
    with pytest.raises(FlaskScrapeError, match="polar angles"):
        generate_scrape_poses(g, tool, -1, 20, 2, 10)
    with pytest.raises(FlaskScrapeError, match="ring step"):
        generate_scrape_poses(g, tool, 0, 20, 2, 0)


def test_program_is_plain_named_flask_waypoints_and_validates():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0, 10, 3, 45, pattern="rings")
    program = build_program(plan, "flask scrape", "ur7e_lab", force_n=4.0)
    assert all(w.pose.frame == "flask" and w.pose.tcp == tool.tcp for w in program.waypoints.values())
    types = [step.type for step in program.steps]
    assert types[:6] == ["comment", "controller", "move_pose", "zero_ft", "controller", "move_pose"]
    assert program.steps[1].params["mode"] == "position" and program.steps[4].params["mode"] == "admittance"
    assert types.count("force_off") == 3 and types.count("force") == len(plan.reachable)   # the wall normal per pose
    assert types[-3:] == ["move_pose", "move_pose", "controller"]
    assert program.steps[2].params["cartesian"] is False
    assert all(step.params.get("cartesian") is True for step in program.steps if step.type == "move_pose"
               and step is not program.steps[2])
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    assert program.validate(profile) == []


@pytest.mark.parametrize("od,bore", [(0.065, 0.020), (0.085, 0.021), (0.105, 0.029)])
def test_suggested_bent_tool_reaches_at_least_45_degrees(od, bore):
    g = _sized_geometry(od, bore)
    suggestion = suggest_bend(g)
    bent = suggestion.tool("bent_tip", 0.001, 0.25, 0.001)
    straight = reachable_band(g, StraightTool("straight_tip", 0.001, 0.25),
                              resolution_deg=1.0, azimuth_step_deg=90.0)
    assert insertion_clearance_reason(g, bent) is None
    assert suggestion.reachable_band_deg is not None
    assert suggestion.reachable_band_deg[1] >= 45.0
    assert straight[1] >= 30.0           # (a thin straight shaft through the middle of the neck reaches about as high)


def test_bent_poses_touch_sphere_pass_pivot_and_clear():
    g = _sized_geometry(0.085, 0.021)
    suggestion = suggest_bend(g)
    tool = suggestion.tool("bent_tip", 0.001, 0.25, 0.001)
    plan = generate_scrape_poses(g, tool, 0.0, 45.0, 6, 45.0)
    assert plan.reachable and not plan.unreachable
    angle = math.radians(tool.bend_angle_deg)
    for target in plan.reachable:
        T = target.pose.T()
        tip = T[:3, 3]
        elbow = tip - tool.bend_length * T[:3, 2]
        shaft = T[:3, :3] @ np.array([-math.sin(angle), 0.0, math.cos(angle)])
        assert np.linalg.norm(tip) == pytest.approx(g.R_inner, abs=0.0001)
        assert np.linalg.norm(np.cross(elbow - plan.pivot, shaft)) < 1e-8
        assert (elbow - plan.pivot) @ shaft >= 0.0
        assert clearance_reason(T, g, tool, plan.pivot) is None


def test_bent_insertion_rejects_profile_wider_than_bore():
    g = _sized_geometry(0.085, 0.021)
    tool = BentTool("bad_bend", 0.0015, 0.25, 0.030, 60.0, 0.0015)
    reason = insertion_clearance_reason(g, tool)
    assert reason is not None and "bend_length*sin(angle)" in reason
    plan = generate_scrape_poses(g, tool, 0.0, 10.0, 2, 90.0)
    assert not plan.reachable
    assert all("cannot pass the neck bore" in pose.reason for pose in plan.unreachable)


def test_bent_program_retracts_between_rings_and_validates():
    g = _sized_geometry(0.085, 0.021)
    suggestion = suggest_bend(g)
    tool = suggestion.tool("bent_tip", 0.001, 0.25, 0.001)
    plan = generate_scrape_poses(g, tool, 0.0, 40.0, 3, 90.0, pattern="rings")
    program = build_program(plan, "bent flask scrape", "ur7e_lab")
    pressed = build_program(plan, "bent flask scrape", "ur7e_lab", force_n=2.0)
    assert next(s for s in pressed.steps if s.type == "force").params["wrench"][:3] == [0.0, 0.0, 2.0]   # end into wall
    with pytest.raises(FlaskScrapeError, match="press axis"):
        build_program(plan, "x", "ur7e_lab", force_n=2.0, press_axis="sideways")
    targets = [s.params.get("target") for s in program.steps if s.type == "move_pose"]
    assert targets.count("pivot_safe") == 4
    assert all(w.pose.frame == "flask" for w in program.waypoints.values())
    ring_safe = [w for name, w in program.waypoints.items() if name.endswith("_safe")
                 and name != "pivot_safe"]
    assert ring_safe
    assert all(clearance_reason(w.pose.T(), g, tool, plan.pivot) is None for w in ring_safe)
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    assert program.validate(profile) == []


def test_default_pattern_scrapes_each_stroke_from_the_top_down():
    """Like the vial scrape: per angle around the axis, from the highest polar angle down to the bottom."""
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 2.0, 12.0, 3, 90.0)
    assert not plan.unreachable and len(plan.reachable) == 12
    for k in range(4):
        stroke = [p for p in plan.reachable if p.ring == k]
        assert [p.theta_deg for p in stroke] == [12.0, 7.0, 2.0]
        assert len({p.azimuth_deg for p in stroke}) == 1
        z = [p.pose.T()[2, 3] for p in stroke]
        assert z[0] > z[1] > z[2]                                      # down the wall
        assert [p.name for p in stroke] == [f"stroke_{k + 1:02d}_{i:03d}" for i in (1, 2, 3)]
    assert [p.azimuth_deg for p in plan.reachable[::3]] == [0.0, 90.0, 180.0, 270.0]
    for p in plan.reachable:                     # TCP X faces the wall at that angle, Z goes from pivot to wall
        T, a = p.pose.T(), math.radians(p.azimuth_deg)
        out = np.array([math.cos(a), math.sin(a), 0.0])
        assert T[:3, 0] @ out > 0.95 and abs(T[:3, 0] @ T[:3, 2]) < 1e-9
        assert np.allclose(np.cross(T[:3, 3] - plan.pivot, T[:3, 2]), 0.0, atol=1e-9)
    with pytest.raises(FlaskScrapeError):
        generate_scrape_poses(g, tool, 2.0, 12.0, 3, 90.0, pattern="spiral")


def test_down_program_enters_at_the_top_presses_scrapes_down_and_retracts_per_stroke():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 2.0, 12.0, 3, 90.0)
    program = build_program(plan, "flask scrape", "ur7e_lab", force_n=3.0)
    t = [(s.type, s.params.get("target")) for s in program.steps]
    i = t.index(("move_pose", "stroke_01_entry_safe"))
    assert t[i:i + 12] == [("move_pose", "stroke_01_entry_safe"), ("move_pose", "stroke_01_near"),
                           ("move_pose", "stroke_01_001"), ("force", None), ("wait", None),
                           ("force", None), ("move_pose", "stroke_01_002"), ("force", None),
                           ("move_pose", "stroke_01_003"), ("force_off", None), ("wait", None),
                           ("move_pose", "stroke_01_exit_safe")]
    assert [x for _, x in t].count("pivot_safe") == 5                 # in, between the 4 strokes, out
    near, first = (program.waypoints[n].pose.T() for n in ("stroke_01_near", "stroke_01_001"))
    assert np.linalg.norm(first[:3, 3] - near[:3, 3]) == pytest.approx(0.005)
    assert np.allclose(np.cross(first[:3, 3] - near[:3, 3], first[:3, 2]), 0.0, atol=1e-9)   # along the shaft
    forces = [s for s in program.steps if s.type == "force"][:3]
    for name, f in zip(("stroke_01_001", "stroke_01_002", "stroke_01_003"), forces):   # into the wall along its normal
        T = program.waypoints[name].pose.T()
        w = T[:3, :3] @ np.array(f.params["wrench"][:3])
        assert np.allclose(w, 3.0 * T[:3, 3] / np.linalg.norm(T[:3, 3]), atol=1e-3), name
        assert f.params["frame"] == "tool" and f.params["wrench"][3:] == [0.0, 0.0, 0.0]
    fixed = build_program(plan, "flask scrape", "ur7e_lab", force_n=3.0, press_axis="+x")
    assert [s.params["wrench"] for s in fixed.steps if s.type == "force"] == [[3.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 4
    entry = program.waypoints["stroke_01_entry_safe"].pose.T()
    assert entry[2, 3] > program.waypoints["stroke_01_001"].pose.T()[2, 3]    # comes in from above
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    assert program.validate(profile) == []


def test_the_rbf_example_scrapes_the_curved_wall_top_to_bottom_through_the_neck():
    """24 strokes from 75 deg down to the bottom; every pose, and the line between two poses, clears the neck."""
    import yaml
    from moveit_gui import flask_setup as FS
    from moveit_gui import transforms as tf
    from moveit_gui.flask_scrape import _neck_clearance, _tool_pivot
    d = yaml.safe_load(open(os.path.join(ROOT, "programs", "example_rbf_scrape_speed.yaml")))
    g = FS.fit(FS.synthetic_touches([0, 0, 0], neck_od_mm=43.0, tip_mm=0.0), FS.fit_settings(tip_mm=0.0, neck_id_mm=40.0))
    tool = StraightTool("active", 0.003, 0.300)
    pivot = _tool_pivot(g, tool, None)
    order = [s["target"] for s in d["steps"] if s["type"] == "move_pose" and s["target"].startswith("stroke_")
             and not s["target"].endswith(("_safe", "_near"))]
    assert len(order) == 24 * 16
    T = {n: PoseT(d["waypoints"][n]["pose"]) for n in d["waypoints"]}
    tops = []
    for k in range(24):
        stroke = order[16 * k:16 * k + 16]
        assert {n[:9] for n in stroke} == {f"stroke_{k + 1:02d}"}
        xyz = np.array([T[n][:3, 3] for n in stroke])
        assert np.all(np.diff(xyz[:, 2]) < 0), stroke[0]                # Z goes down the wall
        assert np.allclose(np.linalg.norm(xyz, axis=1), g.R_inner, atol=1e-6)   # on the curved wall
        top = math.degrees(math.atan2(math.hypot(*xyz[0, :2]), -xyz[0, 2]))
        assert top == pytest.approx(75.0, abs=1e-3) and xyz[-1, 2] == pytest.approx(-g.R_inner, abs=1e-6)
        tops.append(math.degrees(math.atan2(xyz[0, 1], xyz[0, 0])) % 360.0)
        for a, b in zip(stroke, stroke[1:]):
            assert clearance_reason(T[a], g, tool, pivot) is None, a
            for f in (0.25, 0.5, 0.75):        # a linear move between two poses: the shaft stays in the bore
                p = (1 - f) * T[a][:3, 3] + f * T[b][:3, 3]
                qa, qb = tf.matrix_to_quat(T[a][:3, :3]), tf.matrix_to_quat(T[b][:3, :3])
                q = (1 - f) * qa + f * (qb if qa @ qb >= 0 else -qb)
                axis = tf.quat_to_matrix(q / np.linalg.norm(q))[:, 2]
                assert _neck_clearance(p, axis / np.linalg.norm(axis), g, tool, 0.0009) is None, (a, f)
    assert np.allclose(tops, np.arange(0.0, 360.0, 15.0), atol=0.01)     # all the way round (um rounding)
    forces = [s for s in d["steps"] if s["type"] == "force"]
    assert len(forces) == 24 * 16 and all(abs(np.linalg.norm(f["wrench"][:3]) - 3.0) < 1e-3 for f in forces)


def PoseT(p):
    from moveit_gui.geometry import PoseSpec
    return PoseSpec.from_dict(p).T()


def test_a_set_pitch_tilts_the_tool_and_the_neck_still_decides():
    """Pitch: the straight tool's tilt from vertical toward the wall (one value, or top and bottom of a stroke)."""
    from moveit_gui import flask_setup as FS
    g = FS.fit(FS.synthetic_touches([0, 0, 0], neck_od_mm=43.0, tip_mm=0.0), FS.fit_settings(tip_mm=0.0, neck_id_mm=40.0))
    tool = StraightTool("tip", 0.003, 0.300)
    plan = generate_scrape_poses(g, tool, 0.0, 30.0, 7, 90.0, pitch_deg=(30.0, 0.0))
    assert plan.reachable
    for p in plan.reachable:
        T, a = p.pose.T(), math.radians(p.azimuth_deg)
        want = 30.0 * p.theta_deg / 30.0                                 # 30 at the top, 0 at the bottom
        assert math.degrees(math.acos(-T[2, 2])) == pytest.approx(want, abs=1e-6)
        if want > 1e-6:                                                 # leaning toward the wall at that angle
            assert T[:2, 2] @ np.array([math.cos(a), math.sin(a)]) > 0
        assert clearance_reason(T, g, tool, p.pivot) is None
        assert p.pivot is not None and np.linalg.norm(np.cross(T[:3, 3] - p.pivot, T[:3, 2])) < 1e-9
    steep = generate_scrape_poses(g, tool, 0.0, 30.0, 7, 90.0, pitch_deg=60.0)
    assert not steep.reachable and all("neck bore" in u.reason for u in steep.unreachable)
    program = build_program(plan, "pitched", "ur7e_lab", force_n=2.0)
    entry = program.waypoints["stroke_01_entry_safe"].pose.T()
    assert np.allclose(entry[:3, 3], plan.reachable[0].pivot)            # backs out to its own crossing of the neck
    with pytest.raises(FlaskScrapeError, match="pitch"):
        generate_scrape_poses(g, tool, 0.0, 30.0, 7, 90.0, pitch_deg=95.0)


def test_where_the_line_through_the_middle_of_the_neck_is_blocked_an_off_axis_line_reaches():
    """Not one focus point: the shaft may cross the neck off the axis, and the retract goes back along that line."""
    from moveit_gui import flask_setup as FS
    g = FS.fit(FS.synthetic_touches([0, 0, 0], neck_od_mm=33.0, tip_mm=0.0), FS.fit_settings(tip_mm=0.0, neck_id_mm=30.0))
    tool = StraightTool("a", 0.003, 0.300)
    assert not generate_scrape_poses(g, tool, 53.0, 53.0, 1, 90.0, free_line=False).reachable
    plan = generate_scrape_poses(g, tool, 53.0, 53.0, 1, 90.0)
    assert len(plan.reachable) == 4 and not plan.unreachable
    for p in plan.reachable:
        assert p.pivot is not None and abs(math.hypot(p.pivot[0], p.pivot[1])) > 1e-4
        assert clearance_reason(p.pose.T(), g, tool, p.pivot) is None
