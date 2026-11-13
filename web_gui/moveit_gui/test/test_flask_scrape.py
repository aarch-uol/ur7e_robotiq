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
    assert 5.0 < band[1] < 40.0


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
    assert types[:5] == ["comment", "controller", "move_pose", "move_pose", "move_pose"]
    assert types.count("force") == 3 and types.count("force_off") == 3
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
    assert suggestion.reachable_band_deg[1] >= straight[1] + 18.0


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
    with pytest.raises(FlaskScrapeError):
        generate_scrape_poses(g, tool, 2.0, 12.0, 3, 90.0, pattern="spiral")


def test_down_program_enters_at_the_top_presses_scrapes_down_and_retracts_per_stroke():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 2.0, 12.0, 3, 90.0)
    program = build_program(plan, "flask scrape", "ur7e_lab", force_n=3.0)
    t = [(s.type, s.params.get("target")) for s in program.steps]
    i = t.index(("move_pose", "stroke_01_entry_safe"))
    assert t[i:i + 7] == [("move_pose", "stroke_01_entry_safe"), ("move_pose", "stroke_01_001"), ("force", None),
                          ("move_pose", "stroke_01_002"), ("move_pose", "stroke_01_003"), ("force_off", None),
                          ("move_pose", "stroke_01_exit_safe")]
    assert [x for _, x in t].count("pivot_safe") == 5                 # in, between the 4 strokes, out
    entry = program.waypoints["stroke_01_entry_safe"].pose.T()
    assert entry[2, 3] > program.waypoints["stroke_01_001"].pose.T()[2, 3]    # comes in from above
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    assert program.validate(profile) == []


def test_the_rbf_example_scrapes_top_to_bottom():
    import yaml
    d = yaml.safe_load(open(os.path.join(ROOT, "programs", "example_rbf_scrape_speed.yaml")))
    order = [s["target"] for s in d["steps"] if s["type"] == "move_pose" and s["target"].startswith("stroke_")
             and not s["target"].endswith("_safe")]
    assert len(order) == 36
    for k in range(12):
        z = [d["waypoints"][n]["pose"]["position"][2] for n in order[3 * k:3 * k + 3]]
        assert z[0] > z[1] > z[2], order[3 * k]
