"""SETUP > Flask Geometry on the fake robot: touch -> fit -> save (fixture "flask", profiles), and the flask scrape
generator (reach, program). The touches come from the fake TCP pose, placed by `touch_flask` on a synthetic flask."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from test_program_force import cmd, s, tcp_base  # noqa: E402,F401
from moveit_gui import flask_setup as FS  # noqa: E402
from moveit_gui.program import Program  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

CENTRE = np.array([0.45, 0.10, 0.12])
TOOL = {"kind": "straight", "shaft_radius_mm": 3.0, "shaft_length_mm": 250.0}


def touch_flask(s, centre=CENTRE, tip_mm=2.0, noise_mm=0.0, **kw):
    """Put the fake TCP on each synthetic touch of a flask at `centre` and capture it there."""
    pts = FS.synthetic_touches(centre, tip_mm=tip_mm, noise_mm=noise_mm, **kw)
    real = s.core.geo.current_T_base_tcp
    try:
        for kind in FS.KINDS:
            for p in pts[kind]:
                T = np.eye(4)
                T[:3, 3] = p
                s.core.geo.current_T_base_tcp = lambda name, T=T: T
                r = cmd(s, "flask_capture", kind=kind, tip_mm=tip_mm)
    finally:
        s.core.geo.current_T_base_tcp = real
    return r


def test_capture_fit_save_makes_the_flask_frame_and_a_profile(s):
    v = cmd(s, "flask_get")["flask"]
    assert v["result"] is None and v["hint"]["kind"] == "rim"
    r = touch_flask(s)["flask"]
    assert [len(r["points"][k]) for k in FS.KINDS] == [4, 8, 3]
    res = r["result"]
    assert res["od_mm"] == pytest.approx(85.0, abs=0.01) and res["size"] == "250 ml"
    assert res["neck_length_mm"] == pytest.approx(50.0, abs=0.05) and res["warnings"] == []
    assert res["r_inner_mm"] == pytest.approx(41.0, abs=0.01)              # wall 1.5 mm by default
    assert res["draw"]["sphere"]["c"] == pytest.approx(list(CENTRE * 1000), abs=0.01)
    # remove one, the fit still works; a neck inner diameter changes the bore
    r = cmd(s, "flask_remove", kind="bulb", index=0)["flask"]
    assert len(r["points"]["bulb"]) == 7 and r["result"]
    r = cmd(s, "flask_get", neck_id_mm=24)["flask"]
    assert r["result"]["neck_id_mm"] == pytest.approx(24.0)
    r = cmd(s, "flask_save", name="flask A", neck_id_mm=24)
    assert r["ok"] and "250 ml" in r["msg"]
    T = s.core.geo.T_base_frame("flask")
    assert np.allclose(T[:3, 3], CENTRE, atol=1e-5) and np.allclose(T[:3, :3], np.eye(3), atol=1e-6)
    fx = s.core.store.fixtures["flask"]
    assert fx.meta["flask_profile"] == "flask A" and fx.meta["flask_geometry"]["neck_inner_radius"] == pytest.approx(0.012)
    assert "radius_m" not in fx.meta                                       # not a vial of known size
    p = {e["name"]: e for e in r["flask"]["profiles"]}
    assert p["flask A"]["in_use_as"] == ["flask"] and r["flask"]["fixture"]["size"] == "250 ml"


def test_flask_profiles_list_use_delete_and_survive_a_restart(s):
    touch_flask(s)
    cmd(s, "flask_save", name="A")
    cmd(s, "flask_reset")
    touch_flask(s, centre=CENTRE + [0.1, 0, 0], od_mm=105.0, neck_od_mm=32.0)
    r = cmd(s, "flask_save", name="B")
    assert "500 ml" in r["msg"]
    assert np.allclose(s.core.geo.T_base_frame("flask")[:3, 3], CENTRE + [0.1, 0, 0], atol=1e-5)
    r = cmd(s, "flask_profile_use", name="A")
    p = {e["name"]: e for e in r["flask"]["profiles"]}
    assert p["A"]["in_use_as"] == ["flask"] and p["B"]["in_use_as"] == []
    assert np.allclose(s.core.geo.T_base_frame("flask")[:3, 3], CENTRE, atol=1e-5)
    s2 = WebService(s.core)
    try:
        assert {e["name"] for e in s2.cmd_flask_profiles()["profiles"]} == {"A", "B"}
    finally:
        s2.shutdown()
    cmd(s, "flask_profile_delete", name="B")
    assert [e["name"] for e in cmd(s, "flask_profiles")["profiles"]] == ["A"]
    with pytest.raises(CommandError, match="no flask profile"):
        cmd(s, "flask_profile_use", name="B")
    with pytest.raises(CommandError, match="name"):
        cmd(s, "flask_save", name="")


def test_the_file_is_per_robot_under_config_flasks(s, tmp_path):
    touch_flask(s)
    cmd(s, "flask_save", name="A")
    assert (tmp_path / "cfg" / "moveit_gui" / "flasks" / "ur7e_lab.yaml").exists()


def test_save_needs_enough_points_and_the_hint_says_what_next(s):
    with pytest.raises(CommandError, match="3 rim points and 4 bulb"):
        cmd(s, "flask_save", name="x")
    pts = FS.synthetic_touches(CENTRE, rim=3, bulb=2, neck=0)
    s.flask_points = pts
    h = cmd(s, "flask_get")["flask"]["hint"]
    assert h["kind"] == "bulb" and "2 more" in h["text"]
    s.flask_points = FS.synthetic_touches(CENTRE, rim=3, bulb=4, neck=0)
    h = cmd(s, "flask_get")["flask"]["hint"]
    assert h["xyz"] is not None                                            # a fitted sphere: a point to show


def test_bad_touches_give_plain_warnings_not_a_crash(s):
    s.flask_points = FS.synthetic_touches(CENTRE, noise_mm=2.0, seed=3)
    v = cmd(s, "flask_get")["flask"]
    assert v["error"] or any("RMS" in w for w in v["result"]["warnings"])


def test_admin_only_for_capture_save_delete_operator_for_reach(s):
    touch_flask(s)
    cmd(s, "flask_save", name="A")
    for c, a in [("flask_capture", {"kind": "rim"}), ("flask_save", {"name": "B"}), ("flask_reset", {}),
                 ("flask_remove", {"kind": "rim", "index": 0}), ("flask_profile_delete", {"name": "A"})]:
        with pytest.raises(CommandError, match="only an Admin"):
            s.command("a", c, a, role="operator")
    r = s.command("a", "flask_reach", {"tool": TOOL, "theta_max_deg": 10}, role="operator")
    assert r["ok"]
    r = s.command("a", "program_flask_scrape", {"tool": TOOL, "theta_max_deg": 10}, role="operator")
    assert r["ok"]
    s.command("a", "flask_profile_use", {"name": "A"}, role="operator")         # like vial_profile_use


def test_fake_touches_only_on_the_fake_robot(s):
    r = cmd(s, "flask_fake_touches")["flask"]
    assert r["result"]["size"] == "250 ml"
    s.core.fake = False
    try:
        with pytest.raises(CommandError, match="simulated"):
            cmd(s, "flask_fake_touches")
    finally:
        s.core.fake = True


def test_reach_groups_the_reasons_and_the_straight_tool_band_is_small(s):
    with pytest.raises(CommandError, match="save a flask first"):
        cmd(s, "flask_reach", tool=TOOL)
    touch_flask(s)
    cmd(s, "flask_save", name="A", neck_id_mm=24)
    r = cmd(s, "flask_reach", tool=TOOL, theta_min_deg=0, theta_max_deg=40, rings=5, step_deg=30)
    m = r["reach"]
    lo, hi = m["band"]
    assert lo == 0.0 and 10.0 <= hi <= 20.0                     # straight tool: only near the bottom
    assert m["reachable"] > 0 and m["unreachable"] > 0
    assert sum(n for _, n in m["reasons"]) == m["unreachable"]
    assert any(p["ok"] for p in m["poses"])
    assert m["draw"]["pivot"][2] > CENTRE[2] * 1000                      # the pivot is up in the neck
    with pytest.raises(CommandError, match="unknown TCP"):
        cmd(s, "flask_reach", tool=TOOL, tcp="nope")
    with pytest.raises(CommandError, match="bend"):
        cmd(s, "flask_reach", tool={"kind": "bent", "shaft_radius_mm": 3, "shaft_length_mm": 250})
    with pytest.raises(CommandError, match="unknown tool"):
        cmd(s, "flask_reach", tool={"kind": "hook"})
    with pytest.raises(CommandError, match="shaft radius"):
        cmd(s, "flask_reach", tool={"kind": "straight", "shaft_length_mm": 100})


def test_make_program_builds_an_unsaved_program_that_validates(s):
    touch_flask(s)
    cmd(s, "flask_save", name="A", neck_id_mm=24)
    s.program_path = None
    s._mark_saved()
    r = cmd(s, "program_flask_scrape", tool=TOOL, tcp="robotiq_tcp", theta_min_deg=0, theta_max_deg=12, rings=3,
            step_deg=30, press_mm=0.5, speed_mm_s=10)
    assert r["ok"] and r["program"]["dirty"] and s.program_path is None
    assert r["reach"]["reachable"] >= 13
    prog = s.program
    lin = [st for st in prog.steps if st.type == "move_pose" and st.params.get("cartesian")]
    assert lin and all(st.params["speed_mm_s"] == 10.0 for st in lin)
    assert all(w.pose.frame == "flask" and w.pose.tcp == "robotiq_tcp" for w in prog.waypoints.values())
    assert prog.validate(s.core.profile, s.core.geo) == []
    assert cmd(s, "program_check")["problems"] == []                          # the live preflight too
    assert Program.from_dict(prog.to_dict(), s.core.profile).steps            # round-trips as YAML
    assert s.core.runner.state.name == "IDLE"                                 # never runs it
    cmd(s, "program_undo")                                                    # an ordinary program edit
    assert s.program.name != "flask_scrape"
    # with a force: the admittance mode, a force step and force off
    r = cmd(s, "program_flask_scrape", tool=TOOL, theta_max_deg=12, rings=2, step_deg=45, force_n=3)
    types = [st.type for st in s.program.steps]
    assert "force" in types and "force_off" in types
    with pytest.raises(CommandError, match="speed"):
        cmd(s, "program_flask_scrape", tool=TOOL, theta_max_deg=12, speed_mm_s=0)
    with pytest.raises(CommandError, match="no reachable"):
        cmd(s, "program_flask_scrape", tool=TOOL, theta_min_deg=60, theta_max_deg=80, rings=2)


def test_bent_tool_suggest_reach_and_program(s):
    touch_flask(s)
    cmd(s, "flask_save", name="A", neck_id_mm=24)
    g = cmd(s, "flask_suggest_bend", shaft_radius_mm=1.5, shaft_length_mm=250, theta_min_deg=0, theta_max_deg=60)
    sg = g["suggestion"]
    assert sg["length_mm"] > 0 and 0 < sg["angle_deg"] < 90 and sg["band"]
    bent = {"kind": "bent", "shaft_radius_mm": 1.5, "shaft_length_mm": 250,
            "bend_length_mm": sg["length_mm"], "bend_angle_deg": sg["angle_deg"]}
    straight = {**bent, "kind": "straight"}
    rb = cmd(s, "flask_reach", tool=bent, theta_min_deg=0, theta_max_deg=40, rings=3, step_deg=45)["reach"]
    rs = cmd(s, "flask_reach", tool=straight, theta_min_deg=0, theta_max_deg=40, rings=3, step_deg=45)["reach"]
    assert rb["band"][1] > rs["band"][1]                                 # the bend reaches higher up the bulb
    assert all(len(p["pts"]) == 3 for p in rb["poses"])                 # tip, elbow, shaft
    assert all(len(p["pts"]) == 2 for p in rs["poses"])
    r = cmd(s, "program_flask_scrape", tool=bent, theta_min_deg=0, theta_max_deg=min(30, rb["band"][1]), rings=2,
            step_deg=60)
    assert r["ok"] and r["reach"]["reachable"] > 0
    assert s.program.validate(s.core.profile, s.core.geo) == []
    assert cmd(s, "program_check")["problems"] == []
