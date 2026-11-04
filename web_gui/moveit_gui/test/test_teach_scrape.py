"""Teaching (freedrive + record) and the scrape-around-the-vial program generator."""
import math
import os
import types

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtWidgets  # noqa: E402

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402
from moveit_gui.geometry import PoseSpec  # noqa: E402
from moveit_gui.program import Program  # noqa: E402
from moveit_gui.scrape_pattern import ScrapePattern, ScrapePatternError, build  # noqa: E402

from test_gui_v23 import app, pump, win  # noqa: E402,F401  (fixtures)

R = 0.017


def _start():
    # contact pose on the rim at +X of the vial axis, tool tilted 10 deg: an arbitrary taught pose
    return PoseSpec.from_T("vial", "spatula_tip", tf.mm_deg_to_T([R * 1000, 0, 55], [180, 10, 0]))


def _pattern(**kw):
    d = dict(start=_start(), policy="scrape", policy_mode="admittance", motion_mode="admittance",
             step_deg=90.0, count=4)
    d.update(kw)
    return ScrapePattern(**d)


def test_every_angle_is_the_taught_pose_turned_about_the_vial_axis():
    prog = build(_pattern(), "s", "ur7e_lab")
    T0 = _start().T()
    for deg in (0, 90, 180, 270):
        T = prog.waypoints[f"touch_{deg:03d}"].pose.T()
        assert math.hypot(T[0, 3], T[1, 3]) == pytest.approx(R, abs=1e-9)        # still on the rim
        assert T[2, 3] == pytest.approx(T0[2, 3])
        assert math.degrees(math.atan2(T[1, 3], T[0, 3])) % 360 == pytest.approx(deg % 360, abs=1e-6)
        # the tool turns with the angle: same pose relative to a frame rotated about the vial axis
        Rz = tf.mm_deg_to_T([0, 0, 0], [0, 0, deg])
        assert np.allclose(tf.inv(Rz) @ T, T0, atol=1e-6)
        A = prog.waypoints[f"above_{deg:03d}"].pose.T()
        assert np.allclose(A[:3, :3], T[:3, :3]) and A[2, 3] - T[2, 3] == pytest.approx(0.05)
    assert all(w.pose.frame == "vial" and w.pose.tcp == "spatula_tip" for w in prog.waypoints.values())


def test_admittance_policy_needs_no_switching_impedance_switches_around_each_run():
    adm = build(_pattern(), "s", "ur7e_lab")
    assert [s.params["mode"] for s in adm.steps if s.type == "controller"] == ["admittance"]
    assert sum(s.type == "policy" for s in adm.steps) == 4

    imp = build(_pattern(policy_mode="impedance", motion_mode="position"), "s", "ur7e_lab")
    types_ = [s.type for s in imp.steps]
    assert imp.steps[0].params == {"mode": "position"}
    for i, t in enumerate(types_):
        if t == "policy":
            # touch (MoveIt, position) -> impedance -> policy -> position -> lift
            assert imp.steps[i - 2].type == "move_pose" and imp.steps[i - 2].params["cartesian"] is True
            assert imp.steps[i - 1].params == {"mode": "impedance"}
            assert imp.steps[i + 1].params == {"mode": "position"}
            assert imp.steps[i + 2].type == "move_pose"


def test_back_and_forth_and_roundtrip(profile_path=os.path.join(os.path.dirname(__file__), "..", "profiles",
                                                                   "ur7e_lab.yaml")):
    from moveit_gui.profile import RobotProfile
    p = _pattern(step_deg=10.0, count=3, back_and_forth=True)
    assert p.angles() == [0.0, 10.0, 20.0, 20.0, 10.0, 0.0]
    prog = build(p, "s", "ur7e_lab")
    again = Program.from_dict(prog.to_dict(), RobotProfile.load(profile_path))
    assert again.to_dict() == prog.to_dict()
    with pytest.raises(ScrapePatternError):
        build(_pattern(duration=0), "s", "r")


# ------------------------------------------------------------------ GUI
def test_freedrive_from_any_page_returns_to_the_previous_mode(win):
    assert len(win.fd_buttons) >= 3           # OPERATE, PROGRAMS, SETUP > Vial Geometry
    pump(0.3)
    before = win.mode
    win.toggle_freedrive()
    pump(1.5)
    assert win.last_snap.freedrive and all(b.isChecked() for b in win.fd_buttons)
    assert win.fd_buttons[0].text() == "End freedrive"
    win.toggle_freedrive()
    pump(1.5)
    assert not win.last_snap.freedrive and win.mode == before


def test_scrape_dialog_builds_a_program_from_the_vial_and_a_recorded_pose(win):
    from moveit_gui.gui.scrape_dialog import ScrapeDialog
    win.store.put_fixture(FixtureDef("vial", win.profile.base_link, tf.make([0.5, 0.0, 0.05]),
                                     {"method": "vial_xy_circle", "radius_m": R}))
    win.policies.library.manifests["scrape"] = types.SimpleNamespace(required_mode="impedance", duration=5.0)
    n = len(win.fd_buttons)
    d = ScrapeDialog(win)
    assert d.frame.currentText() == "vial" and len(win.fd_buttons) == n + 1
    assert not d.ok.isEnabled()                      # nothing recorded yet
    assert "switches to impedance" in d.mode_lbl.text()
    pump(0.3)
    d.record()
    d.count.setValue(5)
    assert d.ok.isEnabled()
    d.create()
    prog = d.program
    assert sum(s.type == "policy" for s in prog.steps) == 5
    assert all(s.params["duration"] == 5.0 for s in prog.steps if s.type == "policy")
    d.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    pump(0.3)                                        # its freedrive button is gone and refresh still works
    assert len(win.fd_buttons) == n


def test_new_program_in_the_same_row_rebuilds_the_inspector(win):
    from moveit_gui.program import Step
    pg = win.pages["PROGRAMS"]
    pg.program = Program(name="a", steps=[Step("wait", {"seconds": 1.0})])
    pg.refresh_table(0)
    assert pg.insp_title.text() == "1 · Wait"
    pg.program = Program(name="b", steps=[Step("controller", {"mode": "position"})])
    pg.refresh_table(0)
    assert pg.insp_title.text() == "1 · Mode"
