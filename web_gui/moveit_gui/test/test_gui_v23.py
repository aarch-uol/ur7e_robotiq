"""Offscreen checks of the v2.3 pages: behaviour the screens promise, not their layout."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtWidgets  # noqa: E402

from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.gui.main_window import MainWindow  # noqa: E402
from moveit_gui.gui.setup_page import SECTIONS  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def win(app, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile)
    (tmp_path / "pol").mkdir()
    w = MainWindow(bridge, profile, tmp_path, tmp_path, tmp_path / "pol", fake=True)
    yield w
    w.close()
    bridge.shutdown()


def pump(s=0.1):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        QtWidgets.QApplication.processEvents()
        time.sleep(0.005)


def test_navigation_is_the_approved_ia_and_vision_is_out_of_setup_but_kept(win):
    assert list(win.pages) == ["OPERATE", "PROGRAMS", "AUTOMATION", "SETUP", "DIAGNOSTICS"]
    assert not any("vision" in s.lower() for s in SECTIONS)
    assert "Vision baselines" in win.pages["SETUP"].cal.pages


def test_unknown_control_state_disables_motion_everywhere(win):
    win.modes.unknown_reason = "rollback failed"
    assert win.modes.control_state_unknown
    pump(0.6)
    op = win.pages["OPERATE"]
    assert not op.movej_btn.isEnabled() and not op.movep_btn.isEnabled() and not op.change_btn.isEnabled()
    assert "UNKNOWN" in win.mode_badge.text()
    called = []
    win.start_motion(lambda: called.append(1), "x", False)
    pump()
    assert not called


def test_recording_defaults_keep_the_robot_config(win):
    a = win.pages["AUTOMATION"]
    assert a._recording().to_args() == []
    a.rec_obs.setCheckState(QtCore.Qt.Unchecked)
    a.rec_video.setCheckState(QtCore.Qt.Checked)
    a.rec_fps.setValue(15)
    a.rec_root.setText("/data/run")
    assert a._recording().to_args() == ["--no-record-obs", "--record-video", "--video-fps", "15", "--output-root",
                                        "/data/run"]


def test_vial_page_saves_a_base_fixture_and_rejects_collinear(win):
    v = win.pages["SETUP"].vial
    v.points.update({"Rim 1": np.array([.1, 0, .05]), "Rim 2": np.array([.2, 0, .05]), "Rim 3": np.array([.3, 0, .05])})
    v._update()
    assert v.result is None and v.err.text() and not v.save_btn.isEnabled()
    v.points.update({"Rim 1": np.array([.51, .1, .08]), "Rim 2": np.array([.5, .11, .08]),
                     "Rim 3": np.array([.49, .1, .08])})
    v._update()
    v.name.setText("vial_a")
    v.save()
    fx = win.store.fixtures["vial_a"]
    assert fx.parent == win.profile.base_link and fx.meta["method"] == "vial_xy_circle"
    assert np.allclose(fx.T_parent_fixture[:3, 3], [.5, .1, .08])


def test_payload_page_refuses_to_move_in_a_mode_without_moveit(win):
    pay = win.pages["SETUP"].payload
    win.mode = "impedance"
    pay.collect()
    assert not pay._running and not win.busy


def test_save_is_only_emphasised_when_the_program_changed(win):
    from moveit_gui.program import Step
    p = win.pages["PROGRAMS"]
    assert p.save_btn.property("role") == "secondary"
    p.program.steps.append(Step("wait", {"seconds": 1.0}))
    p.refresh_table(0)
    assert p.save_btn.property("role") == "primary"


def test_selecting_a_policy_shows_that_policy(win, tmp_path):
    for n in ("alpha", "beta"):
        (tmp_path / "pol" / f"{n}.yaml").write_text(
            f"name: {n}\nrequired_mode: position\ncommand: [sleep, \"1\"]\n")
    a = win.pages["AUTOMATION"]
    a.reload()
    for n in ("alpha", "beta", "alpha"):
        a.list.setCurrentRow(a.list.row(a.list.findItems(n, QtCore.Qt.MatchExactly)[0]))
        assert a.name_lbl.text() == n.upper()
        a._set_requirements(win.policies.requirements(n))
        a._set_requirements(win.policies.requirements(n))
    assert a.req_box.count() == len(a._reqs)


def test_connectivity_shows_only_what_needs_attention(win):
    win.show()
    pump(0.6)
    snap = win.bridge.snapshot()
    assert win.conn_ok.isVisible() == (all(snap.connected.get(k) for k in win.leds if k != "ExtControl")
                                       and bool(snap.program_running))
    for k, led in win.leds.items():
        state = snap.program_running if k == "ExtControl" else snap.connected.get(k)
        assert led.isVisible() == (state is not True)
