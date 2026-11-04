#!/usr/bin/env python3
"""Render every screen with the fake robot: QT_QPA_PLATFORM=offscreen python3 tools/screenshots.py [out_dir]

Nothing touches the user's config: calibrations and manifests go to a temporary directory.
"""
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
TMP = Path(tempfile.mkdtemp(prefix="moveit_gui_shots"))
os.environ["XDG_CONFIG_HOME"] = str(TMP / "config")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
from PyQt5 import QtCore, QtWidgets  # noqa: E402

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import TcpDef  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.gui.control_stack import ControlStackDialog  # noqa: E402
from moveit_gui.gui.main_window import MainWindow  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402

MANIFEST = """name: {name}
description: {desc}
algorithm: PPO
backend: {backend}
required_mode: {mode}
required_tcp: spatula_tip
control_hz: 5.0
requires_ft: true
model: {d}/model.zip
normalization: {d}/vec_normalize.pkl
training_config: {d}/config.yaml
robot_config: {d}/deploy.yaml
entrypoint: run_policy
duration: 8.0
"""


def policies_dir() -> Path:
    root = TMP / "policies"
    d = root / "files"
    d.mkdir(parents=True)
    for f in ("model.zip", "vec_normalize.pkl", "config.yaml"):
        (d / f).write_text("")
    (d / "deploy.yaml").write_text("servo_admittance: {base_frame: base_link, ee_frame: spatula_tip}\n")
    (d / "deploy.commissioning.yaml").write_text("ready: true\n")
    (root / "scrape_ppo_v4.yaml").write_text(MANIFEST.format(
        name="scrape_ppo_v4", desc="Scraping policy, servo path", backend="servo_admittance", mode="admittance", d=d))
    (root / "scrape_impedance.yaml").write_text(MANIFEST.format(
        name="scrape_impedance", desc="Scraping policy, impedance path", backend="impedance", mode="impedance", d=d))
    return root


def pump(seconds=0.25):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        QtWidgets.QApplication.processEvents()
        time.sleep(0.01)


def wait_for(cond, timeout=30.0):
    t0 = time.monotonic()
    while not cond():
        if time.monotonic() - t0 > timeout:
            raise TimeoutError
        pump(0.02)


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else ROOT / "docs" / "ux")
    out.mkdir(parents=True, exist_ok=True)
    scale = os.environ.get("QT_SCALE_FACTOR", "1")
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    profile = RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml")
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    bridge.servo_available = True
    bridge.extra_frames["spatula_tip"] = tf.make([0, 0, 0.2])
    win = MainWindow(bridge, profile, ROOT / "profiles", ROOT / "programs", policies_dir(), fake=True)
    win.store.put_tcp(TcpDef("spatula_tip", "virtual", parent="tool0", T_parent_tcp=tf.make([0, 0, 0.2])))
    win.set_active_tcp("spatula_tip")
    win.calibration_changed.emit()
    win.show()

    def page(name):
        win.nav.setCurrentRow(list(win.pages).index(name))
        pump()
        return win.pages[name]

    def shot(size, name, target=None):
        win.resize(*size)
        pump(0.5)
        (target or win).grab().save(str(out / f"{name}.png"))
        print(name)

    if scale != "1":
        page("OPERATE")
        shot((1366, 768), "hidpi150_operate")
        return
    sizes = (1366, 768)
    page("OPERATE")
    shot(sizes, "1366_operate_position")
    d = ControlStackDialog(win)
    d._select("admittance")
    d.show()
    pump(0.5)
    d.grab().save(str(out / "1366_stack_dialog.png"))
    d.close()
    assert win.modes.switch_to("admittance")[0]
    pump(0.6)
    shot(sizes, "1366_operate_admittance")
    shot((1920, 1080), "1920_operate")

    p = page("PROGRAMS")
    p.load_file(ROOT / "programs" / "example_pick.yaml")
    for r, s in enumerate(p.program.steps):
        if s.type == "move_pose":
            p.table.selectRow(r)
            break
    shot(sizes, "1366_programs")

    a = page("AUTOMATION")
    pump(1.2)
    a.list.setCurrentRow(a.list.row(a.list.findItems("scrape_ppo_v4", QtCore.Qt.MatchExactly)[0]))
    pump(1.2)
    a.prepare()
    wait_for(lambda: a.base_bar.value() > 20)
    shot(sizes, "1366_automation_preparing")
    wait_for(lambda: a.prepared)
    pump(1.3)
    shot(sizes, "1366_automation_ready")
    shot((1920, 1080), "1920_automation")
    a.details.toggle.setChecked(True)
    pump()
    shot(sizes, "1366_automation_details")
    a.details.toggle.setChecked(False)
    a.run()
    pump(2.0)
    shot(sizes, "1366_automation_running")
    a.stop()
    wait_for(lambda: not win.policies.busy)

    s = page("SETUP")
    for row, name in ((0, "tcp"), (1, "fixtures"), (5, "profile")):
        s.sub.setCurrentRow(row)
        shot(sizes, f"1366_setup_{name}")
    s.sub.setCurrentRow(2)
    v = s.vial
    for n, pt in (("Rim 1", (.512, .1, .08)), ("Rim 2", (.5, .112, .081)), ("Rim 3", (.488, .1, .08)), ("Bottom", (.5, .1, .02))):
        v.points[n] = np.array(pt)
        v.marks[n].set_state(True)
        v.vals[n].setText("  ".join(f"{c * 1000:.1f}" for c in pt) + " mm")
    v._update()
    shot(sizes, "1366_setup_vial")
    v.save()
    s.sub.setCurrentRow(3)
    pay = s.payload
    pay.cal._sleep = lambda _s: None
    assert win.modes.switch_to("position")[0]
    pump(0.6)
    pay.collect()
    wait_for(lambda: not pay._running, 60)
    shot(sizes, "1366_setup_payload")

    s = page("SETUP")
    s.sub.setCurrentRow(4)
    pump(1.5)
    shot(sizes, "1366_setup_cameras")
    s.cameras.watch("/camera/camera/depth/image_rect_raw")
    pump(1.0)
    shot(sizes, "1366_setup_cameras_depth")
    s.cameras.watch("/zed/zed_node/rgb/image_rect_color")
    pump(0.5)
    shot(sizes, "1366_setup_cameras_missing")
    s.sub.setCurrentRow(0)

    page("DIAGNOSTICS")
    pump(0.6)
    shot(sizes, "1366_diagnostics")

    from moveit_gui.gui.scrape_dialog import ScrapeDialog
    p = page("PROGRAMS")
    d = ScrapeDialog(win, p)
    d.policy.setCurrentText("scrape_impedance")
    d.record()
    d.show()
    pump(0.5)
    d.grab().save(str(out / "1366_scrape_dialog.png"))
    d.create()
    p.program, p.path, p._saved = d.program, None, ""
    p.refresh_table(2)
    d.close()
    shot(sizes, "1366_programs_scrape")


if __name__ == "__main__":
    main()
