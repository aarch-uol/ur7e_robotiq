"""Web front end screenshots: fake robot + web server + headless Chrome (playwright, the system Chrome).

    python3 tools/web_screenshots.py [out_dir] [WIDTHxHEIGHT]
"""
import os, sys, time, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tools"))
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp()
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import screenshots as harness
from moveit_gui import transforms as tf
from moveit_gui.calibration import TcpDef
from moveit_gui.core import RobotCore
from moveit_gui.fake_bridge import FakeBridge
from moveit_gui.profile import RobotProfile
from moveit_gui.web.server import Server
from moveit_gui.web.service import WebService
from playwright.sync_api import sync_playwright

out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "ux"
out.mkdir(parents=True, exist_ok=True)
profile = RobotProfile.load(ROOT / "profiles/ur7e_lab.yaml")
bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
bridge.servo_available = True
bridge.extra_frames["spatula_tip"] = tf.make([0, 0, 0.2])
core = RobotCore(bridge, profile, ROOT / "profiles", ROOT / "programs", harness.policies_dir(), fake=True)
core.store.put_tcp(TcpDef("spatula_tip", "virtual", parent="tool0", T_parent_tcp=tf.make([0, 0, 0.2])))
core.set_active_tcp("spatula_tip")
srv = Server(WebService(core), "127.0.0.1", 0).start()
url = f"http://127.0.0.1:{srv.port}/"
W, H = (int(v) for v in (sys.argv[2] if len(sys.argv) > 2 else "1366x768").split("x"))
with sync_playwright() as p:
    b = p.chromium.launch(channel="chrome")
    pg = b.new_page(viewport={"width": W, "height": H})
    errors = []
    pg.on("pageerror", lambda e: errors.append(str(e)))
    pg.on("console", lambda m: m.type == "error" and errors.append(m.text))
    pg.goto(url); pg.wait_for_timeout(1500)
    pg.screenshot(path=str(out / "web_operate_viewing.png"))
    pg.click("#lease-btn"); pg.wait_for_timeout(800)
    pg.screenshot(path=str(out / "web_operate.png"))
    pg.click("nav >> text=PROGRAMS"); pg.wait_for_timeout(500)
    pg.select_option("#pg-files", "example_pick.yaml"); pg.click(".page.on button:text-is('Open')"); pg.wait_for_timeout(800)
    pg.screenshot(path=str(out / "web_programs.png"))
    pg.click("nav >> text=AUTOMATION"); pg.wait_for_timeout(2500)
    pg.screenshot(path=str(out / "web_automation.png"))
    pg.click("nav >> text=SETUP"); pg.wait_for_timeout(500)
    pg.click(".subnav >> text=Cameras"); pg.wait_for_timeout(2000)
    pg.screenshot(path=str(out / "web_setup_cameras.png"))
    pg.click(".subnav >> text=Vial Geometry"); pg.wait_for_timeout(600)
    pg.screenshot(path=str(out / "web_setup_vial.png"))
    pg.click("nav >> text=DIAGNOSTICS"); pg.wait_for_timeout(800)
    pg.screenshot(path=str(out / "web_diagnostics.png"))
    if errors:
        sys.exit("browser errors: " + "; ".join(errors))
    b.close()
srv.stop(); bridge.shutdown()
