"""Web front end: the control lease and its safety rules, commands, and the HTTP layer (no browser)."""
import json
import os
import time
import urllib.error
import urllib.request

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402
from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web import service as svc_mod  # noqa: E402
from moveit_gui.web.server import Server  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    (tmp_path / "progs").mkdir()
    (tmp_path / "pol").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    clock = Clock()
    s = WebService(core, clock=clock)
    s.tick()
    yield s, bridge, clock, tmp_path
    s.shutdown()
    bridge.shutdown()


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


# ------------------------------------------------------------------ lease
def test_commands_need_the_lease_but_stop_does_not(env):
    s, bridge, _, _ = env
    with pytest.raises(CommandError, match="take control"):
        s.command("a", "move_waypoint", {"name": "ready"})
    assert s.command("nobody", "program_list", {})["ok"]          # read-only: no lease
    s.stop("someone")
    assert ("cancel",) in bridge.calls


def test_takeover_is_refused_while_the_robot_moves(env):
    s, bridge, clock, _ = env
    a = s.take_lease("", "alice")["token"]
    bridge.joint_speed = 0.5                                      # a slow move
    s.command(a, "move_joints", {"q_deg": [30, -90, 0, -90, 0, 0], "vel": 0.1})
    assert wait(lambda: s.core.busy)
    s.tick()
    with pytest.raises(CommandError, match="alice"):
        s.take_lease("", "bob")
    s.stop()
    assert wait(lambda: not s.core.busy)
    s.tick()
    assert s.take_lease("", "bob")["name"] == "bob"             # idle: take over is allowed
    with pytest.raises(CommandError):
        s.command(a, "switch_mode", {"mode": "admittance"})


def test_lost_heartbeat_stops_a_moving_robot(env):
    s, bridge, clock, _ = env
    a = s.take_lease("", "alice")["token"]
    bridge.joint_speed = 0.5
    s.command(a, "move_joints", {"q_deg": [30, -90, 0, -90, 0, 0], "vel": 0.1})
    assert wait(lambda: s.core.busy)
    clock.t += 1.0
    assert s.heartbeat(a)
    s.tick()
    assert s.lease.holds(a)
    n = bridge.calls.count(("cancel",))
    clock.t += svc_mod.LEASE_TIMEOUT_S + 0.1                     # browser gone
    s.tick()
    assert s.lease.client is None
    assert bridge.calls.count(("cancel",)) > n
    assert any("lost contact" in line for _, line in s.logs) or any(
        "lost contact" in line for line in bridge.drain_logs())


def test_lost_heartbeat_while_idle_just_releases(env):
    s, bridge, clock, _ = env
    a = s.take_lease("")["token"]
    n = bridge.calls.count(("cancel",))
    clock.t += svc_mod.LEASE_TIMEOUT_S + 0.1
    s.tick()
    assert s.lease.client is None and bridge.calls.count(("cancel",)) == n


# --------------------------------------------------------------- commands
def test_move_runs_as_a_job_and_reports_back(env):
    s, bridge, _, _ = env
    a = s.take_lease("")["token"]
    r = s.command(a, "move_waypoint", {"name": "ready", "vel": 0.5})
    assert r["ok"] and r["job"]
    assert wait(lambda: any(x["id"] == r["job"] for x in s.results))
    res = [x for x in s.results if x["id"] == r["job"]][0]
    assert res["ok"] and res["msg"] == "motion done"
    with pytest.raises(CommandError, match="scaling"):
        s.command(a, "move_waypoint", {"name": "ready", "vel": 5})


def test_motion_rules_are_the_cores(env):
    s, _, _, _ = env
    a = s.take_lease("")["token"]
    s.core.modes.unknown_reason = "rollback failed"
    s.tick()
    with pytest.raises(CommandError, match="UNKNOWN"):
        s.command(a, "move_waypoint", {"name": "ready"})


def test_teach_scrape_save_and_no_paths_from_the_browser(env):
    s, _, _, tmp = env
    a = s.take_lease("")["token"]
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.make([0.5, 0.0, 0.05]), {"method": "vial_xy_circle"}))
    s.tick()
    r = s.command(a, "program_record", {"kind": "pose", "frame": "vial"})
    assert r["program"]["steps"][-1]["type"] == "move_pose" and "vial" in r["program"]["steps"][-1]["detail"]
    r = s.command(a, "program_record", {"kind": "joints"})
    assert r["program"]["steps"][-1]["type"] == "movej"
    r = s.command(a, "program_save", {"name": "taught"})
    assert (tmp / "progs" / "taught.yaml").exists()
    for bad in ("../evil.yaml", "/etc/passwd", "x.txt"):
        with pytest.raises(CommandError):
            s.command(a, "program_save", {"name": bad})
    assert not (tmp / "evil.yaml").exists()
    s.command(a, "program_open", {"name": "taught.yaml"})
    assert s.program.name == "taught" and len(s.program.steps) == 2


def test_program_runs_and_is_visible_in_state(env):
    s, _, _, _ = env
    a = s.take_lease("")["token"]
    s.command(a, "program_set_yaml", {"text": "version: 2\nname: t\nsteps:\n"
                                                "  - {type: movej, target: ready, velocity: 0.5}\n"
                                                "  - {type: wait, seconds: 0.1}\n"})
    s.command(a, "program_run", {})
    assert wait(lambda: s.core.program_result is not None)
    s.tick()
    st = s.state()
    assert st["runner"]["result"][0] is True and st["runner"]["state"] == "IDLE"
    with pytest.raises(CommandError, match="YAML"):
        s.command(a, "program_set_yaml", {"text": "steps: [unclosed"})


def test_vial_capture_and_save_through_the_web(env):
    s, bridge, _, _ = env
    a = s.take_lease("")["token"]
    for name, xyz in (("Rim 1", (0.512, 0.1, 0.08)), ("Rim 2", (0.5, 0.112, 0.08)), ("Rim 3", (0.488, 0.1, 0.08))):
        s.vial_points[name] = list(xyz)
    s.command(a, "vial_capture", {"point": "Bottom"})          # the live TCP pose, through the command
    assert len(s.vial_points["Bottom"]) == 3
    s.vial_points["Bottom"] = [0.5, 0.1, 0.02]
    v = s.vial_view()
    assert v["result"]["r"] == pytest.approx(0.012, abs=1e-6)
    s.command(a, "vial_save", {"name": "vial"})
    assert "vial" in s.core.store.fixtures


def test_state_is_json(env):
    s, _, _, _ = env
    json.dumps(s.state())
    json.dumps(s.info())


# ------------------------------------------------------------------ HTTP
@pytest.fixture
def http(env):
    s, bridge, _, _ = env
    srv = Server(s, "127.0.0.1", 0).start()
    base = f"http://127.0.0.1:{srv.port}"
    yield base, s, bridge
    srv.stop()


def _post(url, body, ctype="application/json"):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": ctype})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_http_static_info_and_json_only_posts(http):
    base, s, _ = http
    with urllib.request.urlopen(base + "/", timeout=5) as r:
        assert r.status == 200 and b"/static/app.js" in r.read()
    with urllib.request.urlopen(base + "/api/info", timeout=5) as r:
        assert json.loads(r.read())["profile"] == "ur7e_lab"
    code, _ = _post(base + "/api/stop", {}, ctype="text/plain")          # a cross-site form cannot do this
    assert code == 409
    code, body = _post(base + "/api/cmd", {"token": "x", "cmd": "move_waypoint", "args": {"name": "ready"}})
    assert code == 409 and "take control" in body["msg"]
    tok = _post(base + "/api/lease", {})[1]["lease"]["token"]
    code, body = _post(base + "/api/cmd", {"token": tok, "cmd": "move_waypoint", "args": {"name": "ready"}})
    assert code == 200 and body["ok"]
    for escape in ("/static/../server.py", "/static/%2e%2e/server.py"):  # no escaping the static folder
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + escape, timeout=5)
        assert e.value.code == 404


def test_http_events_stream_state(http):
    base, _, _ = http
    with urllib.request.urlopen(base + "/api/events", timeout=5) as r:
        head = r.readline().decode()
        data = r.readline().decode()
    assert head.startswith("event: state") and json.loads(data[len("data: "):])["robot_mode"] == "RUNNING"


def test_http_camera_mjpeg_and_missing_topic(http):
    base, _, bridge = http
    try:
        urllib.request.urlopen(base + "/api/camera.mjpeg?topic=/zed/none", timeout=5)
        assert False, "expected 404"
    except urllib.error.HTTPError as e:
        assert e.code == 404
    with urllib.request.urlopen(base + "/api/camera.mjpeg?topic=/camera/camera/color/image_rect_raw", timeout=5) as r:
        assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
        chunk = r.read(64 * 1024)
    assert b"--frame" in chunk and b"\xff\xd8" in chunk                  # a JPEG
    assert wait(lambda: bridge.watched_image() is None, 3.0)            # nobody watches: subscription gone
