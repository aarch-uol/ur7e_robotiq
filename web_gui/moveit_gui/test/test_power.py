"""Robot power from the web GUI: power off from any browser with control; power on and brake release from any
browser with control and "I can see the robot" (MOVEIT_GUI_LOCAL_POWER_ONLY=1: only from the robot PC). The server
decides what is local, not the page."""
import json
import os
import time
import urllib.error
import urllib.request

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.server import Server  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a", "Ana")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def dash(s):
    return [c[1] for c in s.core.bridge.calls if c[0] == "dashboard"]


def logs(s):
    s.tick()
    return [line for _, line in s.logs]


def mode(s, m):
    s.core.bridge.robot_mode = m
    s.tick()


def test_power_on_and_brake_release_need_control_and_seen_from_any_browser(s):
    mode(s, "POWER_OFF")
    with pytest.raises(CommandError, match="take control"):
        s.command("b", "power_on", {"seen": True})                       # without control
    with pytest.raises(CommandError, match="can see the robot"):
        s.command("a", "power_on", {})
    with pytest.raises(CommandError, match="can see the robot"):
        s.command("a", "power_on", {"seen": "yes"})                      # only a real true counts
    with pytest.raises(CommandError, match="power on first"):
        s.command("a", "brake_release", {"seen": True})                  # brakes need a powered robot (IDLE)
    assert dash(s) == []

    assert s.command("a", "power_on", {"seen": True, "local": True})["ok"]   # remote; the page cannot claim local
    assert wait(lambda: dash(s) == ["power_on"])
    mode(s, "IDLE")
    with pytest.raises(CommandError, match="already powered on"):
        s.command("a", "power_on", {"seen": True})
    s.command("a", "brake_release", {"seen": True}, local=True)
    assert wait(lambda: dash(s) == ["power_on", "brake_release"])
    mode(s, "RUNNING")
    with pytest.raises(CommandError, match="already released"):
        s.command("a", "brake_release", {"seen": True})
    lines = logs(s)
    assert any("power on from Ana from a remote browser" in line for line in lines)
    assert any("brake release from Ana on the robot PC" in line for line in lines)
    assert s.info()["remote_power"] is True


def test_local_power_only_limits_power_on_to_the_robot_pc(s, monkeypatch):
    monkeypatch.setenv("MOVEIT_GUI_LOCAL_POWER_ONLY", "1")
    mode(s, "POWER_OFF")
    assert s.info()["remote_power"] is False
    with pytest.raises(CommandError, match="robot PC"):
        s.command("a", "power_on", {"seen": True})                       # remote browser, even with control
    with pytest.raises(CommandError, match="robot PC"):
        s.command("a", "brake_release", {"seen": True, "local": True})   # the page cannot claim local
    assert dash(s) == []
    assert s.command("a", "power_on", {"seen": True}, local=True)["ok"]
    assert wait(lambda: dash(s) == ["power_on"])


def test_power_off_from_any_browser_with_control_stops_motion_first(s):
    mode(s, "RUNNING")
    with pytest.raises(CommandError, match="take control"):
        s.command("viewer", "power_off", {})
    s.command("a", "move_joints", {"q_deg": [30, -90, 90, -90, -90, 0]})
    assert wait(lambda: s.core.busy)
    s.command("a", "power_off", {})                                      # remote: allowed
    assert wait(lambda: dash(s) == ["power_off"])
    assert any("STOP" in line for line in logs(s))
    mode(s, "POWER_OFF")
    with pytest.raises(CommandError, match="already powered off"):
        s.command("a", "power_off", {})


# ------------------------------------------------------------------ the server's "local" decision
@pytest.fixture
def http(s):
    srv = Server(s, "127.0.0.1", 0).start()
    yield f"http://127.0.0.1:{srv.port}", srv.port, s
    srv.stop()


def post(url, body, headers):
    req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json", **headers})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_server_counts_only_a_same_pc_page_without_proxy_headers_as_local(http, monkeypatch):
    monkeypatch.setenv("MOVEIT_GUI_LOCAL_POWER_ONLY", "1")             # the local check matters only then
    base, port, s = http
    mode(s, "POWER_OFF")
    body = {"client": "a", "cmd": "power_on", "args": {"seen": True}}
    refused = [
        {},                                                              # no Origin
        {"Origin": f"https://robot.tailnet.ts.net"},                     # page opened elsewhere (proxy)
        {"Origin": f"http://127.0.0.1:{port + 1}"},                      # another local port (code-server etc.)
        {"Origin": f"http://127.0.0.1:{port}", "Tailscale-User-Login": "x@y"},   # tailscale serve
        {"Origin": f"http://127.0.0.1:{port}", "X-Forwarded-For": "100.64.0.9"},
    ]
    for h in refused:
        code, r = post(base + "/api/cmd", body, h)
        # 409: refused as not-local; 403: the server's own Host/Origin check refused it even earlier
        assert code in (409, 403) and (code == 403 or "robot PC" in r["msg"]), h
    assert dash(s) == []
    code, r = post(base + "/api/cmd", body, {"Origin": f"http://localhost:{port}", "Host": f"localhost:{port}"})
    assert code == 200 and r["ok"], r
    assert wait(lambda: dash(s) == ["power_on"])
    with urllib.request.urlopen(base + "/api/info", timeout=5) as r:
        assert json.loads(r.read())["local"] is True
    req = urllib.request.Request(base + "/api/info", headers={"Tailscale-User-Login": "x@y"})
    with urllib.request.urlopen(req, timeout=5) as r:
        assert json.loads(r.read())["local"] is False


# ------------------------------------------------------------------ Dashboard opened in Local mode, pendant now Remote
REFUSED = ("Expected: Powering on, but received: Command is not allowed due to safety reasons, please switch robot "
           "to Remote Control mode and reconnect to port 29999")


class Dash:
    """The driver's dashboard services: refuses until reconnected (remote=True) or always (remote=False)."""
    def __init__(self, remote=True):
        self.remote, self.fresh, self.calls = remote, False, []

    def __call__(self, c):
        self.calls.append(c)
        if c == "quit":
            return True, "Disconnected"
        if c == "connect":
            self.fresh = True
            return True, "Connected"
        return (True, "Powering on") if self.remote and self.fresh else (False, REFUSED)


def test_dashboard_reconnects_once_when_the_connection_was_opened_in_local_mode():
    from moveit_gui.bridge_api import dashboard_with_reconnect
    d, logs = Dash(remote=True), []
    assert dashboard_with_reconnect(d, "power_on", logs.append) == (True, "Powering on")
    assert d.calls == ["power_on", "quit", "connect", "power_on"] and "reconnecting" in logs[0]
    d = Dash(remote=False)                                   # really in Local mode: refused, no loop
    ok, msg = dashboard_with_reconnect(d, "power_on", print)
    assert not ok and "still refused after reconnecting" in msg and d.calls.count("power_on") == 2
    other = lambda c: (False, "power_on: service not available")
    assert dashboard_with_reconnect(other, "power_on", print) == (False, "power_on: service not available")  # untouched


def test_play_starts_external_control_after_clearing_stale_goals(s):
    b = s.core.bridge
    mode(s, "IDLE")
    with pytest.raises(CommandError, match="release the brakes first"):
        s.command("a", "robot_play", {"program": "x"})
    mode(s, "RUNNING")
    b.program_running = True
    s.tick()
    with pytest.raises(CommandError, match="already running"):
        s.command("a", "robot_play", {"program": "x"})
    b.program_running = None                                   # never ran since the driver started
    s.tick()
    assert "Play program" in s.core.motion_refusal()
    b.pendant_program = "/programs/ros_controlX.urp"           # any name: the operator confirms it
    name = s.command("viewer", "robot_loaded_program", {})["program"]   # read-only
    assert name == "/programs/ros_controlX.urp"
    b.pendant_program = "/programs/pick_and_place.urp"        # changed on the pendant after the confirmation
    with pytest.raises(CommandError, match="not '/programs/ros_controlX.urp' that you confirmed"):
        s.command("a", "robot_play", {"program": name})
    assert dash(s) == []
    b.pendant_program = name
    epoch = b.stop_epoch
    s.command("a", "robot_play", {"program": name})            # any browser with control (it moves nothing)
    assert b.stop_epoch == epoch + 1                           # pending goals cancelled before the start
    assert wait(lambda: dash(s) == ["play"])
    s.tick()
    assert s.core.motion_refusal() is None


def test_dashboard_reconnects_once_when_the_driver_lost_the_connection():
    """The driver's dashboard_client never reconnects by itself: after a drop every call answers
    'Failed to send request to dashboard server. Are you connected to the Dashboard Server?'."""
    from moveit_gui.bridge_api import dashboard_lost, dashboard_with_reconnect
    lost = "Failed to send request to dashboard server. Are you connected to the Dashboard Server?"
    assert dashboard_lost(lost) and dashboard_lost("Attempt to write on a non-connected socket")
    assert not dashboard_lost("Powering on") and not dashboard_lost("No program loaded")
    state, calls, logs = {"up": False}, [], []

    def call(c):
        calls.append(c)
        if c == "connect":
            state["up"] = True
            return True, "Connected"
        if c == "quit":
            return False, lost
        return (True, "scrape.urp") if state["up"] else (False, lost)
    assert dashboard_with_reconnect(call, "get_loaded_program", logs.append) == (True, "scrape.urp")
    assert calls == ["get_loaded_program", "quit", "connect", "get_loaded_program"] and "was lost" in logs[0]
    down = lambda c: (True, "Connected") if c == "connect" else (False, lost)     # controller still down: no loop
    ok, msg = dashboard_with_reconnect(down, "get_loaded_program", print)
    assert not ok and "still no Dashboard connection" in msg
