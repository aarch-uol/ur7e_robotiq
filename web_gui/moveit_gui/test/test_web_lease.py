"""Web front end: lease token, watchdogs that survive a failing core.update(), host/origin checks, edit serialisation."""
import http.client
import json
import os
import threading
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.web import service as svc_mod  # noqa: E402
from moveit_gui.web.server import Server, check_bind_host, host_allowed  # noqa: E402
from moveit_gui.web.service import CommandError  # noqa: E402
from test_web import env, wait  # noqa: E402,F401  (fixture reuse)


def _slow_move(s, bridge, tok):
    bridge.joint_speed = 0.5
    s.command(tok, "move_joints", {"q_deg": [30, -90, 0, -90, 0, 0], "vel": 0.1})
    assert wait(lambda: s.core.busy)


# ------------------------------------------------------------------ token
def test_token_is_secret_and_not_broadcast(env):
    s, _, _, _ = env
    r = s.take_lease("", "alice")
    tok = r["token"]
    assert len(tok) >= 20 and r["client"] and r["client"] != tok
    s.tick()
    assert tok not in json.dumps(s.state()) and tok not in json.dumps(s.info())
    assert s.state()["lease"] == {"client": r["client"], "name": "alice"}
    # the public id is no credential
    with pytest.raises(CommandError, match="take control"):
        s.command(r["client"], "move_waypoint", {"name": "ready"})
    assert not s.heartbeat(r["client"]) and not s.heartbeat("")
    s.release("wrong")
    assert s.lease.holds(tok)


def test_takeover_invalidates_the_old_token(env):
    s, _, _, _ = env
    a = s.take_lease("", "alice")["token"]
    b = s.take_lease("", "bob")["token"]
    assert a != b and not s.heartbeat(a) and s.heartbeat(b)
    with pytest.raises(CommandError, match="take control"):
        s.command(a, "move_waypoint", {"name": "ready"})


def test_any_request_with_the_token_is_a_heartbeat(env):
    s, _, clock, _ = env
    a = s.take_lease("", "alice")["token"]
    for call in (lambda: s.command(a, "program_list", {}),                  # read-only
                 lambda: s.command(a, "hold_renew", {}),
                 lambda: s.command(a, "jog_stop", {})):
        clock.t += svc_mod.LEASE_TIMEOUT_S - 0.5
        call()
        s.tick()
        assert s.lease.holds(a)
    clock.t += svc_mod.LEASE_TIMEOUT_S - 0.5
    s.command("not-the-token", "program_list", {})                           # read-only without token: no renewal
    clock.t += 1.0
    s.tick()
    assert s.lease.client is None


# -------------------------------------------------------------- watchdogs
def test_lease_times_out_even_if_core_update_raises(env):
    s, bridge, clock, _ = env
    a = s.take_lease("", "alice")["token"]
    _slow_move(s, bridge, a)

    def boom():
        raise RuntimeError("update broke")
    s.core.update = boom
    n = bridge.calls.count(("cancel",))
    s.tick()                                                                  # raising update must not escape
    assert s.lease.holds(a)
    clock.t += svc_mod.LEASE_TIMEOUT_S + 0.1
    s.tick()
    assert s.lease.client is None and bridge.calls.count(("cancel",)) > n
    lines = [line for _, line in s.logs] + bridge.drain_logs()
    assert any("lost contact" in line and "no heartbeat for 2.6 s" in line for line in lines)
    assert any("core.update failed" in line for line in lines)
    s.tick(); s.tick()
    assert sum("core.update failed" in line for _, line in s.logs) <= 1      # rate limited


def test_hold_watchdog_runs_even_if_update_raises(env):
    s, bridge, clock, _ = env
    s._hold = {"seen": clock(), "label": "x"}
    s.core.update = lambda: 1 / 0
    n = bridge.calls.count(("cancel",))
    clock.t += svc_mod.HOLD_TIMEOUT_S + 0.1
    s.tick()
    assert s._hold is None and bridge.calls.count(("cancel",)) > n


def test_failing_stop_does_not_block_the_other_watchdog(env):
    s, bridge, clock, _ = env
    a = s.take_lease("", "alice")["token"]
    _slow_move(s, bridge, a)
    calls = []
    s.core.stop_all = lambda *x: (calls.append(1), 1 / 0)
    clock.t += svc_mod.LEASE_TIMEOUT_S + 0.1
    s.tick()
    assert calls and s.lease.client is None


def test_heartbeat_never_waits_for_a_slow_stop(env):
    s, bridge, clock, _ = env
    a = s.take_lease("", "alice")["token"]
    _slow_move(s, bridge, a)
    started = threading.Event()

    def slow_stop(*_):
        started.set()
        time.sleep(1.0)
    s.core.stop_all = slow_stop
    clock.t += svc_mod.LEASE_TIMEOUT_S + 0.1
    t = threading.Thread(target=s.tick)
    t.start()
    assert started.wait(2)
    t0 = time.monotonic()
    assert s.heartbeat("whoever") is False                                    # lease already dropped, answers at once
    assert s.take_lease("", "bob")["token"] if not s.core.busy else True
    assert time.monotonic() - t0 < 0.3
    t.join()


# ------------------------------------------------------------ program edits
def test_concurrent_program_edits_are_serialised(env):
    s, _, _, _ = env
    a = s.take_lease("", "alice")["token"]
    errs = []

    def work():
        try:
            for _ in range(10):
                s.command(a, "program_insert", {"type": "wait"})
        except Exception as e:                                                # pragma: no cover
            errs.append(e)
    ts = [threading.Thread(target=work) for _ in range(6)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert not errs
    assert len(s.program.steps) == 60 and s.program_rev == 60 and len(s._undo) == 50   # one snapshot per edit
    for _ in range(3):
        s.command(a, "program_undo", {})
    assert len(s.program.steps) == 57


# --------------------------------------------------------------------- HTTP
@pytest.fixture
def srv(env):
    s, bridge, _, _ = env
    server = Server(s, "127.0.0.1", 0, allow_hosts=["robot.lab.example.org"]).start()
    yield server, s
    server.stop()


def _req(server, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
    h = {"Content-Type": "application/json", **(headers or {})}
    if body is not None and not isinstance(body, bytes):
        body = json.dumps(body).encode()
    c.request(method, path, body=body, headers=h)
    r = c.getresponse()
    data = r.read()
    out = (r.status, dict(r.getheaders()), data)
    c.close()
    return out


def test_host_allowed():
    for ok in ("127.0.0.1:8180", "localhost", "localhost:1", "[::1]:8180", "100.68.1.2", "robot", "robot:8180",
               "swift.tail1234.ts.net", "swift.tail1234.ts.net:443", "SWIFT.ts.net."):
        assert host_allowed(ok), ok
    for bad in ("", "evil.example.com", "evil.example.com:8180", "a.b.c", "127.0.0.1.evil.com", "ts.net.evil.com"):
        assert not host_allowed(bad), bad
    assert host_allowed("robot.lab.example.org:8180", ["robot.lab.example.org"])


def test_http_host_and_origin_checks(srv):
    server, s = srv
    code, _, _ = _req(server, "GET", "/api/info", headers={"Host": "evil.example.com"})
    assert code == 403
    code, _, _ = _req(server, "GET", "/api/info", headers={"Host": "robot.lab.example.org"})
    assert code == 200
    code, _, _ = _req(server, "POST", "/api/stop", {}, headers={"Origin": "http://evil.example.com"})
    assert code == 403
    code, _, _ = _req(server, "POST", "/api/stop", {}, headers={"Origin": "null"})
    assert code == 403
    code, _, _ = _req(server, "POST", "/api/stop", {}, headers={"Origin": f"http://127.0.0.1:{server.port}",
                                                                  "Host": f"127.0.0.1:{server.port}"})
    assert code == 200
    code, _, _ = _req(server, "POST", "/api/stop", {})                          # no Origin (curl): allowed
    assert code == 200


def test_http_frame_headers_and_bad_content_length(srv):
    server, _ = srv
    code, h, _ = _req(server, "GET", "/")
    assert code == 200 and h["X-Frame-Options"] == "DENY" and "frame-ancestors 'none'" in h["Content-Security-Policy"]
    for bad in ("-5", "abc", "1e3"):
        c = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        c.putrequest("POST", "/api/heartbeat")
        c.putheader("Content-Type", "application/json")
        c.putheader("Content-Length", bad)
        c.endheaders()
        r = c.getresponse()
        assert r.status == 400, bad
        c.close()


def test_http_token_flow_and_state_has_no_secret(srv):
    server, s = srv
    code, _, body = _req(server, "POST", "/api/lease", {"name": "al"})
    lease = json.loads(body)["lease"]
    tok = lease["token"]
    assert json.loads(_req(server, "POST", "/api/heartbeat", {"token": tok})[2])["holds"] is True
    assert json.loads(_req(server, "POST", "/api/heartbeat", {"token": lease["client"]})[2])["holds"] is False
    s.tick()
    c = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
    c.request("GET", "/api/events")
    r = c.getresponse()
    assert tok.encode() not in r.fp.readline() + r.fp.readline()
    c.close()


def test_refuse_unspecified_bind_address(env):
    s, _, _, _ = env
    for h in ("0.0.0.0", "::", "[::]", ""):
        with pytest.raises(ValueError, match="refusing"):
            check_bind_host(h)
        with pytest.raises(ValueError):
            Server(s, h, 0)
    check_bind_host("127.0.0.1"); check_bind_host("100.64.0.1"); check_bind_host("localhost")
