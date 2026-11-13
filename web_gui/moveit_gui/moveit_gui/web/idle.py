"""The page served while the robot stack is stopped (SETUP › Setups › Robot › Stop robot stack).

Nothing of the robot runs then: no driver, no MoveIt, no ROS. This small server (no ROS either) answers on the GUI's
addresses with one page and a Start button. Start removes config_dir()/stack_off and exits with 75: the launcher
(ur7e_gui.sh) then starts the robot stack and the full GUI.
"""
from __future__ import annotations

import argparse
import html
import ipaddress
import json
import os
import signal
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from ..calibration import config_dir

STACK_OFF = "stack_off"          # config_dir()/stack_off: the launcher starts only this page
RESTART_STACK = 75
PROXY_HEADERS = ("Tailscale-User-Login", "Tailscale-User-Name", "X-Forwarded-For", "X-Forwarded-Host",
                 "Forwarded", "X-Real-IP")

PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Robot stack stopped</title>
<style>
body{font:15px system-ui,sans-serif;background:#111418;color:#e6e8eb;display:flex;min-height:100vh;margin:0;
align-items:center;justify-content:center}
main{max-width:30em;padding:2em;background:#1b1f25;border-radius:10px;border:1px solid #2c323a}
h1{font-size:1.3em;margin:0 0 .6em}p{color:#aab2bd;line-height:1.45}
button{font:inherit;padding:.6em 1.4em;border-radius:6px;border:0;background:#2f7de1;color:#fff;cursor:pointer}
button:disabled{opacity:.5;cursor:default}#msg{min-height:1.4em;margin-top:1em}
</style></head><body><main>
<h1>Robot stack stopped</h1>
<p>Nothing of the robot runs on this PC now: no driver, MoveIt or ROS. %(why)s</p>
<p>Start brings the robot stack and the GUI back (about a minute). Then play the External Control program on the
pendant as usual.</p>
<button id="go">Start robot stack</button><div id="msg"></div>
</main><script>
const go = document.getElementById("go"), msg = document.getElementById("msg");
async function wait() {
  for (;;) {
    await new Promise((r) => setTimeout(r, 3000));
    try { const r = await fetch("/api/info", { cache: "no-store" }); if (r.ok) { location.reload(); return; } } catch (e) {}
  }
}
go.onclick = async () => {
  go.disabled = true; msg.textContent = "Starting...";
  try {
    const r = await fetch("/api/stack_start", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    const j = await r.json();
    msg.textContent = j.msg || "";
    if (!j.ok) { go.disabled = false; return; }
  } catch (e) { msg.textContent = "Starting: this page reloads when the GUI is up."; }
  wait();
};
</script></body></html>"""


def marker(cfg: Path = None) -> Path:
    return Path(cfg or config_dir()) / STACK_OFF


def stop_note(cfg: Path = None) -> str:
    try:
        d = json.loads(marker(cfg).read_text() or "{}")
    except (OSError, ValueError):
        return ""
    who, at = str(d.get("by") or ""), str(d.get("at") or "")
    return f"Stopped{' by ' + who if who else ''}{' at ' + at if at else ''}." if (who or at) else ""


def make_handler(done: threading.Event, allow_hosts, roles):
    from .server import host_allowed, host_port

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def _ok_host(self) -> bool:
            if not host_allowed(self.headers.get("Host"), allow_hosts):
                self._json(403, {"ok": False, "msg": "unknown Host header"})
                return False
            o = self.headers.get("Origin")
            if self.command == "POST" and (o == "null" or (o is not None and
                                           host_port(urlparse(o).netloc) != host_port(self.headers.get("Host")))):
                self._json(403, {"ok": False, "msg": "cross-origin request refused"})
                return False
            return True

        def _role(self) -> str:
            ip = self.client_address[0]
            try:
                loop = ipaddress.ip_address(ip.strip("[]")).is_loopback
            except ValueError:
                loop = False
            login = (self.headers.get("Tailscale-User-Login") or "").strip()
            proxied = any(self.headers.get(h) for h in PROXY_HEADERS)
            local = not proxied and (loop or ip == self.connection.getsockname()[0])
            return roles.role(login if login and loop else None, ip, local) if roles else "admin"

        def do_GET(self):
            if not self._ok_host():
                return
            if urlparse(self.path).path == "/api/info":            # the full GUI answers 200 here; this one 503
                self._json(503, {"ok": False, "stack": "stopped"})
                return
            self._send(200, (PAGE % {"why": html.escape(stop_note())}).encode(), "text/html; charset=utf-8")

        def do_POST(self):
            if not self._ok_host():
                return
            if urlparse(self.path).path != "/api/stack_start":
                self._json(404, {"ok": False, "msg": "not found"})
                return
            if self._role() == "viewer":
                self._json(403, {"ok": False, "msg": "a Viewer cannot start the robot stack"})
                return
            try:
                marker().unlink()
            except FileNotFoundError:
                pass
            self._json(200, {"ok": True, "msg": "Starting the robot stack: this page reloads when the GUI is up "
                                                "(about a minute)."})
            print("robot stack start asked from the page", flush=True)
            threading.Timer(0.5, done.set).start()
    return H


def main(argv=None) -> int:
    from .roles import Roles
    from .server import check_bind_host
    ap = argparse.ArgumentParser(description="the page shown while the robot stack is stopped")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--also-host", action="append", default=[])
    ap.add_argument("--allow-host", action="append", default=[])
    ap.add_argument("--port", type=int, default=8180)
    ap.add_argument("--fake", action="store_true")                    # ignored: same arguments as the GUI
    args, _ = ap.parse_known_args(argv)
    for h in [args.host] + args.also_host:
        try:
            check_bind_host(h)
        except ValueError as e:
            ap.error(str(e))
    done = threading.Event()
    roles = Roles(Path(os.environ.get("MOVEIT_GUI_ROLES") or config_dir() / "roles.yaml"))
    handler = make_handler(done, args.allow_host, roles)
    servers = []
    for h in [args.host] + [x for x in args.also_host if x != args.host]:
        s = ThreadingHTTPServer((h, args.port), handler)
        s.daemon_threads = True
        servers.append(s)
        threading.Thread(target=s.serve_forever, kwargs={"poll_interval": 0.3}, daemon=True).start()
    print("Robot stack stopped: start page on " + "  ".join(f"http://{h}:{args.port}/" for h in
                                                            [args.host] + args.also_host), flush=True)
    stop = [0]

    def sig(*_):
        stop[0] = 130
        done.set()
    signal.signal(signal.SIGINT, sig)
    signal.signal(signal.SIGTERM, sig)
    done.wait()
    for s in servers:
        s.shutdown()
    return stop[0] or (RESTART_STACK if not marker().exists() else 0)


def run():
    sys.exit(main())


if __name__ == "__main__":
    run()
