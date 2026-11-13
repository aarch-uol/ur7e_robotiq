"""The page served while the robot stack is stopped (SETUP › Setups › Robot › Stop robot stack).

Nothing of the robot runs then: no driver, no MoveIt, no ROS. This small server (no ROS either) answers on the GUI's
addresses with one page and a Start button. Start removes config_dir()/stack_off and exits with 75: the launcher
(ur7e_gui.sh) then starts the robot stack and the full GUI.

With --progress the launcher runs this page again while the driver starts (nothing else answers on the port then):
no Start button, and /api/info says how far the start is, from config_dir()/stack_progress (stage=driver|moveit|slow).
It exits by itself at stage=gui (the GUI takes the port next), so a lost signal cannot leave it holding the port.
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
STACK_PROGRESS = "stack_progress"  # written by the launcher while the stack starts
STAGES = ("driver", "moveit", "slow")
RESTART_STACK = 75
PROXY_HEADERS = ("Tailscale-User-Login", "Tailscale-User-Name", "X-Forwarded-For", "X-Forwarded-Host",
                 "Forwarded", "X-Real-IP")

PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Robot stopped</title>
<style>
:root{--chrome:#20252c;--chrome2:#2c323b;--line:#3a414c;--text:#eef0f3;--muted:#9ba4b0;--accent:#2b5fb8;--accent2:#244f9c;--ok:#7fd8a3}
*{box-sizing:border-box}html,body{margin:0;min-height:100vh}body{font:14px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif;background:#eceef1;color:#1b2026}
header{height:54px;background:var(--chrome);color:var(--text);display:flex;align-items:center;padding:0 18px;border-bottom:1px solid var(--line)}
.brand{font-weight:750;letter-spacing:.2px}.state{margin-left:auto;color:var(--muted);font-size:12px;display:flex;align-items:center;gap:8px}
.dot{width:9px;height:9px;border-radius:50%%;background:#a2aab3}
main{min-height:calc(100vh - 54px);display:grid;place-items:center;padding:16px}.panel{width:min(520px,calc(100vw - 32px));background:white;border:1px solid #d7dbe0;border-radius:6px;padding:34px;text-align:center;box-shadow:0 8px 28px rgba(0,0,0,.08)}
h1{font-size:28px;margin:0 0 8px}.meta{min-height:20px;color:#687280;margin-bottom:26px}
button{font:inherit;font-weight:700;font-size:16px;min-height:48px;padding:0 28px;border-radius:4px;border:1px solid var(--accent);background:var(--accent);color:white;cursor:pointer}
button:hover:not(:disabled){background:var(--accent2)}button:disabled{opacity:.48;cursor:default}
.steps{display:flex;justify-content:center;gap:8px;flex-wrap:wrap;margin-top:22px}.step{padding:5px 9px;border-radius:12px;background:#eceef1;color:#687280;font-size:12px}.step.on{background:#dfe8f7;color:#244f9c;font-weight:700}.step.ok{background:#e4f2e9;color:#1e8a57;font-weight:700}.step.warn{background:#fbecd6;color:#9a5b00;font-weight:700}
#msg{min-height:22px;margin-top:14px;color:#687280;font-size:12px}.hidden{display:none}
</style></head><body>
<header><div class="brand">MoveIt GUI</div><div class="state"><span class="dot"></span><span>Robot stopped</span></div></header>
<main><section class="panel">
<h1>Robot stopped</h1>
<div class="meta">%(why)s</div>
<button id="go" %(disabled)s>▶ Start robot</button>
<div class="steps %(steps_hidden)s" id="steps"><span class="step on" id="st-start">Starting</span><span class="step" id="st-driver">Driver</span><span class="step" id="st-moveit">MoveIt</span><span class="step" id="st-ready">Ready</span></div>
<div id="msg">%(role_note)s</div>
</section></main>
<script>
const go=document.getElementById("go"),msg=document.getElementById("msg"),steps=document.getElementById("steps");
const initiallyStarting=%(starting)s;
function startUI(){
  go.disabled=true;steps.classList.remove("hidden");msg.textContent="Starting the robot stack…";
}
const ORDER=["start","driver","moveit","ready"],TEXT={start:"Starting the robot stack…",driver:"Starting the robot driver…",moveit:"Starting MoveIt and the GUI…"};
function show(stage,warn){
  const at=ORDER.indexOf(stage);
  ORDER.forEach((k,i)=>{const e=document.getElementById("st-"+k);e.className="step"+(i<at?" ok":i===at?(warn?" warn":" on"):"")});
}
async function waitReady(){
  let stage="start",seen=false;const t0=Date.now();
  for(;;){
    await new Promise((r)=>setTimeout(r,1500));
    let warn=false;
    try{
      const r=await fetch("/api/info",{cache:"no-store"});
      if(r.ok){
        show("ready");msg.textContent="Ready";
        sessionStorage.setItem("stackStarted","1");
        setTimeout(()=>location.reload(),250);
        return;
      }
      const j=await r.json().catch(()=>({}));
      if(j.stage==="driver"||j.stage==="moveit"){stage=j.stage;seen=true}
      if(j.stage==="slow"){stage="driver";seen=true;warn=true;msg.textContent="The driver is not up after 90 s: starting the GUI anyway. Log: ~/ur7e_data/logs/"+(j.log||"launch_*.log")}
    }catch(e){if(seen&&stage==="driver")stage="moveit"}      // the progress page stopped: the GUI is starting
    show(stage,warn);
    if(!warn) msg.textContent=TEXT[stage]||"";
    if(Date.now()-t0>240000){show(stage,true);msg.textContent="This takes longer than usual. Check the robot PC: ~/ur7e_data/logs/ (launch_*.log, gui_*.log)"}
  }
}
if(initiallyStarting){startUI();waitReady()}
go.onclick=async()=>{
  startUI();
  try{
    const r=await fetch("/api/stack_start",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
    const j=await r.json();
    msg.textContent=j.msg||"";
    if(!j.ok){go.disabled=false;steps.classList.add("hidden");return}
  }catch(e){msg.textContent="Starting the robot stack…"}
  waitReady();
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
    at = at.rsplit(" ", 1)[-1] if at else ""
    return f"Stopped{' by ' + who if who else ''}{' at ' + at if at else ''}" if (who or at) else ""


def read_progress(cfg: Path = None) -> dict:
    """The launcher's stage=... / log=... lines ({} if none)."""
    try:
        text = (Path(cfg or config_dir()) / STACK_PROGRESS).read_text()
    except OSError:
        return {}
    d = dict(l.split("=", 1) for l in text.splitlines() if "=" in l)
    out = {}
    if d.get("stage") in STAGES:
        out["stage"] = d["stage"]
    if d.get("log", "").startswith("launch_") and "/" not in d["log"]:
        out["log"] = d["log"]
    return out


def make_handler(done: threading.Event, allow_hosts, roles, progress: bool = False):
    from .server import host_allowed, host_port
    starting = threading.Event()
    if progress:
        starting.set()                                     # the launcher is starting the stack: no Start button
    start_lock = threading.Lock()

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
                self._json(503, {"ok": False, "stack": "starting" if starting.is_set() else "stopped",
                                 "starting": starting.is_set(), **(read_progress() if progress else {})})
                return
            viewer = self._role() == "viewer"
            why = "Starting robot stack…" if starting.is_set() else (stop_note() or "Stopped")
            body = PAGE % {"why": html.escape(why), "disabled": "disabled" if viewer else "",
                           "role_note": "Viewer access: an Operator or Admin can start the robot." if viewer else "",
                           "steps_hidden": "" if starting.is_set() else "hidden",
                           "starting": "true" if starting.is_set() else "false"}
            self._send(200, body.encode(), "text/html; charset=utf-8")

        def do_POST(self):
            if not self._ok_host():
                return
            if urlparse(self.path).path != "/api/stack_start":
                self._json(404, {"ok": False, "msg": "not found"})
                return
            if self._role() == "viewer":
                self._json(403, {"ok": False, "msg": "a Viewer cannot start the robot stack"})
                return
            with start_lock:
                if starting.is_set():
                    self._json(200, {"ok": True, "msg": "Robot stack is already starting.", "starting": True})
                    return
                starting.set()
                try:
                    marker().unlink()
                except FileNotFoundError:
                    pass
                except OSError as e:
                    starting.clear()
                    self._json(500, {"ok": False, "msg": f"cannot start the robot stack: {e}"})
                    return
            self._json(200, {"ok": True, "msg": "Starting the robot stack…", "starting": True})
            print("robot stack start asked from the page", flush=True)
            threading.Timer(0.8, done.set).start()
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
    ap.add_argument("--progress", action="store_true", help="the launcher is starting the stack: show how far")
    args, _ = ap.parse_known_args(argv)
    for h in [args.host] + args.also_host:
        try:
            check_bind_host(h)
        except ValueError as e:
            ap.error(str(e))
    done = threading.Event()
    roles = Roles(Path(os.environ.get("MOVEIT_GUI_ROLES") or config_dir() / "roles.yaml"))
    handler = make_handler(done, args.allow_host, roles, progress=args.progress)
    servers = []
    for h in [args.host] + [x for x in args.also_host if x != args.host]:
        s = ThreadingHTTPServer((h, args.port), handler)
        s.daemon_threads = True
        servers.append(s)
        threading.Thread(target=s.serve_forever, kwargs={"poll_interval": 0.3}, daemon=True).start()
    what = "Robot stack starting: progress page on " if args.progress else "Robot stack stopped: start page on "
    print(what + "  ".join(f"http://{h}:{args.port}/" for h in [args.host] + args.also_host), flush=True)
    stop = [0]

    def sig(*_):
        stop[0] = 130
        done.set()
    signal.signal(signal.SIGINT, sig)
    signal.signal(signal.SIGTERM, sig)
    if args.progress:
        def watch():
            while not done.is_set():
                try:
                    if "stage=gui" in (config_dir() / STACK_PROGRESS).read_text().splitlines():
                        done.set()
                except OSError:
                    pass
                done.wait(0.3)
        threading.Thread(target=watch, daemon=True).start()
    done.wait()
    for s in servers:
        s.shutdown()
    if args.progress:
        return 0
    return stop[0] or (RESTART_STACK if not marker().exists() else 0)


def run():
    sys.exit(main())


if __name__ == "__main__":
    run()
