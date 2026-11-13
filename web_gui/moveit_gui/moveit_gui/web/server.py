"""Web front end server: Python standard library only (runs anywhere the ROS bridge runs).

    ros2 run moveit_gui moveit_gui_web [--profile ur7e_lab] [--fake] [--port 8180] [--host 127.0.0.1]

  GET  /                      the app (static files, no build step, no internet needed)
  GET  /api/info              robot profile: joints, modes, waypoints, cameras, policies
  GET  /api/events            Server-Sent Events: "state" 10 Hz, "log" lines, "result" of jobs
  POST /api/ask               ask whoever has control for it (their page asks them: Give / Keep)
  POST /api/hello             {bid, name}: this browser's name changed (the users list)
  POST /api/lease|heartbeat|release   the control lease (see service.py): /api/lease returns a secret token that
                              every later request must carry as "token"; it is never broadcast
  POST /api/stop              STOP GUI MOTION, from anyone
  POST /api/cmd               {client, cmd, args}
  GET  /api/camera.mjpeg?topic=...    one image topic as MJPEG (only while someone watches)
  GET  /api/robot_model       the URDF for the 3D view (robot_model.py)
  GET  /mesh/<package>/<path> a mesh or texture from that package's share directory

Reach it from another PC on the tailnet without opening a port:   tailscale serve --bg 8180
(the server listens on 127.0.0.1 by default; tailscale serve adds HTTPS and the Tailscale-User-* headers).
The Host header must be an IP, localhost, a single-label name or *.ts.net (else --allow-host NAME): DNS rebinding.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import mimetypes
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from ..robot_model import mesh_path
from .service import CommandError, WebService

STATIC = Path(__file__).resolve().parent / "static"
MAX_BODY = 1 << 20
MAX_UPLOAD = 28 << 20          # /api/upload: an STL for SETUP > Parts (20 MB, base64 in JSON)
CAMERA_FPS = 10.0
ERR_EVERY_S = 30.0


def host_allowed(host_header: str, extra=()) -> bool:
    """The Host header names this server by something a DNS-rebinding page cannot make up: an IP literal,
    localhost, a single-label name (LAN / MagicDNS short name), *.ts.net, or a name given with --allow-host."""
    h = (host_header or "").strip().lower()
    if not h:
        return False
    if h.startswith("["):                                     # [::1]:8180
        name = h[1:].split("]", 1)[0]
    else:
        name = h.rsplit(":", 1)[0] if h.count(":") == 1 else h
    name = name.rstrip(".")
    if not name:
        return False
    try:
        ipaddress.ip_address(name)
        return True
    except ValueError:
        pass
    if name == "localhost" or name in {x.lower() for x in extra}:
        return True
    return "." not in name or name.endswith(".ts.net")


def host_port(netloc: str) -> str:
    return (netloc or "").strip().lower().rstrip(".")


def encode_jpeg(rgb, quality: int = 70) -> bytes:
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", rgb[..., ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            return buf.tobytes()
    except ImportError:
        pass
    import io
    from PIL import Image
    out = io.BytesIO()
    Image.fromarray(rgb).save(out, "JPEG", quality=quality)
    return out.getvalue()


def frame_jpeg(f):
    """CameraFrame -> JPEG bytes. JPEG CompressedImage passes through; PNG is re-encoded."""
    if f.compressed is not None:
        if "jpeg" in f.encoding.lower() or "jpg" in f.encoding.lower():
            return f.compressed
        try:
            import cv2
            import numpy as np
            bgr = cv2.imdecode(np.frombuffer(f.compressed, np.uint8), cv2.IMREAD_COLOR)
            return None if bgr is None else encode_jpeg(bgr[..., ::-1])
        except ImportError:
            return None
    return encode_jpeg(f.rgb) if f.rgb is not None else None


class Hub:
    """Ticks the service, and lets SSE clients wait for the next state."""

    def __init__(self, service: WebService, hz: float = 10.0):
        self.service = service
        self.period = 1.0 / hz
        self.cond = threading.Condition()
        self.version = 0
        self.state_json = "{}"
        self.running = True
        self._last_err = -1e9
        self.camera_lock = threading.Lock()
        self.camera_viewers = 0
        self._thread = threading.Thread(target=self._loop, daemon=True, name="web-tick")
        self._thread.start()

    def _loop(self):
        while self.running:
            t0 = time.monotonic()
            try:
                self.service.tick()
                st = self.service.state()
                w = self.service.core.bridge.watched_image()
                if w is not None:
                    f = w.latest()
                    st["camera"] = {"topic": w.topic, "rate": round(w.stats.rate(), 1), "age": w.stats.age(),
                                    "width": f.width if f else 0, "height": f.height if f else 0,
                                    "encoding": f.encoding if f else "", "frame_id": f.frame_id if f else "",
                                    "error": f.error if f else ""}
                js = json.dumps(st, default=_json_default)
            except Exception as e:                           # keep ticking: the lease watchdog lives here
                js = json.dumps({"error": f"{type(e).__name__}: {e}"})
                now = time.monotonic()
                if now - self._last_err > ERR_EVERY_S:
                    self._last_err = now
                    try:
                        self.service.core.bridge.log(f"web: state update failed: {type(e).__name__}: {e}")
                    except Exception:
                        pass
            with self.cond:
                self.state_json = js
                self.version += 1
                self.cond.notify_all()
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))

    def stop(self):
        self.running = False
        with self.cond:
            self.cond.notify_all()


def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    if hasattr(o, "name"):
        return o.name
    return str(o)


def make_handler(hub: Hub, allow_hosts=()):
    service = hub.service

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "moveit_gui_web"

        def log_message(self, fmt, *args):          # quiet: the robot log is what matters
            pass

        # ------------------------------------------------------------ helpers
        def _who(self) -> str:
            return (self.headers.get("Tailscale-User-Name") or self.headers.get("Tailscale-User-Login") or
                    self.client_address[0])

        # Headers a reverse proxy adds when it forwards someone else's request (tailscale serve, code-server, nginx).
        PROXY_HEADERS = ("Tailscale-User-Login", "Tailscale-User-Name", "X-Forwarded-For", "X-Forwarded-Host",
                         "Forwarded", "X-Real-IP")

        def _local(self, need_origin: bool = True) -> bool:
            """The browser runs on this PC. Power on and brake release are only accepted from such a browser.

            * the TCP peer is this machine: a loopback address, or the address the server was reached on
              (a browser on the robot PC opening its own Tailscale IP connects from that same IP);
            * no proxy header: tailscale serve and other proxies connect from 127.0.0.1 on behalf of a remote user;
            * for commands, the page's Origin is that same local address and port, so a proxy that adds no
              headers still shows up through the remote page address it was opened at.
            An SSH port forward (ssh -L) to this PC cannot be told apart; that needs a login on this PC anyway."""
            if any(self.headers.get(h) for h in self.PROXY_HEADERS):
                return False
            me = self.connection.getsockname()[0]

            def ours(host: str) -> bool:
                host = (host or "").strip("[]")
                if host == "localhost" or host == me:
                    return True
                try:
                    return ipaddress.ip_address(host).is_loopback
                except ValueError:
                    return False

            if not ours(self.client_address[0]):
                return False
            if not need_origin:
                return True
            o = urlparse(self.headers.get("Origin") or "")
            port = o.port or {"http": 80, "https": 443}.get(o.scheme)
            return o.scheme == "http" and ours(o.hostname) and port == self.server.server_address[1]

        def _key(self) -> str:
            """Who this is for roles.py: the Tailscale login when tailscale serve forwarded the request (it comes from
            this PC then; the header from anywhere else could be typed by anyone), else the IP address."""
            login = (self.headers.get("Tailscale-User-Login") or "").strip()
            try:
                loop = ipaddress.ip_address(self.client_address[0].strip("[]")).is_loopback
            except ValueError:
                loop = False
            return login if login and loop else self.client_address[0]

        def _role(self, need_origin: bool = True) -> str:
            return service.roles.role(self._key(), self.client_address[0], self._local(need_origin))

        def _host_ok(self) -> bool:
            """Refuse a Host we are not known by (DNS rebinding) and, on POST, a page from another origin."""
            if not host_allowed(self.headers.get("Host"), allow_hosts):
                self._json(403, {"ok": False, "msg": "unknown Host header (use the server's IP or name; "
                                                     "for a custom DNS name start with --allow-host NAME)"})
                return False
            origin = self.headers.get("Origin")
            if self.command == "POST" and origin is not None and origin != "null":
                if host_port(urlparse(origin).netloc) != host_port(self.headers.get("Host")):
                    self._json(403, {"ok": False, "msg": "cross-origin request refused"})
                    return False
            elif self.command == "POST" and origin == "null":
                self._json(403, {"ok": False, "msg": "cross-origin request refused"})
                return False
            return True

        def _json(self, code: int, obj):
            body = json.dumps(obj, default=_json_default).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _body(self, limit: int = MAX_BODY) -> dict:
            # JSON only: a cross-site HTML form cannot send it without a CORS preflight, which is never granted.
            if not (self.headers.get("Content-Type") or "").startswith("application/json"):
                raise CommandError("Content-Type must be application/json")
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = -1
            if n < 0:
                raise ValueError("bad Content-Length")
            if n > limit:
                raise CommandError("request too large")
            data = json.loads(self.rfile.read(n) or b"{}")
            if not isinstance(data, dict):
                raise CommandError("body must be a JSON object")
            return data

        # --------------------------------------------------------------- GET
        def do_GET(self):
            url = urlparse(self.path)
            path = url.path
            if not self._host_ok():
                return
            try:
                if path in ("/", "/index.html"):
                    return self._static("index.html")
                if path.startswith("/static/"):
                    return self._static(path[len("/static/"):])
                if path == "/api/info":
                    return self._json(200, {**service.info(), "you": self._who(),
                                            "local": self._local(need_origin=False),
                                            "role": self._role(need_origin=False), "key": self._key()})
                if path == "/api/events":
                    q = parse_qs(url.query)
                    return self._events(q.get("bid", [""])[0], q.get("name", [""])[0])
                if path == "/api/camera.mjpeg":
                    return self._camera(parse_qs(url.query).get("topic", [""])[0])
                if path == "/api/robot_model":
                    return self._json(200, service.robot_model())
                if path.startswith("/mesh/"):
                    return self._mesh(unquote(path[len("/mesh/"):]))
                self._json(404, {"ok": False, "msg": "not found"})
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _static(self, name: str):
            p = (STATIC / name).resolve()
            if STATIC not in p.parents or not p.is_file():
                return self._json(404, {"ok": False, "msg": "not found"})
            body = p.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", (mimetypes.guess_type(p.name)[0] or "application/octet-stream") +
                             ("; charset=utf-8" if p.suffix in (".html", ".js", ".css") else ""))
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            if p.suffix in (".html", ".htm"):
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Content-Security-Policy", "frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def _mesh(self, rest: str):
            """/mesh/<package>/<path>: a mesh or texture from that package's share directory, nothing else.
            Relative texture references inside a .dae resolve to the same route."""
            pkg, _, rel = rest.partition("/")
            if pkg == "_parts":                                   # a mesh uploaded in SETUP > Parts (or its hull)
                p = service.part_mesh_file(rel)
            else:
                p = mesh_path(pkg, rel)
            if p is None:
                return self._json(404, {"ok": False, "msg": "not found"})
            body = p.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(p.name)[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache" if pkg == "_parts" else "max-age=3600")
            self.end_headers()
            self.wfile.write(body)

        def _events(self, bid: str = "", name: str = ""):
            uid = service.users.open(bid, name, self._who(), self._key(),
                                     self._role(need_origin=False))          # one user per browser (users.py)
            try:
                self._events_loop(uid)
            finally:
                if uid:
                    service.users.close(bid)

        def _events_loop(self, uid):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            self.close_connection = True
            seen_version, log_seq = -1, 0
            if uid:                                           # this page's own public id (to find itself in "users")
                self.wfile.write(f"event: hello\ndata: {json.dumps({'id': uid})}\n\n".encode())
                self.wfile.flush()
            res_id = max((r["id"] for r in service.results), default=0)
            if service.logs:                                  # last lines of history on connect
                log_seq = max(0, service.logs[-1][0] - 200)
            while hub.running:
                with hub.cond:
                    hub.cond.wait_for(lambda: hub.version != seen_version or not hub.running, timeout=5.0)
                    seen_version, js = hub.version, hub.state_json
                out = [f"event: state\ndata: {js}\n\n"]
                logs = [[seq, line] for seq, line in list(service.logs) if seq > log_seq]
                if logs:
                    log_seq = logs[-1][0]                      # the page dedupes by these sequence numbers
                    out.append(f"event: log\ndata: {json.dumps(logs)}\n\n")
                for r in [r for r in list(service.results) if r["id"] > res_id]:
                    res_id = r["id"]
                    out.append(f"event: result\ndata: {json.dumps(r)}\n\n")
                self.wfile.write("".join(out).encode())
                self.wfile.flush()

        def _camera(self, topic: str):
            bridge = service.core.bridge
            with hub.camera_lock:
                ok, msg = bridge.watch_image(topic or None)
                if ok:
                    hub.camera_viewers += 1
            if not ok:
                return self._json(404, {"ok": False, "msg": msg})
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.close_connection = True
            last = None
            try:
                while hub.running:
                    w = bridge.watched_image()
                    if w is None or w.topic != topic:
                        break                                    # another viewer switched the topic
                    f = w.latest()
                    if f is not None and f.stamp != last and not f.error:
                        last = f.stamp
                        jpg = frame_jpeg(f)
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                             str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                            self.wfile.flush()
                    time.sleep(1.0 / CAMERA_FPS)
            finally:
                with hub.camera_lock:
                    hub.camera_viewers -= 1
                    w = bridge.watched_image()
                    if hub.camera_viewers <= 0 and w is not None and w.topic == topic:
                        bridge.watch_image(None)               # no image traffic while nobody looks

        # -------------------------------------------------------------- POST
        def do_POST(self):
            path = urlparse(self.path).path
            if not self._host_ok():
                return
            try:
                body = self._body(MAX_UPLOAD if path == "/api/upload" else MAX_BODY)
                client = str(body.get("token") or body.get("client") or "")        # the lease secret; STOP and /api/lease need none
                if path == "/api/stop":
                    return self._json(200, service.stop(self._who(), str(body.get("bid") or ""), self._key(), self._role(need_origin=False)))
                if path == "/api/lease":
                    op = " ".join(str(body.get("name") or "").split())[:24]   # operator name typed in the browser
                    bid = str(body.get("bid") or "")
                    if op:
                        service.users.rename(bid, op)
                    op = op or (service.users.label(bid) if service.users.valid(bid) else "")
                    return self._json(200, {"ok": True, "lease": service.take_lease(
                        "", f"{op} @ {self._who()}" if op else self._who(), presented=client, bid=bid,
                        role=self._role(), key=self._key())})
                if path == "/api/ask":
                    bid = str(body.get("bid") or "")
                    return self._json(200, service.ask_control(bid, role=self._role(), key=self._key()))
                if path == "/api/hello":                          # this browser's name changed
                    service.users.rename(str(body.get("bid") or ""), str(body.get("name") or ""))
                    return self._json(200, {"ok": True})
                if path == "/api/heartbeat":
                    return self._json(200, {"ok": True, "holds": service.heartbeat(client)})
                if path == "/api/release":
                    service.release(client)
                    return self._json(200, {"ok": True})
                if path == "/api/cmd":
                    return self._json(200, service.command(client, str(body.get("cmd", "")), body.get("args") or {},
                                                                   local=self._local(), role=self._role()))
                if path == "/api/upload":
                    return self._json(200, service.command(client, "parts_mesh_upload", {
                        "filename": str(body.get("filename", "")), "data_b64": str(body.get("data_b64", ""))},
                        local=self._local(), role=self._role()))
                self._json(404, {"ok": False, "msg": "not found"})
            except CommandError as e:
                self._json(409, {"ok": False, "msg": str(e)})
            except (TypeError, ValueError, KeyError) as e:
                self._json(400, {"ok": False, "msg": f"bad request: {e}"})
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                service.core.bridge.log(f"web: {type(e).__name__}: {e}")
                self._json(500, {"ok": False, "msg": f"{type(e).__name__}: {e}"})

    return Handler


def check_bind_host(host: str):
    """Never listen on every interface: this server can move a robot."""
    try:
        unspecified = ipaddress.ip_address(host.strip("[]")).is_unspecified
    except ValueError:
        unspecified = host == ""
    if unspecified:
        raise ValueError(f"refusing to listen on {host or '(all)'}: that exposes the robot to every network. "
                         "Use 127.0.0.1 with `tailscale serve`, or this PC's Tailscale IP.")


class Server:
    def __init__(self, service: WebService, host: str = "127.0.0.1", port: int = 8180, allow_hosts=()):
        check_bind_host(host)
        self.hub = Hub(service)
        self.httpd = ThreadingHTTPServer((host, port), make_handler(self.hub, tuple(allow_hosts)))
        self.httpd.daemon_threads = True

    @property
    def port(self) -> int:
        return self.httpd.server_address[1]

    def serve_forever(self):
        self.httpd.serve_forever(poll_interval=0.2)

    def start(self) -> "Server":
        threading.Thread(target=self.serve_forever, daemon=True, name="web-http").start()
        return self

    def stop(self):
        self.hub.stop()
        self.httpd.shutdown()
        self.httpd.server_close()
        self.hub.service.shutdown()


def main(argv=None):
    from ..app import data_dir, prepare_dirs, resolve_profile
    from ..core import RobotCore
    from ..profile import RobotProfile
    root = data_dir()
    ap = argparse.ArgumentParser(description="MoveIt GUI web front end")
    ap.add_argument("--profile", default="ur7e_lab")
    ap.add_argument("--profiles-dir", default=str(root / "profiles"))
    ap.add_argument("--programs-dir", default=str(Path.home() / "moveit_gui_programs"))
    ap.add_argument("--policies-dir", default=str(Path.home() / "moveit_gui_policies"))
    ap.add_argument("--fake", action="store_true", help="no ROS: simulated robot")
    ap.add_argument("--urdf", help="--fake only: a URDF file for the 3D view")
    ap.add_argument("--host", default="127.0.0.1",
                    help="127.0.0.1 + `tailscale serve` (recommended), or this PC's Tailscale IP")
    ap.add_argument("--allow-host", action="append", default=[], metavar="NAME",
                    help="also accept this DNS name in the Host header (repeatable), e.g. robot.lab.example.org")
    ap.add_argument("--port", type=int, default=8180)   # 8080 code-server, 8090 a tunnel client on the lab PC
    args = ap.parse_args(argv)
    try:
        check_bind_host(args.host)
    except ValueError as e:
        ap.error(str(e))
    profiles_dir, programs_dir, policies_dir = prepare_dirs(args, root)
    profile = RobotProfile.load(resolve_profile(args.profile, profiles_dir))
    if args.fake:
        from ..fake_bridge import FakeBridge
        bridge = FakeBridge(profile)
        if args.urdf:
            bridge.fake_robot_description = Path(args.urdf).read_text()
    else:
        from ..ros_bridge import RosBridge
        bridge = RosBridge(profile)
    core = RobotCore(bridge, profile, profiles_dir, programs_dir, policies_dir, fake=args.fake)
    service = WebService(core)
    server = Server(service, args.host, args.port, args.allow_host)
    print(f"MoveIt GUI web: http://{args.host}:{server.port}/  ({profile.display_name}"
          f"{', FAKE robot' if args.fake else ''})", flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    service.on_restart = stop.set           # SETUP > Parts / Setups: the launcher restarts on exit code 75 / 76
    server.start()
    stop.wait()
    core.stop_all()
    core.recorder.stop_and_wait("GUI closed")      # finish the bag file; never leave `ros2 bag record` behind
    service.drivers_stop_all()                     # camera drivers / pipelines (kept on a GUI-only restart)
    server.stop()
    bridge.shutdown()
    return service.restart_code or 0


if __name__ == "__main__":
    sys.exit(main())
