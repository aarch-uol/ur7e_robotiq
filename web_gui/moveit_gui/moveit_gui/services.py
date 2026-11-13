"""Drivers and pipelines: ROS processes the GUI starts and stops (SETUP > Drivers & cameras).

A *service* is one `ros2 launch` / `ros2 run` process: a camera driver now, a vision pipeline later. It is
defined in config_dir()/services/<profile>.yaml as a type (realsense, usb_cam, zed, launch, run) plus a few
fields, and it says which topics it makes (its outputs). It counts as ready when every required output has a
publisher, so a program can wait for a pipeline before it reads its result.

Nothing here moves the robot. Each service runs as a child in its own process group (no shell): Stop sends SIGINT
(ROS shuts down cleanly), then SIGTERM, then SIGKILL. The output goes to a log file. A pid file under run_dir lets
a restarted GUI adopt the drivers that are still running (a GUI-only restart leaves the cameras on).
"""
from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import yaml

NAME = re.compile(r"^[a-z][a-z0-9_]{0,39}$")
KINDS = {"camera": "Camera", "driver": "Driver", "pipeline": "Vision pipeline"}
START_GRACE_S = 25.0          # a started service without data for this long: "no data" (still running)
STOP_STEPS = ((signal.SIGINT, 8.0), (signal.SIGTERM, 4.0), (signal.SIGKILL, 2.0))
CHECK_S = 1.0                 # how often the outputs' publishers are counted
KEEP_LOGS = 5                 # log files kept per service
TOPIC = re.compile(r"^/[A-Za-z0-9_/~{}]*$")


class ServiceError(Exception):
    pass


# ====================================================================== types
@dataclass
class Field:
    key: str
    label: str
    default: object = ""
    hint: str = ""
    choices: tuple = ()           # ((value, label), ...) -> a select; bool default -> a checkbox


@dataclass
class Type:
    label: str
    kind: str                     # camera | driver | pipeline (launch/run: the user picks)
    pkg: str                      # the ROS package that must be installed; "" = the user's package field
    install: str                  # how to get it, shown when it is missing
    fields: List[Field]
    help: str = ""


def _res(*wh):
    return tuple((f"{w}x{h}", f"{w} × {h}") for w, h in wh)


def _fps(*v):
    return tuple((str(f), f"{f} fps") for f in v)


# RealSense: the modes the D405 lists (depth_module.depth_profile, checked on the Jetson 2026-11); a D435 / D455
# colour sensor tops out at 60 fps. 1280 × 720 runs at 30 fps at most.
RS_RES = _res((424, 240), (480, 270), (640, 360), (640, 480), (848, 480), (1280, 720))
RS_FPS = _fps(5, 15, 30, 60, 90)
USB_RES = _res((320, 240), (640, 480), (800, 600), (1024, 768), (1280, 720), (1920, 1080))
USB_FPS = _fps(5, 10, 15, 20, 30, 60)
# ZED: grab resolution and its frame rates (zed-ros2-wrapper config/<model>.yaml)
ZED_RES = (("AUTO", "camera default"), ("HD1200", "1920 × 1200"), ("HD1080", "1920 × 1080"), ("SVGA", "960 × 600"),
           ("HD2K", "2208 × 1242"), ("HD720", "1280 × 720"), ("VGA", "672 × 376"))
ZED_FPS = _fps(15, 30, 60, 100, 120)
ZED_GMSL = ("zedx", "zedxm", "zedxnano", "zedxonegs")          # ZED X family
ZED_MODES = {   # resolution -> frame rates
    "gmsl": {"AUTO": (15, 30, 60), "HD1200": (15, 30, 60), "HD1080": (15, 30, 60), "SVGA": (15, 30, 60, 120)},
    "usb": {"AUTO": (15, 30), "HD2K": (15,), "HD1080": (15, 30), "HD720": (15, 30, 60), "VGA": (15, 30, 60, 100)},
}
ZED_PUB = (("1", "full size"), ("2", "half size (less to send)"), ("4", "quarter size"))
# pictures published per second: the camera still grabs at its frame rate (depth quality), but every subscriber
# (the GUI's camera window, recorders) gets only this many. At 60 fps a full-size ZED picture stream alone kept
# most of a Jetson core busy copying.
ZED_PUB_FPS = (("5", "5"), ("10", "10"), ("15", "15"), ("30", "30"), ("0", "as grabbed"))
TYPES: Dict[str, Type] = {
    "realsense": Type("Intel RealSense", "camera", "realsense2_camera",
                      "add ros-jazzy-realsense2-camera to the robot image (docker/Dockerfile) and rebuild; "
                      "the container also needs the USB devices (see docs/WEB.md)",
                      [Field("serial", "Serial number", "", "empty = the only RealSense plugged in"),
                       Field("resolution", "Resolution", "640x480", "colour and depth use the same size", RS_RES),
                       Field("fps", "Frame rate", "30", "1280 × 720: 30 fps at most; D435 / D455 colour: 60",
                             RS_FPS),
                       Field("depth", "Depth", True), Field("align", "Depth lined up with colour", True),
                       Field("namespace", "Namespace", "camera", "topics are /<namespace>/<name>/…")],
                      "colour + depth camera on USB"),
    "usb_cam": Type("USB / webcam (V4L2)", "camera", "usb_cam",
                    "add ros-jazzy-usb-cam to the robot image (docker/Dockerfile) and rebuild; the container also "
                    "needs the /dev/video* devices",
                    [Field("device", "Device", "/dev/video0"),
                     Field("resolution", "Resolution", "640x480", "one your camera has (v4l2-ctl --list-formats-ext)",
                           USB_RES),
                     Field("fps", "Frame rate", "30", choices=USB_FPS),
                     Field("format", "Pixel format", "mjpeg2rgb",
                           choices=(("mjpeg2rgb", "MJPEG (most webcams)"), ("yuyv2rgb", "YUYV")))],
                    "any webcam or USB camera"),
    "zed": Type("Stereolabs ZED", "camera", "zed_wrapper",
                "the ZED container is not running: on a Jetson with the ZED SDK, build its image once "
                "(docker/zed/build.sh), then start with ur7e_gui.sh (it runs the container next to the robot stack)",
                [Field("model", "Model", "zedxnano", choices=tuple((m, l) for m, l in (
                    ("zedxnano", "ZED X Nano"), ("zedx", "ZED X"), ("zedxm", "ZED X Mini"), ("zedxonegs", "ZED X One GS"),
                    ("zedxone4k", "ZED X One 4K"), ("zed2i", "ZED 2i"), ("zed2", "ZED 2"), ("zedm", "ZED Mini"),
                    ("zed", "ZED")))),
                 Field("serial", "Serial number", "", "empty = the only ZED connected"),
                 Field("resolution", "Resolution", "AUTO", "ZED X: 1920 × 1200, 1920 × 1080 or 960 × 600", ZED_RES),
                 Field("fps", "Frame rate", "30", "960 × 600 up to 120 fps; 1920 × 1200 up to 60", ZED_FPS),
                 Field("publish", "Pictures sent", "1", "smaller = less load on the network and the GUI", ZED_PUB),
                 Field("pub_fps", "Pictures per second", "15", "sent to the GUI and others; the camera still runs "
                       "at its frame rate", ZED_PUB_FPS),
                 Field("depth", "Depth", "NEURAL_LIGHT", choices=(
                     ("NEURAL_LIGHT", "Neural light (fast)"), ("NEURAL", "Neural"), ("NEURAL_PLUS", "Neural plus (best, slow)"),
                     ("NONE", "Off (pictures only)")))],
                "stereo depth camera (Jetson, ZED X over GMSL)"),
    "launch": Type("ROS launch file", "", "", "build or install that package in the workspace",
                   [Field("package", "Package", ""), Field("file", "Launch file", "", "e.g. pipeline.launch.py"),
                    Field("args", "Arguments", "", "e.g. camera:=/camera/camera  rate:=10"),
                    Field("outputs", "Output topics", "", "space separated; ready when all publish")],
                   "your own launch file"),
    "run": Type("ROS node", "", "", "build or install that package in the workspace",
                [Field("package", "Package", ""), Field("exe", "Executable", ""),
                 Field("args", "Arguments", "", "e.g. --ros-args -p rate:=10 -r image:=/camera/camera/color/image_raw"),
                 Field("outputs", "Output topics", "", "space separated; ready when all publish")],
                "one node of your own"),
}


def _params(type_: str, params: dict) -> dict:
    t = TYPES[type_]
    params = dict(params or {})
    if type_ in ("realsense", "usb_cam") and "size" in params and "resolution" not in params:   # saved before 2026-11
        w, h, fps = (str(params.pop("size")).split(",") + ["", "", ""])[:3]
        params.update(resolution=f"{w}x{h}", fps=fps)
    out = {}
    for f in t.fields:
        v = (params or {}).get(f.key, f.default)
        if isinstance(f.default, bool):
            v = bool(v) if not isinstance(v, str) else v.lower() in ("1", "true", "yes", "on")
        else:
            v = str("" if v is None else v).strip()
            if "\n" in v or len(v) > 400:
                raise ServiceError(f"{f.label}: one line, at most 400 characters")
            if f.choices and v not in [c[0] for c in f.choices]:
                raise ServiceError(f"{f.label}: one of {', '.join(c[0] for c in f.choices)}")
        out[f.key] = v
    _check_mode(type_, out)
    return out


def _check_mode(type_: str, p: dict):
    """Refuse a resolution and frame rate the camera does not have (where that is known)."""
    fps = int(p.get("fps") or 0)
    if type_ == "realsense" and p["resolution"] == "1280x720" and fps > 30:
        raise ServiceError("1280 × 720 runs at 5, 15 or 30 fps")
    if type_ == "zed":
        fam = "gmsl" if p["model"] in ZED_GMSL else "usb" if p["model"] in ("zed", "zedm", "zed2", "zed2i") else None
        modes = ZED_MODES.get(fam, {})
        if fam and p["resolution"] not in modes:
            raise ServiceError(f"{p['model']}: resolution one of {', '.join(modes)}")
        if fam and fps not in modes[p["resolution"]]:
            raise ServiceError(f"{p['model']} at {p['resolution']}: {', '.join(map(str, modes[p['resolution']]))} fps")


def _split(s: str, what: str) -> List[str]:
    try:
        return shlex.split(s)
    except ValueError as e:
        raise ServiceError(f"{what}: {e}") from None


@dataclass
class Service:
    name: str
    type: str
    params: dict = field(default_factory=dict)
    kind: str = ""
    needs: List[str] = field(default_factory=list)
    autostart: bool = False
    note: str = ""
    label: str = ""               # what the GUI shows, e.g. "D405"; the name stays the ROS namespace

    @property
    def title(self) -> str:
        return self.label or self.name

    @property
    def pkg(self) -> str:
        return TYPES[self.type].pkg or self.params.get("package", "")

    def argv(self) -> List[str]:
        p, n = self.params, self.name
        if self.type == "realsense":
            w, h = p["resolution"].split("x")
            prof = f"{w},{h},{p['fps']}"
            a = ["ros2", "launch", "realsense2_camera", "rs_launch.py", f"camera_namespace:={p['namespace']}",
                 f"camera_name:={n}", f"rgb_camera.color_profile:={prof}", f"enable_depth:={str(p['depth']).lower()}",
                 f"align_depth.enable:={str(p['depth'] and p['align']).lower()}"]
            if p["depth"]:
                # D405 (and D4xx without an RGB module): the colour comes from the depth module, at the same size
                a += [f"depth_module.depth_profile:={prof}", f"depth_module.color_profile:={prof}"]
            if p["serial"]:
                a.append(f"serial_no:='{p['serial']}'")          # quoted: rs_launch reads it as a string
            return a
        if self.type == "usb_cam":
            (w, h), fps = p["resolution"].split("x"), p["fps"]
            return ["ros2", "run", "usb_cam", "usb_cam_node_exe", "--ros-args", "-r", f"__ns:=/{n}",
                    "-p", f"video_device:={p['device']}", "-p", f"image_width:={w}", "-p", f"image_height:={h}",
                    "-p", f"framerate:={float(fps)}", "-p", f"pixel_format:={p['format']}",
                    "-p", f"camera_name:={n}", "-p", f"frame_id:={n}_optical_frame"]
        if self.type == "zed":
            # no positional tracking and no odom TF: the camera sits on the robot or the cell, and its place comes
            # from Camera calibration (a second parent for its camera_link would break TF)
            a = ["ros2", "launch", "zed_wrapper", "zed_camera.launch.py", f"camera_model:={p['model']}",
                 f"camera_name:={n}", "publish_tf:=false", "publish_map_tf:=false",
                 "param_overrides:=" + ";".join(f"{k}:={v}" for k, v in self._zed_overrides().items())]
            if p["serial"]:
                a.append(f"serial_number:={p['serial']}")
            return a
        if self.type == "launch":
            return ["ros2", "launch", p["package"], p["file"], *_split(p["args"], "Arguments")]
        return ["ros2", "run", p["package"], p["exe"], *_split(p["args"], "Arguments")]

    def _zed_overrides(self) -> dict:
        p = self.params
        o = {"pos_tracking.pos_tracking_enabled": "false", "depth.depth_mode": p["depth"],
             "general.grab_frame_rate": p["fps"]}
        if p["resolution"] != "AUTO":
            o["general.grab_resolution"] = p["resolution"]
        pf = str(p.get("pub_fps") or "15")
        if pf != "0" and float(pf) < float(p["fps"]):
            o["general.pub_frame_rate"] = f"{float(pf):.1f}"
        if p["publish"] != "1":
            o.update({"general.pub_resolution": "CUSTOM", "general.pub_downscale_factor": f"{float(p['publish']):.1f}"})
        return o

    def outputs(self) -> List[dict]:
        """[{topic, label, image, required}]: what this service publishes (required ones decide 'ready')."""
        p, n = self.params, self.name

        def o(topic, label, image=False, required=True):
            return {"topic": topic, "label": label, "image": image, "required": required}
        if self.type == "realsense":
            b = f"/{p['namespace']}/{n}" if p["namespace"] else f"/{n}"
            out = [o(f"{b}/color/image_raw", "colour", True), o(f"{b}/color/camera_info", "colour intrinsics")]
            if p["depth"]:
                out.append(o(f"{b}/depth/image_rect_raw", "depth", True))
                if p["align"]:
                    out.append(o(f"{b}/aligned_depth_to_color/image_raw", "depth lined up with colour", True, False))
            return out
        if self.type == "usb_cam":
            return [o(f"/{n}/image_raw", "picture", True), o(f"/{n}/camera_info", "intrinsics")]
        if self.type == "zed":                                    # zed-ros2-wrapper 5.5 names [Jetson, 2026-11]
            b = f"/{n}/zed_node"
            out = [o(f"{b}/rgb/color/rect/image", "left, rectified", True),
                   o(f"{b}/rgb/color/rect/camera_info", "intrinsics")]
            if p["depth"] != "NONE":
                out.append(o(f"{b}/depth/depth_registered", "depth", True))
            return out
        return [o(t, t.rsplit("/", 1)[-1], "image" in t) for t in p.get("outputs", "").split()]

    def to_yaml(self) -> dict:
        d = {"name": self.name, "type": self.type, "params": dict(self.params)}
        if self.type in ("launch", "run"):
            d["kind"] = self.kind
        for k in ("needs", "autostart", "note", "label"):
            if getattr(self, k):
                d[k] = getattr(self, k)
        return d


def make_service(d: dict, known: Optional[set] = None) -> Service:
    """Check and normalise a definition (from the page or the file)."""
    name, type_ = str(d.get("name", "")).strip(), str(d.get("type", ""))
    if not NAME.match(name):
        raise ServiceError("a name is lower_snake_case: a-z, 0-9 and _, starting with a letter (it becomes the "
                           "ROS namespace)")
    if type_ not in TYPES:
        raise ServiceError(f"unknown type '{type_}': one of {', '.join(TYPES)}")
    params = _params(type_, d.get("params") or {})
    kind = TYPES[type_].kind or str(d.get("kind") or "pipeline")
    if kind not in KINDS:
        raise ServiceError(f"kind is one of {', '.join(KINDS)}")
    s = Service(name, type_, params, kind, [str(x) for x in d.get("needs") or [] if str(x) != name],
                bool(d.get("autostart")), str(d.get("note") or "")[:200], " ".join(str(d.get("label") or "").split()))
    if len(s.label) > 40:
        raise ServiceError("display name: at most 40 characters")
    if type_ in ("launch", "run"):
        need = ("package", "file") if type_ == "launch" else ("package", "exe")
        for k in need:
            if not re.match(r"^[A-Za-z0-9_.\-]+$", params[k]):
                raise ServiceError(f"{next(f.label for f in TYPES[type_].fields if f.key == k)}: required, "
                                   "letters, digits, _ . - only")
        for t in params["outputs"].split():
            if not TOPIC.match(t):
                raise ServiceError(f"output topic '{t}' must start with / (letters, digits, _ and / only)")
        s.argv()                                                  # the arguments must split
    if type_ == "usb_cam" and not re.match(r"^/dev/[A-Za-z0-9_\-/]+$", params["device"]):
        raise ServiceError("Device: a /dev path, e.g. /dev/video0")
    if type_ == "zed" and not re.match(r"^[0-9]*$", params["serial"]):
        raise ServiceError("Serial number: digits only (on the camera's label, or in ZED_Explorer)")
    if type_ == "realsense" and not re.match(r"^[A-Za-z0-9_]*$", params["serial"] + params["namespace"]):
        raise ServiceError("Serial number and namespace: letters, digits and _ only")
    if known is not None:
        bad = [n for n in s.needs if n not in known]
        if bad:
            raise ServiceError(f"needs an unknown service: {', '.join(bad)}")
    return s


# ====================================================================== processes
def have_package(pkg: str) -> bool:
    """Is a ROS package installed (the ament index of every prefix in AMENT_PREFIX_PATH)?"""
    for pre in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if pre and (Path(pre) / "share" / "ament_index" / "resource_index" / "packages" / pkg).exists():
            return True
    return False


def spawn_process(argv: List[str], log_path: Path):
    """The real thing: the command in its own process group (SIGINT reaches every child of `ros2 launch`)."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "ab") as f:
        env = {**os.environ, "PYTHONUNBUFFERED": "1", "RCUTILS_COLORIZED_OUTPUT": "0"}
        return subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True, env=env)


FAKE_SCRIPT = r"""
import signal, sys, time
signal.signal(signal.SIGINT, lambda *_: (print("SIGINT: shutting down", flush=True), sys.exit(0)))
print("fake driver: " + " ".join(sys.argv[1:]), flush=True)
n = 0
while True:
    time.sleep(1.0); n += 1
    if n % 10 == 0: print(f"running {n} s", flush=True)
"""


def spawn_fake(argv: List[str], log_path: Path):
    """Fake mode / tests: a real child process that only prints, so start, stop and the log behave as for real."""
    return spawn_process([sys.executable, "-c", FAKE_SCRIPT, *argv], log_path)


class RemoteProc:
    """A service the ZED container runs for us (scripts/zed_runner.py): the same face as a Popen (pid, poll) plus
    signal(). The GUI cannot start containers, so it asks the runner through files in a shared folder."""

    def __init__(self, root: Path, name: str, rid: str):
        self.root, self.name, self.id = Path(root), name, rid
        self.returncode = None
        self.msg = ""
        self.asked = time.monotonic()

    @classmethod
    def start(cls, root: Path, name: str, argv: List[str], log: Path) -> "RemoteProc":
        root = Path(root)
        if not remote_alive(root):
            raise OSError("the ZED container is not running (ur7e_gui.sh starts it when the ZED image is built)")
        rid = f"{time.time():.6f}-{os.getpid()}"
        (root / "req").mkdir(parents=True, exist_ok=True)
        tmp = root / "req" / f"{name}.tmp"
        tmp.write_text(json.dumps({"id": rid, "argv": argv, "log": str(log)}))
        tmp.replace(root / "req" / f"{name}.json")
        return cls(root, name, rid)

    def _state(self) -> dict:
        try:
            d = json.loads((self.root / "state" / f"{self.name}.json").read_text())
        except (OSError, ValueError):
            return {}
        return d if d.get("id") == self.id else {}

    @property
    def pid(self):
        return self._state().get("pid")

    def poll(self):
        if self.returncode is None:
            d = self._state()
            if d.get("rc") is not None:
                self.returncode, self.msg = int(d["rc"]), str(d.get("msg") or "")
            elif not remote_alive(self.root) and time.monotonic() - self.asked > REMOTE_TIMEOUT_S:
                self.returncode, self.msg = -1, "the ZED container stopped"
            elif not d and time.monotonic() - self.asked > REMOTE_TIMEOUT_S:
                self.returncode, self.msg = -1, "the ZED container did not take it"
        return self.returncode

    def signal(self, sig):
        (self.root / "sig").mkdir(parents=True, exist_ok=True)
        (self.root / "sig" / self.name).write_text(f"{self.id} {signal.Signals(sig).name[3:]}")


REMOTE_TIMEOUT_S = 6.0


def remote_alive(root: Path) -> bool:
    """Is the ZED container's runner up (it touches root/alive every second)?"""
    p = Path(root) / "alive"
    try:
        t = p.read_text().strip()
        return time.time() - (float(t) if t else p.stat().st_mtime) < REMOTE_TIMEOUT_S   # empty: caught mid-write
    except (OSError, ValueError):
        return False


class Adopted:
    """A service started by an earlier GUI (still running): only its pid is known, not its exit code."""

    def __init__(self, pid: int):
        self.pid = pid
        self.returncode = None

    def poll(self):
        if self.returncode is None and not _alive(self.pid):
            self.returncode = -1
        return self.returncode


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:                                                          # a zombie is gone for our purposes
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return True


def _cmdline(pid: int) -> List[str]:
    try:
        return [a for a in Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace").split("\0") if a]
    except OSError:
        return []


@dataclass
class Run:
    proc: object
    argv: List[str]
    log: Path
    started: float
    stop_at: Optional[float] = None       # monotonic time of the last stop signal
    stop_step: int = -1
    seen: Dict[str, int] = field(default_factory=dict)   # topic -> publishers (last check)
    checked: float = 0.0
    ready_at: Optional[float] = None


# ====================================================================== manager
class Services:
    """The service list (a yaml file) and the processes. Thread safe: commands and tick() come from different
    threads."""

    def __init__(self, path: Path, run_dir: Path, log_dir: Path, *, spawn: Callable = spawn_process,
                 have: Callable[[str], bool] = have_package, publishers: Callable[[str], int] = lambda t: 0,
                 on_change: Optional[Callable[[Service, bool], None]] = None,
                 log: Callable[[str], None] = lambda m: None, clock: Callable[[], float] = time.monotonic,
                 route: Optional[Callable[["Service"], Optional[Callable]]] = None,
                 remote_root: Optional[Path] = None):
        self.path, self.run_dir, self.log_dir = Path(path), Path(run_dir), Path(log_dir)
        self.spawn, self.have, self.publishers, self.on_change, self.log, self.clock = \
            spawn, have, publishers, on_change, log, clock
        self.route = route                    # a service -> the spawn for it (None: self.spawn)
        self.remote_root = Path(remote_root) if remote_root else None   # the ZED runner's folder (adopting)
        self._lock = threading.RLock()
        self.services: Dict[str, Service] = {}
        self.runs: Dict[str, Run] = {}
        self.last: Dict[str, dict] = {}       # name -> {state, msg} of the last run (failed / stopped)
        self.waiting: Dict[str, float] = {}   # name -> since: start asked, a needed service is not ready yet
        self.error = ""
        self._autostarted = False
        self._load()
        self._adopt()

    # ---------------------------------------------------------------- file
    def _load(self):
        try:
            d = yaml.safe_load(self.path.read_text()) or {}
        except FileNotFoundError:
            return
        except Exception as e:  # noqa: BLE001 - a broken file must not stop the GUI
            self.error = f"{self.path}: {e}"
            return
        rows = d.get("services") or []
        names = {str(r.get("name")) for r in rows if isinstance(r, dict)}
        for r in rows:
            try:
                s = make_service(r, names)
                self.services[s.name] = s
            except (ServiceError, AttributeError, TypeError) as e:
                self.error = f"{self.path}: {r.get('name', '?') if isinstance(r, dict) else r}: {e}"

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("# MoveIt GUI drivers and pipelines (SETUP > Drivers & cameras).\n" +
                       yaml.safe_dump({"version": 1, "services": [s.to_yaml() for s in self.services.values()]},
                                      sort_keys=False))
        tmp.replace(self.path)

    # ---------------------------------------------------------------- edit
    def save(self, d: dict, replace: Optional[str] = None) -> Service:
        with self._lock:
            known = set(self.services) - ({replace} if replace else set()) | {str(d.get("name", ""))}
            s = make_service(d, known)
            if s.name != replace and s.name in self.services:
                raise ServiceError(f"'{s.name}' is already used")
            if replace and replace not in self.services:
                raise ServiceError(f"no service '{replace}'")
            old = self.services.get(replace) if replace else None
            only_label = old is not None and {**old.to_yaml(), "label": ""} == {**s.to_yaml(), "label": ""}
            for n in (replace, s.name):
                if n in self.runs and not only_label:          # a new display name only: fine while it runs
                    raise ServiceError(f"stop '{n}' first")
            if self._cycle(s):
                raise ServiceError("these needs go round in a circle")
            items = list(self.services.items())
            if replace:
                items = [(s.name, s) if k == replace else (k, v) for k, v in items]
                for _, v in items:                               # a rename keeps the others' needs pointing at it
                    v.needs = [s.name if x == replace else x for x in v.needs]
            else:
                items.append((s.name, s))
            self.services = dict(items)
            self.last.pop(s.name, None)
            self._write()
            return s

    def _cycle(self, s: Service) -> bool:
        graph = {**{k: v.needs for k, v in self.services.items()}, s.name: s.needs}
        seen, stack = set(), list(s.needs)
        while stack:
            n = stack.pop()
            if n == s.name:
                return True
            if n not in seen:
                seen.add(n)
                stack.extend(graph.get(n, []))
        return False

    def delete(self, name: str):
        with self._lock:
            self._get(name)
            if name in self.runs:
                raise ServiceError(f"stop '{name}' first")
            users = [k for k, v in self.services.items() if name in v.needs]
            if users:
                raise ServiceError(f"{', '.join(users)} need{'s' if len(users) == 1 else ''} '{name}': change "
                                   "that first")
            del self.services[name]
            self.last.pop(name, None)
            self._write()

    def set_autostart(self, name: str, on: bool):
        with self._lock:
            self._get(name).autostart = bool(on)
            self._write()

    def _get(self, name: str) -> Service:
        s = self.services.get(name)
        if s is None:
            raise ServiceError(f"no service '{name}'")
        return s

    # ---------------------------------------------------------------- run
    def missing(self, s: Service) -> str:
        """Why s cannot start (a package is not installed), '' when it can."""
        if s.pkg and not self.have(s.pkg):
            return f"{s.pkg} is not installed: {TYPES[s.type].install}"
        return ""

    def start(self, name: str) -> str:
        """Start a service, and first whatever it needs. Returns a short message."""
        with self._lock:
            s = self._get(name)
            if name in self.runs:
                return f"{name} is already running"
            order = self._order(name)
            for n in order:
                why = self.missing(self.services[n])
                if why:
                    raise ServiceError(f"{n}: {why}")
            started = []
            for n in order:
                if n in self.runs or n in self.waiting:
                    continue
                if self._needs_ready(self.services[n]):
                    self._spawn(self.services[n])
                    started.append(n)
                else:
                    self.waiting[n] = self.clock()           # tick() starts it once its needs are ready
            extra = [n for n in order if n != name]
            return (f"starting {s.name}" + (f" (after {', '.join(extra)})" if extra else "")
                    if name in started or name in self.waiting else f"{name} is already running")

    def _order(self, name: str) -> List[str]:
        out: List[str] = []

        def visit(n):
            if n in out:
                return
            for m in self._get(n).needs:
                visit(m)
            out.append(n)
        visit(name)
        return out

    def _needs_ready(self, s: Service) -> bool:
        return all(self.state(n) == "running" for n in s.needs)

    def _spawn(self, s: Service):
        argv = s.argv()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log = self.log_dir / f"{s.name}_{stamp}.log"
        self._prune_logs(s.name)
        spawn = (self.route(s) if self.route else None) or self.spawn
        try:
            proc = spawn(argv, log)
        except OSError as e:
            self.last[s.name] = {"state": "failed", "msg": f"could not start: {e}"}
            raise ServiceError(f"{s.name}: could not start: {e}") from None
        self.runs[s.name] = Run(proc, argv, log, self.clock())
        self.waiting.pop(s.name, None)
        self.last.pop(s.name, None)
        self._pidfile(s.name, proc.pid, argv, log, getattr(proc, "id", None))
        self.log(f"service {s.name}: started{' in the ZED container' if isinstance(proc, RemoteProc) else ''}"
                 f" (pid {proc.pid}): {' '.join(argv)}")
        if self.on_change:
            self.on_change(s, True)

    def stop(self, name: str) -> str:
        """Stop a service, and first whatever needs it (a pipeline goes before its camera)."""
        with self._lock:
            self._get(name)
            users = self._users(name)
            for n in users + [name]:
                self.waiting.pop(n, None)
                r = self.runs.get(n)
                if r is not None and r.stop_at is None:
                    self._signal(n, r, 0)
            running = [n for n in users if n in self.runs]
            if name not in self.runs:
                return f"{name} is not running"
            return f"stopping {name}" + (f" (and {', '.join(running)}, which need it)" if running else "")

    def _users(self, name: str) -> List[str]:
        """The services that need name, directly or not, the farthest first."""
        out: List[str] = []

        def visit(n):
            for k, v in self.services.items():
                if n in v.needs and k not in out:
                    visit(k)
                    out.append(k)
        visit(name)
        return out

    def _signal(self, name: str, r: Run, step: int):
        sig = STOP_STEPS[step][0]
        r.stop_step, r.stop_at = step, self.clock()
        if isinstance(r.proc, RemoteProc):
            r.proc.signal(sig)
            return
        try:
            os.killpg(r.proc.pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError as e:
            self.log(f"service {name}: cannot signal pid {r.proc.pid}: {e}")
        if step:
            self.log(f"service {name}: did not stop after {STOP_STEPS[step - 1][0].name}: sent {sig.name}")

    def tick(self):
        """Call often (the web tick): reap, escalate stops, count publishers, start waiting services."""
        with self._lock:
            now = self.clock()
            if not self._autostarted:
                self._autostarted = True
                for n, s in self.services.items():
                    if s.autostart and n not in self.runs:
                        try:
                            self.start(n)
                        except ServiceError as e:
                            self.last[n] = {"state": "failed", "msg": f"autostart: {e}"}
            for n, r in list(self.runs.items()):
                rc = r.proc.poll()
                if rc is not None:
                    self._ended(n, r, rc)
                    continue
                if r.stop_at is not None:
                    if r.stop_step + 1 < len(STOP_STEPS) and now - r.stop_at > STOP_STEPS[r.stop_step][1]:
                        self._signal(n, r, r.stop_step + 1)
                    continue
                if now - r.checked >= CHECK_S:
                    r.checked = now
                    s = self.services.get(n)
                    for o in s.outputs() if s else []:
                        try:
                            r.seen[o["topic"]] = int(self.publishers(o["topic"]))
                        except Exception:  # noqa: BLE001 - a graph query while ROS shuts down
                            r.seen[o["topic"]] = 0
                    if s and r.ready_at is None and self._outputs_ok(s, r):
                        r.ready_at = now
                        self.log(f"service {n}: ready ({now - r.started:.1f} s)")
            for n in list(self.waiting):
                s = self.services.get(n)
                if s is None or any(m not in self.runs and m not in self.waiting for m in s.needs):
                    self.waiting.pop(n, None)                   # a needed service stopped or failed
                    if s is not None:
                        self.last[n] = {"state": "failed", "msg": "not started: a service it needs stopped"}
                elif self._needs_ready(s):
                    try:
                        self._spawn(s)
                    except ServiceError:
                        self.waiting.pop(n, None)

    def _outputs_ok(self, s: Service, r: Run) -> bool:
        req = [o["topic"] for o in s.outputs() if o["required"]]
        return all(r.seen.get(t, 0) > 0 for t in req)

    def _ended(self, n: str, r: Run, rc: int):
        del self.runs[n]
        self._pidfile(n, None)
        asked = r.stop_at is not None
        if asked:
            self.last[n] = {"state": "stopped", "msg": ""}
            self.log(f"service {n}: stopped")
        else:
            code = "" if isinstance(r.proc, Adopted) or rc == -1 else f" (exit code {rc})"
            why = getattr(r.proc, "msg", "")
            self.last[n] = {"state": "failed", "msg": why or f"stopped by itself{code}: see the log"}
            self.log(f"service {n}: stopped by itself{code}; log {r.log}")
        s = self.services.get(n)
        if s and self.on_change:
            self.on_change(s, False)

    def state(self, name: str) -> str:
        """stopped | waiting | starting | running | no_data | stopping | failed | missing."""
        s = self.services.get(name)
        r = self.runs.get(name)
        if r is not None:
            if r.stop_at is not None:
                return "stopping"
            if s is not None and self._outputs_ok(s, r) and r.ready_at is not None:
                return "running"
            return "no_data" if self.clock() - r.started > START_GRACE_S else "starting"
        if name in self.waiting:
            return "waiting"
        if s is not None and self.missing(s):
            return "missing"
        return self.last.get(name, {}).get("state", "stopped")

    def ready(self, name: str) -> bool:
        """For programs: the service runs and all its required outputs publish."""
        with self._lock:
            return self.state(name) == "running"

    def stop_all(self, wait: float = 0.0):
        """Stop everything (the GUI closes for good). With wait: block up to that long, escalating."""
        with self._lock:
            for n in list(self.runs):
                r = self.runs[n]
                if r.stop_at is None:
                    self._signal(n, r, 0)
            self.waiting.clear()
        end = time.monotonic() + wait
        while wait and self.runs and time.monotonic() < end:
            self.tick()
            time.sleep(0.1)
        if self.runs:
            with self._lock:
                for n, r in list(self.runs.items()):
                    self._signal(n, r, len(STOP_STEPS) - 1)

    # ---------------------------------------------------------------- pid files (adopt after a GUI restart)
    def _pidfile(self, name: str, pid: Optional[int], argv=None, log=None, remote_id=None):
        p = self.run_dir / f"{name}.json"
        if pid is None and not remote_id:
            try:
                p.unlink()
            except FileNotFoundError:
                pass
            return
        self.run_dir.mkdir(parents=True, exist_ok=True)
        d = {"pid": pid, "argv": argv, "log": str(log), "t": time.time()}
        if remote_id:
            d["remote"] = remote_id
        p.write_text(json.dumps(d))

    def _adopt(self):
        if not self.run_dir.is_dir():
            return
        for p in sorted(self.run_dir.glob("*.json")):
            name = p.stem
            try:
                d = json.loads(p.read_text())
                argv = list(d["argv"])
                if d.get("remote"):
                    proc = RemoteProc(self.remote_root, name, str(d["remote"])) if self.remote_root else None
                    if name in self.services and proc is not None and proc.poll() is None and proc._state():
                        self.runs[name] = Run(proc, argv, Path(d.get("log", "")), self.clock())
                        self.log(f"service {name}: still running in the ZED container")
                        if self.on_change:
                            self.on_change(self.services[name], True)
                    else:
                        self._pidfile(name, None)
                    continue
                pid = int(d["pid"])
            except (OSError, ValueError, KeyError, TypeError):
                continue
            cmd = _cmdline(pid)
            if name in self.services and _alive(pid) and cmd[-len(argv):] == argv:   # not a reused pid
                self.runs[name] = Run(Adopted(pid), argv, Path(d.get("log", "")), self.clock())
                self.log(f"service {name}: still running from before (pid {pid})")
                if self.on_change:
                    self.on_change(self.services[name], True)
            else:
                self._pidfile(name, None)

    # ---------------------------------------------------------------- logs
    def _logs(self, name: str) -> List[Path]:
        return sorted(self.log_dir.glob(f"{name}_*.log")) if self.log_dir.is_dir() else []

    def _prune_logs(self, name: str):
        for p in self._logs(name)[:-(KEEP_LOGS - 1)]:
            try:
                p.unlink()
            except OSError:
                pass

    def log_tail(self, name: str, lines: int = 60) -> dict:
        with self._lock:
            self._get(name)
            r = self.runs.get(name)
            path = r.log if r else (self._logs(name) or [None])[-1]
        if path is None or not Path(path).is_file():
            return {"file": "", "text": ""}
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 24000))
            text = f.read().decode(errors="replace")
        return {"file": str(path), "text": "\n".join(text.splitlines()[-int(lines):])}

    # ---------------------------------------------------------------- views
    def view(self) -> dict:
        with self._lock:
            now, rows = self.clock(), []
            for n, s in self.services.items():
                r, st = self.runs.get(n), self.state(n)
                outs = [{**o, "publishers": r.seen.get(o["topic"], 0) if r else 0} for o in s.outputs()]
                msg = {"missing": self.missing(s), "waiting": "waiting for " + ", ".join(
                    m for m in s.needs if self.state(m) != "running"),
                       "no_data": "running, but not all its topics publish: see the log",
                       }.get(st) or self.last.get(n, {}).get("msg", "")
                rows.append({"name": n, "label": s.title, "type": s.type, "type_label": TYPES[s.type].label, "kind": s.kind,
                             "state": st, "msg": msg, "pid": r.proc.pid if r else None,
                             "up_s": round(now - r.started) if r else None, "outputs": outs,
                             "needs": s.needs, "autostart": s.autostart, "note": s.note, "display": s.label, "params": s.params,
                             "command": " ".join(shlex.quote(a) for a in s.argv())})
            return {"services": rows, "error": self.error}

    @staticmethod
    def types_view(have: Callable[[str], bool] = have_package) -> List[dict]:
        return [{"type": k, "label": t.label, "kind": t.kind, "help": t.help, "pkg": t.pkg,
                 "installed": not t.pkg or have(t.pkg), "install": t.install,
                 "fields": [{"key": f.key, "label": f.label, "default": f.default, "hint": f.hint,
                             "choices": [list(c) for c in f.choices]} for f in t.fields]}
                for k, t in TYPES.items()]
