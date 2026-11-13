"""Session recording: a rosbag2 (MCAP) of the topics the operator picks, started and stopped from the GUI.

    <recordings_dir>/<YYYYmmdd_HHMMSS>_<name>/bag/          the `ros2 bag record` output (metadata.yaml + *.mcap)
                                              meta.json      the short human summary: start / stop time, operator,
                                                             program, control mode, note, topics
                                              gui_meta.json  what started it, which run it belongs to, why it ended
                                              photos/        stills from a program's `photo` step (if any)
                                              record.log     the recorder's own output (errors land here)

Nothing here moves the robot. The recorder is a child process (`ros2 bag record`) in its own process group, so a
STOP or a closed GUI ends it cleanly (SIGINT: rosbag2 then finishes the file). Limits on duration, size and free
disk space end a recording on their own, because a forgotten recorder of raw camera images fills a disk fast.

Topics are chosen by preset (a rule on the topic name or message type, resolved against what is published now)
plus single topics. The selection, limits and the auto-record switch are saved per robot profile.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

from .calibration import config_dir

Result = Tuple[bool, str]

# id: (label, topic-name regex or None, message-type regex or None, note)
PRESETS: Dict[str, Tuple[str, Optional[str], Optional[str], str]] = {
    "robot": ("Robot state", r"^/(joint_states|tf|tf_static|robot_description)$|speed_scaling|controller_state$"
              r"|^/io_and_status_controller/", None, "joints, TF, speed scaling, controller states"),
    "ft": ("Force / torque", None, r"^geometry_msgs/msg/WrenchStamped$", "every wrench topic"),
    "gripper": ("Gripper", r"gripper|robotiq", None, "gripper commands and status"),
    "policy": ("Policy", r"polic|scrap|/reference_pose$|wrench_reference|set_cartesian_wrench|delta_twist", None,
               "policy status, commands and targets"),
    "planning": ("Planning", r"^/(display_planned_path|monitored_planning_scene|planning_scene|collision_object)$",
                 None, "MoveIt plans and the planning scene"),
    "camera_compressed": ("Cameras (compressed)", None, r"^sensor_msgs/msg/CompressedImage$",
                          "camera images, compressed (small)"),
    "camera_raw": ("Cameras (raw)", None, r"^sensor_msgs/msg/Image$",
                   "uncompressed images: about 30 MB/s per 640x480 camera at 30 fps"),
    # The one-button recording: everything useful for a run. TCP pose is in /tf (tool0) and the UR pose broadcaster.
    "all": ("Everything for a run", r"^/(joint_states|dynamic_joint_states|tf|tf_static|robot_description)$"
            r"|speed_scaling|controller_state$|^/io_and_status_controller/|tcp_pose|pose_broadcaster"
            r"|^/controller_manager/|gripper|robotiq", r"^(geometry_msgs/msg/WrenchStamped"
            r"|sensor_msgs/msg/CompressedImage)$", "joints, TCP pose, F/T, gripper, controllers, compressed cameras"),
    "log": ("ROS log", r"^/rosout$", None, "every node's log messages"),
}
DEFAULT_PRESETS = ["robot", "ft", "gripper", "policy"]
DEFAULT_LIMITS = {"max_s": 600.0, "max_mb": 4000.0}
MIN_FREE_MB = 2000.0              # refuse to start below this, stop below half of it
STOP_WAIT_S = 10.0                # SIGINT, then SIGTERM after this long, then SIGKILL
NAME_RE = re.compile(r"[^A-Za-z0-9_.-]+")


# Bulky sensor data is only ever picked by the camera presets, never by a name rule (e.g. /scraping_vision/annotated_image
# matches the policy preset's "scrap" but is a raw image stream).
BULKY_TYPES = re.compile(r"^sensor_msgs/msg/(Image|CompressedImage|PointCloud2|LaserScan)$")


def preset_topics(preset: str, topics: List[Tuple[str, str]]) -> List[str]:
    _label, name_re, type_re, _note = PRESETS[preset]
    out = []
    for name, typ in topics:
        if name_re and re.search(name_re, name) and not BULKY_TYPES.match(typ):
            out.append(name)
        elif type_re and re.search(type_re, typ):
            out.append(name)
    return sorted(set(out))


def dir_size_mb(path: Path) -> float:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / 1e6


def ros2_bag_spawn(out_dir: Path, topics: List[str], log_path: Path):
    """The real recorder: `ros2 bag record` in its own process group (so SIGINT reaches every child)."""
    log = open(log_path, "ab")
    try:
        return subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", str(out_dir), *topics],
                                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
    finally:
        log.close()


class FakeBagProcess:
    """Stands in for `ros2 bag record` in fake mode and tests: writes a growing file until signalled."""

    def __init__(self, out_dir: Path, topics: List[str], log_path: Path, mb_per_s: float = 0.5):
        self.out_dir, self.topics = Path(out_dir), list(topics)
        self.out_dir.mkdir(parents=True)
        self.pid = -1
        self.returncode: Optional[int] = None
        self._stop = threading.Event()
        self._mb_per_s = mb_per_s
        threading.Thread(target=self._run, daemon=True, name="fake-bag").start()

    def _run(self):
        f = self.out_dir / f"{self.out_dir.name}_0.mcap"
        with open(f, "wb") as fh:
            while not self._stop.wait(0.05):
                fh.write(b"\0" * int(self._mb_per_s * 1e6 * 0.05))
                fh.flush()
        (self.out_dir / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n  storage_identifier: mcap\n  topics_with_message_count:\n"
            + "".join(f"    - topic_metadata: {{name: {t}}}\n" for t in self.topics))
        self.returncode = 0

    def poll(self):
        return self.returncode

    def send_signal(self, _sig):
        self._stop.set()

    def wait(self, timeout=None):
        t0 = time.monotonic()
        while self.returncode is None:
            if timeout is not None and time.monotonic() - t0 > timeout:
                raise subprocess.TimeoutExpired("fake-bag", timeout)
            time.sleep(0.01)
        return self.returncode

    def kill(self):
        self._stop.set()


class Recorder:
    def __init__(self, root: Path, profile_name: str, list_topics: Callable[[], List[Tuple[str, str]]],
                 log: Callable[[str], None], spawn: Optional[Callable] = None,
                 free_mb: Optional[Callable[[Path], float]] = None, settings_path: Optional[Path] = None):
        self.root = Path(root)
        self.list_topics = list_topics
        self.log = log
        self.spawn = spawn or ros2_bag_spawn
        self.free_mb = free_mb or (lambda p: shutil.disk_usage(p).free / 1e6)
        self.settings_path = settings_path or config_dir() / "recording" / f"{profile_name}.json"
        self._lock = threading.Lock()
        self._proc = None
        self._cur: Optional[dict] = None             # the recording in progress
        self._stopping = False
        self.last: Optional[dict] = None             # the last finished recording (its gui_meta)
        self.settings = {"presets": list(DEFAULT_PRESETS), "topics": [], "exclude": [], "auto": False,
                         **DEFAULT_LIMITS}
        try:
            saved = json.loads(self.settings_path.read_text(encoding="utf-8"))
            self.settings.update({k: v for k, v in saved.items() if k in self.settings})
        except (OSError, ValueError):
            pass

    # ------------------------------------------------------------- settings
    def configure(self, presets=None, topics=None, exclude=None, auto=None, max_s=None, max_mb=None) -> Result:
        s = dict(self.settings)
        if presets is not None:
            bad = [p for p in presets if p not in PRESETS]
            if bad:
                return False, f"unknown preset(s): {', '.join(bad)}"
            s["presets"] = list(dict.fromkeys(presets))
        for key, val in (("topics", topics), ("exclude", exclude)):
            if val is not None:
                bad = [t for t in val if not isinstance(t, str) or not t.startswith("/") or len(t) > 256]
                if bad:
                    return False, f"not a topic name: {bad[0]!r}"
                s[key] = sorted(set(val))
        if auto is not None:
            s["auto"] = auto is True or str(auto).lower() == "true"
        if max_s is not None:
            if not 1 <= float(max_s) <= 24 * 3600:
                return False, "the time limit must be 1 s..24 h"
            s["max_s"] = float(max_s)
        if max_mb is not None:
            if not 10 <= float(max_mb) <= 1e6:
                return False, "the size limit must be 10 MB..1 TB"
            s["max_mb"] = float(max_mb)
        self.settings = s
        try:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            self.settings_path.write_text(json.dumps(s, indent=1), encoding="utf-8")
        except OSError as e:
            return True, f"settings applied but not saved ({e})"
        return True, "recording settings saved"

    def selected_topics(self, topics: Optional[List[Tuple[str, str]]] = None) -> List[str]:
        topics = self.list_topics() if topics is None else topics
        out = set(self.settings["topics"])
        for p in self.settings["presets"]:
            out.update(preset_topics(p, topics))
        return sorted(out - set(self.settings["exclude"]))

    def catalog(self) -> dict:
        """Everything published now, with the presets each topic belongs to (for the topic picker)."""
        topics = self.list_topics()
        member = {name: [] for name, _ in topics}
        for p in PRESETS:
            for name in preset_topics(p, topics):
                member[name].append(p)
        chosen = set(self.selected_topics(topics))
        return {"topics": [{"topic": n, "type": t, "presets": member.get(n, []), "selected": n in chosen}
                           for n, t in sorted(topics)],
                "presets": [{"id": k, "label": v[0], "note": v[3], "count": len(preset_topics(k, topics))}
                            for k, v in PRESETS.items()],
                "missing": sorted(set(self.settings["topics"]) - {n for n, _ in topics})}

    # ------------------------------------------------------------- start / stop
    @property
    def recording(self) -> bool:
        return self._cur is not None

    def start(self, name: str = "", source: str = "manual", run: Optional[dict] = None,
              topics: Optional[List[str]] = None, presets: Optional[List[str]] = None,
              info: Optional[dict] = None) -> Result:
        """presets: record these presets (plus the single topics ticked in the settings) instead of the saved
        selection. info: operator / program / mode / note, for meta.json."""
        with self._lock:
            if self._cur is not None:
                return False, f"already recording ({self._cur['name']})"
            if presets:
                bad = [p for p in presets if p not in PRESETS]
                if bad:
                    return False, f"unknown preset(s): {', '.join(bad)}"
                avail = self.list_topics()
                chosen = set(self.settings["topics"])
                for p in presets:
                    chosen.update(preset_topics(p, avail))
                topics = sorted(chosen - set(self.settings["exclude"])) if not topics else topics
            topics = sorted(set(topics)) if topics else self.selected_topics()
            if not topics:
                return False, "nothing to record: no selected topic is published now"
            try:
                self.root.mkdir(parents=True, exist_ok=True)
                free = self.free_mb(self.root)
            except OSError as e:
                return False, f"cannot use {self.root}: {e}"
            if free < MIN_FREE_MB:
                return False, f"only {free:.0f} MB free in {self.root} (need {MIN_FREE_MB:.0f} MB)"
            label = NAME_RE.sub("_", name.strip())[:60].strip("_") or source
            base = f"{time.strftime('%Y%m%d_%H%M%S')}_{label}"
            out, n = self.root / base, 1
            while out.exists():
                n += 1
                out = self.root / f"{base}~{n}"
            out.mkdir()
            try:
                proc = self.spawn(out / "bag", topics, out / "record.log")   # ros2 bag creates bag/ itself
            except (OSError, ValueError) as e:
                return False, f"could not start ros2 bag record: {e}"
            self._proc = proc
            self._stopping = False
            self._cur = {"name": out.name, "dir": str(out), "label": label,
                         "source": source, "run": run or {}, "topics": topics, "started": time.time(),
                         "info": dict(info or {}), "presets": list(presets or self.settings["presets"]),
                         "t0": time.monotonic(), "size_mb": 0.0, "limits": {k: self.settings[k]
                                                                           for k in ("max_s", "max_mb")}}
            self._write_meta(self._cur, ended=None, reason="recording")
        self.log(f"recording started: {out.name} ({len(topics)} topics)")
        return True, f"recording {len(topics)} topics to {out}"

    def stop(self, reason: str = "stopped") -> Result:
        """Asks the recorder to finish (SIGINT). Returns at once; poll() completes it."""
        with self._lock:
            if self._cur is None:
                return True, "not recording"
            if self._stopping:
                return True, "already stopping"
            self._stopping = True
            self._cur["end_reason"] = reason
            proc = self._proc
        try:
            proc.send_signal(signal.SIGINT) if proc.pid < 0 else os.killpg(proc.pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        threading.Thread(target=self._finish, args=(proc,), daemon=True, name="record-stop").start()
        return True, f"stopping the recording ({reason})"

    def stop_and_wait(self, reason: str = "GUI closed", timeout: float = STOP_WAIT_S + 3):
        self.stop(reason)
        t0 = time.monotonic()
        while self._cur is not None and time.monotonic() - t0 < timeout:
            time.sleep(0.05)

    def _finish(self, proc):
        try:
            proc.wait(timeout=STOP_WAIT_S)
        except subprocess.TimeoutExpired:
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    proc.send_signal(sig) if proc.pid < 0 else os.killpg(proc.pid, sig)
                    proc.wait(timeout=3)
                    break
                except (subprocess.TimeoutExpired, ProcessLookupError, OSError):
                    continue
        self._ended(proc.poll())

    def _ended(self, code):
        with self._lock:
            cur, self._cur, self._proc = self._cur, None, None
            self._stopping = False
        if cur is None:
            return
        reason = cur.get("end_reason") or f"the recorder exited by itself (code {code})"
        cur["size_mb"] = dir_size_mb(Path(cur["dir"])) if Path(cur["dir"]).exists() else 0.0
        ok = Path(cur["dir"], "bag", "metadata.yaml").exists()
        if not ok:
            reason += "; no bag was written: see " + str(Path(cur["dir"], "record.log"))
        self.last = self._write_meta(cur, ended=time.time(), reason=reason, ok=ok)
        self.log(f"recording {'saved' if ok else 'FAILED'}: {cur['name']} "
                 f"({cur['size_mb']:.1f} MB, {time.time() - cur['started']:.0f} s): {reason}")

    def _write_meta(self, cur: dict, ended: Optional[float], reason: str, ok: Optional[bool] = None) -> dict:
        meta = {k: cur[k] for k in ("name", "dir", "source", "run", "topics", "started", "size_mb", "limits")}
        meta.update(ended=ended, duration_s=round((ended or time.time()) - cur["started"], 1), reason=reason, ok=ok)
        iso = lambda t: time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t)) if t else None  # noqa: E731
        info = cur.get("info", {})
        small = {"started": iso(cur["started"]), "stopped": iso(ended), "duration_s": meta["duration_s"],
                 "operator": info.get("operator", ""), "program": info.get("program", ""),
                 "mode": info.get("mode", ""), "note": info.get("note", ""), "presets": cur.get("presets", []),
                 "topics": cur["topics"], "size_mb": round(cur["size_mb"], 1), "ok": ok, "reason": reason}
        try:
            Path(cur["dir"], "gui_meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
            Path(cur["dir"], "meta.json").write_text(json.dumps(small, indent=1), encoding="utf-8")
        except OSError:
            pass
        return meta

    def poll(self):
        """Call at the UI rate: limits, disk space, a recorder that died."""
        cur, proc = self._cur, self._proc
        if cur is None or self._stopping:
            return
        code = proc.poll()
        if code is not None:
            self._ended(code)
            return
        now = time.monotonic()
        if now - cur.get("_size_t", 0) > 1.0:
            cur["_size_t"] = now
            cur["size_mb"] = dir_size_mb(Path(cur["dir"])) if Path(cur["dir"]).exists() else 0.0
            try:
                free = self.free_mb(self.root)
            except OSError:
                free = MIN_FREE_MB
            if free < MIN_FREE_MB / 2:
                self.stop(f"disk almost full ({free:.0f} MB free)")
                return
        lim = cur["limits"]
        if now - cur["t0"] > lim["max_s"]:
            self.stop(f"time limit {lim['max_s']:.0f} s reached")
        elif cur["size_mb"] > lim["max_mb"]:
            self.stop(f"size limit {lim['max_mb']:.0f} MB reached")

    # ------------------------------------------------------------- views
    def view(self) -> dict:
        cur = self._cur
        live = None
        if cur is not None:
            live = {"name": cur["name"], "dir": cur["dir"], "source": cur["source"], "run": cur["run"],
                    "topics": len(cur["topics"]), "elapsed_s": round(time.monotonic() - cur["t0"], 1),
                    "size_mb": round(cur["size_mb"], 1), "limits": cur["limits"], "stopping": self._stopping}
        return {"recording": live, "settings": dict(self.settings), "last": self.last, "dir": str(self.root)}

    def recent(self, n: int = 30) -> List[dict]:
        out = []
        try:
            metas = sorted(self.root.glob("*/gui_meta.json"), reverse=True)[:n]
        except OSError:
            return out
        for m in metas:
            try:
                out.append(json.loads(m.read_text(encoding="utf-8")))
            except (OSError, ValueError):
                continue
        return out
