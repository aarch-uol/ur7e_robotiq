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
import tempfile
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


def _atomic_write(path: Path, data: bytes):
    """Publish one complete file or leave the previous file untouched."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp",
                                         delete=False) as f:
            tmp = Path(f.name)
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        tmp = None
        fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if tmp is not None:
            try:
                tmp.unlink()
            except FileNotFoundError:
                pass


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
        self._finalizing = False
        self._done: Optional[threading.Event] = None
        self._free_at = 0.0
        self._free_cached: Optional[float] = None
        self._free_error = ""
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
        try:
            _atomic_write(self.settings_path, json.dumps(s, indent=1).encode("utf-8"))
        except OSError as e:
            return False, f"could not save recording settings ({e})"
        self.settings = s
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
                try:
                    log = out / "record.log"
                    if log.is_file() and log.stat().st_size == 0:
                        log.unlink()
                    out.rmdir()             # only removes it if the failed spawn left it empty
                except OSError:
                    pass
                return False, f"could not start ros2 bag record: {e}"
            self._proc = proc
            self._stopping = False
            self._finalizing = False
            self._done = threading.Event()
            self._cur = {"name": out.name, "dir": str(out), "label": label,
                         "source": source, "run": run or {}, "topics": topics, "started": time.time(),
                         "info": dict(info or {}), "presets": list(presets or self.settings["presets"]),
                         "t0": time.monotonic(), "size_mb": 0.0, "limits": {k: self.settings[k]
                                                                           for k in ("max_s", "max_mb")}}
            try:
                self._write_meta(self._cur, ended=None, reason="recording")
            except OSError as e:
                meta_error = e
            else:
                meta_error = None
        if meta_error is not None:
            self.stop_and_wait(f"cannot save recording metadata: {meta_error}")
            return False, f"could not start recording: cannot save metadata ({meta_error})"
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

    def stop_and_wait(self, reason: str = "GUI closed", timeout: float = STOP_WAIT_S + 3) -> Result:
        with self._lock:
            done = self._done if self._cur is not None else None
            name = self._cur["name"] if self._cur is not None else None
        ok, msg = self.stop(reason)
        if not ok or done is None:
            return ok, msg
        if not done.wait(timeout):
            return False, f"recording finalization timed out after {timeout:.1f} s"
        last = self.last
        if last is None or last.get("name") != name:
            return False, "recording finalization ended without a result"
        return bool(last.get("ok")), str(last.get("reason") or "recording finished")

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
            if self._cur is None or self._finalizing:
                return
            cur, done = self._cur, self._done
            self._finalizing = True
            self._stopping = True
        reason = cur.get("end_reason") or f"the recorder exited by itself (code {code})"
        cur["size_mb"] = dir_size_mb(Path(cur["dir"])) if Path(cur["dir"]).exists() else 0.0
        bag_ok = Path(cur["dir"], "bag", "metadata.yaml").is_file()
        ok = code == 0 and bag_ok
        if code != 0:
            reason += f"; recorder exit code {code}" if code is not None else "; recorder did not exit cleanly"
        if not bag_ok:
            reason += "; no bag metadata was written: see " + str(Path(cur["dir"], "record.log"))
        ended = time.time()
        try:
            last = self._write_meta(cur, ended=ended, reason=reason, ok=ok)
        except OSError as e:
            ok = False
            reason += f"; metadata save failed: {e}"
            last = self._metadata(cur, ended=ended, reason=reason, ok=False)[0]
            try:                         # a transient failure may still let us publish the failure marker
                self._write_meta(cur, ended=ended, reason=reason, ok=False)
            except OSError:
                pass
        last["ok"] = ok
        last["reason"] = reason
        with self._lock:
            self.last = last
            self._cur = None
            self._proc = None
            self._stopping = False
            self._finalizing = False
            self._done = None
            if done is not None:
                done.set()
        self.log(f"recording {'saved' if ok else 'FAILED'}: {cur['name']} "
                 f"({cur['size_mb']:.1f} MB, {ended - cur['started']:.0f} s): {reason}")

    def _metadata(self, cur: dict, ended: Optional[float], reason: str,
                  ok: Optional[bool] = None) -> Tuple[dict, dict]:
        elapsed = time.monotonic() - cur["t0"]
        meta = {k: cur[k] for k in ("name", "dir", "source", "run", "topics", "started", "size_mb", "limits")}
        meta.update(ended=ended, duration_s=round(elapsed, 1), reason=reason, ok=ok,
                    timebase={"timestamps": "system wall clock", "duration": "monotonic clock"})
        iso = lambda t: time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t)) if t else None  # noqa: E731
        info = cur.get("info", {})
        meta.update(operator=info.get("operator", ""), program=info.get("program", ""),
                    presets=cur.get("presets", []))
        small = {"started": iso(cur["started"]), "stopped": iso(ended), "duration_s": meta["duration_s"],
                 "operator": info.get("operator", ""), "program": info.get("program", ""),
                 "mode": info.get("mode", ""), "note": info.get("note", ""), "presets": cur.get("presets", []),
                 "topics": cur["topics"], "size_mb": round(cur["size_mb"], 1), "ok": ok, "reason": reason,
                 "timebase": meta["timebase"]}
        return meta, small

    def _write_meta(self, cur: dict, ended: Optional[float], reason: str, ok: Optional[bool] = None) -> dict:
        meta, small = self._metadata(cur, ended, reason, ok)
        out = Path(cur["dir"])
        _atomic_write(out / "meta.json", json.dumps(small, indent=1).encode("utf-8"))
        _atomic_write(out / "gui_meta.json", json.dumps(meta, indent=1).encode("utf-8"))  # catalog marker last
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
            except OSError as e:
                self.stop(f"cannot check disk space: {e}")
                return
            if free < MIN_FREE_MB / 2:
                self.stop(f"disk almost full ({free:.0f} MB free)")
                return
        lim = cur["limits"]
        if now - cur["t0"] > lim["max_s"]:
            self.stop(f"time limit {lim['max_s']:.0f} s reached")
        elif cur["size_mb"] > lim["max_mb"]:
            self.stop(f"size limit {lim['max_mb']:.0f} MB reached")

    # ------------------------------------------------------------- views
    def _storage_view(self) -> dict:
        now = time.monotonic()
        if now - self._free_at >= 1.0:
            path = self.root if self.root.exists() else self.root.parent
            try:
                self._free_cached = float(self.free_mb(path))
                self._free_error = ""
            except OSError as e:
                self._free_cached = None
                self._free_error = str(e)
            self._free_at = now
        free = self._free_cached
        state = "unknown" if free is None else "critical" if free < MIN_FREE_MB / 2 else "low" if free < MIN_FREE_MB else "healthy"
        return {"state": state, "free_mb": round(free, 1) if free is not None else None,
                "free_gb": round(free / 1000.0, 1) if free is not None else None,
                "can_start": free is not None and free >= MIN_FREE_MB,
                "start_min_mb": MIN_FREE_MB, "stop_min_mb": MIN_FREE_MB / 2,
                "error": self._free_error}

    def view(self) -> dict:
        cur = self._cur
        live = None
        if cur is not None:
            live = {"name": cur["name"], "dir": cur["dir"], "source": cur["source"], "run": cur["run"],
                    "topics": len(cur["topics"]), "elapsed_s": round(time.monotonic() - cur["t0"], 1),
                    "size_mb": round(cur["size_mb"], 1), "limits": cur["limits"], "stopping": self._stopping}
        return {"recording": live, "settings": dict(self.settings), "last": self.last,
                "storage": self._storage_view(), "dir": str(self.root)}

    def _record_dir(self, name: str) -> Optional[Path]:
        name = str(name or "")
        if not name or Path(name).name != name or name in (".", ".."):
            return None
        try:
            root = self.root.resolve()
            raw = self.root / name
            if raw.is_symlink():
                return None
            path = raw.resolve()
        except OSError:
            return None
        return path if path.parent == root and path.is_dir() else None

    def record_info(self, name: str) -> Optional[dict]:
        path = self._record_dir(name)
        if path is None:
            return None
        best = None
        for fn in ("gui_meta.json", "meta.json"):
            try:
                data = json.loads((path / fn).read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    data.setdefault("name", path.name)
                    data.setdefault("dir", str(path))
                    best = best or data
                    if data.get("ok") is not None:
                        return data                         # a finalized meta.json may repair stale start metadata
            except (OSError, ValueError):
                pass
        return best or {"name": path.name, "dir": str(path), "operator": "", "incomplete": True}

    def mcap_path(self, name: str) -> Optional[Path]:
        path = self._record_dir(name)
        if path is None:
            return None
        with self._lock:
            if self._cur is not None and self._cur.get("name") == name:
                return None
        info = self.record_info(name)
        if info is None or info.get("incomplete") or ("ok" in info and info.get("ok") is None):
            return None                                  # never offer a file still being written / not finalized
        try:
            files = [f for f in sorted((path / "bag").glob("*.mcap")) if f.is_file() and not f.is_symlink()]
        except OSError:
            return None
        return files[0] if len(files) == 1 else None

    def delete_recording(self, name: str) -> Result:
        with self._lock:
            if self._cur is not None and self._cur.get("name") == name:
                return False, "stop the recording before deleting it"
        path = self._record_dir(name)
        if path is None:
            return False, "recording not found"
        try:
            shutil.rmtree(path)
        except OSError as e:
            return False, f"could not delete the recording: {e}"
        return True, f"deleted recording {name}"

    def recent(self, n: int = 30) -> List[dict]:
        out = []
        try:
            dirs = sorted((p for p in self.root.iterdir() if p.is_dir() and not p.is_symlink()),
                          key=lambda p: p.name, reverse=True)[:max(0, int(n))]
        except OSError:
            return out
        with self._lock:
            active = self._cur.get("name", "") if self._cur is not None else ""
        for path in dirs:
            row = self.record_info(path.name)
            if row is None:
                continue
            row = dict(row)
            row.setdefault("name", path.name)
            row.setdefault("dir", str(path))
            row.setdefault("operator", "")
            row.setdefault("program", "")
            row.setdefault("source", "")
            row.setdefault("run", {})
            row.setdefault("topics", [])
            row.setdefault("duration_s", 0.0)
            if not isinstance(row.get("started"), (int, float)):
                try:
                    row["started"] = time.mktime(time.strptime(path.name[:15], "%Y%m%d_%H%M%S"))
                except (ValueError, OSError):
                    try:
                        row["started"] = path.stat().st_mtime
                    except OSError:
                        row["started"] = 0.0
            if not isinstance(row.get("size_mb"), (int, float)):
                row["size_mb"] = round(dir_size_mb(path), 1)
            row["active"] = path.name == active
            if not row["active"] and (row.get("incomplete") or ("ok" in row and row.get("ok") is None)):
                row["ok"] = False
                row["reason"] = "interrupted or incomplete recording: final metadata was not saved"
            row.setdefault("reason", "")
            row["mcap"] = self.mcap_path(path.name) is not None
            out.append(row)
        return out
