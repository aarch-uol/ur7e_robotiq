"""Run history: one JSON line per program or policy run, kept per robot profile.

    $XDG_CONFIG_HOME/moveit_gui/history/<profile>.jsonl

Researchers need to find "which run was that" afterwards: what ran, when, who started it, how long, how it ended
(finished / failed + reason / stopped by whom), how many steps were done, the F/T peak, the linked recording and the
log lines of the run. The file is append-only; when it holds more than KEEP_RUNS runs (plus a little slack) the
oldest are dropped. The list shows the log-free summary, `get(id)` returns the full entry with its log lines.
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import List, Optional

from .calibration import config_dir

KEEP_RUNS = 500            # runs kept on disk (and listed in the GUI)
TRIM_SLACK = 50            # the file is rewritten when it is this many runs over, not on every run
LOG_LINES = 100            # log lines kept per run
LOG_CHARS = 200


def outcome(ok: bool, stopped_by: str) -> str:
    return "stopped" if stopped_by else ("finished" if ok else "failed")


class RunHistory:
    def __init__(self, profile_name: str, path: Optional[Path] = None):
        self.path = Path(path) if path else config_dir() / "history" / f"{profile_name}.jsonl"
        self._lock = threading.Lock()
        self.entries: List[dict] = []
        self._lines = 0
        try:
            lines = [x for x in self.path.read_text(encoding="utf-8").splitlines() if x.strip()]
        except OSError:
            lines = []
        self._lines = len(lines)
        for x in lines[-KEEP_RUNS:]:
            try:
                e = json.loads(x)
            except ValueError:
                continue
            e.pop("log", None)
            self.entries.append(e)

    def record(self, kind: str, name: str, started: Optional[float], ok: bool, result: str,
               log_dir: str = "", log: Optional[List[str]] = None, **extra) -> dict:
        now = time.time()
        start = started or now
        e = {"id": f"{start:.3f}", "kind": kind, "name": name, "start": start,
             "duration_s": round(now - started, 2) if started else 0.0,
             "ok": bool(ok), "result": result, "log_dir": log_dir, **extra}
        e["outcome"] = outcome(e["ok"], e.get("stopped_by") or "")
        full = dict(e)
        if log:
            full["log"] = [str(x)[:LOG_CHARS] for x in log[-LOG_LINES:]]
        with self._lock:
            self.entries.append(e)
            del self.entries[:-KEEP_RUNS]
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                with open(self.path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(full) + "\n")
                self._lines += 1
                if self._lines > KEEP_RUNS + TRIM_SLACK:
                    self._trim()
            except OSError:
                pass                       # history is a convenience: never break a run over it
        return e

    def _trim(self):
        lines = [x for x in self.path.read_text(encoding="utf-8").splitlines() if x.strip()][-KEEP_RUNS:]
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
        os.replace(tmp, self.path)
        self._lines = len(lines)

    def recent(self, n: int = 50, name: str = "") -> List[dict]:
        with self._lock:
            es = [e for e in self.entries if not name or e.get("name") == name]
            return list(reversed(es[-n:]))

    def get(self, run_id: str) -> Optional[dict]:
        """The full entry (with its log lines) of one run, or None."""
        with self._lock:
            try:
                for x in reversed(self.path.read_text(encoding="utf-8").splitlines()):
                    if f'"id": "{run_id}"' in x:
                        e = json.loads(x)
                        if e.get("id") == run_id:
                            return e
            except (OSError, ValueError):
                pass
        return None
