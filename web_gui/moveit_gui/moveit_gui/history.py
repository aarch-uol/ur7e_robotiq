"""Run history: one JSON line per program or policy run, kept per robot profile.

    $XDG_CONFIG_HOME/moveit_gui/history/<profile>.jsonl

Researchers need to find "which run was that" afterwards: what ran, when, how long, how it ended and where
the policy logged. The file is append-only and the GUI shows the most recent entries.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import List, Optional

from .calibration import config_dir

KEEP_IN_MEMORY = 200


class RunHistory:
    def __init__(self, profile_name: str, path: Optional[Path] = None):
        self.path = Path(path) if path else config_dir() / "history" / f"{profile_name}.jsonl"
        self._lock = threading.Lock()
        self.entries: List[dict] = []
        try:
            lines = self.path.read_text(encoding="utf-8").splitlines()[-KEEP_IN_MEMORY:]
            self.entries = [json.loads(x) for x in lines if x.strip()]
        except (OSError, ValueError):
            pass

    def record(self, kind: str, name: str, started: Optional[float], ok: bool, result: str,
               log_dir: str = "", **extra) -> dict:
        now = time.time()
        e = {"kind": kind, "name": name, "start": started or now,
             "duration_s": round(now - started, 2) if started else 0.0,
             "ok": bool(ok), "result": result, "log_dir": log_dir, **extra}
        with self._lock:
            self.entries.append(e)
            del self.entries[:-KEEP_IN_MEMORY]
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                with open(self.path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(e) + "\n")
            except OSError:
                pass                       # history is a convenience: never break a run over it
        return e

    def recent(self, n: int = 50) -> List[dict]:
        with self._lock:
            return list(reversed(self.entries[-n:]))
