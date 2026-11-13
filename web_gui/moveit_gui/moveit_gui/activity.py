"""Activity log: who did what and when, for everyone who uses the web GUI.

    $XDG_CONFIG_HOME/moveit_gui/activity/activity.jsonl      (+ activity.jsonl.1 after a rotation)

One JSON line per event: {t, time, who, key, uid, role, kind, what, details}.
  who      the name typed in the browser; key: the Tailscale login or IP (the roles.py key); uid: the public user id
  kind     control | stop | program | policy | record | mode | power | setup | roles | batch | teach
Secrets never go in: no lease tokens, no browser ids (only the public id), and command arguments named like
a secret or carrying big payloads are left out of `details`. Append-only; at ROTATE_BYTES the file is renamed to
.1 (the old .1 is dropped), so at most about twice that is kept.
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

from .calibration import config_dir

ROTATE_BYTES = 2_000_000
KINDS = ("control", "stop", "program", "policy", "record", "mode", "power", "setup", "roles", "batch", "teach")
_SKIP_ARGS = ("token", "bid", "client", "password", "secret", "data_b64", "text", "yaml", "rev")


def summarize_args(args: Optional[dict], limit: int = 160) -> str:
    """"name=foo mode=admittance": short scalar arguments only, nothing secret-looking."""
    parts = []
    for k, v in (args or {}).items():
        if any(s in str(k).lower() for s in _SKIP_ARGS):
            continue
        if isinstance(v, (str, int, float, bool)) and len(str(v)) <= 40:
            parts.append(f"{k}={v}")
    return " ".join(parts)[:limit]


class ActivityLog:
    def __init__(self, path: Optional[Path] = None, rotate_bytes: int = ROTATE_BYTES):
        self.path = Path(path) if path else config_dir() / "activity" / "activity.jsonl"
        self.rotate_bytes = rotate_bytes
        self._lock = threading.Lock()
        self.seq = 0                                   # +1 per event: the page refetches when it changes
        self.on_error = None                           # called once per failure streak with the error text
        self._failing = False

    def add(self, kind: str, what: str, who: Optional[Dict[str, str]] = None, details: str = "") -> dict:
        who = who or {}
        now = time.time()
        e = {"t": round(now, 3), "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
             "who": str(who.get("name") or ""), "key": str(who.get("key") or ""), "uid": str(who.get("uid") or ""),
             "role": str(who.get("role") or ""), "kind": kind, "what": what, "details": str(details or "")[:200]}
        with self._lock:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                if self.path.exists() and self.path.stat().st_size >= self.rotate_bytes:
                    os.replace(self.path, self.path.with_name(self.path.name + ".1"))
                with open(self.path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(e) + "\n")
            except OSError as err:                         # never break an action over the log, but say so once
                if not self._failing and self.on_error:
                    self.on_error(f"activity log not written ({self.path}): {err}")
                self._failing = True
                return e
            self._failing = False
            self.seq += 1                                  # only advance when the event was written
        return e

    def _read(self, p: Path) -> List[dict]:
        out = []
        try:
            for x in p.read_text(encoding="utf-8").splitlines():
                try:
                    out.append(json.loads(x))
                except ValueError:
                    pass
        except OSError:
            pass
        return out

    def recent(self, n: int = 300, who: str = "", kind: str = "") -> List[dict]:
        """Newest first. who matches the name or the key; kind one of KINDS."""
        with self._lock:
            es = self._read(self.path.with_name(self.path.name + ".1")) + self._read(self.path)
        es = [e for e in es if (not who or who in (e.get("who"), e.get("key"))) and (not kind or e.get("kind") == kind)]
        return list(reversed(es[-n:]))

    def people(self) -> List[str]:
        with self._lock:
            es = self._read(self.path.with_name(self.path.name + ".1")) + self._read(self.path)
        return sorted({e["who"] for e in es if e.get("who")})
