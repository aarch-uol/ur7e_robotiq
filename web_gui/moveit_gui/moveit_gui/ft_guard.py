"""F/T limit: warn above a soft limit, stop the GUI's motion above a hard limit.

The check runs on every F/T sample (RosBridge calls feed() from its wrench callback, ~500 Hz), on a short moving
average so a single noisy sample does not trip it. The reading is the one the GUI shows (after the F/T bias), as
|F| (N) and |T| (Nm).

  soft limit  the page shows the reading amber and a warning; nothing stops
  hard limit  the GUI's motion stops at once: MoveIt motion cancelled, program stopped, policy cancelled, jog
              stopped, an applied force zeroed (an admittance controller stays on: it gives way). Once per event:
              it re-arms when the reading is back below the soft limit (or 80 % of the hard one) for 0.5 s.
              In freedrive (the operator's hands on the arm) it only warns.

The stop runs in its own thread, never in the F/T callback. Saved in config_dir()/ft_limit.yaml. Off by default.
It does not replace the robot's own safety (UR force limit, e-stop): it is a GUI-side guard for scraping contacts.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, Optional, Sequence

import yaml

FILTER_N = 5            # samples in the moving average (10 ms at 500 Hz)
REARM_S = 0.5
FIELDS = ("soft_f", "hard_f", "soft_t", "hard_t")   # N, N, Nm, Nm; 0 = that limit off
MAX = {"soft_f": 500.0, "hard_f": 500.0, "soft_t": 50.0, "hard_t": 50.0}


class FtGuardError(ValueError):
    pass


def check_config(d: dict) -> dict:
    out = {"enabled": bool(d.get("enabled", False))}
    for k in FIELDS:
        try:
            v = float(d.get(k, 0) or 0)
        except (TypeError, ValueError):
            raise FtGuardError(f"{k}: a number") from None
        if not math.isfinite(v) or v < 0 or v > MAX[k]:
            raise FtGuardError(f"{k}: 0 to {MAX[k]:g}")
        out[k] = v
    for s, hd, u in (("soft_f", "hard_f", "N"), ("soft_t", "hard_t", "Nm")):
        if out[s] and out[hd] and out[s] >= out[hd]:
            raise FtGuardError(f"the soft limit must be below the hard one ({out[s]:g} >= {out[hd]:g} {u})")
    if out["enabled"] and not (out["hard_f"] or out["hard_t"] or out["soft_f"] or out["soft_t"]):
        raise FtGuardError("set at least one limit")
    return out


class FtGuard:
    def __init__(self, path: Optional[Path], on_hard: Callable[[str], None], log: Callable[[str], None],
                 clock: Callable[[], float] = time.monotonic, threaded: bool = True):
        self.path = Path(path) if path else None
        self.on_hard, self.log, self.clock = on_hard, log, clock
        self.cfg = {"enabled": False, **{k: 0.0 for k in FIELDS}}
        self._lock = threading.Lock()
        self._win: deque = deque(maxlen=FILTER_N)
        self.f = self.t = 0.0                    # filtered |F|, |T|
        self.level = 0                           # 0 ok, 1 over soft, 2 over hard
        self.peak_f = self.peak_t = 0.0
        self.tripped = False                     # hard limit hit, not re-armed yet
        self._calm_since: Optional[float] = None
        self.last_trip: Optional[dict] = None
        self.freedrive = lambda: False           # set by the core
        self.commanding = lambda: True           # set by the core: is the GUI moving the arm (else nothing to stop)
        self._threaded = threaded
        self._go = threading.Event()
        self._why = ""
        if threaded:
            threading.Thread(target=self._worker, name="ft-guard", daemon=True).start()
        self.load()

    # ------------------------------------------------------------ config
    def load(self):
        if not self.path or not self.path.exists():
            return
        try:
            self.cfg = check_config(yaml.safe_load(self.path.read_text()) or {})
        except (OSError, yaml.YAMLError, FtGuardError) as e:
            self.log(f"F/T limit: {self.path} not used ({e}): limit off")

    def save(self, d: dict) -> dict:
        cfg = check_config(d)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(yaml.safe_dump(cfg, sort_keys=True))
            tmp.replace(self.path)
        with self._lock:
            self.cfg = cfg
            self.tripped, self._calm_since, self.level = False, None, 0
        return cfg

    # ------------------------------------------------------------ samples
    def feed(self, w: Optional[Sequence[float]]):
        """One F/T sample (Fx Fy Fz Tx Ty Tz). Cheap; never blocks on the stop."""
        if w is None or len(w) < 6:
            return
        fire = ""
        with self._lock:
            self._win.append(w)
            n = len(self._win)
            m = [sum(s[i] for s in self._win) / n for i in range(6)]
            self.f, self.t = math.hypot(m[0], m[1], m[2]), math.hypot(m[3], m[4], m[5])
            self.peak_f, self.peak_t = max(self.peak_f, self.f), max(self.peak_t, self.t)
            c = self.cfg
            if not c["enabled"]:
                self.level = 0
                return
            hard = (c["hard_f"] and self.f > c["hard_f"]) or (c["hard_t"] and self.t > c["hard_t"])
            soft = (c["soft_f"] and self.f > c["soft_f"]) or (c["soft_t"] and self.t > c["soft_t"])
            self.level = 2 if hard else 1 if soft else 0
            if hard and not self.tripped:
                self.tripped, self._calm_since = True, None
                fire = (f"|F| {self.f:.1f} N > {c['hard_f']:g} N" if c["hard_f"] and self.f > c["hard_f"]
                        else f"|T| {self.t:.2f} Nm > {c['hard_t']:g} Nm")
                self.last_trip = {"at": time.time(), "f": round(self.f, 2), "t": round(self.t, 3), "why": fire,
                                  "stopped": None}
            elif self.tripped:
                calm = not soft and self.f <= 0.8 * (c["hard_f"] or math.inf) and \
                    self.t <= 0.8 * (c["hard_t"] or math.inf)
                if not calm:
                    self._calm_since = None
                elif self._calm_since is None:
                    self._calm_since = self.clock()
                elif self.clock() - self._calm_since >= REARM_S:
                    self.tripped, self._calm_since = False, None
        if fire:
            self._fire(fire)

    def _fire(self, why: str):
        if self.freedrive():
            self.last_trip["stopped"] = False
            self.log(f"F/T HARD LIMIT in freedrive: {why} (only a warning: your hands are on the arm)")
            return
        self.last_trip["stopped"] = bool(self.commanding())
        if self._threaded:
            self._why = why
            self._go.set()
        else:
            self._stop(why)

    def _worker(self):
        while True:
            self._go.wait()
            self._go.clear()
            self._stop(self._why)

    def _stop(self, why: str):
        try:
            self.on_hard(why)
        except Exception as e:  # pragma: no cover - the core logs its own parts
            self.log(f"F/T limit stop failed: {type(e).__name__}: {e}")

    def reset_peak(self):
        with self._lock:
            self.peak_f = self.peak_t = 0.0

    def view(self) -> dict:
        with self._lock:
            return {**self.cfg, "f": round(self.f, 2), "t": round(self.t, 3), "level": self.level,
                    "peak_f": round(self.peak_f, 2), "peak_t": round(self.peak_t, 3), "tripped": self.tripped,
                    "last_trip": dict(self.last_trip) if self.last_trip else None}
