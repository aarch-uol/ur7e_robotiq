"""F/T software bias (SETUP > F/T + Payload): zero, clear or type the bias of the ATI reading. No hardware bias.

The ATI box is never biased (no RDT bias command, no setting.cgi). The bias is the `offset.*` parameters of
force_torque_sensor_broadcaster (ros2_controllers 4.42), which it re-reads every cycle and ADDS to the sensor
values before publishing ~/wrench. That topic is what the GUI shows, what payload calibration samples and what
TopicSensorBridge hands to the admittance controller, so one change applies everywhere, at once.

    published = raw - bias            (bias = -offset)

* Zero now: averages the published wrench for `zero_s` with the arm still, then bias += mean, so the current
  load reads 0. Refused if the reading moves more than `max_spread_*` (someone touching the tool, vibration).
  With a payload loaded on the admittance controller (gravity_compensation.CoG, from the payload calibration),
  Zero keeps that payload in the reading: bias = mean - expected tool weight at this pose, so only the sensor's
  drift is removed. The published wrench then stays raw-like (tool weight + contact, as the payload fit and the
  MuJoCo training see it) and the controller's gravity compensation still removes exactly the weight at every
  pose. Zeroing the weight away as well would make the controller subtract it a second time.
  `plain=True` zeroes everything (the old behaviour; for a sensor without a payload model).
* Clear: bias 0, the raw sensor values (what the payload calibration expects).
* Set: any six values, checked against max_force_n / max_torque_nm.
* Never while something uses the reading: an F/T mode (requires_ft) is active, a force is applied, a program,
  policy, motion or payload calibration runs. A step in the reading would move an admittance-controlled arm.
* Every change is read back. Named sets are saved (config_dir()/ft_bias/<profile>.json) with the last applied
  bias. The broadcaster restarts with its YAML offsets (0) when the robot stack restarts: the GUI puts the last
  bias back by itself as soon as nothing uses the reading (`auto_restore`, default on; checked every poll_s), and
  logs it. With auto_restore: false it only shows that the bias is gone and offers to re-apply it.

Profile (all optional):

    ft_bias:
      node: /ft/force_torque_sensor_broadcaster
      max_force_n: 200
      max_torque_nm: 15
      zero_s: 1.0
      max_spread_n: 1.0
      max_spread_nm: 0.1
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

PARAMS = ["offset.force.x", "offset.force.y", "offset.force.z",
          "offset.torque.x", "offset.torque.y", "offset.torque.z"]
AXES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
DEFAULTS = {"node": "", "auto_restore": True, "max_force_n": 200.0, "max_torque_nm": 15.0, "zero_s": 1.0,
            "max_spread_n": 1.0, "max_spread_nm": 0.1, "sample_period_s": 0.02, "poll_s": 5.0}
EPS = 1e-6


class FtBiasError(ValueError):
    pass


G = 9.80665


def gravity_wrench(R_ref_sensor, weight_n: float, cog_m) -> List[float]:
    """The payload's weight seen by the sensor: [F, cog x F] in the sensor frame, gravity along -Z of the reference
    frame. Same convention as calibration_math.compute_expected_wrench and the admittance controller."""
    R = [[float(v) for v in row[:3]] for row in R_ref_sensor[:3]]
    g = [0.0, 0.0, -abs(float(weight_n))]
    f = [sum(R[r][c] * g[r] for r in range(3)) for c in range(3)]          # R^T @ g
    c = [float(v) for v in cog_m]
    t = [c[1] * f[2] - c[2] * f[1], c[2] * f[0] - c[0] * f[2], c[0] * f[1] - c[1] * f[0]]
    return f + t


def _r(v: List[float]) -> List[float]:
    return [round(float(x), 4) for x in v]


def _fmt(v: List[float]) -> str:
    return (f"F [{', '.join(f'{x:+.2f}' for x in v[:3])}] N, "
            f"T [{', '.join(f'{x:+.3f}' for x in v[3:])}] Nm")


def _same(a, b, tol=1e-3) -> bool:
    return a is not None and b is not None and all(abs(x - y) <= tol for x, y in zip(a, b))


class FtBias:
    def __init__(self, cfg: dict, wrench_topic: str, bridge, log: Callable[[str], None],
                 refusal: Callable[[], Optional[str]], settings_path: Optional[Path] = None):
        self.cfg = {**DEFAULTS, **(cfg or {})}
        if not self.cfg["node"]:
            # the broadcaster node is the topic's namespace: /ft/force_torque_sensor_broadcaster/wrench
            self.cfg["node"] = wrench_topic.rsplit("/", 1)[0] if wrench_topic and wrench_topic.count("/") > 1 else ""
        self.node: str = self.cfg["node"]
        self.bridge, self.log, self.refusal = bridge, log, refusal
        self.settings_path = settings_path
        self.live: Optional[List[float]] = None          # bias on the broadcaster, last read
        self.error = ""
        self.read_at = 0.0
        self.busy = ""                                   # "zeroing" while sampling
        self._lock = threading.Lock()
        self._last_poll = 0.0
        self._polling = False
        self._blocked = ""                               # last reason an automatic restore waited (logged once)
        s = self._load()
        self.presets: Dict[str, dict] = dict(s.get("presets") or {})
        self.last: Optional[dict] = s.get("last")        # {bias, at, how}: the last bias this GUI applied
        # () -> (expected payload wrench at this pose or None, note). None + "" = no payload loaded: zero all.
        # None + a note = a payload is loaded but cannot be predicted now: refuse rather than zero it away.
        self.payload: Optional[Callable[[], Tuple[Optional[List[float]], str]]] = None

    @property
    def available(self) -> bool:
        return bool(self.node)

    # ------------------------------------------------------------------ read
    def read(self) -> Tuple[bool, str]:
        if not self.available:
            return False, "no F/T broadcaster in this profile"
        got = self.bridge.get_node_parameters(self.node, PARAMS)
        if got is None:
            self.live, self.error = None, f"{self.node} is not running (its parameter service did not answer)"
            return False, self.error
        if any(got.get(p) is None for p in PARAMS):
            self.live = None
            self.error = (f"{self.node} has no offset parameters: force_torque_sensor_broadcaster older than "
                          "ros2_controllers 4.x cannot take a software bias")
            return False, self.error
        self.live = [-float(got[p]) + 0.0 for p in PARAMS]
        self.read_at, self.error = time.time(), ""
        return True, "read"

    def poll(self, may_read: bool):
        """Re-read every poll_s in the background (shows a broadcaster restart, which drops the bias)."""
        if not self.available or not may_read or self._polling or time.monotonic() - self._last_poll < self.cfg["poll_s"]:
            return
        self._last_poll = time.monotonic()
        self._polling = True

        def run():
            try:
                self.read()
                if self.cfg.get("auto_restore", True) and self.lost():
                    self.restore()
            finally:
                self._polling = False
        threading.Thread(target=run, daemon=True).start()

    # ----------------------------------------------------------------- checks
    def check(self, bias) -> List[float]:
        try:
            v = [float(x) for x in bias]
        except (TypeError, ValueError):
            raise FtBiasError("the bias is six numbers: Fx Fy Fz in N, Tx Ty Tz in Nm") from None
        if len(v) != 6 or not all(math.isfinite(x) for x in v):
            raise FtBiasError("the bias is six finite numbers: Fx Fy Fz in N, Tx Ty Tz in Nm")
        mf, mt = float(self.cfg["max_force_n"]), float(self.cfg["max_torque_nm"])
        for i, x in enumerate(v):
            lim, unit = (mf, "N") if i < 3 else (mt, "Nm")
            if abs(x) > lim:
                raise FtBiasError(f"{AXES[i]} bias {x:g} {unit} is outside ±{lim:g} {unit} (profile ft_bias)")
        return v

    def why_not(self, refusal: Optional[Callable[[], Optional[str]]] = None) -> Optional[str]:
        if not self.available:
            return "no F/T broadcaster in this profile"
        if self.busy:
            return f"already {self.busy}"
        why = (refusal or self.refusal)()
        return f"not while {why}: the reading would jump under it" if why else None

    # ------------------------------------------------------------------ apply
    def _send(self, bias: List[float], how: str) -> Tuple[bool, str]:
        ok, msg = self.bridge.set_node_parameters(self.node, {p: float(-b) + 0.0 for p, b in zip(PARAMS, bias)})
        if not ok:
            self.read()
            return False, f"F/T bias not set: {msg}"
        self.read()
        if not _same(self.live, bias):
            return False, f"F/T bias read back as {_fmt(self.live) if self.live else 'nothing'}, not {_fmt(bias)}"
        self.last = {"bias": _r(bias), "at": time.strftime("%Y-%m-%d %H:%M:%S"), "how": how}
        self._save()
        raw = all(abs(x) < EPS for x in bias)
        text = "F/T bias cleared: the reading is the raw sensor value" if raw else f"F/T bias set ({how}): {_fmt(bias)}"
        self.log(text)
        return True, text

    def restore(self) -> Tuple[bool, str]:
        """Put the last bias this GUI applied back after the broadcaster restarted (robot stack restart).
        Keeps how it was made (a Zero now stays "payload kept"). Not while anything uses the reading."""
        if not self.lost():
            return False, "nothing to restore"
        why = self.why_not() or self._reading_in_use()
        if why:
            if why != self._blocked:                  # fail closed: wait and retry on the next poll, say so once
                self._blocked = why
                self.log(f"F/T bias restore waits: {why}")
            return False, why
        self._blocked = ""
        last = dict(self.last)
        ok, msg = self._send(list(last["bias"]), last.get("how", "restored"))
        if ok:
            self.last = {**self.last, "at": last.get("at", self.last["at"]),
                         "restored": time.strftime("%Y-%m-%d %H:%M:%S")}
            self._save()
            self.log(f"F/T bias restored after the F/T broadcaster restarted (set {last.get('at')}, "
                     f"{last.get('how')}): {_fmt(last['bias'])}")
        return ok, msg

    def _reading_in_use(self) -> Optional[str]:
        """Why the bias must not be stepped by an automatic restore: only a fresh controller list that shows no F/T
        mode controller active (and a profile to tell which are F/T modes) proves nothing uses the reading."""
        profile = getattr(self.bridge, "profile", None)
        states = self.bridge.list_controllers()
        if profile is None or states is None:
            return "the controller state is unknown (no fresh controller list)"
        for m in profile.modes.values():
            if m.requires_ft and any(states.get(c) == "active" for c in m.activate):
                return f"{m.label} is active and uses the F/T reading"
        return None

    def set(self, bias, how: str = "typed") -> Tuple[bool, str]:
        v = self.check(bias)
        why = self.why_not()
        if why:
            return False, why
        return self._send(v, how)

    def clear(self) -> Tuple[bool, str]:
        return self.set([0.0] * 6, "cleared")

    def zero(self, sample: Callable[[], Tuple[Optional[List[float]], bool]],
             stopped: Callable[[], bool] = lambda: False,
             refusal: Optional[Callable[[], Optional[str]]] = None, plain: bool = False) -> Tuple[bool, str]:
        """Zero at the current load (keeping the controller's payload unless plain). sample() -> (published wrench or None, arm still). refusal: the rule to
        use instead of the GUI's (a program's zero_ft step: its own run must not block it)."""
        refusal = refusal or self.refusal
        why = self.why_not(refusal)
        if why:
            return False, why
        ok, msg = self.read()
        if not ok:
            return False, msg
        start = list(self.live)
        self.busy = "zeroing"
        try:
            got: List[List[float]] = []
            t_end = time.monotonic() + float(self.cfg["zero_s"])
            while time.monotonic() < t_end:
                if stopped():
                    return False, "F/T zero stopped: bias unchanged"
                w, still = sample()
                if not still:
                    return False, "F/T zero refused: the arm moved. Hold it still and try again"
                if w is None:
                    return False, "F/T zero refused: no F/T data"
                got.append([float(x) for x in w])
                time.sleep(float(self.cfg["sample_period_s"]))
            if len(got) < 5:
                return False, "F/T zero refused: too few samples"
            for i in range(6):
                col = [g[i] for g in got]
                spread = max(col) - min(col)
                lim = float(self.cfg["max_spread_n"] if i < 3 else self.cfg["max_spread_nm"])
                if spread > lim:
                    return False, (f"F/T zero refused: {AXES[i]} moved {spread:.3g} {'N' if i < 3 else 'Nm'} while "
                                   f"sampling (limit {lim:g}). Do not touch the tool and try again")
            mean = [sum(g[i] for g in got) / len(got) for i in range(6)]
            exp, note = (None, "") if plain or self.payload is None else self.payload()
            if exp is None and note:
                return False, f"F/T zero refused: {note}"
            new = [start[i] + mean[i] - (exp[i] if exp else 0.0) for i in range(6)]
            try:
                new = self.check(new)
            except FtBiasError as e:
                return False, f"F/T zero refused: {e}"
        finally:
            self.busy = ""
        if refusal():                            # something started while sampling
            return False, f"not while {refusal()}: the reading would jump under it"
        ok, msg = self._send(new, "zeroed, payload kept" if exp else "zeroed")
        return ok, (msg + f". {note}" if ok and exp else msg)

    # ---------------------------------------------------------------- presets
    def save_preset(self, name: str, bias) -> Tuple[bool, str]:
        name = (name or "").strip()
        if not name or len(name) > 40:
            raise FtBiasError("a set needs a name of 1 to 40 characters")
        self.presets[name] = {"bias": _r(self.check(bias)), "at": time.strftime("%Y-%m-%d %H:%M")}
        self._save()
        return True, f"F/T bias set '{name}' saved (not applied)"

    def delete_preset(self, name: str) -> Tuple[bool, str]:
        if self.presets.pop(name, None) is None:
            raise FtBiasError(f"no F/T bias set named '{name}'")
        self._save()
        return True, f"F/T bias set '{name}' deleted"

    def _load(self) -> dict:
        try:
            return json.loads(self.settings_path.read_text()) if self.settings_path else {}
        except (OSError, ValueError):
            return {}

    def _save(self):
        if self.settings_path:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.settings_path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"presets": self.presets, "last": self.last}, indent=2))
            tmp.replace(self.settings_path)

    # ------------------------------------------------------------------- view
    def admittance_refusal(self, payload_loaded: bool) -> Optional[str]:
        """Why an F/T-driven mode must not start now, or None. With a payload loaded on the controller, the reading
        must be tool weight + contact (the controller removes the weight): a bias that was lost (stack restart:
        the sensor's own offset is back) or a Zero all (weight removed twice) would make the arm drift."""
        if not payload_loaded or not self.available:
            return None
        ok, msg = self.read()
        if not ok:
            return f"the F/T bias cannot be read ({msg})"
        if self.lost():
            return ("the F/T bias was lost (the robot stack restarted, the sensor's own offset is back): "
                    "SETUP > F/T + Payload: re-apply it or press Zero now")
        if all(abs(x) < EPS for x in (self.live or [])):
            return "the F/T bias is not set (raw sensor, its offset reads as a push): SETUP > F/T + Payload: Zero now"
        if self.last and self.last.get("how") == "zeroed":
            return ("the last F/T zero removed the tool weight too (Zero all, or a zero before the payload was "
                    "applied): the controller would subtract it twice. SETUP > F/T + Payload: Zero now")
        return None

    def lost(self) -> bool:
        """The last bias this GUI applied is no longer on the broadcaster (it restarted with its YAML)."""
        return bool(self.last and self.live is not None and not _same(self.live, self.last["bias"]))

    def view(self) -> dict:
        raw = self.live is not None and all(abs(x) < EPS for x in self.live)
        return {"available": self.available, "node": self.node, "bias": _r(self.live) if self.live else None,
                "raw": raw, "error": self.error, "busy": self.busy, "last": self.last,
                "lost": self.lost(), "presets": self.presets, "why_not": self.why_not(),
                "limits": [self.cfg["max_force_n"], self.cfg["max_torque_nm"]]}
