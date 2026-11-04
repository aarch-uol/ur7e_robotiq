"""Controller gains (SETUP > Controller gains): read, check, ramp in and read back admittance / impedance parameters.

The rows each mode exposes come from the profile (`gains:`), so the GUI never guesses parameter names:

    gains:
      admittance:
        node: /admittance_controller
        rows:
          - {key: stiffness, label: Stiffness, param: admittance.stiffness, size: 6, min: 0, max: 3000}
          - {key: axes, label: Compliant axes, param: admittance.selected_axes, size: 6, type: bool, live: false}
      impedance:
        node: /cartesian_impedance_controller
        enable: {stiffness.update_stiffness: true}       # set first: the controller ignores runtime values without it
        rows:
          - {key: k_trans, label: Stiffness, params: [stiffness.translation.x, ...], unit: N/m, min: 0, max: 1500}
          - {key: dtau, label: Torque rate, param: delta_tau_max, readonly: "read at configure: edit the YAML"}

A row is one array parameter (`param` + `size`) or a list of scalar parameters (`params`). Rules:

* Values are checked against the profile's min/max before anything is sent (the controller has its own, wider,
  bounds and would refuse some values half way through a set).
* Outside the mode, values are set in one call; the controller uses them on its next activation.
* While the mode is ACTIVE the change is ramped (ramp_s, 10 Hz) so no gain steps, each value may change at most
  LIVE_MAX_FACTOR x per apply, `live: false` rows (e.g. which axes are compliant) are refused, and nothing may be
  running: no program, policy, motion or applied force. The impedance controller re-applies its parameter wrench
  (zero) on every parameter change, so a streamed force would drop for a moment.
* STOP during a ramp leaves the gains where the ramp was and says so.
* Every apply is read back; a value the controller did not take is reported, not hidden.

Saved values: every successful apply is also saved (config_dir()/gains/<profile>.json, with the presets) and put
back on the controller automatically, so nobody copies YAML: at GUI start and whenever the controller is found
with other values (it was restarted) while its mode is NOT active, and just before switching into the mode
(ControlModeManager.before_activate). An active controller is never changed behind the operator's back: a
mismatch there is shown, with a button to restore (ramped, like any live apply). "Forget saved" drops them; the
controller then keeps its current values until it restarts with its YAML.
"""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

RAMP_HZ = 10.0
DEFAULT_RAMP_S = 1.0
LIVE_MAX_FACTOR = 2.0
AXES6 = ["X", "Y", "Z", "Rx", "Ry", "Rz"]


class GainsError(ValueError):
    pass


@dataclass
class GainRow:
    key: str
    label: str
    names: List[str]                 # ROS parameter names: one array parameter, or one scalar per axis
    array: bool                      # True: names[0] is an array parameter of `size` elements
    size: int
    kind: str = "float"              # float | bool
    unit: str = ""
    axes: List[str] = field(default_factory=list)
    lo: List[float] = field(default_factory=list)
    hi: List[float] = field(default_factory=list)
    live: bool = True                # may change while the mode is active
    readonly: str = ""               # shown, never sent (why)
    help: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "GainRow":
        if "param" in d:
            names, array, size = [str(d["param"])], int(d.get("size", 1)) > 1, int(d.get("size", 1))
        else:
            names = [str(n) for n in d["params"]]
            array, size = False, len(names)
        axes = [str(a) for a in d.get("axes", [])] or (AXES6[:size] if size in (3, 6) else
                                                         [""] if size == 1 else [str(i + 1) for i in range(size)])
        if size == 3 and not d.get("axes") and any("rotation" in n for n in names):
            axes = ["Rx", "Ry", "Rz"]

        def per_axis(v, default):
            v = default if v is None else v
            return [float(x) for x in v] if isinstance(v, (list, tuple)) else [float(v)] * size
        row = cls(key=str(d["key"]), label=str(d.get("label", d["key"])), names=names, array=array, size=size,
                  kind=str(d.get("type", "float")), unit=str(d.get("unit", "")), axes=axes,
                  lo=per_axis(d.get("min"), -math.inf), hi=per_axis(d.get("max"), math.inf),
                  live=bool(d.get("live", True)), readonly=str(d.get("readonly", "") or ""), help=str(d.get("help", "")))
        if len(row.lo) != size or len(row.hi) != size or len(row.axes) != size:
            raise GainsError(f"gains row '{row.key}': min / max / axes need {size} values")
        return row

    # values <-> ROS parameters
    def to_params(self, values: List) -> Dict[str, object]:
        conv = (lambda v: bool(v)) if self.kind == "bool" else (lambda v: float(v))
        if self.array:
            return {self.names[0]: [conv(v) for v in values]}
        return {n: conv(v) for n, v in zip(self.names, values)}

    def from_params(self, got: Dict[str, object]) -> Optional[List]:
        if self.array:
            v = got.get(self.names[0])
            if v is None:
                return None
            v = list(v) if isinstance(v, (list, tuple)) else [v]
            return v if len(v) == self.size else None
        vals = [got.get(n) for n in self.names]
        return None if any(v is None for v in vals) else vals

    def check(self, values) -> List:
        if not isinstance(values, (list, tuple)) or len(values) != self.size:
            raise GainsError(f"{self.label}: needs {self.size} value(s)")
        if self.kind == "bool":
            if not all(isinstance(v, bool) for v in values):
                raise GainsError(f"{self.label}: values must be true / false")
            return list(values)
        out = []
        for axis, v, lo, hi in zip(self.axes, values, self.lo, self.hi):
            try:
                f = float(v)
            except (TypeError, ValueError):
                raise GainsError(f"{self.label} {axis}: '{v}' is not a number") from None
            if not math.isfinite(f) or not (lo <= f <= hi):
                raise GainsError(f"{self.label} {axis}: {f:g} is outside {lo:g} … {hi:g}")
            out.append(f)
        return out

    def view(self) -> dict:
        fin = lambda xs: [x if math.isfinite(x) else None for x in xs]   # noqa: E731
        return {"key": self.key, "label": self.label, "unit": self.unit, "axes": self.axes, "kind": self.kind,
                "min": fin(self.lo), "max": fin(self.hi), "live": self.live, "readonly": self.readonly,
                "help": self.help, "params": self.names}


@dataclass
class GainSet:
    mode: str
    node: str
    rows: List[GainRow]
    enable: Dict[str, object] = field(default_factory=dict)
    ramp_s: float = DEFAULT_RAMP_S
    note: str = ""

    @classmethod
    def from_dict(cls, mode: str, d: dict) -> "GainSet":
        if not d.get("node"):
            raise GainsError(f"gains.{mode}: 'node' is required")
        rows = [GainRow.from_dict(r) for r in d.get("rows", []) or []]
        if len({r.key for r in rows}) != len(rows):
            raise GainsError(f"gains.{mode}: row keys must be unique")
        return cls(mode=mode, node=str(d["node"]), rows=rows, enable=dict(d.get("enable") or {}),
                   ramp_s=float(d.get("ramp_s", DEFAULT_RAMP_S)), note=str(d.get("note", "")))

    def row(self, key: str) -> GainRow:
        for r in self.rows:
            if r.key == key:
                return r
        raise GainsError(f"{self.mode}: no gain row '{key}'")

    def all_names(self) -> List[str]:
        return [n for r in self.rows for n in r.names] + list(self.enable)


def _same(a, b) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    try:
        return abs(float(a) - float(b)) <= 1e-9 * max(1.0, abs(float(a)))
    except (TypeError, ValueError):
        return a == b


def _nest(flat: Dict[str, object]) -> dict:
    out: dict = {}
    for k, v in flat.items():
        d = out
        parts = k.split(".")
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = v
    return out


def _yaml(d: dict, indent: int = 0) -> List[str]:
    lines = []
    for k, v in d.items():
        pad = "  " * indent
        if isinstance(v, dict):
            lines.append(f"{pad}{k}:")
            lines += _yaml(v, indent + 1)
        elif isinstance(v, list):
            lines.append(f"{pad}{k}: [{', '.join(_fmt(x) for x in v)}]")
        else:
            lines.append(f"{pad}{k}: {_fmt(v)}")
    return lines


def _fmt(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(round(v, 6)) if v != int(v) else f"{v:.1f}"
    return str(v)


class Gains:
    """Controller gains of every profile mode that declares them."""

    def __init__(self, sets: Dict[str, GainSet], bridge, log: Callable[[str], None],
                 current_mode: Callable[[], str], refusal: Callable[[], Optional[str]],
                 settings_path: Optional[Path] = None):
        self.sets = sets
        self.bridge, self.log = bridge, log
        self.current_mode = current_mode
        self.refusal = refusal                       # why nothing may be applied live right now (None: free)
        self.settings_path = settings_path
        self.live: Dict[str, Dict[str, List]] = {}       # mode -> row key -> values, last read
        self.baseline: Dict[str, Dict[str, List]] = {}   # mode -> first values read this session
        self.read_at: Dict[str, float] = {}
        self.error: Dict[str, str] = {}
        self.applying: Optional[dict] = None             # {mode, step, steps}
        self._lock = threading.Lock()
        settings = self._load_settings()
        self.presets: Dict[str, dict] = dict(settings.get("presets") or {})
        # mode -> {parameter: value}: put back on the controller automatically (see the module doc)
        self.saved: Dict[str, Dict[str, object]] = {m: dict(v) for m, v in (settings.get("saved") or {}).items()
                                                    if m in sets}
        self.saved_at: Dict[str, str] = dict(settings.get("saved_at") or {})
        self.unsynced: Dict[str, str] = {}               # mode -> why the controller differs from the saved values
        self.restored_at: Dict[str, float] = {}
        self._last_poll = 0.0
        self._polling = False
        self._blocked: Dict[str, str] = {}               # mode -> last reason an automatic restore waited (logged once)

    @classmethod
    def from_profile(cls, profile_gains: dict, *args, **kw) -> "Gains":
        sets = {mode: GainSet.from_dict(mode, d) for mode, d in (profile_gains or {}).items()}
        return cls(sets, *args, **kw)

    def gain_set(self, mode: str) -> GainSet:
        if mode not in self.sets:
            raise GainsError(f"mode '{mode}' has no tunable gains in this profile")
        return self.sets[mode]

    # ------------------------------------------------------------------ read
    def read(self, mode: str) -> Tuple[bool, str]:
        gs = self.gain_set(mode)
        got = self.bridge.get_node_parameters(gs.node, gs.all_names())
        if got is None:
            self.error[mode] = f"{gs.node} is not running (its parameter service did not answer)"
            return False, self.error[mode]
        vals, missing = {}, []
        for r in gs.rows:
            v = r.from_params(got)
            if v is None:
                missing.append(r.key)
            else:
                vals[r.key] = [bool(x) for x in v] if r.kind == "bool" else [float(x) for x in v]
        self.live[mode] = vals
        self.live[mode + "/enable"] = {k: got.get(k) for k in gs.enable}   # type: ignore[assignment]
        self.baseline.setdefault(mode, json.loads(json.dumps(vals)))
        self.read_at[mode] = time.time()
        self.error[mode] = (f"{gs.node} has no parameter(s) for: {', '.join(missing)}" if missing else "")
        return True, "read" + (f" ({self.error[mode]})" if missing else "")

    # ----------------------------------------------------------------- apply
    def plan(self, mode: str, values: Dict[str, List]) -> Tuple[Dict[str, List], Dict[str, List], List[str]]:
        """Check a request against the profile and the live values. Returns (old, new, warnings) of changed rows."""
        gs = self.gain_set(mode)
        if mode not in self.live:
            raise GainsError("read the controller first")
        old, new, warn = {}, {}, []
        for key, v in (values or {}).items():
            r = gs.row(key)
            if r.readonly:
                raise GainsError(f"{r.label} cannot be set here: {r.readonly}")
            v = r.check(v)
            cur = self.live[mode].get(key)
            if cur is None:
                raise GainsError(f"{r.label}: the controller did not report it")
            if all(_same(a, b) for a, b in zip(cur, v)):
                continue
            old[key], new[key] = cur, v
        active = self.current_mode() == mode
        if active and new:
            why = self.refusal()
            if why:
                raise GainsError(f"not while {why}: the controller is active")
            for key in new:
                r = gs.row(key)
                if not r.live:
                    raise GainsError(f"{r.label} changes only outside {mode}: switch to another mode first")
                for axis, a, b in zip(r.axes, old[key], new[key]):
                    if a > 0 and b > 0 and max(a / b, b / a) > LIVE_MAX_FACTOR + 1e-9:
                        raise GainsError(f"{r.label} {axis}: {a:g} → {b:g} is more than {LIVE_MAX_FACTOR:g}× while "
                                         f"{mode} is active: change it in steps, or outside {mode}")
            warn.append(f"ramped over {gs.ramp_s:g} s: {mode} is active")
        return old, new, warn

    def apply(self, mode: str, values: Dict[str, List], stopped: Callable[[], bool] = lambda: False
              ) -> Tuple[bool, str]:
        if not self._lock.acquire(blocking=False):
            return False, "another gain change is still running"
        try:
            gs = self.gain_set(mode)
            ok, msg = self.read(mode)                       # plan against what the controller has right now
            if not ok:
                return False, msg
            old, new, _ = self.plan(mode, values)
            if not new:
                return True, "nothing changed"
            enable = self.live.get(mode + "/enable") or {}
            flags = {k: v for k, v in gs.enable.items() if not _same(enable.get(k), v)}
            if flags:                                       # first, alone: they re-apply the current values only
                ok, msg = self.bridge.set_node_parameters(gs.node, flags)
                if not ok:
                    return False, f"{gs.node}: {msg}"
            active = self.current_mode() == mode
            steps = max(1, int(round(gs.ramp_s * RAMP_HZ))) if active else 1
            for i in range(1, steps + 1):
                if active and (stopped() or self.current_mode() != mode):
                    self.read(mode)
                    return False, (f"gain ramp stopped at step {i - 1}/{steps}"
                                   f"{' (STOP)' if stopped() else ' (mode changed)'}: the controller keeps the "
                                   "values it had reached")
                if active and (why := self.refusal()):
                    self.read(mode)
                    return False, f"gain ramp stopped at step {i - 1}/{steps}: {why}"
                self.applying = {"mode": mode, "step": i, "steps": steps}
                a = i / steps
                params: Dict[str, object] = {}
                for key, b in new.items():
                    r = gs.row(key)
                    vals = b if (a >= 1 or r.kind == "bool") else [x + (y - x) * a for x, y in zip(old[key], b)]
                    params.update(r.to_params(vals))
                ok, msg = self.bridge.set_node_parameters(gs.node, params)
                if not ok:
                    self.read(mode)
                    return False, f"{gs.node} refused the change at step {i}/{steps}: {msg}"
                if i < steps:
                    time.sleep(1.0 / RAMP_HZ)
            self.read(mode)
            bad = [f"{gs.row(k).label} (read back {self.live[mode].get(k)})" for k, v in new.items()
                   if not all(_same(a, b) for a, b in zip(self.live[mode].get(k) or [], v))]
            if bad:
                return False, f"{gs.node} did not take: {', '.join(bad)}"
            what = "; ".join(f"{gs.row(k).label} {self._short(old[k])} → {self._short(v)}" for k, v in new.items())
            when = "now" if active else f"on the next activation of {mode}"
            saved: Dict[str, object] = dict(gs.enable)
            for key, v in new.items():
                saved.update(gs.row(key).to_params(v))
            self.remember(mode, saved)
            self.log(f"gains {mode}: {what} (in effect {when}; saved)")
            return True, f"{what}. In effect {when}. Saved: restored automatically after a controller restart."
        finally:
            self.applying = None
            self._lock.release()

    @staticmethod
    def _short(v: List) -> str:
        if v and isinstance(v[0], bool):
            return "".join("✓" if x else "·" for x in v)
        return "[" + ", ".join(f"{x:g}" for x in v) + "]" if len(v) > 1 else f"{v[0]:g}"

    # -------------------------------------------------------------- policies
    def compare(self, mode: str, expected: Dict[str, object]) -> Tuple[Optional[bool], str]:
        """Does the live controller match parameter values a policy was trained with? (None: unknown)"""
        gs = self.sets.get(mode)
        if gs is None:
            return False, f"the profile has no gains for '{mode}'"
        if mode not in self.live or time.time() - self.read_at.get(mode, 0) > 5.0:
            ok, msg = self.read(mode)
            if not ok:
                return None, msg
        flat: Dict[str, object] = {}
        for r in gs.rows:
            v = self.live[mode].get(r.key)
            if v is not None:
                flat.update(r.to_params(v))
        diffs = []
        for name, want in expected.items():
            have = flat.get(name)
            if have is None:
                diffs.append(f"{name} is not a tunable gain here")
                continue
            w = list(want) if isinstance(want, (list, tuple)) else [want]
            h = list(have) if isinstance(have, (list, tuple)) else [have]
            if len(w) != len(h) or not all(_close(a, b) for a, b in zip(h, w)):
                diffs.append(f"{name} {self._short(h)} (trained {self._short(w)})")
        return (not diffs), ("; ".join(diffs[:3]) + (f" … +{len(diffs) - 3}" if len(diffs) > 3 else "")
                             if diffs else "match the trained values")

    def values_from_params(self, mode: str, flat: Dict[str, object]) -> Dict[str, List]:
        """Row values for a {parameter: value} dict (a policy's trained gains), filled from the live values."""
        gs = self.gain_set(mode)
        if mode not in self.live:
            raise GainsError("read the controller first")
        out = {}
        for r in gs.rows:
            if r.readonly or not any(n in flat for n in r.names):
                continue
            cur = list(self.live[mode][r.key])
            if r.array:
                out[r.key] = list(flat[r.names[0]])
            else:
                out[r.key] = [flat.get(n, c) for n, c in zip(r.names, cur)]
        return out

    # ------------------------------------------------------------------ yaml
    def yaml_snippet(self, mode: str, values: Optional[Dict[str, List]] = None) -> str:
        gs = self.gain_set(mode)
        values = values or self.live.get(mode) or {}
        flat: Dict[str, object] = {}
        for r in gs.rows:
            if r.key in values and not r.readonly:
                flat.update(r.to_params(values[r.key]))
        flat.update(gs.enable)
        return "\n".join(_yaml({gs.node.strip("/").split("/")[-1]: {"ros__parameters": _nest(flat)}}))

    # ------------------------------------------------------------ saved values
    POLL_S = 3.0

    def _load_settings(self) -> dict:
        try:
            return json.loads(self.settings_path.read_text()) if self.settings_path else {}
        except (OSError, ValueError):
            return {}

    def _save_presets(self):
        if self.settings_path:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.settings_path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"presets": self.presets, "saved": self.saved, "saved_at": self.saved_at},
                                      indent=2))
            tmp.replace(self.settings_path)

    def remember(self, mode: str, params: Dict[str, object]):
        """Keep these parameter values for `mode` and restore them from now on (also used by the payload apply)."""
        self.gain_set(mode)
        self.saved.setdefault(mode, {}).update({k: (list(v) if isinstance(v, (list, tuple)) else v)
                                                for k, v in params.items()})
        self.saved_at[mode] = time.strftime("%Y-%m-%d %H:%M")
        self.unsynced.pop(mode, None)
        self._save_presets()

    def forget(self, mode: str) -> Tuple[bool, str]:
        if not self.saved.pop(mode, None):
            return True, "nothing saved"
        self.saved_at.pop(mode, None)
        self.unsynced.pop(mode, None)
        self._save_presets()
        return True, (f"saved {mode} gains forgotten: the controller keeps its current values until it restarts "
                      "with its YAML")

    def _differences(self, mode: str) -> Optional[Dict[str, object]]:
        """Saved values the controller does not have now (None: could not read the node)."""
        want = self.saved.get(mode) or {}
        if not want:
            return {}
        gs = self.gain_set(mode)
        names = list(want) + [k for k in gs.enable if k not in want]
        got = self.bridge.get_node_parameters(gs.node, names)
        if got is None:
            return None
        diff = {k: v for k, v in want.items() if not _eq(got.get(k), v)}
        if diff:      # the runtime flags must be on for the impedance controller to use them
            diff.update({k: v for k, v in gs.enable.items() if not _eq(got.get(k), v)})
        return diff

    def _not_confirmed_inactive(self, mode: str) -> Optional[str]:
        """None only if a fresh controller list says the mode's controller is inactive; else why we can't tell."""
        name = self.sets[mode].node.rstrip("/").rsplit("/", 1)[-1]
        states = self.bridge.list_controllers()
        if states is None:
            return "the controller list is unavailable, so it is not known that the controller is inactive"
        state = states.get(name)
        if state != "inactive":
            return f"controller {name} is {state or 'not listed'}, not confirmed inactive"
        return None

    def restore(self, mode: str, why: str) -> Tuple[bool, str]:
        """Put the saved values back on an INACTIVE controller. Active: refused (the operator decides)."""
        if mode not in self.sets or not self.saved.get(mode):
            return True, "nothing saved"
        if self.current_mode() == mode:
            return False, f"{mode} is active: restore it from SETUP > Controller gains"
        why_not = self._not_confirmed_inactive(mode)
        if why_not:                                   # fail closed: unknown / stale state is not "inactive"
            if self._blocked.get(mode) != why_not:
                self._blocked[mode] = why_not
                self.log(f"gains {mode}: restore waits ({why_not})")
            return False, why_not
        self._blocked.pop(mode, None)
        if not self._lock.acquire(blocking=False):
            return False, "a gain change is running"
        try:
            diff = self._differences(mode)
            if diff is None:
                return False, f"{self.sets[mode].node} did not answer"
            if not diff:
                self.unsynced.pop(mode, None)
                return True, "already set"
            gs = self.sets[mode]
            flags = {k: v for k, v in diff.items() if k in gs.enable}
            rest = {k: v for k, v in diff.items() if k not in gs.enable}
            for part in (flags, rest):
                if part:
                    ok, msg = self.bridge.set_node_parameters(gs.node, part)
                    if not ok:
                        self.unsynced[mode] = f"restore failed: {msg}"
                        return False, msg
            left = self._differences(mode)
            if left:
                self.unsynced[mode] = f"the controller did not take: {', '.join(left)}"
                return False, self.unsynced[mode]
            self.unsynced.pop(mode, None)
            self.restored_at[mode] = time.time()
            self.log(f"gains {mode}: restored {len(rest)} saved value(s) on {gs.node} ({why})")
            return True, f"restored {len(rest)} saved value(s)"
        finally:
            self._lock.release()

    def poll(self, may_change: bool):
        """Called at the UI rate: every POLL_S, compare / restore saved values in the background."""
        now = time.monotonic()
        if self._polling or now - self._last_poll < self.POLL_S or not any(self.saved.values()):
            return
        self._last_poll = now
        self._polling = True

        def run():
            try:
                for mode in [m for m, v in self.saved.items() if v]:
                    if self.current_mode() == mode or not may_change:
                        diff = self._differences(mode)
                        if diff:
                            self.unsynced[mode] = ("the active controller differs from the saved values: "
                                                   + ", ".join(k for k in diff if k not in self.sets[mode].enable))
                        elif diff == {}:
                            self.unsynced.pop(mode, None)
                    else:
                        ok, _ = self.restore(mode, "controller (re)started")
                        if ok and mode in self.live:
                            self.read(mode)
            except Exception as e:                       # never let a poll thread die silently
                self.log(f"gains poll: {type(e).__name__}: {e}")
            finally:
                self._polling = False
        threading.Thread(target=run, daemon=True).start()

    # --------------------------------------------------------------- presets

    def save_preset(self, name: str, mode: str, values: Dict[str, List]) -> Tuple[bool, str]:
        name = (name or "").strip()
        if not name or len(name) > 40:
            raise GainsError("a preset needs a name of 1-40 characters")
        gs = self.gain_set(mode)
        clean = {k: gs.row(k).check(v) for k, v in (values or {}).items() if not gs.row(k).readonly}
        self.presets[name] = {"mode": mode, "values": clean, "saved": time.strftime("%Y-%m-%d %H:%M")}
        self._save_presets()
        return True, f"preset '{name}' saved (not applied)"

    def delete_preset(self, name: str) -> Tuple[bool, str]:
        if self.presets.pop(name, None) is None:
            raise GainsError(f"no preset '{name}'")
        self._save_presets()
        return True, f"preset '{name}' deleted"

    # ------------------------------------------------------------------ view
    def view(self) -> dict:
        cur = self.current_mode()
        return {"modes": {m: {"node": gs.node, "note": gs.note, "ramp_s": gs.ramp_s, "active": cur == m,
                              "rows": [r.view() for r in gs.rows], "live": self.live.get(m),
                              "baseline": self.baseline.get(m), "read_at": self.read_at.get(m),
                              "error": self.error.get(m, ""), "saved": self.saved.get(m) or {},
                              "saved_at": self.saved_at.get(m), "unsynced": self.unsynced.get(m, ""),
                              "restored_at": self.restored_at.get(m)}
                          for m, gs in self.sets.items()},
                "presets": self.presets, "applying": self.applying, "max_factor": LIVE_MAX_FACTOR}


def _eq(a, b) -> bool:
    if isinstance(a, (list, tuple)) or isinstance(b, (list, tuple)):
        a, b = list(a or []), list(b or [])
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a is not None and _same(a, b)


def _close(a, b, rel=0.02) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    a, b = float(a), float(b)
    return abs(a - b) <= max(1e-6, rel * max(abs(a), abs(b)))
