"""SETUP > F/T + payload > Match simulation: map the F/T sensor's axes onto the policy's (sim) axes.

A trained policy observes the F/T as [fx, fy, fz, tx, ty, tz] in the axes the simulation used. When the sensor is
mounted turned (a camera bracket, a different tool), those axes no longer match. Instead of hand-written sign tables
the person in control pushes the tool tip along base +X, +Y and +Z while the robot holds still; each push gives the
sensor's force change s_i. The simulation's answer to the same push (the sim reference, made by
tools/sim_ft_reference.py in the policy repo) is r_i. The fit picks the signed permutation M (one of 48) with
M s_i / |s_i| closest to r_i / |r_i|. The force sign convention is inside M (MuJoCo reports the reaction wrench, so
the sim signs may be opposite to the physical push: that is why the fit matches to the sim, not to physics).

Torques are not assumed to follow the force signs (the FR3's measured config had ft_obs_scale [-1,-1,-1, 1,1,1]).
A push at the tip, away from the sensor, also twists it: the sign of tx and ty comes from comparing M t_i with the
sim torque of the same pushes (each axis from the pushes whose sim torque on it is at least MIN_TORQUE_NM). A push
through the tip hardly twists about z, so tz is not measured: it keeps the robot config's tz sign relative to its
force sign when that is clear, else it takes the tx / ty sign. The runner applies (observations section):

    ft_obs_axes: M          (rows = policy axes, columns = sensor axes; f_policy = M f_sensor, t_policy = M t_sensor)
    ft_obs_scale: [1, 1, 1, s_t, s_t, s_tz]     (forces 1: their signs are in M)

The result is one of the files each profile owns (setups.members(): config_dir()/ft_match/<robot>.json), so it
switches with control like gains and payload. Policy runs write an effective robot config with these keys
(effective_robot_config) and pass that instead of the manifest's file.

Sim reference JSON: {"pushes": {"+X": [6], "+Y": [6], "+Z": [6]}, "no_contact": {"<tilt_deg>": [6], ...}, ...}
"""
from __future__ import annotations

import itertools
import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

AXES = ("+X", "+Y", "+Z")
MIN_PUSH_N = 5.0              # a push weaker than this is refused (too close to the noise)
MAX_ANGLE_DEG = 15.0          # a push more than this off the chosen matrix is refused
MIN_TORQUE_NM = 0.3           # a torque change (or sim torque) under this cannot show its sign
STEADY_S = 1.0                # a push counts when it stays steady this long
STEADY_SPREAD_N = 1.5         # ... within this of its mean (each force axis)
BASELINE_S = 0.5              # the no-contact reading before each push
BASELINE_SPREAD_N = 1.0
RECORD_S = 0.5                # "Record" averages the last this many seconds
REFERENCE_NAME = "sim_ft_reference.json"
NAMES = ("fx", "fy", "fz", "tx", "ty", "tz")


class FtMatchError(ValueError):
    pass


# ----------------------------------------------------------------------------- sim reference
def reference_path(manifest) -> Optional[Path]:
    """The manifest's `sim_ft_reference`, else sim_ft_reference.json next to its robot_config (None: neither)."""
    p = str(getattr(manifest, "sim_ft_reference", "") or "")
    if p:
        return Path(p).expanduser()
    rc = manifest.expanded_files().get("robot_config")
    return Path(rc).with_name(REFERENCE_NAME) if rc else None


def _six(v, what: str) -> List[float]:
    try:
        out = [float(x) for x in v]
    except (TypeError, ValueError):
        raise FtMatchError(f"{what}: not a list of numbers") from None
    if len(out) != 6 or not all(math.isfinite(x) for x in out):
        raise FtMatchError(f"{what}: needs 6 finite numbers [fx, fy, fz, tx, ty, tz]")
    return out


def load_reference(path) -> dict:
    """{"pushes": {axis: [6]}, "no_contact": {tilt: [6]}, "path": str, "meta": {other keys}}. FtMatchError if bad."""
    path = Path(path)
    try:
        d = json.loads(path.read_text())
    except FileNotFoundError:
        raise FtMatchError(f"no sim reference at {path}") from None
    except (OSError, ValueError) as e:
        raise FtMatchError(f"sim reference {path.name} unreadable: {e}") from None
    if not isinstance(d, dict) or not isinstance(d.get("pushes"), dict):
        raise FtMatchError(f"sim reference {path.name}: no 'pushes'")
    pushes = {}
    for ax in AXES:
        if ax not in d["pushes"]:
            raise FtMatchError(f"sim reference {path.name}: no '{ax}' push")
        pushes[ax] = _six(d["pushes"][ax], f"sim reference {path.name} push {ax}")
        if np.linalg.norm(pushes[ax][:3]) < 1e-6:
            raise FtMatchError(f"sim reference {path.name}: the {ax} push has no force")
    nc = {}
    for k, v in (d.get("no_contact") or {}).items():
        nc[str(k)] = _six(v, f"sim reference {path.name} no_contact {k}")
    return {"pushes": pushes, "no_contact": nc, "path": str(path),
            "meta": {k: v for k, v in d.items() if k not in ("pushes", "no_contact")}}


# ----------------------------------------------------------------------------- the fit
def signed_permutations() -> List[np.ndarray]:
    """All 48 3x3 signed permutation matrices (24 rotations and 24 reflections)."""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1.0, -1.0), repeat=3):
            M = np.zeros((3, 3))
            for row, col in enumerate(perm):
                M[row, col] = signs[row]
            out.append(M)
    return out


def _angle(a, b) -> float:
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def best_orthogonal(S: np.ndarray, R: np.ndarray, det: float) -> np.ndarray:
    """The orthogonal Q with det(Q) = det minimising sum |Q s_i - r_i|^2 (rows of S, R; Kabsch)."""
    U, _, Vt = np.linalg.svd(R.T @ S)
    d = np.sign(det) * np.sign(np.linalg.det(U @ Vt))
    return U @ np.diag([1.0, 1.0, d or 1.0]) @ Vt


def explain(M, scale=None) -> List[str]:
    """"policy fx = -sensor fy" for each policy axis: forces, and torques when `scale` (ft_obs_scale) is given."""
    M = np.asarray(M, float)
    sc = [1.0] * 6 if scale is None else [float(x) for x in scale]
    out = []
    for i in range(6 if scale is not None else 3):
        j = int(np.argmax(np.abs(M[i % 3])))
        neg = M[i % 3, j] * sc[i] < 0
        out.append(f"policy {NAMES[i]} = {'-' if neg else ''}sensor {NAMES[j + (3 if i >= 3 else 0)]}")
    return out


def torque_signs(M, deltas: Dict[str, Sequence[float]], ref: Dict[str, Sequence[float]],
                 old_scale=None, min_nm: float = MIN_TORQUE_NM) -> dict:
    """The torque entries of ft_obs_scale from the torque part of the pushes (forces' M already chosen).

    Returns {"scale": [3] | None, "votes": {"tx": [...], "ty": [...]}, "tz": str, "problems", "repeat"}."""
    M = np.asarray(M, float)
    votes, problems, repeat, signs = {}, [], [], {}
    for k, name in ((0, "tx"), (1, "ty")):
        votes[name] = []
        for ax in AXES:
            tr = float(ref[ax][3 + k])
            if abs(tr) < min_nm:
                continue                                  # the sim hardly twists about this axis for this push
            tp = float((M @ np.asarray(deltas[ax], float)[3:6])[k])
            v = {"push": ax, "sim": round(tr, 3), "measured": round(tp, 3),
                 "sign": 0 if abs(tp) < min_nm else (1 if tp * tr > 0 else -1)}
            votes[name].append(v)
            if v["sign"] == 0:
                problems.append(f"the {ax} push changed {name} by {abs(tp):.2f} Nm, under {min_nm:g} Nm: its sign "
                                f"cannot be read. Repeat the {ax} push at the very tip, along the arrow")
                repeat.append(ax)
        if not votes[name]:
            problems.append(f"the sim reference has no {name} torque of {min_nm:g} Nm or more in any push: the {name} "
                            "sign cannot be matched")
            continue
        got = {v["sign"] for v in votes[name] if v["sign"]}
        if len(got) > 1:
            pushes = [v["push"] for v in votes[name]]
            problems.append(f"the {name} sign disagrees between the {' and '.join(pushes)} pushes: repeat them")
            repeat += pushes
        elif got:
            signs[name] = got.pop()
    if "tx" in signs and "ty" in signs and signs["tx"] != signs["ty"]:
        problems.append("tx and ty need opposite signs, which a rigid sensor cannot do: repeat the +X and +Y pushes")
        repeat += ["+X", "+Y"]
    if problems or len(signs) < 2:
        return {"scale": None, "votes": votes, "tz": "", "problems": problems,
                "repeat": [ax for ax in AXES if ax in repeat]}
    st = float(signs["tx"])
    old = [float(x) for x in old_scale] if old_scale is not None and len(old_scale) == 6 else None
    if old and len({math.copysign(1, x) for x in old[:3]}) == 1 and old[5] != 0:
        tz = math.copysign(1, old[5]) * math.copysign(1, old[0])
        how = "tz not measured: its sign relative to the forces kept from the robot config"
    else:
        tz = st
        how = "tz not measured: set like tx and ty"
    return {"scale": [st, st, tz], "votes": votes, "tz": how, "problems": [], "repeat": []}


def fit(deltas: Dict[str, Sequence[float]], ref: Dict[str, Sequence[float]], min_n: float = MIN_PUSH_N,
        max_deg: float = MAX_ANGLE_DEG, old_scale=None, min_nm: float = MIN_TORQUE_NM) -> dict:
    """Match the pushes (sensor-frame wrench changes) to the sim reference.

    Returns {"ok", "M", "scale" (ft_obs_scale), "torque" (torque_signs), "problems", "repeat",
    "pushes": {axis: {"n", "angle_deg"}}, "rotation_deg", "error"}. ok is False (with problems and the pushes to
    repeat) when a push is weak or too far off the chosen M, or its torque sign cannot be read or disagrees.
    `old_scale`: the robot config's ft_obs_scale now (for the tz sign).
    """
    missing = [ax for ax in AXES if deltas.get(ax) is None]
    if missing:
        return {"ok": False, "M": None, "problems": [f"no {ax} push yet" for ax in missing], "repeat": missing,
                "pushes": {}, "rotation_deg": None, "error": None, "scale": None, "torque": None}
    S = np.array([np.asarray(deltas[ax], float)[:3] for ax in AXES])
    Rf = np.array([np.asarray(ref[ax], float)[:3] for ax in AXES])
    mags = np.linalg.norm(S, axis=1)
    weak = [ax for ax, n in zip(AXES, mags) if n < min_n]
    pushes = {ax: {"n": round(float(n), 2), "angle_deg": None} for ax, n in zip(AXES, mags)}
    if weak:
        return {"ok": False, "M": None, "repeat": weak, "pushes": pushes, "rotation_deg": None, "error": None,
                "scale": None, "torque": None,
                "problems": [f"the {ax} push was {pushes[ax]['n']:.1f} N, under {min_n:g} N: repeat it, pushing "
                             "harder" for ax in weak]}
    sh = S / mags[:, None]
    rh = Rf / np.linalg.norm(Rf, axis=1)[:, None]
    best, err = None, None
    for M in signed_permutations():
        e = float(np.sum((sh @ M.T - rh) ** 2))
        if err is None or e < err - 1e-12:
            best, err = M, e
    for ax, s, r in zip(AXES, sh, rh):
        pushes[ax]["angle_deg"] = round(_angle(best @ s, r), 1)
    Q = best_orthogonal(sh, rh, np.linalg.det(best))
    rot = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(best.T @ Q) - 1.0) / 2.0))))
    off = [ax for ax in AXES if pushes[ax]["angle_deg"] > max_deg]
    problems = [f"the {ax} push is {pushes[ax]['angle_deg']:.0f}° off the best match (at most {max_deg:g}°): "
                f"repeat the {ax} push, straight along the arrow" for ax in off]
    tq = torque_signs(best, deltas, ref, old_scale, min_nm) if not off else None
    scale = [1.0, 1.0, 1.0] + tq["scale"] if tq and tq["scale"] else None
    if tq:
        problems += tq["problems"]
        off = [ax for ax in AXES if ax in off or ax in tq["repeat"]]
    return {"ok": not problems, "M": best.astype(int).tolist(), "scale": scale, "torque": tq, "problems": problems,
            "repeat": off, "pushes": pushes, "rotation_deg": round(rot, 1), "error": round(err, 4)}


def apply_axes(M, w6, scale=None) -> List[float]:
    """What the policy observes: [M f, M t] * scale."""
    M = np.asarray(M, float)
    w = np.asarray(w6, float)
    out = np.concatenate([M @ w[:3], M @ w[3:6]])
    if scale is not None:
        out = out * np.asarray(scale, float)
    return [round(float(x), 3) for x in out]


def suggest_tip(M, R_parent_sensor: Optional[np.ndarray] = None) -> dict:
    """The tool frame rotation whose x is the sensor axis mapped to policy x (y: policy y; z = x × y).

    In the sensor frame, and in the TCP's parent frame when R_parent_sensor is given. Shown, never applied."""
    from . import transforms as tf
    M = np.asarray(M, float)
    x, y = M[0], M[1]
    z = np.cross(x, y)
    R = np.column_stack([x, y, z])                      # R_sensor_tip
    T = np.eye(4)
    T[:3, :3] = R
    out = {"rpy_deg_sensor": [round(v, 1) + 0.0 for v in tf.T_to_mm_deg(T)[1]],
           "z_flipped": bool(np.dot(z, M[2]) < 0)}
    if R_parent_sensor is not None:
        T[:3, :3] = np.asarray(R_parent_sensor, float) @ R
        out["rpy_deg_parent"] = [round(v, 1) + 0.0 for v in tf.T_to_mm_deg(T)[1]]
    return out


# ----------------------------------------------------------------------------- detecting a push
class PushCapture:
    """One push: a no-contact baseline first, then the push, taken when it is steady (or on Record).

    feed(t, wrench6) at any rate; `state` is baseline -> push -> done. `delta` is the push minus the baseline.
    `release_from` (the previous push's baseline): the baseline waits until the reading is back near it, so a hand
    still on the tool after the previous push is not read as no contact (its release would look like a push)."""

    def __init__(self, min_n: float = MIN_PUSH_N, steady_s: float = STEADY_S, spread_n: float = STEADY_SPREAD_N,
                 baseline_s: float = BASELINE_S, baseline_spread_n: float = BASELINE_SPREAD_N, release_from=None):
        self.release_from = None if release_from is None else np.asarray(release_from, float)[:6]
        self.min_n, self.steady_s, self.spread_n = min_n, steady_s, spread_n
        self.baseline_s, self.baseline_spread_n = baseline_s, baseline_spread_n
        self.state = "baseline"
        self.samples: deque = deque()
        self.baseline: Optional[np.ndarray] = None
        self.delta: Optional[np.ndarray] = None
        self.push: Optional[np.ndarray] = None
        self.note = "Hands off the tool: reading the no-contact baseline"
        self.now = np.zeros(6)

    def _window(self, t: float, length: float) -> np.ndarray:
        return np.array([w for ts, w in self.samples if ts >= t - length])

    def _span(self) -> float:
        return self.samples[-1][0] - self.samples[0][0] if self.samples else 0.0

    def feed(self, t: float, w) -> str:
        if self.state == "done" or w is None:
            return self.state
        w = np.asarray(w, float)[:6]
        self.samples.append((t, w))
        keep = max(self.steady_s, self.baseline_s, RECORD_S) + 0.2
        while self.samples and self.samples[0][0] < t - keep:
            self.samples.popleft()
        if self.state == "baseline":
            if self.release_from is not None and np.linalg.norm(w[:3] - self.release_from[:3]) > self.min_n / 2:
                self.note = "Let go of the tool"
                self.samples.clear()
                return self.state
            if self._span() < self.baseline_s * 0.9:
                return self.state
            win = self._window(t, self.baseline_s)
            if np.max(np.abs(win[:, :3] - win[:, :3].mean(0))) > self.baseline_spread_n:
                self.note = "The reading moves: keep hands off the tool for the baseline"
                return self.state
            self.baseline = win.mean(0)
            self.state, self.note = "push", "Push now"
            self.samples.clear()
            return self.state
        self.now = w - self.baseline
        win = self._window(t, self.steady_s)
        f = win[:, :3] - self.baseline[:3]
        n = float(np.linalg.norm(self.now[:3]))
        self.note = f"Push harder: {n:.1f} N (at least {self.min_n:g} N)" if n < self.min_n else "Hold it"
        if len(win) < 2 or self._span() < self.steady_s * 0.9 or np.min(np.linalg.norm(f, axis=1)) < self.min_n:
            return self.state
        if np.max(np.abs(f - f.mean(0))) > self.spread_n:
            self.note = "Hold the push steady"
            return self.state
        self._take(win)
        return self.state

    def record(self, t: float) -> bool:
        """Take the push now (the last RECORD_S averaged). False if there is no baseline or no samples yet."""
        if self.state != "push":
            return False
        win = self._window(t, RECORD_S)
        if not len(win):
            return False
        self._take(win)
        return True

    def _take(self, win: np.ndarray):
        self.push = win.mean(0)
        self.delta = self.push - self.baseline
        self.state, self.note = "done", f"Recorded {np.linalg.norm(self.delta[:3]):.1f} N"

    def view(self) -> dict:
        r = lambda v: None if v is None else [round(float(x), 3) for x in v]   # noqa: E731
        return {"state": self.state, "note": self.note, "baseline": r(self.baseline), "push": r(self.push),
                "delta": r(self.delta), "now_n": round(float(np.linalg.norm(self.now[:3])), 2)}


# ----------------------------------------------------------------------------- the saved result
class FtMatchStore:
    """config_dir()/ft_match/<robot>.json: the result in use (a member of each profile's settings)."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._cache = (None, None)

    def load(self) -> Optional[dict]:
        try:
            st = self.path.stat().st_mtime_ns
        except OSError:
            return None
        if self._cache[0] != st:
            try:
                d = json.loads(self.path.read_text())
                d = d if isinstance(d, dict) and d.get("ft_obs_axes") else None
            except (OSError, ValueError):
                d = None
            self._cache = (st, d)
        return self._cache[1]

    def save(self, result: dict):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(result, indent=1))
        tmp.replace(self.path)

    def clear(self) -> bool:
        try:
            self.path.unlink()
            return True
        except FileNotFoundError:
            return False


def store_path(cfg: Path, robot: str) -> Path:
    return Path(cfg) / "ft_match" / f"{robot}.json"


# ----------------------------------------------------------------------------- policy runs
OBS_SECTION = "observations"
KEEP_EFFECTIVE = 20


def read_obs(robot_config) -> dict:
    """The observations section of a runner robot config ({} if unreadable)."""
    try:
        d = yaml.safe_load(Path(robot_config).read_text()) or {}
        return dict(d.get(OBS_SECTION) or {})
    except Exception:
        return {}


def effective_robot_config(src, result: dict, dest_dir: Path, policy: str, owner: str = "") -> Path:
    """The manifest's robot config with ft_obs_axes and ft_obs_scale from `result`, written under dest_dir."""
    src = Path(src)
    d = yaml.safe_load(src.read_text()) or {}
    if not isinstance(d, dict):
        raise FtMatchError(f"{src.name}: not a YAML mapping")
    obs = dict(d.get(OBS_SECTION) or {})
    obs["ft_obs_axes"] = [[int(x) for x in row] for row in result["ft_obs_axes"]]
    obs["ft_obs_scale"] = [float(x) for x in result.get("ft_obs_scale") or [1.0] * 6]
    d[OBS_SECTION] = obs
    dest_dir = Path(dest_dir)
    dest_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in policy) or "policy"
    out = dest_dir / f"{safe}_{stamp}_{src.stem}.yaml"
    n = 1
    while out.exists():
        n += 1
        out = dest_dir / f"{safe}_{stamp}_{n}_{src.stem}.yaml"
    head = (f"# Effective robot config written by moveit_gui for policy '{policy}'.\n"
            f"# From {src}; observations.ft_obs_axes and ft_obs_scale from the Match simulation result"
            f"{' of profile ' + repr(owner) if owner else ''} ({result.get('at', '')}).\n")
    out.write_text(head + yaml.safe_dump(d, sort_keys=False))
    old = sorted(dest_dir.glob("*.yaml"), key=lambda p: p.stat().st_mtime)
    for p in old[:-KEEP_EFFECTIVE]:
        try:
            p.unlink()
        except OSError:
            pass
    return out
