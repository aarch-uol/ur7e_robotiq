"""Record and replay: a joint trajectory taught by hand, run back as it was moved (program step `replay`).

PROGRAMS > Teach & more > Teach by hand, Path, "Insert as Replay". While recording, the joint states are sampled
(teach.py, about every 10 ms, with the joint-state time stamps). On Insert:

1. resample and smooth: the samples are put on a regular RESAMPLE_S grid and smoothed with a Gaussian of
   SMOOTH_S (time-stamp jitter of the joint states would otherwise read as accelerations; a hand-guided motion has
   nothing that fast in it, and the path moves by about a * SMOOTH_S^2 / 2, 0.2 mrad at 1 rad/s^2);
2. trim: the standstill before the arm first moved and after it last moved is cut off (the time spent pressing
   Record / Finish);
3. decimate: samples are dropped where the motion is a straight line in (time, joints): every recorded sample stays
   within JOINT_TOL_RAD (per joint) of the linear interpolation between the kept points around it (RDP over time,
   like teach.py does for the TCP). A pause in the motion keeps its start and end, so the timing is kept;
4. stored in the step as `points: [[t s, q1 .. qn rad], ...]` (t from 0, profile joint order).

Running it (ProgramRunner, program.py):

* the arm first gets to the first point with an ordinary planned MoveJ (the step's `velocity`), unless it is there;
* checks, refused before anything moves: joint position limits (URDF), the keep-out zones (the TCP and every joint
  origin of the arm at every point, by forward kinematics);
* the timing is the recorded one divided by `speed` (1.0 = as taught) and the GUI speed override. Velocities are
  central differences (zero at both ends), so the trajectory controller runs cubic splines through positions and
  velocities. If the splines would exceed a joint velocity (URDF) or acceleration limit (profile
  moveit.joint_accel_limit_rad_s2), the whole replay is slowed down uniformly until they do not (fit_limits);
* it is sent as ONE trajectory through the active mode's controller (bridge.execute_joint_path, the same path as a
  planned MoveIt trajectory: STOP / pause cancel it). A pause resumes from about where it stopped.

Nothing here moves the robot.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

JOINT_TOL_RAD = 0.0005          # decimation: every recorded sample within 0.03° of the replayed (linear) path
MOVE_TOL_RAD = 0.001            # trim: the arm "moved" once a joint is this far from where it stood
START_TOL_RAD = 0.01            # the arm counts as at the first point (MoveIt's allowed start tolerance)
SPEED_RANGE = (0.05, 2.0)       # step `speed`: 5 .. 200 % of the taught timing (limits still slow it down)
MAX_POINTS = 50000
MIN_DT = 1e-4
RESAMPLE_S = 0.01               # the recording on a 10 ms grid
SMOOTH_S = 0.02                 # Gaussian sigma of the smoothing
LIMIT_MARGIN = 1.01             # a slowed replay ends just under the limits
DEFAULT_JOINT_VELOCITY = math.pi
DEFAULT_JOINT_ACCEL = 5.0


class ReplayError(ValueError):
    pass


# ------------------------------------------------------------------ recording -> step
def trim(t: np.ndarray, Q: np.ndarray, tol: float = MOVE_TOL_RAD) -> Tuple[np.ndarray, np.ndarray]:
    """Cut the standstill before the first motion and after the last one (one still sample stays at each end)."""
    if len(t) < 2:
        raise ReplayError("the recording has fewer than 2 joint samples: record longer")
    d0 = np.max(np.abs(Q - Q[0]), axis=1)
    d1 = np.max(np.abs(Q - Q[-1]), axis=1)
    moved0, moved1 = np.nonzero(d0 > tol)[0], np.nonzero(d1 > tol)[0]
    if not len(moved0):
        raise ReplayError("the arm did not move (joints) while recording")
    a = max(0, int(moved0[0]) - 1)
    b = min(len(t) - 1, int(moved1[-1]) + 1)
    return t[a:b + 1] - t[a], Q[a:b + 1]


def smooth(t: np.ndarray, Q: np.ndarray, dt: float = RESAMPLE_S, sigma: float = SMOOTH_S
           ) -> Tuple[np.ndarray, np.ndarray]:
    """The samples on a regular grid of dt, smoothed with a Gaussian of sigma (ends padded: at rest stays at rest)."""
    tg = np.arange(0.0, float(t[-1] - t[0]) + 1e-9, dt) + t[0]
    if tg[-1] < t[-1] - 1e-9:
        tg = np.r_[tg, t[-1]]
    G = np.column_stack([np.interp(tg, t, Q[:, j]) for j in range(Q.shape[1])])
    r = int(math.ceil(3 * sigma / dt))
    if r < 1 or len(tg) < 3:
        return tg - tg[0], G
    w = np.exp(-0.5 * (np.arange(-r, r + 1) * dt / sigma) ** 2)
    w /= w.sum()
    P = np.vstack([np.repeat(G[:1], r, axis=0), G, np.repeat(G[-1:], r, axis=0)])
    S = np.column_stack([np.convolve(P[:, j], w, mode="valid") for j in range(G.shape[1])])
    return tg - tg[0], S


def decimate(t: np.ndarray, Q: np.ndarray, tol: float = JOINT_TOL_RAD) -> List[int]:
    """Indices to keep: linear interpolation in time between kept samples is within tol of every sample."""
    n = len(t)
    if n <= 2:
        return list(range(n))
    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        span = t[j] - t[i]
        s = (t[i + 1:j] - t[i]) / span if span > 0 else np.linspace(0, 1, j - i + 1)[1:-1]
        lin = Q[i] + s[:, None] * (Q[j] - Q[i])
        err = np.max(np.abs(Q[i + 1:j] - lin), axis=1)
        m = int(np.argmax(err))
        if err[m] > tol:
            k = i + 1 + m
            keep.add(k)
            stack.append((i, k))
            stack.append((k, j))
    return sorted(keep)


def from_recording(samples: Sequence[Tuple[float, Sequence[float]]], tol: float = JOINT_TOL_RAD) -> dict:
    """Joint samples [(t, q)] -> {points (the step's), samples, duration_s, max_dev_rad}."""
    if len(samples) < 2:
        raise ReplayError("the recording has fewer than 2 joint samples: record longer")
    t = np.array([float(s[0]) for s in samples])
    Q = np.array([[float(x) for x in s[1]] for s in samples])
    order = np.r_[True, np.diff(t) > MIN_DT]           # same joint state read twice: one sample
    t, Q = t[order], Q[order]
    if len(t) < 2:
        raise ReplayError("the recording has fewer than 2 joint samples: record longer")
    t, Q = smooth(t, Q)
    t, Q = trim(t, Q)
    keep = decimate(t, Q, tol)
    if len(keep) > MAX_POINTS:
        raise ReplayError(f"the path needs {len(keep)} points (limit {MAX_POINTS}): record a shorter one")
    pts = [[round(float(t[k]), 4)] + [round(float(x), 6) for x in Q[k]] for k in keep]
    return {"points": pts, "samples": len(t), "duration_s": round(float(t[-1]), 2)}


# ---------------------------------------------------------------- step -> numbers
def parse(params: dict, n_joints: int) -> Tuple[np.ndarray, np.ndarray]:
    """(t, Q) of a replay step; ReplayError with a short sentence when malformed."""
    pts = params.get("points")
    if not isinstance(pts, list) or len(pts) < 2:
        raise ReplayError("replay needs points [[t, q1..qn], ...] (at least 2)")
    if len(pts) > MAX_POINTS:
        raise ReplayError(f"replay has {len(pts)} points (limit {MAX_POINTS})")
    try:
        A = np.array([[float(x) for x in p] for p in pts], float)
    except (TypeError, ValueError):
        raise ReplayError("replay points must be lists of numbers") from None
    if A.ndim != 2 or A.shape[1] != n_joints + 1:
        raise ReplayError(f"replay points need t and {n_joints} joint values each")
    if not np.all(np.isfinite(A)):
        raise ReplayError("replay points must be finite numbers")
    t, Q = A[:, 0], A[:, 1:]
    if abs(t[0]) > 1e-9 or np.any(np.diff(t) < MIN_DT):
        raise ReplayError("replay times must start at 0 and increase")
    return t, Q


def speed_of(params: dict) -> float:
    v = params.get("speed", 1.0)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not (SPEED_RANGE[0] <= float(v) <= SPEED_RANGE[1]):
        raise ReplayError(f"replay speed must be {SPEED_RANGE[0] * 100:g}..{SPEED_RANGE[1] * 100:g} %")
    return float(v)


def velocities(t: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """Central differences, zero at both ends (the replay starts and ends at rest)."""
    V = np.zeros_like(Q)
    if len(t) > 2:
        V[1:-1] = (Q[2:] - Q[:-2]) / (t[2:] - t[:-2])[:, None]
    return V


def spline_peaks(t: np.ndarray, Q: np.ndarray, V: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Peak |velocity| and |acceleration| per joint of the cubic Hermite splines the trajectory controller runs
    through (t, Q, V). Acceleration of a cubic is linear in time: its peak is at a segment end."""
    h = np.diff(t)[:, None]
    dq = np.diff(Q, axis=0)
    v0, v1 = V[:-1], V[1:]
    a0 = (6 * dq - h * (4 * v0 + 2 * v1)) / h ** 2
    a1 = (-6 * dq + h * (2 * v0 + 4 * v1)) / h ** 2
    acc = np.max(np.maximum(np.abs(a0), np.abs(a1)), axis=0)
    vel = np.max(np.abs(V), axis=0)
    for s in (0.25, 0.5, 0.75):                       # inside a segment: v(s) of the cubic
        vs = v0 + a0 * (s * h) + (a1 - a0) / (2 * h) * (s * h) ** 2
        vel = np.maximum(vel, np.max(np.abs(vs), axis=0))
    return vel, acc


def fit_limits(t: np.ndarray, Q: np.ndarray, vlim: np.ndarray, alim: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """(t, V, k): the timing slowed uniformly by k >= 1 so the splines stay within the limits."""
    V = velocities(t, Q)
    vel, acc = spline_peaks(t, Q, V)
    rv = float(np.max(vel / vlim)) if len(vel) else 0.0
    ra = float(np.max(acc / alim)) if len(acc) else 0.0
    if rv <= 1.0 and ra <= 1.0:
        return t, V, 1.0
    k = max(rv, math.sqrt(ra)) * LIMIT_MARGIN       # time x k: velocities / k, accelerations / k^2
    return t * k, V / k, k


def joint_limits(model: Optional[dict], profile, names: Sequence[str]) -> Tuple[np.ndarray, np.ndarray]:
    """Velocity (URDF, default pi rad/s) and acceleration (profile moveit.joint_accel_limit_rad_s2) per joint."""
    by = {j["name"]: j for j in (model or {}).get("joints", [])}
    try:
        a = float((getattr(profile, "moveit", None) or {}).get("joint_accel_limit_rad_s2", DEFAULT_JOINT_ACCEL))
    except (TypeError, ValueError):
        a = DEFAULT_JOINT_ACCEL
    if not (a > 0 and math.isfinite(a)):
        a = DEFAULT_JOINT_ACCEL
    v = [float((by.get(n) or {}).get("max_velocity") or DEFAULT_JOINT_VELOCITY) for n in names]
    return np.array(v), np.full(len(names), a)


# ------------------------------------------------------------------ safety checks
def position_problems(model: Optional[dict], names: Sequence[str], Q: np.ndarray) -> List[str]:
    out = []
    by = {j["name"]: j for j in (model or {}).get("joints", [])}
    for i, n in enumerate(names):
        lim = (by.get(n) or {}).get("limit")
        if not lim or (by[n].get("type") == "continuous"):
            continue
        lo, hi = float(np.min(Q[:, i])), float(np.max(Q[:, i]))
        if lo < lim["lower"] - 1e-6 or hi > lim["upper"] + 1e-6:
            out.append(f"joint {n} goes to {math.degrees(lo if lo < lim['lower'] else hi):.1f}°, outside its limits "
                       f"[{math.degrees(lim['lower']):.0f}°, {math.degrees(lim['upper']):.0f}°]")
    return out


def _chain(model: dict, base: str, link: str) -> Optional[list]:
    """The joints from base to link with their fixed origins precomputed: [(name, type, axis, T_origin, mimic)]."""
    from .program_path import _origin
    by_child = {j["child"]: j for j in model.get("joints", [])}
    chain, cur = [], link
    while cur != base:
        j = by_child.get(cur)
        if j is None:
            return None
        chain.append(j)
        cur = j["parent"]
    return [(j["name"], j["type"], np.asarray(j.get("axis", [1, 0, 0]), float), _origin(j), j.get("mimic"))
            for j in reversed(chain)]


def frames_along(chain: list, q: Dict[str, float]) -> List[np.ndarray]:
    """T_base_child of every joint of the chain (the last one: the link)."""
    from .program_path import _rot
    T, out = np.eye(4), []
    for name, typ, axis, O, mimic in chain:
        T = T @ O
        v = q.get(name, 0.0)
        if mimic:
            v = q.get(mimic["joint"], 0.0) * mimic["multiplier"] + mimic["offset"]
        if typ in ("revolute", "continuous"):
            M = np.eye(4)
            M[:3, :3] = _rot(axis, v)
            T = T @ M
        elif typ == "prismatic":
            M = np.eye(4)
            M[:3, 3] = axis * v
            T = T @ M
        out.append(T)
    return out


def keepout_problems(model: Optional[dict], base: str, link: str, T_link_tcp: np.ndarray, names: Sequence[str],
                     Q: np.ndarray, boxes: Sequence[Tuple[str, np.ndarray, Sequence[float]]]) -> List[str]:
    """The keep-out boxes [(name, T_base_box, size_m)] the TCP or a joint origin enters at some point."""
    if not boxes:
        return []
    if not model or not model.get("joints"):
        return ["cannot check the path against the keep-out zones: no robot model (robot_description)"]
    chain = _chain(model, base, link)
    if chain is None:
        return [f"cannot check the path against the keep-out zones: no chain {base} -> {link} in the robot model"]
    from .keepout import point_in_box
    hit: Dict[str, Tuple[int, str]] = {}
    for k, q in enumerate(Q):
        Ts = frames_along(chain, dict(zip(names, (float(x) for x in q))))
        pts = [("TCP", (Ts[-1] @ T_link_tcp)[:3, 3])] + [(chain[i][0], T[:3, 3]) for i, T in enumerate(Ts)]
        for bname, T_box, size in boxes:
            if bname in hit:
                continue
            for what, p in pts:
                if point_in_box(p, T_box, size):
                    hit[bname] = (k, what)
                    break
    return [f"the path enters keep-out zone {b} (the {what} at point {k + 1} of {len(Q)})"
            for b, (k, what) in hit.items()]


def check(model: Optional[dict], profile, names: Sequence[str], Q: np.ndarray, link: str, T_link_tcp: np.ndarray,
          boxes) -> Tuple[List[str], np.ndarray, np.ndarray]:
    """(problems, velocity limits, acceleration limits) of a replay from where the robot model / zones are now."""
    problems = position_problems(model, names, Q)
    problems += keepout_problems(model, profile.base_link, link, T_link_tcp, names, Q, boxes)
    vlim, alim = joint_limits(model, profile, names)
    return problems, vlim, alim


def resume_index(t_run: np.ndarray, Q: np.ndarray, k0: int, elapsed: float, q_now: Sequence[float]) -> int:
    """After a pause: the point to go on from. t_run: the (scaled) times the interrupted run used, from k0;
    elapsed: how long it ran. The nearest point (joints) to where the arm is, within 1 s of the time estimate."""
    rel = t_run - t_run[k0]
    k_est = int(np.searchsorted(rel, elapsed, side="right")) - 1
    k_est = min(max(k0, k_est), len(Q) - 1)
    near = np.nonzero(np.abs(rel - rel[k_est]) <= 1.0)[0]
    near = near[near >= k0]
    if not len(near):
        return k_est
    d = np.max(np.abs(Q[near] - np.asarray(q_now, float)), axis=1)
    return int(near[int(np.argmin(d))])


def summary(params: dict) -> str:
    pts = params.get("points") or []
    try:
        d = float(pts[-1][0])
    except (IndexError, TypeError, ValueError):
        d = 0.0
    sp = params.get("speed", 1.0)
    try:
        spt = f" · {float(sp) * 100:g} %" if float(sp) != 1.0 else ""
    except (TypeError, ValueError):
        spt = ""
    name = f"{params['name']} · " if params.get("name") else ""
    return f"{name}{len(pts)} points, {d:.1f} s as taught{spt}"
