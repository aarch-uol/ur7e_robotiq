"""Teach by hand: the operator moves the arm (freedrive, or jogging) and the GUI writes the program.

Two ways (PROGRAMS > Teach & more > Teach by hand; the web side is web/teaching.py):

* Waypoints: every Mark (or, optionally, every time the arm stood still for about a second after it moved) adds a
  waypoint and a move step.
* Path: the joint states are sampled at JOINT_HZ (100 Hz, with their own time stamps) and the TCP pose at SAMPLE_HZ
  (20 Hz) while recording. Inserted as a Replay (default) the joint samples become one `replay` step that runs the
  motion back as it was taught (replay.py). Inserted as MoveL points (and for a pressed path) the TCP path is
  simplified into a few MoveL waypoints with Ramer-Douglas-Peucker (RDP) on the position, with an orientation
  tolerance as well.

Nothing here moves the robot. Replaying is an ordinary program run.

Simplification and its guarantee
--------------------------------
A MoveL from A to B goes along the straight segment AB, and MoveIt interpolates the orientation with the same
fraction s (slerp(R_A, R_B, s) at A + s (B - A)). For a sample k between two kept samples i < k < j:

    s_k      = projection of p_k onto the segment, clamped to [0, 1] (time fraction if the segment is < 0.5 mm:
               a turn in place)
    pos dev  = | p_k - (p_i + s_k (p_j - p_i)) |                  (the distance to the segment)
    rot dev  = angle between R_k and slerp(R_i, R_j, s_k)

RDP keeps splitting a segment at its worst sample (the largest of pos dev / pos tol and rot dev / rot tol) until
every sample is within both tolerances of the segment between the two kept samples around it. That is the
guarantee: `deviation()` measures exactly this, and its maximum is what the page shows.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from . import transforms as tf

SAMPLE_HZ = 20.0                            # TCP poses (review, MoveL points)
JOINT_HZ = 100.0                            # joint states (replay); the TCP every JOINT_HZ / SAMPLE_HZ samples
MAX_SAMPLES = int(SAMPLE_HZ * 600)          # 10 minutes of path
MAX_JOINT_SAMPLES = int(JOINT_HZ * 600)
POS_TOL_MM = (0.1, 50.0)                    # allowed tolerance ranges
ROT_TOL_DEG = (0.1, 45.0)
TURN_IN_PLACE_M = 0.0005                    # a segment shorter than this is a turn in place: s by time
MOVING_MM_S = 2.0                           # slower than this counts as standing (for the average speed)


class TeachError(ValueError):
    pass


def _quats(Ts: Sequence[np.ndarray]) -> np.ndarray:
    """Unit quaternions (xyzw) of the rotations, signs made continuous (q and -q are the same rotation)."""
    Q = np.array([tf.matrix_to_quat(np.asarray(T)[:3, :3]) for T in Ts], float).reshape(-1, 4)
    for k in range(1, len(Q)):
        if float(np.dot(Q[k], Q[k - 1])) < 0:
            Q[k] = -Q[k]
    return Q


def _slerp(qa: np.ndarray, qb: np.ndarray, s: np.ndarray) -> np.ndarray:
    """slerp from qa to qb (one pair) at the fractions s (n,): (n, 4)."""
    d = float(np.dot(qa, qb))
    if d < 0:
        qb, d = -qb, -d
    s = np.asarray(s, float)[:, None]
    if d > 0.9995:                                     # almost the same: normalised lerp is exact enough
        q = (1 - s) * qa + s * qb
    else:
        th = math.acos(min(1.0, d))
        q = (np.sin((1 - s) * th) * qa + np.sin(s * th) * qb) / math.sin(th)
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def _angle(qa: np.ndarray, qb: np.ndarray) -> np.ndarray:
    """Rotation angle (rad) between rows of qa and qb."""
    d = np.clip(np.abs(np.sum(qa * qb, axis=1)), 0.0, 1.0)
    return 2.0 * np.arccos(d)


def segment_deviation(t: np.ndarray, P: np.ndarray, Q: np.ndarray, i: int, j: int) -> Tuple[np.ndarray, np.ndarray]:
    """(pos dev m, rot dev rad) of the samples strictly between i and j from the MoveL i -> j (module docstring)."""
    if j - i < 2:
        return np.zeros(0), np.zeros(0)
    seg = P[j] - P[i]
    L2 = float(seg @ seg)
    inner = P[i + 1:j]
    if L2 > TURN_IN_PLACE_M ** 2:
        s = np.clip((inner - P[i]) @ seg / L2, 0.0, 1.0)
    else:
        span = float(t[j] - t[i])
        s = (t[i + 1:j] - t[i]) / span if span > 0 else np.arange(1, j - i) / float(j - i)
        s = np.clip(s, 0.0, 1.0)
    dp = np.linalg.norm(inner - (P[i] + s[:, None] * seg), axis=1)
    dr = _angle(Q[i + 1:j], _slerp(Q[i], Q[j], s))
    return dp, dr


def simplify_indices(t, P, Q, pos_tol_m: float, rot_tol_rad: float) -> List[int]:
    """RDP on position + orientation: the indices of the samples to keep (first and last always)."""
    n = len(P)
    if n <= 2:
        return list(range(n))
    if not (pos_tol_m > 0 and rot_tol_rad > 0):
        raise TeachError("the tolerances must be > 0")
    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:                                      # explicit stack: a long path must not hit the recursion limit
        i, j = stack.pop()
        if j - i < 2:
            continue
        dp, dr = segment_deviation(t, P, Q, i, j)
        score = np.maximum(dp / pos_tol_m, dr / rot_tol_rad)
        m = int(np.argmax(score))
        if score[m] > 1.0:
            k = i + 1 + m
            keep.add(k)
            stack.append((i, k))
            stack.append((k, j))
    return sorted(keep)


def deviation(t, P, Q, keep: Sequence[int]) -> Tuple[float, float]:
    """The largest (pos dev m, rot dev rad) of every sample from the MoveL segment it falls in."""
    mp = mr = 0.0
    for a, b in zip(keep[:-1], keep[1:]):
        dp, dr = segment_deviation(t, P, Q, a, b)
        if len(dp):
            mp, mr = max(mp, float(dp.max())), max(mr, float(dr.max()))
    return mp, mr


def check_tolerances(pos_tol_mm, rot_tol_deg) -> Tuple[float, float]:
    try:
        p, r = float(pos_tol_mm), float(rot_tol_deg)
    except (TypeError, ValueError):
        raise TeachError("the tolerances are numbers (mm and °)") from None
    if not (math.isfinite(p) and POS_TOL_MM[0] <= p <= POS_TOL_MM[1]):
        raise TeachError(f"position tolerance must be {POS_TOL_MM[0]:g}..{POS_TOL_MM[1]:g} mm")
    if not (math.isfinite(r) and ROT_TOL_DEG[0] <= r <= ROT_TOL_DEG[1]):
        raise TeachError(f"orientation tolerance must be {ROT_TOL_DEG[0]:g}..{ROT_TOL_DEG[1]:g}°")
    return p, r


@dataclass
class Simplified:
    keep: List[int]
    max_mm: float
    max_deg: float


class TaughtPath:
    """Samples (time s, T_base_tcp) of one path recording."""

    def __init__(self, samples: Sequence[Tuple[float, np.ndarray]]):
        if len(samples) < 2:
            raise TeachError("the path has fewer than 2 samples: record longer")
        self.t = np.array([float(s[0]) for s in samples])
        self.T = [np.asarray(s[1], float) for s in samples]
        self.P = np.array([T[:3, 3] for T in self.T])
        self.Q = _quats(self.T)

    def __len__(self):
        return len(self.T)

    def simplify(self, pos_tol_mm: float, rot_tol_deg: float) -> Simplified:
        p, r = check_tolerances(pos_tol_mm, rot_tol_deg)
        keep = simplify_indices(self.t, self.P, self.Q, p / 1000.0, math.radians(r))
        mp, mr = deviation(self.t, self.P, self.Q, keep)
        return Simplified(keep, mp * 1000.0, math.degrees(mr))

    def stats(self) -> dict:
        """Length (mm), duration (s), time spent moving (s) and the average speed while moving (mm/s)."""
        d = np.linalg.norm(np.diff(self.P, axis=0), axis=1) * 1000.0          # mm per sample step
        dt = np.diff(self.t)
        moving = (dt > 0) & (d > MOVING_MM_S * np.where(dt > 0, dt, 1.0))
        length = float(d.sum())
        t_move = float(dt[moving].sum())
        avg = float(d[moving].sum()) / t_move if t_move > 0 else 0.0
        return {"length_mm": round(length, 1), "duration_s": round(float(self.t[-1] - self.t[0]), 2),
                "moving_s": round(t_move, 2), "avg_mm_s": round(avg, 1)}


def suggested_speed(avg_mm_s: float, limit_mm_s: float) -> Optional[float]:
    """The taught average speed as a Max mm/s suggestion: a whole number in [1, limit], None without motion."""
    if not avg_mm_s or avg_mm_s <= 0:
        return None
    return float(max(1, min(int(round(avg_mm_s)), int(limit_mm_s))))


class StillDetector:
    """Auto-mark: True once the TCP stood still for `still_s` (within still_mm / still_deg) after it moved at least
    move_mm / move_deg away from the last mark (or the start), so standing after a mark does not mark again."""

    def __init__(self, still_s: float = 1.0, still_mm: float = 1.0, still_deg: float = 1.0,
                 move_mm: float = 10.0, move_deg: float = 5.0):
        self.still_s, self.still_m, self.still_rad = still_s, still_mm / 1000.0, math.radians(still_deg)
        self.move_m, self.move_rad = move_mm / 1000.0, math.radians(move_deg)
        self.ref: Optional[np.ndarray] = None
        self.win: deque = deque()
        self._lock = threading.RLock()              # a manual Mark resets it from another thread

    def reset(self, T: np.ndarray):
        """The last mark (or the start) is here."""
        with self._lock:
            self.ref = np.asarray(T, float).copy()
            self.win.clear()

    def feed(self, t: float, T: np.ndarray) -> bool:
        with self._lock:
            return self._feed(t, np.asarray(T, float))

    def _feed(self, t: float, T: np.ndarray) -> bool:
        if self.ref is None:
            self.reset(T)
            return False
        self.win.append((t, T))
        while len(self.win) > 1 and t - self.win[1][0] >= self.still_s:
            self.win.popleft()                         # keep just over still_s of samples
        if t - self.win[0][0] < self.still_s:
            return False
        for _, Tk in self.win:
            if np.linalg.norm(Tk[:3, 3] - T[:3, 3]) > self.still_m or tf.angle_between(Tk, T) > self.still_rad:
                return False
        if np.linalg.norm(T[:3, 3] - self.ref[:3, 3]) < self.move_m and tf.angle_between(T, self.ref) < self.move_rad:
            return False                               # still where the last mark was
        self.reset(T)
        return True


class Sampler:
    """Calls get_T() at `hz` in its own thread and passes (t, its result) to on_sample. A failing get_T (no TF)
    skips that sample. Stops by itself after max_samples (on_full is called once)."""

    def __init__(self, get_T: Callable[[], np.ndarray], on_sample: Callable[[float, np.ndarray], None],
                 hz: float = SAMPLE_HZ, max_samples: int = MAX_SAMPLES, on_full: Callable[[], None] = lambda: None,
                 clock: Callable[[], float] = time.monotonic):
        self.get_T, self.on_sample, self.on_full = get_T, on_sample, on_full
        self.period, self.max_samples, self.clock = 1.0 / hz, max_samples, clock
        self.count = self.missed = 0
        self.last_error = ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="teach-sampler")

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(1.0)

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def _run(self):
        nxt = self.clock()
        while not self._stop.is_set():
            try:
                T = self.get_T()
            except Exception as e:                     # noqa: BLE001 - TF gone for a moment: skip the sample
                self.missed += 1
                self.last_error = str(e)
            else:
                self.count += 1
                try:
                    self.on_sample(self.clock(), T)
                except Exception as e:                 # noqa: BLE001 - never let the sampler die silently
                    self.last_error = str(e)
                if self.count >= self.max_samples:
                    self._stop.set()
                    self.on_full()
                    return
            nxt += self.period
            delay = nxt - self.clock()
            if delay < 0:
                nxt, delay = self.clock(), 0.0
            self._stop.wait(delay)
