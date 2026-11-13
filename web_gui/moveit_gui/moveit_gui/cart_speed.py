"""Cartesian speed (mm/s) of linear moves (MoveL): OPERATE > Pose and the program's linear motion steps.

Jazzy's GetCartesianPath.srv and MotionPlanRequest carry `max_cartesian_speed` / `cartesian_speed_limited_link`,
but move_group does not use them for Cartesian paths (MoveIt 2.12 in the image and 2.13 main: the
compute_cartesian_path capability only runs TOTG at the velocity / acceleration scaling; its own "\\todo" says so).
Setting them would change nothing. They also only accept a URDF link, not a virtual TCP.

So the GUI times the returned path itself (retime_linear), for the TCP POINT (a virtual TCP too) by forward
kinematics from the URDF (program_path.fk):

1. constant_speed: a CONSTANT TCP speed, as scraping wants. The path's points are densified (densify), the TCP's
   arc length s_i is computed at each, and s(t) is a trapezoid: ramp up at the Cartesian acceleration, cruise at
   the speed, ramp down. Times come from s_i; joint velocities are dq/ds * ds/dt; accelerations are left out, so
   the JointTrajectoryController uses cubic splines through positions + velocities. Joint velocity limits (URDF x
   Speed %) and acceleration limits (profile x Accel %) are checked on those splines (spline_extrema: between the
   points too); if one would be exceeded, the whole move is slowed (speed and acceleration together) and the log
   says so.
2. If that is not possible (e.g. the TCP point does not move: a pure rotation), or as a safety check after it:
   limit_speed slows the planned trajectory down uniformly until its peak is at or under the speed. That only ever
   makes a trajectory slower, so it never runs faster than asked.
"""
from __future__ import annotations

import copy
import math
from typing import Callable, Optional, Sequence

import numpy as np

DEFAULT_MAX_MM_S = 250.0
DEFAULT_CART_ACCEL_MM_S2 = 500.0       # moveit.cartesian_accel_mm_s2: ramp of a constant-speed move at Accel 100 %
DEFAULT_JOINT_ACCEL = 5.0              # rad/s^2, moveit.joint_accel_limit_rad_s2 (MoveIt's joint_limits.yaml)
DEFAULT_JOINT_VELOCITY = math.pi       # rad/s, for a joint whose URDF has no velocity limit
DENSIFY_STEP_M = 0.001                 # densify: at most 1 mm of TCP travel between points
DENSIFY_TOL_M = 1e-4                   # ... where joint interpolation stays within 0.1 mm of the straight line
CRUISE_TOL = 0.02                      # the result's peak may be 2 % over the speed (finite differences)
# steps whose linear moves take speed_mm_s (move_pose only when cartesian, move_relative unless cartesian: false)
SPEED_STEPS = ("move_pose", "move_relative", "move_until_contact")


def max_mm_s(profile) -> float:
    """The profile's upper limit (moveit.max_cartesian_speed_mm_s), default 250 mm/s."""
    try:
        v = float((getattr(profile, "moveit", None) or {}).get("max_cartesian_speed_mm_s", DEFAULT_MAX_MM_S))
    except (TypeError, ValueError):
        return DEFAULT_MAX_MM_S
    return v if v > 0 and math.isfinite(v) else DEFAULT_MAX_MM_S


def check_mm_s(v, limit: float) -> float:
    """A speed_mm_s value: a number in (0, limit]. Raises ValueError with a short sentence."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"speed must be a number of mm/s, got {v!r}")
    v = float(v)
    if not (math.isfinite(v) and 0.0 < v <= limit):
        raise ValueError(f"speed must be in (0, {limit:g}] mm/s")
    return v


def optional_mm_s(v, limit: float) -> float:
    """What the OPERATE page sends: empty / None / 0 = no limit (0.0), else check_mm_s."""
    if v is None or v == "":
        return 0.0
    if isinstance(v, str):
        try:
            v = float(v)
        except ValueError:
            raise ValueError(f"speed must be a number of mm/s, got {v!r}") from None
    if not isinstance(v, bool) and isinstance(v, (int, float)) and float(v) == 0.0:
        return 0.0
    return check_mm_s(v, limit)


def _secs(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def _set_secs(d, t: float):
    ns = int(round(t * 1e9))
    d.sec, d.nanosec = divmod(ns, 1_000_000_000)


PointAt = Callable[[dict], np.ndarray]      # {joint: value} -> TCP point position in the base frame (m)
SPLINE_SAMPLES = 8                          # samples per segment where the controller's cubic is checked


def spline_extrema(t, Q, Qd):
    """Peak |velocity| and |acceleration| per joint of the cubic Hermite splines through (t, Q, Qd), the
    JointTrajectoryController's interpolation for positions + velocities: between the points too, not only at them."""
    t, Q, Qd = np.asarray(t, float), np.asarray(Q, float), np.asarray(Qd, float)
    h = np.diff(t)[:, None]
    ok = h[:, 0] > 1e-9
    if not np.any(ok):
        return np.zeros(Q.shape[1]), np.zeros(Q.shape[1])
    h, q0, q1, v0, v1 = h[ok], Q[:-1][ok], Q[1:][ok], Qd[:-1][ok], Qd[1:][ok]
    # q(u) = a0 + a1 u + a2 u^2 + a3 u^3 on u in [0, h]
    a2 = (3 * (q1 - q0) / h - 2 * v0 - v1) / h
    a3 = (2 * (q0 - q1) / h + v0 + v1) / (h * h)
    vmax = np.maximum(np.abs(v0), np.abs(v1)).max(axis=0)
    u_star = np.where(np.abs(a3) > 1e-12, -a2 / (3 * np.where(np.abs(a3) > 1e-12, a3, 1.0)), -1.0)
    inside = (u_star > 0) & (u_star < h)
    v_star = v0 + 2 * a2 * u_star + 3 * a3 * u_star ** 2          # the velocity's extremum inside a segment
    vmax = np.maximum(vmax, np.where(inside, np.abs(v_star), 0.0).max(axis=0))
    amax = np.maximum(np.abs(2 * a2), np.abs(2 * a2 + 6 * a3 * h)).max(axis=0)   # linear: largest at an end
    return vmax, amax


def _spline_positions(t, Q, Qd, n: int = SPLINE_SAMPLES):
    """Positions on the cubic splines at n - 1 times inside each segment (the points themselves excluded)."""
    t, Q, Qd = np.asarray(t, float), np.asarray(Q, float), np.asarray(Qd, float)
    out = []
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        if h <= 1e-9:
            continue
        for k in range(1, n):
            x = k / n
            h00, h10, h01, h11 = 2 * x**3 - 3 * x**2 + 1, x**3 - 2 * x**2 + x, -2 * x**3 + 3 * x**2, x**3 - x**2
            out.append((t[i] + x * h, h00 * Q[i] + h10 * h * Qd[i] + h01 * Q[i + 1] + h11 * h * Qd[i + 1]))
    return out


def peak_speed(traj, point_at: PointAt) -> float:
    """Highest speed (m/s) of the TCP point along a JointTrajectory: per segment (distance / time) and at each
    point from its joint velocities (a small step along qd), so a long segment does not hide a peak."""
    names = list(traj.joint_names)
    pts = list(traj.points)
    P = [np.asarray(point_at(dict(zip(names, p.positions))), float) for p in pts]
    peak = 0.0
    for i in range(1, len(pts)):
        dt = _secs(pts[i].time_from_start) - _secs(pts[i - 1].time_from_start)
        if dt > 1e-9:
            peak = max(peak, float(np.linalg.norm(P[i] - P[i - 1])) / dt)
    h = 1e-3
    for p, Pi in zip(pts, P):
        qd = list(getattr(p, "velocities", None) or [])
        if len(qd) == len(names):
            q2 = {n: float(q) + float(v) * h for n, q, v in zip(names, p.positions, qd)}
            peak = max(peak, float(np.linalg.norm(np.asarray(point_at(q2), float) - Pi)) / h)
    if len(pts) >= 2 and all(len(getattr(p, "velocities", None) or []) == len(names) for p in pts):
        # between the points, on the controller's cubic splines: a peak there is hidden from both checks above
        t = [_secs(p.time_from_start) for p in pts]
        Q = [list(p.positions) for p in pts]
        Qd = [list(p.velocities) for p in pts]
        samples = [(t[0], np.asarray(P[0]))]
        for ts, q in _spline_positions(t, Q, Qd):
            samples.append((ts, np.asarray(point_at(dict(zip(names, q))), float)))
        samples.append((t[-1], np.asarray(P[-1])))
        samples.sort(key=lambda x: x[0])
        for (ta, pa), (tb, pb) in zip(samples, samples[1:]):
            if tb - ta > 1e-9:
                peak = max(peak, float(np.linalg.norm(pb - pa)) / (tb - ta))
    return peak


def limit_speed(traj, point_at: PointAt, v_max: float) -> float:
    """Slow `traj` down in place (uniform time scaling) so the TCP point stays at or below v_max (m/s).
    Returns the factor k >= 1 the duration grew by (1.0 = it was slow enough)."""
    if v_max <= 0 or len(traj.points) < 2:
        return 1.0
    peak = peak_speed(traj, point_at)
    if peak <= v_max:
        return 1.0
    k = peak / v_max
    for p in traj.points:
        _set_secs(p.time_from_start, _secs(p.time_from_start) * k)
        if getattr(p, "velocities", None):
            p.velocities = [float(v) / k for v in p.velocities]
        if getattr(p, "accelerations", None):
            p.accelerations = [float(a) / (k * k) for a in p.accelerations]
    return k


def point_fk(model: dict, base: str, link: str, point: Optional[Sequence[float]] = None) -> Optional[PointAt]:
    """point_at for a point fixed in `link` (xyz in m, None = the link origin), from a parsed URDF; None when the
    link is not below `base` in the model."""
    from .program_path import fk
    if fk(model, {}, base, link) is None:
        return None
    p = np.r_[np.asarray(point if point is not None else [0.0, 0.0, 0.0], float), 1.0]
    return lambda q: (fk(model, q, base, link) @ p)[:3]


# ------------------------------------------------------------- constant speed
class RetimeError(ValueError):
    pass


def _profile_float(profile, key: str, default: float) -> float:
    try:
        v = float((getattr(profile, "moveit", None) or {}).get(key, default))
    except (TypeError, ValueError):
        return default
    return v if v > 0 and math.isfinite(v) else default


def cart_accel_m_s2(profile, acc_scaling: float) -> float:
    """The ramp of a constant-speed move: moveit.cartesian_accel_mm_s2 (default 500) x the acceleration scaling
    (Accel % and GUI SPEED, as MoveIt scales the joint accelerations), in m/s^2."""
    return _profile_float(profile, "cartesian_accel_mm_s2", DEFAULT_CART_ACCEL_MM_S2) / 1000.0 * max(0.01, acc_scaling)


def joint_limits(model: dict, profile, names, vel_scaling: float, acc_scaling: float):
    """(velocity, acceleration) limits per joint: URDF velocity x Speed %, profile joint accel x Accel %."""
    by_name = {j["name"]: j for j in model.get("joints", [])}
    a = _profile_float(profile, "joint_accel_limit_rad_s2", DEFAULT_JOINT_ACCEL) * max(0.01, acc_scaling)
    vel = {}
    for n in names:
        v = (by_name.get(n) or {}).get("max_velocity")
        vel[n] = (float(v) if v else DEFAULT_JOINT_VELOCITY) * max(0.01, vel_scaling)
    return vel, {n: a for n in names}


def _seg_dist(p, a, b) -> float:
    ab = b - a
    L2 = float(ab @ ab)
    t = 0.0 if L2 < 1e-18 else min(1.0, max(0.0, float((p - a) @ ab) / L2))
    return float(np.linalg.norm(p - (a + t * ab)))


def densify(Q: np.ndarray, point_at_q, step: float = DENSIFY_STEP_M, tol: float = DENSIFY_TOL_M):
    """Points every <= `step` of TCP travel, by joint interpolation between the planned points, kept only where FK
    shows the TCP within `tol` of the straight line between them. There is no IK here: where joint interpolation
    bends more, MoveIt's own points stay as they are (the controller's spline goes between them as it did before).
    Returns (joint points, worst deviation of a segment left as it was, m)."""
    out, worst = [Q[0]], 0.0
    P = [point_at_q(q) for q in Q]
    for i in range(1, len(Q)):
        q0, q1, p0, p1 = Q[i - 1], Q[i], P[i - 1], P[i]
        m = int(math.ceil(float(np.linalg.norm(p1 - p0)) / step))
        if m > 1:
            cand = [q0 + (q1 - q0) * (k / m) for k in range(1, m)]
            dev = max(_seg_dist(point_at_q(c), p0, p1) for c in cand)
            if dev < tol:
                out.extend(cand)
            else:
                worst = max(worst, dev)
        out.append(q1)
    return np.array(out), worst


def trapezoid(s: np.ndarray, L: float, v: float, a: float):
    """Times t_i and speeds ds/dt at arc lengths s_i of a ramp (a) - cruise (v) - ramp profile over L.
    A path too short to reach v ramps up and down (triangle). Returns (t, sdot, duration)."""
    vp = min(v, math.sqrt(a * L))
    ta, sa = vp / a, vp * vp / (2 * a)
    T = 2 * ta + (L - 2 * sa) / vp
    t = np.empty(len(s))
    for i, si in enumerate(s):
        si = min(max(float(si), 0.0), L)
        if si <= sa:
            t[i] = math.sqrt(2 * si / a)
        elif si <= L - sa:
            t[i] = ta + (si - sa) / vp
        else:
            t[i] = T - math.sqrt(max(0.0, 2 * (L - si) / a))
    sdot = np.minimum(np.minimum(a * t, vp), a * np.maximum(T - t, 0.0))
    return t, sdot, T


def trapezoid_duration(L: float, v: float, a: float) -> float:
    """Duration of trapezoid(): L / v + v / a when the cruise is reached, else 2 sqrt(L / a)."""
    if L <= 0:
        return 0.0
    return L / v + v / a if L >= v * v / a else 2 * math.sqrt(L / a)


def constant_speed(traj, point_at: PointAt, v: float, a: float, vel_limits: dict, acc_limits: dict) -> dict:
    """Retime `traj` (in place, all or nothing) so the TCP point goes at a constant v (m/s) with ramps at a
    (m/s^2), within the joint limits ({joint: rad/s}, {joint: rad/s^2}); slows the whole move if they need it.
    Raises RetimeError when it cannot. Returns {v, a, duration, points, slowed, sparse_dev}."""
    names = list(traj.joint_names)
    if len(traj.points) < 2 or v <= 0 or a <= 0:
        raise RetimeError("nothing to retime")
    Q = np.array([[float(x) for x in p.positions] for p in traj.points])
    if Q.shape[1] != len(names):
        raise RetimeError("positions do not match the joint names")
    at_q = lambda q: np.asarray(point_at(dict(zip(names, q))), float)   # noqa: E731
    Q, sparse_dev = densify(Q, at_q)
    P = np.array([at_q(q) for q in Q])
    ds = np.linalg.norm(np.diff(P, axis=0), axis=1)
    keep = np.r_[True, ds > 1e-9]               # no zero-length steps (s must increase)
    keep[-1] = True
    Q, P = Q[keep], P[keep]
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    L = float(s[-1])
    if L < 1e-5 or len(s) < 2:
        raise RetimeError("the TCP point hardly moves (a rotation in place)")
    dqds = np.gradient(Q, s, axis=0) if len(s) > 2 else np.vstack([(Q[1] - Q[0]) / L] * 2)
    vlim = np.array([vel_limits[n] for n in names])
    alim = np.array([acc_limits[n] for n in names])
    v_eff, a_eff, slowed = float(v), float(a), 1.0
    for _ in range(10):
        t, sdot, T = trapezoid(s, L, v_eff, a_eff)
        Qd = dqds * sdot[:, None]
        Qd[0] = Qd[-1] = 0.0
        vpk, apk = spline_extrema(t, Q, Qd)          # what the controller's splines do, between the points too
        rv = float(np.max(vpk / vlim))
        ra = float(np.max(apk / alim))
        if rv <= 1.0 and ra <= 1.0:
            break
        k = max(rv, math.sqrt(ra), 1.0) * 1.02       # time x k: velocities / k, accelerations / k^2
        v_eff, a_eff, slowed = v_eff / k, a_eff / (k * k), slowed * k
    else:
        raise RetimeError("the joint limits cannot be kept at any speed")
    Qd[0] = Qd[-1] = 0.0
    tmpl = traj.points[0]
    pts = []
    for q, qd, ti in zip(Q, Qd, t):
        p = copy.deepcopy(tmpl)
        p.positions = [float(x) for x in q]
        p.velocities = [float(x) for x in qd]
        p.accelerations = []                     # cubic splines through positions + velocities
        if hasattr(p, "effort"):
            p.effort = []
        _set_secs(p.time_from_start, float(ti))
        pts.append(p)
    traj.points = pts
    return {"v": v_eff, "a": a_eff, "duration": T, "points": len(pts), "slowed": slowed, "sparse_dev": sparse_dev}


def retime_linear(traj, point_at: PointAt, v: float, a: float, vel_limits: dict, acc_limits: dict,
                  log: Callable[[str], None] = lambda _m: None) -> None:
    """What a linear move with a mm/s gets: constant speed when possible, then the safety check (and the uniform
    slow-down as the fallback) so the TCP point's peak is never over v (+2 % for finite differences)."""
    try:
        info = constant_speed(traj, point_at, v, a, vel_limits, acc_limits)
        log(f"linear move at a constant {info['v'] * 1000:.1f} mm/s (ramps {info['a'] * 1000:.0f} mm/s²), "
            f"{info['duration']:.2f} s, {info['points']} points")
        if info["slowed"] > 1.0:
            log(f"linear move slowed x{info['slowed']:.2f} from {v * 1000:.1f} mm/s: the joint limits need it")
        if info["sparse_dev"] > 0:
            log(f"linear move: some planned points left sparse (joint interpolation off the line by up to "
                f"{info['sparse_dev'] * 1000:.2f} mm)")
    except RetimeError as e:
        log(f"constant speed not possible ({e}): the planned move is only slowed to its peak")
    if peak_speed(traj, point_at) > v * (1.0 + CRUISE_TOL):
        k = limit_speed(traj, point_at, v)
        log(f"linear move slowed x{k:.2f} to keep the TCP at {v * 1000:.0f} mm/s")
