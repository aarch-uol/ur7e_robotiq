"""Applying a Cartesian force with the admittance or the impedance controller, without Qt.

Both controllers take a geometry_msgs/WrenchStamped target and keep the LAST one they received for as long as
they run: neither has a timeout. So the robot PC streams the target while the operator applies it (the front end
renews every < HOLD_TIMEOUT_S), ramps it up and down (the impedance controller's wrench filter is 1.0: a new
target is a step), and on release, STOP, a lost renewal or an error sends zero.

Meaning, the same for both: the wrench the TOOL EXERTS on the environment, at the active TCP. In free air the
arm therefore moves along the force until the controller's spring balances it (about F / stiffness); against a
surface it presses with F.

* Admittance (ros2_controllers admittance_controller, ~/wrench_reference): the reference is ADDED to the measured
  F/T reading, so the controller behaves as if pushed by it. It must be in the F/T sensor frame (other frames
  are dropped), about the sensor origin. Only the controller's selected_axes respond.
* Impedance (cartesian_impedance_controller, set_cartesian_wrench): tau = J^T F at its end effector, F in its
  root frame. Sent in the root frame: for any other frame the controller's transform adds a translation to the
  force vector.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

import numpy as np

HOLD_TIMEOUT_S = 0.5
STREAM_HZ = 50.0
FRAMES = ("tool", "base")

DEFAULTS = {
    "max_force_n": 20.0,
    "max_torque_nm": 2.0,
    "ramp_force_nps": 10.0,         # ramp up/down on Apply and Release
    "ramp_torque_nmps": 1.0,
    "max_duration_s": 120.0,        # an application ends on its own after this
}


def force_config(profile) -> dict:
    cfg = dict(DEFAULTS)
    cfg.update(getattr(profile, "force", None) or {})
    return cfg


class ForceError(ValueError):
    pass


@dataclass
class WrenchTarget:
    wrench: List[float]             # Fx Fy Fz [N], Tx Ty Tz [Nm], exerted by the tool, about the active TCP
    frame: str = "tool"             # axes: "tool" (active TCP) or "base"

    @classmethod
    def parse(cls, wrench: Sequence, frame: str, cfg: dict) -> "WrenchTarget":
        try:
            w = [float(v) for v in wrench]
        except (TypeError, ValueError):
            raise ForceError("the wrench must be six numbers") from None
        if len(w) != 6 or not all(np.isfinite(w)):
            raise ForceError("the wrench must be six finite numbers: Fx Fy Fz Tx Ty Tz")
        if frame not in FRAMES:
            raise ForceError(f"frame must be one of {', '.join(FRAMES)}")
        f, t = np.linalg.norm(w[:3]), np.linalg.norm(w[3:])
        if f > cfg["max_force_n"] + 1e-9:
            raise ForceError(f"force {f:.1f} N is over the limit of {cfg['max_force_n']:g} N (profile force)")
        if t > cfg["max_torque_nm"] + 1e-9:
            raise ForceError(f"torque {t:.2f} Nm is over the limit of {cfg['max_torque_nm']:g} Nm (profile force)")
        return cls(w, frame)


def wrench_for_controller(target: WrenchTarget, T_base_tcp: np.ndarray, T_base_out: np.ndarray,
                          T_base_point: np.ndarray) -> List[float]:
    """The target, moved from the TCP to the controller's point and expressed in the controller's frame.

    T_base_out: the frame the controller wants the vector in. T_base_point: the point it applies the wrench at.
    Moving a wrench from p_tcp to p: tau_p = tau_tcp + (p_tcp - p) x F."""
    w = np.asarray(target.wrench, float)
    R = T_base_tcp[:3, :3] if target.frame == "tool" else np.eye(3)
    F, tau = R @ w[:3], R @ w[3:]
    tau = tau + np.cross(T_base_tcp[:3, 3] - T_base_point[:3, 3], F)
    R_out = T_base_out[:3, :3].T
    return (R_out @ F).tolist() + (R_out @ tau).tolist()


def ramp_toward(current: np.ndarray, goal: np.ndarray, dt: float, rate_f: float, rate_t: float) -> np.ndarray:
    """Move each half (force, torque) toward its goal by at most rate * dt, along the straight line."""
    out = current.copy()
    for sl, rate in ((slice(0, 3), rate_f), (slice(3, 6), rate_t)):
        d = goal[sl] - current[sl]
        n = float(np.linalg.norm(d))
        step = rate * dt
        out[sl] = goal[sl] if n <= step else current[sl] + d * (step / n)
    return out


class ForceStream:
    """One force application. send(w6) at STREAM_HZ with the ramped wrench (in the target's terms); the
    caller converts it for the controller. Ends on stop(), a missed renewal, max duration or an exception:
    released -> ramps down to zero first; everything else -> zero at once. Then on_end(reason)."""

    def __init__(self, target: WrenchTarget, send: Callable[[List[float]], None], on_end: Callable[[str], None],
                 cfg: dict, hold_timeout_s: float = HOLD_TIMEOUT_S, rate_hz: float = STREAM_HZ,
                 clock: Callable[[], float] = time.monotonic):
        self._send, self._on_end, self.cfg = send, on_end, cfg
        self.hold_timeout_s, self.period = hold_timeout_s, 1.0 / rate_hz
        self._clock = clock
        self._lock = threading.Lock()
        self._goal = np.asarray(target.wrench, float)
        self.target = target
        self.current = np.zeros(6)
        self._t0 = self._last_hold = clock()
        self._stop = threading.Event()
        self._release = threading.Event()
        self._reason = "released"
        self._thread = threading.Thread(target=self._run, daemon=True, name="force-stream")

    def start(self):
        self._thread.start()

    def hold(self):
        self._last_hold = self._clock()

    def retarget(self, target: WrenchTarget):
        """Change the wrench while applying (ramped). Same frame only: the caller restarts for another."""
        with self._lock:
            self.target = target
            self._goal = np.asarray(target.wrench, float)

    def release(self):
        """Ramp down to zero, then end (reason 'released')."""
        self._release.set()

    def stop(self, reason: str):
        """Zero at once, then end."""
        self._reason = reason
        self._stop.set()

    @property
    def alive(self) -> bool:
        return self._thread.is_alive() and not self._stop.is_set()

    @property
    def releasing(self) -> bool:
        return self._release.is_set()

    def join(self, timeout: float = 2.0):
        self._thread.join(timeout)

    def _run(self):
        reason = None
        t_prev = self._clock()
        try:
            while not self._stop.is_set():
                now = self._clock()
                if not self._release.is_set():
                    if now - self._last_hold > self.hold_timeout_s:
                        reason = "not renewed (released, page closed or connection lost): zeroed at once"
                        break
                    if now - self._t0 > float(self.cfg["max_duration_s"]):
                        self._release.set()
                        self._reason = f"maximum duration ({self.cfg['max_duration_s']:g} s): released"
                with self._lock:
                    goal = np.zeros(6) if self._release.is_set() else self._goal
                self.current = ramp_toward(self.current, goal, min(now - t_prev, 0.1),
                                           float(self.cfg["ramp_force_nps"]), float(self.cfg["ramp_torque_nmps"]))
                t_prev = now
                self._send(self.current.tolist())
                if self._release.is_set() and not self.current.any():
                    break
                self._stop.wait(self.period)
        except Exception as e:                      # a failing TF lookup etc. ends the application
            reason = f"stopped: {e}: zeroed at once"
        finally:
            self.current = np.zeros(6)
            try:
                for _ in range(3):                  # zero is the one message that must arrive
                    self._send([0.0] * 6)
            finally:
                self._on_end(reason or self._reason)


def selected_axes_note(selected: Optional[Sequence[bool]], frame: str) -> str:
    """Human note for admittance's selected_axes (which control-frame axes respond at all)."""
    if selected is None:
        return ""
    names = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
    on = [n for n, s in zip(names, selected) if s]
    if len(on) == 6:
        return ""
    if not on:
        return "admittance has no selected axes: no force has any effect"
    return f"admittance responds only along {', '.join(on)} of {frame} (selected_axes): other components do nothing"
