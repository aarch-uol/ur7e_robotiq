"""Jogging, without Qt: shared by the desktop and web front ends through RobotCore.

Two kinds, as on a teach pendant:

* Step jog: one press = one short MoveIt move (Cartesian moves are linear). Works in every mode that takes
  MoveIt trajectories, and goes through the same rules as any other manual move.
* Hold-to-jog (continuous): MoveIt Servo, while a button or a spring-back slider is held. The robot PC streams
  the Servo commands, never the browser: the front end starts a hold and must renew it every < HOLD_TIMEOUT_S
  (sliders send their values with each renewal). Release, a lost network, a closed tab or a frozen UI all end
  the renewals, and streaming stops (Servo itself also halts when commands stop arriving).
  Each logical mode has its own Servo node. Position temporarily switches scaled_joint_trajectory_controller ->
  joint_trajectory_controller, which /position_servo_node streams JointTrajectory to; Admittance has /servo_node
  streaming to admittance_trajectory_controller. Both are trajectory outputs: MoveIt 2.12 Servo starts every
  Float64MultiArray step from the measured state, which a lagging real arm turns into almost no motion.

Cartesian axes are in the base frame or in the active TCP's frame; rotations turn about the active TCP in
both kinds (Servo moves its own tip, tool0, so the twist sent to Servo carries the lever-arm term).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from . import transforms as tf

CART_AXES = ("x", "y", "z", "rx", "ry", "rz")
FRAMES = ("base", "tool")
HOLD_TIMEOUT_S = 0.5          # a hold not renewed for this long ends streaming (Tailscale relays add jitter)
RESTORE_DELAY_S = 0.25        # after the zero command, before Position takes the joints back from Servo
STREAM_HZ = 100.0              # old MoveIt-GUI streamed Servo commands from a 10 ms ROS timer

DEFAULTS = {
    "step_mm": [0.5, 1.0, 5.0, 10.0],
    "step_deg": [0.5, 1.0, 5.0, 10.0],
    # Hold-to-jog top speeds at 100 % of the Velocity setting (speed_units, fed to Servo as-is).
    "max_linear_mps": 0.05,
    "max_angular_dps": 15.0,
    "max_joint_dps": 15.0,
}


def jog_config(profile) -> dict:
    cfg = dict(DEFAULTS)
    cfg.update(getattr(profile, "jog", None) or {})
    return cfg


class JogError(ValueError):
    pass


@dataclass
class JogCommand:  # one axis, one direction (buttons)
    kind: str          # "cartesian" | "joint"
    axis: int          # 0..5 (x y z rx ry rz) or the joint index
    sign: int          # +1 / -1
    frame: str = "base"

    @classmethod
    def parse(cls, kind: str, axis, sign, frame: str = "base", n_joints: int = 6) -> "JogCommand":
        if kind not in ("cartesian", "joint"):
            raise JogError(f"unknown jog kind '{kind}'")
        if isinstance(axis, str) and kind == "cartesian":
            if axis not in CART_AXES:
                raise JogError(f"unknown axis '{axis}'")
            axis = CART_AXES.index(axis)
        try:
            axis = int(axis)
            sign = int(sign)
        except (TypeError, ValueError):
            raise JogError("axis and direction must be numbers") from None
        if sign not in (1, -1):
            raise JogError("direction must be +1 or -1")
        if not 0 <= axis < (6 if kind == "cartesian" else n_joints):
            raise JogError(f"axis {axis} out of range")
        if kind == "cartesian" and frame not in FRAMES:
            raise JogError(f"jog frame must be one of {', '.join(FRAMES)}")
        return cls(kind, axis, sign, frame)

    def describe(self) -> str:
        d = "+" if self.sign > 0 else "−"
        if self.kind == "joint":
            return f"J{self.axis + 1}{d}"
        return f"{CART_AXES[self.axis].upper()}{d} ({self.frame})"


@dataclass
class JogVelocity:
    """A velocity request for all axes at once (sliders): values in -1..1 of the top speed, before shaping."""
    kind: str          # "cartesian" | "joint"
    values: List[float]
    frame: str = "base"

    @classmethod
    def parse(cls, kind: str, values, frame: str = "base", n_joints: int = 6) -> "JogVelocity":
        if kind not in ("cartesian", "joint"):
            raise JogError(f"unknown jog kind '{kind}'")
        if kind == "cartesian" and frame not in FRAMES:
            raise JogError(f"jog frame must be one of {', '.join(FRAMES)}")
        return cls(kind, parse_values(values, 6 if kind == "cartesian" else n_joints), frame)

    @classmethod
    def from_command(cls, cmd: JogCommand, n: int) -> "JogVelocity":
        v = [0.0] * (6 if cmd.kind == "cartesian" else n)
        v[cmd.axis] = float(cmd.sign)
        return cls(cmd.kind, v, cmd.frame)

    def describe(self) -> str:
        names = CART_AXES if self.kind == "cartesian" else [f"j{i + 1}" for i in range(len(self.values))]
        moving = [f"{n.upper()}{'+' if v > 0 else '−'}" for n, v in zip(names, self.values) if v]
        what = " ".join(moving) or "0"
        return f"{what} ({self.frame})" if self.kind == "cartesian" else what


def parse_values(values, n: int) -> List[float]:
    try:
        v = [float(x) for x in values]
    except (TypeError, ValueError):
        raise JogError("jog values must be numbers") from None
    if len(v) != n:
        raise JogError(f"expected {n} jog values, got {len(v)}")
    if not all(math.isfinite(x) and -1.0 <= x <= 1.0 for x in v):
        raise JogError("jog values must be between -1 and 1")
    return v


def shape(x: float) -> float:
    """Slider response: quadratic, so the first half of the travel is the slow, fine half (25 % speed)."""
    return math.copysign(x * x, x)


def _axis_vector(cmd: JogCommand, R_base_tcp: np.ndarray) -> np.ndarray:
    """Unit direction of a Cartesian jog axis, expressed in the base frame."""
    e = np.zeros(3)
    e[cmd.axis % 3] = float(cmd.sign)
    return R_base_tcp @ e if cmd.frame == "tool" else e


def cartesian_step(T_base_tcp: np.ndarray, cmd: JogCommand, step_m: float, step_rad: float) -> np.ndarray:
    """Target TCP pose after one step. Rotations turn about the TCP point."""
    R, p = T_base_tcp[:3, :3], T_base_tcp[:3, 3]
    u = _axis_vector(cmd, R)
    if cmd.axis < 3:
        return tf.from_rotation(R, p + u * step_m)
    return tf.from_rotation(_rotation(u, step_rad) @ R, p)


def _rotation(u: np.ndarray, angle: float) -> np.ndarray:
    """Rotation matrix about the unit axis u (Rodrigues)."""
    K = np.array([[0.0, -u[2], u[1]], [u[2], 0.0, -u[0]], [-u[1], u[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def joint_step(q: Sequence[float], cmd: JogCommand, step_rad: float) -> List[float]:
    out = list(q)
    out[cmd.axis] += cmd.sign * step_rad
    return out


def servo_twist(cmd: JogCommand, T_base_tcp: np.ndarray, T_base_tip: np.ndarray,
                lin_speed: float, ang_speed: float) -> Tuple[List[float], List[float]]:
    """Twist of Servo's tip (tool0), in the base frame, that moves the TCP as one jog axis asks."""
    return servo_twist_vec(JogVelocity.from_command(cmd, 6), T_base_tcp, T_base_tip, lin_speed, ang_speed)


def servo_twist_vec(jv: JogVelocity, T_base_tcp: np.ndarray, T_base_tip: np.ndarray,
                    lin_speed: float, ang_speed: float) -> Tuple[List[float], List[float]]:
    """Twist of Servo's tip (tool0), in the base frame, for a TCP velocity of values x top speeds (in the base
    or the TCP axes). Rotations turn about the TCP: a rotation w moves the tip with v = w x (p_tip - p_tcp)."""
    R = T_base_tcp[:3, :3] if jv.frame == "tool" else np.eye(3)
    lin = R @ np.asarray(jv.values[:3], dtype=float) * lin_speed
    w = R @ np.asarray(jv.values[3:6], dtype=float) * ang_speed
    v = lin + np.cross(w, T_base_tip[:3, 3] - T_base_tcp[:3, 3])
    return v.tolist(), w.tolist()


def servo_route(profile, mode: str) -> Tuple[Optional[str], dict]:
    """Return the configured Servo route for logical control `mode`.

    Each route owns its Servo node/topics and names the ros2_control mode that
    must be active while streaming. Position therefore uses the UR-native
    forward-position controller, while Admittance keeps its trajectory chain.
    """
    sv = getattr(profile, "servo", None) or {}
    routes = sv.get("routes") or {}
    cfg = routes.get(mode)
    return (mode, cfg) if isinstance(cfg, dict) else (None, {})


class JogStream:
    """One hold-to-jog. Calls send() at STREAM_HZ until stopped or not renewed; then sends zero once and
    calls on_end(reason). Thread-safe: start/hold/stop come from request threads."""

    def __init__(self, send: Callable[[], None], send_zero: Callable[[], None], on_end: Callable[[str], None],
                 hold_timeout_s: float = HOLD_TIMEOUT_S, rate_hz: float = STREAM_HZ,
                 clock: Callable[[], float] = time.monotonic):
        self._send, self._send_zero, self._on_end = send, send_zero, on_end
        self.hold_timeout_s, self.period = hold_timeout_s, 1.0 / rate_hz
        self._clock = clock
        self._last_hold = clock()
        self._stop = threading.Event()
        self._reason = "released"
        self._thread = threading.Thread(target=self._run, daemon=True, name="jog-stream")

    def start(self):
        self._thread.start()

    def hold(self):
        self._last_hold = self._clock()

    def stop(self, reason: str = "released"):
        self._reason = reason
        self._stop.set()

    @property
    def alive(self) -> bool:
        return self._thread.is_alive() and not self._stop.is_set()

    def join(self, timeout: float = 1.0):
        self._thread.join(timeout)

    def _run(self):
        reason = None
        try:
            while not self._stop.is_set():
                if self._clock() - self._last_hold > self.hold_timeout_s:
                    reason = "hold not renewed (button released or connection lost)"
                    break
                self._send()
                self._stop.wait(self.period)
        except Exception as e:                      # a failing TF lookup etc. ends the jog, never continues it
            reason = f"stopped: {e}"
        finally:
            try:
                self._send_zero()
            finally:
                self._on_end(reason or self._reason)
