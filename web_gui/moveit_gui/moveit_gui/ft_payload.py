"""GUI workflow around ft_sensor_calibration/calibration_math.py (UR poses, GUI owned steps).

The workflow owns moving to the poses, collecting ATI data, reading the sensor orientation and the
progress. The maths (mass, CoG, bias, observability, quality gates) stays in calibration_math and is
imported, not copied. Results are saved next to the GUI calibration; controller YAML is never touched.
A passed result can be applied, on request, to the running admittance controller's gravity compensation.
"""
from __future__ import annotations

import importlib
import json
import os
import sys
import threading
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from . import transforms as tf
from .bridge_api import wrench_fresh
from .calibration import config_dir

DEFAULT_SOURCE = "~/franka_ros2/ft_sensor_calibration"
G = 9.80665          # calibration_math.GRAVITY: the fitted mass and the applied weight use the same g


def _atomic_write(path: Path, text: str):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp",
                                         mode="w", encoding="utf-8", delete=False) as f:
            tmp = Path(f.name)
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        tmp = None
        fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


class PayloadCalibrationError(RuntimeError):
    pass


def load_calibration_math():
    """The real module, from the environment, MOVEIT_GUI_FT_CALIBRATION_PATH or ~/franka_ros2; else the GUI's copy."""
    src = os.path.expanduser(os.environ.get("MOVEIT_GUI_FT_CALIBRATION_PATH", DEFAULT_SOURCE))
    for attempt in (False, True):
        try:
            return importlib.import_module("ft_sensor_calibration.calibration_math")
        except ImportError:
            sys.modules.pop("ft_sensor_calibration", None)
            if attempt or not Path(src).is_dir():
                break
            sys.path.insert(0, src)
    try:                                    # the copy shipped with the GUI (the robot container has no ~/franka_ros2)
        return importlib.import_module("moveit_gui.vendor.ft_sensor_calibration.calibration_math")
    except ImportError as e:
        raise PayloadCalibrationError(f"ft_sensor_calibration.calibration_math not found (looked in {src}, and the "
                                      f"copy in moveit_gui failed: {e}); set MOVEIT_GUI_FT_CALIBRATION_PATH") from None


@dataclass
class PoseSample:
    name: str
    orientation_xyzw: List[float]
    mean_wrench: List[float]
    std_wrench: List[float]
    count: int


@dataclass
class PayloadResult:
    mass: float
    cog: List[float]
    quality: dict
    info: dict
    samples: List[PoseSample] = field(default_factory=list)
    accepted: bool = False          # the operator chose to use it although a check failed

    @property
    def passed(self) -> bool:
        return bool(self.quality.get("passed"))

    @property
    def usable(self) -> bool:
        return self.passed or self.accepted


class PayloadCalibration:
    """Steps through profile.ft_calibration. Blocking methods run in a worker thread."""

    def __init__(self, bridge, profile, sleep=time.sleep):
        self.bridge, self.profile = bridge, profile
        cfg = profile.ft_calibration
        self.poses = list(cfg.get("poses", []))
        self.sensor_frame = str(cfg.get("sensor_frame", "ft_fts_link"))
        self.reference_frame = str(cfg.get("reference_frame", profile.base_link))
        self.samples_per_pose = int(cfg.get("samples_per_pose", 50))
        self.sample_period_s = float(cfg.get("sample_period_s", 0.02))
        self.settle_s = float(cfg.get("settle_s", 2.0))
        self.velocity = float(cfg.get("velocity_scaling", 0.1))
        for p in self.poses:
            if len(p.get("joints", [])) != len(profile.joint_names):
                raise PayloadCalibrationError(f"pose '{p.get('name')}' does not have "
                                              f"{len(profile.joint_names)} joints")
        # Where a reviewed result can be applied: the admittance controller's gravity compensation.
        ap = cfg.get("apply_to") or {}
        self.apply_node = ap.get("node")
        self.apply_mode = ap.get("mode", "admittance")
        self.apply_frame = str(ap.get("frame", self.sensor_frame))
        self.samples: List[Optional[PoseSample]] = [None] * len(self.poses)
        self.result: Optional[PayloadResult] = None
        self.saved_path: Optional[Path] = None
        # (mode, params) after a successful apply: RobotCore keeps them with the saved controller gains
        self.on_applied = None
        self._stop = threading.Event()
        self._sleep = sleep

    def cancel(self):
        self._stop.set()
        self.bridge.cancel_motion()

    def reset(self):
        self._stop.clear()
        self.samples = [None] * len(self.poses)
        self.result = None

    def run_pose(self, index: int, on_progress: Callable[[int, int], None] = lambda *_: None) -> PoseSample:
        """Move to one pose, settle, verify stillness, collect the wrench and the sensor orientation."""
        pose = self.poses[index]
        self._stop.clear()
        if not self.bridge.move_joints(list(pose["joints"]), self.velocity, self.velocity):
            raise PayloadCalibrationError(f"{pose['name']}: motion failed or was cancelled")
        self._sleep(self.settle_s)
        if self.bridge.is_stationary() is not True:
            raise PayloadCalibrationError(f"{pose['name']}: the arm is not verifiably stationary")
        T = self.bridge.lookup_transform(self.reference_frame, self.sensor_frame)
        if T is None:
            raise PayloadCalibrationError(f"{pose['name']}: no transform {self.reference_frame} -> {self.sensor_frame}")
        data = []
        while len(data) < self.samples_per_pose:
            if self._stop.is_set():
                raise PayloadCalibrationError(f"{pose['name']}: cancelled")
            w = wrench_fresh(self.bridge.snapshot())
            if w is None:
                raise PayloadCalibrationError(f"{pose['name']}: the ATI stopped publishing (no fresh reading)")
            data.append(list(w))
            on_progress(len(data), self.samples_per_pose)
            self._sleep(self.sample_period_s)
        arr = np.asarray(data, dtype=float)
        sample = PoseSample(pose["name"], [float(v) for v in tf.matrix_to_quat(T[:3, :3])],
                            arr.mean(axis=0).tolist(), arr.std(axis=0).tolist(), len(arr))
        self.samples[index] = sample
        return sample

    def evaluate(self) -> PayloadResult:
        """Fit and quality-gate with calibration_math, from the poses collected so far (3 or more)."""
        got = [s for s in self.samples if s is not None]
        if len(got) < 3:
            raise PayloadCalibrationError(f"collect at least 3 poses first ({len(got)} so far): the fit needs "
                                          "3 different tool orientations")
        cm = load_calibration_math()
        orientations = [np.array(s.orientation_xyzw) for s in got]
        wrenches = [np.array(s.mean_wrench) for s in got]
        stds = [np.array(s.std_wrench) for s in got]
        # The software bias (SETUP > F/T bias) is often a zero taken with the tool on, i.e. ~ the tool's weight at
        # that pose: up to ~10 N, past the fit's own bias bounds (+-5 N), which then bends the mass. Estimate the
        # constant bias exactly first (linear in weight + bias) and let calibration_math fit the rest.
        pre = linear_bias(orientations, wrenches)
        mass, cog, info = cm.analyze_payload_measurements(orientations, wrenches, baseline_bias=pre,
                                                          estimate_residual_bias=True)
        # A constant bias is not a calibration error here: Zero now (payload kept) removes it at runtime, which
        # is the "runtime bias correction" the gate asks for. Reported, not failed.
        quality = cm.evaluate_calibration_quality(mass, cog, info, stds, max_total_force_bias=float("inf"),
                                                  max_total_torque_bias=float("inf"))
        tb = np.asarray(info.get("total_bias", np.zeros(6)), dtype=float)
        if np.linalg.norm(tb[:3]) > 0.75 or np.linalg.norm(tb[3:]) > 0.05:
            quality.setdefault("warnings", []).append(
                f"the reading carries a constant bias of {np.linalg.norm(tb[:3]):.2f} N / "
                f"{np.linalg.norm(tb[3:]):.3f} Nm (fitted out; Apply moves the F/T bias by it, no Zero needed)")
        self.result = PayloadResult(float(mass), [float(v) for v in cog], quality, info, got)
        return self.result

    def save(self, directory: Optional[Path] = None) -> Path:
        """Write the result to its own file. Applying it to a controller is a separate, deliberate step."""
        if self.result is None:
            raise PayloadCalibrationError("nothing to save: evaluate first")
        r = self.result
        directory = Path(directory) if directory else config_dir() / "ft_payload"
        directory.mkdir(parents=True, exist_ok=True)
        stamp = f"{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}"
        path = directory / f"{self.profile.name}_{stamp}.json"
        counter = 0
        while path.exists():
            counter += 1
            path = directory / f"{self.profile.name}_{stamp}_{counter}.json"
        _atomic_write(path, json.dumps({
            "robot": self.profile.name, "sensor_frame": self.sensor_frame, "reference_frame": self.reference_frame,
            "mass_kg": r.mass, "cog_m": r.cog, "passed": r.passed,
            "accepted_despite_failed_checks": bool(r.accepted and not r.passed),
            "issues": r.quality.get("issues", []), "warnings": r.quality.get("warnings", []),
            "metrics": _jsonable(r.quality.get("metrics", {})),
            "residual_bias": _jsonable(r.info.get("residual_bias")),
            "samples": [vars(s) for s in r.samples],
            "applied_to_controller": False,
            "admittance_yaml": self.admittance_yaml()}, indent=2))
        self.saved_path = path
        return path

    # ------------------------------------------------------- admittance controller
    def admittance_params(self) -> Dict[str, object]:
        """ros2_controllers admittance_controller (Jazzy) gravity_compensation parameters.

        CoG.force is the weight magnitude, mass * g, in N (the controller applies it along -Z of the world).
        CoG.pos is in gravity_compensation.frame.id, measured from that frame's origin: calibration_math fits
        the CoG in the sensor frame about the sensor origin, so the two frames must be the same.
        """
        if self.result is None:
            raise PayloadCalibrationError("nothing to apply: evaluate first")
        if self.apply_frame != self.sensor_frame:
            raise PayloadCalibrationError(f"the CoG is fitted in {self.sensor_frame}, the controller compensates in "
                                          f"{self.apply_frame}")
        return {"gravity_compensation.frame.id": self.sensor_frame,
                "gravity_compensation.CoG.pos": [round(float(v), 6) for v in self.result.cog],
                "gravity_compensation.CoG.force": round(float(self.result.mass) * G, 4)}

    def admittance_yaml(self) -> str:
        p = self.admittance_params()
        pos = ", ".join(f"{v:.6f}" for v in p["gravity_compensation.CoG.pos"])
        return ("gravity_compensation:\n"
                f"  frame:\n    id: {p['gravity_compensation.frame.id']}\n    external: false\n"
                f"  CoG:\n    pos: [{pos}]\n    force: {p['gravity_compensation.CoG.force']}"
                f"   # {self.result.mass:.3f} kg * {G}\n")

    def apply_to_admittance(self, current_mode: str) -> Tuple[bool, str]:
        """Set the parameters on the running controller node and read them back. Not saved to any YAML.

        Refused while admittance is the active mode: changing the compensation under an active admittance law
        is a step in the measured wrench, and the arm would move by weight / stiffness.
        """
        if not self.apply_node:
            return False, "profile ft_calibration.apply_to.node is not set"
        if self.result is None:
            return False, "nothing to apply: evaluate first"
        if not self.result.usable:
            return False, "the fit did not pass its quality checks (accept it first to use it anyway)"
        if current_mode == self.apply_mode:
            return False, f"switch out of {self.apply_mode} first: the new compensation would move the arm"
        params = self.admittance_params()
        ok, msg = self.bridge.set_node_parameters(self.apply_node, params)
        if not ok:
            return False, msg
        back = self.bridge.get_node_parameters(self.apply_node, list(params))
        for k, v in params.items():
            got = None if back is None else back.get(k)
            same = got == v if isinstance(v, str) else (
                got is not None and np.allclose(np.atleast_1d(got), np.atleast_1d(v), atol=1e-6))
            if not same:
                return False, f"{self.apply_node} did not take {k} (read back {got})"
        kept = False
        if self.on_applied is not None:
            try:
                self.on_applied(self.apply_mode, params)
                kept = True
            except Exception:                        # a profile without gains for that mode: not kept
                kept = False
        if self.saved_path and self.saved_path.exists():
            d = json.loads(self.saved_path.read_text())
            d["applied_to_controller"] = {"node": self.apply_node, "at": time.strftime("%Y-%m-%d %H:%M:%S")}
            _atomic_write(self.saved_path, json.dumps(d, indent=2))
        return True, (f"{self.apply_node}: CoG {params['gravity_compensation.CoG.pos']} m, weight "
                      f"{params['gravity_compensation.CoG.force']} N. " +
                      ("Saved: restored automatically after a controller restart." if kept else
                       "Running controller only: lost when the controller restarts (the web GUI saves and restores it)."))


def linear_bias(orientations_xyzw, wrenches) -> np.ndarray:
    """Constant [F; T] bias by linear least squares, gravity along -Z of the reference frame:
    F = w * g_s + bF  and  T = c x (w * g_s) + bT, solved for w, bF, then (w c), bT."""
    G_s, F, T = [], [], []
    for q, wr in zip(orientations_xyzw, wrenches):
        Rm = tf.quat_to_matrix(q)[:3, :3]
        G_s.append(Rm.T @ np.array([0.0, 0.0, -1.0]))
        F.append(np.asarray(wr[:3], float))
        T.append(np.asarray(wr[3:], float))
    n = len(G_s)
    A = np.zeros((3 * n, 4))
    for k, g in enumerate(G_s):
        A[3 * k:3 * k + 3, 0] = g
        A[3 * k:3 * k + 3, 1:] = np.eye(3)
    x = np.linalg.lstsq(A, np.concatenate(F), rcond=None)[0]
    w = x[0]
    # T = -[w g]x c + bT : linear in c and bT
    B = np.zeros((3 * n, 6))
    for k, g in enumerate(G_s):
        f = w * g
        B[3 * k:3 * k + 3, :3] = -np.array([[0, -f[2], f[1]], [f[2], 0, -f[0]], [-f[1], f[0], 0]])
        B[3 * k:3 * k + 3, 3:] = np.eye(3)
    y = np.linalg.lstsq(B, np.concatenate(T), rcond=None)[0]
    return np.concatenate([x[1:], y[3:]])


def _jsonable(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    return v
