"""In-process stand-in for RosBridge: no ROS, no robot.

Used by the unit tests and by `moveit_gui --fake` to try the interface.
Every number here is FAKE (made up for the demo), including the tool0 ->
robotiq_tcp offset and the detector bias; none of it describes the real UR7e.

Fake TF tree:
  base_link --(state)--> tool0 --(static, fake 150 mm)--> robotiq_tcp
  base_link == world; base = base_link rotated 180 deg about z (UR convention)
  base_link --> fake_detector/object   (true object + constant bias + noise)
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import transforms as tf
from .cart_speed import cart_accel_m_s2, trapezoid_duration
from .bridge_api import Snapshot, gripper_status_dict, reconnect_gripper
from .control_modes import detect_mode, moveit_controller_for
from .profile import RobotProfile

FAKE_T_TOOL0_ROBOTIQ = tf.make([0.0, 0.0, 0.150])
FAKE_OBJECT_TRUE = tf.mm_deg_to_T([500, 100, 20], [0, 0, 20])
FAKE_DETECTOR_BIAS = tf.mm_deg_to_T([6, -4, 3], [0, 0, 2.0])     # applied in the object frame
FAKE_DETECTOR_FRAME = "fake_detector/object"
FAKE_T_TOOL0_FT = tf.make([0.0, 0.0, 0.0433])
FAKE_PAYLOAD_MASS = 0.9
# The simulated RealSense on the wrist (eye-in-hand): its driver tree camera_link -> optical frame, where camera_link
# truly is on tool0 (what a hand-eye calibration should find), and its intrinsics. The ChArUco board sits 0.45 m in
# front of the camera at the start pose. Nothing publishes tool0 -> camera_link until a part does (SETUP > Parts).
FAKE_CAM_TOPIC, FAKE_CAM_INFO = "/camera/camera/color/image_rect_raw", "/camera/camera/color/camera_info"
FAKE_CAM_LINK, FAKE_CAM_OPTICAL = "camera_link", "camera_color_optical_frame"
FAKE_T_LINK_OPTICAL = tf.mm_deg_to_T([0, 15, 0], [-90, 0, -90])
FAKE_T_TOOL0_CAMLINK = tf.mm_deg_to_T([0, -75, 35], [0, -90, 90])
FAKE_CAM_K = [910.0, 0.0, 640.0, 0.0, 910.0, 360.0, 0.0, 0.0, 1.0]
FAKE_PAYLOAD_COG = np.array([0.01, 0.005, 0.06])


def fake_fk(q: List[float]) -> np.ndarray:
    """Not UR kinematics: a smooth made-up map from joints to a tool0 pose."""
    x = 0.4 * math.cos(q[0]) + 0.1 * math.cos(q[1])
    y = 0.4 * math.sin(q[0])
    z = 0.3 + 0.2 * math.sin(-q[1] - math.pi / 2)
    R = (tf.quat_to_matrix(tf.rpy_to_quat(0, 0, q[0]))
         @ tf.quat_to_matrix(tf.rpy_to_quat(math.pi, 0, 0))
         @ tf.quat_to_matrix(tf.rpy_to_quat(0, q[3] + math.pi / 2, 0))
         @ tf.quat_to_matrix(tf.rpy_to_quat(q[4] + math.pi / 2, 0, q[5])))
    return tf.from_rotation(R, [x, y, z])


class FakeBridge:
    def __init__(self, profile: RobotProfile, joint_speed: float = 1.0, move_time_scale: float = 1.0,
                 detector_noise: float = 1.0, seed: int = 0):
        self.profile = profile
        self._lock = threading.Lock()
        self._logs: deque = deque(maxlen=500)
        n = len(profile.joint_names)
        home = profile.waypoints.get("home")
        self._q = list(home.joints) if home and home.joints else [0.0] * n
        self._qd = [0.0] * n
        self.T_base_tool0 = fake_fk(self._q)
        self._cancel = threading.Event()
        self.stop_epoch = 0          # bumped by every cancel_motion (as RosBridge): a STOP during "planning" is not lost
        self.plan_delay = 0.0        # seconds of simulated MoveIt planning before a move starts (tests)
        self.joint_speed = joint_speed
        self.time_scale = move_time_scale
        self.last_cart_speed = 0.0          # m/s limit of the last pose move (0 = none)
        self.last_move_s = 0.0              # simulated duration of the last move_pose
        self.detector_noise = detector_noise
        self._rng = np.random.default_rng(seed)
        self.controllers: Dict[str, str] = {}
        for m in profile.modes.values():
            for c in m.activate:
                self.controllers[c] = "inactive"
        for c in profile.modes[profile.default_mode].activate:
            self.controllers[c] = "active"
        self.controllers["joint_state_broadcaster"] = "active"
        self.controllers["io_and_status_controller"] = "active"
        self.robot_mode = "RUNNING"
        self.safety_mode = "NORMAL"
        self.program_running = True
        self.program_paused = False
        self.pendant_program = "/programs/external_control.urp"   # what the pendant has loaded
        self.remote_control = True
        self.speed = 1.0
        self.gripper_pos = float(profile.gripper.get("open_position", 0.0))
        self.gripper_object = 3                  # Robotiq gOBJ: 3 = at target, 2 = object held
        self.gripper_fault = 0                   # Robotiq gFLT
        self.digital_out = [False] * 8
        self.freedrive = False
        # The lab's controller YAML (ur_controllers_gui.yaml), so SETUP > Controller gains has something to read
        self.node_params = {
            "/admittance_controller": {
                "admittance.stiffness": [250.0, 200.0, 180.0, 20.0, 20.0, 20.0],
                "admittance.damping_ratio": [2.5, 3.0, 2.8, 2.5, 2.5, 2.5],
                "admittance.mass": [3.0, 12.0, 10.0, 4.0, 4.0, 4.0],
                "admittance.selected_axes": [True, False, False, False, False, False],
                "admittance.joint_damping": 12.0, "ft_sensor.filter_coefficient": 0.1,
                "admittance.selected_axes_frame": "robotiq_tcp", "control.frame.id": "robotiq_tcp"},
            "/cartesian_impedance_controller": {
                **{f"stiffness.translation.{a}": v for a, v in zip("xyz", (150.0, 400.0, 400.0))},
                **{f"stiffness.rotation.{a}": 15.0 for a in "xyz"},
                **{f"damping.{k}.{a}": 1.0 for k in ("translation", "rotation") for a in "xyz"},
                "delta_tau_max": 0.1, "stiffness.update_stiffness": False, "damping.update_damping_factors": False},
            # ft_controllers.yaml sets no offsets: the broadcaster starts raw (offset.* added to the reading)
            "/ft/force_torque_sensor_broadcaster": {f"offset.{k}.{a}": 0.0 for k in ("force", "torque") for a in "xyz"},
        }
        # Simulated image topics: topic -> (encoding, width, height, hz). ZED and the vision midpoints image are
        # left out on purpose, so SETUP > Cameras shows a camera that is not set up.
        self.fake_cameras = {
            "/camera/camera/color/image_rect_raw": ("rgb8", 1280, 720, 30.0),
            "/camera/camera/depth/image_rect_raw": ("16UC1", 848, 480, 30.0),
            "/camera/camera/infra1/image_rect_raw": ("mono8", 848, 480, 30.0),
            "/scraping_vision/annotated_image": ("bgr8", 640, 480, 10.0),
        }
        self.fake_published = set()        # other topics a fake driver 'publishes' (SETUP > Drivers & cameras)
        self._image_watch = None
        self._image_stop = threading.Event()
        self.ft_live = True
        g = profile.gripper
        self.gripper_hw = {g["hardware"]: "active"} if g.get("hardware") else {}
        self.gripper_controllers = {c: "active" for c in (g.get("controllers")
                                    or ["robotiq_gripper_controller", "robotiq_activation_controller"])}
        self.fake_claims_available = True
        self.calls: List[tuple] = []          # for tests
        self.fail_next_switch = False
        # test hooks for fail-closed behaviour
        self.fake_joint_state_age: Optional[float] = 0.0
        self.fake_wrench_age: float = 0.0                   # tests set it > WRENCH_STALE_S to freeze the sensor
        self.shutdown_wrenches: list = []                   # zeros published by shutdown(), for tests
        self.fake_velocities_available = True
        self.stale_frames: set = set()
        self.extra_frames: Dict[str, np.ndarray] = {}   # child of tool0, e.g. a tool frame in TF
        self.part_frames: Dict[str, Tuple[str, np.ndarray]] = {}     # SETUP > Parts static TF: child -> (parent, T)
        self.part_objects: Dict[str, dict] = {}                       # SETUP > Parts collision objects by id
        self.parts_scene_readable = True
        T_base_opt = self.T_base_tool0 @ FAKE_T_TOOL0_CAMLINK @ FAKE_T_LINK_OPTICAL
        self.fake_board = None                                       # board spec (handeye.check_board) once used
        self.fake_T_base_board = T_base_opt @ tf.mm_deg_to_T([-122.5, -87.5, 450], [0, 0, 0])
        self.fake_action_ready = True          # MoveIt action server of the active controller
        self.fake_reference_result: Tuple[bool, str] = (True, "")
        self.fail_rollback = False             # every switch after the first is rejected
        self.servo_available = False
        self.servo_route_available: Dict[str, bool] = {}
        self.last_route: Optional[str] = None  # controller named in the last execution
        self.ft_noise = 0.01
        self.ft_raw_offset = np.zeros(6)      # test hook: what the bare sensor reads with nothing attached
        # test hook: a horizontal surface at base z = z_m; below it the tool is pushed back up with k N/m
        self.fake_surface: Optional[dict] = None
        # ros2_control hardware components (name -> lifecycle label), as list_hardware_components reports them
        self.hardware = {"ur7e": "active", "TopicSensorBridge": "active"}
        self.hardware_fails_again: set = set()   # test hook: re-activation does not hold
        self.servo_type: Optional[str] = None    # "joint" | "twist" after servo_command_type
        self.servo_route: Optional[str] = None
        self.fake_servo_status: Tuple[int, str] = (0, "No warnings")
        self._servo_t = 0.0
        self.servo_out_n = 0
        self.wrenches: deque = deque(maxlen=2000)   # (topic, frame, wrench) as published, for tests
        self.fake_robot_description: Optional[str] = None
        # MoveIt planning scene: world collision objects as move_group would hold them (keep-out zones)
        self.scene: Dict[str, dict] = {"user_table": {"frame": profile.base_link, "size": [1.0, 1.0, 0.01],
                                                      "T": tf.make([0, 0, -0.5])}}   # not ours: never touched
        self.scene_readable = True             # test hook: get_planning_scene unavailable
        self.scene_drop = 0                    # test hook: lose the next n publishes (subscription not matched)

    # ------------------------------------------------------------- state
    def log(self, msg: str):
        self._logs.append(f"{time.strftime('%H:%M:%S')}  {msg}")

    def drain_logs(self) -> List[str]:
        out = []
        while self._logs:
            out.append(self._logs.popleft())
        return out

    def snapshot(self) -> Snapshot:
        with self._lock:
            q = list(self._q)
            qd = list(self._qd)
        return Snapshot(
            connected={"ROS": True, "UR": True, "MoveIt": True, "Gripper": self.gripper_controllers.get(self.profile.gripper.get("controller")) == "active", "F/T": self.ft_live,
                       "Servo": self.servo_available},
            joints=q, joint_velocities=qd if self.fake_velocities_available else None,
            joint_state_age=self.fake_joint_state_age,
            robot_mode=self.robot_mode, safety_mode=self.safety_mode,
            program_running=self.program_running, speed_scaling=self.speed,
            program_state=("PLAYING" if self.program_running else self.program_paused and "PAUSED" or "STOPPED"),
            remote_control=self.remote_control,
            controllers=dict(self.controllers),
            claimed_interfaces=self.controller_claims() or {},
            gripper_controllers=dict(self.gripper_controllers),
            gripper_joint=[self.gripper_pos, 0.0, 0.0],
            wrench=self._fake_wrench() if self.ft_live else None,
            gripper_position=self.gripper_pos, freedrive=self.freedrive,
            wrench_age=self.fake_wrench_age if self.ft_live else None, controllers_age=0.0, spin_age=0.0,
            gripper_status=gripper_status_dict(SimpleNamespace(
                object_detection=self.gripper_object, motor_current=0.0, gripper_fault=self.gripper_fault,
                gripper_fault_severity=3 if self.gripper_fault else 0)),
        )

    def wrench_samples(self, since: float) -> List[list]:
        """Like RosBridge.wrench_history.since: 100 Hz samples (generated up to now, at most 1 s back)."""
        if not self.ft_live:
            return []
        now = time.time()
        t = max(since, now - 1.0)
        out = []
        t = (int(t * 100) + 1) / 100.0
        while t <= now:
            out.append([round(t, 3)] + [round(v, 4) for v in self._fake_wrench()])
            t += 0.01
        return out

    def _fake_wrench(self) -> List[float]:
        """A fixed fake payload seen through the current sensor orientation, plus noise."""
        with self._lock:
            R = (self.T_base_tool0 @ FAKE_T_TOOL0_FT)[:3, :3]
        f = R.T @ np.array([0.0, 0.0, -FAKE_PAYLOAD_MASS * 9.80665])
        wrench = np.concatenate([f, np.cross(FAKE_PAYLOAD_COG, f)])
        wrench += self._rng.normal(0.0, self.ft_noise, 6) * np.array([1, 1, 1, 0.1, 0.1, 0.1])
        wrench += self.ft_raw_offset
        if self.fake_surface:
            with self._lock:
                z = (self.T_base_tool0 @ FAKE_T_TOOL0_ROBOTIQ)[2, 3]
            pen = float(self.fake_surface["z_m"]) - z
            if pen > 0:
                f_up = R.T @ np.array([0.0, 0.0, float(self.fake_surface.get("k", 5000.0)) * pen])
                wrench[:3] += f_up
        off = self.node_params.get("/ft/force_torque_sensor_broadcaster", {})
        wrench += np.array([float(off.get(f"offset.{k}.{a}", 0.0)) for k in ("force", "torque") for a in "xyz"])
        return wrench.tolist()

    def joint_state_age(self) -> Optional[float]:
        return self.fake_joint_state_age

    def trajectory_action_ready(self, controller: str) -> bool:
        return self.fake_action_ready and self.controllers.get(controller) == "active"

    def verify_reference_at_current_pose(self, check: dict) -> Tuple[bool, str]:
        self.calls.append(("reference_check", check.get("state_topic")))
        return self.fake_reference_result

    def servo_ready(self, route: Optional[str] = None) -> bool:
        if route in self.servo_route_available:
            return self.servo_route_available[route]
        return self.servo_available

    def servo_status(self, route: Optional[str] = None) -> Tuple[Optional[int], str]:
        return (self.fake_servo_status if self.servo_ready(route) else (None, ""))

    def servo_output_count(self, route: Optional[str] = None) -> Optional[int]:
        return self.servo_out_n

    def joint_velocities_available(self) -> bool:
        return self.fake_velocities_available

    def is_stationary(self) -> Optional[bool]:
        age = self.fake_joint_state_age
        if age is None or age > self.profile.joint_state_timeout_s or not self.fake_velocities_available:
            return None
        with self._lock:
            return all(abs(v) < self.profile.stationary_joint_velocity for v in self._qd)

    # ------------------------------------------------------------------ TF
    def _T_base(self, frame: str) -> Optional[np.ndarray]:
        if frame in self.stale_frames:
            return None
        if frame in (self.profile.base_link, "world"):
            return np.eye(4)
        if frame == "base":
            return tf.make([0, 0, 0], tf.rpy_to_quat(0, 0, math.pi))
        with self._lock:
            T_tool0 = self.T_base_tool0.copy()
        if frame == "tool0":
            return T_tool0
        if frame == "ft_fts_link":
            return T_tool0 @ FAKE_T_TOOL0_FT
        if frame in self.extra_frames:
            return T_tool0 @ self.extra_frames[frame]
        if frame in self.part_frames:
            parent, T = self.part_frames[frame]
            Tp = self._T_base(parent) if parent != frame else None
            return None if Tp is None else Tp @ T
        if frame == FAKE_CAM_OPTICAL:
            Tl = self._T_base(FAKE_CAM_LINK) if FAKE_CAM_LINK in self.part_frames else None
            return None if Tl is None else Tl @ FAKE_T_LINK_OPTICAL
        if frame == "robotiq_tcp":
            return None if "tool0" in self.stale_frames else T_tool0 @ FAKE_T_TOOL0_ROBOTIQ
        if frame == FAKE_DETECTOR_FRAME:
            return self._detect()
        return None

    def _detect(self) -> np.ndarray:
        n = self.detector_noise
        noise = tf.mm_deg_to_T(self._rng.normal(0, 0.5 * n, 3), self._rng.normal(0, 0.3 * n, 3))
        return FAKE_OBJECT_TRUE @ FAKE_DETECTOR_BIAS @ noise

    def lookup_transform(self, parent: str, child: str) -> Optional[np.ndarray]:
        if (parent, child) == (FAKE_CAM_LINK, FAKE_CAM_OPTICAL):          # the camera driver's own static tree
            return FAKE_T_LINK_OPTICAL.copy()
        Tp, Tc = self._T_base(parent), self._T_base(child)
        if Tp is None or Tc is None:
            return None
        return tf.inv(Tp) @ Tc

    def read_pose_source(self, source_type: str, source: str, reference_frame: str) -> Optional[np.ndarray]:
        if source_type == "tf":
            return self.lookup_transform(reference_frame, source)
        if source_type == "pose_topic" and source == "/fake_detector/pose":
            return self.lookup_transform(reference_frame, FAKE_DETECTOR_FRAME)
        return None

    # ---------------------------------------------------------- controllers
    def list_controllers(self) -> Optional[Dict[str, str]]:
        return dict(self.controllers)

    def controller_claims(self) -> Optional[Dict[str, List[str]]]:
        """What ros2_control would report: chained controllers claim the downstream controller's
        reference interfaces, the last controller in a chain claims the hardware interfaces."""
        if not self.fake_claims_available:
            return None
        out: Dict[str, List[str]] = {}
        for mode in self.profile.modes.values():
            if not all(self.controllers.get(c) == "active" for c in mode.activate):
                continue
            chain = mode.chain or mode.activate
            for upstream, downstream in zip(chain, chain[1:]):
                out[upstream] = [f"{downstream}/{j}/position" for j in self.profile.joint_names]
            last = chain[-1]
            if mode.command_interface != "none":
                out[last] = [f"{j}/{mode.command_interface}" for j in self.profile.joint_names]
            else:
                out[last] = []
        return out

    def switch_controllers(self, activate, deactivate, strict=True, timeout_s=5.0) -> bool:
        self.calls.append(("switch", tuple(activate), tuple(deactivate)))
        if self.fail_rollback and sum(1 for c in self.calls if c[0] == "switch") > 1:
            return False
        if self.fail_next_switch:
            self.fail_next_switch = False
            return False
        if strict and any(c not in self.controllers for c in list(activate) + list(deactivate)):
            return False
        for c in deactivate:
            self.controllers[c] = "inactive"
        for c in activate:
            self.controllers[c] = "active"
        self.log(f"controllers: +{list(activate)} -{list(deactivate)}")
        return True

    def hardware_states(self) -> Optional[Dict[str, str]]:
        return dict(self.hardware)

    def activate_hardware(self, name: str) -> Tuple[bool, str]:
        self.calls.append(("activate_hardware", name))
        if name not in self.hardware:
            return False, f"{name} is not loaded"
        self.hardware[name] = "unconfigured" if name in self.hardware_fails_again else "active"
        return True, f"{name} active"

    # ---------------------------------------------------------------- servo
    def servo_command_type(self, kind: str, route: Optional[str] = None) -> Tuple[bool, str]:
        route = route or self.profile.servo.get("default_route")
        self.calls.append(("servo_type", route, kind))
        if not self.servo_ready(route):
            return False, "MoveIt Servo is not running"
        self.servo_type, self.servo_route, self._servo_t = kind, route, 0.0
        return True, f"servo command type {kind}"

    def _servo_route_active(self, route: Optional[str]) -> bool:
        cfg = (self.profile.servo.get("routes") or {}).get(route or "") or {}
        return detect_mode(self.profile, self.controllers) == cfg.get("control_mode")

    def _servo_dt(self) -> float:
        """Fake Servo: like the real one it only moves while commands keep coming (0.1 s timeout)."""
        now = time.monotonic()
        dt = 0.0 if self._servo_t == 0.0 else min(now - self._servo_t, 0.1)
        self._servo_t = now
        return dt

    def servo_twist(self, linear, angular, frame: str, route: Optional[str] = None):
        self.calls.append(("servo_twist", tuple(float(v) for v in linear),
                           tuple(float(v) for v in angular), frame, route))
        if self.servo_type != "twist" or route != self.servo_route or not self._servo_route_active(route):
            return
        dt = self._servo_dt()
        self.servo_out_n += 1
        w = np.asarray(angular, dtype=float)
        n = float(np.linalg.norm(w))
        with self._lock:
            T = self.T_base_tool0.copy()
            R = T[:3, :3]
            if n > 1e-12:
                u, a = w / n, n * dt
                K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
                R = (np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K) @ R
            self.T_base_tool0 = tf.from_rotation(R, T[:3, 3] + np.asarray(linear, dtype=float) * dt)
            moving = n > 1e-12 or float(np.linalg.norm(linear)) > 1e-12
            self._qd = [0.01 if moving else 0.0] + [0.0] * (len(self._q) - 1)

    def servo_joint_jog(self, velocities, route: Optional[str] = None):
        self.calls.append(("joint_jog", tuple(float(v) for v in velocities), route))
        if self.servo_type != "joint" or route != self.servo_route or not self._servo_route_active(route):
            return
        dt = self._servo_dt()
        self.servo_out_n += 1
        with self._lock:
            self._qd = [float(v) for v in velocities]
            self._q = [q + v * dt for q, v in zip(self._q, self._qd)]
            self.T_base_tool0 = fake_fk(self._q)

    # --------------------------------------------------------------- motion
    def cancel_motion(self):
        self.calls.append(("cancel",))
        self.stop_epoch += 1
        self._cancel.set()

    def _plan_wait(self):
        """Simulated planning time. Returns the stop epoch the move belongs to."""
        epoch = self.stop_epoch
        if self.plan_delay > 0:
            time.sleep(self.plan_delay)
        return epoch

    def _interpolate(self, target: List[float], vel: float, epoch: Optional[int] = None) -> bool:
        epoch = self.stop_epoch if epoch is None else epoch
        self._cancel.clear()
        speed = max(1e-3, self.joint_speed * vel * self.time_scale)
        dt = 0.02
        while True:
            if self._cancel.is_set() or self.stop_epoch != epoch:
                with self._lock:
                    self._qd = [0.0] * len(self._qd)
                self.log("motion cancelled")
                return False
            if self.robot_mode != "RUNNING" or self.program_running is False:
                with self._lock:                                  # held at speed scaling 0: waits, as on the UR
                    self._qd = [0.0] * len(self._qd)
                time.sleep(dt)
                continue
            with self._lock:
                err = [t - c for t, c in zip(target, self._q)]
                m = max(abs(e) for e in err) if err else 0.0
                if m < 1e-4:
                    self._q = list(target)
                    self._qd = [0.0] * len(self._q)
                    self.T_base_tool0 = fake_fk(self._q)
                    return True
                step = min(1.0, speed * dt / m)
                self._qd = [e * step / dt for e in err]
                self._q = [c + e * step for c, e in zip(self._q, err)]
                self.T_base_tool0 = fake_fk(self._q)
            time.sleep(dt / self.time_scale if self.time_scale > 1 else dt)

    def _route(self) -> bool:
        """Same rule as RosBridge: name the controller of the active mode, refuse if there is none."""
        self.last_route = moveit_controller_for(self.profile, self.controllers)
        if self.last_route is None:
            self.log("motion refused: no active mode accepts MoveIt trajectories")
            return False
        return True

    # ---------------------------------------------------------- plan / execute
    def plan_joints(self, joints, vel=0.1, acc=0.1):
        """Fake plan: a straight joint-space line, timed at joint_speed * vel."""
        self.calls.append(("plan_joints", tuple(round(j, 4) for j in joints)))
        with self._lock:
            q0 = list(self._q)
        span = max(abs(a - b) for a, b in zip(joints, q0)) if q0 else 0.0
        duration = max(0.2, span / max(1e-3, self.joint_speed * vel))
        pts = []
        for i in range(11):
            s = i / 10
            t = duration * s
            pts.append(SimpleNamespace(positions=[a + (b - a) * s for a, b in zip(q0, joints)],
                                       time_from_start=SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9))))
        return SimpleNamespace(joint_names=list(self.profile.joint_names), points=pts)

    def plan_pose(self, target_link, T_base_link, cartesian=False, vel=0.1, acc=0.1, cart_speed=0.0,
                  speed_point=None):
        self.calls.append(("plan_pose", target_link, tuple(np.round(T_base_link[:3, 3], 5)), bool(cartesian)))
        self.last_cart_speed = float(cart_speed or 0.0) if cartesian else 0.0
        self.log("fake bridge has no IK: pose plans are not simulated (joint targets are)")
        return None

    def execute_trajectory(self, trajectory) -> bool:
        self.calls.append(("execute", len(trajectory.points)))
        epoch = self.stop_epoch
        if not self._route():
            return False
        return self._interpolate(list(trajectory.points[-1].positions), 1.0, epoch)

    # ------------------------------------------------------------- force / model
    def publish_wrench(self, topic: str, frame: str, wrench):
        with self._lock:
            self.wrenches.append((topic, frame, [round(float(v), 4) for v in wrench]))

    def robot_description(self):
        return self.fake_robot_description

    def move_joints(self, joints, vel=0.1, acc=0.1) -> bool:
        self.calls.append(("movej", tuple(round(j, 4) for j in joints)))
        if not self._route():
            return False
        self.log(f"MoveJ -> {[round(j, 3) for j in joints]}")
        epoch = self._plan_wait()
        return self._interpolate(list(joints), vel, epoch)

    def move_pose(self, target_link: str, T_base_link: np.ndarray, cartesian=False, vel=0.1, acc=0.1,
                  cart_speed=0.0, speed_point=None) -> bool:
        """Fake: moves the tool0 pose directly (no IK); joints stay where they are. A linear move with cart_speed
        (m/s) takes what RosBridge's constant-speed retime takes: the straight distance of speed_point (the TCP, xyz
        in target_link) / cart_speed + the ramp time (cart_speed.trapezoid_duration). last_move_s is that simulated
        robot time (before move_time_scale)."""
        self.calls.append(("move_pose", target_link, tuple(np.round(T_base_link[:3, 3], 5)), bool(cartesian)))
        self.last_cart_speed = float(cart_speed or 0.0) if cartesian else 0.0
        if not self._route():
            return False
        if target_link == "tool0":
            T_goal = T_base_link
        elif target_link == "robotiq_tcp":
            T_goal = T_base_link @ tf.inv(FAKE_T_TOOL0_ROBOTIQ)
        else:
            self.log(f"fake bridge cannot plan for link {target_link}")
            return False
        self.log(f"{'MoveL' if cartesian else 'MovePose'} {target_link} -> "
                 f"{[round(v, 4) for v in T_base_link[:3, 3]]}")
        epoch = self._plan_wait()
        self._cancel.clear()
        with self._lock:
            T0 = self.T_base_tool0.copy()
        q0, q1 = tf.matrix_to_quat(T0[:3, :3]), tf.matrix_to_quat(T_goal[:3, :3])
        if float(np.dot(q0, q1)) < 0:
            q1 = -q1
        duration = 0.2 / max(vel, 0.05)                    # 10 / vel steps of 0.02 s (before time_scale)
        if self.last_cart_speed > 0:
            T_tool0_link = tf.inv(T_goal) @ T_base_link       # target_link in tool0, fixed
            pt = T_tool0_link @ np.r_[np.asarray(speed_point if speed_point is not None else [0, 0, 0], float), 1.0]
            dist = float(np.linalg.norm((T_goal @ pt)[:3] - (T0 @ pt)[:3]))
            if dist > 1e-5:                                   # as RosBridge: constant speed, ramps at the
                duration = trapezoid_duration(dist, self.last_cart_speed,   # Cartesian acceleration (cart_speed.py)
                                              cart_accel_m_s2(self.profile, acc))
        self.last_move_s = duration
        steps = max(2, int(duration / 0.02 / self.time_scale))
        for i in range(1, steps + 1):
            if self._cancel.is_set() or self.stop_epoch != epoch:
                self.log("motion cancelled")
                return False
            a = i / steps
            q = (1 - a) * q0 + a * q1
            with self._lock:
                self.T_base_tool0 = tf.make((1 - a) * T0[:3, 3] + a * T_goal[:3, 3], q / np.linalg.norm(q))
            time.sleep(0.02)
        return True

    def set_fake_tool0(self, T_base_tool0: np.ndarray):
        """Test / demo helper: teleport tool0 (stands in for freedrive)."""
        with self._lock:
            self.T_base_tool0 = np.array(T_base_tool0, dtype=float)

    # -------------------------------------------------------------- gripper
    def gripper_move(self, position: float, effort: float) -> bool:
        self.calls.append(("gripper", round(position, 3)))
        self.log(f"gripper -> {position:.3f} ({effort:.0f} N)")
        time.sleep(0.2 / self.time_scale)
        self.gripper_pos = position
        return True

    def fake_gripper_start_failed(self):
        """Test hook: the stack started while the robot was powered off (what the real launch log shows)."""
        g = self.profile.gripper
        self.gripper_hw = {g["hardware"]: "unconfigured"} if g.get("hardware") else {}
        first = (g.get("controllers") or [g.get("controller")])[0]
        self.gripper_controllers = {first: "unconfigured"}

    def gripper_reconnect(self) -> Tuple[bool, str]:
        g = self.profile.gripper

        def hw_activate(name):
            if self.robot_mode not in ("IDLE", "RUNNING"):           # no tool power: the gripper does not answer
                return False, "unconfigured"
            self.gripper_hw[name] = "active"
            return True, "active"

        def need_hw():
            return all(v == "active" for v in self.gripper_hw.values())

        def set_state(name, new):
            self.gripper_controllers[name] = new
            return True

        ops = SimpleNamespace(
            hw_states=lambda: dict(self.gripper_hw), hw_activate=hw_activate,
            controllers=lambda: dict(self.gripper_controllers),
            set_params_file=lambda c, f: (self.calls.append(("params_file", c, f)) or True, "ok"),
            load=lambda c: set_state(c, "unconfigured"),
            configure=lambda c: need_hw() and set_state(c, "inactive"),
            activate=lambda names: need_hw() and all(set_state(c, "active") for c in names))
        self.calls.append(("gripper_reconnect",))
        return reconnect_gripper(ops, g.get("hardware"), list(g.get("controllers") or [g.get("controller")]),
                                 g.get("params_file"), self.log)

    def gripper_reactivate(self) -> Tuple[bool, str]:
        self.gripper_fault = 0
        return True, "gripper reactivated (fake)"

    # ------------------------------------------------------------- UR stuff
    def set_node_parameters(self, node_name, params):
        self.calls.append(("set_parameters", node_name, dict(params)))
        self.node_params.setdefault(node_name, {}).update(params)
        return True, "parameters set"

    def get_node_parameters(self, node_name, names):
        if node_name not in self.node_params:
            return None                                   # like a node whose parameter service does not answer
        p = self.node_params.get(node_name, {})
        return {n: p.get(n) for n in names}

    def dashboard(self, command: str) -> Tuple[bool, str]:
        self.calls.append(("dashboard", command))
        effects = {
            "power_on": lambda: setattr(self, "robot_mode", "IDLE"),
            "power_off": lambda: (setattr(self, "robot_mode", "POWER_OFF"), setattr(self, "program_running", False),
                                  setattr(self, "program_paused", False)),
            "brake_release": lambda: setattr(self, "robot_mode", "RUNNING"),
            "play": lambda: (setattr(self, "program_running", True), setattr(self, "program_paused", False)),
            "pause": lambda: (setattr(self, "program_running", False), setattr(self, "program_paused", True)),
            "stop": lambda: (setattr(self, "program_running", False), setattr(self, "program_paused", False)),
            "unlock_protective_stop": lambda: setattr(self, "safety_mode", "NORMAL"),
        }
        if command in effects:
            effects[command]()
        self.log(f"dashboard {command}: ok")
        return True, f"{command}: ok (fake)"

    def loaded_program(self) -> Tuple[bool, str]:
        return True, self.pendant_program

    def load_program(self, filename: str) -> Tuple[bool, str]:
        return True, f"loaded {filename} (fake)"

    def resend_robot_program(self) -> Tuple[bool, str]:
        self.program_running = True
        return True, "program resent (fake)"

    def zero_ft_sensor(self) -> Tuple[bool, str]:
        return True, "UR wrist F/T zeroed (fake)"

    def set_speed_slider(self, fraction: float) -> bool:
        self.calls.append(("speed", fraction))
        self.speed = fraction
        return True

    def set_digital_out(self, pin: int, state: bool) -> bool:
        self.calls.append(("io", pin, state))
        self.digital_out[pin] = state
        return True

    def set_freedrive_keepalive(self, on: bool):
        self.freedrive = on

    # ------------------------------------------------------ planning scene
    def scene_set_box(self, object_id: str, size_m, T_base_box: np.ndarray) -> Tuple[bool, str]:
        self.calls.append(("scene_set_box", object_id, [float(v) for v in size_m], np.array(T_base_box)))
        if self.scene_drop > 0:
            self.scene_drop -= 1
            return True, "published"
        self.scene[object_id] = {"frame": self.profile.base_link, "size": [float(v) for v in size_m],
                                 "T": np.array(T_base_box, dtype=float)}
        return True, "published"

    def scene_remove(self, object_id: str) -> Tuple[bool, str]:
        self.calls.append(("scene_remove", object_id))
        self.scene.pop(object_id, None)
        return True, "published"

    def scene_objects(self) -> Optional[Dict[str, dict]]:
        if not self.scene_readable:
            return None
        return {k: {"frame": v["frame"], "size": list(v["size"]), "T": np.array(v["T"])} for k, v in self.scene.items()}

    def set_scaling(self, vel: float, acc: float):
        pass

    # ---------------------------------------------------------------- topics
    FAKE_TOPICS = [("/joint_states", "sensor_msgs/msg/JointState"), ("/tf", "tf2_msgs/msg/TFMessage"),
                   ("/tf_static", "tf2_msgs/msg/TFMessage"), ("/rosout", "rcl_interfaces/msg/Log"),
                   ("/speed_scaling_state_broadcaster/speed_scaling", "std_msgs/msg/Float64"),
                   ("/ft_data", "geometry_msgs/msg/WrenchStamped"),
                   ("/robotiq_gripper_controller/gripper_cmd/_action/status", "action_msgs/msg/GoalStatusArray"),
                   ("/policy_runner/status", "std_msgs/msg/String"),
                   ("/display_planned_path", "moveit_msgs/msg/DisplayTrajectory")]

    def topic_types(self):
        return sorted(self.FAKE_TOPICS + [(t, "sensor_msgs/msg/Image") for t in self.fake_cameras]
                      + [(t + "/compressed", "sensor_msgs/msg/CompressedImage") for t in self.fake_cameras])

    # ---------------------------------------------------------------- cameras
    def image_topics(self):
        return sorted((t, "sensor_msgs/msg/Image") for t in self.fake_cameras)

    def topic_publishers(self, topic):
        return 1 if topic in self.fake_cameras or topic in self.fake_published else 0

    def watch_image(self, topic):
        from .camera import ImageWatch
        self._image_stop.set()
        self._image_watch = None
        if not topic:
            return True, "preview off"
        if topic not in self.fake_cameras:
            return False, f"{topic}: no image publisher"
        enc, w, h, hz = self.fake_cameras[topic]
        watch = ImageWatch(topic)
        stop = self._image_stop = threading.Event()

        def run():
            k = 0
            while not stop.wait(1.0 / hz):
                data, step = fake_image(enc, w, h, k)
                watch.on_image(enc, w, h, step, data, False, "camera_color_optical_frame")
                k += 1
        threading.Thread(target=run, daemon=True, name="fake-camera").start()
        self._image_watch = watch
        return True, f"watching {topic}"

    def watched_image(self):
        return self._image_watch

    # ---------------------------------------------------------------- parts (SETUP > Parts)
    def parts_publish(self, frames, objects, remove_ids):
        self.calls.append(("parts_publish", len(frames), [o["id"] for o in objects], [i for i, _ in remove_ids]))
        for parent, child, T in frames:
            self.part_frames[child] = (parent, np.array(T, dtype=float))
        for oid, _ in remove_ids:
            self.part_objects.pop(oid, None)
        for o in objects:
            self.part_objects[o["id"]] = dict(o)
        return True, "published"

    def parts_scene_ids(self):
        if not self.parts_scene_readable:
            return None
        return {i: (o["link"] if o["attach"] else "") for i, o in self.part_objects.items()}

    def tf_parent(self, frame: str) -> Optional[str]:
        if frame == FAKE_CAM_OPTICAL:
            return FAKE_CAM_LINK
        return self.part_frames.get(frame, (None,))[0]

    # ---------------------------------------------------------------- camera calibration
    def camera_info(self, topic: str, timeout_s: float = 3.0):
        if topic != FAKE_CAM_INFO:
            return None, f"{topic}: no CameraInfo within {timeout_s:.0f} s"
        return {"K": list(FAKE_CAM_K), "D": [0.0] * 5, "model": "plumb_bob", "width": 1280, "height": 720,
                "frame_id": FAKE_CAM_OPTICAL}, "ok"

    def grab_image(self, topic: str, timeout_s: float = 3.0):
        """The wrist camera's view of the board from where the fake arm is now (rendered, a little noise)."""
        from . import handeye as he
        if topic != FAKE_CAM_TOPIC:
            return None, f"{topic}: no image within {timeout_s:.0f} s"
        b = self.fake_board or he.check_board(None)
        with self._lock:
            T_base_opt = self.T_base_tool0 @ FAKE_T_TOOL0_CAMLINK @ FAKE_T_LINK_OPTICAL
        T_cam_board = tf.inv(T_base_opt) @ self.fake_T_base_board
        if T_cam_board[2, 3] <= 0.05:
            return np.full((720, 1280), 110, np.uint8), FAKE_CAM_OPTICAL
        self._grab_n = getattr(self, "_grab_n", 0) + 1
        return he.render_board(b, FAKE_CAM_K, T_cam_board, (1280, 720), seed=self._grab_n), FAKE_CAM_OPTICAL

    def grab_photo(self, topic: str, timeout_s: float = 3.0):
        """A generated JPEG of a simulated camera (a program's photo step)."""
        from .camera import decode_image, encode_jpeg
        base = topic[:-len("/compressed")] if topic.endswith("/compressed") else topic
        if base not in self.fake_cameras:
            return None, f"{topic}: no image within {timeout_s:.0f} s"
        enc, w, h, _hz = self.fake_cameras[base]
        self._grab_n = getattr(self, "_grab_n", 0) + 1
        data, step = fake_image(enc, w, h, self._grab_n)
        return encode_jpeg(decode_image(enc, w, h, step, data, max_width=w)), ""

    def shutdown(self):
        self._image_stop.set()
        self.set_freedrive_keepalive(False)
        self.cancel_motion()
        for m in self.profile.modes.values():            # like RosBridge: zero every wrench target on the way out
            topic = m.wrench.get("topic")
            if topic:
                for _ in range(3):
                    self.publish_wrench(topic, m.wrench.get("frame", self.profile.base_link), [0.0] * 6)
                    self.shutdown_wrenches.append(topic)


def fake_image(encoding: str, w: int, h: int, k: int):
    """A grey bench with a vial seen from above, slowly drifting: enough to see the preview is live."""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy, r = w * (0.5 + 0.05 * np.sin(k / 30.0)), h * 0.5, min(w, h) * 0.18
    d = np.hypot(xx - cx, yy - cy)
    grey = 90 + 60 * xx / w
    grey = np.where(np.abs(d - r) < max(3, r * 0.06), 230, np.where(d < r, 40 + 30 * d / r, grey))
    if encoding == "16UC1":
        depth = np.where(d < r, 420 + 40 * (1 - d / r), 500 + 20 * yy / h).astype("<u2")
        return depth.tobytes(), w * 2
    if encoding == "mono8":
        return grey.astype(np.uint8).tobytes(), w
    rgb = np.stack([grey, grey * 0.95, grey * 0.85], axis=2).astype(np.uint8)
    if encoding == "bgr8":
        rgb = rgb[..., ::-1]
    return np.ascontiguousarray(rgb).tobytes(), w * 3
