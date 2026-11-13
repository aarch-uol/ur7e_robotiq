"""The one long-lived ROS 2 node behind the GUI (ROS 2 Jazzy, ur_robot_driver).

* one node, one MultiThreadedExecutor spinning in a background thread
* subscriptions only update a locked state object; the GUI polls snapshot()
* blocking calls (motion, services) are made from worker threads and wait on
  futures that the executor thread completes, never by spinning themselves
* every name comes from the RobotProfile

It attaches to an already-running driver + MoveIt stack; it never launches
or kills ROS processes.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from types import SimpleNamespace
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time

from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration as DurationMsg
from controller_manager_msgs.srv import (ConfigureController, ListControllers, ListHardwareComponents, LoadController,
                                         SetHardwareComponentState, SwitchController)
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from lifecycle_msgs.msg import State as LifecycleState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger

import numpy as np
import yaml

from . import cart_speed as cart_speed_mod
from . import transforms as tf
from .bridge_api import (DASHBOARD_COMMANDS, DASHBOARD_TIMEOUT_S, Snapshot, WrenchHistory, dashboard_lost,
                         dashboard_with_reconnect, gripper_status_dict, reconnect_gripper)
from .camera import IMAGE as IMAGE_TYPE, IMAGE_TYPES, ImageWatch
from .control_modes import ReferenceShiftError, moveit_controller_for, shift_to_reference
from .profile import RobotProfile, reorder_joints

try:  # UR-specific messages: optional so the GUI also works with non-UR profiles
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    from ur_dashboard_msgs.srv import GetLoadedProgram, GetProgramState, IsInRemoteControl, Load
    from ur_msgs.srv import SetIO, SetSpeedSliderFraction
    HAVE_UR_MSGS = True
except ImportError:  # pragma: no cover
    HAVE_UR_MSGS = False

try:
    from control_msgs.action import GripperCommand
except ImportError:  # pragma: no cover
    GripperCommand = None
try:
    from control_msgs.action import ParallelGripperCommand   # Jazzy+
except ImportError:  # pragma: no cover
    ParallelGripperCommand = None

try:  # Robotiq status (gOBJ, gCU, gFLT) from robotiq_gripper_status_broadcaster
    from robotiq_msgs.msg import GripperStatus
except ImportError:  # pragma: no cover
    GripperStatus = None

# The UR driver publishes robot mode, safety mode and program state latched, and only when they change: a
# volatile subscriber never learns the current value.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

try:
    from control_msgs.msg import JointJog
except ImportError:  # pragma: no cover
    JointJog = None

try:
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import ExecuteTrajectory
except ImportError:  # pragma: no cover
    FollowJointTrajectory = None
    ExecuteTrajectory = None

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:  # pragma: no cover
    Buffer = None


def _enum_name(msg_cls, value: int) -> str:
    """Map an int constant of a ROS msg class (e.g. RobotMode.RUNNING) to its name."""
    for attr in dir(msg_cls):
        if attr.isupper() and getattr(msg_cls, attr) == value and isinstance(getattr(msg_cls, attr), int):
            return attr
    return str(value)


def _pose_to_T(pose) -> np.ndarray:
    p, q = pose.position, pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    if not any(quat):                     # an unset Pose (all zeros) means identity
        quat = [0.0, 0.0, 0.0, 1.0]
    return tf.make([p.x, p.y, p.z], tf.normalize_quat(quat))


class CameraTopics:
    """Topics and camera images, read only (the camera preview, calibration, a program's photo, driver checks).
    Needs self.node, self.cb, self._tf (a tf2 Buffer or None), self._image_sub and self._image_watch."""

    def _next_image(self, topic: str, timeout_s: float):
        """(type, message) of the next message on an Image / CompressedImage topic, or (None, why)."""
        types = dict(self.node.get_topic_names_and_types()).get(topic, [])
        t = next((x for x in types if x in IMAGE_TYPES), None)
        if t is None:
            return None, f"{topic}: no image publisher"
        got, ev = {}, threading.Event()

        def on_msg(m):
            if not ev.is_set():
                got["m"] = m
                ev.set()
        if t == IMAGE_TYPE:
            from sensor_msgs.msg import Image as Msg
        else:
            from sensor_msgs.msg import CompressedImage as Msg
        sub = self.node.create_subscription(Msg, topic, on_msg, qos_profile_sensor_data, callback_group=self.cb)
        try:
            if not ev.wait(timeout_s):
                return None, f"{topic}: no image within {timeout_s:.0f} s"
        finally:
            self.node.destroy_subscription(sub)
        return t, got["m"]

    def grab_image(self, topic: str, timeout_s: float = 3.0):
        """(gray uint8 HxW at full resolution, frame_id) of the next message on an Image / CompressedImage topic,
        or (None, why)."""
        import cv2
        t, m = self._next_image(topic, timeout_s)
        if t is None:
            return None, m
        try:
            if t == IMAGE_TYPE:
                from .camera import to_gray
                img = to_gray(m.encoding, m.width, m.height, m.step, m.data, bool(m.is_bigendian))
            else:
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
                if img is None:
                    return None, f"{topic}: could not decode {m.format}"
        except Exception as e:
            return None, f"{topic}: {e}"
        return img, m.header.frame_id

    def grab_photo(self, topic: str, timeout_s: float = 3.0):
        """(JPEG bytes at full resolution, "") of the next frame of an image topic, or (None, why). A JPEG
        CompressedImage is passed through unchanged."""
        t, m = self._next_image(topic, timeout_s)
        if t is None:
            return None, m
        try:
            if t == IMAGE_TYPE:
                from .camera import decode_image, encode_jpeg
                return encode_jpeg(decode_image(m.encoding, m.width, m.height, m.step, m.data,
                                                bool(m.is_bigendian), max_width=max(1, int(m.width)))), ""
            if "jpeg" in str(m.format).lower() or "jpg" in str(m.format).lower():
                return bytes(m.data), ""
            import cv2
            from .camera import encode_jpeg
            bgr = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return None, f"{topic}: could not decode {m.format}"
            return encode_jpeg(bgr[..., ::-1]), ""
        except Exception as e:
            return None, f"{topic}: {e}"

    def camera_info(self, topic: str, timeout_s: float = 3.0) -> Tuple[Optional[dict], str]:
        from sensor_msgs.msg import CameraInfo
        got, ev = {}, threading.Event()

        def on_msg(m):
            if not ev.is_set():
                got["m"] = m
                ev.set()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        sub = self.node.create_subscription(CameraInfo, topic, on_msg, qos, callback_group=self.cb)
        try:
            if not ev.wait(timeout_s):
                return None, f"{topic}: no CameraInfo within {timeout_s:.0f} s"
        finally:
            self.node.destroy_subscription(sub)
        m = got["m"]
        return {"K": [float(v) for v in m.k], "D": [float(v) for v in m.d], "model": m.distortion_model,
                "width": int(m.width), "height": int(m.height), "frame_id": m.header.frame_id}, "ok"

    def tf_parent(self, frame: str) -> Optional[str]:
        """The TF parent of a frame (e.g. the camera driver's camera_link above its optical frame), or None."""
        if self._tf is None:
            return None
        try:
            d = yaml.safe_load(self._tf.all_frames_as_yaml()) or {}
        except Exception:
            return None
        p = (d.get(frame) or {}).get("parent")
        return p if p and p != "NO_PARENT" else None

    def topic_types(self) -> List[Tuple[str, str]]:
        """(topic, type) for every topic in the graph now (one row per type), for the recorder."""
        try:
            return sorted((name, t) for name, types in self.node.get_topic_names_and_types() for t in types)
        except Exception:  # noqa: BLE001 - a graph query during shutdown
            return []

    def image_topics(self) -> List[Tuple[str, str]]:
        return sorted((name, t) for name, types in self.node.get_topic_names_and_types() for t in types
                      if t in IMAGE_TYPES)

    def topic_publishers(self, topic: str) -> int:
        return self.node.count_publishers(topic)

    def _subscribe_image(self, topic: str, w: ImageWatch):
        """(subscription feeding w, '') or (None, why)."""
        types = dict(self.node.get_topic_names_and_types()).get(topic, [])
        t = next((x for x in types if x in IMAGE_TYPES), None)
        if t is None:
            return None, f"{topic}: no image publisher" if not types else f"{topic} is {', '.join(types)}"
        if t == IMAGE_TYPE:
            from sensor_msgs.msg import Image as Msg
            cb = lambda m: w.on_image(m.encoding, m.width, m.height, m.step, m.data, bool(m.is_bigendian),  # noqa
                                      m.header.frame_id)
        else:
            from sensor_msgs.msg import CompressedImage as Msg
            cb = lambda m: w.on_compressed(m.format, m.data, m.header.frame_id)  # noqa
        return self.node.create_subscription(Msg, topic, cb, qos_profile_sensor_data, callback_group=self.cb), ""

    def watch_image(self, topic: Optional[str]) -> Tuple[bool, str]:
        """Preview one image topic (or none). Only one subscription exists at a time (the desktop GUI)."""
        if self._image_sub is not None:
            self.node.destroy_subscription(self._image_sub)
            self._image_sub, self._image_watch = None, None
        if not topic:
            return True, "preview off"
        w = ImageWatch(topic)
        sub, why = self._subscribe_image(topic, w)
        if sub is None:
            return False, why
        self._image_sub, self._image_watch = sub, w
        return True, f"watching {topic}"

    _streams_lock = threading.Lock()

    def open_image(self, topic: str):
        """(ImageWatch, msg) for one camera window in the browser: one subscription per topic, shared by everyone
        watching it, so several cameras show at once. close_image(topic) when the window goes. (None, why)."""
        with self._streams_lock:
            st = self.__dict__.setdefault("_streams", {})          # topic -> [subscription, watch, viewers]
            if topic in st:
                st[topic][2] += 1
                return st[topic][1], f"watching {topic}"
            w = ImageWatch(topic)
            sub, why = self._subscribe_image(topic, w)
            if sub is None:
                return None, why
            st[topic] = [sub, w, 1]
            return w, f"watching {topic}"

    def close_image(self, topic: str):
        with self._streams_lock:
            e = self.__dict__.get("_streams", {}).get(topic)
            if e is None:
                return
            e[2] -= 1
            if e[2] <= 0:                                           # nobody looks: no image traffic
                self.node.destroy_subscription(e[0])
                del self._streams[topic]

    def open_images(self) -> dict:
        """{topic: ImageWatch} of what the browser watches now."""
        with self._streams_lock:
            return {t: e[1] for t, e in self.__dict__.get("_streams", {}).items()}

    def watched_image(self) -> Optional[ImageWatch]:
        return self._image_watch


class RealCameras(CameraTopics):
    """Real cameras next to a fake robot (moveit_gui_web --fake --real-cameras): its own node that only reads
    topics and TF, so camera drivers can be tried on a PC without a robot. It publishes nothing."""

    def __init__(self, node_name: str = "moveit_gui_cameras"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.cb = ReentrantCallbackGroup()
        self._image_sub = None
        self._image_watch: Optional[ImageWatch] = None
        self._tf = None
        if Buffer is not None:
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self.node, spin_thread=False)
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        threading.Thread(target=self._executor.spin, daemon=True, name="ros-cameras").start()

    def shutdown(self):
        self._executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class RosBridge(CameraTopics):
    STALE_S = 1.0
    SPIN_STALE_S = 2.0              # executor heartbeat older than this: the ROS thread is stuck
    CONTROLLERS_STALE_S = 3.0       # polled every 0.5 s by _health_tick
    stop_epoch = 0                  # see __init__ (per instance once a STOP bumps it)

    def __init__(self, profile: RobotProfile, node_name: str = "moveit_gui"):
        self.profile = profile
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._logs: deque = deque(maxlen=1000)
        self._snap = Snapshot(connected={k: False for k in ("ROS", "UR", "MoveIt", "Gripper", "F/T")})
        self._snap.connected["ROS"] = True
        self._t_joints = self._t_wrench = self._t_grip_msg = 0.0
        self._t_controllers = 0.0                       # last controller list (poll or call); 0 = never
        self._t_spin = time.monotonic()                 # executor heartbeat
        self._shutting_down = False
        self.wrench_history = WrenchHistory()           # 100 Hz F/T for the web chart
        self.on_wrench = None                           # every F/T sample, for the F/T limit (core: ft_guard.feed)
        self._joints_received = False
        self._velocities_ok = False
        self._freedrive_on = False
        self._pose_subs: Dict[str, object] = {}
        self._pose_latest: Dict[str, Tuple[float, PoseStamped]] = {}
        self._image_sub = None
        self._image_watch: Optional[ImageWatch] = None

        t, s = profile.topics, profile.services
        n = self.node
        # raw: most messages are dropped unread (_on_joints_raw), which saves decoding them in Python
        n.create_subscription(JointState, t.get("joint_states", "/joint_states"), self._on_joints_raw,
                              qos_profile_sensor_data, callback_group=self.cb, raw=True)
        if t.get("wrench"):
            n.create_subscription(WrenchStamped, t["wrench"], self._on_wrench,
                                  qos_profile_sensor_data, callback_group=self.cb)
        if t.get("speed_scaling"):
            n.create_subscription(Float64, t["speed_scaling"], self._on_speed, 10, callback_group=self.cb)
        if t.get("program_running"):
            n.create_subscription(Bool, t["program_running"], self._on_prog, LATCHED, callback_group=self.cb)
        if HAVE_UR_MSGS:
            if t.get("robot_mode"):
                n.create_subscription(RobotMode, t["robot_mode"], self._on_robot_mode, LATCHED,
                                      callback_group=self.cb)
            if t.get("safety_mode"):
                n.create_subscription(SafetyMode, t["safety_mode"], self._on_safety_mode, LATCHED,
                                      callback_group=self.cb)

        # --- services
        cm = s.get("controller_manager", "/controller_manager")
        self._list_cli = n.create_client(ListControllers, f"{cm}/list_controllers", callback_group=self.cb)
        gcm = profile.gripper.get("controller_manager")
        self._gripper_list_cli = (n.create_client(ListControllers, f"{gcm}/list_controllers", callback_group=self.cb)
                                  if gcm else None)
        self._switch_cli = n.create_client(SwitchController, f"{cm}/switch_controller", callback_group=self.cb)
        # the gripper's own controller manager, for Reconnect gripper (created once: a fresh client needs discovery time)
        self._gripper_cm_cli = {srv: n.create_client(t, f"{gcm}/{srv}", callback_group=self.cb) for t, srv in (
            (ListHardwareComponents, "list_hardware_components"),
            (SetHardwareComponentState, "set_hardware_component_state"), (ListControllers, "list_controllers"),
            (LoadController, "load_controller"), (ConfigureController, "configure_controller"),
            (SwitchController, "switch_controller"))} if gcm else {}
        self._hw_list_cli = n.create_client(ListHardwareComponents, f"{cm}/list_hardware_components",
                                            callback_group=self.cb)
        self._hw_set_cli = n.create_client(SetHardwareComponentState, f"{cm}/set_hardware_component_state",
                                           callback_group=self.cb)
        dash = s.get("dashboard_ns", "/dashboard_client")
        self._remote_t = 0.0                     # last is_in_remote_control query (health tick)
        self._dash = {c: n.create_client(Trigger, f"{dash}/{c}", callback_group=self.cb)
                      for c in DASHBOARD_COMMANDS + ("connect", "quit")}
        io = s.get("io_and_status_ns", "/io_and_status_controller")
        self._resend_cli = n.create_client(Trigger, f"{io}/resend_robot_program", callback_group=self.cb)
        self._zero_ft_cli = n.create_client(Trigger, f"{io}/zero_ftsensor", callback_group=self.cb)
        if HAVE_UR_MSGS:
            self._load_cli = n.create_client(Load, f"{dash}/load_program", callback_group=self.cb)
            self._loaded_cli = n.create_client(GetLoadedProgram, f"{dash}/get_loaded_program", callback_group=self.cb)
            self._remote_cli = n.create_client(IsInRemoteControl, f"{dash}/is_in_remote_control",
                                               callback_group=self.cb)
            # PLAYING / PAUSED / STOPPED: a paused program is not a stopped one (Resume, not a restart)
            self._prog_cli = n.create_client(GetProgramState, f"{dash}/program_state", callback_group=self.cb)
            self._io_cli = n.create_client(SetIO, f"{io}/set_io", callback_group=self.cb)
            self._speed_cli = n.create_client(SetSpeedSliderFraction, f"{io}/set_speed_slider",
                                              callback_group=self.cb)

        # --- gripper
        g = profile.gripper
        self._gripper_cli = None
        self._gripper_kind = g.get("action_type", "gripper_command")
        if g.get("enabled", False):
            act = ParallelGripperCommand if self._gripper_kind == "parallel_gripper" else GripperCommand
            if act is not None:
                self._gripper_cli = ActionClient(n, act, g["action"], callback_group=self.cb)
            else:
                self.log(f"gripper action type '{self._gripper_kind}' not available in this ROS install")
        self._gripper_reactivate_cli = (n.create_client(Trigger, g["reactivate_service"], callback_group=self.cb)
                                        if g.get("reactivate_service") else None)
        if g.get("enabled") and g.get("status_topic") and GripperStatus is not None:
            n.create_subscription(GripperStatus, g["status_topic"], self._on_gripper_status, LATCHED,
                                  callback_group=self.cb)

        # --- robot model for the 3D view (URDF, latched by robot_state_publisher)
        self._robot_description: Optional[str] = None
        n.create_subscription(String, t.get("robot_description", "/robot_description"), self._on_robot_description,
                              LATCHED, callback_group=self.cb)
        # --- Cartesian force targets (OPERATE > Force), one publisher per controller topic
        self._wrench_pubs: Dict[str, object] = {}
        for m in profile.modes.values():
            if m.wrench.get("topic"):
                self._wrench_pubs[m.wrench["topic"]] = n.create_publisher(WrenchStamped, m.wrench["topic"], 10)

        # --- freedrive keepalive
        fd = profile.freedrive
        self._fd_pub = n.create_publisher(Bool, fd["enable_topic"], 10) if fd.get("enable_topic") else None
        if self._fd_pub:
            n.create_timer(1.0 / float(fd.get("keepalive_hz", 10.0)), self._freedrive_tick, callback_group=self.cb)

        # --- TF (TCPs, fixtures' parents, vision frames). Its own node and thread: /tf comes at ~500 Hz, and on the
        # shared executor it queued behind every other callback. On a loaded Jetson the buffer then fell more than
        # tf_timeout_s behind and the TCP pose went missing.
        self._tf = self._tf_node = None
        if Buffer is not None:
            self._tf = Buffer()
            self._tf_node = Node(node_name + "_tf")
            self._tf_listener = TransformListener(self._tf, self._tf_node, spin_thread=True)

        n.create_timer(0.5, self._health_tick, callback_group=self.cb)

        # --- MoveIt (pymoveit2, upstream). Executes through move_group, so MoveIt's
        # controller manager must route to the controller that is active in each mode.
        self.moveit = None
        self._follow_clients: Dict = {}
        self._execute_handle = None
        self._param_lock = threading.Lock()
        self._param_clients: Dict = {}            # parameter service -> client (see _param_call)
        # Chained admittance: the trajectory controller's REFERENCE joints (controller_state), so a move starts there
        # and not at the pushed, measured joints (see control_modes.shift_to_reference). {controller: (t, {joint: q})}
        self._ctrl_ref: Dict[str, tuple] = {}
        try:
            from control_msgs.msg import JointTrajectoryControllerState
        except ImportError:  # pragma: no cover
            JointTrajectoryControllerState = None
        for m in profile.modes.values():
            rs = m.reference_start
            if rs and m.moveit_controller and JointTrajectoryControllerState is not None:
                topic = rs.get("state_topic") or f"/{m.moveit_controller}/controller_state"

                def on_state(msg, c=m.moveit_controller):
                    vel = list(msg.reference.velocities) or [0.0] * len(msg.joint_names)
                    self._ctrl_ref[c] = (time.monotonic(), dict(zip(msg.joint_names, msg.reference.positions)),
                                         dict(zip(msg.joint_names, vel)))
                n.create_subscription(JointTrajectoryControllerState, topic, on_state, 10, callback_group=self.cb)
                if FollowJointTrajectory is not None:          # ready for a hold from cancel_motion (any thread)
                    self._follow_clients[m.moveit_controller] = ActionClient(
                        n, FollowJointTrajectory, f"/{m.moveit_controller}/follow_joint_trajectory",
                        callback_group=self.cb)
        # Bumped by every cancel_motion. A move remembers it when it starts (before planning) and refuses to send,
        # or cancels, its ExecuteTrajectory goal if a STOP came in meanwhile: pymoveit2 cannot cancel a plan() in
        # progress and cancel_motion alone only reaches a goal that is already executing.
        self.stop_epoch = 0
        self._execute_cli = None
        self._servos: Dict[str, dict] = {}
        try:
            from moveit_msgs.msg import ServoStatus
            from moveit_msgs.srv import ServoCommandType
            routes = profile.servo.get("routes") or {}
            for name, cfg in routes.items():
                if not isinstance(cfg, dict):
                    continue
                ep = {
                    "cfg": cfg,
                    "cli": (n.create_client(ServoCommandType, cfg["command_type_service"], callback_group=self.cb)
                            if cfg.get("command_type_service") else None),
                    "status": (0, "", 0.0),
                    "twist_pub": (n.create_publisher(TwistStamped, cfg["twist_topic"], 10)
                                  if cfg.get("twist_topic") else None),
                    "joint_pub": (n.create_publisher(JointJog, cfg["joint_topic"], 10)
                                  if cfg.get("joint_topic") and JointJog is not None else None),
                }
                self._servos[name] = ep
                ep["out_n"] = 0
                out_type = {"float64_array": Float64MultiArray, "joint_trajectory": JointTrajectory}.get(
                    cfg.get("output_type", ""))
                if cfg.get("output_topic") and out_type is not None:
                    # Diagnostics only: how many commands this Servo node actually sent to its controller.
                    n.create_subscription(out_type, cfg["output_topic"],
                                          lambda _m, ep=ep: ep.__setitem__("out_n", ep["out_n"] + 1),
                                          10, callback_group=self.cb)
                if cfg.get("status_topic"):
                    n.create_subscription(
                        ServoStatus, cfg["status_topic"],
                        lambda msg, route=name: self._on_servo_status(route, msg),
                        10, callback_group=self.cb)
        except ImportError:  # pragma: no cover
            pass
        if ExecuteTrajectory is not None:
            self._execute_cli = ActionClient(n, ExecuteTrajectory, "/execute_trajectory", callback_group=self.cb)
        # STOP must not rely on cancelling /execute_trajectory alone: MoveIt 2.12 accepts that cancel but its cancel
        # callback never calls stopExecution (checked in the image: preemptExecuteTrajectoryCallback has no caller),
        # so the trajectory keeps running in the controller. With the External Control program stopped it even
        # waits there at speed scaling 0 and moves on when Play is pressed. So STOP also
        #   * tells move_group to stop ("stop" on trajectory_execution_event, its own stop path), and
        #   * cancels every goal on each trajectory controller MoveIt can use (empty CancelGoal = all goals);
        #     the controller then holds its current position.
        self._exec_event_pub = n.create_publisher(String, "/trajectory_execution_event", 10)
        self._fjt_cancel = {c: n.create_client(CancelGoal, f"/{c}/follow_joint_trajectory/_action/cancel_goal",
                                               callback_group=self.cb)
                            for c in sorted({m.moveit_controller for m in profile.modes.values()
                                             if m.moveit_controller})}
        try:
            from pymoveit2 import MoveIt2
            m = profile.moveit
            self.moveit = MoveIt2(node=n, joint_names=profile.joint_names,
                                  base_link_name=m.get("base_link", "base_link"),
                                  end_effector_name=profile.planning_link,
                                  group_name=m.get("group", "ur_manipulator"),
                                  callback_group=self.cb,
                                  use_move_group_action=bool(m.get("use_move_group_action", True)))
            self.set_scaling(float(m.get("default_velocity_scaling", 0.1)),
                             float(m.get("default_acceleration_scaling", 0.1)))
        except Exception as e:  # pragma: no cover
            self.log(f"MoveIt interface unavailable: {e}")

        # 4, not 6: TF has its own thread now, and more Python threads mostly wait on each other for the GIL
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(n)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True, name="ros-executor")
        self._spin_thread.start()
        self.log(f"ROS bridge up for profile '{profile.name}'")

    def _spin_loop(self):
        """Spin the executor until shutdown. One failing callback must not end every update silently: log it and
        keep spinning. self._t_spin is the heartbeat the snapshot reports (connected["ROS"])."""
        while not self._shutting_down:
            try:
                self._executor.spin_once(timeout_sec=0.1)
                self._t_spin = time.monotonic()
            except Exception as e:
                if self._shutting_down or not rclpy.ok():
                    return
                try:
                    self.log(f"ROS executor error (spinning on): {type(e).__name__}: {e}")
                except Exception:
                    pass
                time.sleep(0.1)

    # =============================================================== helpers
    def log(self, msg: str):
        self._logs.append(f"{time.strftime('%H:%M:%S')}  {msg}")
        self.node.get_logger().info(msg)

    def drain_logs(self) -> List[str]:
        out = []
        while self._logs:
            out.append(self._logs.popleft())
        return out

    def snapshot(self) -> Snapshot:
        age = self.joint_state_age()          # takes the lock itself: must NOT be called while holding it
        now = time.monotonic()
        t_wr, t_ct, t_sp = (getattr(self, n, 0.0) for n in ("_t_wrench", "_t_controllers", "_t_spin"))
        spin_age = now - t_sp if t_sp else None
        ctrl_age = now - t_ct if t_ct else None
        with self._lock:
            s = self._snap
            connected = dict(s.connected)
            connected["ROS"] = spin_age is not None and spin_age < self.SPIN_STALE_S
            # A controller list nobody refreshed says nothing about the controllers now: report none, so the
            # mode reads "unknown". robot_mode / safety_mode / program_running are published on change only (the
            # driver's latched topics) and cannot be aged here; core.robot_refusal uses joint_state_age for them.
            stale_ctrl = ctrl_age is not None and ctrl_age > self.CONTROLLERS_STALE_S
            return Snapshot(connected=connected, wrench_age=now - t_wr if t_wr else None,
                            controllers_age=ctrl_age, spin_age=spin_age, joints=s.joints and list(s.joints),
                            joint_velocities=s.joint_velocities and list(s.joint_velocities),
                            joint_state_age=age, robot_mode=s.robot_mode, safety_mode=s.safety_mode,
                            program_running=s.program_running, speed_scaling=s.speed_scaling,
                            program_state=s.program_state, remote_control=s.remote_control,
                            controllers={} if stale_ctrl else dict(s.controllers),
                            claimed_interfaces={} if stale_ctrl else
                            {k: list(v) for k, v in s.claimed_interfaces.items()},
                            gripper_controllers=dict(s.gripper_controllers),
                            gripper_joint=s.gripper_joint and list(s.gripper_joint),
                            wrench=s.wrench and list(s.wrench),
                            gripper_position=s.gripper_position, freedrive=self._freedrive_on,
                            gripper_status=s.gripper_status and dict(s.gripper_status))

    @staticmethod
    def _wait_future(future, timeout_s: float):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout_s):
            return None
        return future.result()

    def _call(self, client, request, timeout_s: float = 3.0):
        if not client.wait_for_service(timeout_sec=0.5):
            return None
        return self._wait_future(client.call_async(request), timeout_s)

    # ========================================================= subscriptions
    # The arm's joint states come at 500 Hz and the gripper's at its controller rate, both on /joint_states. The GUI
    # needs about 100 Hz (view, status, standstill); every message in Python cost CPU the Jetson did not have.
    JOINTS_MIN_DT = 0.01
    GRIPPER_MIN_DT = 0.02

    def _on_joints_raw(self, data: bytes):
        now = time.monotonic()
        if now - self._t_joints < self.JOINTS_MIN_DT and now - self._t_grip_msg < self.GRIPPER_MIN_DT:
            return
        self._on_joints(deserialize_message(data, JointState))

    def _on_joints(self, msg: JointState):
        # The gripper has its own controller manager: its joint usually arrives in a separate message on the
        # same topic, so read it before deciding whether this message carries the arm.
        gname = self.profile.gripper.get("joint_name")
        if gname and gname in msg.name:
            k = list(msg.name).index(gname)
            gpos = float(msg.position[k])
            gjoint = [gpos, float(msg.velocity[k]) if len(msg.velocity) > k else float("nan"),
                      float(msg.effort[k]) if len(msg.effort) > k else float("nan")]
            with self._lock:
                self._snap.gripper_position = gpos
                self._snap.gripper_joint = gjoint
                self._t_grip_msg = time.monotonic()
        q = reorder_joints(msg.name, msg.position, self.profile.joint_names)
        if q is None:
            return            # e.g. the gripper-only JointState message
        # Velocities only count if every arm joint has a finite value (else: cannot verify standstill)
        qd = reorder_joints(msg.name, msg.velocity, self.profile.joint_names) if msg.velocity else None
        with self._lock:
            self._snap.joints = q
            self._snap.joint_velocities = qd
            self._velocities_ok = qd is not None
            self._t_joints = time.monotonic()
            self._joints_received = True

    def _on_robot_description(self, msg):
        with self._lock:
            self._robot_description = msg.data

    def robot_description(self) -> Optional[str]:
        with self._lock:
            return self._robot_description

    def publish_wrench(self, topic: str, frame: str, wrench: List[float]):
        pub = self._wrench_pubs.get(topic)
        if pub is None:
            raise RuntimeError(f"no wrench publisher for {topic} (profile control_modes.*.wrench)")
        msg = WrenchStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame
        f, t = msg.wrench.force, msg.wrench.torque
        f.x, f.y, f.z, t.x, t.y, t.z = (float(v) for v in wrench)
        pub.publish(msg)

    def _on_gripper_status(self, msg):
        with self._lock:
            self._snap.gripper_status = gripper_status_dict(msg)

    def _on_wrench(self, msg: WrenchStamped):
        f, t = msg.wrench.force, msg.wrench.torque
        with self._lock:
            self._snap.wrench = w = [f.x, f.y, f.z, t.x, t.y, t.z]
        self._t_wrench = time.monotonic()
        self.wrench_history.add(time.time(), w)
        cb = self.on_wrench
        if cb is not None:
            try:
                cb(w)
            except Exception as e:  # pragma: no cover - never break the F/T subscription
                self.log(f"F/T limit check failed: {e}")

    def _on_speed(self, msg: Float64):
        with self._lock:
            # ur_controllers' SpeedScalingStateBroadcaster publishes PERCENT (100 = full speed); the snapshot
            # holds a fraction, like the fake and the GUI speed override (100 % showed as "10000%")
            self._snap.speed_scaling = float(msg.data) / 100.0

    def _on_prog(self, msg: Bool):
        with self._lock:
            self._ext_control = bool(msg.data)           # the driver's own word: External Control is connected
            self._snap.program_running = bool(msg.data)
            if msg.data:
                self._snap.program_state = "PLAYING"
            elif self._snap.program_state == "PLAYING":
                self._snap.program_state = None          # paused or stopped: the Dashboard poll says which

    def _on_robot_mode(self, msg):
        with self._lock:
            self._snap.robot_mode = _enum_name(RobotMode, msg.mode)
            if self._snap.robot_mode not in ("RUNNING", "?"):
                # No program runs on an arm that is off or braked. Seen on the robot: after a power cycle the
                # driver did not notice the program stop, so robot_program_running stayed true (it is only
                # published on change). The Dashboard poll below sets it true again once a program runs.
                self._snap.program_running = False

    def _on_safety_mode(self, msg):
        with self._lock:
            self._snap.safety_mode = _enum_name(SafetyMode, msg.mode)

    # ================================================================ timers
    def _health_tick(self):
        now = time.monotonic()
        with self._lock:
            c = self._snap.connected
            c["UR"] = now - self._t_joints < self.STALE_S
            c["F/T"] = now - self._t_wrench < self.STALE_S
            # The graph lists actions that only have a CLIENT too (ours), so ask whether a server answers.
            c["MoveIt"] = bool(self._execute_cli is not None and self._execute_cli.server_is_ready())
            c["Gripper"] = bool(self._gripper_cli and self._gripper_cli.server_is_ready())
            servos = getattr(self, "_servos", {})
            c["Servo"] = bool(servos) and all(self.servo_ready(route) for route in servos)
        if self._list_cli.service_is_ready():
            fut = self._list_cli.call_async(ListControllers.Request())
            fut.add_done_callback(self._on_controller_list)
        # Local / Remote Control from the Dashboard (a read-only query), every few seconds
        # and whether a program runs, as PolyScope sees it: the driver's topic can miss a stop (see _on_robot_mode)
        cli = getattr(self, "_remote_cli", None)
        if cli is not None and now - self._remote_t > 3.0:
            self._remote_t = now
            if cli.service_is_ready():
                cli.call_async(IsInRemoteControl.Request()).add_done_callback(self._on_remote_control)
            else:
                with self._lock:
                    self._snap.remote_control = None
            pcli = getattr(self, "_prog_cli", None)
            if pcli is not None and pcli.service_is_ready():
                pcli.call_async(GetProgramState.Request()).add_done_callback(self._on_dashboard_program)
        if self._gripper_list_cli is not None and self._gripper_list_cli.service_is_ready():
            fut = self._gripper_list_cli.call_async(ListControllers.Request())
            fut.add_done_callback(self._on_gripper_controller_list)

    def _on_remote_control(self, fut):
        try:
            res = fut.result()
        except Exception:
            res = None
        with self._lock:
            self._snap.remote_control = bool(res.remote_control) if res is not None and res.success else None
        if res is not None and not res.success and dashboard_lost(res.answer):
            self._reconnect_dashboard_bg()

    def _reconnect_dashboard_bg(self):
        """The driver's dashboard_client never reconnects by itself: reopen it, at most every 10 s, off the
        executor thread (connect may take seconds). Only the TCP connection to port 29999: no robot command."""
        now = time.monotonic()
        if getattr(self, "_dash_reconnecting", False) or now - getattr(self, "_dash_reconnect_t", -1e9) < 10.0:
            return
        self._dash_reconnecting, self._dash_reconnect_t = True, now

        def run():
            try:
                self.log("dashboard: the connection to the Dashboard Server was lost; reconnecting")
                self._dash_call("quit")
                self._dash_call("connect")
            finally:
                self._dash_reconnecting = False
        threading.Thread(target=run, daemon=True).start()

    def _on_dashboard_program(self, fut):
        try:
            res = fut.result()
        except Exception:
            return
        if res is not None and res.success:
            st = (res.state.state or "").strip().upper()
            with self._lock:
                self._snap.program_state = st or None
                # PLAYING says only that some program plays. The arm takes motion only when it is External
                # Control connected to this driver (it may point at another PC): the driver's topic says that.
                # Without a topic in the profile, trust the Dashboard.
                ext = getattr(self, "_ext_control", None) if self.profile.topics.get("program_running") else True
                self._snap.program_running = st == "PLAYING" and ext is True
        elif res is not None and dashboard_lost(res.answer):
            with self._lock:
                self._snap.program_state = None
            self._reconnect_dashboard_bg()

    def _on_controller_list(self, fut):
        try:
            res = fut.result()
        except Exception:
            return
        if res is not None:
            with self._lock:
                self._snap.controllers = {c.name: c.state for c in res.controller}
                self._snap.claimed_interfaces = {c.name: list(c.claimed_interfaces) for c in res.controller}
            self._t_controllers = time.monotonic()

    def _on_gripper_controller_list(self, fut):
        try:
            res = fut.result()
        except Exception:
            return
        if res is not None:
            with self._lock:
                self._snap.gripper_controllers = {c.name: c.state for c in res.controller}

    def _freedrive_tick(self):
        if self._freedrive_on:
            self._fd_pub.publish(Bool(data=True))

    # ===================================================== controller_manager
    def list_controllers(self) -> Optional[Dict[str, str]]:
        res = self._call(self._list_cli, ListControllers.Request())
        if res is None:
            return None
        states = {c.name: c.state for c in res.controller}
        with self._lock:
            self._snap.controllers = dict(states)
            self._snap.claimed_interfaces = {c.name: list(c.claimed_interfaces) for c in res.controller}
        self._t_controllers = time.monotonic()
        return states

    def controller_claims(self) -> Optional[Dict[str, List[str]]]:
        """Fresh claimed interfaces of the arm controllers (ControllerState.claimed_interfaces)."""
        res = self._call(self._list_cli, ListControllers.Request())
        if res is None:
            return None
        return {c.name: list(c.claimed_interfaces) for c in res.controller}

    def switch_controllers(self, activate, deactivate, strict=True, timeout_s=5.0) -> bool:
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT if strict else SwitchController.Request.BEST_EFFORT
        req.activate_asap = False
        req.timeout = DurationMsg(sec=int(timeout_s), nanosec=int((timeout_s % 1) * 1e9))
        res = self._call(self._switch_cli, req, timeout_s + 2.0)
        if res is None:
            self.log("switch_controller: no response")
            return False
        if not res.ok:
            self.log(f"switch_controller rejected: {getattr(res, 'message', '')}")
        return bool(res.ok)

    def hardware_states(self) -> Optional[Dict[str, str]]:
        """ros2_control hardware components of the arm's controller_manager: name -> lifecycle label."""
        res = self._call(self._hw_list_cli, ListHardwareComponents.Request())
        if res is None:
            return None
        return {c.name: c.state.label for c in res.component}

    def activate_hardware(self, name: str) -> Tuple[bool, str]:
        req = SetHardwareComponentState.Request()
        req.name = name
        req.target_state = LifecycleState(id=LifecycleState.PRIMARY_STATE_ACTIVE, label="active")
        res = self._call(self._hw_set_cli, req, 5.0)
        if res is None:
            return False, "set_hardware_component_state: no response"
        self.log(f"hardware {name} -> {res.state.label} ({'ok' if res.ok else 'rejected'})")
        return bool(res.ok), res.state.label

    # ================================================================ motion
    def set_scaling(self, vel: float, acc: float):
        if self.moveit is not None:
            self.moveit.max_velocity = float(vel)
            self.moveit.max_acceleration = float(acc)

    def cancel_motion(self):
        self.stop_epoch += 1
        self._exec_event_pub.publish(String(data="stop"))
        # Chained admittance: a cancelled goal makes the trajectory controller hold at the MEASURED joints, which a
        # force has pushed off the reference: the reference jumps there and the push doubles (every pause added
        # it again). Replace the goal with a hold at its own reference instead: the arm stops, the push stays.
        held = {c for c, cfg in self._reference_controllers() if self._hold_at_reference(c, cfg)}
        for name, cli in self._fjt_cancel.items():
            if name in held:
                continue
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())      # zero goal id and stamp: cancel all of its goals
        handle = self._execute_handle
        if handle is not None and not held:
            handle.cancel_goal_async()                    # (a hold preempts it; a cancel could beat the hold)
        if self.moveit is not None:
            try:
                self.moveit.cancel_execution()
            except Exception as e:  # pragma: no cover
                self.log(f"cancel failed: {e}")

    def wrench_samples(self, since: float) -> List[list]:
        return self.wrench_history.since(since)

    def joint_state_age(self) -> Optional[float]:
        with self._lock:
            return (time.monotonic() - self._t_joints) if self._joints_received else None

    def joint_velocities_available(self) -> bool:
        with self._lock:
            return self._velocities_ok

    def is_stationary(self) -> Optional[bool]:
        """True/False, or None when it cannot be verified (stale state or no velocities)."""
        age = self.joint_state_age()
        with self._lock:
            qd = self._snap.joint_velocities
        if age is None or age > self.profile.joint_state_timeout_s or qd is None:
            return None
        return all(abs(v) < self.profile.stationary_joint_velocity for v in qd)

    # ================================================================== TF
    def lookup_transform(self, parent: str, child: str) -> Optional[np.ndarray]:
        """T_parent_child from TF, or None if unavailable or stale.

        Static chains come back with stamp 0 and are always fresh; dynamic ones
        must be newer than profile.tf_timeout_s.
        """
        if self._tf is None:
            return None
        try:
            tr = self._tf.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        st = tr.header.stamp
        if st.sec != 0 or st.nanosec != 0:
            age = (self.node.get_clock().now() - Time.from_msg(st)).nanoseconds * 1e-9
            if age > self.profile.tf_timeout_s:
                return None
        p, q = tr.transform.translation, tr.transform.rotation
        try:
            return tf.make([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        except tf.TransformError:
            return None

    def _on_pose(self, topic: str, msg: PoseStamped):
        self._pose_latest[topic] = (time.monotonic(), msg)

    def read_pose_source(self, source_type: str, source: str, reference_frame: str) -> Optional[np.ndarray]:
        """Latest detection as T_R_obj in `reference_frame`, or None if missing / stale."""
        if source_type == "tf":
            return self.lookup_transform(reference_frame, source)
        if source_type != "pose_topic":
            return None
        if source not in self._pose_subs:
            self._pose_subs[source] = self.node.create_subscription(
                PoseStamped, source, lambda m, t=source: self._on_pose(t, m), 10, callback_group=self.cb)
            return None
        latest = self._pose_latest.get(source)
        if latest is None or time.monotonic() - latest[0] > self.profile.tf_timeout_s:
            return None
        msg = latest[1]
        pp, pq = msg.pose.position, msg.pose.orientation
        try:
            T_hdr_obj = tf.make([pp.x, pp.y, pp.z], [pq.x, pq.y, pq.z, pq.w])
        except tf.TransformError:
            return None
        hdr = msg.header.frame_id or reference_frame
        T_ref_hdr = np.eye(4) if hdr == reference_frame else self.lookup_transform(reference_frame, hdr)
        return None if T_ref_hdr is None else T_ref_hdr @ T_hdr_obj

    def _moveit_wait(self) -> bool:
        timeout = float(self.profile.moveit.get("planning_timeout_s", 60.0))
        ok = self.moveit.wait_until_executed(timeout_sec=timeout, cancel_on_timeout=True)
        if ok is None:
            # Unknown outcome is not success: the next step must not run on an unverified motion.
            self.log("MoveIt returned no execution result; treating the motion as failed")
            return False
        return bool(ok)

    def _execute_on_active_controller(self, trajectory, epoch: Optional[int] = None) -> bool:
        """Execute a planned trajectory through the controller of the active mode.

        Plain MoveGroup execution would use MoveIt's default controller regardless of the mode, so the
        controller is named in the ExecuteTrajectory goal (moveit_msgs: controller_names). It is read
        fresh from the controller manager, so there is no routing state to get out of step.
        """
        epoch = self.stop_epoch if epoch is None else epoch
        controller = moveit_controller_for(self.profile, self.list_controllers())
        if controller is None:
            self.log("motion refused: no active mode accepts MoveIt trajectories")
            return False
        if self._execute_cli is None or not self._execute_cli.wait_for_server(timeout_sec=1.0):
            self.log("execute_trajectory action server is not available")
            return False
        mode = next((m for m in self.profile.modes.values()
                     if m.moveit_controller == controller and m.reference_start), None)
        if mode is not None:
            return self._execute_from_reference(controller, mode.reference_start, trajectory, epoch)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = trajectory
        goal.controller_names = [controller]
        if self.stop_epoch != epoch:
            self.log("motion not started: STOP was pressed while it was being planned")
            return False
        self.log(f"executing through {controller}")
        handle = self._wait_future(self._execute_cli.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            self.log(f"{controller}: trajectory goal was rejected")
            return False
        self._execute_handle = handle
        if self.stop_epoch != epoch:
            # STOP came in while the goal was being sent: cancel it, then wait below until MoveIt reports it ended.
            self.log("STOP was pressed while the motion was being sent: cancelling it")
            handle.cancel_goal_async()
        timeout = self._execution_timeout(trajectory)
        try:
            result = self._wait_future(handle.get_result_async(), timeout)
        finally:
            self._execute_handle = None
        if result is None:
            # Unknown outcome is not success: the next step must not run on an unverified motion.
            handle.cancel_goal_async()
            self.log("MoveIt returned no execution result; treating the motion as failed")
            return False
        ok = result.result.error_code.val == 1          # MoveItErrorCodes.SUCCESS
        if not ok:
            self.log(f"{controller}: execution failed (MoveIt error {result.result.error_code.val})")
        return ok

    def _reference_controllers(self):
        return [(m.moveit_controller, m.reference_start) for m in self.profile.modes.values()
                if m.reference_start and m.moveit_controller]

    def _fresh_reference(self, controller: str, cfg: dict):
        """(positions, velocities, age_s) of the controller's reference, or None when not fresh (not active)."""
        got = self._ctrl_ref.get(controller)
        if got is None:
            return None
        age = time.monotonic() - got[0]
        if age > float(cfg.get("max_age_s", 0.5)):
            return None
        return got[1], (got[2] if len(got) > 2 else {}), age

    def _hold_at_reference(self, controller: str, cfg: dict) -> bool:
        """Preempt whatever the controller runs with one point at its reference now (extrapolated over the age of
        the reading). False when there is no fresh reference (controller not active) or no action server."""
        got = self._fresh_reference(controller, cfg)
        client = self._follow_clients.get(controller)
        if got is None or client is None or FollowJointTrajectory is None or not client.server_is_ready():
            return False
        pos, vel, age = got
        names = [j for j in self.profile.joint_names if j in pos]
        if len(names) != len(self.profile.joint_names):
            return False
        from trajectory_msgs.msg import JointTrajectoryPoint
        p = JointTrajectoryPoint()
        a = min(age, 0.05)
        p.positions = [float(pos[j]) + float(vel.get(j, 0.0)) * a for j in names]
        p.velocities = [0.0] * len(names)
        p.time_from_start = DurationMsg(sec=0, nanosec=int(float(cfg.get("hold_s", 0.1)) * 1e9))
        traj = JointTrajectory()
        traj.joint_names = names
        traj.points = [p]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        client.send_goal_async(goal)
        self.log(f"{controller}: holding at its reference (not at the pushed, measured joints)")
        return True

    def _execute_from_reference(self, controller: str, cfg: dict, trajectory, epoch: int) -> bool:
        """Chained admittance: shift the plan so it starts at the trajectory controller's reference and send it
        to that controller directly. (MoveIt's execution would refuse a start away from the measured joints, and
        starting at the measured joints doubles the push of an applied force.) STOP cancels it like any goal
        (cancel_motion: the controller's cancel service and this handle)."""
        got = self._fresh_reference(controller, cfg)
        if got is None:
            self.log(f"motion refused: no fresh reference from {controller} (controller_state)")
            return False
        names = list(trajectory.joint_names)
        pts = [list(p.positions) for p in trajectory.points]
        try:
            shifted, d = shift_to_reference(pts, names, got[0], float(cfg.get("max_shift_rad", math.radians(5.0))))
        except ReferenceShiftError as e:
            self.log(f"motion refused: {e}")
            return False
        worst = max(abs(v) for v in d)
        if worst > 1e-4:
            self.log(f"starting at the {controller} reference ({math.degrees(worst):.2f} deg from the measured "
                     "joints: a force is pushing the arm)")
        for p, q in zip(trajectory.points, shifted):
            p.positions = [float(v) for v in q]
        traj = JointTrajectory()
        traj.joint_names = names                       # header stamp 0: start now
        traj.points = list(trajectory.points)
        client = self._follow_clients.get(controller)
        if client is None:
            client = ActionClient(self.node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory",
                                  callback_group=self.cb)
            self._follow_clients[controller] = client
        if not client.wait_for_server(timeout_sec=1.0):
            self.log(f"{controller}: follow_joint_trajectory is not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if self.stop_epoch != epoch:
            self.log("motion not started: STOP was pressed while it was being planned")
            return False
        self.log(f"executing through {controller} (from its reference)")
        handle = self._wait_future(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            self.log(f"{controller}: trajectory goal was rejected")
            return False
        self._execute_handle = handle
        if self.stop_epoch != epoch:
            self.log("STOP was pressed while the motion was being sent: cancelling it")
            handle.cancel_goal_async()
        try:
            result = self._wait_future(handle.get_result_async(), self._execution_timeout(traj))
        finally:
            self._execute_handle = None
        if result is None:
            handle.cancel_goal_async()
            self.log(f"{controller}: no execution result; treating the motion as failed")
            return False
        code = result.result.error_code               # FollowJointTrajectory: 0 = SUCCESSFUL
        if code != 0 or result.status != 4:            # GoalStatus.STATUS_SUCCEEDED
            self.log(f"{controller}: execution failed (error {code} {result.result.error_string!r}, status "
                     f"{result.status})")
            return False
        return True

    def _plan_timeout(self) -> float:
        """Give up on a plan when move_group does not answer (it died mid-plan): the GUI must not stay busy."""
        return max(10.0, 2.0 * float(self.profile.moveit.get("planning_timeout_s", 60.0)))

    def _execution_timeout(self, trajectory) -> float:
        """Time to wait for a trajectory: its own duration at the robot's current speed scaling, plus a margin.
        A slow move at low speed is not cancelled; a hung execution still ends."""
        try:
            tfs = trajectory.points[-1].time_from_start
            duration = float(tfs.sec) + float(tfs.nanosec) * 1e-9
        except Exception:
            return float(self.profile.moveit.get("planning_timeout_s", 60.0))
        scaling = self._snap.speed_scaling
        scaling = 1.0 if not scaling else min(max(float(scaling), 0.05), 1.0)
        return duration / scaling * 1.5 + 10.0

    def plan_joints(self, joints, vel=0.1, acc=0.1):
        """A JointTrajectory to `joints` (profile order), or None. Nothing moves."""
        if self.moveit is None:
            self.log("plan: MoveIt not available")
            return None
        self.set_scaling(vel, acc)
        try:
            traj = self.moveit.plan(joint_positions=list(joints), joint_names=self.profile.joint_names,
                                    timeout_sec=self._plan_timeout())
        except Exception as e:
            self.log(f"plan failed: {e}")
            return None
        if traj is None:
            self.log("plan: planning failed")
        return traj

    def plan_pose(self, target_link: str, T_base_link: np.ndarray, cartesian=False, vel=0.1, acc=0.1,
                  cart_speed=0.0, speed_point=None):
        """A JointTrajectory that brings `target_link` to T_base_link (in profile.base_link), or None.
        cart_speed (m/s, <= 0 = none; linear moves only): the trajectory is retimed so the point `speed_point`
        (xyz in target_link, the TCP) goes at that constant speed, never faster (cart_speed.py: move_group ignores
        max_cartesian_speed)."""
        if self.moveit is None:
            self.log("plan: MoveIt not available")
            return None
        limit = bool(cartesian) and float(cart_speed or 0.0) > 0.0
        point_at = None
        if limit:
            point_at = self._speed_point(target_link, speed_point)
            if point_at is None:         # never move faster than asked: no plan rather than an unlimited one
                self.log(f"plan: cannot limit the speed of {target_link} (no robot model with that link)")
                return None
        position, quat_xyzw = tf.to_pos_quat(T_base_link)
        self.set_scaling(vel, acc)
        self.log(f"plan {'linear ' if cartesian else ''}{target_link} -> "
                 f"{[round(v, 4) for v in position]} {[round(v, 4) for v in quat_xyzw]}"
                 + (f" at <= {cart_speed * 1000.0:.0f} mm/s" if limit else ""))
        try:
            traj = self.moveit.plan(position=position, quat_xyzw=quat_xyzw, target_link=target_link,
                                    cartesian=bool(cartesian), frame_id=self.profile.base_link,
                                    timeout_sec=self._plan_timeout())
        except Exception as e:
            self.log(f"plan failed: {e}")
            return None
        if traj is None:
            self.log("plan: planning failed")
        elif limit:
            try:
                # constant TCP speed within the joint limits (URDF velocity x Speed %, profile acceleration x
                # Accel %); the uniform slow-down stays as the fallback and the final peak check
                vlim, alim = cart_speed_mod.joint_limits(self._fk_model, self.profile, traj.joint_names, vel, acc)
                cart_speed_mod.retime_linear(traj, point_at, float(cart_speed),
                                             cart_speed_mod.cart_accel_m_s2(self.profile, acc), vlim, alim, self.log)
            except Exception as e:      # a malformed trajectory: do not run it unlimited
                self.log(f"plan: speed limit failed: {e}")
                return None
        return traj

    def _speed_point(self, link: str, point):
        """cart_speed point_at for a point in `link`, from robot_description (parsed once per description)."""
        xml = self.robot_description()
        if not xml:
            return None
        if getattr(self, "_fk_xml", None) is not xml:
            try:
                from .robot_model import parse_urdf
                self._fk_model = parse_urdf(xml)
            except Exception as e:
                self.log(f"robot_description not usable for the speed limit: {e}")
                return None
            self._fk_xml = xml
        return cart_speed_mod.point_fk(self._fk_model, self.profile.base_link, link, point)

    def execute_trajectory(self, trajectory) -> bool:
        return self._execute_on_active_controller(trajectory)

    def move_joints(self, joints, vel=0.1, acc=0.1) -> bool:
        self.log(f"MoveJ -> {[round(j, 4) for j in joints]}")
        epoch = self.stop_epoch
        traj = self.plan_joints(joints, vel, acc)
        return traj is not None and self._execute_on_active_controller(traj, epoch)

    def move_pose(self, target_link: str, T_base_link: np.ndarray, cartesian=False, vel=0.1, acc=0.1,
                  cart_speed=0.0, speed_point=None) -> bool:
        """Move `target_link` to T_base_link (pose in profile.base_link). TCP/frame conversion
        already happened in geometry.Geometry.planning_target. cart_speed / speed_point: see plan_pose."""
        epoch = self.stop_epoch
        traj = self.plan_pose(target_link, T_base_link, cartesian, vel, acc, cart_speed, speed_point)
        return traj is not None and self._execute_on_active_controller(traj, epoch)

    # ========================================================= planning scene
    # Keep-out zones (keepout.py). pymoveit2 PUBLISHES collision objects on /collision_object (fire and forget,
    # volatile): nothing confirms them, so keepout.KeepoutScene reads the scene back (scene_objects) and
    # re-publishes what is missing. Nothing here moves the robot.
    SCENE_READ_TIMEOUT_S = 2.0

    def scene_set_box(self, object_id: str, size_m, T_base_box: np.ndarray) -> Tuple[bool, str]:
        """Add (or replace: MoveIt's ADD on an existing id replaces it) a world box, pose in base_link."""
        if self.moveit is None:
            return False, "MoveIt interface unavailable"
        position, quat_xyzw = tf.to_pos_quat(T_base_box)
        try:
            self.moveit.add_collision_box(id=object_id, size=tuple(float(v) for v in size_m), position=position,
                                          quat_xyzw=quat_xyzw, frame_id=self.profile.base_link)
        except Exception as e:
            return False, f"{type(e).__name__}: {e}"
        return True, "published"

    def scene_remove(self, object_id: str) -> Tuple[bool, str]:
        if self.moveit is None:
            return False, "MoveIt interface unavailable"
        try:
            self.moveit.remove_collision_object(object_id)
        except Exception as e:
            return False, f"{type(e).__name__}: {e}"
        return True, "published"

    def scene_objects(self) -> Optional[Dict[str, dict]]:
        """World collision objects from move_group's get_planning_scene: {id: {frame, size, T}} (T in base_link,
        None when that frame cannot be resolved; size of a single box primitive, else None). None = not readable."""
        if self.moveit is None:
            return None
        try:
            if not self.moveit.update_planning_scene(timeout_sec=self.SCENE_READ_TIMEOUT_S):
                return None
            scene = self.moveit.planning_scene
        except Exception as e:
            self.log(f"planning scene read failed: {e}")
            return None
        if scene is None:
            return None
        out: Dict[str, dict] = {}
        base = self.profile.base_link
        for co in scene.world.collision_objects:
            size = None
            T_obj_prim = np.eye(4)
            if len(co.primitives) == 1 and int(co.primitives[0].type) == 1:      # shape_msgs SolidPrimitive.BOX
                size = [float(v) for v in co.primitives[0].dimensions]
                if len(co.primitive_poses) == 1:
                    T_obj_prim = _pose_to_T(co.primitive_poses[0])
            frame = co.header.frame_id or base
            T_frame = np.eye(4) if frame == base else self.lookup_transform(base, frame)
            T = None if T_frame is None else T_frame @ _pose_to_T(co.pose) @ T_obj_prim
            out[co.id] = {"frame": frame, "size": size, "T": T}
        return out

    # Parts (parts.py): static TF for part frames the robot model does not have yet, and MoveIt collision objects
    # (attached to the arm link a part rides on, or world objects for the cell). Published on move_group's
    # collision_object / attached_collision_object topics; parts_scene_ids reads them back. Never moves the robot.
    def parts_publish(self, frames, objects, remove_ids) -> Tuple[bool, str]:
        try:
            from geometry_msgs.msg import Pose, Point, TransformStamped
            from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
            from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
            from tf2_ros import StaticTransformBroadcaster
        except ImportError as e:
            return False, f"missing ROS package: {e}"
        n = self.node
        if getattr(self, "_parts_pubs", None) is None:
            latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self._parts_pubs = (n.create_publisher(CollisionObject, "/collision_object", 100),
                                n.create_publisher(AttachedCollisionObject, "/attached_collision_object", 100))
            self._parts_static = StaticTransformBroadcaster(n, qos=latched)
        co_pub, aco_pub = self._parts_pubs
        if frames:
            msgs = []
            for parent, child, T in frames:
                m = TransformStamped()
                m.header.frame_id, m.child_frame_id = parent, child
                p, q = tf.to_pos_quat(T)
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = p
                (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
                 m.transform.rotation.w) = q
                msgs.append(m)
            self._parts_static.sendTransform(msgs)

        def pose(T):
            p, q = tf.to_pos_quat(T)
            out = Pose()
            out.position.x, out.position.y, out.position.z = p
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
            return out

        for oid, link in remove_ids:
            a = AttachedCollisionObject()
            a.link_name, a.object.id, a.object.operation = link or "", oid, CollisionObject.REMOVE
            if link:
                aco_pub.publish(a)                 # detaches (MoveIt puts it in the world) ...
            c = CollisionObject()
            c.id, c.operation = oid, CollisionObject.REMOVE
            c.header.frame_id = self.profile.base_link
            co_pub.publish(c)                      # ... and this removes it from the world
        for o in objects:
            c = CollisionObject()
            c.id, c.operation = o["id"], CollisionObject.ADD
            c.header.frame_id = o["link"]
            c.header.stamp = n.get_clock().now().to_msg()
            c.pose = pose(np.eye(4))
            for x in o.get("geoms") or [{"geom": o["geom"], "T_link_obj": o["T_link_obj"]}]:
                g, T = x["geom"], x["T_link_obj"]
                if g["type"] == "mesh":
                    from .parts import read_mesh
                    from urllib.parse import unquote, urlparse
                    try:
                        tri = read_mesh(unquote(urlparse(g["uri"]).path)) * np.asarray(g["scale"], float)
                    except Exception as e:
                        return False, f"{o['id']}: {e}"
                    mesh = Mesh()
                    mesh.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in tri.reshape(-1, 3)]
                    mesh.triangles = [MeshTriangle(vertex_indices=[3 * k, 3 * k + 1, 3 * k + 2]) for k in range(len(tri))]
                    c.meshes.append(mesh)
                    c.mesh_poses.append(pose(T))
                else:
                    prim = SolidPrimitive()
                    if g["type"] == "box":
                        prim.type, prim.dimensions = SolidPrimitive.BOX, [float(v) for v in g["size"]]
                    elif g["type"] == "cylinder":
                        prim.type, prim.dimensions = SolidPrimitive.CYLINDER, [float(g["length"]), float(g["radius"])]
                    else:
                        prim.type, prim.dimensions = SolidPrimitive.SPHERE, [float(g["radius"])]
                    c.primitives.append(prim)
                    c.primitive_poses.append(pose(T))
            if o["attach"]:
                a = AttachedCollisionObject()
                a.link_name, a.object, a.touch_links = o["link"], c, list(o["touch_links"])
                aco_pub.publish(a)
            else:
                co_pub.publish(c)
        return True, "published"

    def parts_scene_ids(self) -> Optional[Dict[str, str]]:
        """{object id: attached link or ""} for the gui_part/ objects in move_group's planning scene; None = unreadable."""
        try:
            from moveit_msgs.msg import PlanningSceneComponents
            from moveit_msgs.srv import GetPlanningScene
        except ImportError:
            return None
        if getattr(self, "_scene_cli", None) is None:
            self._scene_cli = self.node.create_client(GetPlanningScene, "/get_planning_scene", callback_group=self.cb)
        if not self._scene_cli.service_is_ready():
            return None
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        res = self._call(self._scene_cli, req, timeout_s=self.SCENE_READ_TIMEOUT_S)
        if res is None:
            return None
        out = {co.id: "" for co in res.scene.world.collision_objects if co.id.startswith("gui_part/")}
        for a in res.scene.robot_state.attached_collision_objects:
            if a.object.id.startswith("gui_part/"):
                out[a.object.id] = a.link_name
        return out






    # ======================================================== transition checks
    def trajectory_action_ready(self, controller: str) -> bool:
        client = self._follow_clients.get(controller)
        if client is None:
            client = ActionClient(self.node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory",
                                  callback_group=self.cb)
            self._follow_clients[controller] = client
        return client.wait_for_server(timeout_sec=2.0)

    def verify_reference_at_current_pose(self, check: dict) -> Tuple[bool, str]:
        """The patched controller sets its reference from the measured pose in on_activate, before the
        first torque. Confirm that from controller_state; no reference is ever published from here."""
        try:
            from cartesian_impedance_controller.msg import ControllerState
        except ImportError:
            return False, "cartesian_impedance_controller messages are not available to the GUI"
        topic = check.get("reference_topic")
        if topic and self.node.count_publishers(topic) > 0:
            return False, f"something is publishing on {topic}; it could overwrite the reference"
        got: List = []
        event = threading.Event()

        def on_state(msg):
            got.append(msg)
            event.set()
        sub = self.node.create_subscription(ControllerState, check["state_topic"], on_state, 1,
                                            callback_group=self.cb)
        try:
            if not event.wait(float(check.get("timeout_s", 1.0))):
                return False, f"no message on {check['state_topic']}"
        finally:
            self.node.destroy_subscription(sub)
        cur, ref = got[-1].current_pose, got[-1].reference_pose
        T_cur = tf.make([cur.position.x, cur.position.y, cur.position.z],
                        [cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w])
        T_ref = tf.make([ref.position.x, ref.position.y, ref.position.z],
                        [ref.orientation.x, ref.orientation.y, ref.orientation.z, ref.orientation.w])
        dist = float(np.linalg.norm(T_cur[:3, 3] - T_ref[:3, 3]))
        angle = float(tf.angle_between(T_cur, T_ref))
        if dist > float(check.get("max_position_error_m", 0.002)):
            return False, f"reference is {dist * 1000:.1f} mm from the measured pose"
        if angle > float(check.get("max_orientation_error_rad", 0.02)):
            return False, f"reference is {math.degrees(angle):.2f} deg from the measured pose"
        return True, ""

    def _servo_endpoint(self, route: Optional[str] = None) -> Tuple[str, Optional[dict]]:
        name = route or self.profile.servo.get("default_route") or ""
        return name, self._servos.get(name)

    def _on_servo_status(self, route: str, msg):
        ep = self._servos.get(route)
        if ep is not None:
            ep["status"] = (int(msg.code), str(msg.message), time.monotonic())

    def servo_status(self, route: Optional[str] = None) -> Tuple[Optional[int], str]:
        """One Servo route's last status; code None if none arrived in the last second."""
        _, ep = self._servo_endpoint(route)
        if ep is None:
            return None, ""
        code, text, t = ep["status"]
        return (code, text) if time.monotonic() - t < 1.0 else (None, "")

    def servo_output_count(self, route: Optional[str] = None) -> Optional[int]:
        """Commands this route's Servo node has published to its controller so far (None: not monitored)."""
        _, ep = self._servo_endpoint(route)
        return ep["out_n"] if ep is not None and ep["cfg"].get("output_topic") else None

    def servo_ready(self, route: Optional[str] = None) -> bool:
        """The selected Servo node answers, subscribes to commands and is running its loop."""
        name, ep = self._servo_endpoint(route)
        if ep is None or ep["cli"] is None or not ep["cli"].service_is_ready():
            return False
        topic = ep["cfg"].get("twist_topic")
        return bool(topic) and self.node.count_subscribers(topic) > 0 and self.servo_status(name)[0] is not None

    def servo_command_type(self, kind: str, route: Optional[str] = None) -> Tuple[bool, str]:
        """Tell Servo which command it gets next: 'joint' (JointJog) or 'twist'."""
        from moveit_msgs.srv import ServoCommandType
        name, ep = self._servo_endpoint(route)
        if ep is None or ep["cli"] is None:
            return False, f"no MoveIt Servo route '{name}'"
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG if kind == "joint" else ServoCommandType.Request.TWIST
        res = self._call(ep["cli"], req, 2.0)
        if res is None:
            return False, f"MoveIt Servo route '{name}' did not answer"
        return bool(res.success), "ok" if res.success else "Servo refused the command type"

    def servo_twist(self, linear, angular, frame: str, route: Optional[str] = None):
        """Velocity of Servo's tip (m/s, rad/s) in `frame`; stamped now, Servo drops stale commands."""
        _, ep = self._servo_endpoint(route)
        if ep is None or ep["twist_pub"] is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(v) for v in linear)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (float(v) for v in angular)
        ep["twist_pub"].publish(msg)

    def servo_joint_jog(self, velocities, route: Optional[str] = None):
        _, ep = self._servo_endpoint(route)
        if ep is None or ep["joint_pub"] is None:
            return
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = list(self.profile.joint_names)
        msg.velocities = [float(v) for v in velocities]
        ep["joint_pub"].publish(msg)

    # =============================================================== gripper
    def gripper_move(self, position: float, effort: float) -> bool:
        if self._gripper_cli is None or not self._gripper_cli.wait_for_server(timeout_sec=1.0):
            self.log("gripper action server not available")
            return False
        if self._gripper_kind == "parallel_gripper":
            goal = ParallelGripperCommand.Goal()
            goal.command.name = [self.profile.gripper.get("joint_name", "")]
            goal.command.position = [float(position)]
            goal.command.effort = [float(effort)]
        else:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(effort)
        gh = self._wait_future(self._gripper_cli.send_goal_async(goal), 3.0)
        if gh is None or not gh.accepted:
            self.log("gripper goal rejected")
            return False
        res = self._wait_future(gh.get_result_async(), 10.0)
        if res is None:
            self.log("gripper: no result")
            return False
        r = res.result
        # A 2F-85 closing on an object stalls before the target: that's a successful grasp.
        reached, stalled = getattr(r, "reached_goal", True), getattr(r, "stalled", False)
        self.log(f"gripper -> {position:.3f}: reached={reached} stalled={stalled}")
        return bool(reached or stalled)

    def gripper_reconnect(self) -> Tuple[bool, str]:
        """Bring the gripper's hardware and controllers up again on its own controller manager (bridge_api)."""
        g = self.profile.gripper
        gcm = g.get("controller_manager")
        if not gcm:
            return False, "no gripper controller manager in this profile"
        clients = self._gripper_cm_cli

        def cli(t, srv):
            return clients[srv]

        def hw_states():
            res = self._call(cli(ListHardwareComponents, "list_hardware_components"), ListHardwareComponents.Request())
            return None if res is None else {c.name: c.state.label for c in res.component}

        def hw_activate(name):
            req = SetHardwareComponentState.Request(name=name)
            req.target_state = LifecycleState(id=LifecycleState.PRIMARY_STATE_ACTIVE, label="active")
            res = self._call(cli(SetHardwareComponentState, "set_hardware_component_state"), req, 10.0)
            if res is None:
                return False, "no response"
            self.log(f"gripper hardware {name} -> {res.state.label} ({'ok' if res.ok else 'rejected'})")
            return bool(res.ok), res.state.label

        def controllers():
            res = self._call(cli(ListControllers, "list_controllers"), ListControllers.Request())
            return None if res is None else {c.name: c.state for c in res.controller}

        def load(name):
            res = self._call(cli(LoadController, "load_controller"), LoadController.Request(name=name), 10.0)
            return bool(res and res.ok)

        def configure(name):
            res = self._call(cli(ConfigureController, "configure_controller"), ConfigureController.Request(name=name),
                             10.0)
            return bool(res and res.ok)

        def activate(names):
            req = SwitchController.Request(activate_controllers=list(names), strictness=SwitchController.Request.STRICT)
            req.timeout = DurationMsg(sec=5)
            res = self._call(cli(SwitchController, "switch_controller"), req, 7.0)
            return bool(res and res.ok)

        ops = SimpleNamespace(hw_states=hw_states, hw_activate=hw_activate, controllers=controllers, load=load,
                              configure=configure, activate=activate,
                              set_params_file=lambda c, f: self.set_node_parameters(gcm, {f"{c}.params_file": [f]}))
        ok, msg = reconnect_gripper(ops, g.get("hardware"), list(g.get("controllers") or [g.get("controller")]),
                                    g.get("params_file"), self.log)
        self.log(msg)
        return ok, msg

    def gripper_reactivate(self) -> Tuple[bool, str]:
        if self._gripper_reactivate_cli is None:
            return False, "no reactivate service configured"
        return self._trigger(self._gripper_reactivate_cli, "gripper reactivate")

    # ============================================================= UR driver
    def _trigger(self, client, label: str, timeout_s: float = 3.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=0.5):
            msg, res = f"{label}: service not available", None
        else:
            res = self._wait_future(client.call_async(Trigger.Request()), timeout_s)
            msg = (f"{label}: no answer within {timeout_s:.0f} s (the robot may still be doing it: check the robot "
                   "mode)" if res is None else f"{label}: {res.message or ('ok' if res.success else 'failed')}")
        self.log(msg)
        return (res is not None and res.success), msg

    def _param_call(self, srv_type, service: str, request):
        """A parameter service call on a client kept per service. A new client per call had to discover the service
        again each time (0.5 s): on a busy ROS graph that missed, and the F/T bias / gains read as "not running"."""
        with self._param_lock:
            cli = self._param_clients.get(service)
            first = cli is None
            if first:
                cli = self._param_clients[service] = self.node.create_client(srv_type, service, callback_group=self.cb)
        if first and not cli.wait_for_service(timeout_sec=2.0):
            return None
        return self._call(cli, request)

    def set_node_parameters(self, node_name: str, params: Dict[str, object]) -> Tuple[bool, str]:
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        req = SetParameters.Request(parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in params.items()])
        res = self._param_call(SetParameters, f"{node_name}/set_parameters", req)
        if res is None:
            return False, f"{node_name}/set_parameters not available"
        bad = [f"{k}: {r.reason}" for k, r in zip(params, res.results) if not r.successful]
        return (not bad), ("; ".join(bad) if bad else "parameters set")

    def get_node_parameters(self, node_name: str, names: List[str]) -> Optional[Dict[str, object]]:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        res = self._param_call(GetParameters, f"{node_name}/get_parameters", GetParameters.Request(names=list(names)))
        if res is None:
            return None
        return {n: parameter_value_to_python(v) for n, v in zip(names, res.values)}

    def dashboard(self, command: str) -> Tuple[bool, str]:
        if command not in DASHBOARD_COMMANDS:
            return False, f"unknown dashboard command {command}"
        # the driver's ~/connect and ~/quit reopen the Dashboard connection after the pendant went Local -> Remote
        return dashboard_with_reconnect(self._dash_call, command, self.log)

    def _dash_call(self, c: str) -> Tuple[bool, str]:
        return self._trigger(self._dash[c], c, DASHBOARD_TIMEOUT_S.get(c, 5.0))

    def _dash_query(self, name: str, once: Callable[[], Tuple[bool, str]]) -> Tuple[bool, str]:
        """A dashboard query that reconnects (quit, connect) once when the driver lost the Dashboard connection."""
        return dashboard_with_reconnect(lambda c: once() if c == name else self._dash_call(c), name, self.log)

    def loaded_program(self) -> Tuple[bool, str]:
        """(True, program file name) of the program loaded on the pendant, or (False, why). Read-only."""
        if not HAVE_UR_MSGS:
            return False, "ur_dashboard_msgs not installed"

        def once():
            res = self._call(self._loaded_cli, GetLoadedProgram.Request(), 5.0)
            if res is None:
                return False, "get_loaded_program: no answer"
            return bool(res.success), (res.program_name if res.success else res.answer)
        return self._dash_query("get_loaded_program", once)

    def load_program(self, filename: str) -> Tuple[bool, str]:
        if not HAVE_UR_MSGS:
            return False, "ur_dashboard_msgs not installed"

        def once():
            res = self._call(self._load_cli, Load.Request(filename=filename))
            if res is None:
                return False, "load_program: service not available"
            return bool(res.success), res.answer
        return self._dash_query("load_program", once)

    def resend_robot_program(self) -> Tuple[bool, str]:
        return self._trigger(self._resend_cli, "resend robot program")

    def zero_ft_sensor(self) -> Tuple[bool, str]:
        # NOTE: this zeroes the UR's built-in wrist F/T. The ATI Gamma is biased by its own driver.
        return self._trigger(self._zero_ft_cli, "zero UR F/T sensor")

    def set_speed_slider(self, fraction: float) -> bool:
        if not HAVE_UR_MSGS:
            return False
        res = self._call(self._speed_cli, SetSpeedSliderFraction.Request(speed_slider_fraction=float(fraction)))
        ok = bool(res and res.success)
        self.log(f"speed slider {fraction:.2f}: {'ok' if ok else 'failed'}")
        return ok

    def set_digital_out(self, pin: int, state: bool) -> bool:
        if not HAVE_UR_MSGS:
            return False
        req = SetIO.Request(fun=SetIO.Request.FUN_SET_DIGITAL_OUT, pin=int(pin), state=1.0 if state else 0.0)
        res = self._call(self._io_cli, req)
        ok = bool(res and res.success)
        self.log(f"DO{pin} = {int(state)}: {'ok' if ok else 'failed'}")
        return ok

    def set_freedrive_keepalive(self, on: bool):
        if self._fd_pub is None:
            return
        was = self._freedrive_on
        self._freedrive_on = bool(on)
        if was and not on:
            self._fd_pub.publish(Bool(data=False))

    # ============================================================== shutdown
    def shutdown(self):
        """Leave the robot at rest: the admittance / impedance controllers keep their last wrench target for ever,
        so zero every wrench publisher (a few times, it is best effort) before the node goes."""
        self._shutting_down = True
        try:
            self.set_freedrive_keepalive(False)
        except Exception as e:
            self.log(f"shutdown: freedrive keepalive: {e}")
        try:
            self.cancel_motion()
        except Exception as e:
            self.log(f"shutdown: cancel motion: {e}")
        for topic, pub in list(self._wrench_pubs.items()):
            frame = next((m.wrench.get("frame", self.profile.base_link) for m in self.profile.modes.values()
                          if m.wrench.get("topic") == topic), self.profile.base_link)
            for _ in range(3):
                try:
                    self.publish_wrench(topic, frame, [0.0] * 6)
                except Exception as e:
                    self.log(f"shutdown: zero wrench on {topic}: {e}")
                    break
                time.sleep(0.02)
        time.sleep(0.1)                                  # let DDS send the last zeros before the node goes
        self._executor.shutdown(timeout_sec=1.0)
        if getattr(self, "_tf_node", None) is not None:
            try:
                self._tf_listener.executor.shutdown(timeout_sec=1.0)
                self._tf_node.destroy_node()
            except Exception as e:
                self.log(f"shutdown: TF node: {e}")
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
