"""What the GUI, the mode manager and the program runner expect from a bridge.

Implemented by RosBridge (real robot / URSim / mock hardware) and FakeBridge
(no ROS). All motion methods block until the motion ends and return True on
success; call them from worker threads, never from the Qt thread. The GUI
reads state by polling snapshot(), so no ROS callback ever touches a widget.

Methods every bridge provides (besides the UR / gripper / controller calls):
  snapshot() -> Snapshot
  joint_state_age() -> Optional[float]       seconds since the last arm JointState, None = never
  joint_velocities_available() -> bool       last JointState carried velocities for all arm joints
  is_stationary() -> Optional[bool]          None = cannot verify (stale / no velocities)
  controller_claims() -> Optional[Dict[str, List[str]]]   arm controller -> claimed interfaces
  lookup_transform(parent, child) -> Optional[4x4]   T_parent_child; None if missing or stale
  read_pose_source(source_type, source, reference_frame) -> Optional[4x4]   T_R_obj_detected
  trajectory_action_ready(controller) -> bool  FollowJointTrajectory server of that controller is up
  verify_reference_at_current_pose(check) -> (ok, why)  direct torque: reference == measured pose
  servo_ready(route=None) -> bool             selected MoveIt Servo node is ready
  servo_command_type(kind, route=None)        select Twist or JointJog input on that Servo route
  servo_twist(..., route=None)                publish a stamped Cartesian Servo command
  servo_joint_jog(..., route=None)            publish a JointJog Servo command
  set_node_parameters(node, {name: value}) -> (ok, msg)     rcl_interfaces SetParameters
  get_node_parameters(node, [names]) -> Optional[dict]      None = node not reachable
  image_topics() -> [(topic, type)]          advertised sensor_msgs Image / CompressedImage topics
  topic_publishers(topic) -> int
  watch_image(topic | None) -> (ok, msg)     the ONE previewed image topic (camera.ImageWatch)
  watched_image() -> Optional[ImageWatch]
  grab_photo(topic, timeout_s=3.0) -> (jpeg bytes | None, why)   next frame of an image topic, full size (program photo step)
  scene_set_box(object_id, size_m, T_base_box) -> (ok, msg)   add / replace a world collision box in MoveIt's
                                             planning scene, pose in profile.base_link (keepout.py)
  scene_remove(object_id) -> (ok, msg)       remove a world collision object
  scene_objects() -> Optional[{id: {"frame", "size", "T"}}]   world collision objects read back from MoveIt
                                             (T in the base frame or None, size [m] of a box or None);
                                             None = planning scene not readable
  move_joints(joints, vel, acc) -> bool
  move_pose(target_link, T_base_link, cartesian, vel, acc, cart_speed=0.0, speed_point=None) -> bool
                                             cart_speed: m/s limit of a linear move (<= 0 = none) for the point
                                             speed_point (xyz m in target_link: the TCP); plan_pose takes the same
"""
from __future__ import annotations

import threading
from collections import deque

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple


class WrenchHistory:
    """F/T samples for the live chart: the sensor runs at 500 Hz, the page gets them at up to `hz` (not just the
    one value per 10 Hz state), so the chart is smooth and shows short spikes. Thread-safe; keeps `keep_s`."""

    def __init__(self, hz: float = 100.0, keep_s: float = 2.0):
        self.period, self.keep_s = 1.0 / hz, keep_s
        self._buf: "deque" = deque()
        self._lock = threading.Lock()

    def add(self, t: float, w: List[float]):
        with self._lock:
            if self._buf and t - self._buf[-1][0] < self.period * 0.9:     # 0.9: float steps of a 500 Hz input
                return
            self._buf.append((t, list(w)))
            while self._buf and t - self._buf[0][0] > self.keep_s:
                self._buf.popleft()

    def since(self, t: float) -> List[list]:
        """[[t, Fx, Fy, Fz, Tx, Ty, Tz], ...] newer than t (wall clock seconds)."""
        with self._lock:
            return [[round(ts, 3)] + [round(v, 4) for v in w] for ts, w in self._buf if ts > t]


@dataclass
class Snapshot:
    connected: Dict[str, bool] = field(default_factory=dict)     # ROS, UR, MoveIt, Gripper, F/T, Servo
    joints: Optional[List[float]] = None                         # profile.joint_names order
    joint_velocities: Optional[List[float]] = None
    joint_state_age: Optional[float] = None                      # seconds, None = never received
    robot_mode: str = "?"
    safety_mode: str = "?"
    program_running: Optional[bool] = None
    program_state: Optional[str] = None                          # Dashboard: PLAYING | PAUSED | STOPPED (None = unknown)
    remote_control: Optional[bool] = None                        # pendant in Remote Control (None = unknown)
    speed_scaling: Optional[float] = None
    controllers: Dict[str, str] = field(default_factory=dict)    # arm CM: name -> active / inactive
    claimed_interfaces: Dict[str, List[str]] = field(default_factory=dict)   # arm CM: name -> claims
    gripper_controllers: Dict[str, str] = field(default_factory=dict)        # gripper CM (separate)
    gripper_joint: Optional[List[float]] = None                  # [position, velocity, effort]
    wrench: Optional[List[float]] = None                         # Fx Fy Fz Tx Ty Tz
    gripper_position: Optional[float] = None
    freedrive: bool = False
    gripper_status: Optional[dict] = None                        # gripper_status_dict(), None = not reported
    wrench_age: Optional[float] = None                           # seconds since the last F/T sample, None = never
    controllers_age: Optional[float] = None                      # seconds since the controller list was refreshed
    spin_age: Optional[float] = None                             # seconds since the ROS executor last spun (heartbeat)


WRENCH_STALE_S = 0.2        # a force-gated motion (move until contact, F/T zero) needs a reading at most this old


def wrench_fresh(snap: "Snapshot") -> Optional[List[float]]:
    """The snapshot's wrench, or None when there is none or it is older than WRENCH_STALE_S."""
    if not snap.wrench or snap.wrench_age is None or snap.wrench_age > WRENCH_STALE_S:
        return None
    return list(snap.wrench)


# Robotiq GripperStatus (robotiq_msgs), decoded for display. gOBJ is the gripper's own grasp answer.
GRIPPER_OBJECT = {0: "moving", 1: "object detected (opening)", 2: "object held", 3: "at target, no object"}
GRIPPER_FAULTS = {
    0: "none", 5: "action delayed: reactivate", 7: "activation required", 8: "over temperature",
    9: "no communication", 10: "under voltage", 11: "automatic release in progress", 12: "internal fault",
    13: "activation fault", 14: "overcurrent", 15: "automatic release complete", 255: "not reported",
}
GRIPPER_SEVERITY = {0: "none", 1: "warning", 2: "minor", 3: "major", 255: "unknown"}


def gripper_status_dict(msg) -> dict:
    obj, flt, sev = int(msg.object_detection), int(msg.gripper_fault), int(msg.gripper_fault_severity)
    return {"object": GRIPPER_OBJECT.get(obj, f"code {obj}"), "object_code": obj,
            "holding": obj in (1, 2),
            "current_a": round(float(msg.motor_current), 3),
            "fault": GRIPPER_FAULTS.get(flt, f"undefined fault {flt}"), "fault_code": flt,
            "severity": GRIPPER_SEVERITY.get(sev, "major"),
            # a major fault only clears on reactivation; unknown codes count as major (msg definition)
            "needs_reactivate": sev == 3 or flt in (5, 7, 13) or (flt not in GRIPPER_FAULTS)}


DASHBOARD_COMMANDS = (
    "power_on", "power_off", "brake_release", "play", "pause", "stop",
    "unlock_protective_stop", "restart_safety", "close_popup", "close_safety_popup",
)

# The driver answers power_on only once the robot reached IDLE (urcl commandPowerOnWithResponse waits for it) and
# brake_release once it is RUNNING: that takes seconds to tens of seconds, not the 3 s of an ordinary service call.
DASHBOARD_TIMEOUT_S = {"power_on": 90.0, "brake_release": 60.0, "restart_safety": 30.0}


def dashboard_local_refusal(msg: str) -> bool:
    """The UR Dashboard Server decides Local / Remote Control when the connection opens. A connection made while the
    pendant was in Local mode keeps refusing after the switch to Remote:
    "Command is not allowed due to safety reasons, please switch robot to Remote Control mode and reconnect to
    port 29999"."""
    m = (msg or "").lower()
    return "remote control" in m and "reconnect" in m


# The driver's dashboard_client does not reconnect by itself: once the socket dropped (robot controller restart,
# network blip, a request that timed out) every call answers one of these until ~/connect is called.
_DASHBOARD_LOST = ("are you connected to the dashboard server", "non-connected socket",
                   "did not receive answer from dashboard server", "not connected to the dashboard")


def dashboard_lost(msg: str) -> bool:
    m = (msg or "").lower()
    return any(k in m for k in _DASHBOARD_LOST)


def needs_dashboard_reconnect(msg: str) -> bool:
    return dashboard_local_refusal(msg) or dashboard_lost(msg)


def dashboard_with_reconnect(call: Callable[[str], Tuple[bool, str]], command: str,
                             log: Callable[[str], None]) -> Tuple[bool, str]:
    """Send a dashboard command; when the connection is stale (opened in Local mode) or lost, reconnect (quit,
    connect) and send it once more. If the pendant really is in Local mode the second answer is the same refusal,
    and that is returned."""
    ok, msg = call(command)
    if ok or not needs_dashboard_reconnect(msg):
        return ok, msg
    log("dashboard: the connection was opened in Local mode; reconnecting to the Dashboard Server and retrying once"
        if dashboard_local_refusal(msg) else
        "dashboard: the connection to the Dashboard Server was lost; reconnecting and retrying once")
    call("quit")                                   # may fail if already closed: connect is what matters
    ok_c, msg_c = call("connect")
    if not ok_c:
        return False, f"{msg} (reconnect failed: {msg_c})"
    ok, msg2 = call(command)
    if not ok and dashboard_local_refusal(msg2):
        msg2 += " — still refused after reconnecting: check that the pendant shows Remote Control"
    elif not ok and dashboard_lost(msg2):
        msg2 += " — still no Dashboard connection after reconnecting: is the robot controller up (ping, port 29999)?"
    return ok, msg2


# Gripper reconnect. The Robotiq gripper is powered from the UR tool flange: when the stack starts while the robot is
# powered off, its ros2_control hardware fails to configure ("no gripper answered"), the controller spawner dies and
# nothing retries. This redoes what the launch does, once the robot has power: hardware -> active, then load /
# configure / activate the gripper controllers. Already-active parts are left alone, so it is safe to press twice.
GRIPPER_NO_ANSWER = ("the gripper did not answer: check the robot is powered on and Tool I/O supplies 24 V "
                     "(pendant: Installation > Tool I/O), then try again")


def reconnect_gripper(ops, hardware: Optional[str], controllers: List[str], params_file: Optional[str],
                      log: Callable[[str], None] = lambda m: None) -> Tuple[bool, str]:
    """ops: hw_states() -> {name: label} | None, hw_activate(name) -> (ok, label), controllers() -> {name: state} | None,
    set_params_file(name, path) -> (ok, msg), load(name) -> bool, configure(name) -> bool, activate(names) -> bool."""
    if hardware:
        hw = ops.hw_states()
        if hw is None:
            return False, "the gripper controller manager does not answer: is the robot stack running?"
        st = hw.get(hardware)
        if st is None:
            return False, f"no hardware '{hardware}' on the gripper controller manager"
        if st == "finalized":
            return False, f"gripper hardware '{hardware}' is finalized: restart the robot stack"
        if st != "active":
            log(f"gripper hardware {hardware} is {st}: activating")
            ok, label = ops.hw_activate(hardware)
            if not ok:
                return False, GRIPPER_NO_ANSWER + f" (hardware {label or st})"
    states = ops.controllers()
    if states is None:
        return False, "the gripper controller manager does not answer: is the robot stack running?"
    todo = []
    for c in controllers:
        s = states.get(c)
        if s is None:
            if params_file:
                ok, why = ops.set_params_file(c, params_file)
                if not ok:
                    log(f"{c}: could not set params_file ({why}); loading with the controller manager's parameters")
            if not ops.load(c):
                return False, f"could not load the gripper controller {c}"
            s = "unconfigured"
        if s == "unconfigured":
            if not ops.configure(c):
                return False, f"could not configure the gripper controller {c}"
            s = "inactive"
        if s == "inactive":
            todo.append(c)
        elif s != "active":
            return False, f"gripper controller {c} is {s}"
    if todo and not ops.activate(todo):
        return False, f"could not start {', '.join(todo)}: see the log"
    return True, ("gripper reconnected: started " + ", ".join(todo)) if todo else "gripper already connected"
