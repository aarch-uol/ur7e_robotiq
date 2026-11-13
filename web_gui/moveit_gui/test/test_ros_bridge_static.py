"""Tests of RosBridge logic that need no ROS: rclpy & messages are stubbed with MagicMock."""
import sys
import threading
import time
import json
import struct
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

STUBS = ["rclpy", "rclpy.action", "rclpy.callback_groups", "rclpy.executors", "rclpy.node", "rclpy.qos", "rclpy.serialization",
         "rclpy.time", "builtin_interfaces", "builtin_interfaces.msg", "controller_manager_msgs",
         "controller_manager_msgs.srv", "geometry_msgs", "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
         "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv", "lifecycle_msgs", "lifecycle_msgs.msg"]
STUBS += ["trajectory_msgs", "trajectory_msgs.msg", "action_msgs", "action_msgs.srv"]


@pytest.fixture
def ros_bridge_module():
    with mock.patch.dict(sys.modules, {m: mock.MagicMock() for m in STUBS}):
        sys.modules.pop("moveit_gui.ros_bridge", None)
        import moveit_gui.ros_bridge as rb
        yield rb
        sys.modules.pop("moveit_gui.ros_bridge", None)


def _bare_bridge(rb):
    from moveit_gui.bridge_api import Snapshot
    from moveit_gui.profile import RobotProfile
    b = object.__new__(rb.RosBridge)
    b.profile = RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml")
    b._lock = threading.Lock()
    b._snap = Snapshot()
    b._t_joints = time.monotonic()
    b._joints_received = True
    b._velocities_ok = True
    b._freedrive_on = False
    b._ctrl_ref, b._follow_clients = {}, {}
    b._policy_step_buf = deque()
    return b


def test_policy_step_nonfinite_values_become_null_without_dropping_row(ros_bridge_module):
    b = _bare_bridge(ros_bridge_module)
    b.log = lambda *_: None
    b._on_policy_step(type("Msg", (), {"data": json.dumps({
        "t": 1.0, "step": 4, "action": [float("nan"), float("inf"), -float("inf")],
        "command": [0.1, 0.2], "obs_sigma": {"vision": float("nan")},
        "other": [1, {"bad": float("inf")}],
    }, allow_nan=True)})())
    rows = b.policy_step_samples(0)
    assert rows == [{"t": 1.0, "step": 4, "action": [None, None, None],
                    "command": [0.1, 0.2], "obs_sigma": {"vision": None},
                    "other": [1, {"bad": None}], "_received": rows[0]["_received"]}]


def test_snapshot_does_not_deadlock(ros_bridge_module):
    b = _bare_bridge(ros_bridge_module)
    out = {}
    t = threading.Thread(target=lambda: out.setdefault("snap", b.snapshot()), daemon=True)
    t.start()
    t.join(2.0)
    assert not t.is_alive(), "RosBridge.snapshot() deadlocked on its own lock"
    assert out["snap"].joint_state_age is not None


def test_unknown_moveit_result_is_failure(ros_bridge_module):
    b = _bare_bridge(ros_bridge_module)
    b.moveit = mock.MagicMock()
    b.moveit.wait_until_executed.return_value = None
    b._logs = __import__("collections").deque()
    b.node = mock.MagicMock()
    assert b._moveit_wait() is False


def test_is_stationary_unknown_without_velocities(ros_bridge_module):
    b = _bare_bridge(ros_bridge_module)
    assert b.is_stationary() is None          # no velocities in the snapshot
    b._snap.joint_velocities = [0.0] * 6
    assert b.is_stationary() is True
    b._t_joints = time.monotonic() - 5
    assert b.is_stationary() is None          # stale


def test_point_cloud_subscription_is_lazy_and_decodes_latest_sample(ros_bridge_module):
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b.node = mock.MagicMock()
    b.cb = mock.MagicMock()
    b._tf = None
    b.node.get_topic_names_and_types.return_value = [("/cloud", ["sensor_msgs/msg/PointCloud2"])]
    sub = object()
    callbacks = []
    b.node.create_subscription.side_effect = lambda _cls, _topic, cb, *_args, **_kw: (callbacks.append(cb) or sub)
    fields = [SimpleNamespace(name=n, offset=i * 4, datatype=7, count=1) for i, n in enumerate(("x", "y", "z"))]
    raw = struct.pack("<fff", 1.0, 2.0, 3.0)
    msg = SimpleNamespace(width=1, height=1, point_step=12, row_step=12, fields=fields,
                          is_bigendian=False, data=raw,
                          header=SimpleNamespace(frame_id=b.profile.base_link))
    b.lookup_transform = lambda _parent, _child: np.eye(4)

    assert b.point_cloud_topics() == [("/cloud", "sensor_msgs/msg/PointCloud2")]
    assert b.node.create_subscription.call_count == 0
    assert b.point_cloud_snapshot("/cloud")[2] == "waiting for point cloud"
    callbacks[0](msg)
    points, colors, why = b.point_cloud_snapshot("/cloud")

    assert why == "" and points.tolist() == [[1.0, 2.0, 3.0]] and colors.shape == (1, 3)
    assert b.node.create_subscription.call_count == 1


def test_trajectory_goal_names_the_controller_of_the_active_mode(ros_bridge_module):
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b.log = lambda *_: None
    b.list_controllers = lambda: {"admittance_controller": "inactive", "admittance_trajectory_controller": "inactive",
                                  "scaled_joint_trajectory_controller": "active"}
    b._execute_cli = mock.MagicMock()
    rb.ExecuteTrajectory = mock.MagicMock()
    b._wait_future = mock.MagicMock(return_value=None)     # goal rejected after being sent: enough to inspect it
    assert b._execute_on_active_controller(mock.MagicMock()) is False
    goal = b._execute_cli.send_goal_async.call_args[0][0]
    assert goal.controller_names == ["scaled_joint_trajectory_controller"]


def _admittance_bridge(rb, ref):
    b = _bare_bridge(rb)
    logs = []
    b.log = logs.append
    b.list_controllers = lambda: {"admittance_controller": "active", "admittance_trajectory_controller": "active",
                                  "scaled_joint_trajectory_controller": "inactive"}
    b._execute_cli = mock.MagicMock()
    b._follow_clients = {"admittance_trajectory_controller": mock.MagicMock()}
    b._ctrl_ref = {} if ref is None else {"admittance_trajectory_controller": (time.monotonic(), ref)}
    b.stop_epoch = 0
    b._execute_handle = None
    b._snap.speed_scaling = 1.0
    return b, logs


def _traj(rb, names, rows):
    from types import SimpleNamespace
    return SimpleNamespace(joint_names=list(names), points=[SimpleNamespace(positions=list(r)) for r in rows])


def test_admittance_moves_start_at_the_controller_reference(ros_bridge_module):
    """A force pushes the arm off the trajectory controller's reference; a move must start at the reference (a
    start at the measured joints doubles the push), so it goes to the controller shifted, not through MoveIt."""
    rb = ros_bridge_module
    from moveit_gui.profile import RobotProfile
    J = RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml").joint_names
    ref = {j: 0.1 * i for i, j in enumerate(J)}
    measured = [0.1 * i + (0.02 if i == 1 else 0.0) for i in range(len(J))]       # pushed 0.02 rad on joint 1
    b, logs = _admittance_bridge(rb, ref)
    client = b._follow_clients["admittance_trajectory_controller"]
    b._wait_future = mock.MagicMock(return_value=None)
    rb.FollowJointTrajectory = mock.MagicMock()
    rb.FollowJointTrajectory.Goal = lambda: mock.MagicMock()
    assert b._execute_on_active_controller(_traj(rb, J, [measured, [v + 0.1 for v in measured]])) is False
    b._execute_cli.send_goal_async.assert_not_called()                          # not through MoveIt
    goal = client.send_goal_async.call_args[0][0]
    pts = [list(p.positions) for p in goal.trajectory.points]
    assert pts[0] == [ref[j] for j in J]                                        # starts at the reference
    assert abs(pts[1][1] - (measured[1] + 0.1 - 0.02)) < 1e-9                   # the whole path shifted
    assert any("a force is pushing the arm" in line for line in logs)


def test_admittance_move_refused_without_a_reference_or_when_pushed_far(ros_bridge_module):
    rb = ros_bridge_module
    from moveit_gui.profile import RobotProfile
    J = RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml").joint_names
    b, logs = _admittance_bridge(rb, None)
    assert b._execute_on_active_controller(_traj(rb, J, [[0.0] * 6])) is False
    assert any("no fresh reference" in line for line in logs)
    b, logs = _admittance_bridge(rb, {j: 0.0 for j in J})
    b._ctrl_ref["admittance_trajectory_controller"] = (time.monotonic() - 5.0, {j: 0.0 for j in J})   # stale
    assert b._execute_on_active_controller(_traj(rb, J, [[0.0] * 6])) is False
    b, logs = _admittance_bridge(rb, {j: 0.0 for j in J})
    assert b._execute_on_active_controller(_traj(rb, J, [[0.0, 0.3, 0, 0, 0, 0]])) is False
    assert any("off its reference" in line for line in logs)
    b._follow_clients["admittance_trajectory_controller"].send_goal_async.assert_not_called()


def test_trajectory_is_refused_when_no_active_mode_takes_moveit(ros_bridge_module):
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b.log = lambda *_: None
    b.list_controllers = lambda: {"cartesian_impedance_controller": "active"}
    b._execute_cli = mock.MagicMock()
    assert b._execute_on_active_controller(mock.MagicMock()) is False
    b._execute_cli.send_goal_async.assert_not_called()


def test_health_tick_reports_moveit_only_when_its_action_server_answers(ros_bridge_module):
    """The graph also lists actions that only have a client (the GUI's own), so names prove nothing.
    (An earlier version also called Node.get_action_names_and_types(), which does not exist, and killed the
    executor thread.)"""
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b.node = mock.create_autospec(object, instance=True)
    b._t_wrench = 0.0
    b._gripper_cli = b._servo_cli = b._gripper_list_cli = None
    b._list_cli = mock.MagicMock()
    b._list_cli.service_is_ready.return_value = False
    b._execute_cli = mock.MagicMock()
    b._execute_cli.server_is_ready.return_value = False
    b._health_tick()
    assert b._snap.connected["MoveIt"] is False
    b._execute_cli.server_is_ready.return_value = True
    b._health_tick()
    assert b._snap.connected["MoveIt"] is True



def _executing_bridge(rb):
    b = _bare_bridge(rb)
    b.log = lambda *_: None
    b.list_controllers = lambda: {"scaled_joint_trajectory_controller": "active"}
    b._execute_cli = mock.MagicMock()
    b._execute_handle = None
    rb.ExecuteTrajectory = mock.MagicMock()
    return b


def test_stop_while_planning_never_sends_the_goal(ros_bridge_module):
    """pymoveit2 cannot cancel plan(): a STOP during planning bumps stop_epoch and nothing is sent afterwards."""
    rb = ros_bridge_module
    b = _executing_bridge(rb)

    def plan(*_a, **_k):
        b.cancel_motion()                         # STOP pressed while MoveIt plans
        return mock.MagicMock()
    b.moveit = mock.MagicMock()
    b.moveit.plan.side_effect = plan
    assert b.move_joints([0.0] * 6) is False
    b._execute_cli.send_goal_async.assert_not_called()


def test_stop_while_the_goal_is_being_sent_cancels_it(ros_bridge_module):
    rb = ros_bridge_module
    b = _executing_bridge(rb)
    handle = mock.MagicMock(accepted=True)
    result = mock.MagicMock()
    result.result.error_code.val = -7             # PREEMPTED, as MoveIt reports a cancelled goal
    calls = iter([handle, result])

    def wait_future(_future, _timeout):
        out = next(calls)
        if out is handle:
            b.stop_epoch += 1                     # STOP arrived while the goal was in flight
        return out
    b._wait_future = wait_future
    assert b._execute_on_active_controller(mock.MagicMock()) is False
    handle.cancel_goal_async.assert_called()


def test_stop_reaches_move_group_and_every_trajectory_controller(ros_bridge_module):
    """MoveIt 2.12 accepts a cancel of /execute_trajectory without stopping the trajectory, so STOP also sends
    "stop" on trajectory_execution_event and cancels all goals of each trajectory controller MoveIt can use."""
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b.stop_epoch = 0
    b._exec_event_pub = mock.MagicMock()
    ready, down = mock.MagicMock(), mock.MagicMock()
    ready.service_is_ready.return_value = True
    down.service_is_ready.return_value = False
    b._fjt_cancel = {"scaled_joint_trajectory_controller": ready, "admittance_trajectory_controller": down}
    b._execute_handle = mock.MagicMock()
    b.moveit = None
    b.cancel_motion()
    assert b.stop_epoch == 1
    rb.String.assert_called_with(data="stop")
    b._exec_event_pub.publish.assert_called_once()
    ready.call_async.assert_called_once()                # a controller that is not up is skipped, not waited for
    down.call_async.assert_not_called()
    b._execute_handle.cancel_goal_async.assert_called_once()


def test_remote_control_state_comes_from_the_dashboard_answer(ros_bridge_module):
    b = _bare_bridge(ros_bridge_module)
    fut = mock.MagicMock()
    fut.result.return_value = mock.MagicMock(success=True, remote_control=True)
    b._on_remote_control(fut)
    assert b._snap.remote_control is True
    fut.result.return_value = mock.MagicMock(success=True, remote_control=False)
    b._on_remote_control(fut)
    assert b._snap.remote_control is False
    fut.result.return_value = mock.MagicMock(success=False, remote_control=False)    # no answer: unknown
    b._on_remote_control(fut)
    assert b._snap.remote_control is None


def test_program_running_is_cleared_when_the_arm_stops_running_and_set_by_the_dashboard(ros_bridge_module):
    """On the UR7e the driver did not report the program stopping after a power cycle; robot_program_running is
    only published on change, so it stayed true. Leaving RUNNING clears it; the Dashboard poll sets the truth."""
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b._snap.program_running = True
    rb.RobotMode = object                                  # ur_dashboard_msgs is not stubbed here
    rb._enum_name = lambda enum, v: v                      # use the mode name directly
    for mode, expect in (("RUNNING", True), ("POWER_OFF", False)):
        b._on_robot_mode(mock.MagicMock(mode=mode))
        assert b._snap.program_running is expect, mode
    b._on_robot_mode(mock.MagicMock(mode="RUNNING"))        # brakes released again: still not running
    assert b._snap.program_running is False
    fut = mock.MagicMock()

    def answer(state, success=True, text=""):
        fut.result.return_value = mock.MagicMock(success=success, answer=text, state=mock.MagicMock(state=state))
        b._on_dashboard_program(fut)
    answer("PLAYING")                     # a program plays, but the driver never saw External Control connect
    assert b._snap.program_running is False and b._snap.program_state == "PLAYING"
    b._on_prog(mock.MagicMock(data=True))                                # External Control connected
    answer("PLAYING")
    assert b._snap.program_running is True and b._snap.program_state == "PLAYING"
    answer("", success=False, text="timeout")                            # no answer: keep what we know
    assert b._snap.program_running is True
    answer("PAUSED")                                                     # a pause is not a stop
    assert b._snap.program_running is False and b._snap.program_state == "PAUSED"
    b._reconnect_dashboard_bg = mock.MagicMock()
    answer("", success=False, text="Failed to send request to dashboard server. Are you connected to the "
                                   "Dashboard Server?")                  # the driver lost the Dashboard
    assert b._snap.program_state is None and b._reconnect_dashboard_bg.called


def test_pendant_speed_percent_becomes_a_fraction(ros_bridge_module):
    """ur_controllers' SpeedScalingStateBroadcaster publishes percent: 100 must not show as 10000 %."""
    b = _bare_bridge(ros_bridge_module)
    b._on_speed(mock.MagicMock(data=100.0))
    assert b._snap.speed_scaling == 1.0
    b._on_speed(mock.MagicMock(data=25.0))
    assert b._snap.speed_scaling == 0.25


def test_pause_in_admittance_holds_at_the_reference_instead_of_cancelling(ros_bridge_module):
    """A cancelled goal makes the trajectory controller hold at the measured (pushed) joints: the push doubled at
    every pause. cancel_motion sends a hold at the reference to that controller and does not cancel it."""
    rb = ros_bridge_module
    from moveit_gui.profile import RobotProfile
    J = RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml").joint_names
    ref = {j: 0.1 * i for i, j in enumerate(J)}
    vel = {j: 0.0 for j in J}
    b, logs = _admittance_bridge(rb, ref)
    b._ctrl_ref["admittance_trajectory_controller"] = (time.monotonic(), ref, vel)
    b._exec_event_pub = mock.MagicMock()
    b.moveit = None
    cancels = {c: mock.MagicMock() for c in ("admittance_trajectory_controller", "scaled_joint_trajectory_controller")}
    b._fjt_cancel = cancels
    rb.FollowJointTrajectory = mock.MagicMock()
    rb.FollowJointTrajectory.Goal = lambda: mock.MagicMock()
    rb.JointTrajectory = lambda: mock.MagicMock()
    b._execute_handle = mock.MagicMock()
    b.cancel_motion()
    client = b._follow_clients["admittance_trajectory_controller"]
    goal = client.send_goal_async.call_args[0][0]
    assert goal.trajectory.points[0].positions == [ref[j] for j in J]
    cancels["admittance_trajectory_controller"].call_async.assert_not_called()   # no cancel: it would hold pushed
    cancels["scaled_joint_trajectory_controller"].call_async.assert_called()      # other controllers as before
    b._execute_handle.cancel_goal_async.assert_not_called()                       # the hold preempts it
    assert any("holding at its reference" in line for line in logs)
    # no fresh reference (the controller is not active): cancel as before
    b2, _ = _admittance_bridge(rb, None)
    b2._exec_event_pub, b2.moveit, b2._fjt_cancel = mock.MagicMock(), None, {"admittance_trajectory_controller": mock.MagicMock()}
    b2._execute_handle = mock.MagicMock()
    b2.cancel_motion()
    b2._fjt_cancel["admittance_trajectory_controller"].call_async.assert_called()
    b2._execute_handle.cancel_goal_async.assert_called()


def test_joint_states_are_read_at_about_100_hz(ros_bridge_module):
    """500 Hz arm + the gripper on /joint_states: messages that come too soon are dropped without decoding."""
    rb = ros_bridge_module
    b = _bare_bridge(rb)
    b._t_grip_msg = 0.0
    seen = []
    rb.deserialize_message = lambda data, cls: seen.append(data) or mock.MagicMock()
    b._on_joints = lambda msg: setattr(b, "_t_joints", rb.time.monotonic())
    b._t_grip_msg = rb.time.monotonic()                  # gripper fresh too
    b._t_joints = rb.time.monotonic()
    for _ in range(5):
        b._on_joints_raw(b"x")                           # 5 messages within 10 ms: none decoded
    assert seen == []
    b._t_joints -= 0.02                                  # the arm is due
    b._on_joints_raw(b"y")
    assert seen == [b"y"]


def test_servo_outside_commanders_counts_only_other_nodes(ros_bridge_module):
    """servo_pause.py keeps Servo running while a policy run without the GUI publishes its commands."""
    from types import SimpleNamespace
    me = SimpleNamespace(node_name="moveit_gui", node_namespace="/")
    other = SimpleNamespace(node_name="policy_runner", node_namespace="/")
    topics = {"/servo_node/delta_twist_cmds": [me, other], "/servo_node/delta_joint_cmds": [me]}
    b = object.__new__(ros_bridge_module.RosBridge)
    b.node = SimpleNamespace(get_name=lambda: "moveit_gui", get_namespace=lambda: "/",
                             get_publishers_info_by_topic=lambda t: topics.get(t, []))
    b._servo_endpoint = lambda route: (route, {"cfg": {"twist_topic": "/servo_node/delta_twist_cmds",
                                                       "joint_topic": "/servo_node/delta_joint_cmds"}})
    assert b.servo_outside_commanders("admittance") == 1
    topics["/servo_node/delta_twist_cmds"] = [me]
    assert b.servo_outside_commanders("admittance") == 0
