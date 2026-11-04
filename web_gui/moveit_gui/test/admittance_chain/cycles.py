"""Force + motion cycles on the real admittance chain, through the GUI's RosBridge (move start, pause / STOP).
Prints one 'METRIC name value' line per check; test_admittance_chain_docker.py asserts on them."""
import sys, time, threading
import numpy as np
import rclpy
sys.path.insert(0, "/mg")
from moveit_gui.profile import RobotProfile
from moveit_gui.ros_bridge import RosBridge
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
import tf2_ros

C = "admittance_trajectory_controller"
prof = RobotProfile.load("/mg/profiles/ur7e_lab.yaml")
cfg = prof.modes["admittance"].reference_start
b = RosBridge(prof, "mg_cycles")
import os
if os.environ.get("EMULATE_OLD") == "1":      # the behaviour before the fix, to prove the checks catch it
    b._hold_at_reference = lambda *a, **k: False                       # pause / STOP cancel the goal
    _orig = b._fresh_reference

    def _measured_as_reference(c, cfg_):                               # moves start at the measured joints
        got = _orig(c, cfg_)
        return None if got is None else (dict(zip(prof.joint_names, b.snapshot().joints)), {}, got[2])
    b._fresh_reference = _measured_as_reference
n = rclpy.create_node("cycles_helper")
ex = MultiThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
wpub = n.create_publisher(WrenchStamped, "/admittance_controller/wrench_reference", 10)
F = [0.0]


def wcb():
    m = WrenchStamped(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = "ft_fts_link"
    m.wrench.force.x = F[0]; wpub.publish(m)
n.create_timer(0.02, wcb)


def tcp():
    t = buf.lookup_transform("base_link", "robotiq_tcp", rclpy.time.Time()).transform.translation
    return np.array([t.x, t.y, t.z]) * 1000.0


def metric(name, v):
    print(f"METRIC {name} {v:.2f}", flush=True)


def watch(secs):
    out, t0 = [], time.time()
    while time.time() - t0 < secs:
        out.append(tcp()); time.sleep(0.02)
    return out


def move(dq, secs):
    """A MoveIt-like plan: starts at the MEASURED joints (as MoveIt plans), pan by dq over secs."""
    J = list(prof.joint_names)
    q = dict(zip(J, b.snapshot().joints))
    tr = JointTrajectory(); tr.joint_names = J
    for k in range(7):
        pos = [q[j] for j in J]; pos[0] += dq * k / 6
        t = secs * k / 6
        tr.points.append(JointTrajectoryPoint(positions=pos, time_from_start=Duration(sec=int(t), nanosec=int(t % 1 * 1e9))))
    res = {}
    th = threading.Thread(target=lambda: res.setdefault("ok", b._execute_from_reference(C, cfg, tr, b.stop_epoch)))
    th.start()
    return th, res


t0 = time.time()
while time.time() - t0 < 15 and (C not in b._ctrl_ref or not b.snapshot().joints):
    time.sleep(0.1)
time.sleep(1.0)
P0 = tcp()
F[0] = 5.0
time.sleep(3.5)
P1 = tcp()
metric("push_mm", np.linalg.norm(P1 - P0))

# 1) hold cycles: a move that goes nowhere, paused, then the next one (resume); the arm must not walk out
for i in range(5):
    th, _ = move(0.0, 2.0)
    time.sleep(0.6)
    b.cancel_motion()
    th.join(10)
    time.sleep(0.4)
metric("walk_after_5_hold_cycles_mm", np.linalg.norm(tcp() - P1))

# 2) motion cycles: a real move, paused mid-way (the arm must stop), resumed by the next move (no jump at start)
worst_pause, worst_start = 0.0, 0.0
for i in range(3):
    Pa = tcp()
    th, _ = move(0.08, 4.0)
    start = watch(0.25)
    worst_start = max(worst_start, max(np.linalg.norm(p - Pa) for p in start))
    time.sleep(1.0)
    Pc = tcp()
    b.cancel_motion()
    after = watch(1.0)
    worst_pause = max(worst_pause, max(np.linalg.norm(p - Pc) for p in after))
    th.join(10)
metric("worst_move_after_pause_mm", worst_pause)
metric("worst_move_first_0p25s_mm", worst_start)          # planned motion alone: ~1-2 mm in 0.25 s

# 3) STOP: the force is released (zeroed) and motion cancelled: the arm eases back by the push, no jump
Ps = tcp()
F[0] = 0.0
b.cancel_motion()
track = watch(3.0)
steps = [np.linalg.norm(track[k + 1] - track[k]) for k in range(len(track) - 1)]
metric("stop_back_mm", np.linalg.norm(track[-1] - Ps))      # ~ the push, back to the reference
metric("stop_biggest_20ms_step_mm", max(steps))
print("DONE", flush=True)
