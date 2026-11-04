"""Robot stack restart + GUI restart, with the real F/T broadcaster (ft_controllers.yaml on a mock sensor) and the
real GUI core (RobotCore + WebService + RosBridge). Prints 'RESULT name ok|FAIL detail' lines, then DONE."""
import os, subprocess, sys, tempfile, time
from pathlib import Path
sys.path.insert(0, "/mg")
tmp = Path(tempfile.mkdtemp()); os.environ["XDG_CONFIG_HOME"] = str(tmp / "cfg")
for d in ("progs", "pol", "profiles"):
    (tmp / d).mkdir()
from moveit_gui.profile import RobotProfile
from moveit_gui.ros_bridge import RosBridge
from moveit_gui.core import RobotCore
from moveit_gui.web.service import WebService
from moveit_gui.ft_bias import PARAMS

URDF = Path("/suite/ft_mock.urdf").read_text()
prof = RobotProfile.load("/mg/profiles/ur7e_lab.yaml")
BIAS = [1.0116, -1.2787, 0.9052, 0.0251, 0.0663, 0.0154]


def stack():
    """The robot stack: a mock UR arm (main controller manager) and the /ft controller manager + broadcaster
    spawner, as drivers.launch.py starts them. Returns the processes to stop for a restart."""
    arm = subprocess.Popen(["/opt/ros/jazzy/lib/controller_manager/ros2_control_node", "--ros-args",
                            "--params-file", "/opt/ros/jazzy/share/ur_robot_driver/config/ur7e_update_rate.yaml",
                            "--params-file", os.environ["ARM_YAML"]],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
                    "scaled_joint_trajectory_controller", "-c", "/controller_manager"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
    cm = subprocess.Popen(["/opt/ros/jazzy/lib/controller_manager/ros2_control_node", "--ros-args", "-r", "__ns:=/ft",
                           "--params-file", os.environ["FT_YAML"]],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "run", "controller_manager", "spawner", "force_torque_sensor_broadcaster",
                    "-c", "/ft/controller_manager", "--param-file", os.environ["FT_YAML"]],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
    return [arm, cm]


def gui():
    b = RosBridge(prof)
    core = RobotCore(b, prof, tmp / "profiles", tmp / "progs", tmp / "pol", fake=False)
    return WebService(core)


def run_for(s, secs, until=lambda: False):
    t0 = time.time()
    while time.time() - t0 < secs and not until():
        s.tick(); time.sleep(0.1)
    return time.time() - t0


def on_sensor(s):
    """The bias the broadcaster carries now (a fresh read, not the GUI's last one)."""
    got = s.core.bridge.get_node_parameters("/ft/force_torque_sensor_broadcaster", PARAMS)
    return None if got is None or any(got.get(p) is None for p in PARAMS) else [-float(got[p]) for p in PARAMS]


def back(s):
    v = on_sensor(s)
    return v is not None and all(abs(a - b) < 1e-6 for a, b in zip(v, BIAS))


def stop(procs):
    for p in procs:
        p.terminate(); p.wait(10)


def restart_stack(cm):
    stop(cm)
    return stack()


def result(name, ok, detail):
    print(f"RESULT {name} {'ok' if ok else 'FAIL'} {detail}", flush=True)


def logs(s):
    return [l for _, l in s.logs if "bias" in str(l).lower()]


ARM = Path(os.environ["ARM_URDF"]).read_text()
arm_rsp = subprocess.Popen(["/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher", "--ros-args",
                            "-p", f"robot_description:={ARM}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
# /ft/robot_description, latched (the controller manager reads it from the topic, as in drivers.launch.py)
rsp = subprocess.Popen(["/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher", "--ros-args", "-r", "__ns:=/ft",
                        "-p", f"robot_description:={URDF}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
cm = stack()
s = gui()
run_for(s, 3)
ok, msg = s.core.ft_bias.set(BIAS, "zeroed, payload kept")
print("set:", ok, msg, flush=True)

# 1) the robot stack restarts while the GUI keeps running
cm = restart_stack(cm)
print("after the stack restart the sensor has:", on_sensor(s), flush=True)
t = run_for(s, 30, lambda: back(s))
result("stack_restart_gui_running", back(s), f"{t:.1f}s sensor={on_sensor(s)} logs={logs(s)[-3:]}")

# 2) the GUI and the robot stack restart together (the launcher)
s.shutdown(); s.core.bridge.shutdown()
cm = restart_stack(cm)
s = gui()
print("after the stack restart the sensor has:", on_sensor(s), flush=True)
t = run_for(s, 30, lambda: back(s))
result("stack_and_gui_restart", back(s), f"{t:.1f}s sensor={on_sensor(s)} logs={logs(s)[-3:]}")

# 3) the GUI restarts before the stack is back (the GUI comes up first, the broadcaster later)
s.shutdown(); s.core.bridge.shutdown()
stop(cm)
s = gui()
run_for(s, 8)
cm = stack()
print("after the stack restart the sensor has:", on_sensor(s), flush=True)
t = run_for(s, 30, lambda: back(s))
result("gui_first_then_stack", back(s), f"{t:.1f}s sensor={on_sensor(s)} logs={logs(s)[-3:]}")
s.shutdown(); s.core.bridge.shutdown(); stop(cm)
print("DONE", flush=True)
