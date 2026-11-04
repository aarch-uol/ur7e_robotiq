"""Robot lifecycle as the GUI sees it: power, brakes, the External Control program, and what happens to a running
motion when the robot stops being able to execute it. These are the sequences that went wrong on the UR7e:

* after a power cycle the driver never reported the program as stopped, so a MoveJ was sent, held at speed 0,
  and the GUI showed MOVING until STOP (and would have moved on Play);
* jogs were started while the program was not running;
* STOP did not reach the trajectory (MoveIt 2.12 ignores a cancel of /execute_trajectory).
"""
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
HOME = [0, -90, 90, -90, -90, 0]
AWAY = [30, -80, 80, -90, -90, 10]


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=5.0, move_time_scale=1.0)     # slow: moves last long enough
    bridge.servo_available = True
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a", "Ana")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def tick_until(s, cond, timeout=10.0):
    """The server tick plus the browser's heartbeat (the lease ends after 2.5 s without one)."""
    return wait(lambda: (s.heartbeat("a") or True) and (s.tick() or True) and cond(), timeout)


def logs(s):
    s.tick()
    return [line for _, line in s.logs]


def state(s):
    s.tick()
    st = s.state()
    return st["robot_mode"], st["program_running"], st["idle"]


def power_cycle(s, local=True):
    """Power off, power on, release brakes: the buttons on OPERATE, as on the robot today."""
    s.command("a", "power_off", {})
    assert tick_until(s, lambda: s.core.last_snap.robot_mode == "POWER_OFF")
    s.command("a", "power_on", {"seen": True}, local=local)
    assert tick_until(s, lambda: s.core.last_snap.robot_mode == "IDLE")
    s.command("a", "brake_release", {"seen": True}, local=local)
    assert tick_until(s, lambda: s.core.last_snap.robot_mode == "RUNNING")


def play(s):
    name = s.command("a", "robot_loaded_program", {})["program"]
    s.command("a", "robot_play", {"program": name})
    assert tick_until(s, lambda: s.core.last_snap.program_running is True)


# ------------------------------------------------------------------ power cycle -> Play -> motion
def test_after_a_power_cycle_motion_and_jogs_wait_for_play(s):
    assert state(s) == ("RUNNING", True, True)
    power_cycle(s)
    mode, prog, idle = state(s)
    assert (mode, prog, idle) == ("RUNNING", False, True)          # brakes off, program NOT running
    assert "Play program" in s.state()["motion_refusal"]
    with pytest.raises(CommandError, match="Play program"):
        s.command("a", "move_joints", {"q_deg": AWAY})
    with pytest.raises(CommandError, match="Play program"):
        s.command("a", "jog_start", {"kind": "cartesian", "axis": "x", "sign": 1})
    assert s.state()["jog"]["hold_refusal"] and "Play program" in s.state()["jog"]["hold_refusal"]
    play(s)
    assert s.state()["motion_refusal"] is None
    job = s.command("a", "move_joints", {"q_deg": AWAY})["job"]
    assert tick_until(s, lambda: any(r["id"] == job for r in s.results), 30)
    res = next(r for r in s.results if r["id"] == job)
    assert res["ok"], res["msg"]
    assert abs(s.state()["joints_deg"][0] - AWAY[0]) < 0.5


def test_power_off_is_seen_as_program_stopped_even_if_the_driver_says_nothing(s):
    b = s.core.bridge
    b.robot_mode = "POWER_OFF"           # only the robot mode changes; the program flag is left as it was
    b.program_running = True
    s.tick()
    # the real bridge clears program_running when the mode leaves RUNNING (test_ros_bridge_static); the core
    # refuses on the robot mode anyway
    assert "power on" in s.core.motion_refusal()


# ------------------------------------------------------------------ watchdog: a running motion when the robot stops
def test_a_move_ends_at_once_when_the_program_stops(s):
    s.command("a", "move_joints", {"q_deg": AWAY})
    assert tick_until(s, lambda: s.core.busy)
    s.core.bridge.program_running = False                        # program stopped on the pendant mid-move
    assert tick_until(s, lambda: s.core.idle(), 5)                # not MOVING until someone presses STOP
    assert any("STOPPED: the robot cannot execute" in line for line in logs(s))
    assert ("cancel",) in s.core.bridge.calls                     # the goal was cancelled, so Play cannot resume it
    assert tick_until(s, lambda: s.results)
    assert "motion stopped: the External Control program is not running" in s.results[-1]["msg"]   # the toast says why


def test_a_move_ends_when_the_arm_powers_off(s):
    s.command("a", "move_joints", {"q_deg": AWAY})
    assert tick_until(s, lambda: s.core.busy)
    s.command("a", "power_off", {})
    assert tick_until(s, lambda: s.core.idle(), 5)
    assert s.core.bridge.robot_mode == "POWER_OFF"


def test_a_jog_ends_when_a_protective_stop_comes(s):
    s.command("a", "jog_start", {"kind": "cartesian", "axis": "x", "sign": 1})
    assert tick_until(s, lambda: s.core.jogging)
    s.core.bridge.safety_mode = "PROTECTIVE_STOP"
    assert tick_until(s, lambda: not s.core.jogging, 5)
    assert any("PROTECTIVE_STOP" in line for line in logs(s))


def test_a_program_run_pauses_when_the_program_stops_and_stop_ends_it(s):
    s.command("a", "program_new", {})
    s.command("a", "program_insert", {"type": "wait", "params": {"seconds": 5}})
    s.command("a", "program_run", {})
    assert tick_until(s, lambda: s.core.runner.state.name != "IDLE")
    s.core.bridge.program_running = False
    assert tick_until(s, lambda: s.core.runner.state.name == "PAUSED", 5)      # paused, not ended: it can resume
    s.stop("test")
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE", 5)


def test_the_watchdog_fires_once_and_leaves_an_idle_gui_alone(s):
    s.core.bridge.program_running = False
    for _ in range(5):
        s.tick()
    assert not any("STOPPED: the robot cannot execute" in line for line in logs(s))   # nothing was running
    s.core.bridge.program_running = True
    s.command("a", "move_joints", {"q_deg": AWAY})
    assert tick_until(s, lambda: s.core.busy)
    s.core.bridge.program_running = False
    assert tick_until(s, lambda: s.core.idle(), 5)
    for _ in range(5):
        s.tick()
    assert sum("STOPPED: the robot cannot execute" in line for line in logs(s)) == 1


# ------------------------------------------------------------------ STOP and a held goal
def test_stop_ends_a_move_that_is_held_at_speed_zero(s):
    s.command("a", "move_joints", {"q_deg": AWAY})
    assert tick_until(s, lambda: s.core.busy)
    q0 = list(s.core.bridge._q)
    s.stop("test")
    assert tick_until(s, lambda: s.core.idle(), 5)
    s.core.bridge.program_running = True                          # Play pressed later: nothing resumes
    time.sleep(0.3)
    assert max(abs(a - b) for a, b in zip(q0, s.core.bridge._q)) < 0.05


def test_play_refuses_while_moving_and_clears_pending_goals(s):
    s.command("a", "move_joints", {"q_deg": AWAY})
    assert tick_until(s, lambda: s.core.busy)
    with pytest.raises(CommandError):
        s.command("a", "robot_play", {"program": s.core.bridge.pendant_program})
    s.stop("test")
    assert tick_until(s, lambda: s.core.idle(), 5)
