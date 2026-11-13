"""MoveIt Servo paused while no jog or policy uses it (servo_pause.py)."""
import threading
import time
from types import SimpleNamespace

from moveit_gui.jog import JogCommand
from moveit_gui.policy_executor import PolicyExecutor
from moveit_gui.servo_pause import ServoPause

from test_jog import _admittance_with_servo, _hold, core  # noqa: F401  (the fixture)


class Bridge:
    def __init__(self, service=True, fail_pause=False):
        self.service, self.fail_pause = service, fail_pause
        self.paused = {}
        self.calls = []
        self.outside = {}

    def servo_outside_commanders(self, route):
        return self.outside.get(route, 0)

    def servo_paused(self, route):
        return bool(self.paused.get(route)) if self.service else None

    def servo_pause(self, route, paused):
        self.calls.append((route, paused))
        if paused and self.fail_pause:
            return False, "refused"
        self.paused[route] = paused
        return True, ""


def wait(cond, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


def make(bridge, now):
    return ServoPause(bridge, ["position", "admittance"], log=bridge.calls.append, clock=lambda: now[0], idle_s=5.0)


def test_an_unused_route_is_paused_after_the_idle_time_and_resumed_for_a_user():
    b, now = Bridge(), [100.0]
    sp = make(b, now)
    sp.tick()
    assert b.calls == []                                     # not idle long enough yet
    now[0] += 5.0
    sp.tick()
    assert wait(lambda: b.paused == {"position": True, "admittance": True})
    assert sp.hold("admittance") == (True, "") and b.paused["admittance"] is False
    now[0] += 60.0
    sp.tick()
    time.sleep(0.05)
    assert b.paused["admittance"] is False                   # held: never paused under a user
    sp.release("admittance")
    now[0] += 4.9
    sp.tick()
    time.sleep(0.05)
    assert b.paused["admittance"] is False                   # Servo's halt has time to finish first
    now[0] += 0.2
    sp.tick()
    assert wait(lambda: b.paused["admittance"] is True)


def test_two_users_keep_it_running_until_both_are_done():
    b, now = Bridge(), [0.0]
    sp = make(b, now)
    sp.hold("admittance")
    sp.hold("admittance")
    sp.release("admittance")
    now[0] += 10
    sp.tick()
    time.sleep(0.05)
    assert not b.paused.get("admittance")
    sp.release("admittance")
    now[0] += 10
    sp.tick()
    assert wait(lambda: b.paused.get("admittance") is True)


def test_without_the_pause_service_nothing_changes():
    b, now = Bridge(service=False), [0.0]
    sp = make(b, now)
    now[0] += 60
    sp.tick()
    assert sp.hold("position") == (True, "")
    time.sleep(0.05)
    assert b.calls == []


def test_a_refused_pause_is_logged_once_and_tried_again_later():
    b, now = Bridge(fail_pause=True), [0.0]
    sp = ServoPause(b, ["position"], log=b.calls.append, clock=lambda: now[0], idle_s=5.0)
    b.paused["position"] = False
    for _ in range(3):
        now[0] += 6
        sp.tick()
        assert wait(lambda: not sp._busy)
    assert sum(1 for c in b.calls if isinstance(c, str)) == 1
    assert b.calls.count(("position", True)) == 3


def test_a_runner_outside_the_gui_keeps_servo_running_and_resumes_a_paused_one():
    b, now = Bridge(), [0.0]
    sp = make(b, now)
    b.paused = {"position": True, "admittance": True}       # paused earlier, while nobody used it
    b.outside["admittance"] = 1                              # a policy started with ros2 run
    now[0] += 1.0
    sp.tick()
    assert wait(lambda: b.paused["admittance"] is False)
    assert b.paused["position"] is True                      # only the route it commands
    assert any(isinstance(c, str) and "outside the GUI" in c for c in b.calls)
    for _ in range(3):                                       # minutes later, still running: never paused under it
        now[0] += 60.0
        sp.tick()
        assert wait(lambda: not sp._busy)
    assert b.paused["admittance"] is False
    b.outside["admittance"] = 0                              # the runner exited
    now[0] += 1.0
    sp.tick()
    time.sleep(0.05)
    assert b.paused["admittance"] is False                   # the idle time starts now
    now[0] += 5.0
    sp.tick()
    assert wait(lambda: b.paused["admittance"] is True)


def test_a_jog_resumes_a_paused_servo_and_it_is_paused_again_afterwards(core):
    _admittance_with_servo(core)
    core.bridge.servo_pause_service = True
    core.servo_pause.idle_s = 0.2
    time.sleep(0.25)
    core.update()
    assert wait(lambda: core.bridge.servo_paused_routes.get("admittance") is True)
    assert core.jog_hold_refusal() is None                   # paused still counts as ready
    tcp = core.store.active_tcp
    T0 = core.geo.current_T_base_tcp(tcp)
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "x", 1, "base"), 1.0)
    assert ok, msg
    assert core.bridge.servo_paused_routes["admittance"] is False
    _hold(core, 0.4)
    core.update()                                            # a tick while jogging: still not paused
    assert core.bridge.servo_paused_routes["admittance"] is False
    core.jog_stop()
    assert core.geo.current_T_base_tcp(tcp)[0, 3] - T0[0, 3] > 0.01   # it moved: Servo really ran
    time.sleep(0.25)
    core.update()
    assert wait(lambda: core.bridge.servo_paused_routes["admittance"] is True)


def test_a_failed_resume_refuses_the_jog_and_frees_the_robot(core):
    _admittance_with_servo(core)
    core.bridge.servo_pause_service = True
    core.bridge.servo_paused_routes["admittance"] = True
    core.bridge.servo_pause = lambda route, paused: (False, "did not answer")
    ok, msg = core.jog_start(JogCommand.parse("cartesian", "x", 1, "base"), 1.0)
    assert not ok and "could not be resumed" in msg
    assert not core.busy and not core.servo_pause.held("admittance")


def _executor(backend):
    ex = PolicyExecutor.__new__(PolicyExecutor)
    ex.library = SimpleNamespace(manifests={"p": SimpleNamespace(backend=backend, control_hz=5.0)}, running=False)
    ex.modes = SimpleNamespace(profile=SimpleNamespace(servo={"policy_route": "admittance"}))
    ex.preflight = lambda name: []
    ex.done_listeners = []
    ex.current = None
    ex._cancelled = threading.Event()
    ex._cancel_lock = threading.Lock()
    ex._cancel_generation = 0
    ex.ran = []
    ex.backend = SimpleNamespace(busy=False, run=lambda goal, hz, emit, finished: ex.ran.append(finished))
    return ex


def test_a_servo_policy_holds_servo_until_it_ends():
    b, now = Bridge(), [0.0]
    sp = make(b, now)
    b.paused["admittance"] = True
    ex = _executor("servo_admittance")
    ex.servo_pause = sp
    assert ex.start(SimpleNamespace(policy="p"))
    assert b.paused["admittance"] is False and sp.held("admittance")
    ex.ran[0](SimpleNamespace(ok=True, stop_reason="done"))
    assert not sp.held("admittance")


def test_other_policies_leave_servo_alone():
    b, now = Bridge(), [0.0]
    sp = make(b, now)
    ex = _executor("subprocess")
    ex.servo_pause = sp
    assert ex.start(SimpleNamespace(policy="p"))
    assert b.calls == [] and not sp.held("admittance")
