"""F/T limit: soft warns, hard stops the GUI's motion once per event and re-arms when calm."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.ft_guard import FtGuard, FtGuardError, check_config  # noqa: E402
from test_web import env, wait  # noqa: E402,F401  (fixture reuse)
from test_web_lease import _slow_move  # noqa: E402


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def _guard(**cfg):
    stops, logs, c = [], [], Clock()
    g = FtGuard(None, on_hard=stops.append, log=logs.append, clock=c, threaded=False)
    g.save({"enabled": True, "soft_f": 20, "hard_f": 40, "soft_t": 0, "hard_t": 3, **cfg})
    return g, stops, logs, c


def F(fz, tz=0.0):
    return [0.0, 0.0, fz, 0.0, 0.0, tz]


def test_config_checks():
    with pytest.raises(FtGuardError, match="below the hard"):
        check_config({"enabled": True, "soft_f": 40, "hard_f": 30})
    with pytest.raises(FtGuardError):
        check_config({"enabled": True, "hard_f": -1})
    with pytest.raises(FtGuardError, match="at least one"):
        check_config({"enabled": True})
    assert check_config({})["enabled"] is False


def test_soft_warns_hard_stops_once_then_rearms():
    g, stops, _, c = _guard()
    for _ in range(10):
        g.feed(F(25))
    assert g.view()["level"] == 1 and not stops
    for _ in range(10):
        g.feed(F(45))
    assert g.view()["level"] == 2 and len(stops) == 1 and "40 N" in stops[0]
    for _ in range(10):
        g.feed(F(50))                                  # still over: no second stop
    assert len(stops) == 1 and g.tripped
    for _ in range(10):
        g.feed(F(5))
    c.t += 0.6
    g.feed(F(5))
    assert not g.tripped
    for _ in range(10):
        g.feed(F(45))
    assert len(stops) == 2                             # re-armed: a new event stops again


def test_one_noisy_sample_does_not_trip():
    g, stops, _, _ = _guard()
    for _ in range(10):
        g.feed(F(10))
    g.feed(F(120))                                     # average of 5: (4*10 + 120) / 5 = 32 < 40
    assert not stops


def test_torque_limit_and_off():
    g, stops, _, _ = _guard()
    for _ in range(6):
        g.feed(F(0, 3.5))
    assert stops and "Nm" in stops[0]
    g2, s2, _, _ = _guard(enabled=False)
    for _ in range(6):
        g2.feed(F(400))
    assert not s2 and g2.view()["level"] == 0


def test_freedrive_only_warns():
    g, stops, logs, _ = _guard()
    g.freedrive = lambda: True
    for _ in range(6):
        g.feed(F(60))
    assert not stops and any("freedrive" in m for m in logs) and g.view()["last_trip"]["stopped"] is False


def test_hard_limit_stops_a_gui_move(env):
    s, bridge, _, _ = env
    a = s.take_lease("", "alice")["token"]
    s.command(a, "ft_guard_save", {"enabled": True, "hard_f": 40})
    assert s.state()["ft_guard"]["enabled"]
    _slow_move(s, bridge, a)
    n = bridge.calls.count(("cancel",))
    for _ in range(6):
        s.core.ft_guard.feed(F(55))
    assert wait(lambda: bridge.calls.count(("cancel",)) > n)
    assert wait(lambda: not s.core.busy)
    assert s.state()["ft_guard"]["last_trip"]["stopped"] is True
    assert any("F/T HARD LIMIT" in m for m in [l for _, l in s.logs] + bridge.drain_logs())


def test_saving_needs_admin(env):
    s, _, _, _ = env
    a = s.take_lease("", "op", role="operator")["token"]
    from moveit_gui.web.service import CommandError
    with pytest.raises(CommandError, match="Admin"):
        s.command(a, "ft_guard_save", {"enabled": True, "hard_f": 40}, role="operator")
