import pytest

from ur7e_safety.watchdog_logic import WrenchWatchdog, is_zero


def test_is_zero():
    assert is_zero([0, 0.0, -0.0])
    assert not is_zero([0, 0, 0, 0, 0, 0.01])


def test_never_trips_before_first_message():
    w = WrenchWatchdog(0.3)
    assert not w.poll(100.0)


def test_normal_traffic_does_not_trip():
    w = WrenchWatchdog(0.3)
    t = 0.0
    for _ in range(100):
        w.on_message(t, True)
        t += 0.02
        assert not w.poll(t)


def test_trips_once_after_timeout_when_nonzero():
    w = WrenchWatchdog(0.3)
    w.on_message(1.0, True)
    assert not w.poll(1.29)
    assert w.poll(1.31)
    assert not w.poll(1.5)
    assert not w.poll(10.0)


def test_zero_last_message_never_trips():
    w = WrenchWatchdog(0.3)
    w.on_message(1.0, True)
    w.on_message(1.02, False)
    assert not w.poll(5.0)


def test_rearms_after_new_message():
    w = WrenchWatchdog(0.3)
    w.on_message(0.0, True)
    assert w.poll(0.5)
    w.on_message(1.0, True)
    assert not w.poll(1.1)
    assert w.poll(1.4)


def test_bad_timeout():
    with pytest.raises(ValueError):
        WrenchWatchdog(0.0)
