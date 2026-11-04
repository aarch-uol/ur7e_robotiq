"""Pure timing logic of the wrench watchdog (no ROS imports, unit-testable)."""


def is_zero(values, eps=0.0):
    """True if every component has magnitude <= eps."""
    return all(abs(v) <= eps for v in values)


class WrenchWatchdog:
    """Tracks one wrench topic.

    on_message(now, nonzero) is called for every received message. poll(now)
    returns True exactly once per timeout event: when the last message was
    non-zero and nothing arrived for timeout_s. The caller then publishes the
    zero wrench (repeatedly, as it sees fit). A new message re-arms the watchdog.
    """

    def __init__(self, timeout_s=0.3):
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be > 0")
        self.timeout_s = timeout_s
        self._last_time = None
        self._last_nonzero = False
        self._tripped = False

    def on_message(self, now, nonzero):
        self._last_time = now
        self._last_nonzero = bool(nonzero)
        self._tripped = False

    def poll(self, now):
        if self._tripped or self._last_time is None or not self._last_nonzero:
            return False
        if now - self._last_time > self.timeout_s:
            self._tripped = True
            return True
        return False
