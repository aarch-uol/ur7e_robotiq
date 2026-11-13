"""Where the web GUI's time goes, on the robot PC (no external profiler needed).

TickStats: how long each state update takes, per part; logged every LOG_EVERY_S while updates run late.
sample(): a few seconds of stack samples of every Python thread; the busiest places, as log lines.
"""
import collections
import sys
import threading
import time
import traceback
from typing import Callable, Dict, List

LOG_EVERY_S = 30.0


class TickStats:
    def __init__(self, period_s: float, log: Callable[[str], None]):
        self.period, self.log = period_s, log
        self._t = collections.defaultdict(list)
        self._n = 0
        self._since = time.monotonic()
        self._lock = threading.Lock()

    def add(self, parts: Dict[str, float]):
        with self._lock:
            self._n += 1
            for k, v in parts.items():
                self._t[k].append(v)
            now = time.monotonic()
            if now - self._since < LOG_EVERY_S:
                return
            dt, n, t = now - self._since, self._n, self._t
            self._t, self._n, self._since = collections.defaultdict(list), 0, now
        rate = n / dt
        if rate >= 0.8 / self.period:                       # on time: say nothing
            return
        parts = ", ".join(f"{k} {1000 * sum(v) / len(v):.0f}/{1000 * max(v):.0f}" for k, v in t.items())
        self.log(f"web: state updates late: {rate:.1f}/s (want {1 / self.period:.0f}); ms avg/max: {parts}")


def sample(seconds: float = 10.0, hz: float = 100.0, top: int = 6) -> List[str]:
    """Stack samples of every thread. Per thread: how often it was seen running Python code (not waiting) and
    its most common innermost places in this package (with the frame that called in)."""
    me = threading.get_ident()
    names = {t.ident: t.name for t in threading.enumerate()}
    seen = collections.Counter()
    places: Dict[int, collections.Counter] = collections.defaultdict(collections.Counter)
    n = 0
    t_end = time.monotonic() + seconds
    waiting = ("wait", "sleep", "select", "poll", "accept", "recv", "readinto", "spin_once", "_wait_for_tstate_lock",
               "acquire", "get", "readline")
    while time.monotonic() < t_end:
        n += 1
        for ident, frame in sys._current_frames().items():
            if ident == me:
                continue
            stack = traceback.extract_stack(frame)
            if not stack or stack[-1].name in waiting:
                continue
            seen[ident] += 1
            ours = [f for f in stack if "moveit_gui" in f.filename]
            leaf = stack[-1]
            where = f"{leaf.filename.rsplit('/', 1)[-1]}:{leaf.lineno} {leaf.name}"
            caller = ours[-1] if ours and ours[-1] is not leaf else (stack[-2] if len(stack) > 1 else None)
            if caller is not None:
                where += f" <- {caller.filename.rsplit('/', 1)[-1]}:{caller.lineno} {caller.name}"
            places[ident][where] += 1
        time.sleep(1.0 / hz)
    out = [f"web profile: {n} samples in {seconds:.1f} s (share of samples a thread was running Python)"]
    for ident, k in seen.most_common():
        out.append(f"  thread {names.get(ident, ident)}: {100 * k / n:.0f}%")
        for where, c in places[ident].most_common(top):
            out.append(f"      {100 * c / n:4.0f}%  {where}")
    return out
