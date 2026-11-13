"""Pause MoveIt Servo while nobody uses it.

gui.launch.py starts two Servo nodes (Position jog and Admittance jog/policies), and both run their loop, collision
checks and status at full rate even when nothing is jogged: about half a Jetson core together. Servo's own
~/pause_servo (std_srvs/SetBool) stops the collision checks, the status and the output; unpausing resets Servo's
smoothing to the arm's current state, so the first command after it starts from where the arm is.

hold(route) unpauses a route before a jog or a servo_admittance policy uses it; release(route) when done. A route
nobody holds is paused IDLE_S after its last use (long after Servo's own halt has finished). A Servo without the
pause service (older stack, simulated robot) is left as it is: hold() then does nothing.

A policy or script run without the GUI publishes Servo commands itself. While any node other than this GUI publishes
on a route's command topics, the route counts as in use: it is never paused, and a paused one is resumed as soon as
that publisher appears (before its first command, since publishers show up when they are created).
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Callable, Iterable, Tuple

IDLE_S = 60.0                  # paused after a minute without a jog: never between the jogs of one task
OUTSIDE_CHECK_S = 1.0          # how often the command topics are checked for publishers other than the GUI


class ServoPause:
    def __init__(self, bridge, routes: Iterable[str], log: Callable[[str], None] = print,
                 clock: Callable[[], float] = time.monotonic, idle_s: float = IDLE_S):
        self.b, self.log, self.clock, self.idle_s = bridge, log, clock, idle_s
        self.routes = [r for r in routes if r]
        self.users = collections.Counter()
        self.last_use = {r: clock() for r in self.routes}
        self._lock = threading.Lock()                   # one pause/unpause decision at a time
        self._busy = False
        self._failed = set()                            # routes whose pause failed: logged once
        self.outside = {r: 0 for r in self.routes}       # publishers other than the GUI on its command topics
        self._outside_at = None

    def _paused(self, route):
        f = getattr(self.b, "servo_paused", None)
        return f(route) if f is not None else None

    def hold(self, route: str) -> Tuple[bool, str]:
        """Unpause `route` for a user (blocking, a service call). (False, why) if Servo refused."""
        if not route:
            return True, ""
        with self._lock:
            self.users[route] += 1
            self.last_use[route] = self.clock()
            if self._paused(route) is None:             # no pause service: nothing to do
                return True, ""
            ok, msg = self.b.servo_pause(route, False)
            if not ok:
                self.users[route] -= 1
                return False, f"MoveIt Servo could not be resumed: {msg}"
        return True, ""

    def release(self, route: str):
        if not route:
            return
        with self._lock:
            self.users[route] = max(0, self.users[route] - 1)
            self.last_use[route] = self.clock()

    def held(self, route: str) -> bool:
        return self.users[route] > 0

    def _outside(self, route):
        f = getattr(self.b, "servo_outside_commanders", None)
        n = f(route) if f is not None else None
        return n or 0

    def tick(self):
        """Call often (the UI rate): pauses the idle routes and resumes the ones commanded from outside the GUI,
        in the background (service calls)."""
        if self._busy:
            return
        now = self.clock()
        if self._outside_at is None or now - self._outside_at >= OUTSIDE_CHECK_S:
            self._outside_at = now
            for r in self.routes:
                n = self._outside(r)
                if n and not self.outside[r]:
                    self.log(f"MoveIt Servo ({r}): commanded from outside the GUI, kept running")
                self.outside[r] = n
        todo = []
        for r in self.routes:
            paused = self._paused(r)
            if self.outside[r]:
                self.last_use[r] = now
                if paused is True:
                    todo.append((r, False))
            elif not self.users[r] and now - self.last_use.get(r, now) >= self.idle_s and paused is False:
                todo.append((r, True))
        if not todo:
            return
        self._busy = True
        threading.Thread(target=self._apply, args=(todo,), daemon=True, name="servo-pause").start()

    def _apply(self, todo):
        try:
            for r, pause in todo:
                with self._lock:
                    if pause and (self.users[r] or self.outside[r]
                                  or self.clock() - self.last_use.get(r, 0.0) < self.idle_s):
                        continue                         # someone took it meanwhile
                    ok, msg = self.b.servo_pause(r, pause)
                    if ok:
                        self._failed.discard(r)
                    else:
                        self.last_use[r] = self.clock()  # try again after another idle period
                        if r not in self._failed:
                            self._failed.add(r)
                            what = "paused" if pause else "resumed"
                            self.log(f"MoveIt Servo ({r}) could not be {what}: {msg}")
        finally:
            self._busy = False
