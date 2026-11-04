"""Interface boundary between the GUI and the existing policy runner.

The GUI orchestrates and visualises; the runner (mujoco_multiagent_rl.policy_runner in
~/franka_ros2) owns model loading, normalisation, observations, sim2real, F/T startup bias,
rate limiting and safety. Nothing of that is reimplemented here.

Target design: a ROS action wrapping the existing runner, conceptually

    RunPolicy.action
      goal:      policy, backend, duration, robot_config, recording
      feedback:  state, elapsed, step, action, wrench, compensated_wrench,
                 baseline_count/baseline_total, pose_fresh, ready
      result:    success, stop_reason, log_dir

The Python dataclasses below mirror that message (docs/RunPolicy.action) so the UI can be built and
tested now. /policy_runner/status is NOT a feedback source: the runner declares the publisher
(node.py) but never publishes on it. Two backends exist today, both used only through PolicyExecutor:
  MockPolicyRunner        fake/demo mode: simulates baseline collection and steps
  SubprocessPolicyRunner  real mode: the interim subprocess launcher (PolicyLibrary); it can
                          start/stop the runner but receives NO feedback (no baseline, no actions)
A RosActionPolicyRunner will replace the subprocess one once RunPolicy.action exists.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from .policies import RecordingOptions

STATES = ("idle", "preparing", "baseline", "ready", "running", "stopping", "done", "failed")


@dataclass
class RunPolicyGoal:
    policy: str
    backend: str = ""
    duration: Optional[float] = None
    robot_config: str = ""
    recording: RecordingOptions = field(default_factory=RecordingOptions)


@dataclass
class RunPolicyFeedback:
    state: str = "idle"
    elapsed: float = 0.0
    step: int = 0
    action: Optional[List[float]] = None
    wrench: Optional[List[float]] = None
    compensated_wrench: Optional[List[float]] = None
    baseline_count: int = 0
    baseline_total: int = 0
    pose_fresh: Optional[bool] = None
    observation_ok: Optional[bool] = None
    normalizer_ok: Optional[bool] = None
    message: str = ""


@dataclass
class RunPolicyResult:
    success: bool
    stop_reason: str
    log_dir: str = ""


FeedbackCb = Callable[[RunPolicyFeedback], None]
DoneCb = Callable[[RunPolicyResult], None]


class MockPolicyRunner:
    """Demo stand-in for the RunPolicy action. All numbers are invented."""
    supports_feedback = True

    def __init__(self, baseline_total: int = 50, baseline_hz: float = 25.0):
        self.baseline_total = baseline_total
        self.baseline_hz = baseline_hz
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.state = "idle"

    @property
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def prepare(self, goal: RunPolicyGoal, on_feedback: FeedbackCb, on_done: DoneCb):
        self._start(self._prepare, goal, on_feedback, on_done)

    def run(self, goal: RunPolicyGoal, control_hz: float, on_feedback: FeedbackCb, on_done: DoneCb):
        self._start(self._run, goal, on_feedback, on_done, control_hz)

    def cancel(self):
        self._stop.set()

    def _start(self, fn, *args):
        if self.busy:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=fn, args=args, daemon=True)
        self._thread.start()

    def _prepare(self, goal, on_feedback, on_done):
        fb = RunPolicyFeedback(state="baseline", baseline_total=self.baseline_total, pose_fresh=True,
                               observation_ok=True, normalizer_ok=True)
        for i in range(1, self.baseline_total + 1):
            if self._stop.is_set():
                self.state = "idle"
                on_done(RunPolicyResult(False, "cancelled during baseline"))
                return
            fb.baseline_count = i
            fb.wrench = [0.21 + 0.02 * math.sin(i), -0.08, 9.31 + 0.03 * math.cos(i), 0.01, -0.02, 0.0]
            on_feedback(fb)
            time.sleep(1.0 / self.baseline_hz)
        self.state = "ready"
        fb.state = "ready"
        fb.message = "Force startup baseline locked"
        on_feedback(fb)
        on_done(RunPolicyResult(True, "ready"))

    def _run(self, goal, on_feedback, on_done, control_hz):
        self.state = "running"
        t0, step = time.monotonic(), 0
        duration = goal.duration or 8.0
        while time.monotonic() - t0 < duration:
            if self._stop.is_set():
                self.state = "done"
                on_done(RunPolicyResult(False, "stopped by operator", "/tmp/fake_policy_log"))
                return
            step += 1
            el = time.monotonic() - t0
            on_feedback(RunPolicyFeedback(state="running", elapsed=el, step=step,
                                          action=[0.5 * math.sin(el), 2.0, 0.001, -0.002, 0.0],
                                          wrench=[0.4, -0.1, 5.0 + math.sin(el), 0.0, 0.05, 0.0],
                                          compensated_wrench=[0.2, -0.02, -4.3 + math.sin(el), 0.0, 0.07, 0.0],
                                          baseline_count=self.baseline_total, baseline_total=self.baseline_total,
                                          pose_fresh=True))
            time.sleep(1.0 / max(control_hz, 1.0))
        self.state = "done"
        on_done(RunPolicyResult(True, "duration reached", "/tmp/fake_policy_log"))


class SubprocessPolicyRunner:
    """Interim adapter over PolicyLibrary (subprocess). Start/stop only; no runner feedback."""
    supports_feedback = False

    def __init__(self, library, modes):
        self.library, self.modes = library, modes
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def prepare(self, goal: RunPolicyGoal, on_feedback: FeedbackCb, on_done: DoneCb):
        on_feedback(RunPolicyFeedback(state="ready", message="Preflight only: the runner collects its force startup "
                                      "baseline when it starts and reports no progress until RunPolicy exists"))
        on_done(RunPolicyResult(True, "preflight only"))

    def run(self, goal: RunPolicyGoal, control_hz: float, on_feedback: FeedbackCb, on_done: DoneCb):
        if self.busy:
            return
        self._stop.clear()

        def work():
            on_feedback(RunPolicyFeedback(state="running", message="running as subprocess (no feedback)"))
            msgs = []
            ok = self.library.run(goal.policy, self.modes, duration=goal.duration, stop_event=self._stop,
                                  log=msgs.append, recording=goal.recording)
            on_done(RunPolicyResult(ok, msgs[-1] if msgs else ("finished" if ok else "failed")))
        self._thread = threading.Thread(target=work, daemon=True)
        self._thread.start()

    def cancel(self):
        self._stop.set()
        self.library.stop()
