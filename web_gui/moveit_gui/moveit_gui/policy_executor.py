"""PolicyExecutor: the one policy runtime path, used by AUTOMATION Run and the PROGRAM `policy` step.

It owns preflight, start, feedback, cancellation, completion and failure semantics. Underneath is a
backend from policy_action.py: the subprocess runner today, the RunPolicy action later. Inference, the
force startup baseline and recording all stay in the runner.

A requirement is hard if it can block a run. "Required means required": an unavailable MoveIt Servo
fails a servo_admittance policy, it is not shown as a question mark.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional

import yaml

from .policies import PolicyLibrary, PolicyManifest, RecordingOptions
from .policy_action import RunPolicyFeedback, RunPolicyGoal, RunPolicyResult

OK, FAIL, RUNNER = "ok", "fail", "runner"          # RUNNER = only the runner can check it, at its start
WARN = "warn"                                      # shown, never blocks a run


@dataclass
class Requirement:
    label: str
    state: str
    detail: str = ""


def commissioning_path(robot_config: str) -> Path:
    p = Path(robot_config)
    return p.with_name(p.stem + ".commissioning.yaml")


def deployment_requirements(m: PolicyManifest, bridge) -> List[Requirement]:
    """Is the runner's robot config really for this robot, and has it been commissioned?

    The status is a sidecar file because the runner rejects unknown keys in its own config.
    """
    path = m.expanded_files().get("robot_config")
    if not path:
        return []
    try:
        cfg = yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError) as e:
        return [Requirement("Robot config", FAIL, str(e))]
    frames = [str(v) for section in ("topics", "servo_admittance", "sensors") for k, v in (cfg.get(section) or {}).items()
              if "frame" in k and v]
    fr3 = [f for f in frames if "fr3" in f.lower()]
    if fr3:
        return [Requirement("Robot config", FAIL, f"contains FR3 frames: {', '.join(sorted(set(fr3)))}")]
    out = [Requirement("Robot config", OK, Path(path).name)]
    side = commissioning_path(path)
    try:
        status = yaml.safe_load(side.read_text()) or {}
    except (OSError, yaml.YAMLError):
        status = {}
    if status.get("ready") is True:
        out.append(Requirement("Deployment commissioned", OK))
    else:
        missing = ", ".join(status.get("unavailable", [])[:3]) or f"no {side.name}"
        out.append(Requirement("Deployment commissioned", FAIL, f"not commissioned: {missing}"))
    if m.backend == "servo_admittance":
        sa = cfg.get("servo_admittance") or {}
        base, ee = sa.get("base_frame"), sa.get("ee_frame")
        if m.required_tcp and ee != m.required_tcp:
            out.append(Requirement("Runner pose frame", FAIL, f"robot config tracks '{ee}', policy needs "
                                                              f"'{m.required_tcp}'"))
        elif not base or not ee or bridge.lookup_transform(str(base), str(ee)) is None:
            out.append(Requirement("Runner pose frame", FAIL, f"TF {base} -> {ee} is not available to the runner"))
        else:
            out.append(Requirement("Runner pose frame", OK, str(ee)))
    return out


class PolicyExecutor:
    def __init__(self, library: PolicyLibrary, modes, bridge, backend, active_tcp: Callable[[], str],
                 frame_exists: Callable[[str], bool] = lambda _f: True):
        self.library, self.modes, self.bridge, self.backend = library, modes, bridge, backend
        self.active_tcp = active_tcp
        self.frame_exists = frame_exists
        self.feedback_listeners: List[Callable[[RunPolicyFeedback], None]] = []
        self.done_listeners: List[Callable[[RunPolicyResult], None]] = []
        self.current: Optional[str] = None
        self._cancelled = threading.Event()
        # (mode, {param: value}) -> (ok | None, detail): live controller gains vs. the trained ones (RobotCore)
        self.gains_check: Optional[Callable[[str, dict], tuple]] = None
        self.range_check: Optional[Callable[[PolicyManifest], List[str]]] = None

    # ------------------------------------------------------------ library view
    @property
    def manifests(self):
        return self.library.manifests

    @property
    def errors(self):
        return self.library.errors

    @property
    def directory(self):
        return self.library.directory

    def reload(self):
        self.library.reload()

    @property
    def supports_feedback(self) -> bool:
        return self.backend.supports_feedback

    @property
    def busy(self) -> bool:
        return bool(self.backend.busy) or self.library.running

    @property
    def running(self) -> bool:
        return self.busy

    # --------------------------------------------------------------- preflight
    def requirements(self, name: str) -> List[Requirement]:
        m = self.manifests.get(name)
        if m is None:
            return [Requirement("Policy", FAIL, f"unknown policy '{name}'")]
        return self._static_requirements(m) + self._live_requirements(m)

    def _static_requirements(self, m: PolicyManifest) -> List[Requirement]:
        out: List[Requirement] = []
        files = m.expanded_files()
        for key, label in (("model", "Model"), ("normalization", "Normalizer"), ("training_config", "Training config")):
            if key in files:
                found = Path(files[key]).exists()
                out.append(Requirement(label, OK if found else FAIL, Path(files[key]).name if found else "file not found"))
        for msg in m.contract_problems():
            out.append(Requirement("Runner contract", FAIL, msg))
        if m.required_mode not in self.modes.profile.modes:
            out.append(Requirement("Control mode", FAIL, f"'{m.required_mode}' is not a mode of this robot"))
        out += deployment_requirements(m, self.bridge)
        if "robot_config" in files:
            got = self.library.ft_axes_in_use()
            if got:
                owner, res = got
                out.append(Requirement("F/T axes", OK, f"Match simulation of {owner or 'the settings in use'} "
                                                       f"({res.get('at', '')})"))
            else:
                out.append(Requirement("F/T axes", WARN, "no Match simulation result in this profile: the robot "
                                                         "config's own F/T settings are used"))
        if m.required_tcp:
            tcp = self.active_tcp()
            out.append(Requirement(f"TCP {m.required_tcp}", OK if tcp == m.required_tcp else FAIL,
                                   "" if tcp == m.required_tcp else f"active TCP is {tcp}"))
        for f in m.required_frames:
            out.append(Requirement(f"Frame {f}", OK if self.frame_exists(f) else FAIL,
                                   "" if self.frame_exists(f) else "not defined in SETUP"))
        return out

    def _live_requirements(self, m: PolicyManifest) -> List[Requirement]:
        out: List[Requirement] = []
        snap = self.bridge.snapshot()
        if m.requires_ft:
            live = bool(snap.connected.get("F/T"))
            out.append(Requirement("ATI Gamma", OK if live else FAIL, "live" if live else "F/T topic is silent"))
        mode = self.modes.profile.modes.get(m.required_mode)
        label = mode.label if mode else m.required_mode
        current = self.modes.current_mode()
        out.append(Requirement(label, OK if current == m.required_mode else FAIL,
                               "active" if current == m.required_mode else f"robot is in '{current}'"))
        if m.backend == "servo_admittance":
            route = self.modes.profile.servo.get("policy_route", "admittance")
            ready = bool(self.bridge.servo_ready(route))
            out.append(Requirement("MoveIt Servo", OK if ready else FAIL, "ready" if ready else "not available"))
        if m.controller_gains and self.gains_check is not None:
            ok, detail = self.gains_check(m.required_mode, m.controller_gains)
            out.append(Requirement("Controller gains", OK if ok else FAIL,
                                   detail if ok is not None else f"could not read them: {detail}"))
        if self.range_check is not None:
            try:
                warnings = self.range_check(m)
            except Exception as e:  # range warnings never make the readiness page fail
                warnings = [f"range check unavailable: {e}"]
            if warnings:
                out.append(Requirement("Policy ranges", WARN, "; ".join(warnings)))
        if not m.command:
            out.append(Requirement("Observation contract", RUNNER, "validated by the runner at start"))
        return out

    def preflight(self, name: str) -> List[str]:
        return [f"{r.label}: {r.detail}" if r.detail else r.label for r in self.requirements(name) if r.state == FAIL]

    def program_problems(self, m: PolicyManifest) -> List[str]:
        """What a program can already fail on before it runs; the mode may be set by an earlier step."""
        return [f"{r.label}: {r.detail}" if r.detail else r.label
                for r in self._static_requirements(m) if r.state == FAIL] + m.preflight(self.modes.profile.modes)

    # ----------------------------------------------------------------- runtime
    def goal(self, name: str, duration: Optional[float] = None,
             recording: Optional[RecordingOptions] = None) -> RunPolicyGoal:
        m = self.manifests[name]
        return RunPolicyGoal(policy=name, backend=m.backend,
                             duration=duration if duration is not None else m.duration,
                             robot_config=m.expanded_files().get("robot_config", ""),
                             recording=recording or RecordingOptions())

    def _emit(self, fb: RunPolicyFeedback):
        for fn in list(self.feedback_listeners):
            fn(fb)

    def prepare(self, goal: RunPolicyGoal, on_done: Callable[[RunPolicyResult], None]) -> bool:
        problems = self.preflight(goal.policy)
        if problems:
            on_done(RunPolicyResult(False, "preflight failed: " + "; ".join(problems)))
            return False
        self.backend.prepare(goal, self._emit, on_done)
        return True

    def start(self, goal: RunPolicyGoal, on_done: Optional[Callable[[RunPolicyResult], None]] = None) -> bool:
        """Non-blocking. Every run goes through here, with a fresh preflight."""
        def finished(res: RunPolicyResult):
            self.current = None
            for fn in list(self.done_listeners):
                fn(res)
            if on_done:
                on_done(res)
        problems = self.preflight(goal.policy)
        if problems or self.busy:
            finished(RunPolicyResult(False, "preflight failed: " + "; ".join(problems) if problems
                                     else "a policy is already running"))
            return False
        route = self.modes.profile.servo.get("policy_route", "admittance") \
            if self.manifests[goal.policy].backend == "servo_admittance" else ""
        sp = getattr(self, "servo_pause", None)
        if route and sp is not None:
            ok, msg = sp.hold(route)                     # the runner talks to Servo itself: resume it first
            if not ok:
                finished(RunPolicyResult(False, msg))
                return False
            done = finished

            def finished(res):                           # noqa: F811  (released when the policy ends)
                sp.release(route)
                done(res)
        self._cancelled.clear()
        self.current = goal.policy
        self.started = (goal.policy, time.time())
        hz = self.manifests[goal.policy].control_hz or 5.0
        self.backend.run(goal, hz, self._emit, finished)
        return True

    def run(self, name: str, modes=None, duration=None, stop_event: Optional[threading.Event] = None, log=print,
            recording: Optional[RecordingOptions] = None) -> bool:
        """Blocking form of start(), for the program runner's worker thread."""
        if name not in self.manifests:
            log(f"unknown policy '{name}'")
            return False
        done = threading.Event()
        result: List[RunPolicyResult] = []

        def finished(res):
            result.append(res)
            done.set()
        if not self.start(self.goal(name, duration, recording), finished):
            log(f"policy '{name}': {result[0].stop_reason if result else 'not started'}")
            return False
        log(f"policy '{name}' started")
        while not done.wait(0.05):
            if stop_event is not None and stop_event.is_set():
                self.cancel()
                stop_event = None
        log(f"policy '{name}': {result[0].stop_reason}")
        return result[0].success

    def cancel(self):
        self._cancelled.set()
        self.backend.cancel()

    stop = cancel
