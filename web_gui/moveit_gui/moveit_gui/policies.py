"""Policy library: a policy is a *command source*, not a controller.

Each policy is described by a small manifest in policies/<name>.yaml:

    name: scrape_ppo_v4
    algorithm: PPO
    required_mode: admittance         # control mode that must be ACTIVE (never switched silently)
    backend: servo_admittance         # passed to the runner as --robot-controller
    required_tcp: spatula_tip         # the ACTIVE TCP must be this one
    required_frames: [vial_rack]      # only if the policy really depends on a saved frame
    control_hz: 5.0
    requires_ft: true
    model: ~/policies/v4/model.zip
    normalization: ~/policies/v4/vec_normalize.pkl
    training_config: ~/policies/v4/config.yaml
    robot_config: ~/ur7e_ws/.../ur7e_policy_deployment.yaml
    entrypoint: run_policy            # or run_scraping_policy
    controller_gains:                 # optional: the gains it was trained with (checked before a run)
      admittance.stiffness: [250, 200, 180, 20, 20, 20]
    duration: 8.0

The command line is generated to match the real runner
(mujoco_multiagent_rl.policy_runner.ros2.node / scraping_policy). The runner reads its arguments
with parse_known_args, so anything else on the line (for example `-p model_path:=...`) would be
ignored and the runner would fall back to selecting a checkpoint itself. A hand-written `command`
is still accepted for other tools, but ROS-parameter model paths are refused and a declared backend
must be on the line.

The GUI does not load models. PolicyExecutor (policy_executor.py) owns the runtime path; the plan is
to replace the subprocess underneath it with a RunPolicy ROS action (docs/RunPolicy.action).
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml


BACKEND_MODE = {"impedance": "impedance", "servo_admittance": "admittance"}   # backend -> control mode
ENTRYPOINTS = {"run_policy": "run_policy.py", "run_scraping_policy": "run_scraping_policy.py"}
FAKE_RUNNER_ARGS = ("model_path:=", "vecnorm_path:=", "model:=", "normalization:=")


@dataclass
class PolicyManifest:
    name: str
    command: List[str]
    required_mode: str
    description: str = ""
    cwd: Optional[str] = None
    files: Dict[str, str] = field(default_factory=dict)
    env: Dict[str, str] = field(default_factory=dict)
    duration: Optional[float] = None
    stop_signal: str = "SIGINT"
    backend: str = ""
    algorithm: str = ""
    control_hz: Optional[float] = None
    requires_ft: bool = False
    required_tcp: str = ""
    required_frames: List[str] = field(default_factory=list)
    entrypoint: str = "run_policy"
    package: str = "my_robot_policy"
    # Controller parameters the policy was trained with ({parameter: value}); checked against the live controller
    controller_gains: Dict[str, object] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> "PolicyManifest":
        d = yaml.safe_load(path.read_text()) or {}
        cmd = d.get("command", [])
        if isinstance(cmd, str):
            raise ValueError(f"{path.name}: 'command' must be a list (no shell strings)")
        files = dict(d.get("files", {}) or {})
        if not cmd and not (d.get("model") and d.get("normalization")):
            raise ValueError(f"{path.name}: needs model and normalization (the runner would otherwise "
                             "select a checkpoint by itself), or an explicit command")
        for key in ("model", "normalization", "training_config", "robot_config"):
            if d.get(key):
                files[key] = str(d[key])
        return cls(name=d.get("name", path.stem), command=[str(c) for c in cmd],
                   required_mode=d["required_mode"], description=d.get("description", ""),
                   cwd=d.get("cwd"), files=files, env=d.get("env", {}) or {},
                   duration=d.get("duration"), stop_signal=d.get("stop_signal", "SIGINT"),
                   backend=str(d.get("backend", "")), algorithm=str(d.get("algorithm", "")),
                   control_hz=float(d["control_hz"]) if d.get("control_hz") else None,
                   requires_ft=bool(d.get("requires_ft", False)),
                   required_tcp=str(d.get("required_tcp", "") or ""),
                   required_frames=[str(f) for f in d.get("required_frames", []) or []],
                   entrypoint=str(d.get("entrypoint", "run_policy")),
                   package=str(d.get("package", "my_robot_policy")),
                   controller_gains=dict(d.get("controller_gains") or {}))

    def expanded_files(self) -> Dict[str, str]:
        return {k: os.path.expanduser(v) for k, v in self.files.items()}

    def build_command(self, recording: Optional["RecordingOptions"] = None,
                      duration: Optional[float] = None) -> List[str]:
        """The exact runner command line. A hand-written `command` only gets its {file} keys filled in."""
        files = self.expanded_files()
        if self.command:
            return [c.format(**files) for c in self.command]
        cmd = ["ros2", "run", self.package, ENTRYPOINTS.get(self.entrypoint, self.entrypoint),
               "--model", files["model"], "--normalization", files["normalization"]]
        if "training_config" in files:
            cmd += ["--config", files["training_config"]]
        if "robot_config" in files:
            cmd += ["--robot-config", files["robot_config"]]
        if self.backend:
            cmd += ["--robot-controller", self.backend]
        if duration and self.entrypoint == "run_scraping_policy":
            cmd += ["--policy-duration", f"{float(duration):g}"]
        return cmd + (recording.to_args() if recording else [])

    def contract_problems(self) -> List[str]:
        """The manifest must describe something the real runner can actually execute."""
        problems = []
        if self.backend and self.backend not in BACKEND_MODE and not self.command:
            problems.append(f"backend '{self.backend}' is not one of the runner's: {', '.join(sorted(BACKEND_MODE))}")
        if self.backend in BACKEND_MODE and self.required_mode != BACKEND_MODE[self.backend]:
            problems.append(f"backend '{self.backend}' executes through {BACKEND_MODE[self.backend]}, but the "
                            f"manifest requires mode '{self.required_mode}'")
        if self.entrypoint not in ENTRYPOINTS and not self.command:
            problems.append(f"entrypoint '{self.entrypoint}' is not one of: {', '.join(sorted(ENTRYPOINTS))}")
        if self.command:
            line = " ".join(self.command)
            for bad in FAKE_RUNNER_ARGS:
                if bad in line:
                    problems.append(f"command uses '{bad}': the runner has no such parameter and would ignore it "
                                    "(use model / normalization in the manifest)")
            if self.backend and f"--robot-controller {self.backend}" not in line:
                problems.append(f"command does not pass --robot-controller {self.backend}: the backend would be "
                                "display-only")
        elif "robot_config" not in self.files:
            problems.append("robot_config is required: the runner would otherwise load its FR3 default")
        return problems

    def preflight(self, known_modes) -> List[str]:
        problems = self.contract_problems()
        if self.required_mode not in known_modes:
            problems.append(f"required_mode '{self.required_mode}' is not a mode of this robot")
        for k, v in self.expanded_files().items():
            if not Path(v).exists():
                problems.append(f"{k}: {v} not found")
        if self.cwd and not Path(os.path.expanduser(self.cwd)).is_dir():
            problems.append(f"cwd {self.cwd} not found")
        return problems

    def live_preflight(self, modes, bridge) -> List[str]:
        """Static checks + robot state: required mode active, F/T live, nothing else running."""
        problems = self.preflight(modes.profile.modes)
        current = modes.current_mode()
        if current != self.required_mode:
            problems.append(f"needs control mode '{self.required_mode}', robot is in '{current}' "
                            "(add a controller step before the policy)")
        if self.requires_ft and not bridge.snapshot().connected.get("F/T"):
            problems.append("needs live F/T data; the F/T topic is silent")
        return problems


@dataclass
class RecordingOptions:
    """Maps 1:1 onto runner arguments. None = keep the robot config's setting."""
    record_obs: Optional[bool] = None
    record_video: Optional[bool] = None
    video_fps: Optional[float] = None
    output_root: Optional[str] = None

    def to_args(self) -> List[str]:
        out: List[str] = []
        if self.record_obs is not None:
            out.append("--record-obs" if self.record_obs else "--no-record-obs")
        if self.record_video is not None:
            out.append("--record-video" if self.record_video else "--no-record-video")
        if self.video_fps:
            out += ["--video-fps", f"{float(self.video_fps):g}"]
        if self.output_root:
            out += ["--output-root", os.path.expanduser(self.output_root)]
        return out


def _load_prctl():
    """libc prctl, resolved once in the parent: after fork only async-signal-safe work may run (no imports)."""
    if not sys.platform.startswith("linux"):
        return None
    try:
        import ctypes
        return ctypes.CDLL(None, use_errno=True).prctl
    except Exception:
        return None


_PRCTL = _load_prctl()


def _die_with_parent():
    """preexec_fn: the child gets SIGTERM if the GUI thread that started it dies (PR_SET_PDEATHSIG). Linux only."""
    if _PRCTL is not None:
        _PRCTL(1, signal.SIGTERM, 0, 0, 0)


class PolicyLibrary:
    def __init__(self, directory: str | Path):
        self.directory = Path(directory)
        self.manifests: Dict[str, PolicyManifest] = {}
        self.errors: Dict[str, str] = {}
        self._proc: Optional[subprocess.Popen] = None
        self._current: Optional[PolicyManifest] = None
        self._lock = threading.Lock()
        self.reload()

    def reload(self):
        self.manifests.clear()
        self.errors.clear()
        if not self.directory.is_dir():
            return
        for f in sorted(self.directory.glob("*.yaml")):
            try:
                m = PolicyManifest.load(f)
                self.manifests[m.name] = m
            except Exception as e:
                self.errors[f.name] = str(e)

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def run(self, name: str, modes, duration=None, stop_event: Optional[threading.Event] = None,
            log=print, recording: Optional[RecordingOptions] = None) -> bool:
        """Blocking. Returns True if the policy ran to its duration or exited with code 0."""
        m = self.manifests.get(name)
        if m is None:
            log(f"unknown policy '{name}'")
            return False
        # Never switch modes silently: the program must contain an explicit controller step.
        problems = m.live_preflight(modes, modes.bridge)
        if problems:
            log(f"policy '{name}' preflight failed: " + "; ".join(problems))
            return False
        cmd = m.build_command(recording, duration if duration is not None else m.duration)
        env = dict(os.environ, **{k: str(v) for k, v in m.env.items()})
        cwd = os.path.expanduser(m.cwd) if m.cwd else None
        duration = float(duration if duration is not None else (m.duration or 0)) or None
        with self._lock:
            if self.running:
                log("a policy is already running")
                return False
            log(f"policy '{name}' start: {' '.join(cmd)}")
            try:
                # own session = own process group (killpg reaches the whole tree); PDEATHSIG ends it with the GUI
                self._proc = subprocess.Popen(cmd, cwd=cwd, env=env, start_new_session=True,
                                              preexec_fn=_die_with_parent)
            except Exception as e:      # a failed start must end as a failure, never as an exception or a hang
                log(f"policy '{name}' failed to start: {e}")
                return False
            self._current = m
        t0 = time.monotonic()
        proc = self._proc
        try:
            while True:
                rc = proc.poll()
                if rc is not None:
                    log(f"policy '{name}' exited with code {rc}")
                    return rc == 0
                if stop_event is not None and stop_event.is_set():
                    self._terminate(m, wait=True)
                    log(f"policy '{name}' stopped")
                    return False
                if duration and time.monotonic() - t0 >= duration:
                    self._terminate(m, wait=True)
                    log(f"policy '{name}' finished ({duration:.1f} s)")
                    return True
                time.sleep(0.05)
        finally:
            self._proc = None
            self._current = None

    def _terminate(self, m: PolicyManifest, grace_s: float = 3.0, wait: bool = False):
        """Signal the policy's process group at once; escalate (stop signal -> SIGTERM -> SIGKILL) in a
        background thread so a caller on the tick / STOP path never blocks. wait=True joins it (worker thread)."""
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        sig = getattr(signal, m.stop_signal, signal.SIGINT)
        with self._lock:
            t = getattr(proc, "_gui_escalation", None)
            if t is None:
                try:
                    os.killpg(proc.pid, sig)            # immediate, before any waiting
                except ProcessLookupError:
                    return
                t = threading.Thread(target=self._escalate, args=(proc, grace_s), daemon=True)
                proc._gui_escalation = t
                t.start()
        if wait:
            t.join()

    @staticmethod
    def _escalate(proc, grace_s):
        """The stop signal is already sent; give the runner grace_s, then SIGTERM, then SIGKILL."""
        for s in (signal.SIGTERM, signal.SIGKILL):
            try:
                proc.wait(grace_s)
                return
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(proc.pid, s)
            except ProcessLookupError:
                return
        proc.wait()

    def stop(self):
        """Thread-safe and non-blocking: called by the GUI Stop button while run() blocks elsewhere."""
        m = self._current
        if m is not None and self.running:
            self._terminate(m)
