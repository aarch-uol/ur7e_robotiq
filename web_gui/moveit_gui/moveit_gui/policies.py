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
    sim_ft_reference: ~/policies/v4/sim_ft_reference.json   # optional (SETUP > F/T + payload > Match simulation);
                                      # default: sim_ft_reference.json next to robot_config
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

F/T axes: when the profile in use has a Match simulation result (ft_match.py), a run gets an effective
robot config (the manifest's robot_config with observations.ft_obs_axes / ft_obs_scale from the result,
written under config_dir()/policy_runs/) in its place, on the generated command line and in a hand-written
command's {robot_config}. Without a result the manifest's file is used as it is.

The GUI does not load models. PolicyExecutor (policy_executor.py) owns the runtime path; the plan is
to replace the subprocess underneath it with a RunPolicy ROS action (docs/RunPolicy.action).
"""
from __future__ import annotations

import os
import json
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import yaml


BACKEND_MODE = {"impedance": "impedance", "servo_admittance": "admittance"}   # backend -> control mode
ENTRYPOINTS = {"run_policy": "run_policy.py", "run_scraping_policy": "run_scraping_policy.py"}
FAKE_RUNNER_ARGS = ("model_path:=", "vecnorm_path:=", "model:=", "normalization:=")
POLICY_NAME = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")


class PolicyImportError(ValueError):
    """A policy import was invalid or the card helper could not be used."""


def validate_policy_name(name: str) -> str:
    name = str(name or "").strip()
    if not POLICY_NAME.fullmatch(name):
        raise PolicyImportError("policy names use lower-case letters, digits, _ and -; start with a letter")
    return name


def allowed_policy_roots(directory: str | Path, roots=None) -> List[Path]:
    """Return resolved roots for model browsing. The GUI policy folder is always allowed."""
    directory = Path(directory).expanduser().resolve()
    if roots is None:
        raw = os.environ.get("MOVEIT_GUI_POLICY_IMPORT_ROOTS", "")
        roots = [x for x in raw.split(os.pathsep) if x.strip()] if raw else [Path.home() / "ws" / "scraping" / "policies"]
    out = [directory]
    for root in roots:
        p = Path(root).expanduser().resolve()
        if p not in out:
            out.append(p)
    return out


def policy_path_allowed(path: str | Path, roots) -> bool:
    try:
        p = Path(path).expanduser().resolve()
        return any(p == r or r in p.parents for r in (Path(x).expanduser().resolve() for x in roots))
    except (OSError, RuntimeError):
        return False


def _helper_command(command=None) -> List[str]:
    value = command or os.environ.get("MOVEIT_GUI_POLICY_CARD_COMMAND") or "python3 tools/export_policy_card.py"
    if isinstance(value, (list, tuple)):
        return [str(x) for x in value]
    return shlex.split(str(value))


def _checkpoint_path(value, folder: Path) -> Optional[Path]:
    p = Path(str(value).strip()).expanduser()
    if not p.is_absolute():
        p = folder / p
    try:
        p = p.resolve()
    except OSError:
        return None
    return p if p.is_file() and p.suffix.lower() in {".zip", ".pt", ".pth", ".ckpt"} else None


def list_policy_checkpoints(folder: str | Path, command=None, timeout: float = 30.0) -> List[Path]:
    """Ask the scraping helper for checkpoints; never imports its ML environment.

    The helper's normal output is JSON. Plain path-per-line output is accepted too, which keeps this
    usable with older helper revisions and simple test doubles.
    """
    folder = Path(folder).expanduser().resolve()
    if not folder.is_dir():
        raise PolicyImportError(f"folder not found: {folder}")
    try:
        r = subprocess.run(_helper_command(command) + ["--list", str(folder)], capture_output=True, text=True,
                           timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError) as e:
        raise PolicyImportError(f"policy card helper failed: {e}") from None
    if r.returncode != 0:
        detail = (r.stderr or r.stdout).strip().splitlines()
        raise PolicyImportError(f"policy card helper failed{': ' + detail[-1] if detail else ''}")
    text = (r.stdout or "").strip()
    values = []
    try:
        data = json.loads(text) if text else []
        if isinstance(data, dict):
            data = data.get("checkpoints", data.get("models", data.get("weights", [])))
        if isinstance(data, list):
            values = [x.get("path", x.get("model", x)) if isinstance(x, dict) else x for x in data]
    except json.JSONDecodeError:
        values = [x.strip() for x in text.splitlines() if x.strip() and not x.lstrip().startswith(("#", "INFO", "WARNING"))]
    out = []
    for value in values:
        p = _checkpoint_path(value, folder)
        if p is not None and p not in out:
            out.append(p)
    return sorted(out)


def _card_defaults(directory: Path, checkpoint: Path, manifests: Dict[str, "PolicyManifest"]):
    for m in manifests.values():
        model = m.expanded_files().get("model")
        if model and Path(model).resolve() == checkpoint:
            return m
    for m in manifests.values():
        model = m.expanded_files().get("model")
        if model and Path(model).parent == checkpoint.parent:
            return m
    return None


def _read_card(path: Path) -> dict:
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as e:
        raise PolicyImportError(f"policy card is missing or invalid: {e}") from None
    if not isinstance(data, dict) or data.get("version") != 1:
        raise PolicyImportError("policy card must have version 1")
    for key in ("model", "training_config", "algorithm", "recurrent", "control_hz", "action_names", "observation_dim"):
        if key not in data:
            raise PolicyImportError(f"policy card is missing '{key}'")
    return data


def policy_range_warnings(manifest: "PolicyManifest", raw_ft=None, tcp_z=None, ft_result=None) -> List[str]:
    """Compare current readings with the card's training ranges. Warnings only, never a block.

    F/T goes through the axes and scale a run would use: the Match simulation result in use (`ft_result`, the
    same one command_for() writes into the run's robot config), else the robot config's observations section.
    Without either the F/T comparison is skipped: a guessed sign would only make false warnings.
    """
    card = manifest.card()
    if not card:
        return []
    warnings = []
    axes = scale = None
    if ft_result and ft_result.get("ft_obs_axes") and ft_result.get("ft_obs_scale"):
        axes, scale = ft_result["ft_obs_axes"], ft_result["ft_obs_scale"]
    else:
        robot_config = manifest.expanded_files().get("robot_config")
        config = {}
        if robot_config:
            try:
                config = yaml.safe_load(Path(robot_config).read_text()) or {}
            except (OSError, yaml.YAMLError):
                config = {}
        obs = config.get("observations") or {}
        axes, scale = obs.get("ft_obs_axes"), obs.get("ft_obs_scale")
    if not axes or not scale:
        warnings.append("F/T axes not measured (SETUP > F/T + payload > Match simulation): F/T range not checked")
    elif raw_ft is not None:
        try:
            transformed = []
            for start in (0, 3):
                vec = [float(raw_ft[start + i]) for i in range(3)]
                transformed.extend(sum(float(axes[row][col]) * vec[col] for col in range(3)) for row in range(3))
            transformed = [v * float(scale[i]) for i, v in enumerate(transformed)]
            rng = (card.get("physical_ranges") or {}).get("ft_sensor")
            if rng and len(rng.get("min", [])) >= 6 and len(rng.get("max", [])) >= 6:
                bad = [(i, transformed[i], rng["min"][i], rng["max"][i]) for i in range(6)
                       if transformed[i] < float(rng["min"][i]) or transformed[i] > float(rng["max"][i])]
                if bad:
                    warnings.append("F/T outside training range: " + ", ".join(
                        f"{['fx','fy','fz','tx','ty','tz'][i]} {v:.3g} (range {lo:.3g}..{hi:.3g})"
                        for i, v, lo, hi in bad))
        except (TypeError, ValueError, IndexError):
            warnings.append("F/T axes or scale is invalid")
    rng = (card.get("physical_ranges") or {}).get("ee_pos_z")
    if rng and tcp_z is not None:
        try:
            lo, hi = float(rng["min"]), float(rng["max"])
            if not lo <= float(tcp_z) <= hi:
                warnings.append(f"TCP z {float(tcp_z):.4g} m outside training range {lo:.4g}..{hi:.4g} m")
        except (TypeError, ValueError, KeyError):
            warnings.append("TCP z training range is invalid")
    return warnings


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
    sim_ft_reference: str = ""                 # SETUP > F/T + payload > Match simulation (ft_match.py)
    policy_card: str = ""                       # optional policy_card.json next to the checkpoint

    @classmethod
    def load(cls, path: Path) -> "PolicyManifest":
        d = yaml.safe_load(path.read_text()) or {}
        cmd = d.get("command", [])
        if isinstance(cmd, str):
            raise ValueError(f"{path.name}: 'command' must be a list (no shell strings)")
        files = dict(d.get("files", {}) or {})
        if not cmd and not (d.get("model") and ("normalization" in d or d.get("policy_card"))):
            raise ValueError(f"{path.name}: needs model and normalization (or a policy card), or an explicit command")
        for key in ("model", "normalization", "training_config", "robot_config", "policy_card"):
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
                   controller_gains=dict(d.get("controller_gains") or {}),
                   sim_ft_reference=str(d.get("sim_ft_reference", "") or ""),
                   policy_card=str(d.get("policy_card", "") or ""))

    def expanded_files(self) -> Dict[str, str]:
        return {k: os.path.expanduser(v) for k, v in self.files.items()}

    def card(self) -> Optional[dict]:
        path = self.expanded_files().get("policy_card")
        if not path:
            return None
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, json.JSONDecodeError):
            return None
        return data if isinstance(data, dict) else None

    def build_command(self, recording: Optional["RecordingOptions"] = None,
                      duration: Optional[float] = None, files: Optional[Dict[str, str]] = None) -> List[str]:
        """The exact runner command line. A hand-written `command` only gets its {file} keys filled in.
        `files` replaces some of the manifest's files (the effective robot config)."""
        files = {**self.expanded_files(), **(files or {})}
        if self.command:
            return [c.format(**files) for c in self.command]
        cmd = ["ros2", "run", self.package, ENTRYPOINTS.get(self.entrypoint, self.entrypoint),
               "--model", files["model"]]
        if files.get("normalization"):
            cmd += ["--normalization", files["normalization"]]
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
    def __init__(self, directory: str | Path, allowed_roots=None, card_command=None):
        self.directory = Path(directory)
        self.allowed_roots = allowed_policy_roots(self.directory, allowed_roots)
        self.card_command = card_command
        self.manifests: Dict[str, PolicyManifest] = {}
        self.errors: Dict[str, str] = {}
        self._proc: Optional[subprocess.Popen] = None
        self._current: Optional[PolicyManifest] = None
        self._lock = threading.Lock()
        # () -> (profile name, Match simulation result) or None (ft_match.py); set by RobotCore
        self.ft_axes: Optional[Callable[[], Optional[tuple]]] = None
        self.effective_dir: Optional[Path] = None      # where effective robot configs go (default: config_dir())
        self.notify: Callable[[str], None] = lambda _m: None   # the GUI log (run()'s `log` is the result's)
        self.reload()

    def import_policy(self, folder: str | Path, checkpoint: str | Path, name: str, description: str = "",
                      backend: str = "", required_mode: str = "", required_tcp: str = "", overwrite: bool = False):
        """Create a manifest from one checkpoint and, when possible, export its policy card.

        Card export is deliberately best-effort: an unavailable scraping environment must not prevent an
        operator from keeping the model as a normal manifest.
        """
        folder = Path(folder).expanduser().resolve()
        if not policy_path_allowed(folder, self.allowed_roots):
            raise PolicyImportError("that folder is outside the allowed policy roots")
        checkpoint = _checkpoint_path(checkpoint, folder)
        if checkpoint is None or not policy_path_allowed(checkpoint, self.allowed_roots) \
                or not (checkpoint.parent == folder or folder in checkpoint.parents):
            raise PolicyImportError("pick a checkpoint under the selected folder and allowed roots")
        name = validate_policy_name(name)
        target = self.directory / f"{name}.yaml"
        if target.exists() and not overwrite:
            raise PolicyImportError(f"policy '{name}' already exists; confirm overwrite to replace it")
        old = _card_defaults(folder, checkpoint, self.manifests)
        backend = str(backend or (old.backend if old else "servo_admittance"))
        required_mode = str(required_mode or (old.required_mode if old else BACKEND_MODE.get(backend, "admittance")))
        required_tcp = str(required_tcp or (old.required_tcp if old else ""))
        card_path = checkpoint.parent / "policy_card.json"
        card = None
        card_error = ""
        if card_path.exists() and not overwrite:
            try:
                existing = _read_card(card_path)
                if Path(str(existing.get("model", ""))).expanduser().resolve() == checkpoint:
                    card = existing
                else:
                    raise PolicyImportError("policy_card.json already exists; confirm overwrite to replace it")
            except (OSError, PolicyImportError) as e:
                card_error = str(e)
        else:
            try:
                result = subprocess.run(_helper_command(self.card_command) + [str(checkpoint), "--out", str(card_path)],
                                        capture_output=True, text=True, timeout=120, check=False)
                if result.returncode == 0:
                    card = _read_card(card_path)
                else:
                    detail = (result.stderr or result.stdout).strip().splitlines()
                    card_error = detail[-1] if detail else f"exit {result.returncode}"
            except (OSError, subprocess.SubprocessError, PolicyImportError) as e:
                card_error = str(e)
        if card is None:
            card_error = card_error or "helper did not write a valid policy card"
        model = str(checkpoint)
        normalization = (str(card.get("normalization")) if card and card.get("normalization") else
                         (old.expanded_files().get("normalization") if old else None))
        training_config = (str(card.get("training_config")) if card and card.get("training_config") else
                           (old.expanded_files().get("training_config") if old else None))
        robot_config = old.expanded_files().get("robot_config") if old else None
        data = {"name": name, "description": str(description or (old.description if old else "")),
                "required_mode": required_mode, "backend": backend, "required_tcp": required_tcp,
                "model": model, "normalization": normalization, "training_config": training_config,
                "policy_card": str(card_path) if card else None,
                "algorithm": str(card.get("algorithm", "")) if card else (old.algorithm if old else ""),
                "control_hz": card.get("control_hz") if card else (old.control_hz if old else None),
                "requires_ft": bool(old.requires_ft) if old else True,
                "entrypoint": old.entrypoint if old else "run_policy",
                "package": old.package if old else "my_robot_policy"}
        if robot_config:
            data["robot_config"] = robot_config
        target.parent.mkdir(parents=True, exist_ok=True)
        tmp = target.with_suffix(target.suffix + ".tmp")
        tmp.write_text(yaml.safe_dump({k: v for k, v in data.items() if v is not None or k == "normalization"}, sort_keys=False))
        tmp.replace(target)
        self.reload()
        msg = f"imported {name}"
        if card:
            msg += f" with policy card {card_path.name}"
        else:
            msg += f"; policy card unavailable ({card_error})"
        return {"name": name, "manifest": str(target), "card": str(card_path) if card else "",
                "card_error": "" if card else card_error, "msg": msg}

    def edit_manifest(self, name: str, new_name: Optional[str] = None, description: Optional[str] = None,
                      overwrite: bool = False):
        m = self.manifests.get(name)
        if m is None:
            raise PolicyImportError(f"unknown policy '{name}'")
        if self.running and self._current is not None and self._current.name == name:
            raise PolicyImportError(f"'{name}' is running: stop it first")
        new_name = validate_policy_name(new_name or name)
        source = self.directory / f"{name}.yaml"
        target = self.directory / f"{new_name}.yaml"
        if target != source and target.exists() and not overwrite:
            raise PolicyImportError(f"policy '{new_name}' already exists; confirm overwrite to replace it")
        data = yaml.safe_load(source.read_text()) or {}
        data["name"] = new_name
        if description is not None:
            data["description"] = str(description)
        tmp = target.with_suffix(target.suffix + ".tmp")
        tmp.write_text(yaml.safe_dump(data, sort_keys=False))
        tmp.replace(target)
        if target != source:
            source.unlink()
        self.reload()
        return {"name": new_name, "msg": f"saved policy {new_name}"}

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

    def ft_axes_in_use(self) -> Optional[tuple]:
        """(profile name, result) of the Match simulation result in use, or None."""
        try:
            got = self.ft_axes() if self.ft_axes else None
        except Exception:
            return None
        return got if got and got[1] else None

    def command_for(self, name: str, recording: Optional[RecordingOptions] = None, duration=None,
                    log=print) -> List[str]:
        """The command line for a run of `name`, with the effective robot config when there is a result."""
        m = self.manifests[name]
        src = m.expanded_files().get("robot_config")
        got = self.ft_axes_in_use() if src else None
        files = None
        if got:
            from .ft_match import effective_robot_config
            from .calibration import config_dir
            owner, res = got
            path = effective_robot_config(src, res, self.effective_dir or config_dir() / "policy_runs", name, owner)
            files = {"robot_config": str(path)}
            msg = (f"policy '{name}': F/T axes from the Match simulation of profile '{owner or '(none)'}' "
                   f"({res.get('at', '?')}, {', '.join(res.get('explain') or [])}): robot config {path}")
            log(msg)
            self.notify(msg)
        elif src:
            msg = f"policy '{name}': no Match simulation result in this profile: robot config {src} as it is"
            log(msg)
            self.notify(msg)
        return m.build_command(recording, duration, files)

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
        try:
            cmd = self.command_for(name, recording, duration if duration is not None else m.duration, log)
        except Exception as e:
            log(f"policy '{name}': could not write the effective robot config: {e}")
            return False
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
