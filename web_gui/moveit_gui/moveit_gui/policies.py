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
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import yaml


BACKEND_MODE = {"impedance": "impedance", "servo_admittance": "admittance"}   # backend -> control mode
ENTRYPOINTS = {"run_policy": "run_policy.py", "run_scraping_policy": "run_scraping_policy.py"}
FAKE_RUNNER_ARGS = ("model_path:=", "vecnorm_path:=", "model:=", "normalization:=")
POLICY_NAME = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")
UPLOAD_FOLDER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
CHECKPOINT_SUFFIXES = {".zip", ".pt", ".pth", ".ckpt"}
POLICY_UPLOAD_SUFFIXES = {".zip", ".pkl", ".yaml", ".yml", ".json", ".pt", ".pth"}
MAX_POLICY_UPLOAD_BYTES = 300 << 20


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
    value = command if command is not None else os.environ.get("MOVEIT_GUI_POLICY_CARD_COMMAND", "")
    if not value:
        return []
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
    return p if p.is_file() and p.suffix.lower() in CHECKPOINT_SUFFIXES else None


def list_policy_checkpoints(folder: str | Path, command=None, timeout: float = 30.0) -> List[Path]:
    """List checkpoint files without starting the card helper or loading model data."""
    folder = Path(folder).expanduser().resolve()
    if not folder.is_dir():
        raise PolicyImportError(f"folder not found: {folder}")
    out = []
    for root, dirs, files in os.walk(folder, followlinks=False):
        dirs[:] = [d for d in dirs if not (Path(root) / d).is_symlink()]
        for name in files:
            p = Path(root) / name
            if p.is_symlink() or p.suffix.lower() not in CHECKPOINT_SUFFIXES:
                continue
            try:
                if p.is_file():
                    out.append(p.resolve())
            except OSError:
                continue
    return sorted(set(out), key=lambda p: str(p))


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


def split_range_notes(lines):
    """(warnings, notes): notes say a check was not done here ("not checked", "checked during the run")."""
    notes = [x for x in lines if "not checked" in x or "checked during the run" in x]
    return [x for x in lines if x not in notes], notes


def policy_range_warnings(manifest: "PolicyManifest", raw_ft=None, tcp_z=None, ft_result=None) -> List[str]:
    """Compare current readings with the card's training ranges. Warnings only, never a block.

    F/T is deliberately deferred to the runner: its startup bias and simulation baseline mean that a raw
    pre-run wrench is not in the same observation space as the card.
    """
    card = manifest.card()
    if not card:
        return ["Policy ranges not checked: no matching policy card"]
    if card.get("normalization_stats") is False:
        return ["Policy ranges not checked: no normalisation statistics"]
    warnings = []
    if (card.get("physical_ranges") or {}).get("ft_sensor"):
        # The pre-run wrench is not the runner's observation: it has a startup bias and may have a simulation
        # baseline.  The runner publishes the authoritative live out_of_range list during a run.
        warnings.append("F/T: checked during the run")
    rng = (card.get("physical_ranges") or {}).get("ee_pos_z")
    if rng:
        robot_config = manifest.expanded_files().get("robot_config")
        if not robot_config:
            warnings.append("TCP height: not checked (no robot config)")
        elif tcp_z is None:
            warnings.append("TCP height: not checked (TCP pose unavailable)")
        else:
            try:
                config = yaml.safe_load(Path(robot_config).read_text()) or {}
                obs = config.get("observations") or {}
                controller_bias = f"obs_position_bias_xyz_ee_{manifest.backend}" if manifest.backend else ""
                bias = (obs.get(controller_bias) if controller_bias else None) or obs.get("obs_position_bias_xyz_ee") or \
                        obs.get("ee_pos_bias_xyz") or obs.get("obs_position_bias_xyz") or \
                        obs.get("position_bias_xyz") or obs.get("ee_position_offset_xyz") or \
                        obs.get("position_offset_xyz")
                z = float(tcp_z) + (float(bias[2]) if bias is not None and len(bias) >= 3 else 0.0)
                lo, hi = float(rng["min"]), float(rng["max"])
                if not lo <= z <= hi:
                    warnings.append(f"TCP z {z:.4g} m outside training range {lo:.4g}..{hi:.4g} m")
            except (OSError, TypeError, ValueError, KeyError, IndexError, yaml.YAMLError):
                warnings.append("TCP height: not checked (robot config is invalid)")
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
    policy_card: str = ""                       # optional card in this policy's folder
    no_normalization: bool = False              # explicit opt-out for models that have no normaliser
    source: Optional[Path] = field(default=None, repr=False, compare=False)

    @classmethod
    def load(cls, path: Path) -> "PolicyManifest":
        d = yaml.safe_load(path.read_text()) or {}
        cmd = d.get("command", [])
        if isinstance(cmd, str):
            raise ValueError(f"{path.name}: 'command' must be a list (no shell strings)")
        files = dict(d.get("files", {}) or {})
        if not cmd and not d.get("model"):
            raise ValueError(f"{path.name}: needs model and normalization, or an explicit command")
        if "normalization" in d and not d.get("normalization") and not bool(d.get("no_normalization")):
            raise ValueError(f"{path.name}: normalization is empty; set no_normalization: true to opt out")
        if not cmd and not d.get("normalization") and not bool(d.get("no_normalization")):
            raise ValueError(f"{path.name}: needs a normalization file or explicit no_normalization: true")
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
                   policy_card=str(d.get("policy_card", "") or ""),
                   no_normalization=bool(d.get("no_normalization", False)), source=path.resolve())

    def expanded_files(self) -> Dict[str, str]:
        base = self.source.parent if self.source is not None else Path.cwd()
        out = {}
        for k, v in self.files.items():
            p = Path(os.path.expanduser(v))
            if not p.is_absolute():
                p = base / p
            out[k] = str(p)
        return out

    def card(self) -> Optional[dict]:
        path = self.expanded_files().get("policy_card")
        if not path:
            return None
        card_path = Path(path)
        if card_path.is_symlink():
            return None
        try:
            data = json.loads(card_path.read_text())
        except (OSError, json.JSONDecodeError):
            return None
        if not isinstance(data, dict):
            return None
        model = self.expanded_files().get("model")
        card_model = data.get("model")
        if model and not card_model:
            return None
        if model and card_model:
            cp = Path(os.path.expanduser(str(card_model)))
            if not cp.is_absolute():
                cp = card_path.parent / cp
            try:
                if cp.resolve() != Path(model).expanduser().resolve():
                    return None
            except OSError:
                return None
        return data

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
            have = set(self.files)
            for key in sorted({k for c in self.command for k in re.findall(r"{(\w+)}", c)} - have):
                problems.append(f"command needs {{{key}}} but the policy has no {key} file"
                                + (" (add robot_config.yaml to the policy folder)" if key == "robot_config" else ""))
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
    def __init__(self, directory: str | Path, allowed_roots=None, card_command=None, run_command=None):
        self.directory = Path(directory)
        self.allowed_roots = allowed_policy_roots(self.directory, allowed_roots)
        self.card_command = card_command
        # Command template for imported policies, e.g. "python3 /ws/scraping/run_policy.py --model {model} ...".
        # None: an earlier manifest for the same folder, else MOVEIT_GUI_POLICY_RUN_COMMAND, else ros2 run.
        self.run_command = run_command
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
                      backend: str = "", required_mode: str = "", required_tcp: str = "", overwrite: bool = False,
                      no_normalization: bool = False):
        """Copy a checkpoint and its deployment inputs into one self-contained policy folder."""
        folder = Path(folder).expanduser().resolve()
        if not policy_path_allowed(folder, self.allowed_roots):
            raise PolicyImportError("that folder is outside the allowed policy roots")
        checkpoint_input = Path(checkpoint).expanduser()
        if not checkpoint_input.is_absolute():
            checkpoint_input = folder / checkpoint_input
        if checkpoint_input.is_symlink():
            raise PolicyImportError("symlinked checkpoints are not allowed")
        checkpoint = _checkpoint_path(checkpoint_input, folder)
        if checkpoint is None or not policy_path_allowed(checkpoint, self.allowed_roots) \
                or not (checkpoint.parent == folder or folder in checkpoint.parents):
            raise PolicyImportError("pick a checkpoint under the selected folder and allowed roots")
        name = validate_policy_name(name)
        target = self.directory / name
        legacy_target = self.directory / f"{name}.yaml"
        if (target.exists() or legacy_target.exists()) and not overwrite:
            raise PolicyImportError(f"policy '{name}' already exists; confirm overwrite to replace it")
        old = _card_defaults(folder, checkpoint, self.manifests)
        backend = str(backend or (old.backend if old else "servo_admittance"))
        required_mode = str(required_mode or (old.required_mode if old else BACKEND_MODE.get(backend, "admittance")))
        required_tcp = str(required_tcp or (old.required_tcp if old else ""))
        self.directory.mkdir(parents=True, exist_ok=True)
        stage = Path(tempfile.mkdtemp(prefix=f".{name}.tmp-", dir=self.directory))
        os.chmod(stage, 0o755)                       # mkdtemp makes it owner-only; the runner may be another user
        card = None
        card_error = ""
        card_source = None
        card_name = f"{checkpoint.stem}.policy_card.json"
        try:
            helper = _helper_command(self.card_command)
            if helper:
                try:
                    result = subprocess.run(helper + [str(checkpoint), "--out", str(stage / card_name)],
                                            capture_output=True, text=True, timeout=120, check=False)
                    if result.returncode == 0:
                        try:
                            card = _read_card(stage / card_name)
                            card_source = stage / card_name
                        except PolicyImportError as e:
                            card_error = str(e)
                    else:
                        detail = (result.stderr or result.stdout).strip().splitlines()
                        card_error = detail[-1] if detail else f"exit {result.returncode}"
                except (OSError, subprocess.SubprocessError) as e:
                    card_error = str(e)
            else:
                card_error = "card tool not set (--policy-card-command)"
            if card is None:
                for candidate in (checkpoint.with_name(f"{checkpoint.stem}.policy_card.json"),
                                  checkpoint.parent / "policy_card.json"):
                    if not candidate.exists():
                        continue
                    if candidate.is_symlink():
                        raise PolicyImportError("symlinked policy cards are not allowed")
                    try:
                        existing = _read_card(candidate)
                        model = Path(str(existing.get("model", ""))).expanduser()
                        if not model.is_absolute():
                            model = candidate.parent / model
                        if model.resolve() == checkpoint:
                            card, card_source = existing, candidate
                            break
                    except (OSError, PolicyImportError):
                        continue

            model_dst = self._copy_import_file(checkpoint, stage, checkpoint.name)
            def card_source_path(value):
                if not value:
                    return None
                p = Path(str(value)).expanduser()
                if not p.is_absolute():
                    bases = [card_source.parent if card_source else checkpoint.parent, checkpoint.parent]
                    for base in bases:
                        candidate = base / p
                        if candidate.exists():
                            return candidate
                    p = bases[0] / p
                return p

            def find_file(kind, card_key, names):
                # The card (made from this checkpoint) or an earlier manifest wins. Otherwise look only next to
                # the checkpoint and one folder up (SB3 layout: run/config.yaml, run/best_model/model.zip), never
                # deeper: a logs folder holds many runs and a stray file from another run must never be picked.
                value = card_source_path(card.get(card_key)) if card else None
                if value is None and old:
                    value = Path(old.expanded_files()[kind]) if old.expanded_files().get(kind) else None
                if value is not None and value.is_file():
                    return value
                bases = [checkpoint.parent]
                if checkpoint.parent != folder and folder in checkpoint.parents:
                    bases.append(checkpoint.parent.parent)
                for base in bases:
                    for candidate in names:
                        p = base / candidate
                        if p.is_file():
                            return p
                    hits = []
                    for p in sorted(base.iterdir()):
                        if not p.is_file() or p.is_symlink():
                            continue
                        low, suf = p.name.lower(), p.suffix.lower()
                        if kind == "normalization" and suf == ".pkl" and any(x in low for x in ("norm", "vec")):
                            hits.append(p)
                        elif kind == "training_config" and suf in {".yaml", ".yml"} and \
                                any(x in low for x in ("config", "train")) and "robot" not in low:
                            hits.append(p)
                        elif kind == "robot_config" and suf in {".yaml", ".yml"} and \
                                any(x in low for x in ("robot", "deploy")):
                            hits.append(p)
                    if kind == "normalization" and len(hits) > 1:
                        stem = checkpoint.stem.replace("_model", "").replace("model", "").strip("_")
                        same = [h for h in hits if stem and stem in h.name]
                        if len(same) == 1:
                            return same[0]
                        raise PolicyImportError("several normalisation files next to the checkpoint ("
                                                + ", ".join(h.name for h in hits) + "): make the card first")
                    if hits:
                        return hits[0]
                return None

            normalization_src = None if no_normalization else find_file(
                "normalization", "normalization", ("vec_normalize.pkl", "vecnorm.pkl", "normalization.pkl", "normalizer.pkl"))
            if normalization_src is None and not no_normalization:
                raise PolicyImportError("no normalisation file found; tick 'no normaliser (model has none)' to import without one")
            norm_dst = None if no_normalization else self._copy_import_file(normalization_src, stage, normalization_src.name)
            training_src = find_file("training_config", "training_config", ("config.yaml", "config.yml", "training.yaml", "training.yml"))
            robot_src = find_file("robot_config", "robot_config", ("robot_config.yaml", "robot_config.yml", "deployment.yaml", "deployment.yml"))
            training_dst = self._copy_import_file(training_src, stage, training_src.name) if training_src else None
            robot_dst = self._copy_import_file(robot_src, stage, robot_src.name) if robot_src else None

            if card is not None:
                card = dict(card)
                card["model"] = model_dst.name
                card["normalization"] = norm_dst.name if norm_dst else None
                card["training_config"] = training_dst.name if training_dst else None
                card["robot_config"] = robot_dst.name if robot_dst else None
                if no_normalization:
                    card["normalization_stats"] = False
                (stage / card_name).write_text(json.dumps(card, indent=2, sort_keys=True) + "\n")
            else:
                (stage / card_name).unlink(missing_ok=True)
                card_name = ""
            data = {"name": name, "description": str(description or (old.description if old else "")),
                    "required_mode": required_mode, "backend": backend, "required_tcp": required_tcp,
                    "model": model_dst.name, "normalization": norm_dst.name if norm_dst else None,
                    "training_config": training_dst.name if training_dst else None,
                    "robot_config": robot_dst.name if robot_dst else None,
                    "policy_card": card_name or None,
                    "no_normalization": bool(no_normalization),
                    "algorithm": str(card.get("algorithm", "")) if card else (old.algorithm if old else ""),
                    "control_hz": card.get("control_hz") if card else (old.control_hz if old else None),
                    "requires_ft": bool(old.requires_ft) if old else True,
                    "command": (list(old.command) if old and old.command else
                                _helper_command(self.run_command if self.run_command is not None else
                                                os.environ.get("MOVEIT_GUI_POLICY_RUN_COMMAND", "")) or None),
                    "entrypoint": old.entrypoint if old else "run_policy",
                    "package": old.package if old else "my_robot_policy"}
            manifest_path = stage / "manifest.yaml"
            manifest_path.write_text(yaml.safe_dump({k: v for k, v in data.items()
                                                     if v is not None or k in ("normalization", "no_normalization")},
                                                    sort_keys=False))
            self._replace_policy_folder(stage, target, legacy_target, overwrite)
        except Exception:
            shutil.rmtree(stage, ignore_errors=True)
            raise
        self.reload()
        msg = f"imported {name}"
        if card:
            msg += f" with policy card {card_name}"
        else:
            msg += f"; Policy card not made: {card_error or 'helper did not write a valid policy card'}"
        if not robot_dst:
            msg += "; no robot config found next to the checkpoint: add robot_config.yaml to the policy folder before running"
        return {"name": name, "manifest": str(target / "manifest.yaml"),
                "card": str(target / card_name) if card else "", "card_error": "" if card else card_error, "msg": msg}

    def _copy_import_file(self, source: Optional[Path], stage: Path, name: str) -> Optional[Path]:
        if source is None:
            return None
        source = Path(source).expanduser()
        if source.is_symlink():
            raise PolicyImportError(f"symlinked policy file is not allowed: {source.name}")
        if not source.is_file():
            raise PolicyImportError(f"policy file not found: {source}")
        if not policy_path_allowed(source, self.allowed_roots):
            raise PolicyImportError(f"policy file is outside the allowed roots: {source.name}")
        dest = stage / Path(name).name
        if dest.exists() and dest != source:
            raise PolicyImportError(f"policy files have the same name: {dest.name}")
        shutil.copy2(source, dest, follow_symlinks=False)
        return dest

    @staticmethod
    def _replace_policy_folder(stage: Path, target: Path, legacy: Path, overwrite: bool):
        backups = []
        try:
            for existing in (target, legacy):
                if not existing.exists() and not existing.is_symlink():
                    continue
                if not overwrite:
                    raise PolicyImportError(f"policy '{target.name}' already exists; confirm overwrite to replace it")
                backup = existing.parent / f".{existing.name}.old-{uuid.uuid4().hex}"
                os.replace(existing, backup)
                backups.append((existing, backup))
            os.replace(stage, target)
        except Exception:
            for existing, backup in reversed(backups):
                if not existing.exists() and backup.exists():
                    os.replace(backup, existing)
            raise
        finally:
            for _, backup in backups:
                if backup.is_dir() and not backup.is_symlink():
                    shutil.rmtree(backup, ignore_errors=True)
                elif backup.exists() or backup.is_symlink():
                    backup.unlink(missing_ok=True)

    def save_uploaded_folder(self, folder_name: str, files: List[tuple[str, bytes]]) -> Path:
        """Save browser-uploaded policy inputs under an always-allowed import root."""
        if not UPLOAD_FOLDER.fullmatch(str(folder_name or "")):
            raise PolicyImportError("uploaded folder name is invalid")
        total = 0
        checked = []
        for relative, data in files:
            rel = Path(str(relative).replace("\\", "/"))
            if rel.is_absolute() or not rel.parts or ".." in rel.parts or any(not p for p in rel.parts):
                raise PolicyImportError("uploaded file path is invalid")
            if rel.suffix.lower() not in POLICY_UPLOAD_SUFFIXES:
                raise PolicyImportError(f"file type not allowed: {rel.name}")
            if not isinstance(data, (bytes, bytearray)):
                raise PolicyImportError("uploaded file data is invalid")
            total += len(data)
            if total > MAX_POLICY_UPLOAD_BYTES:
                raise PolicyImportError("policy folder upload is over 300 MB")
            checked.append((rel, bytes(data)))
        if not checked:
            raise PolicyImportError("uploaded policy folder is empty")
        uploads = self.directory / "uploads"
        if uploads.is_symlink():
            raise PolicyImportError("policy uploads directory cannot be a symlink")
        uploads.mkdir(parents=True, exist_ok=True)
        target = uploads / str(folder_name)
        if target.exists() or target.is_symlink():
            raise PolicyImportError(f"uploaded folder '{folder_name}' already exists")
        stage = Path(tempfile.mkdtemp(prefix=f".{folder_name}.tmp-", dir=uploads))
        os.chmod(stage, 0o755)
        try:
            seen = set()
            for rel, data in checked:
                if rel in seen:
                    raise PolicyImportError(f"uploaded file appears twice: {rel}")
                seen.add(rel)
                dest = stage / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
            os.replace(stage, target)
        except Exception:
            shutil.rmtree(stage, ignore_errors=True)
            raise
        return target

    def edit_manifest(self, name: str, new_name: Optional[str] = None, description: Optional[str] = None,
                      overwrite: bool = False):
        m = self.manifests.get(name)
        if m is None:
            raise PolicyImportError(f"unknown policy '{name}'")
        if self.running and self._current is not None and self._current.name == name:
            raise PolicyImportError(f"'{name}' is running: stop it first")
        requested_name = name if new_name is None else str(new_name).strip()
        new_name = validate_policy_name(requested_name) if requested_name != name else name
        source = Path(m.source) if m.source else self.directory / f"{name}.yaml"
        folder_source = source.parent if source.parent != self.directory and source.name in {"manifest.yaml", "policy.yaml"} else None
        target = (self.directory / new_name if folder_source else self.directory / f"{new_name}.yaml")
        move_folder = bool(folder_source and new_name != name)
        if ((folder_source and move_folder and target.exists()) or
                (not folder_source and target != source and target.exists())) and not overwrite:
            raise PolicyImportError(f"policy '{new_name}' already exists; confirm overwrite to replace it")
        data = yaml.safe_load(source.read_text()) or {}
        data["name"] = new_name
        if description is not None:
            data["description"] = str(description)
        if folder_source:
            target_manifest = folder_source / source.name
            tmp = target_manifest.with_suffix(target_manifest.suffix + ".tmp")
        else:
            target_manifest = target
            tmp = target.with_suffix(target.suffix + ".tmp")
        tmp.write_text(yaml.safe_dump(data, sort_keys=False))
        tmp.replace(target_manifest)
        if move_folder:
            backup = None
            if target.exists():
                backup = self.directory / f".{target.name}.old-{uuid.uuid4().hex}"
                os.replace(target, backup)
            try:
                os.replace(folder_source, target)
            except Exception:
                if backup is not None and backup.exists():
                    os.replace(backup, target)
                raise
            if backup is not None:
                shutil.rmtree(backup, ignore_errors=True)
        elif not folder_source and target != source:
            source.unlink()
        self.reload()
        return {"name": new_name, "msg": f"saved policy {new_name}"}

    def reload(self):
        manifests: Dict[str, PolicyManifest] = {}
        errors: Dict[str, str] = {}
        if not self.directory.is_dir():
            with self._lock:
                self.manifests, self.errors = manifests, errors
            return
        paths = list(self.directory.glob("*.yaml"))
        paths += list(self.directory.glob("*/manifest.yaml"))
        paths += list(self.directory.glob("*/policy.yaml"))
        for f in sorted(set(paths)):
            try:
                m = PolicyManifest.load(f)
                manifests[m.name] = m
            except Exception as e:
                errors[str(f.relative_to(self.directory))] = str(e)
        with self._lock:
            self.manifests, self.errors = manifests, errors

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def ft_axes_in_use(self) -> Optional[tuple]:
        """(profile name, result) of the Match simulation result in use, or None."""
        try:
            got = self.ft_axes() if self.ft_axes else None
        except Exception:
            return None
        return got if got and got[1] and not got[1].get("stale") else None

    def ft_axes_stale(self) -> Optional[str]:
        """Why the saved Match result cannot be used now (the reading axes changed since), or None."""
        try:
            got = self.ft_axes() if self.ft_axes else None
        except Exception:
            return None
        return (got[1] or {}).get("stale") if got else None

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
