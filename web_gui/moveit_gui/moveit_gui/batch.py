"""Batch runs: one program run several times, one thing changed per run.

    version: 1
    program: scrape_vial.yaml          # the program it was made for (information only)
    kind: vials                        # vials | sweep | repeat
    vials: [rack A, rack B]            # kind vials: each profile becomes `fixture` for one run
    fixture: vial
    step: [3, 1]                       # kind sweep: the step (its path in the tree), checked against step_type
    step_type: force
    param: wrench.2                    # a step parameter (a list element: name.index), see SWEEPABLE
    values: [3, 4, 5]
    repeats: 3                         # kind repeat
    pause: true                        # ask the operator before every run after the first ("Next vial ready?")
    on_fail: stop                      # stop | continue

Saved next to the program as <program>.batch.yaml.

Safety: a batch only runs programs through the ProgramRunner (its preflight on every run, the lease and busy rules
of the core). The core counts the whole batch as one program run: nothing else may move, switch modes or change the
vial between runs. STOP (core.stop_all) ends the batch. Every run gets its own COPY of the program with the one
change applied: the open program is never edited. A vial batch puts the fixture back the way it was at the end,
also after STOP or a failure.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import yaml

from .program import Program, ProgramError, path_label

BATCH_VERSION = 1
KINDS = ("repeat", "vials", "sweep")
MAX_RUNS = 200
SUFFIX = ".batch.yaml"

# the numeric step parameters a sweep may change: step type -> [(key, label)]; "name.i" is element i of a list
_V = ("velocity", "Velocity (0-1)")
_MM_S = ("speed_mm_s", "Max mm/s")
SWEEPABLE = {
    "wait": [("seconds", "Seconds")],
    "policy": [("duration", "Duration s")],
    "force": [(f"wrench.{i}", n) for i, n in enumerate(("Fx N", "Fy N", "Fz N", "Tx Nm", "Ty Nm", "Tz Nm"))],
    "move_until_contact": [("force_n", "Contact |ΔF| N"), ("distance_mm", "At most mm"), _MM_S, _V],
    "move_relative": [("offset_mm.0", "Offset X mm"), ("offset_mm.1", "Offset Y mm"), ("offset_mm.2", "Offset Z mm"),
                      ("rotate_deg.2", "Rotate Z °"), _MM_S, _V],
    "move_pose": [_MM_S, _V],
    "movej": [_V],
    "loop": [("times", "Times")],
    "gripper": [("effort", "Effort N")],
}
_LIST_LEN = {"wrench": 6, "offset_mm": 3, "rotate_deg": 3}


class BatchError(ValueError):
    pass


def sweep_options(step) -> List[dict]:
    """[{key, label, value}] a sweep can change on this step (value: what it is now, None = not set)."""
    out = []
    for key, label in SWEEPABLE.get(step.type, []):
        if key == "speed_mm_s" and not step.linear:
            continue
        try:
            v = get_param(step.params, key)
        except BatchError:
            v = None
        out.append({"key": key, "label": label, "value": v})
    return out


def get_param(params: dict, key: str):
    name, _, idx = key.partition(".")
    v = params.get(name)
    if idx:
        if v is None:
            return 0.0
        try:
            return float(v[int(idx)])
        except (TypeError, ValueError, IndexError):
            raise BatchError(f"{key}: not a list value") from None
    return v


def set_param(params: dict, key: str, value) -> None:
    name, _, idx = key.partition(".")
    if idx:
        cur = list(params.get(name) or [0.0] * _LIST_LEN.get(name, 3))
        i = int(idx)
        if not 0 <= i < len(cur):
            raise BatchError(f"{key}: no element {i}")
        cur[i] = float(value)
        params[name] = cur
    elif name == "times":
        params[name] = int(round(float(value)))
    else:
        params[name] = float(value)


def _fmt(v) -> str:
    return f"{float(v):g}"


@dataclass
class BatchDef:
    kind: str = "repeat"
    repeats: int = 1
    vials: List[str] = field(default_factory=list)
    fixture: str = "vial"
    step: List[int] = field(default_factory=list)
    step_type: str = ""
    param: str = ""
    values: List[float] = field(default_factory=list)
    pause: bool = False
    on_fail: str = "stop"
    program: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "BatchDef":
        if not isinstance(d, dict):
            raise BatchError("a batch is a mapping")
        if int(d.get("version", BATCH_VERSION)) != BATCH_VERSION:
            raise BatchError(f"unsupported batch version {d.get('version')}")
        kind = d.get("kind", "repeat")
        if kind not in KINDS:
            raise BatchError(f"kind is one of {', '.join(KINDS)}")
        on_fail = d.get("on_fail", "stop")
        if on_fail not in ("stop", "continue"):
            raise BatchError("on_fail is stop or continue")
        b = cls(kind=kind, pause=bool(d.get("pause", False)), on_fail=on_fail, program=str(d.get("program") or ""),
                fixture=str(d.get("fixture") or "vial").strip())
        try:
            if kind == "repeat":
                r = d.get("repeats", 1)
                if isinstance(r, bool) or float(r) != int(float(r)) or not 1 <= int(float(r)) <= MAX_RUNS:
                    raise BatchError(f"repeats is a whole number 1..{MAX_RUNS}")
                b.repeats = int(float(r))
            elif kind == "vials":
                v = d.get("vials") or []
                if not isinstance(v, list) or not v or not all(isinstance(x, str) and x.strip() for x in v):
                    raise BatchError("pick at least one vial profile")
                b.vials = [x.strip() for x in v]
                if not b.fixture:
                    raise BatchError("give the fixture the program uses (e.g. vial)")
            else:
                step = d.get("step")
                if not isinstance(step, (list, tuple)) or not step:
                    raise BatchError("pick the step to change")
                b.step = [int(i) for i in step]
                b.step_type = str(d.get("step_type") or "")
                b.param = str(d.get("param") or "")
                if not b.param:
                    raise BatchError("pick the parameter to change")
                vals = d.get("values")
                if isinstance(vals, str):
                    vals = [x for x in vals.replace(";", ",").replace(" ", ",").split(",") if x]
                if not isinstance(vals, list) or not vals:
                    raise BatchError("give the values, e.g. 2, 3, 4")
                b.values = [float(x) for x in vals]
                if not all(math.isfinite(x) for x in b.values):
                    raise BatchError("the values must be finite numbers")
        except (TypeError, ValueError) as e:
            if isinstance(e, BatchError):
                raise
            raise BatchError(f"bad number in the batch ({e})") from None
        if len(b.runs()) > MAX_RUNS:
            raise BatchError(f"at most {MAX_RUNS} runs")
        return b

    def to_dict(self) -> dict:
        d: dict = {"version": BATCH_VERSION, "program": self.program, "kind": self.kind}
        if self.kind == "repeat":
            d["repeats"] = self.repeats
        elif self.kind == "vials":
            d.update(vials=list(self.vials), fixture=self.fixture)
        else:
            d.update(step=list(self.step), step_type=self.step_type, param=self.param, values=list(self.values))
        d.update(pause=self.pause, on_fail=self.on_fail)
        return d

    def runs(self) -> List[dict]:
        """One entry per run: {label, vial?, value?}."""
        if self.kind == "repeat":
            return [{"label": f"repeat {k}"} for k in range(1, self.repeats + 1)]
        if self.kind == "vials":
            return [{"label": f"vial {v}", "vial": v} for v in self.vials]
        return [{"label": f"step {path_label(self.step)} {self.param} = {_fmt(v)}", "value": v} for v in self.values]


def load(path: Path) -> BatchDef:
    try:
        d = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as e:
        raise BatchError(f"{Path(path).name}: {e}") from None
    return BatchDef.from_dict(d)


def save(path: Path, b: BatchDef) -> None:
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text("# MoveIt GUI batch run (PROGRAMS > Batch): one program, run several times.\n" +
                   yaml.safe_dump(b.to_dict(), sort_keys=False, default_flow_style=None), encoding="utf-8")
    tmp.replace(path)


def batch_path(program_path: Path) -> Path:
    p = Path(program_path)
    return p.with_name(p.stem + SUFFIX)


def variant(program: Program, b: BatchDef, run: dict) -> Program:
    """A copy of the program with this run's change applied (the original is not touched)."""
    prog = Program(name=program.name, robot=program.robot,
                   steps=[type(s).from_dict(s.to_dict()) for s in program.steps],
                   waypoints=copy.deepcopy(program.waypoints))
    if b.kind == "sweep":
        try:
            st = prog.step_at(b.step)
        except ProgramError:
            raise BatchError(f"there is no step {path_label(b.step)} any more: pick the step again") from None
        if b.step_type and st.type != b.step_type:
            raise BatchError(f"step {path_label(b.step)} is a {st.title} now, not {b.step_type}: pick the step again")
        if b.param not in {k for k, _ in SWEEPABLE.get(st.type, [])}:
            raise BatchError(f"{st.title} has no parameter {b.param} to change")
        set_param(st.params, b.param, run["value"])
    return prog


def check(program: Program, b: BatchDef, profile, vial_names) -> List[str]:
    """Problems found before anything runs: unknown vials, a step that is gone, values a run would refuse."""
    problems: List[str] = []
    if b.kind == "vials":
        missing = [v for v in b.vials if v not in set(vial_names)]
        if missing:
            problems.append(f"unknown vial profile{'s' * (len(missing) > 1)}: {', '.join(missing)}")
    for k, run in enumerate(b.runs(), 1):
        try:
            prog = variant(program, b, run)
        except BatchError as e:
            problems.append(str(e))
            break
        if b.kind == "sweep":
            found = [p for p in prog.validate(profile) if p.startswith(f"step {path_label(b.step)}:")]
            problems.extend(f"run {k} ({run['label']}): {p.split(': ', 1)[-1]}" for p in found)
    return problems


class BatchHooks:
    """What a batch needs from the GUI besides the runner: the vial selection (web/service.py fills it in)."""

    def use_vial(self, name: str, fixture: str) -> None:
        raise BatchError("vial batches need the GUI")

    def save_fixture(self, fixture: str):
        return None

    def restore_fixture(self, saved) -> str:
        return ""


class BatchRunner:
    """Runs a batch in its own thread through core.runner. While active, the core is busy (core.idle())."""

    def __init__(self, core, clock: Callable[[], float] = time.time):
        self.core = core
        self.clock = clock
        self._lock = threading.Lock()           # between "stopped?" and runner.start: a STOP never comes in between
        self._stop = threading.Event()
        self._answer = threading.Event()
        self._go = False
        self._thread: Optional[threading.Thread] = None
        self.active = False
        self.phase = "idle"                     # idle | running | waiting | ending
        self.rows: List[dict] = []
        self.question = ""
        self.ask_id = 0                         # +1 per question: the page asks once per id
        self.name = ""
        self.batch: Optional[BatchDef] = None
        self.total = 0
        self.result: Optional[Tuple[bool, str]] = None
        self.started_at: Optional[float] = None
        self.restored = ""

    def log(self, msg: str):
        self.core.bridge.log(f"batch: {msg}")

    def start(self, program: Program, b: BatchDef, hooks: BatchHooks, name: str = "") -> Tuple[bool, str]:
        with self._lock:
            if self.active:
                return False, "a batch is running"
            self.active, self.phase = True, "running"
            self._stop.clear()
            self._answer.clear()
            self.rows, self.question, self.result, self.restored = [], "", None, ""
            self.batch, self.name, self.total = b, name or program.name, len(b.runs())
            self.started_at = self.clock()
        self._thread = threading.Thread(target=self._run, args=(program, b, hooks), daemon=True, name="batch")
        self._thread.start()
        return True, f"batch started: {self.total} runs"

    def stop(self, why: str = "stopped"):
        """STOP: no further run starts; the run now is stopped by the caller (core.stop_all stops the runner)."""
        with self._lock:
            if not self.active:
                return
            if not self._stop.is_set():
                self._stop_why = why
            self._stop.set()
        self._answer.set()

    def answer(self, go: bool) -> Tuple[bool, str]:
        if self.phase != "waiting":
            return False, "the batch is not waiting for an answer"
        self._go = bool(go)
        self._answer.set()
        return True, "next run" if go else "stopping the batch"

    def join(self, timeout: Optional[float] = None):
        if self._thread:
            self._thread.join(timeout)

    def _wait_operator(self, question: str) -> bool:
        self._go = False
        self._answer.clear()
        self.question = question
        self.ask_id += 1
        self.phase = "waiting"
        while not self._answer.wait(0.1):
            if self._stop.is_set():
                break
        self.phase, self.question = "running", ""
        return self._go and not self._stop.is_set()

    def _run(self, program: Program, b: BatchDef, hooks: BatchHooks):
        core, runner = self.core, self.core.runner
        runs = b.runs()
        saved, have_saved = None, False
        why_end = ""
        try:
            if b.kind == "vials":
                saved, have_saved = hooks.save_fixture(b.fixture), True
            for k, run in enumerate(runs, 1):
                if self._stop.is_set():
                    break
                if k > 1 and b.pause:
                    q = ("Next vial ready?" if b.kind == "vials" else "Ready for the next run?") + \
                        f"\n\nRun {k} of {len(runs)}: {run['label']}."
                    if not self._wait_operator(q):
                        if not self._stop.is_set():
                            why_end = "stopped by the operator"
                            self._stop_why = why_end
                            self._stop.set()
                        break
                row = {"n": k, "change": run["label"], "ok": None, "result": "running", "duration_s": None,
                       "recording": "", "start": self.clock()}
                with self._lock:
                    if self._stop.is_set():
                        break
                    self.rows.append(row)
                    started, prep_err = False, ""
                    try:
                        prog = variant(program, b, run)
                        if run.get("vial"):
                            hooks.use_vial(run["vial"], b.fixture)
                        if core.modes.control_state_unknown:
                            raise BatchError("control state is UNKNOWN: recover first")
                        core.program_result = None
                        core.last_run_recording = ""
                        started = runner.start(prog, 0, False, False)
                    except Exception as e:             # noqa: BLE001 - a run that cannot start is a failed run
                        prep_err = str(e)
                if started:
                    while runner.alive():
                        runner.join(0.1)
                    ok, msg = core.program_result or (False, "no result")
                else:
                    ok, msg = False, prep_err or (core.program_result or (False, "the run did not start"))[1]
                row.update(ok=bool(ok), result=(msg.splitlines() or [""])[0] if not prep_err else msg,
                           duration_s=round(self.clock() - row["start"], 1),
                           recording=getattr(core, "last_run_recording", "") or "")
                self.log(f"run {k}/{len(runs)} ({run['label']}): {'ok' if ok else 'FAILED'}: {row['result']}")
                if self._stop.is_set():
                    break
                if not ok and b.on_fail == "stop":
                    why_end = f"run {k} failed"
                    break
        except Exception as e:                         # noqa: BLE001 - always end cleanly and restore
            why_end = f"error: {e}"
        finally:
            self.phase = "ending"
            if have_saved:
                try:
                    self.restored = hooks.restore_fixture(saved)
                    if self.restored:
                        self.log(self.restored)
                except Exception as e:                 # noqa: BLE001 - say it loudly: the vial is not the original
                    self.restored = f"could not put fixture {b.fixture} back: {e}"
                    self.log("FAILED: " + self.restored)
            done = [r for r in self.rows if r["ok"] is not None]
            good = sum(1 for r in done if r["ok"])
            stopped = self._stop.is_set()
            if stopped:
                why_end = getattr(self, "_stop_why", "") or "stopped"
            ok = not stopped and not why_end and good == len(runs)
            msg = f"{good} of {len(runs)} runs ok" + (f", {len(done) - good} failed" if len(done) > good else "")
            if why_end:
                msg += f" ({why_end})"
            self.result = (ok, msg)
            self.log(msg)
            try:
                core.history.record("batch", self.name, self.started_at, ok, msg, kind_of=b.kind, runs=len(runs))
            except Exception:                          # noqa: BLE001 - history is a convenience
                pass
            self.question = ""
            self.phase = "idle"
            self.active = False

    def view(self) -> dict:
        b = self.batch
        return {"active": self.active, "phase": self.phase, "name": self.name, "total": self.total,
                "done": sum(1 for r in self.rows if r["ok"] is not None), "rows": [dict(r) for r in self.rows],
                "question": self.question, "ask_id": self.ask_id, "result": self.result, "restored": self.restored,
                "kind": b.kind if b else None}
