"""PROGRAMS > Batch (WebService mixin): run the open program several times, one thing changed per run.
The runner and the file format are in moveit_gui/batch.py; this adds the commands and the vial selection."""
from __future__ import annotations

import copy

from ..batch import BatchDef, BatchError, BatchHooks, batch_path, check, load, save, variant
from ..program import Program


class _VialHooks(BatchHooks):
    """A vial batch makes each vial profile the program's fixture in turn, and puts the fixture back afterwards."""

    def __init__(self, svc):
        self.svc = svc

    def use_vial(self, name: str, fixture: str) -> None:
        self.svc._vial_put(fixture, name, self.svc.vials.get(name))

    def save_fixture(self, fixture: str):
        return fixture, copy.deepcopy(self.svc.core.store.fixtures.get(fixture))

    def restore_fixture(self, saved) -> str:
        fixture, fx = saved
        store = self.svc.core.store
        if fx is None:
            if fixture in store.fixtures:
                store.delete_fixture(fixture)
                return f"fixture {fixture} removed again (there was none before the batch)"
            return ""
        store.put_fixture(copy.deepcopy(fx), replace=True)
        was = fx.meta.get("vial_profile")
        return f"fixture {fixture} put back as before the batch" + (f" (vial profile {was})" if was else "")


class BatchCommands:
    def _batch_def(self, batch) -> BatchDef:
        try:
            d = BatchDef.from_dict(dict(batch or {}))
        except BatchError as e:
            raise self._err(str(e)) from None
        d.program = self.program_path.name if self.program_path else ""
        return d

    def _batch_problems(self, d: BatchDef):
        names = [e["name"] for e in self.vials.view(self.core.store.fixtures)]
        return check(self.program, d, self.core.profile, names)

    def cmd_batch_get(self):
        """The batch saved next to the open program (None if there is none)."""
        if not self.program_path:
            return self._ok("no batch: the program is not saved", batch=None, file=None)
        p = batch_path(self.program_path)
        if not p.exists():
            return self._ok("no saved batch", batch=None, file=p.name)
        try:
            return self._ok(f"loaded {p.name}", batch=load(p).to_dict(), file=p.name)
        except BatchError as e:
            raise self._err(str(e)) from None
    cmd_batch_get.read_only = True

    def cmd_batch_check(self, batch=None):
        d = self._batch_def(batch)
        problems = self._batch_problems(d)
        return self._ok("batch OK" if not problems else "problems found", problems=problems, runs=len(d.runs()))
    cmd_batch_check.read_only = True

    def cmd_batch_save(self, batch=None):
        if not self.program_path:
            raise self._err("save the program first: the batch is kept next to it")
        self._own(self._program_owner())            # next to another person's program: theirs, not yours
        d = self._batch_def(batch)
        p = batch_path(self.program_path)
        save(p, d)
        self.core.bridge.log(f"batch saved: {p}")
        return self._ok(f"saved {p.name}", batch=d.to_dict(), file=p.name)

    def cmd_batch_run(self, batch=None):
        """Start the batch. Every run is an ordinary program run (preflight, lease, busy rules); STOP ends it all."""
        c = self.core
        if getattr(self, "teach", None) is not None:
            raise self._err("finish teaching first")
        if not c.idle():
            raise self._err("robot is busy")
        if c.modes.control_state_unknown:
            raise self._err("control state is UNKNOWN: recover first")
        d = self._batch_def(batch)
        problems = self._batch_problems(d)
        prog = Program.from_dict(self.program.to_dict(), c.profile)          # what runs: the program as it is now
        if not problems and d.kind != "vials":
            try:
                problems = c.runner.preflight(variant(prog, d, d.runs()[0]))
            except BatchError as e:
                problems = [str(e)]
        if problems:
            raise self._err("batch check failed: " + "; ".join(problems))
        c.program_result = None
        ok, msg = c.batch.start(prog, d, _VialHooks(self), self.program.name)
        if not ok:
            raise self._err(msg)
        c.bridge.log(f"batch: {self.lease.name or 'operator'} started {d.kind} of {self.program.name}: {len(d.runs())} runs"
                     + (", asking before each next run" if d.pause else "") + f", on failure {d.on_fail}")
        return self._ok(msg)

    def cmd_batch_answer(self, go=True):
        ok, msg = self.core.batch.answer(bool(go))
        if not ok:
            raise self._err(msg)
        return self._ok(msg)

    def cmd_batch_stop(self):
        """Ends the batch and the run now (like STOP, without stopping anything else)."""
        if not self.core.batch.active:
            return self._ok("no batch is running")
        self.core.batch.stop("stopped from PROGRAMS")
        self.core.runner.stop()
        return self._ok("stopping the batch")
    cmd_batch_stop.read_only = True                     # stopping never needs the lease
