"""Each profile's own settings: the robot runs the settings of the person who has control.

The files in use (setups.members(): TCPs, fixtures, keep-out zones, F/T bias and limit, gains + payload, vials,
flasks, parts, camera calibrations, recording settings, F/T match simulation) belong to one profile at a time,
named in config_dir()/profiles/active.yaml. When someone else takes control, their settings are switched in:

    1. the files in use are copied to the profile they belong to   (profiles/<old>/live/files/)
    2. the new person's copy is put in place                         (profiles/<new>/live/files/)
    3. the GUI restarts so every part of it reads them (the robot stack too when the parts or the base differ)

So what one person adds or changes (a keep-out box, a TCP, gains) stays in their profile and never shows up in
anyone else's. A profile without settings yet starts from a copy of the settings in use, one of its setups, or a
shared setup (the person picks).

profiles/<name>/live/live.yaml says when the copy was made, on which end-effector base, and which setup it was
(the setups/current.yaml marker, put back with it).
"""
from __future__ import annotations

import shutil
import tempfile
import time
from pathlib import Path
from typing import Optional

import yaml

from .setups import CURRENT, _atomic_write, _fsync_dir, _fsync_tree, copy_files, put_in_place
from .user_profiles import ProfileError, Profiles

ACTIVE = "active.yaml"


class WorkSets:
    def __init__(self, cfg: Path, robot_profile: str, profiles: Optional[Profiles] = None):
        self.cfg, self.robot = Path(cfg), robot_profile
        self.profiles = profiles or Profiles(self.cfg)
        self.marker = self.profiles.dir / ACTIVE

    # ---- whose settings are in use
    def active(self) -> str:
        """The profile whose settings the files in use are ("" = nobody's yet: the files from before profiles)."""
        try:
            st = self.marker.stat().st_mtime_ns
        except OSError:
            return ""
        if getattr(self, "_cache", (None,))[0] != st:      # read on every command and state: only when it changed
            try:
                d = yaml.safe_load(self.marker.read_text()) or {}
            except Exception:
                d = {}
            self._cache = (st, str(d.get("name") or "") if isinstance(d, dict) else "")
        n = self._cache[1]
        try:
            return self.profiles.find(n) if n else ""
        except ProfileError:
            return ""                                    # that profile was deleted

    def set_active(self, name: str):
        self.marker.parent.mkdir(parents=True, exist_ok=True)
        _atomic_write(self.marker, yaml.safe_dump({"name": str(name), "at": time.strftime("%Y-%m-%d %H:%M")}))

    def _dir(self, name: str) -> Path:
        return self.profiles.path(self.profiles.find(name)) / "live"

    def has(self, name: str) -> bool:
        try:
            return (self._dir(name) / "live.yaml").is_file()
        except ProfileError:
            return False

    def info(self, name: str) -> dict:
        try:
            return yaml.safe_load((self._dir(name) / "live.yaml").read_text()) or {}
        except Exception:
            return {}

    # ---- copies
    def save(self, name: str, base: str = "") -> list:
        """Copy the files in use into profile `name` (its settings, as they are now)."""
        d = self._dir(name)
        d.parent.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(prefix=".live-tmp-", dir=d.parent))
        try:
            holds = copy_files(self.cfg, self.robot, tmp / "files")
            marker = self.cfg / "setups" / CURRENT
            try:
                setup = (yaml.safe_load(marker.read_text()) or {}) if marker.is_file() else {}
            except Exception:
                setup = {}
            info = {"saved": time.strftime("%Y-%m-%d %H:%M"), "robot": self.robot, "base": str(base or ""),
                    "holds": holds, "setup": setup}
            _atomic_write(tmp / "live.yaml", yaml.safe_dump(info, sort_keys=False))
            _fsync_tree(tmp)
        except Exception:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        if d.exists():
            old = Path(tempfile.mkdtemp(prefix=".live-old-", dir=d.parent))
            shutil.rmtree(old)
            d.rename(old)
            try:
                tmp.rename(d)
            except Exception:
                old.rename(d)
                shutil.rmtree(tmp, ignore_errors=True)
                raise
            else:
                shutil.rmtree(old)
        else:
            try:
                tmp.rename(d)
            except Exception:
                shutil.rmtree(tmp, ignore_errors=True)
                raise
        _fsync_dir(d.parent)
        return holds

    def load(self, name: str) -> dict:
        """Put profile `name`'s settings in place. Returns its live.yaml (base, setup, ...)."""
        d = self._dir(name)
        if not (d / "live.yaml").is_file():
            raise ProfileError(f"profile '{self.profiles.find(name)}' has no settings of its own yet")
        info = self.info(name)
        if info.get("robot") and info["robot"] != self.robot:
            raise ProfileError(f"those settings are for robot profile {info['robot']}, this GUI runs {self.robot}")
        put_in_place(self.cfg, self.robot, d / "files")
        marker = self.cfg / "setups" / CURRENT
        setup = info.get("setup") or {}
        if setup.get("name"):
            marker.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write(marker, yaml.safe_dump(setup))
        elif marker.is_file():
            marker.unlink()
            _fsync_dir(marker.parent)
        return info

    # ---- a switch that fails puts back what was there
    def snapshot(self) -> Path:
        """A copy of the files in use, the setup marker and the active marker (restore() / drop())."""
        self.profiles.dir.mkdir(parents=True, exist_ok=True)
        snap = Path(tempfile.mkdtemp(prefix=".switch-", dir=self.profiles.dir))
        try:
            copy_files(self.cfg, self.robot, snap / "files")
            for f, name in ((self.cfg / "setups" / CURRENT, "current.yaml"), (self.marker, ACTIVE),
                            (self.cfg / "base", "base")):
                if f.is_file():
                    shutil.copy2(f, snap / name)
            return snap
        except Exception:
            self.drop(snap)
            raise

    def restore(self, snap: Path):
        put_in_place(self.cfg, self.robot, snap / "files")
        for f, name in ((self.cfg / "setups" / CURRENT, "current.yaml"), (self.marker, ACTIVE),
                        (self.cfg / "base", "base")):
            if (snap / name).is_file():
                f.parent.mkdir(parents=True, exist_ok=True)
                _atomic_write(f, (snap / name).read_text())       # a new mtime: active() reads it again
            elif f.is_file():
                f.unlink()

    @staticmethod
    def drop(snap: Path):
        shutil.rmtree(snap, ignore_errors=True)

    def put_setup(self, files: Path):
        """Put a setup's files in place (a profile without settings of its own, starting from a setup)."""
        put_in_place(self.cfg, self.robot, Path(files))

    # ---- crash-safe profile switches
    def _journal(self, txn: Path, state: str, **extra):
        d = {"version": 1, "state": state, **extra}
        _atomic_write(txn / "journal.yaml", yaml.safe_dump(d, sort_keys=False))

    def begin_switch(self, name: str) -> Path:
        """Start a durable switch transaction and snapshot the complete live state."""
        self.profiles.dir.mkdir(parents=True, exist_ok=True)
        txn = Path(tempfile.mkdtemp(prefix=".switch-", dir=self.profiles.dir))
        try:
            self._journal(txn, "started", name=str(name))
            (txn / "old").mkdir()
            copy_files(self.cfg, self.robot, txn / "old" / "files")
            for f, rel in ((self.cfg / "setups" / CURRENT, "current.yaml"), (self.marker, ACTIVE),
                           (self.cfg / "base", "base")):
                if f.is_file():
                    shutil.copy2(f, txn / "old" / rel)
            _fsync_tree(txn / "old")
            self._journal(txn, "snapshotted", name=str(name))
            return txn
        except Exception:
            self.drop(txn)
            raise

    def stage_switch(self, txn: Path, files: Path, setup: Optional[dict] = None,
                     preserve_setup: bool = False):
        """Stage a complete target tree before touching the files in use."""
        new = txn / "new"
        if new.exists():
            shutil.rmtree(new)
        new.mkdir()
        shutil.copytree(Path(files), new / "files")
        if setup is not None and setup.get("name"):
            _atomic_write(new / "current.yaml", yaml.safe_dump(setup, sort_keys=False))
        elif preserve_setup and (txn / "old" / "current.yaml").is_file():
            shutil.copy2(txn / "old" / "current.yaml", new / "current.yaml")
        _fsync_tree(new)
        self._journal(txn, "staged", name=self._journal_name(txn))

    @staticmethod
    def _journal_name(txn: Path) -> str:
        try:
            return str((yaml.safe_load((txn / "journal.yaml").read_text()) or {}).get("name") or "")
        except Exception:
            return ""

    def apply_switch(self, txn: Path):
        self._journal(txn, "applying", name=self._journal_name(txn))
        put_in_place(self.cfg, self.robot, txn / "new" / "files")
        marker = self.cfg / "setups" / CURRENT
        staged = txn / "new" / "current.yaml"
        if staged.is_file():
            _atomic_write(marker, staged.read_text())
        elif marker.is_file():
            marker.unlink()
            _fsync_dir(marker.parent)
        self._journal(txn, "applied", name=self._journal_name(txn))

    def commit_switch(self, txn: Path):
        """Publish ownership last; the journal remains until the cleanup is durable."""
        self.set_active(self._journal_name(txn))
        self._journal(txn, "committed", name=self._journal_name(txn))
        self.drop(txn)

    def abort_switch(self, txn: Path):
        self.restore(txn / "old")
        self.drop(txn)

    def recover(self, log=None):
        """Finish or undo switches left by a killed process before normal startup."""
        if not self.profiles.dir.is_dir():
            return
        for profile in (p for p in self.profiles.dir.iterdir() if p.is_dir()):
            live = profile / "live"
            olds = sorted(profile.glob(".live-old-*"), key=lambda p: p.stat().st_mtime_ns, reverse=True)
            if not live.exists() and olds:
                olds[0].rename(live)
            for stale in profile.glob(".live-tmp-*"):
                shutil.rmtree(stale, ignore_errors=True)
            for stale in olds[1:] if live.exists() else olds:
                shutil.rmtree(stale, ignore_errors=True)
        for txn in sorted(self.profiles.dir.glob(".switch-*")):
            journal = txn / "journal.yaml"
            if not journal.is_file():
                shutil.rmtree(txn, ignore_errors=True)
                continue
            try:
                d = yaml.safe_load(journal.read_text()) or {}
                name, state = str(d.get("name") or ""), str(d.get("state") or "")
                if state == "committed":
                    shutil.rmtree(txn)
                elif name and self.active().lower() == name.lower() and (txn / "new" / "files").is_dir():
                    self.apply_switch(txn)
                    self.commit_switch(txn)
                    if log:
                        log("web: recovered settings switch (completed)")
                else:
                    self.abort_switch(txn)
                    if log:
                        log("web: recovered settings switch (rolled back)")
            except Exception as e:
                if log:
                    log(f"web: settings recovery failed: {type(e).__name__}: {e}")
