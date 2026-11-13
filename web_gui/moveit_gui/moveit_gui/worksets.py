"""Each profile's own settings: the robot runs the settings of the person who has control.

The files in use (setups.members(): TCPs, fixtures, keep-out zones, F/T bias and limit, gains + payload, vials,
flasks, parts, camera calibrations, recording settings) belong to one profile at a time, named in
config_dir()/profiles/active.yaml. When someone else takes control, their settings are switched in:

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
import time
from pathlib import Path
from typing import Optional

import yaml

from .setups import CURRENT, copy_files, put_in_place
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
        tmp = self.marker.with_suffix(".tmp")
        tmp.write_text(yaml.safe_dump({"name": str(name), "at": time.strftime("%Y-%m-%d %H:%M")}))
        tmp.replace(self.marker)

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
        tmp = d.with_name("live.partial")
        if tmp.exists():
            shutil.rmtree(tmp)
        holds = copy_files(self.cfg, self.robot, tmp / "files")
        marker = self.cfg / "setups" / CURRENT
        setup = {}
        try:
            setup = (yaml.safe_load(marker.read_text()) or {}) if marker.is_file() else {}
        except Exception:
            setup = {}
        info = {"saved": time.strftime("%Y-%m-%d %H:%M"), "robot": self.robot, "base": str(base or ""),
                "holds": holds, "setup": setup}
        (tmp / "live.yaml").write_text(yaml.safe_dump(info, sort_keys=False))
        if d.exists():
            old = d.with_name("live.old")
            if old.exists():
                shutil.rmtree(old)
            d.rename(old)
            try:
                tmp.rename(d)
            except Exception:
                old.rename(d)
                raise
            else:
                shutil.rmtree(old)
        else:
            tmp.rename(d)
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
            marker.write_text(yaml.safe_dump(setup))
        elif marker.is_file():
            marker.unlink()
        return info

    def put_setup(self, files: Path):
        """Put a setup's files in place (a profile without settings of its own, starting from a setup)."""
        put_in_place(self.cfg, self.robot, Path(files))
