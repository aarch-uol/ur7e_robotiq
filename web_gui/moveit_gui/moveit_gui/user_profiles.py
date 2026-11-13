"""Profiles: one per person (Cenk, Ayesha, ...), so everyone keeps their own setups and screen layout.

    config_dir()/profiles/<name>/setups/<setup>/   that person's setups (setups.py: parts, TCPs, calibrations, ...)
    config_dir()/profiles/<name>/prefs.json        the browser layout: OPERATE panels, 3D view options, ...

The setups saved before profiles existed stay in config_dir()/setups/ and show as "Shared". The robot runs one set of
files whoever is using it: a profile only says where a setup is kept. A setup or a layout can be copied from another
profile.
"""
from __future__ import annotations

import json
import re
import shutil
from pathlib import Path
from typing import Dict, List

NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,23}$")
# the browser settings a profile keeps (localStorage keys of the web GUI); the rest stay per browser
PREF_KEYS = ("op-layouts1", "op-joints", "pg-jog-open", "pg-path", "v3-frames", "v3-pose", "v3-show", "v3-zones")
PREFS_MAX = 256 * 1024


class ProfileError(ValueError):
    pass


class Profiles:
    def __init__(self, cfg: Path):
        self.dir = Path(cfg) / "profiles"

    def path(self, name: str) -> Path:
        name = str(name or "").strip()
        if not NAME.match(name):
            raise ProfileError("a profile name is 1-24 letters, digits, _ or - (starting with a letter)")
        return self.dir / name

    def exists(self, name: str) -> bool:
        try:
            return self.path(name).is_dir()
        except ProfileError:
            return False

    def find(self, name: str) -> str:
        """The profile's name as stored (names are matched without case: "cenk" finds "Cenk")."""
        name = str(name or "").strip()
        for n in self.names():
            if n.lower() == name.lower():
                return n
        raise ProfileError(f"no profile '{name}'")

    def names(self) -> List[str]:
        if not self.dir.is_dir():
            return []
        return sorted((d.name for d in self.dir.iterdir() if d.is_dir() and NAME.match(d.name)), key=str.lower)

    def list(self) -> List[dict]:
        out = []
        for n in self.names():
            s = self.dir / n / "setups"
            out.append({"name": n, "setups": sum(1 for d in s.iterdir() if (d / "setup.yaml").is_file()
                                                 and not d.name.startswith("_")) if s.is_dir() else 0,
                        "layout": (self.dir / n / "prefs.json").is_file()})
        return out

    def create(self, name: str) -> str:
        p = self.path(name)
        try:
            return self.find(p.name)                       # already there (any case): use it
        except ProfileError:
            pass
        (p / "setups").mkdir(parents=True)
        return p.name

    def delete(self, name: str):
        """Only an empty profile (no setups): a person's setups are never deleted with it."""
        p = self.path(self.find(name))
        s = p / "setups"
        if s.is_dir() and any((d / "setup.yaml").is_file() for d in s.iterdir()):
            raise ProfileError(f"profile '{p.name}' still has setups: delete or copy them first")
        shutil.rmtree(p)

    def setups_dir(self, name: str) -> Path:
        return self.path(self.find(name)) / "setups"

    # ---- browser layout
    def prefs(self, name: str) -> Dict[str, str]:
        f = self.path(self.find(name)) / "prefs.json"
        try:
            d = json.loads(f.read_text())
        except (OSError, ValueError):
            return {}
        return {k: v for k, v in d.items() if k in PREF_KEYS and isinstance(v, str)} if isinstance(d, dict) else {}

    def save_prefs(self, name: str, prefs: dict) -> Dict[str, str]:
        if not isinstance(prefs, dict):
            raise ProfileError("prefs: a {key: text} map")
        d = {k: v for k, v in prefs.items() if k in PREF_KEYS and isinstance(v, str)}
        text = json.dumps(d, sort_keys=True)
        if len(text) > PREFS_MAX:
            raise ProfileError(f"the layout is too big ({len(text) // 1024} kB, at most {PREFS_MAX // 1024} kB)")
        f = self.path(self.find(name)) / "prefs.json"
        tmp = f.with_suffix(".tmp")
        tmp.write_text(text)
        tmp.replace(f)
        return d

    def copy_prefs(self, src: str, dst: str) -> Dict[str, str]:
        d = self.prefs(src)
        if not d:
            raise ProfileError(f"profile '{self.find(src)}' has no saved layout yet")
        return self.save_prefs(dst, d)
