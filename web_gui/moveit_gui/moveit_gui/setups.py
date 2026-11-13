"""Named setups: everything that belongs to one physical configuration of the robot, saved and switched as one.

"scraping, spatula" and "pick, gripper + wrist camera" differ in their parts, TCPs, payload, F/T bias, gains,
camera calibrations, fixtures and keep-out zones. A setup is a copy of those files (config_dir()/setups/<name>/,
with setup.yaml describing it). Loading one puts its files back in place and restarts the GUI, so every store reads
them fresh; when its parts differ from the running robot model the robot stack restarts too (the GUI offers it).
Before a load, the current files are kept as an automatic setup ("_before_<name>_<time>", the newest 5 kept).

A setup also remembers the end-effector base it was made on (bases.py: gripper, F/T + gripper, ...): loading it
puts that base back, so the robot stack restarts when the base differs from the running one.

Programs, policies and recordings are not part of a setup: they are shared by every setup.
"""
from __future__ import annotations

import filecmp
import json
import re
import shutil
import time
from pathlib import Path
from typing import Dict, List, Optional

import yaml

NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _.-]{0,39}$")
AUTO_PREFIX = "_before_"
AUTO_KEEP = 5
CURRENT = "current.yaml"           # setups/current.yaml: the setup last saved or loaded
DERIVED = ("parts URDF/SRDF",)     # rebuilt from the parts: copied, but not a change of its own


class SetupError(ValueError):
    pass


def members(profile: str) -> Dict[str, str]:
    """What a setup holds: {key: path relative to config_dir()} (a trailing / = a directory)."""
    return {
        "TCPs, fixtures, vision": f"calibrations/{profile}.yaml",
        "keep-out zones": f"calibrations/{profile}.keepout.yaml",
        "F/T bias": f"ft_bias/{profile}.json",
        "controller gains + payload": f"gains/{profile}.json",
        "vial profiles": f"vials/{profile}.yaml",
        "flask profiles": f"flasks/{profile}.yaml",
        "parts": f"parts/{profile}.yaml",
        "part meshes": "parts/meshes/",
        "parts URDF/SRDF": "parts/generated/",
        "camera calibrations": f"cameras/{profile}.yaml",
    }


def _payload_files(cfg: Path, profile: str) -> List[Path]:
    return sorted((cfg / "ft_payload").glob(f"{profile}_*.json"))


def _same_tree(a: Path, b: Path) -> bool:
    """Two directories hold the same files (missing or empty = the same as each other)."""
    fa = {p.relative_to(a) for p in a.rglob("*") if p.is_file()} if a.is_dir() else set()
    fb = {p.relative_to(b) for p in b.rglob("*") if p.is_file()} if b.is_dir() else set()
    return fa == fb and all(filecmp.cmp(a / r, b / r, shallow=True) for r in fa)


VOLATILE = {"restored"}               # written by the GUI itself (ft_bias: when it put the bias back after a restart)


def _drop_volatile(v):
    if isinstance(v, dict):
        return {k: _drop_volatile(x) for k, x in v.items() if k not in VOLATILE}
    return [_drop_volatile(x) for x in v] if isinstance(v, list) else v


def _same_file(a: Path, b: Path) -> bool:
    if a.is_file() != b.is_file():
        return False
    if not a.is_file() or filecmp.cmp(a, b, shallow=False):
        return True
    if a.suffix == ".json":
        try:
            return _drop_volatile(json.loads(a.read_text())) == _drop_volatile(json.loads(b.read_text()))
        except (OSError, ValueError):
            return False
    return False


class Setups:
    def __init__(self, cfg: Path, profile: str):
        self.cfg, self.profile = Path(cfg), profile
        self.dir = self.cfg / "setups"

    def _path(self, name: str) -> Path:
        name = str(name).strip()
        if not (NAME.match(name) or re.match(r"^_before_[A-Za-z0-9_.-]{1,40}$", name)):
            raise SetupError("a setup name is 1-40 letters, digits, space, _ . - (starting with a letter or digit)")
        return self.dir / name

    def list(self) -> List[dict]:
        out = []
        if not self.dir.is_dir():
            return out
        for d in sorted(self.dir.iterdir()):
            m = d / "setup.yaml"
            if not m.is_file():
                continue
            try:
                info = yaml.safe_load(m.read_text()) or {}
            except Exception as e:
                info = {"error": str(e)}
            out.append({"name": d.name, "auto": d.name.startswith(AUTO_PREFIX), "note": info.get("note", ""),
                        "saved": info.get("saved", ""), "holds": info.get("holds", []),
                        "profile": info.get("profile", ""), "base": info.get("base", ""),
                        "error": info.get("error", "")})
        return out

    # ---- which setup is in use, and what differs from it
    def current(self) -> str:
        """The setup last saved or loaded ("" if none, or it was deleted since)."""
        try:
            n = str((yaml.safe_load((self.dir / CURRENT).read_text()) or {}).get("name") or "")
        except Exception:
            return ""
        return n if n and (self.dir / n / "setup.yaml").is_file() else ""

    def _mark(self, name: str):
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / CURRENT).write_text(yaml.safe_dump({"name": name, "at": time.strftime("%Y-%m-%d %H:%M")}))

    def present(self) -> List[str]:
        """What a save would hold now (the keys of members(), plus the last payload calibration)."""
        out = []
        for key, rel in members(self.profile).items():
            src = self.cfg / rel.rstrip("/")
            if (rel.endswith("/") and src.is_dir() and any(src.iterdir())) or (not rel.endswith("/") and src.is_file()):
                out.append(key)
        return out + (["last payload calibration"] if _payload_files(self.cfg, self.profile) else [])

    def changes(self, name: str) -> List[str]:
        """What differs between the files in use and setup `name` (keys of members(); derived files left out)."""
        files = self._path(name) / "files"
        out = []
        for key, rel in members(self.profile).items():
            if key in DERIVED:
                continue
            a, b = self.cfg / rel.rstrip("/"), files / rel.rstrip("/")
            if not (_same_tree(a, b) if rel.endswith("/") else _same_file(a, b)):
                out.append(key)
        live, kept = _payload_files(self.cfg, self.profile), sorted((files / "ft_payload").glob("*.json"))
        if bool(live) != bool(kept) or (live and not filecmp.cmp(live[-1], kept[-1], shallow=False)):
            out.append("last payload calibration")
        return out

    def info(self, name: str) -> dict:
        d = self._path(name)
        if not (d / "setup.yaml").is_file():
            raise SetupError(f"no setup '{name}'")
        return yaml.safe_load((d / "setup.yaml").read_text()) or {}

    def _copy_in(self, dest: Path, note: str, base: Optional[str] = None) -> List[str]:
        tmp = dest.with_name(dest.name + ".partial")
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir(parents=True)
        holds = []
        for key, rel in members(self.profile).items():
            src = self.cfg / rel.rstrip("/")
            if rel.endswith("/"):
                if src.is_dir() and any(src.iterdir()):
                    shutil.copytree(src, tmp / "files" / rel.rstrip("/"))
                    holds.append(key)
            elif src.is_file():
                (tmp / "files" / rel).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, tmp / "files" / rel)
                holds.append(key)
        pf = _payload_files(self.cfg, self.profile)
        if pf:
            (tmp / "files" / "ft_payload").mkdir(parents=True, exist_ok=True)
            shutil.copy2(pf[-1], tmp / "files" / "ft_payload" / pf[-1].name)
            holds.append("last payload calibration")
        info = {"version": 1, "profile": self.profile, "note": str(note or "")[:300],
                "saved": time.strftime("%Y-%m-%d %H:%M"), "holds": holds}
        if base:
            info["base"] = str(base)
        (tmp / "setup.yaml").write_text(yaml.safe_dump(info, sort_keys=False))
        if dest.exists():
            old = dest.with_name(dest.name + ".old")
            if old.exists():
                shutil.rmtree(old)
            dest.rename(old)
            tmp.rename(dest)
            shutil.rmtree(old)
        else:
            tmp.rename(dest)
        return holds

    def save(self, name: str, note: str = "", replace: bool = False, base: Optional[str] = None) -> List[str]:
        d = self._path(name)
        if d.name.startswith(AUTO_PREFIX):
            raise SetupError(f"names starting with {AUTO_PREFIX} are kept for automatic backups")
        if d.exists() and not replace:
            raise SetupError(f"a setup '{d.name}' exists (save again to replace it)")
        self.dir.mkdir(parents=True, exist_ok=True)
        holds = self._copy_in(d, note, base)
        self._mark(d.name)
        return holds

    def delete(self, name: str):
        d = self._path(name)
        if not (d / "setup.yaml").is_file():
            raise SetupError(f"no setup '{name}'")
        shutil.rmtree(d)

    def load(self, name: str, base: Optional[str] = None) -> dict:
        """Put the setup's files in place (base: the running one, kept in the automatic backup).
        Returns {"backup": name, "holds": [...], "base": the setup's base or ""}; the caller restarts."""
        d = self._path(name)
        files = d / "files"
        if not (d / "setup.yaml").is_file():
            raise SetupError(f"no setup '{name}'")
        info = yaml.safe_load((d / "setup.yaml").read_text()) or {}
        if info.get("profile") and info["profile"] != self.profile:
            raise SetupError(f"setup '{name}' is for robot profile {info['profile']}, this GUI runs {self.profile}")
        backup = f"{AUTO_PREFIX}{re.sub(r'[^A-Za-z0-9_.-]', '_', d.name)[:16]}_{time.strftime('%m%d_%H%M%S')}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self._copy_in(self.dir / backup, f"automatic: the files in use before '{d.name}' was loaded", base)
        for rel in members(self.profile).values():
            dst, src = self.cfg / rel.rstrip("/"), files / rel.rstrip("/")
            if dst.is_dir():
                shutil.rmtree(dst)
            elif dst.exists():
                dst.unlink()
            if src.is_dir():
                shutil.copytree(src, dst)
            elif src.is_file():
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
        for p in sorted((files / "ft_payload").glob("*.json")) if (files / "ft_payload").is_dir() else []:
            (self.cfg / "ft_payload").mkdir(parents=True, exist_ok=True)
            # a new name, so it is the newest file (the one SETUP shows as the last calibration)
            shutil.copy2(p, self.cfg / "ft_payload" / f"{self.profile}_{time.strftime('%Y%m%d_%H%M%S')}.json")
        self._prune()
        self._mark(d.name)
        b = str(info.get("base") or "")
        from .bases import BASES
        return {"backup": backup, "holds": info.get("holds", []), "base": b if b in BASES else ""}

    def _prune(self):
        autos = sorted((d for d in self.dir.iterdir() if d.name.startswith(AUTO_PREFIX) and d.is_dir()),
                       key=lambda d: d.stat().st_mtime)
        for d in autos[:-AUTO_KEEP]:
            shutil.rmtree(d)
