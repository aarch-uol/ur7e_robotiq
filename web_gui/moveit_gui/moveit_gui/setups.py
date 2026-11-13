"""Named setups: everything that belongs to one physical configuration of the robot, saved and switched as one.

"scraping, spatula" and "pick, gripper + wrist camera" differ in their parts, TCPs, payload, F/T bias, gains,
camera calibrations, fixtures and keep-out zones. A setup is a copy of those files (config_dir()/setups/<name>/,
with setup.yaml describing it). Loading one puts its files back in place and restarts the GUI, so every store reads
them fresh; when its parts differ from the running robot model the robot stack restarts too (the GUI offers it).
Before a load, the current files are kept as an automatic setup ("_before_<name>_<time>", the newest 5 kept).

A setup also remembers the end-effector base it was made on (bases.py: gripper, F/T + gripper, ...): loading it
puts that base back, so the robot stack restarts when the base differs from the running one.

Programs, policies and recordings are not part of a setup (programs are kept per profile, see web/service.py).
A profile's own settings in use are kept the same way (worksets.py).

Setups are kept per person (user_profiles.py: config_dir()/profiles/<name>/setups/); the older ones in
config_dir()/setups/ are the "Shared" ones (owner ""). Whoever's setup it is, the one in use is marked in
config_dir()/setups/current.yaml with its owner.
"""
from __future__ import annotations

import filecmp
import json
import re
import shutil
import tempfile
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
        "F/T limit": "ft_limit.yaml",
        "recording settings": f"recording/{profile}.json",
    }


# Added later: a setup saved before them does not hold them, and loading it leaves those files as they are
OPTIONAL = ("F/T limit", "recording settings")


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


def copy_files(cfg: Path, profile: str, dest: Path) -> List[str]:
    """Copy the files in use (members(), and the last payload calibration) into `dest` (made empty first).
    Returns what was there. `dest/.strict` marks a complete set: put_in_place then also removes what it lacks."""
    cfg, dest = Path(cfg), Path(dest)
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    holds = []
    for key, rel in members(profile).items():
        src = cfg / rel.rstrip("/")
        if rel.endswith("/"):
            if src.is_dir() and any(src.iterdir()):
                shutil.copytree(src, dest / rel.rstrip("/"))
                holds.append(key)
        elif src.is_file():
            (dest / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest / rel)
            holds.append(key)
    pf = _payload_files(cfg, profile)
    if pf:
        (dest / "ft_payload").mkdir(parents=True, exist_ok=True)
        shutil.copy2(pf[-1], dest / "ft_payload" / pf[-1].name)
        holds.append("last payload calibration")
    (dest / ".strict").write_text("")
    return holds


def _put_in_place(cfg: Path, profile: str, files: Path):
    """Apply one prepared file set. Caller owns rollback."""
    strict = (files / ".strict").exists()
    for key, rel in members(profile).items():
        dst, src = cfg / rel.rstrip("/"), files / rel.rstrip("/")
        if key in OPTIONAL and not strict and not src.exists():
            continue
        if dst.is_dir():
            shutil.rmtree(dst)
        elif dst.exists():
            dst.unlink()
        if src.is_dir():
            shutil.copytree(src, dst)
        elif src.is_file():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    kept = sorted((files / "ft_payload").glob("*.json")) if (files / "ft_payload").is_dir() else []
    if kept and not (_payload_files(cfg, profile) and
                     filecmp.cmp(_payload_files(cfg, profile)[-1], kept[-1], shallow=False)):
        (cfg / "ft_payload").mkdir(parents=True, exist_ok=True)
        # a new name, so it is the newest file (the one SETUP shows as the last calibration)
        shutil.copy2(kept[-1], cfg / "ft_payload" / f"{profile}_{time.strftime('%Y%m%d_%H%M%S')}.json")


def put_in_place(cfg: Path, profile: str, files: Path):
    """Make the files in use those of files; restore the previous set if a copy fails midway."""
    cfg, files = Path(cfg), Path(files)
    cfg.mkdir(parents=True, exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix=".settings-rollback-", dir=cfg))
    try:
        copy_files(cfg, profile, backup)
        try:
            _put_in_place(cfg, profile, files)
        except Exception:
            _put_in_place(cfg, profile, backup)
            raise
    finally:
        shutil.rmtree(backup, ignore_errors=True)


class Setups:
    def __init__(self, cfg: Path, profile: str, owner: str = "", root: Optional[Path] = None):
        """owner: the person's profile name ("" = the shared setups); root: where that person's setups are."""
        self.cfg, self.profile, self.owner = Path(cfg), profile, str(owner or "")
        self.dir = Path(root) if root is not None else self.cfg / "setups"
        self.marker = self.cfg / "setups" / CURRENT

    def in_use(self) -> Dict[str, str]:
        """{"owner", "name"} of the setup last saved or loaded, whoever's it is (the marker only, not checked)."""
        try:
            d = yaml.safe_load(self.marker.read_text()) or {}
        except Exception:
            return {"owner": "", "name": ""}
        return {"owner": str(d.get("owner") or ""), "name": str(d.get("name") or "")}

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
                        "cameras": info.get("cameras"), "copied_from": info.get("copied_from", ""),
                        "error": info.get("error", "")})
        return out

    # ---- which setup is in use, and what differs from it
    def current(self) -> str:
        """The setup last saved or loaded ("" if none, or it was deleted since)."""
        u = self.in_use()
        n = u["name"]
        return n if n and u["owner"].lower() == self.owner.lower() and (self.dir / n / "setup.yaml").is_file() else ""

    def _mark(self, name: str):
        self.marker.parent.mkdir(parents=True, exist_ok=True)
        d = {"name": name, "at": time.strftime("%Y-%m-%d %H:%M")}
        if self.owner:
            d["owner"] = self.owner
        self.marker.write_text(yaml.safe_dump(d))

    def copy_from(self, other: "Setups", name: str, new_name: str = "", replace: bool = False) -> str:
        """Copy setup `name` of another profile (or the shared ones) into this one, as `new_name` (default: the
        same name). The files in use don't change."""
        src = other._path(name)
        if not (src / "setup.yaml").is_file():
            raise SetupError(f"no setup '{name}'" + (f" in profile {other.owner}" if other.owner else " in Shared"))
        d = self._path(new_name or src.name)
        if d.name.startswith(AUTO_PREFIX):
            raise SetupError(f"names starting with {AUTO_PREFIX} are kept for automatic backups")
        if d.resolve() == src.resolve():
            raise SetupError("that is the same setup")
        if d.exists() and not replace:
            raise SetupError(f"this profile already has a setup '{d.name}' (copy again to replace it)")
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = d.with_name(d.name + ".partial")
        if tmp.exists():
            shutil.rmtree(tmp)
        shutil.copytree(src, tmp)
        info = yaml.safe_load((tmp / "setup.yaml").read_text()) or {}
        info["copied_from"] = f"{other.owner or 'Shared'}/{src.name}"
        (tmp / "setup.yaml").write_text(yaml.safe_dump(info, sort_keys=False))
        if d.exists():
            shutil.rmtree(d)
        tmp.rename(d)
        return d.name

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
            if key in OPTIONAL and not b.exists() and not (files / ".strict").exists():
                continue
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

    def _copy_in(self, dest: Path, note: str, base: Optional[str] = None,
                 cameras: Optional[List[str]] = None) -> List[str]:
        tmp = dest.with_name(dest.name + ".partial")
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir(parents=True)
        holds = copy_files(self.cfg, self.profile, tmp / "files")
        info = {"version": 1, "profile": self.profile, "note": str(note or "")[:300],
                "saved": time.strftime("%Y-%m-%d %H:%M"), "holds": holds}
        if base:
            info["base"] = str(base)
        if cameras is not None:                          # the camera drivers it runs (SETUP > Drivers & cameras)
            info["cameras"] = [str(c) for c in cameras]
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

    def save(self, name: str, note: str = "", replace: bool = False, base: Optional[str] = None,
             cameras: Optional[List[str]] = None) -> List[str]:
        d = self._path(name)
        if d.name.startswith(AUTO_PREFIX):
            raise SetupError(f"names starting with {AUTO_PREFIX} are kept for automatic backups")
        if d.exists() and not replace:
            raise SetupError(f"a setup '{d.name}' exists (save again to replace it)")
        self.dir.mkdir(parents=True, exist_ok=True)
        holds = self._copy_in(d, note, base, cameras)
        self._mark(d.name)
        return holds

    def delete(self, name: str):
        d = self._path(name)
        if not (d / "setup.yaml").is_file():
            raise SetupError(f"no setup '{name}'")
        shutil.rmtree(d)

    def load(self, name: str, base: Optional[str] = None, cameras: Optional[List[str]] = None,
             backup_to: Optional["Setups"] = None) -> dict:
        """Put the setup's files in place (base, cameras: the running ones, kept in the automatic backup).
        Returns {"backup": name, "holds": [...], "base": the setup's base or "", "cameras": its camera drivers, or
        None for a setup saved before setups kept them}; the caller starts / stops the cameras and restarts."""
        d = self._path(name)
        files = d / "files"
        if not (d / "setup.yaml").is_file():
            raise SetupError(f"no setup '{name}'")
        info = yaml.safe_load((d / "setup.yaml").read_text()) or {}
        if info.get("profile") and info["profile"] != self.profile:
            raise SetupError(f"setup '{name}' is for robot profile {info['profile']}, this GUI runs {self.profile}")
        backup = f"{AUTO_PREFIX}{re.sub(r'[^A-Za-z0-9_.-]', '_', d.name)[:16]}_{time.strftime('%m%d_%H%M%S')}"
        bk = backup_to or self                         # the backup goes to the one loading it
        bk.dir.mkdir(parents=True, exist_ok=True)
        bk._copy_in(bk.dir / backup, f"automatic: the files in use before '{d.name}' was loaded", base, cameras)
        put_in_place(self.cfg, self.profile, files)
        bk._prune()
        self._mark(d.name)
        b = str(info.get("base") or "")
        from .bases import BASES
        cams = info.get("cameras")
        return {"backup": backup, "holds": info.get("holds", []), "base": b if b in BASES else "",
                "cameras": [str(c) for c in cams] if isinstance(cams, list) else None}

    def _prune(self):
        autos = sorted((d for d in self.dir.iterdir() if d.name.startswith(AUTO_PREFIX) and d.is_dir()),
                       key=lambda d: d.stat().st_mtime)
        for d in autos[:-AUTO_KEEP]:
            shutil.rmtree(d)
