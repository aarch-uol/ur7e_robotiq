"""Vial profiles: several named vials (centre of the bottom in the base frame, radius, height), one of them put to
work as a fixture (normally "vial", the frame programs use) with one click.

Built-in entries come from the profile's example_fixtures (e.g. "baseline" = scraping.py's target_xyz) and cannot
be changed or deleted here. Saved ones live in config_dir()/vials/<profile>.yaml. Using an entry writes an ordinary
fixture (calibration store), so programs, keep-out boxes and the preview see nothing new.
"""
from __future__ import annotations

import math
import re
import time
from pathlib import Path
from typing import Dict, List, Optional

import yaml

NAME = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_ .-]{0,39}$")


class VialProfileError(ValueError):
    pass


class VialLibrary:
    def __init__(self, path: Path, examples: Optional[dict] = None, base_link: str = "base_link"):
        self.path = Path(path)
        self.base_link = base_link
        self.builtin: Dict[str, dict] = {}
        for key, e in (examples or {}).items():
            if e.get("parent", base_link) != base_link or e.get("kind", "vial") != "vial":
                continue                                   # vials only; profiles are in the base frame
            name = str(e.get("label") or ("baseline" if key == "vial" else key))
            self.builtin[name] = {"xyz_mm": [float(v) for v in e.get("xyz_mm", [0, 0, 0])],
                                  "radius_mm": e.get("radius_mm"), "height_mm": e.get("height_mm"),
                                  "source": str(e.get("source", "profile example")), "builtin": True}
        self.saved: Dict[str, dict] = self._load()

    def _load(self) -> Dict[str, dict]:
        try:
            d = yaml.safe_load(self.path.read_text()) or {}
        except FileNotFoundError:
            return {}
        except Exception:
            return {}
        return {str(k): v for k, v in (d.get("vials") or {}).items() if isinstance(v, dict) and "xyz_mm" in v}

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("# MoveIt GUI vial profiles: centre of the vial bottom in the robot base frame (mm).\n" +
                       yaml.safe_dump({"version": 1, "vials": self.saved}, sort_keys=False))
        tmp.replace(self.path)

    def entries(self) -> Dict[str, dict]:
        return {**{n: dict(v) for n, v in self.builtin.items()}, **{n: {**v, "builtin": False} for n, v in self.saved.items()}}

    def get(self, name: str) -> dict:
        e = self.entries().get(name)
        if e is None:
            raise VialProfileError(f"no vial profile '{name}'")
        return e

    def save(self, name: str, xyz_mm, radius_mm=None, height_mm=None, source: str = "") -> dict:
        name = str(name).strip()
        if not NAME.match(name):
            raise VialProfileError("a vial name is 1-40 letters, digits, space, _ . -")
        if name in self.builtin:
            raise VialProfileError(f"'{name}' is built in (robot profile): pick another name")
        xyz = [float(v) for v in xyz_mm]
        if len(xyz) != 3 or not all(math.isfinite(v) and abs(v) < 3000 for v in xyz):
            raise VialProfileError("the centre needs x, y, z in mm (within 3 m)")
        def opt(v, what):
            if v in (None, ""):
                return None
            v = float(v)
            if not 0 < v < 500:
                raise VialProfileError(f"{what} must be in (0, 500) mm")
            return round(v, 2)
        e = {"xyz_mm": [round(v, 2) for v in xyz], "radius_mm": opt(radius_mm, "radius"),
             "height_mm": opt(height_mm, "height"), "source": str(source or "typed"),
             "saved": time.strftime("%Y-%m-%d %H:%M")}
        self.saved[name] = e
        self._write()
        return e

    def delete(self, name: str):
        if name in self.builtin:
            raise VialProfileError(f"'{name}' is built in (robot profile) and cannot be deleted")
        if self.saved.pop(name, None) is None:
            raise VialProfileError(f"no vial profile '{name}'")
        self._write()

    def view(self, fixtures: dict) -> List[dict]:
        """Every profile, with the fixtures it is in use as (same name in the fixture meta and same place)."""
        out = []
        for n, e in self.entries().items():
            used = []
            for fname, fx in fixtures.items():
                if fx.meta.get("vial_profile") != n or fx.parent != self.base_link:
                    continue
                p = [float(v) * 1000.0 for v in fx.T_parent_fixture[:3, 3]]
                if all(abs(a - b) < 0.05 for a, b in zip(p, e["xyz_mm"])):
                    used.append(fname)
            out.append({"name": n, **e, "in_use_as": used})
        return out
