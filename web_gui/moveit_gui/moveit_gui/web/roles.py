"""Roles: what each person may do in the web GUI.

  viewer    watches only: cannot take control (STOP still works for everyone)
  operator  takes control and runs the robot: moves, programs, gripper, force, policies, recording
  admin     also changes the setup: TCPs, calibrations, zones, gains, parts, drivers, hand-eye, roles, GUI restart

Who is who: the Tailscale login when the page comes through `tailscale serve` (the request then arrives from this
PC with the Tailscale-User-Login header, which tailscale serve sets; a header from anywhere else is ignored), else the
IP address it connects from. A browser on the robot PC itself is always admin, so nobody can lock the lab out.

Stored in config_dir()/roles.yaml:
    default: operator          # anyone not listed
    people: {alice@example.com: admin, 100.101.102.103: viewer}
With no file everyone is admin, which is how the GUI behaved before roles existed.
"""
from __future__ import annotations

import threading
from pathlib import Path
from typing import Dict, Optional

import yaml

ROLES = ("viewer", "operator", "admin")
RANK = {r: i for i, r in enumerate(ROLES)}


class RolesError(ValueError):
    pass


def _clean_key(k) -> str:
    k = str(k or "").strip()
    if not k or len(k) > 120 or any(c.isspace() for c in k):
        raise RolesError(f"'{k}' is not a login or an IP address")
    return k


class Roles:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.Lock()
        self.default = "admin"
        self.people: Dict[str, str] = {}
        self.exists = False
        self.load()

    def load(self):
        with self._lock:
            self.default, self.people, self.exists = "admin", {}, self.path.exists()
            if not self.exists:
                return
            try:
                d = yaml.safe_load(self.path.read_text()) or {}
            except (OSError, yaml.YAMLError):
                self.default = "viewer"                   # a broken file must not give everyone admin
                return
            if d.get("default") in ROLES:
                self.default = d["default"]
            for k, r in (d.get("people") or {}).items():
                if r in ROLES:
                    self.people[str(k)] = r

    def save(self, default: str, people: Dict[str, str]):
        if default not in ROLES:
            raise RolesError(f"default must be one of {', '.join(ROLES)}")
        clean = {}
        for k, r in (people or {}).items():
            if r not in ROLES:
                raise RolesError(f"{k}: role must be one of {', '.join(ROLES)}")
            clean[_clean_key(k)] = r
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(yaml.safe_dump({"default": default, "people": clean}, sort_keys=True))
        tmp.replace(self.path)
        self.load()

    def role(self, login: Optional[str], ip: str, local: bool) -> str:
        if local:
            return "admin"
        with self._lock:
            for k in (login, ip):
                if k and k in self.people:
                    return self.people[k]
            return self.default

    def view(self) -> dict:
        with self._lock:
            return {"default": self.default, "people": dict(self.people), "saved": self.exists, "roles": list(ROLES)}


def at_least(role: str, need: str) -> bool:
    return RANK.get(role, 0) >= RANK[need]
