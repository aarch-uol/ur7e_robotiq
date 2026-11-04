"""End-effector bases: what is bolted to the flange as a fixed, unedited part of the robot model.

The robot's URDF / SRDF xacro (ur7e_robotiq) build one of four bases from UR7E_EE_CONFIG; the launcher
(ur7e_gui.sh) sets it from config_dir()/base at every stack start. The GUI never edits a base: SETUP > Parts adds
parts on top of it, and a setup remembers the base it was made on. Switching the base means a robot stack restart
(new URDF, SRDF and controllers).
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

BASES = {
    "ft_gripper": ("F/T sensor + gripper", "ATI F/T sensor on the flange, Robotiq 2F-85 on it"),
    "gripper": ("Gripper", "Robotiq 2F-85 straight on the flange (tool0)"),
    "ft": ("F/T sensor", "ATI F/T sensor on the flange; your own tool goes on it as parts"),
    "none": ("Bare arm", "nothing on the flange; your tool goes on tool0 as parts"),
}
DEFAULT = "ft_gripper"
FT_LINK, GRIPPER_LINK = "ft_fts_link", "robotiq_85_base_link"
FILE = "base"


class BaseError(ValueError):
    pass


def check(base) -> str:
    b = str(base or "").strip()
    if b not in BASES:
        raise BaseError(f"the base is one of {', '.join(BASES)}, not '{b}'")
    return b


def has_ft(base: str) -> bool:
    return base in ("ft", "ft_gripper")


def has_gripper(base: str) -> bool:
    return base in ("gripper", "ft_gripper")


def from_model(model: Optional[dict]) -> Optional[str]:
    """The base a robot model was built with, from its links (None without a model)."""
    if not model or not model.get("links"):
        return None
    ft, gr = FT_LINK in model["links"], GRIPPER_LINK in model["links"]
    return {(True, True): "ft_gripper", (False, True): "gripper", (True, False): "ft", (False, False): "none"}[(ft, gr)]


def running(model: Optional[dict]) -> str:
    """The base of the running stack: the launcher's UR7E_EE_CONFIG, else what the robot model shows, else the
    default."""
    env = os.environ.get("UR7E_EE_CONFIG", "").strip()
    if env in BASES:
        return env
    return from_model(model) or DEFAULT


def read_next(cfg: Path) -> Optional[str]:
    """The base the launcher uses at the next stack start (None: no file, so the default)."""
    try:
        b = (Path(cfg) / FILE).read_text().strip()
    except OSError:
        return None
    return b if b in BASES else None


def write_next(cfg: Path, base: str):
    p = Path(cfg) / FILE
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(check(base) + "\n")
    tmp.replace(p)


def view(cfg: Path, model: Optional[dict]) -> dict:
    run = running(model)
    nxt = read_next(cfg) or DEFAULT
    return {"running": run, "next": nxt, "model": from_model(model),
            "choices": [{"name": k, "label": v[0], "text": v[1]} for k, v in BASES.items()]}
