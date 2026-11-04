"""Generate a "scrape around the vial" program from one taught pose.

This is the UR/GUI form of the FR3 experiment in sample_scraping/scripts/policy_scraping.py: go above the
vial, then for each angle around the rim move to the contact pose, run the policy for a fixed time and move
back up. The FR3 script built its orientation from hard-coded RPY constants for fr3_hand_tcp. Here nothing
is hard-coded: the operator teaches ONE contact pose by hand (freedrive), in the vial fixture frame, and
every other angle is that pose rotated about the fixture's Z axis.

The vial fixture saved by SETUP > Vial Geometry is base-aligned with its origin at the vial's bottom
centre, so its Z axis is the vial axis (under the same level-rim, vertical-vial assumption as the fit).

The result is an ordinary program (schema v2): every pose is a named waypoint the operator can inspect,
edit or re-record in PROGRAMS before running.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from . import transforms as tf
from .geometry import PoseSpec
from .program import Program, ProgramWaypoint, Step


class ScrapePatternError(ValueError):
    pass


@dataclass
class ScrapePattern:
    start: PoseSpec                 # taught contact pose, T_vial_tcp
    policy: str
    policy_mode: str                # control mode the policy needs (manifest required_mode)
    motion_mode: str                # mode for the MoveIt moves; == policy_mode when that mode supports MoveIt
    duration: float = 8.0           # s per angle
    step_deg: float = 10.0
    count: int = 37                 # angles in one pass, including the start (37 x 10 deg = full circle + 1)
    back_and_forth: bool = False    # second pass in the opposite direction, as the FR3 script did
    approach_m: float = 0.05        # above the contact pose, along the vial axis
    velocity: float = 0.1           # MoveIt velocity scaling for the moves
    contact_velocity: float = 0.03  # for the last, linear move onto the rim

    def angles(self) -> List[float]:
        if self.count < 1:
            raise ScrapePatternError("need at least one angle")
        fwd = [i * self.step_deg for i in range(self.count)]
        return fwd + (fwd[::-1] if self.back_and_forth else [])


def _rotated(start: PoseSpec, deg: float, lift_m: float = 0.0) -> PoseSpec:
    Rz = tf.from_rotation(np.array([[math.cos(math.radians(deg)), -math.sin(math.radians(deg)), 0.0],
                                    [math.sin(math.radians(deg)), math.cos(math.radians(deg)), 0.0],
                                    [0.0, 0.0, 1.0]]))
    T = Rz @ start.T()
    T[2, 3] += lift_m
    return PoseSpec.from_T(start.frame, start.tcp, T)


def _name(kind: str, deg: float) -> str:
    d = round(deg) % 360
    return f"{kind}_{d:03d}"


def build(p: ScrapePattern, name: str, robot: str) -> Program:
    if p.duration <= 0:
        raise ScrapePatternError("policy duration must be positive")
    if p.approach_m <= 0:
        raise ScrapePatternError("approach height must be positive")
    wps: Dict[str, ProgramWaypoint] = {}
    for deg in p.angles():
        for kind, lift in (("above", p.approach_m), ("touch", 0.0)):
            n = _name(kind, deg)
            if n not in wps:
                wps[n] = ProgramWaypoint(n, pose=_rotated(p.start, deg, lift))
    switch = p.policy_mode != p.motion_mode
    steps: List[Step] = [Step("controller", {"mode": p.motion_mode})]
    first = True
    for deg in p.angles():
        above, touch = _name("above", deg), _name("touch", deg)
        # Planned move to the first approach pose; afterwards only short linear moves near the vial.
        steps.append(Step("move_pose", {"target": above, "cartesian": not first, "velocity": p.velocity}))
        first = False
        steps.append(Step("move_pose", {"target": touch, "cartesian": True, "velocity": p.contact_velocity}))
        if switch:
            steps.append(Step("controller", {"mode": p.policy_mode}))
        steps.append(Step("policy", {"policy": p.policy, "duration": float(p.duration)}))
        if switch:
            steps.append(Step("controller", {"mode": p.motion_mode}))
        steps.append(Step("move_pose", {"target": above, "cartesian": True, "velocity": p.contact_velocity}))
    return Program(name=name, robot=robot, steps=steps, waypoints=wps)


# ------------------------------------------------------------------ force baseline (no policy)
PRESS_AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


@dataclass
class ScrapeBaseline:
    """The scripted baseline the learned policy is compared with: at each angle, find the wall, press with a
    fixed force and stroke along the vial axis, then release and lift. Same taught pose and angles as
    ScrapePattern; every number is a step parameter the operator can change in PROGRAMS afterwards."""
    start: PoseSpec                 # taught contact pose (tool touching the wall), T_vial_tcp
    force_mode: str                 # a mode that takes a force AND MoveIt moves (Admittance)
    motion_mode: str                # free-air moves and the F/T zero (Position)
    press_axis: str = "+z"          # TCP axis the tool presses into the wall along
    force_n: float = 5.0            # pressing force
    contact_n: float = 2.0          # |ΔF| that counts as touching the wall
    standoff_mm: float = 3.0        # each angle starts this far off the wall, then searches for it
    search_mm: float = 10.0         # how far past the taught pose to search
    stroke_mm: float = 15.0         # along the vial axis: down this far, then back up
    strokes: int = 2
    step_deg: float = 10.0
    count: int = 37
    back_and_forth: bool = False
    approach_m: float = 0.05
    velocity: float = 0.1
    contact_velocity: float = 0.02
    stroke_velocity: float = 0.02

    def angles(self) -> List[float]:
        return ScrapePattern(self.start, "", "", "", step_deg=self.step_deg, count=self.count,
                             back_and_forth=self.back_and_forth).angles()


def build_baseline(p: ScrapeBaseline, name: str, robot: str, max_force_n: float = 20.0) -> Program:
    if p.press_axis not in PRESS_AXES:
        raise ScrapePatternError(f"press axis is one of {', '.join(PRESS_AXES)}")
    if not 0 < p.force_n <= max_force_n:
        raise ScrapePatternError(f"pressing force must be in (0, {max_force_n:g}] N (profile force limit)")
    if not 0 < p.contact_n <= 50:
        raise ScrapePatternError("contact threshold must be in (0, 50] N")
    if p.strokes < 1 or p.stroke_mm <= 0:
        raise ScrapePatternError("need at least one stroke of a positive length")
    if p.approach_m <= 0 or p.standoff_mm < 0 or p.search_mm <= 0:
        raise ScrapePatternError("approach, standoff and search distances must be positive")
    axis = np.asarray(PRESS_AXES[p.press_axis], float)
    wps: Dict[str, ProgramWaypoint] = {}
    for deg in p.angles():
        above, near = _name("above", deg), _name("near", deg)
        if above in wps:
            continue
        wps[above] = ProgramWaypoint(above, pose=_rotated(p.start, deg, p.approach_m))
        T = _rotated(p.start, deg).T()
        T[:3, 3] -= T[:3, :3] @ axis * (p.standoff_mm / 1000.0)        # back off the wall along the press axis
        wps[near] = ProgramWaypoint(near, pose=PoseSpec.from_T(p.start.frame, p.start.tcp, T))
    first_above = _name("above", p.angles()[0])
    steps: List[Step] = [
        Step("comment", {"text": f"Scrape baseline: {p.force_n:g} N along tool {p.press_axis}, "
                                 f"{p.strokes} x {p.stroke_mm:g} mm strokes per angle"}),
        Step("controller", {"mode": p.motion_mode}),
        Step("move_pose", {"target": first_above, "cartesian": False, "velocity": p.velocity}),
        Step("zero_ft", {}),                                   # in free air, before the F/T-driven mode
        Step("controller", {"mode": p.force_mode}),
    ]
    for k, deg in enumerate(p.angles()):
        above, near = _name("above", deg), _name("near", deg)
        body = []
        if k:
            body.append(Step("move_pose", {"target": above, "cartesian": True, "velocity": p.velocity}))
        body += [
            Step("move_pose", {"target": near, "cartesian": True, "velocity": p.contact_velocity}),
            Step("move_until_contact", {"direction": axis.tolist(), "frame": "tool",
                                        "distance_mm": float(p.standoff_mm + p.search_mm),
                                        "force_n": float(p.contact_n), "velocity": p.contact_velocity,
                                        "on_no_contact": "fail"}),
            Step("force", {"wrench": (axis * p.force_n).tolist() + [0.0, 0.0, 0.0], "frame": "tool"}),
            Step("loop", {"times": int(p.strokes)}, [
                Step("move_relative", {"offset_mm": [0.0, 0.0, -float(p.stroke_mm)], "rotate_deg": [0.0, 0.0, 0.0],
                                       "frame": p.start.frame, "cartesian": True, "velocity": p.stroke_velocity}),
                Step("move_relative", {"offset_mm": [0.0, 0.0, float(p.stroke_mm)], "rotate_deg": [0.0, 0.0, 0.0],
                                       "frame": p.start.frame, "cartesian": True, "velocity": p.stroke_velocity}),
            ]),
            Step("force_off", {}),
            Step("move_pose", {"target": above, "cartesian": True, "velocity": p.contact_velocity}),
        ]
        steps.append(Step("folder", {"name": f"angle {round(deg) % 360:03d}°"}, body))
    steps.append(Step("controller", {"mode": p.motion_mode}))
    return Program(name=name, robot=robot, steps=steps, waypoints=wps)
