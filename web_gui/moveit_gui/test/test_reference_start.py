"""Chained admittance: a move starts at the trajectory controller's reference, not the pushed, measured joints."""
import math
import os

import pytest

from moveit_gui.control_modes import ReferenceShiftError, shift_to_reference
from moveit_gui.profile import RobotProfile

ROOT = os.path.join(os.path.dirname(__file__), "..")


def test_shift_starts_at_the_reference_and_keeps_the_path():
    names = ["a", "b"]
    pts = [[0.10, 0.20], [0.15, 0.20], [0.20, 0.25]]                 # planned from the measured (pushed) joints
    ref = {"a": 0.09, "b": 0.22, "extra": 1.0}
    out, d = shift_to_reference(pts, names, ref, math.radians(5))
    assert out[0] == pytest.approx([0.09, 0.22])                       # first point = reference: no jump
    assert d == pytest.approx([-0.01, 0.02])
    for p, q in zip(pts, out):                                         # the same path, offset by the push
        assert [y - x for x, y in zip(p, q)] == pytest.approx(d)


def test_shift_refuses_a_big_push_or_a_missing_joint():
    with pytest.raises(ReferenceShiftError, match="off its reference"):
        shift_to_reference([[0.0, 0.0]], ["a", "b"], {"a": 0.0, "b": 0.2}, math.radians(5))
    with pytest.raises(ReferenceShiftError, match="no reference for b"):
        shift_to_reference([[0.0, 0.0]], ["a", "b"], {"a": 0.0}, 1.0)
    with pytest.raises(ReferenceShiftError, match="empty"):
        shift_to_reference([], ["a"], {"a": 0.0}, 1.0)


def test_admittance_starts_moves_at_the_reference():
    p = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    rs = p.modes["admittance"].reference_start
    assert rs["state_topic"] == "/admittance_trajectory_controller/controller_state" and rs["max_shift_rad"] > 0
    assert not p.modes["position"].reference_start                     # position mode: MoveIt executes as before
