"""Web pivot calibration uses the desktop solver and never commands robot motion."""
import numpy as np
import pytest

from moveit_gui import transforms as tf
from moveit_gui.web.service import CommandError
from test_web import env  # noqa: F401


def test_pivot_capture_solve_save(env, monkeypatch):
    s, bridge, _, _ = env
    token = s.take_lease("", "operator")["token"]
    parent = "tool0"
    tip = np.array([0.01, -0.02, 0.14])
    fixed = np.array([0.4, 0.2, 0.3])
    poses = []
    for rpy in ((0, 0, 0), (30, 0, 0), (0, 35, 0), (0, 0, 40), (-25, 20, 15), (25, -30, 20)):
        R = tf.mm_deg_to_T([0, 0, 0], rpy)[:3, :3]
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = fixed - R @ tip
        poses.append(T)
    it = iter(poses)
    monkeypatch.setattr(s.core.geo, "current_T_base_tcp", lambda _: next(it))
    before = list(bridge.calls)
    for _ in poses:
        s.command(token, "pivot_capture", {"parent": parent})
    assert bridge.calls == before
    result = s.command(token, "pivot_solve", {})["pivot"]["result"]
    assert np.allclose(result["xyz_mm"], tip * 1000, atol=0.001)
    assert result["rms_mm"] == 0
    s.command(token, "pivot_save", {"name": "my_tip", "rpy_deg": [0, 0, 0]})
    stored = s.core.store.tcps["my_tip"]
    assert stored.meta["method"] == "pivot" and stored.meta["sample_count"] == 6
    assert np.allclose(stored.T_parent_tcp[:3, 3], tip)


def test_pivot_invalidates_solution_and_checks_lease(env, monkeypatch):
    s, _, _, _ = env
    token = s.take_lease("", "operator")["token"]
    with pytest.raises(CommandError, match="take control"):
        s.command("viewer", "pivot_capture", {"parent": "tool0"})
    with pytest.raises(CommandError, match="insufficient"):
        s.command(token, "pivot_solve", {})
    with pytest.raises(CommandError, match="solve"):
        s.command(token, "pivot_save", {"name": "tip", "rpy_deg": [0, 0, 0]})
    monkeypatch.setattr(s.core.geo, "current_T_base_tcp", lambda _: np.eye(4))
    s.command(token, "pivot_capture", {"parent": "tool0"})
    assert len(s.command("viewer", "pivot_get", {})["pivot"]["samples"]) == 1
    s.command(token, "pivot_remove", {"index": 0})
    assert not s.command(token, "pivot_get", {})["pivot"]["samples"]
