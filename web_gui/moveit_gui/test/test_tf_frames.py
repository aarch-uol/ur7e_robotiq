"""The TF frames overlay's backend command."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    (tmp_path / "progs").mkdir()
    (tmp_path / "pol").mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    yield svc
    svc.shutdown()
    bridge.shutdown()


def test_tf_frames_needs_no_lease_and_lists_frames(s):
    r = s.command("viewer", "tf_frames", {})     # a client without the lease
    assert r["ok"]
    by = {f["name"]: f for f in r["frames"]}
    assert r["base"] == s.core.profile.base_link
    assert by[r["base"]]["xyz_mm"] == [0.0, 0.0, 0.0]
    assert "tool0" in by and len(by["tool0"]["xyz_mm"]) == 3 and len(by["tool0"]["rpy_deg"]) == 3
    assert by["ft_fts_link"]["kind"] == "ft"
    active = [f for f in r["frames"] if f["kind"] == "tcp" and f["active"]]
    assert len(active) == 1 and "xyz_mm" in active[0]


def test_tf_frames_reports_unresolvable_frame(s):
    s.core.bridge.stale_frames.add("ft_fts_link")
    r = s.command("viewer", "tf_frames", {})
    f = next(f for f in r["frames"] if f["name"] == "ft_fts_link")
    assert f.get("error") and "xyz_mm" not in f


def test_tf_frames_has_unique_names(s):
    names = [f["name"] for f in s.command("viewer", "tf_frames", {})["frames"]]
    assert len(names) == len(set(names))
