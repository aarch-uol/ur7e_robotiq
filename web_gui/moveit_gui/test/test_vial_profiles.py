"""Several vials: named profiles (built-in baseline = scraping.py's), one of them used as the `vial` fixture."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from test_program_force import cmd, s, tcp_base  # noqa: E402,F401
from moveit_gui.web.service import CommandError, WebService  # noqa: E402


def names(r):
    return {v["name"]: v for v in r["vials"]}


def test_the_baseline_is_built_in_and_can_be_used_as_the_vial(s):
    v = names(cmd(s, "vial_profiles"))
    b = v["baseline"]
    assert b["builtin"] and b["xyz_mm"] == [525.0, 16.0, 40.0] and b["radius_mm"] == 17.0 and b["in_use_as"] == []
    r = cmd(s, "vial_profile_use", name="baseline")
    assert r["ok"] and names(r)["baseline"]["in_use_as"] == ["vial"]
    assert np.allclose(s.core.geo.T_base_frame("vial")[:3, 3], [0.525, 0.016, 0.040])
    with pytest.raises(CommandError, match="built in"):
        cmd(s, "vial_profile_delete", name="baseline")
    with pytest.raises(CommandError, match="built in"):
        cmd(s, "vial_profile_save", name="baseline", source="typed", xyz_mm=[0, 0, 0])


def test_switch_between_saved_vials_and_they_survive_a_restart(s):
    cmd(s, "vial_profile_save", name="rack A", source="typed", xyz_mm=[400, -100, 20], radius_mm=12, height_mm=45)
    cmd(s, "vial_profile_save", name="rack B", source="typed", xyz_mm=[450, 120, 20], radius_mm=12)
    cmd(s, "vial_profile_use", name="rack A")
    assert np.allclose(s.core.geo.T_base_frame("vial")[:3, 3], [0.4, -0.1, 0.02])
    r = cmd(s, "vial_profile_use", name="rack B")
    v = names(r)
    assert v["rack B"]["in_use_as"] == ["vial"] and v["rack A"]["in_use_as"] == []
    assert s.core.store.fixtures["vial"].meta["radius_m"] == pytest.approx(0.012)
    cmd(s, "vial_profile_use", name="rack A", fixture="vial_2")              # a second vial for one program
    assert set(names(cmd(s, "vial_profiles"))["rack A"]["in_use_as"]) == {"vial_2"}
    # a new GUI (restart) reads them back
    s2 = WebService(s.core)
    try:
        v = names(s2.cmd_vial_profiles())
        assert v["rack A"]["height_mm"] == 45 and v["rack B"]["height_mm"] is None
        assert v["rack B"]["in_use_as"] == ["vial"]
    finally:
        s2.shutdown()
    cmd(s, "vial_profile_delete", name="rack A")
    assert "rack A" not in names(cmd(s, "vial_profiles"))
    assert "vial_2" in s.core.store.fixtures                                  # the fixture stays


def test_a_touched_vial_and_the_current_fixture_can_be_kept_as_profiles(s):
    p = tcp_base(s)[:3, 3]
    for k, (dx, dy) in enumerate([(0.01, 0), (-0.01, 0), (0, 0.01)]):
        s.vial_points[["Rim 1", "Rim 2", "Rim 3"][k]] = [p[0] + dx, p[1] + dy, p[2]]
    s.vial_points["Bottom"] = [p[0], p[1], p[2] - 0.04]
    r = cmd(s, "vial_profile_save", name="touched", source="touch", use=True)
    e = names(r)["touched"]
    assert e["radius_mm"] == pytest.approx(10.0, abs=0.01) and e["height_mm"] == pytest.approx(40.0, abs=0.01)
    assert e["in_use_as"] == ["vial"]
    r = cmd(s, "vial_profile_save", name="copy", source="fixture", fixture="vial")
    assert names(r)["copy"]["xyz_mm"] == e["xyz_mm"] and names(r)["copy"]["radius_mm"] == pytest.approx(10.0)
    with pytest.raises(CommandError, match="name"):
        cmd(s, "vial_profile_save", name="", source="typed", xyz_mm=[0, 0, 0])
