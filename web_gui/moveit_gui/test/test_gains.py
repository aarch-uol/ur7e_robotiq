"""SETUP > Controller gains: read, limits, live ramp and its refusals, read-back, presets, YAML, policy check."""
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.gains import GainsError  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
ADM, IMP = "/admittance_controller", "/cartesian_impedance_controller"


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "cfg"))
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    bridge = FakeBridge(profile, joint_speed=40.0, move_time_scale=0.05)
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def cmd(s, _command, client="a", **args):
    return s.command(client, _command, args)


def sets(s, node):
    return [c[2] for c in s.core.bridge.calls if c[0] == "set_parameters" and c[1] == node]


def last_result(s):
    assert wait(lambda: s.results)
    return s.results[-1]


def test_read_is_free_and_shows_both_modes(s):
    r = cmd(s, "gains_get", client="viewer", refresh=True)                     # no control needed
    assert r["modes"]["admittance"]["live"]["stiffness"] == [250, 200, 180, 20, 20, 20]
    assert r["modes"]["impedance"]["live"]["stiffness"] == [150, 400, 400, 15, 15, 15]
    assert "admittance_controller:" in r["yaml"]["admittance"]
    with pytest.raises(CommandError, match="take control"):
        cmd(s, "gains_apply", client="viewer", mode="admittance", values={"joint_damping": [10]})


def test_limits_types_and_readonly_rows_are_checked_before_anything_is_sent(s):
    cmd(s, "gains_get", refresh=True)
    for values, why in (({"stiffness": [5000, 200, 180, 20, 20, 20]}, "outside"),
                        ({"stiffness": [250, 200]}, "6 value"),
                        ({"selected_axes": [1, 0, 0, 0, 0, 0]}, "true / false"),
                        ({"nope": [1]}, "no gain row")):
        with pytest.raises(CommandError, match=why):
            cmd(s, "gains_check", mode="admittance", values=values)
    with pytest.raises(CommandError, match="configure"):
        cmd(s, "gains_check", mode="impedance", values={"delta_tau_max": [0.2]})
    assert not sets(s, ADM)


def test_inactive_apply_is_one_call_enable_flags_first_and_read_back(s):
    cmd(s, "gains_get", refresh=True)
    cmd(s, "gains_apply", mode="impedance", values={"stiffness": [200, 400, 400, 15, 15, 15]})
    res = last_result(s)
    assert res["ok"], res["msg"]
    assert "next activation" in res["msg"]
    calls = sets(s, IMP)
    assert calls[0] == {"stiffness.update_stiffness": True, "damping.update_damping_factors": True}
    assert calls[1]["stiffness.translation.x"] == 200.0 and len(calls) == 2
    assert s.core.gains.live["impedance"]["stiffness"][0] == 200.0
    assert s.core.gains.baseline["impedance"]["stiffness"][0] == 150.0         # session start kept


def test_active_mode_ramps_and_refuses_big_steps_axes_and_a_busy_robot(s):
    s.core.mode = "admittance"
    s.core.update = lambda *a, **k: None                                       # keep the forced mode
    cmd(s, "gains_get", refresh=True)
    with pytest.raises(CommandError, match="more than 2"):
        cmd(s, "gains_apply", mode="admittance", values={"stiffness": [600, 200, 180, 20, 20, 20]})
    with pytest.raises(CommandError, match="only outside"):
        cmd(s, "gains_apply", mode="admittance", values={"selected_axes": [True, True, False, False, False, False]})
    s.core.busy = True
    with pytest.raises(CommandError, match="moving"):
        cmd(s, "gains_apply", mode="admittance", values={"joint_damping": [10]})
    s.core.busy = False
    cmd(s, "gains_apply", mode="admittance", values={"stiffness": [400, 200, 180, 20, 20, 20]})
    res = last_result(s)
    assert res["ok"] and "now" in res["msg"], res["msg"]
    xs = [c["admittance.stiffness"][0] for c in sets(s, ADM)]
    assert len(xs) == 10 and xs == sorted(xs) and xs[0] < 300 and xs[-1] == 400.0    # 1 s at 10 Hz, monotonic


def test_stop_during_a_ramp_keeps_where_it_got_and_says_so(s):
    s.core.mode = "admittance"
    s.core.update = lambda *a, **k: None
    cmd(s, "gains_get", refresh=True)
    cmd(s, "gains_apply", mode="admittance", values={"stiffness": [400, 200, 180, 20, 20, 20]})
    assert wait(lambda: s.core.gains.applying and s.core.gains.applying["step"] >= 3)
    s.stop("test")
    res = last_result(s)
    assert not res["ok"] and "STOP" in res["msg"]
    x = s.core.gains.live["admittance"]["stiffness"][0]
    assert 250 < x < 400                                                       # read back where it stopped


def test_presets_and_yaml(s, tmp_path):
    cmd(s, "gains_get", refresh=True)
    r = cmd(s, "gains_preset_save", name="soft x", mode="admittance",
            values={"stiffness": [100, 200, 180, 20, 20, 20]})
    assert r["presets"]["soft x"]["values"]["stiffness"][0] == 100
    assert (tmp_path / "cfg" / "moveit_gui" / "gains" / "ur7e_lab.json").exists()
    assert not sets(s, ADM)                                                    # saving never applies
    y = cmd(s, "gains_check", mode="admittance", values={"stiffness": [100, 200, 180, 20, 20, 20]})["yaml"]
    assert "stiffness: [100.0, 200.0, 180.0, 20.0, 20.0, 20.0]" in y
    cmd(s, "gains_preset_delete", name="soft x")
    with pytest.raises(CommandError):
        cmd(s, "gains_preset_delete", name="soft x")


def test_controller_not_running_is_reported(s):
    del s.core.bridge.node_params[IMP]
    r = cmd(s, "gains_get", refresh=True)
    assert "not running" in r["modes"]["impedance"]["error"]
    with pytest.raises(CommandError, match="read the controller first"):
        cmd(s, "gains_check", mode="impedance", values={"stiffness": [1, 1, 1, 1, 1, 1]})


def test_policy_trained_gains_are_checked_and_can_be_applied(s, tmp_path):
    (tmp_path / "pol" / "adm.yaml").write_text(
        "name: adm\nrequired_mode: admittance\ncommand: [sleep, '1']\n"
        "controller_gains:\n  admittance.stiffness: [300, 200, 180, 20, 20, 20]\n  admittance.mass: [3, 12, 10, 4, 4, 4]\n")
    s.core.policies.reload()
    req = {r["label"]: r for r in cmd(s, "policy_requirements", name="adm")["requirements"]}
    g = req["Controller gains"]
    assert g["state"] == "fail" and "trained [300" in g["detail"] and g["fix"]["cmd"] == "gains_apply_policy"
    cmd(s, "gains_apply_policy", name="adm")
    assert last_result(s)["ok"]
    req = {r["label"]: r for r in cmd(s, "policy_requirements", name="adm")["requirements"]}
    assert req["Controller gains"]["state"] == "ok"


def test_profile_rows_are_validated():
    from moveit_gui.gains import GainSet
    with pytest.raises(GainsError, match="node"):
        GainSet.from_dict("x", {"rows": []})
    with pytest.raises(GainsError, match="min / max"):
        GainSet.from_dict("x", {"node": "/n", "rows": [{"key": "k", "param": "p", "size": 6, "max": [1, 2]}]})


# ------------------------------------------------------------------ saved values, restored automatically
def test_apply_is_saved_and_restored_after_a_controller_restart(s, tmp_path):
    cmd(s, "gains_get", refresh=True)
    cmd(s, "gains_apply", mode="impedance", values={"stiffness": [200, 400, 400, 15, 15, 15]})
    assert last_result(s)["ok"]
    saved = s.core.gains.saved["impedance"]
    assert saved["stiffness.translation.x"] == 200.0 and saved["stiffness.update_stiffness"] is True
    # the controller restarts with its YAML values
    p = s.core.bridge.node_params[IMP]
    p.update({"stiffness.translation.x": 150.0, "stiffness.update_stiffness": False})
    s.core.gains._last_poll = 0
    s.core.gains.poll(True)
    assert wait(lambda: p["stiffness.translation.x"] == 200.0 and p["stiffness.update_stiffness"] is True)
    assert wait(lambda: not s.core.gains._polling)
    assert any("restored" in line for line in s.core.bridge._logs)


def test_saved_values_survive_a_gui_restart(s, tmp_path):
    from moveit_gui.gains import Gains
    cmd(s, "gains_get", refresh=True)
    cmd(s, "gains_apply", mode="admittance", values={"joint_damping": [15]})
    assert last_result(s)["ok"]
    g2 = Gains.from_profile(s.core.profile.gains, s.core.bridge, print, lambda: "position", lambda: None,
                            settings_path=s.core.gains.settings_path)
    assert g2.saved["admittance"]["admittance.joint_damping"] == 15.0


def test_switching_into_a_mode_restores_first_and_an_active_one_is_left_alone(s):
    g = s.core.gains
    g.remember("admittance", {"admittance.joint_damping": 20.0})
    p = s.core.bridge.node_params[ADM]
    ok, msg = s.core.switch_mode("admittance")
    assert ok, msg
    assert p["admittance.joint_damping"] == 20.0                     # set before the controllers were switched
    order = [c[0] for c in s.core.bridge.calls if c[0] in ("set_parameters", "switch")]
    assert order.index("set_parameters") < order.index("switch")
    s.tick()
    assert wait(lambda: (s.tick() or True) and s.core.mode == "admittance")
    p["admittance.joint_damping"] = 12.0                             # active controller changed under us
    g._last_poll = 0
    g.poll(True)
    assert wait(lambda: "differs" in g.unsynced.get("admittance", ""))
    assert p["admittance.joint_damping"] == 12.0                     # not changed behind the operator's back
    assert "differs" in cmd(s, "gains_get")["modes"]["admittance"]["unsynced"]
    cmd(s, "gains_restore", mode="admittance")                       # the operator restores: ramped live apply
    assert last_result(s)["ok"] and p["admittance.joint_damping"] == 20.0


def test_forget_and_payload_apply_is_kept(s):
    g = s.core.gains
    g.remember("admittance", {"admittance.joint_damping": 20.0})
    cmd(s, "gains_forget", mode="admittance")
    assert not g.saved.get("admittance")
    g.remember("admittance", {"gravity_compensation.CoG.force": 9.0})   # what the payload apply hands over
    assert s.payload is None or s.payload.on_applied == g.remember
