"""SETUP > F/T + payload > Match simulation in the web GUI (web/ftmatch_cmds.py), on the fake robot: pushes taken
by themselves, the fit, Save per profile and switched with control."""
import json

import numpy as np
import pytest
import yaml

from moveit_gui.web.service import CommandError

from test_ft_match import SIM, SIM_TORQUE_FLIPPED, UR_SENSOR
from test_profiles import as_
from test_workcell import env, run  # noqa: F401  (the fixture)


def _policy(tmp, with_reference=True):
    rc = tmp / "robot" / "ur7e_policy_deployment.yaml"
    rc.parent.mkdir(exist_ok=True)
    rc.write_text(yaml.safe_dump({"observations": {"ft_obs_scale": [-1, 1, 1, 1, 1, 1]}}))
    if with_reference:
        (rc.parent / "sim_ft_reference.json").write_text(json.dumps({"pushes": SIM, "no_contact": {"0": [0] * 6}}))
    (tmp / "pol" / "scrape.yaml").write_text(yaml.safe_dump({
        "name": "scrape", "required_mode": "admittance", "backend": "servo_admittance",
        "command": ["true", "--robot-config", "{robot_config}", "--robot-controller", "servo_admittance"],
        "robot_config": str(rc)}))


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


def _drive(s, b, clock, wrench, seconds):
    for _ in range(int(seconds * 10)):
        b.ft_raw_offset = np.array(wrench, float)
        clock.t += 0.1
        s.tick()


def _push(s, b, clock, axis, sensor):
    assert s.ftm["axis"] == axis
    _drive(s, b, clock, [0] * 6, 0.8)                        # hands off: baseline
    assert s.ftm["caps"][axis].state == "push"
    _drive(s, b, clock, sensor, 1.5)                         # a steady push
    _drive(s, b, clock, [0] * 6, 0.1)


def _match(s, b, sensor=UR_SENSOR):
    clock = Clock()
    s.ftm_clock = clock
    b.ft_noise = 0.0
    run(s, "ftmatch_start", policy="scrape")
    for ax in ("+X", "+Y", "+Z"):
        _push(s, b, clock, ax, sensor[ax])
    return s.ftmatch_view()


def test_the_wizard_takes_three_pushes_fits_and_saves(env):
    s, b, tmp = env
    _policy(tmp)
    s.core.policies.reload()
    v = s.ftmatch_view()
    assert v["refs"] == [{"policy": "scrape", "path": str(tmp / "robot" / "sim_ft_reference.json"), "found": True}]
    v = _match(s, b)
    fit = v["fit"]
    assert fit["ok"] and fit["M"] == [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]
    assert fit["explain"][1] == "policy fy = -sensor fx"
    assert fit["rows"]["+X"]["after"][0] == pytest.approx(-10.2, abs=0.2)       # the policy sees what the sim does
    assert fit["rows"]["+X"]["before"][0] == pytest.approx(-0.4, abs=0.2)      # the old config: almost nothing
    assert v["no_contact"]["raw"] and v["tip"] and len(fit["suggested_tip"]["rpy_deg_sensor"]) == 3
    run(s, "ftmatch_save")
    saved = s.core.ft_match.load()
    assert saved["ft_obs_axes"] == fit["M"] and saved["ft_obs_scale"] == [1.0] * 6 == fit["scale"]
    assert saved["explain"][3] == "policy tx = sensor ty" and saved["torque"]["votes"]["ty"][0]["push"] == "+X"
    assert saved["no_contact"]["raw"] is not None and saved["sim_reference"].endswith("sim_ft_reference.json")
    assert not s.ftm["active"]


def test_torques_that_need_the_other_sign_are_saved_with_it(env):
    s, b, tmp = env
    _policy(tmp)
    ref = tmp / "robot" / "sim_ft_reference.json"
    ref.write_text(json.dumps({"pushes": SIM_TORQUE_FLIPPED}))
    s.core.policies.reload()
    fit = _match(s, b)["fit"]
    # the robot config has ft_obs_scale [-1, 1, 1, ...]: forces not all one sign, so tz follows tx / ty
    assert fit["ok"] and fit["scale"] == [1, 1, 1, -1, -1, -1] and "set like tx and ty" in fit["torque"]["tz"]
    assert fit["rows"]["+Y"]["after"][3] == pytest.approx(-1.92, abs=0.1)
    run(s, "ftmatch_save")
    assert s.core.ft_match.load()["ft_obs_scale"] == [1.0, 1.0, 1.0, -1.0, -1.0, -1.0]
    cmd = s.core.policies.library.command_for("scrape", log=lambda m: None)
    obs = yaml.safe_load(open(cmd[cmd.index("--robot-config") + 1]))["observations"]
    assert obs["ft_obs_scale"] == [1.0, 1.0, 1.0, -1.0, -1.0, -1.0]


def test_a_crooked_push_is_refused_and_named(env):
    s, b, tmp = env
    _policy(tmp)
    s.core.policies.reload()
    v = _match(s, b, dict(UR_SENSOR, **{"+Y": [6.0, 6.0, 0, -1.2, 1.2, 0]}))
    assert not v["fit"]["ok"] and v["fit"]["repeat"] == ["+Y"]
    with pytest.raises(CommandError, match="nothing to save"):
        run(s, "ftmatch_save")
    run(s, "ftmatch_push", axis="+Y")                         # repeat only that one
    clock = s.ftm_clock
    _push(s, b, clock, "+Y", UR_SENSOR["+Y"])
    assert s.ftmatch_view()["fit"]["ok"]


def test_record_by_hand_and_the_preconditions(env):
    s, b, tmp = env
    _policy(tmp)
    s.core.policies.reload()
    clock = Clock()
    s.ftm_clock = clock
    with pytest.raises(CommandError, match="no push is waiting"):
        run(s, "ftmatch_record")
    run(s, "ftmatch_start")
    _drive(s, b, clock, [0] * 6, 0.8)
    _drive(s, b, clock, [0, -6, 0, 0, 0, 0], 0.6)
    assert run(s, "ftmatch_record")["ok"] and s.ftm["axis"] == "+Y"
    b.ft_live = False
    s.tick()
    assert "F/T" in s.ftm["error"]
    with pytest.raises(CommandError, match="F/T"):
        run(s, "ftmatch_record")


def test_no_sim_reference_explains_and_does_not_start(env):
    s, b, tmp = env
    _policy(tmp, with_reference=False)
    s.core.policies.reload()
    assert s.ftmatch_view()["refs"][0]["found"] is False
    with pytest.raises(CommandError, match="no sim reference"):
        run(s, "ftmatch_start")
    assert not s.ftm["active"]


def test_the_result_belongs_to_the_profile_and_switches_with_control(env):
    s, b, tmp = env
    cfg = tmp / "cfg" / "moveit_gui"
    _policy(tmp)
    s.core.policies.reload()
    s.on_restart = lambda: None
    for n in ("Cenk", "Ayesha"):
        run(s, "profile_create", name=n)
    as_(s, "Cenk")
    as_(s, "Ayesha")
    run(s, "profile_activate")
    _match(s, b)
    run(s, "ftmatch_save")
    assert s.core.ft_match.load()["profile"] == "Ayesha"
    lib = s.core.policies.library
    assert lib.ft_axes_in_use()[0] == "Ayesha"
    as_(s, "Cenk")                                         # Cenk's settings: no result
    run(s, "profile_activate")
    assert s.worksets.active() == "Cenk" and s.core.ft_match.load() is None
    req = {r["label"]: r["state"] for r in run(s, "policy_requirements", name="scrape")["requirements"]}
    assert req["F/T axes"] == "warn"
    cmd = lib.command_for("scrape", log=lambda m: None)
    assert cmd[cmd.index("--robot-config") + 1] == str(tmp / "robot" / "ur7e_policy_deployment.yaml")
    assert (cfg / "profiles" / "Ayesha" / "live" / "files" / "ft_match" / "ur7e_lab.json").is_file()
    as_(s, "Ayesha")                                       # and back: hers again
    run(s, "profile_activate")
    assert s.core.ft_match.load()["ft_obs_axes"] == [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]
    cmd = lib.command_for("scrape", log=lambda m: None)
    eff = cmd[cmd.index("--robot-config") + 1]
    assert yaml.safe_load(open(eff))["observations"]["ft_obs_axes"][2] == [0, 0, -1]
