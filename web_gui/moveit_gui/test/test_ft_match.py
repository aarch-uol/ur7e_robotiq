"""SETUP > F/T + payload > Match simulation (ft_match.py): the fit, the push detector, the reference loader and the
effective robot config for policy runs."""
import json

import numpy as np
import pytest
import yaml

from moveit_gui import ft_match as FM

# The UR7e case: tool and sensor turned 90° about z. Scraping goes along base +X = sensor -Y.
# A physical push along base +X/+Y/+Z, as the sensor reads it:
#   +X push -> sensor -Y,  +Y push -> sensor +X,  +Z push -> sensor +Z  (proper rotation, 90° about z)
# The tip is 0.2 m along sensor z, so a push there also twists the sensor: t = p x f.
TIP = np.array([0.0, 0.0, 0.2])


def _at_tip(f):
    return list(f) + np.cross(TIP, f).round(3).tolist()


UR_SENSOR = {"+X": _at_tip([0.4, -10.2, 0.6]), "+Y": _at_tip([9.6, 0.8, -0.5]), "+Z": _at_tip([0.3, -0.2, 10.4])}
# MuJoCo reports the reaction: the policy sees -push along each base axis (forces and torques)
SIM = {"+X": [-10.0, 0.0, 0.0, 0.0, -2.0, 0.0], "+Y": [0.0, -10.0, 0.0, 2.0, 0.0, 0.0],
       "+Z": [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]}
# FR3-like: the same forces, but the sim's torques have the other sign (its config had ft_obs_scale [-1,-1,-1,1,1,1])
SIM_TORQUE_FLIPPED = {k: v[:3] + [-x for x in v[3:]] for k, v in SIM.items()}
UR_M = [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]


def test_48_signed_permutations():
    P = FM.signed_permutations()
    assert len(P) == 48 and len({tuple(m.ravel()) for m in P}) == 48
    assert sum(1 for m in P if np.linalg.det(m) > 0) == 24


def test_standing_sides_give_the_plain_push_directions():
    expected = {
        "front": {"+X": ("toward", -1, "-X"), "+Y": ("left", -1, "-Y"), "+Z": ("down", -1, "-Z")},
        "behind": {"+X": ("toward", 1, "+X"), "+Y": ("left", 1, "+Y"), "+Z": ("down", -1, "-Z")},
        "left": {"+X": ("left", 1, "+X"), "+Y": ("toward", -1, "-Y"), "+Z": ("down", -1, "-Z")},
        "right": {"+X": ("left", -1, "-X"), "+Y": ("toward", 1, "+Y"), "+Z": ("down", -1, "-Z")},
    }
    for side, axes in expected.items():
        got = FM.standing_pushes(side)["pushes"]
        assert {ax: (p["key"], p["sign"], p["base"]) for ax, p in got.items()} == axes


def test_fit_picks_the_ur_matrix():
    r = FM.fit(UR_SENSOR, SIM)
    assert r["ok"], r["problems"]
    M = np.array(r["M"])
    # the sim reports the reaction, so every sign flips against the push:
    assert M.tolist() == UR_M and r["scale"] == [1, 1, 1, 1, 1, 1]
    assert FM.explain(M) == ["policy fx = sensor fy", "policy fy = -sensor fx", "policy fz = -sensor fz"]
    assert FM.explain(M, r["scale"])[3:] == ["policy tx = sensor ty", "policy ty = -sensor tx", "policy tz = -sensor tz"]
    assert [v["push"] for v in r["torque"]["votes"]["tx"]] == ["+Y"] and "tz not measured" in r["torque"]["tz"]
    assert all(p["angle_deg"] < 6 for p in r["pushes"].values())
    assert 0 <= r["rotation_deg"] < 6
    # torques use the same M
    assert FM.apply_axes(M, [1, 2, 3, 4, 5, 6]) == [2, -1, -3, 5, -4, -6]


def test_negative_pushes_fit_like_the_equivalent_positive_pushes():
    positive = FM.fit(UR_SENSOR, SIM)
    negative = {ax: [-x for x in w] for ax, w in UR_SENSOR.items()}
    signs = {ax: -1 for ax in FM.AXES}
    labels = {ax: FM.standing_pushes("front")["pushes"][ax]["label"] for ax in FM.AXES}
    result = FM.fit(negative, SIM, push_signs=signs, push_labels=labels)
    assert result["ok"] and result["M"] == positive["M"] and result["scale"] == positive["scale"]


def test_fit_with_physical_signs_is_a_rotation():
    sim = {k: [-x for x in v] for k, v in SIM.items()}       # a sim that reports the push itself
    r = FM.fit(UR_SENSOR, sim)
    assert r["ok"] and r["M"] == [[0, -1, 0], [1, 0, 0], [0, 0, 1]] and r["scale"] == [1] * 6
    assert np.linalg.det(np.array(r["M"])) == pytest.approx(1)


def test_torques_get_their_own_sign_fr3_like():
    r = FM.fit(UR_SENSOR, SIM_TORQUE_FLIPPED, old_scale=[-1, -1, -1, 1, 1, 1])
    assert r["ok"], r["problems"]
    assert r["M"] == UR_M                                   # forces as before
    # tx, ty measured -1; tz kept opposite to the forces as in the old config
    assert r["scale"] == [1, 1, 1, -1, -1, -1]
    assert "kept from the robot config" in r["torque"]["tz"]
    assert FM.apply_axes(r["M"], UR_SENSOR["+Y"], r["scale"])[3] == pytest.approx(-1.92, abs=0.01)
    r = FM.fit(UR_SENSOR, SIM_TORQUE_FLIPPED)               # no old scale: tz like tx / ty
    assert r["scale"] == [1, 1, 1, -1, -1, -1] and "set like tx and ty" in r["torque"]["tz"]


def test_a_push_too_small_to_read_the_torque_sign_is_refused():
    flat = dict(UR_SENSOR, **{"+Y": [9.6, 0.8, -0.5, 0.05, 0.1, 0.0]})   # pushed at the sensor, not at the tip
    r = FM.fit(flat, SIM)
    assert not r["ok"] and r["repeat"] == ["+Y"] and r["scale"] is None
    assert "+Y push changed tx by 0.10 Nm" in r["problems"][0]


def test_torque_signs_that_disagree_are_refused():
    sim = dict(SIM, **{"+Z": [0, 0, -10.0, -2.0, 0, 0]})   # +Z twists tx too in this sim, with the other sign
    sensor = dict(UR_SENSOR, **{"+Z": [0.3, -0.2, 10.4, 0.0, 2.0, 0.0]})
    r = FM.fit(sensor, sim)
    assert not r["ok"] and "disagrees" in r["problems"][0] and set(r["repeat"]) == {"+Y", "+Z"}


def test_fit_refuses_a_weak_push_and_names_it():
    weak = dict(UR_SENSOR, **{"+Y": [3.0, 0.2, 0.1, 0, 0, 0]})
    r = FM.fit(weak, SIM)
    assert not r["ok"] and r["repeat"] == ["+Y"] and "under 5 N" in r["problems"][0]


def test_fit_refuses_a_crooked_push_and_names_it():
    bad = dict(UR_SENSOR, **{"+Z": _at_tip([5.0, 0.0, 8.0])})   # 32° off sensor z
    r = FM.fit(bad, SIM)
    assert not r["ok"] and r["repeat"] == ["+Z"] and "+Z push" in r["problems"][0]
    assert r["M"] == [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]


def test_fit_without_all_pushes():
    r = FM.fit({"+X": UR_SENSOR["+X"]}, SIM)
    assert not r["ok"] and r["repeat"] == ["+Y", "+Z"]


def test_suggested_tip_orientation():
    s = FM.suggest_tip([[0, 1, 0], [-1, 0, 0], [0, 0, -1]])
    # tip x = sensor +y, tip y = sensor -x: 90° about z; z = x × y = sensor +z, opposite of the policy z row
    assert s["rpy_deg_sensor"] == [0.0, 0.0, 90.0] and s["z_flipped"]
    s = FM.suggest_tip([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], R_parent_sensor=np.diag([1.0, -1.0, -1.0]))
    assert not s["z_flipped"] and len(s["rpy_deg_parent"]) == 3


def test_push_capture_baseline_then_steady_push():
    c = FM.PushCapture()
    base = np.array([1.0, -2.0, 3.0, 0.1, 0.0, 0.0])
    t = 0.0
    for _ in range(8):                       # 0.7 s of no contact
        c.feed(t, base)
        t += 0.1
    assert c.state == "push" and np.allclose(c.baseline, base)
    for k in range(5):                       # a weak push: not taken
        c.feed(t, base + [0, -3, 0, 0, 0, 0])
        t += 0.1
    assert c.state == "push" and "harder" in c.note
    for k in range(13):                      # 10 N steady for 1.2 s
        c.feed(t, base + [0.2 * (k % 2), -10, 0.3, 0, 0, 0])
        t += 0.1
    assert c.state == "done" and c.delta[1] == pytest.approx(-10, abs=0.1)


def test_push_capture_waits_for_a_still_baseline_and_record_works():
    c = FM.PushCapture()
    for k in range(10):                      # someone touches the tool: no baseline
        c.feed(k * 0.1, [0, 0, 3.0 * (k % 2), 0, 0, 0])
    assert c.state == "baseline" and "hands off" in c.note
    assert not c.record(1.0)
    c = FM.PushCapture()
    for k in range(7):
        c.feed(k * 0.1, [0] * 6)
    for k in range(8, 14):
        c.feed(k * 0.1, [0, 0, 7, 0, 0, 0])
    assert c.record(1.3) and c.state == "done" and c.delta[2] == pytest.approx(7)


def test_reference_loader_and_path(tmp_path):
    ref = tmp_path / "sim_ft_reference.json"
    ref.write_text(json.dumps({"pushes": SIM, "no_contact": {"0": [0, 0, -7.8, 0, 0, 0]}, "push_n": 10}))
    d = FM.load_reference(ref)
    assert d["pushes"]["+X"][0] == -10 and d["no_contact"]["0"][2] == -7.8 and d["meta"]["push_n"] == 10
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"pushes": {"+X": [1, 2, 3, 4, 5, 6], "+Y": [1, 2, 3]}}))
    with pytest.raises(FM.FtMatchError, match="Y"):
        FM.load_reference(bad)
    with pytest.raises(FM.FtMatchError, match="no sim reference"):
        FM.load_reference(tmp_path / "none.json")

    class Man:
        sim_ft_reference = ""
        def expanded_files(self):
            return {"robot_config": str(tmp_path / "cfg" / "robot.yaml")}
    m = Man()
    assert FM.reference_path(m) == tmp_path / "cfg" / "sim_ft_reference.json"
    m.sim_ft_reference = str(ref)
    assert FM.reference_path(m) == ref


def test_effective_robot_config_overrides_only_the_ft_keys(tmp_path):
    src = tmp_path / "ur7e_policy_deployment.yaml"
    src.write_text(yaml.safe_dump({"observations": {"euler_scale": [-1, -1, 1], "ft_obs_scale": [-1, 1, 1, 1, 1, 1]},
                                   "safety": {"z_force_safety_limit": 15.0}}))
    res = {"ft_obs_axes": [[0, 1, 0], [-1, 0, 0], [0, 0, -1]], "ft_obs_scale": [1] * 6, "at": "now"}
    out = FM.effective_robot_config(src, res, tmp_path / "eff", "scrape", owner="A")
    d = yaml.safe_load(out.read_text())
    assert d["observations"] == {"euler_scale": [-1, -1, 1], "ft_obs_scale": [1.0] * 6,
                                 "ft_obs_axes": [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]}
    assert d["safety"] == {"z_force_safety_limit": 15.0}
    assert "profile 'A'" in out.read_text() and out.parent == tmp_path / "eff"
    assert yaml.safe_load(src.read_text())["observations"]["ft_obs_scale"][0] == -1     # the source is untouched


# ----------------------------------------------------------------------------- policy runs
def _lib(tmp_path, command=None):
    from moveit_gui.policies import PolicyLibrary, PolicyManifest
    rc = tmp_path / "ur7e_policy_deployment.yaml"
    rc.write_text(yaml.safe_dump({"observations": {"ft_obs_scale": [-1, 1, 1, 1, 1, 1]}}))
    for f in ("model.zip", "norm.pkl"):
        (tmp_path / f).write_text("x")
    lib = PolicyLibrary(tmp_path / "none")
    lib.effective_dir = tmp_path / "runs"
    files = {"model": str(tmp_path / "model.zip"), "normalization": str(tmp_path / "norm.pkl"), "robot_config": str(rc)}
    lib.manifests["p"] = PolicyManifest(name="p", command=command or [], required_mode="admittance", files=files,
                                        backend="servo_admittance")
    return lib, rc


RESULT = {"ft_obs_axes": [[0, 1, 0], [-1, 0, 0], [0, 0, -1]], "ft_obs_scale": [1] * 6, "at": "2026-11-11 10:00",
          "explain": ["policy fx = sensor fy"]}


@pytest.mark.parametrize("command", [None, ["runner", "--robot-config", "{robot_config}", "--robot-controller",
                                            "servo_admittance"]])
def test_policy_run_gets_the_effective_robot_config(tmp_path, command):
    lib, rc = _lib(tmp_path, command)
    lib.ft_axes = lambda: ("ProfileA", RESULT)
    logs, notes = [], []
    lib.notify = notes.append
    cmd = lib.command_for("p", log=logs.append)
    eff = cmd[cmd.index("--robot-config") + 1]
    assert eff != str(rc) and eff.startswith(str(tmp_path / "runs"))
    obs = yaml.safe_load(open(eff))["observations"]
    assert obs["ft_obs_axes"] == RESULT["ft_obs_axes"] and obs["ft_obs_scale"] == [1.0] * 6
    assert "profile 'ProfileA'" in logs[0] and notes == logs


@pytest.mark.parametrize("command", [None, ["runner", "--robot-config", "{robot_config}", "--robot-controller",
                                            "servo_admittance"]])
def test_policy_run_without_a_result_uses_the_manifest_file_and_warns(tmp_path, command):
    from types import SimpleNamespace
    from moveit_gui.policy_executor import WARN, PolicyExecutor
    lib, rc = _lib(tmp_path, command)
    lib.ft_axes = lambda: ("ProfileB", None)
    logs = []
    cmd = lib.command_for("p", log=logs.append)
    assert cmd[cmd.index("--robot-config") + 1] == str(rc) and "no Match simulation" in logs[0]
    assert not (tmp_path / "runs").exists()
    modes = SimpleNamespace(profile=SimpleNamespace(modes={"admittance": 1}))
    ex = PolicyExecutor(lib, modes, SimpleNamespace(lookup_transform=lambda a, b: None), backend=None,
                        active_tcp=lambda: "")
    req = {r.label: r for r in ex._static_requirements(lib.manifests["p"])}
    assert req["F/T axes"].state == WARN
    assert not any("F/T axes" in p for p in ex.program_problems(lib.manifests["p"]))   # a warning, not a blocker
    lib.ft_axes = lambda: ("ProfileB", RESULT)
    req = {r.label: r for r in ex._static_requirements(lib.manifests["p"])}
    assert req["F/T axes"].state == "ok" and "ProfileB" in req["F/T axes"].detail


def test_push_capture_waits_for_the_hand_to_let_go():
    c = FM.PushCapture(release_from=[0] * 6)
    for k in range(10):                      # still pushing from the previous push: steady, but not no contact
        c.feed(k * 0.1, [0, -10, 0, 0, 0, 0])
    assert c.state == "baseline" and "Let go" in c.note
    for k in range(10, 17):
        c.feed(k * 0.1, [0.1, 0, 0, 0, 0, 0])
    assert c.state == "push"
