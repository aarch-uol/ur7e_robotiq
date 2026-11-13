"""SETUP > F/T + Payload > F/T bias: software offsets on the F/T broadcaster (never the ATI hardware bias)."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.core import RobotCore  # noqa: E402
from moveit_gui.fake_bridge import FakeBridge  # noqa: E402
from moveit_gui.ft_bias import PARAMS, FtBias  # noqa: E402
from moveit_gui.profile import RobotProfile  # noqa: E402
from moveit_gui.web.service import CommandError, WebService  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
NODE = "/ft/force_torque_sensor_broadcaster"
RAW = np.array([1.2, -0.8, 2.5, 0.03, -0.02, 0.01])       # what this fake ATI reads unbiased


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
    bridge.ft_raw_offset = RAW.copy()
    bridge.ft_noise = 0.005
    for d in ("progs", "pol"):
        (tmp_path / d).mkdir()
    core = RobotCore(bridge, profile, tmp_path, tmp_path / "progs", tmp_path / "pol", fake=True)
    core.ft_bias.cfg["zero_s"] = 0.3
    svc = WebService(core)
    svc.tick()
    svc.take_lease("a")
    yield svc
    svc.shutdown()
    bridge.shutdown()


def cmd(s, _command, client="a", **args):
    return s.command(client, _command, args)


def result(s, n0):
    assert wait(lambda: len(s.results) > n0)
    return s.results[-1]


def sets(s):
    return [c[2] for c in s.core.bridge.calls if c[0] == "set_parameters" and c[1] == NODE]


def reading(s, n=40):
    return np.mean([s.core.bridge.snapshot().wrench for _ in range(n)], axis=0)


def offsets(s):
    p = s.core.bridge.node_params[NODE]
    return [p[k] for k in PARAMS]


def test_reads_raw_at_start_and_anyone_may_read(s):
    f = cmd(s, "ft_bias_get", client="viewer", refresh=True)["ft_bias"]
    assert f["available"] and f["node"] == NODE and f["raw"] and f["bias"] == [0.0] * 6 and not f["lost"]
    with pytest.raises(CommandError, match="take control"):
        cmd(s, "ft_bias_zero", client="viewer")
    assert s.state()["ft_bias"]["raw"]


def test_zero_makes_the_current_load_read_zero_and_clear_brings_the_raw_values_back(s):
    before = reading(s)
    n = len(s.results)
    cmd(s, "ft_bias_zero")
    r = result(s, n)
    assert r["ok"], r["msg"]
    f = s.core.ft_bias.view()
    assert not f["raw"] and f["last"]["how"] == "zeroed"
    assert np.allclose(f["bias"], before, atol=0.02)                      # bias = what it read
    assert np.allclose(offsets(s), -np.asarray(f["bias"]), atol=1e-4)     # broadcaster offset = -bias (added)
    assert np.allclose(reading(s), 0.0, atol=0.02)                        # the GUI, payload and admittance see 0
    assert all(isinstance(v, float) for v in sets(s)[-1].values())        # DOUBLE parameters, not ints
    n = len(s.results)
    cmd(s, "ft_bias_clear")
    assert result(s, n)["ok"]
    assert s.core.ft_bias.view()["raw"] and offsets(s) == [0.0] * 6
    assert np.allclose(reading(s), before, atol=0.02)                     # back to the unbiased values
    assert any("cleared" in line for line in s.core.bridge._logs)


def test_typed_values_are_checked_before_anything_is_sent(s):
    cmd(s, "ft_bias_get", refresh=True)
    for bias, why in (([1, 2, 3], "six"), ([101, 0, 0, 0, 0, 0], "outside"), ([0, 0, 0, 0, 0, 11], "outside"),
                      (["x", 0, 0, 0, 0, 0], "six numbers"), ([float("nan")] * 6, "finite")):
        with pytest.raises(CommandError, match=why):
            cmd(s, "ft_bias_set", bias=bias)
    assert not sets(s)
    before = reading(s)
    n = len(s.results)
    cmd(s, "ft_bias_set", bias=[1, -2, 3.5, 0.1, 0, -0.05])
    assert result(s, n)["ok"]
    assert s.core.ft_bias.live == pytest.approx([1, -2, 3.5, 0.1, 0, -0.05])
    assert np.allclose(reading(s), before - [1, -2, 3.5, 0.1, 0, -0.05], atol=0.02)   # reading = raw - bias


@pytest.mark.parametrize("busy, why", [
    ("admittance", "Admittance uses the F/T reading"),
    ("force", "a force is applied"),
    ("moving", "the robot is moving"),
])
def test_refused_while_anything_uses_the_reading(s, busy, why, monkeypatch):
    c = s.core
    if busy == "admittance":
        c.mode = "admittance"
        c.update = lambda *a, **k: None
    elif busy == "force":
        monkeypatch.setattr(type(c), "force_active", property(lambda self: True))
    else:
        c.busy = True
    try:
        for command, args in (("ft_bias_zero", {}), ("ft_bias_clear", {}), ("ft_bias_set", {"bias": [0] * 6})):
            with pytest.raises(CommandError, match=why):
                cmd(s, command, **args)
        assert why in c.ft_bias.view()["why_not"]
        assert not sets(s)
    finally:
        c.busy = False


def test_zero_refuses_a_moving_arm_a_touched_tool_and_stop(tmp_path):
    bridge = FakeBridge(RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml")))
    b = FtBias({"zero_s": 0.2}, "/ft/force_torque_sensor_broadcaster/wrench", bridge, print, lambda: None)
    assert b.node == NODE                                       # derived from the wrench topic
    ok, msg = b.zero(lambda: ([0.0] * 6, False))
    assert not ok and "arm moved" in msg
    k = iter(range(10 ** 6))
    ok, msg = b.zero(lambda: ([float(next(k) % 2) * 3, 0, 0, 0, 0, 0], True))   # 3 N jumps: someone touches it
    assert not ok and "Fx moved" in msg
    ok, msg = b.zero(lambda: ([1.0] * 6, True), stopped=lambda: True)
    assert not ok and "stopped" in msg
    ok, msg = b.zero(lambda: (None, True))
    assert not ok and "no F/T data" in msg
    assert not [c for c in bridge.calls if c[0] == "set_parameters"]
    assert b.busy == ""


def test_with_auto_restore_off_a_restarted_stack_drops_the_bias_and_the_gui_only_offers_it_back(s):
    s.core.ft_bias.cfg["auto_restore"] = False                            # profile ft_bias.auto_restore: false
    n = len(s.results)
    cmd(s, "ft_bias_set", bias=[2, 0, 0, 0, 0, 0])
    assert result(s, n)["ok"]
    s.core.bridge.node_params[NODE].update({k: 0.0 for k in PARAMS})      # the broadcaster restarted with its YAML
    count = len(sets(s))
    s.core.ft_bias._last_poll = 0
    s.core.ft_bias.poll(True)
    assert wait(lambda: s.core.ft_bias.view()["lost"])
    f = s.state()["ft_bias"]
    assert f["raw"] and f["last"]["bias"][0] == 2.0
    for _ in range(3):
        s.tick()
    assert len(sets(s)) == count                                          # not put back


def test_named_sets_are_saved_used_and_survive_a_gui_restart(s):
    with pytest.raises(CommandError, match="outside"):
        cmd(s, "ft_bias_preset_save", name="bad", bias=[500, 0, 0, 0, 0, 0])
    r = cmd(s, "ft_bias_preset_save", name="gripper, no part", bias=[0.5, 0, 9.1, 0, 0.2, 0])
    assert r["ft_bias"]["presets"]["gripper, no part"]["bias"][2] == 9.1
    assert not sets(s)                                                    # saving never applies
    n = len(s.results)
    cmd(s, "ft_bias_use", name="gripper, no part")
    assert result(s, n)["ok"] and s.core.ft_bias.live[2] == pytest.approx(9.1)
    b2 = FtBias(s.core.profile.ft_bias, "", s.core.bridge, print, lambda: None,
                settings_path=s.core.ft_bias.settings_path)
    assert "gripper, no part" in b2.presets and b2.last["how"] == "set 'gripper, no part'"
    cmd(s, "ft_bias_preset_delete", name="gripper, no part")
    with pytest.raises(CommandError, match="no F/T bias set"):
        cmd(s, "ft_bias_use", name="gripper, no part")


def test_an_old_broadcaster_without_offsets_is_reported(s):
    s.core.bridge.node_params[NODE] = {}
    f = cmd(s, "ft_bias_get", refresh=True)["ft_bias"]
    assert "no offset parameters" in f["error"] and f["bias"] is None
    del s.core.bridge.node_params[NODE]
    assert "not running" in cmd(s, "ft_bias_get", refresh=True)["ft_bias"]["error"]


def test_payload_fit_is_not_applied_after_the_bias_changed(s):
    s.core.bridge.ft_raw_offset = np.zeros(6)
    s.payload.settle_s, s.payload.samples_per_pose, s.payload.sample_period_s = 0.0, 20, 0.005
    s.payload.base_velocity = 1.0
    n = len(s.results)
    cmd(s, "payload_collect")
    assert wait(lambda: (s.heartbeat("a") or True) and (s.tick() or True) and len(s.results) > n
                and not s.payload_progress["running"], 60)
    assert s.results[-1]["ok"], s.results[-1]["msg"]
    assert not s.payload_view()["result"]["bias_changed"]
    n = len(s.results)
    cmd(s, "ft_bias_set", bias=[0, 0, 1, 0, 0, 0])
    assert result(s, n)["ok"]
    assert s.payload_view()["result"]["bias_changed"]
    with pytest.raises(CommandError, match="bias changed"):
        cmd(s, "payload_apply")
    n = len(s.results)
    cmd(s, "ft_bias_clear")                                               # set it back: the fit holds again
    assert result(s, n)["ok"] and not s.payload_view()["result"]["bias_changed"]


def _load_payload(s, mass=None, cog=None):
    from moveit_gui import fake_bridge as fb
    s.core.bridge.node_params["/admittance_controller"].update({
        "gravity_compensation.frame.id": "ft_fts_link",
        "gravity_compensation.CoG.pos": list(fb.FAKE_PAYLOAD_COG if cog is None else cog),
        "gravity_compensation.CoG.force": (fb.FAKE_PAYLOAD_MASS if mass is None else mass) * 9.80665})


def test_zero_keeps_the_payload_the_admittance_controller_compensates(s):
    """Only the drift is removed: the reading stays tool weight + contact, so the controller's gravity
    compensation (and a policy trained on the raw sensor in MuJoCo) still match at every pose."""
    from moveit_gui.ft_bias import gravity_wrench
    _load_payload(s)
    c, b = s.core, s.core.bridge
    n = len(s.results)
    cmd(s, "ft_bias_zero")
    r = result(s, n)
    assert r["ok"] and "payload kept" in r["msg"] and "0.900 kg" in r["msg"], r["msg"]
    assert np.allclose(c.ft_bias.view()["bias"], RAW, atol=0.03)              # exactly the drift
    # tilt the wrist: the reading follows the tool weight, as the controller's model predicts
    q = list(b.snapshot().joints)
    assert b.move_joints(q[:4] + [q[4] + 0.6, q[5] + 0.4], 1.0, 1.0)
    T = b.lookup_transform("base_link", "ft_fts_link")
    expected = gravity_wrench(T, 0.9 * 9.80665, [0.01, 0.005, 0.06])
    assert np.allclose(reading(s), expected, atol=0.03)
    # Zero all: the old behaviour, everything reads 0
    n = len(s.results)
    cmd(s, "ft_bias_zero", plain=True)
    assert result(s, n)["ok"] and np.allclose(reading(s), 0.0, atol=0.03)


def test_zero_refuses_when_the_loaded_payload_cannot_be_read(s):
    _load_payload(s)
    s.core.bridge.node_params.pop("/admittance_controller")                   # controller not running
    n = len(s.results)
    cmd(s, "ft_bias_zero")
    r = result(s, n)
    assert not r["ok"] and "Zero all" in r["msg"]


def test_gravity_wrench_matches_calibration_math():
    from moveit_gui.ft_bias import gravity_wrench
    from moveit_gui.ft_payload import load_calibration_math
    cm = load_calibration_math()
    R = cm.rotation_matrix_from_quaternion(np.array([0.2, -0.4, 0.1, 0.89]) / np.linalg.norm([0.2, -0.4, 0.1, 0.89]))
    f, t = cm.compute_expected_wrench(R, 1.0277, np.array([0.0153, 0.0073, 0.0558]))
    assert np.allclose(gravity_wrench(R, 1.0277 * 9.80665, [0.0153, 0.0073, 0.0558]), np.concatenate([f, t]))


def _collect(s, index):
    n = len(s.results)
    cmd(s, "payload_collect", index=index)
    assert wait(lambda: (s.heartbeat("a") or True) and (s.tick() or True) and len(s.results) > n
                and not s.payload_progress["running"], 60)
    return s.results[-1]


def test_payload_poses_are_collected_one_at_a_time_and_evaluated_on_request(s):
    from moveit_gui import fake_bridge as fb
    s.payload.settle_s, s.payload.samples_per_pose, s.payload.sample_period_s = 0.0, 20, 0.005
    s.payload.base_velocity = 1.0
    r = _collect(s, 1)
    assert r["ok"] and "tilt_fwd collected (1 of 7)" in r["msg"], r["msg"]
    v = s.payload_view()
    assert v["samples"][1] and not any(v["samples"][i] for i in (0, 2, 3, 4, 5, 6))   # only that pose
    assert v["result"] is None and len(v["joints"]) == 7
    with pytest.raises(CommandError, match="at least 3"):
        cmd(s, "payload_evaluate")
    for k in (0, 3, 5):                                                    # skip the others
        assert _collect(s, k)["ok"]
    r = cmd(s, "payload_evaluate")
    assert r["ok"] and "from 4 pose(s)" in r["msg"], r["msg"]
    res = s.payload_view()["result"]
    assert abs(res["mass"] - fb.FAKE_PAYLOAD_MASS) < 0.03
    # a bias change between poses would mix two offsets: refused until Start over
    n = len(s.results)
    cmd(s, "ft_bias_set", bias=[0, 0, 1, 0, 0, 0])
    assert result(s, n)["ok"]
    with pytest.raises(CommandError, match="Start over"):
        cmd(s, "payload_collect", index=2)
    assert cmd(s, "payload_reset")["ok"] and not any(s.payload_view()["samples"])
    assert _collect(s, 2)["ok"]


def test_calibration_math_is_found_without_franka_ros2(monkeypatch, tmp_path):
    """The robot container has no ~/franka_ros2: the GUI's own copy is used."""
    import sys
    from moveit_gui import ft_payload
    monkeypatch.setenv("MOVEIT_GUI_FT_CALIBRATION_PATH", str(tmp_path / "nowhere"))
    monkeypatch.setattr(sys, "path", [p for p in sys.path if "franka_ros2" not in p])
    for k in [k for k in sys.modules if k.startswith("ft_sensor_calibration")]:
        monkeypatch.delitem(sys.modules, k)
    cm = ft_payload.load_calibration_math()
    assert cm.__name__ == "moveit_gui.vendor.ft_sensor_calibration.calibration_math"
    assert hasattr(cm, "analyze_payload_measurements")


def test_a_zero_with_the_tool_on_does_not_bend_the_payload_fit():
    """Real UR7e run (2026-11-03): zeroed at `down`, so the reading carries ~ the tool weight (9.8 N) as bias,
    past calibration_math's +-5 N bias bounds. The linear pre-fit removes it: 1.00 kg, not 0.63 kg."""
    import json
    from moveit_gui import ft_payload as fp
    d = json.load(open(os.path.join(ROOT, "test", "data", "payload_ur7e_zeroed_at_down.json")))
    cal = object.__new__(fp.PayloadCalibration)
    cal.samples = [fp.PoseSample(s["name"], s["orientation_xyzw"], s["mean_wrench"], s["std_wrench"], s["count"])
                   for s in d["samples"]]
    r = cal.evaluate()
    assert r.passed, r.quality.get("issues")
    assert abs(r.mass - 1.001) < 0.005 and abs(r.cog[2] - 0.0502) < 0.002
    assert any("Zero now" in w for w in r.quality["warnings"])


def test_admittance_is_refused_while_the_bias_does_not_match_the_loaded_payload(s):
    from moveit_gui import fake_bridge as fb
    c, b = s.core, s.core.bridge
    params = {"gravity_compensation.frame.id": "ft_fts_link",
              "gravity_compensation.CoG.pos": list(fb.FAKE_PAYLOAD_COG),
              "gravity_compensation.CoG.force": fb.FAKE_PAYLOAD_MASS * 9.80665}
    b.node_params["/admittance_controller"].update(params)
    c.gains.remember("admittance", params)                       # what Apply to admittance does

    def switch():
        ok, msg = c.switch_mode("admittance")
        if ok:
            assert c.switch_mode("position")[0]
        return ok, msg

    ok, msg = switch()                                            # raw: the sensor's own offset reads as a push
    assert not ok and "not set" in msg and "Zero now" in msg, msg
    n = len(s.results)
    cmd(s, "ft_bias_zero", plain=True)                            # Zero all: weight removed twice
    assert result(s, n)["ok"]
    ok, msg = switch()
    assert not ok and "subtract it twice" in msg, msg
    n = len(s.results)
    cmd(s, "ft_bias_zero")                                        # Zero now: payload kept
    assert result(s, n)["ok"]
    ok, msg = switch()
    assert ok, msg
    b.node_params[NODE].update({p: 0.0 for p in PARAMS})          # the stack restarted: the bias is gone
    ok, msg = switch()
    assert not ok and "lost" in msg, msg


def test_admittance_without_a_payload_is_not_blocked_by_the_bias(s):
    assert s.core.switch_mode("admittance")[0]                    # nothing loaded: nothing to double-count


def test_the_bias_comes_back_by_itself_after_a_stack_restart(s):
    c, b = s.core, s.core.bridge
    n = len(s.results)
    cmd(s, "ft_bias_zero")
    assert result(s, n)["ok"]
    kept = dict(c.ft_bias.last)
    # something uses the reading: it waits
    c.ft_bias.refusal = lambda: "Admittance uses the F/T reading"
    b.node_params[NODE].update({p: 0.0 for p in PARAMS})                      # the broadcaster restarted
    c.ft_bias._last_poll = 0.0
    c.ft_bias.poll(True)
    assert wait(lambda: not c.ft_bias._polling)
    assert c.ft_bias.lost() and offsets(s) == [0.0] * 6
    v = c.ft_bias.view()                                     # the page says it is gone and why it waits
    assert v["lost"] and v["auto_restore"] and "Admittance uses the F/T reading" in v["restore_waits"]
    # free again: restored on the next poll, as it was made
    c.ft_bias.refusal = lambda: None
    c.ft_bias._last_poll = 0.0
    c.ft_bias.poll(True)
    assert wait(lambda: not c.ft_bias._polling and not c.ft_bias.lost())
    assert np.allclose(offsets(s), -np.asarray(kept["bias"]), atol=1e-4)
    assert c.ft_bias.last["how"] == kept["how"] and c.ft_bias.last.get("restored")
    assert any("restored after the F/T broadcaster restarted" in line for line in b._logs)


def test_the_saved_payload_file_and_bias_are_shown_without_a_new_calibration(s):
    """After a restart nothing is evaluated in this session, but SETUP still shows what is kept: the payload the
    GUI restores on the controller, the last calibration file and the last bias."""
    import json
    from moveit_gui.calibration import config_dir
    sv = s.payload_view()["saved"]
    assert sv["controller"] is None and sv["file"] is None and sv["bias"] is None
    s.core.gains.remember("admittance", {"gravity_compensation.frame.id": "ft_fts_link",
                                         "gravity_compensation.CoG.pos": [0.0001, 0.0, 0.0502],
                                         "gravity_compensation.CoG.force": 9.8193})
    d = config_dir() / "ft_payload"
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{s.core.profile.name}_20261103_164836.json").write_text(json.dumps(
        {"mass_kg": 1.0007, "cog_m": [0.0001, 0.0, 0.0502], "passed": True, "samples": [{}] * 7}))
    n = len(s.results)
    cmd(s, "ft_bias_zero")
    assert result(s, n)["ok"]
    sv = s.payload_view()["saved"]
    assert s.payload_view()["result"] is None
    assert abs(sv["controller"]["mass"] - 1.0013) < 1e-3 and sv["controller"]["frame"] == "ft_fts_link"
    assert sv["controller"]["at"] and sv["controller"]["node"]
    assert sv["file"]["passed"] and sv["file"]["poses"] == 7 and sv["file"]["at"] == "2026-11-03 16:48"
    assert sv["bias"]["in_use"] and sv["bias"]["how"].startswith("zeroed")
    s.core.bridge.node_params[NODE].update({p: 0.0 for p in PARAMS})          # the stack restarted
    s.core.ft_bias.read()
    assert not s.payload_view()["saved"]["bias"]["in_use"]


def test_the_log_says_what_the_broadcaster_has_at_gui_start_and_when_it_is_missing(s):
    """A GUI restart logs, once, whether the sensor carries the saved bias (the GUI log is the only record)."""
    c, b = s.core, s.core.bridge
    n = len(s.results)
    cmd(s, "ft_bias_zero")
    assert result(s, n)["ok"]
    fb = c.ft_bias
    seen = []
    fb.log = seen.append
    fb._seen = False                                             # as in a new GUI process
    b.node_params[NODE].update({p: 0.0 for p in PARAMS})         # ... after the stack restarted
    fb.read()
    assert len(seen) == 1 and "NOT the saved one" in seen[0] and "re-applying it" in seen[0]
    fb.read()
    assert len(seen) == 1                                        # once, not every poll
    orig = b.get_node_parameters
    b.get_node_parameters = lambda *a: None                      # the broadcaster is not up (yet)
    fb.read(); fb.read()
    assert len(seen) == 2 and "is not running" in seen[1]
    b.get_node_parameters = orig
    fb.read()                                                    # back: says what it has again
    assert len(seen) == 3 and seen[2].startswith("F/T bias on")


def test_a_failed_payload_fit_is_used_only_after_the_operator_accepts_it(s, tmp_path):
    import json
    from moveit_gui import ft_payload as fp
    cal = s.payload
    cal.result = fp.PayloadResult(1.18, [0.0, -0.015, 0.07],
                                  {"passed": False, "issues": ["torque noise 0.0300 > 0.0100 Nm"]}, {})
    assert s.payload_view()["result"]["can_apply"] and not s.payload_view()["result"]["accepted"]
    ok, msg = cal.apply_to_admittance("position")
    assert not ok and "accept" in msg
    with pytest.raises(CommandError, match="confirm"):
        cmd(s, "payload_apply")
    cmd(s, "payload_save", accept_failed=True)
    assert cal.result.accepted and s.payload_view()["result"]["accepted"]
    saved = json.loads(cal.saved_path.read_text())
    assert saved["passed"] is False and saved["accepted_despite_failed_checks"] is True
    assert cal.apply_to_admittance("position")[0]
