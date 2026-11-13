"""Teach by hand > Path > Insert as Replay: the joint motion recorded and run back as one trajectory (replay.py)."""
import math
import time

import numpy as np
import pytest
import yaml

from moveit_gui import replay as rp
from moveit_gui.keepout import make_box
from moveit_gui.program import Program
from moveit_gui.web.service import CommandError

from test_program_force import cmd, s, tick_until, wait  # noqa: F401  (s: the fake-robot WebService fixture)
from test_program_franka import _chain_urdf


# ------------------------------------------------------------------ maths
def _hand(n=600, dt=0.01, seed=0):
    """A hand-like motion: still, a smooth swing of all joints, still again (with a little noise)."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) * dt
    s = np.clip((t - 1.0) / 3.0, 0, 1)
    s = s * s * (3 - 2 * s)
    Q = np.outer(s, [0.6, -0.4, 0.5, 0.3, -0.2, 0.8]) + 0.1 * np.sin(2 * np.pi * 0.4 * t)[:, None] * (s * (1 - s))[:, None]
    return t, Q + rng.normal(0, 1e-5, Q.shape)


def test_trim_and_decimate_keep_every_sample_within_the_tolerance():
    t, Q = _hand()
    tt, QQ = rp.trim(t, Q)
    assert tt[0] == 0.0 and tt[-1] < t[-1] - 1.5                 # the standstill before and after is cut
    keep = rp.decimate(tt, QQ, rp.JOINT_TOL_RAD)
    assert len(keep) < len(tt) / 3
    for k in range(len(tt)):                                      # linear between kept points: within the tolerance
        lin = np.array([np.interp(tt[k], tt[keep], QQ[keep, j]) for j in range(6)])
        assert np.max(np.abs(lin - QQ[k])) <= rp.JOINT_TOL_RAD + 1e-12
    out = rp.from_recording(list(zip(t, Q.tolist())))
    assert out["points"][0][0] == 0.0 and len(out["points"][0]) == 7 and out["samples"] > 200
    with pytest.raises(rp.ReplayError, match="did not move"):
        rp.from_recording([(0.0, [0] * 6), (0.01, [0] * 6), (0.02, [0] * 6)])


def test_a_pause_in_the_motion_keeps_its_timing():
    t = np.arange(0, 3, 0.01)
    q = np.interp(t, [0, 1, 2, 3], [0, 0.5, 0.5, 1.0])         # move, stand 1 s, move
    keep = rp.decimate(t, np.c_[q, q], rp.JOINT_TOL_RAD)
    assert any(abs(t[k] - 1.0) < 0.011 for k in keep) and any(abs(t[k] - 2.0) < 0.011 for k in keep)


def test_fit_limits_slows_a_too_fast_motion_uniformly():
    t = np.linspace(0, 0.5, 51)
    Q = np.outer(np.sin(np.pi * t / 0.5) ** 2, [2.0, 0, 0, 0, 0, 0])      # 2 rad in 0.25 s: far too fast
    vlim, alim = np.full(6, math.pi), np.full(6, 5.0)
    t2, V, k = rp.fit_limits(t, Q, vlim, alim)
    assert k > 1 and np.allclose(t2, t * k)
    vel, acc = rp.spline_peaks(t2, Q, V)
    assert np.all(vel <= vlim + 1e-9) and np.all(acc <= alim + 1e-9)
    slow = np.linspace(0, 20, 51)                                       # within the limits: untouched
    t3, _, k3 = rp.fit_limits(slow, Q * 0.1, vlim, alim)
    assert k3 == 1.0 and np.array_equal(t3, slow)


def test_parse_refuses_malformed_points_and_speeds():
    with pytest.raises(rp.ReplayError, match="at least 2"):
        rp.parse({"points": [[0, 1, 2, 3, 4, 5, 6]]}, 6)
    with pytest.raises(rp.ReplayError, match="6 joint values"):
        rp.parse({"points": [[0, 1], [1, 2]]}, 6)
    with pytest.raises(rp.ReplayError, match="increase"):
        rp.parse({"points": [[0] + [0] * 6, [0] + [0] * 6]}, 6)
    with pytest.raises(rp.ReplayError, match="speed"):
        rp.speed_of({"speed": 5})


def test_resume_index_finds_where_the_arm_stopped():
    t = np.linspace(0, 10, 101)
    Q = np.outer(t / 10, np.ones(6))
    assert rp.resume_index(t, Q, 0, 4.0, Q[42]) == 42
    assert rp.resume_index(t, Q, 50, 1.0, Q[61]) == 61


# ------------------------------------------------------------------ teaching and running on the fake robot
def _drive_joints(s, seconds=1.5, amp=(0.4, -0.3, 0.3, 0.2, -0.2, 0.5)):
    """Freedrive stand-in: the joints swing smoothly from where they are, then stand still."""
    q0 = np.array(s.core.bridge._q, float)
    time.sleep(0.2)
    t0 = time.monotonic()
    while True:
        a = (time.monotonic() - t0) / seconds
        if a >= 1:
            break
        sm = a * a * (3 - 2 * a)
        s.core.bridge.set_fake_joints(q0 + sm * np.asarray(amp))
        time.sleep(0.005)
    s.core.bridge.set_fake_joints(q0 + np.asarray(amp))
    time.sleep(0.3)
    return q0, q0 + np.asarray(amp)


def _teach(s, **kw):
    cmd(s, "teach_start", mode="path")
    q0, q1 = _drive_joints(s, **kw)
    r = cmd(s, "teach_finish")
    return q0, q1, r["teach"]["result"]


def test_a_taught_path_is_inserted_as_a_replay_and_runs_back_as_taught(s):
    q0, q1, res = _teach(s)
    rv = res["replay"]
    assert rv["points"] >= 10 and rv["samples"] >= 60 and 1.2 < rv["duration_s"] < 2.0
    with pytest.raises(CommandError, match="MoveL points"):
        cmd(s, "teach_insert", press_n=5)
    r = cmd(s, "teach_insert", speed_pct=100)
    st = s.program.steps[r["selected"][0]]
    assert st.type == "replay" and s.teach is None and "speed" not in st.params
    pts = np.asarray(st.params["points"])
    assert pts[0, 0] == 0.0 and np.allclose(pts[0, 1:], q0, atol=2e-3) and np.allclose(pts[-1, 1:], q1, atol=2e-3)
    assert "points" not in s.program_view()["program"]["rows"][0]["params"]      # not sent with every row
    # stored in the YAML, and loads back
    again = Program.from_dict(yaml.safe_load(s._program_yaml()), s.core.profile)
    assert again.steps[0].params["points"] == st.params["points"]
    assert not again.validate(s.core.profile)
    # run it from elsewhere: a planned MoveJ to the start, then one trajectory with the taught timing
    s.core.bridge.set_fake_joints(q0 + 0.2)
    s.core.bridge.calls.clear()
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None, 30)
    assert s.core.program_result[0], s.core.program_result
    kinds = [c[0] for c in s.core.bridge.calls if c[0] in ("movej", "execute_joint_path", "move_pose")]
    assert kinds == ["movej", "execute_joint_path"]
    assert np.allclose(s.core.bridge.calls[[c[0] for c in s.core.bridge.calls].index("movej")][1], pts[0, 1:], atol=1e-4)
    names, times, Q = s.core.bridge.last_joint_path
    assert names == list(s.core.profile.joint_names) and len(Q) == len(pts)
    k = times[-1] / pts[-1, 0]                                                    # as taught (x k >= 1 if the
    assert 1.0 <= k < 1.5 and np.allclose(times, pts[:, 0] * k) and np.allclose(Q, pts[:, 1:])  # fake's jitter needs it)
    assert np.allclose(s.core.bridge._q, pts[-1, 1:], atol=1e-6)


def test_speed_override_and_step_speed_scale_the_timing(s):
    _teach(s)
    cmd(s, "teach_insert", speed_pct=50)
    st = s.program.steps[0]
    assert st.params["speed"] == 0.5
    pts = np.asarray(st.params["points"])
    s.core.set_speed_override(0.5)
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None, 30)
    assert s.core.program_result[0], s.core.program_result
    _, times, _ = s.core.bridge.last_joint_path
    assert times[-1] == pytest.approx(pts[-1, 0] * 4, rel=1e-6)


def test_a_too_fast_recording_is_slowed_not_refused(s):
    d = {"version": 2, "name": "t", "steps": [{"type": "replay", "points": [
        [round(k * 0.01, 4)] + [round(2.0 * math.sin(math.pi * k / 40) ** 2, 6)] + [0.0] * 5 for k in range(41)]}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    s.core.bridge.set_fake_joints([0.0] * 6)
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None, 30)
    assert s.core.program_result[0], s.core.program_result
    _, times, _ = s.core.bridge.last_joint_path
    assert times[-1] > 0.4 * 3                                           # x several: within pi rad/s, 5 rad/s^2
    assert any("replay slowed" in line for _, line in s.logs)


def test_stop_ends_a_replay_and_pause_resumes_from_where_it_was(s):
    s.core.bridge.time_scale = 1.0
    q0, q1, _ = _teach(s, seconds=3.0)
    cmd(s, "teach_insert")
    cmd(s, "program_run")
    assert tick_until(s, lambda: any(c[0] == "execute_joint_path" for c in s.core.bridge.calls), 10)
    time.sleep(1.0)
    cmd(s, "program_pause")
    assert tick_until(s, lambda: s.core.runner.state.name == "PAUSED", 5)
    time.sleep(0.2)
    q_paused = np.array(s.core.bridge._q)
    assert 0.05 < np.max(np.abs(q_paused - q0)) and np.max(np.abs(q_paused - q1)) > 0.05     # half way
    n_exec = sum(c[0] == "execute_joint_path" for c in s.core.bridge.calls)
    cmd(s, "program_resume")
    assert tick_until(s, lambda: sum(c[0] == "execute_joint_path" for c in s.core.bridge.calls) > n_exec, 10)
    _, times, Q = s.core.bridge.last_joint_path
    assert np.max(np.abs(np.asarray(Q[0]) - q_paused)) < 0.05 and times[-1] < 2.9    # the rest, not from the start
    time.sleep(0.3)
    s.core.stop_all("test")
    assert tick_until(s, lambda: s.core.program_result is not None, 10)
    assert not s.core.program_result[0]
    assert np.max(np.abs(np.asarray(s.core.bridge._q) - q1)) > 1e-3              # it did not finish


def test_keepout_zone_and_joint_limits_refuse_the_replay_before_anything_moves(s):
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)   # TCP at (0.3, 0, 0.443) at 0
    pts = [[0.0] + [0.0] * 6, [1.0, math.pi / 2] + [0.0] * 5]                      # base swings 90°: TCP to +y
    d = {"version": 2, "name": "t", "steps": [{"type": "replay", "points": pts}]}
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    s.core.keepout_store.put(make_box("wall", "base_link", [100, 100, 100], [0, 300, 443], [0, 0, 0]))
    s.core.bridge.set_fake_joints([0.0] * 6)
    with pytest.raises(CommandError, match="keep-out zone wall"):
        cmd(s, "program_run")
    s.core.keepout_store.delete("wall")
    pts[1][1] = 7.0                                                                # outside the URDF limit of ±6
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    with pytest.raises(CommandError, match="outside its limits"):
        cmd(s, "program_run")
    assert not any(c[0] in ("movej", "execute_joint_path") for c in s.core.bridge.calls)


def test_the_preview_draws_the_replayed_path(s):
    s.core.bridge.fake_robot_description = _chain_urdf(s.core.profile.joint_names)
    s.core.bridge.set_fake_joints([0.0] * 6)
    s.tick()
    pts = [[k * 0.1, k * 0.05] + [0.0] * 5 for k in range(11)]
    cmd(s, "program_set_yaml", text=yaml.safe_dump({"version": 2, "name": "t", "steps": [{"type": "replay", "points": pts}]}))
    r = cmd(s, "program_preview_path")
    assert len(r["points"]) == 12 and r["joints"][-1][0] == pytest.approx(0.5)
    end = np.asarray(r["points"][-1]) / 1000
    assert np.allclose(end, [0.3 * math.cos(0.5), 0.3 * math.sin(0.5), 0.443], atol=1e-3)
    assert "Replay" in r["what"][1]


def test_old_taught_paths_load_and_run_as_before(s):
    """The previous format (a folder of MoveL steps) is unchanged: it loads, validates and is still offered."""
    cmd(s, "teach_start", mode="path")
    _drive_joints(s)
    cmd(s, "teach_finish")
    r = cmd(s, "teach_insert", kind="movel")
    folder = s.program.steps[r["selected"][0]]
    assert folder.type == "folder" and all(c.type == "move_pose" for c in folder.children)
    assert Program.from_dict(yaml.safe_load(s._program_yaml()), s.core.profile).steps[0].type == "folder"
