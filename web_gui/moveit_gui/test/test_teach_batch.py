"""Teach by hand (waypoints and simplified paths) and batch runs (vials, parameter sweep, repeats), fake robot."""
import math
import os
import threading
import time

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation, Slerp

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.batch import BatchDef, BatchError, load, save, variant  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402
from moveit_gui.program import Program  # noqa: E402
from moveit_gui.teach import StillDetector, TaughtPath, TeachError, simplify_indices  # noqa: E402
from moveit_gui.web.service import CommandError  # noqa: E402

from test_program_force import cmd, s, tick_until, wait  # noqa: E402,F401  (s: the fake-robot WebService fixture)


# ======================================================================= path simplification maths
def _independent_deviation(t, P, R, keep):
    """Reference check written without moveit_gui.teach: every sample against the MoveL segment it falls in
    (scipy slerp at the projection fraction; time fraction for a turn in place)."""
    worst_p = worst_r = 0.0
    for a, b in zip(keep[:-1], keep[1:]):
        seg = P[b] - P[a]
        L2 = float(seg @ seg)
        sl = Slerp([0.0, 1.0], Rotation.from_matrix([R[a], R[b]]))
        for k in range(a + 1, b):
            s_ = float(np.clip((P[k] - P[a]) @ seg / L2, 0, 1)) if L2 > 0.0005 ** 2 else (t[k] - t[a]) / (t[b] - t[a])
            dp = float(np.linalg.norm(P[k] - (P[a] + s_ * seg)))
            dr = float((sl([s_])[0].inv() * Rotation.from_matrix(R[k])).magnitude())
            worst_p, worst_r = max(worst_p, dp), max(worst_r, dr)
    return worst_p * 1000.0, math.degrees(worst_r)


def _wiggly(n=800, seed=0):
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 40, n)
    P = np.c_[0.4 + 0.1 * np.sin(t / 5), 0.05 * np.cos(t / 3), 0.3 + 0.02 * np.sin(t)] + rng.normal(0, 3e-5, (n, 3))
    rv = np.c_[0.3 * np.sin(t / 7), 0.2 * np.cos(t / 4), t / 20]
    R = Rotation.from_rotvec(rv).as_matrix()
    return t, P, R


def _samples(t, P, R):
    return [(float(tk), tf.from_rotation(Rk, Pk)) for tk, Pk, Rk in zip(t, P, R)]


@pytest.mark.parametrize("pos_tol,rot_tol", [(0.5, 1.0), (1.0, 2.0), (3.0, 5.0), (10.0, 20.0)])
def test_every_sample_is_within_the_tolerances_of_the_simplified_path(pos_tol, rot_tol):
    t, P, R = _wiggly()
    path = TaughtPath(_samples(t, P, R))
    r = path.simplify(pos_tol, rot_tol)
    assert r.keep[0] == 0 and r.keep[-1] == len(P) - 1 and r.keep == sorted(set(r.keep))
    assert 2 < len(r.keep) < len(P) / 4                       # simplified, but not to a straight line
    dp, dr = _independent_deviation(t, P, R, r.keep)
    assert dp <= pos_tol + 1e-9 and dr <= rot_tol + 1e-6      # the guarantee, checked independently
    assert r.max_mm == pytest.approx(dp, abs=1e-6) and r.max_deg == pytest.approx(dr, abs=1e-4)   # what the page shows


def test_a_looser_tolerance_never_needs_more_points():
    t, P, R = _wiggly(seed=3)
    path = TaughtPath(_samples(t, P, R))
    counts = [len(path.simplify(p, r).keep) for p, r in [(0.5, 1), (1, 2), (2, 4), (5, 10), (20, 40)]]
    assert counts == sorted(counts, reverse=True)


def test_a_straight_noisy_line_is_two_points_and_a_corner_is_kept():
    rng = np.random.default_rng(1)
    t = np.linspace(0, 2, 200)
    P = np.c_[0.3 + 0.1 * t, np.zeros_like(t), 0.2 * np.ones_like(t)] + rng.normal(0, 1e-4, (200, 3))
    R = [np.eye(3)] * 200
    assert TaughtPath(_samples(t, P, R)).simplify(1.0, 2.0).keep == [0, 199]
    # an L: along x, then along y; the corner sample (index 99) must stay
    P2 = np.array([[0.3 + 0.001 * k, 0, 0.2] for k in range(100)] + [[0.399, 0.001 * k, 0.2] for k in range(1, 100)])
    r = TaughtPath(_samples(np.arange(199) * 0.05, P2, [np.eye(3)] * 199)).simplify(1.0, 2.0)
    assert r.keep == [0, 99, 198]


def test_a_turn_in_place_is_simplified_by_orientation():
    t = np.linspace(0, 3, 61)
    R = Rotation.from_euler("z", np.linspace(0, 90, 61), degrees=True).as_matrix()
    R2 = R.copy()
    R2[30:] = (Rotation.from_euler("z", np.linspace(45, 90, 31), degrees=True) *
               Rotation.from_euler("x", np.linspace(0, 40, 31), degrees=True)).as_matrix()       # a bend half way
    P = np.tile([0.4, 0.0, 0.3], (61, 1))
    assert TaughtPath(_samples(t, P, R)).simplify(1.0, 2.0).keep == [0, 60]   # slerp follows a steady turn
    r = TaughtPath(_samples(t, P, R2)).simplify(1.0, 2.0)
    assert len(r.keep) >= 3 and r.max_deg <= 2.0
    assert _independent_deviation(t, P, R2, r.keep)[1] <= 2.0 + 1e-6


def test_a_long_path_is_quick_and_has_no_recursion_limit():
    t, P, R = _wiggly(n=12000, seed=5)                         # 10 minutes at 20 Hz
    path = TaughtPath(_samples(t, P, R))
    t0 = time.monotonic()
    r = path.simplify(0.5, 1.0)
    assert time.monotonic() - t0 < 10.0
    assert _independent_deviation(t, P, R, r.keep)[0] <= 0.5 + 1e-9
    # degenerate input: every sample the same point
    same = TaughtPath([(k * 0.05, tf.make([0.4, 0, 0.3])) for k in range(3000)])
    assert same.simplify(1, 2).keep == [0, 2999]
    assert simplify_indices(np.arange(2), np.zeros((2, 3)), np.tile([0, 0, 0, 1.0], (2, 1)), 0.001, 0.01) == [0, 1]


def test_tolerances_and_the_average_speed():
    t, P, R = _wiggly(n=50)
    path = TaughtPath(_samples(t, P, R))
    for bad in [(0, 2), (1, 0), (100, 2), (1, 90), ("x", 2)]:
        with pytest.raises(TeachError):
            path.simplify(*bad)
    with pytest.raises(TeachError, match="fewer than 2"):
        TaughtPath([(0.0, np.eye(4))])
    # 1 s standing, 2 s at 50 mm/s along x, 1 s standing: the average while moving is 50 mm/s
    t = np.arange(0, 4.0001, 0.05)
    x = np.clip(t - 1.0, 0, 2.0) * 0.05
    st = TaughtPath(_samples(t, np.c_[0.3 + x, 0 * x, 0.2 + 0 * x], [np.eye(3)] * len(t))).stats()
    assert st["length_mm"] == pytest.approx(100.0, abs=0.5) and st["duration_s"] == pytest.approx(4.0)
    assert st["avg_mm_s"] == pytest.approx(50.0, abs=2.0) and st["moving_s"] == pytest.approx(2.0, abs=0.1)


def test_still_detector_marks_once_after_moving():
    d = StillDetector(still_s=1.0)
    d.reset(tf.make([0.4, 0, 0.3]))
    hits = []
    for k in range(80):                                        # 20 Hz: stand 1 s, move 30 mm over 1 s, stand 2 s
        tk = k * 0.05
        x = 0.4 + 0.03 * min(max(tk - 1.0, 0.0), 1.0)
        hits.append(d.feed(tk, tf.make([x, 0, 0.3])))
    assert sum(hits) == 1 and 2.9 < hits.index(True) * 0.05 < 3.2    # one mark, ~1 s after the arm stopped


# ======================================================================= teaching on the fake robot
def tool0(s):
    return s.core.bridge.T_base_tool0.copy()


def move_by(s, dx=0.0, dy=0.0, dz=0.0, rz_deg=0.0):
    """Stands in for freedrive: move tool0 by hand."""
    T = tool0(s)
    T[:3, 3] += [dx, dy, dz]
    T[:3, :3] = tf.mm_deg_to_T([0, 0, 0], [0, 0, rz_deg])[:3, :3] @ T[:3, :3]
    s.core.bridge.set_fake_tool0(T)
    s.tick()


def prog(s):
    return s.program


def test_marks_add_named_waypoints_and_move_steps_in_the_chosen_frame(s):
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.make([0.5, 0.0, 0.05]), {}))
    cmd(s, "program_insert", type="wait")
    r = cmd(s, "teach_start", mode="waypoints", move="linear", frame="vial", path=[0], where="before")
    assert r["teach"]["mode"] == "waypoints" and s.teach_view()["frame"] == "vial"
    r1 = cmd(s, "teach_mark")
    move_by(s, dx=0.02)
    r2 = cmd(s, "teach_mark")
    assert [st.type for st in prog(s).steps] == ["move_pose", "move_pose", "wait"]       # before the selection, in order
    assert r1["selected"] == [0] and r2["selected"] == [1]
    w1, w2 = prog(s).waypoints["taught_001"], prog(s).waypoints["taught_002"]
    assert w1.pose.frame == "vial" and w1.pose.tcp == s.core.store.active_tcp
    assert prog(s).steps[1].params == {"target": "taught_002", "cartesian": True, "velocity": 0.1}
    assert np.allclose(w2.pose.T()[:3, 3] - w1.pose.T()[:3, 3], [0.02, 0, 0], atol=1e-6)    # vial axes = base axes here
    # MoveJ (joints) and undo
    cmd(s, "teach_finish")
    cmd(s, "teach_start", mode="waypoints", move="joints")
    s.tick()
    cmd(s, "teach_mark")
    assert prog(s).steps[-1].type == "movej" and prog(s).waypoints["taught_003"].joints is not None
    cmd(s, "program_undo")
    assert "taught_003" not in prog(s).waypoints and len(prog(s).steps) == 3
    cmd(s, "teach_cancel")
    assert s.teach is None and s.state()["teach"] is None


def test_teaching_needs_control_and_blocks_runs(s):
    with pytest.raises(CommandError, match="take control"):
        s.command("someone-else", "teach_start", {})
    cmd(s, "teach_start", mode="waypoints")
    with pytest.raises(CommandError, match="already teaching"):
        cmd(s, "teach_start")
    with pytest.raises(CommandError, match="finish teaching"):
        cmd(s, "program_run")
    with pytest.raises(CommandError, match="finish teaching"):
        cmd(s, "batch_run", batch={"kind": "repeat", "repeats": 1})
    calls = len(s.core.bridge.calls)
    cmd(s, "teach_mark")
    assert len(s.core.bridge.calls) == calls                   # teaching never moves the robot
    # another browser takes over: teaching ends, the mark stays
    s.take_lease("b")
    s.tick()
    assert s.teach is None and "taught_001" in prog(s).waypoints


def test_unknown_frame_is_refused(s):
    with pytest.raises(CommandError):
        cmd(s, "teach_start", frame="no_such_fixture")
    assert s.teach is None


def test_auto_mark_when_the_arm_stood_still(s):
    cmd(s, "teach_start", mode="waypoints", move="pose", auto=True)
    time.sleep(1.3)
    assert prog(s).waypoints == {}                             # standing at the start does not mark
    rev = s.program_rev
    move_by(s, dz=0.03)
    assert wait(lambda: "taught_001" in prog(s).waypoints, 4.0)
    assert s.program_rev > rev and s._undo                     # an ordinary undoable edit
    assert prog(s).steps[0].params["cartesian"] is False       # MoveJ to pose
    time.sleep(1.5)
    assert "taught_002" not in prog(s).waypoints               # still standing: no second mark
    cmd(s, "teach_finish")
    assert s.teach is None


def _drive_l_path(s, seconds=1.6, done=None):
    """Hand motion: 40 mm along x, then 30 mm along y, at ~45 mm/s."""
    T0 = tool0(s)
    t0 = time.monotonic()
    while True:
        a = (time.monotonic() - t0) / seconds
        if a >= 1:
            break
        d = a * 0.07
        T = T0.copy()
        T[:3, 3] += [min(d, 0.04), max(d - 0.04, 0.0), 0.0]
        s.core.bridge.set_fake_tool0(T)
        time.sleep(0.01)
    T = T0.copy()
    T[:3, 3] += [0.04, 0.03, 0]
    s.core.bridge.set_fake_tool0(T)
    time.sleep(0.3)


def test_a_taught_path_becomes_a_folder_of_movel_steps_and_replays(s):
    cmd(s, "program_insert", type="wait")
    cmd(s, "teach_start", mode="path", pos_tol_mm=1.0, rot_tol_deg=2.0)
    time.sleep(0.2)
    _drive_l_path(s)
    r = cmd(s, "teach_finish")
    res = r["teach"]["result"]
    assert r["teach"]["phase"] == "review" and res["samples"] >= 25
    assert res["points"] == 3 and res["max_mm"] <= 1.0          # start, corner, end
    assert res["length_mm"] == pytest.approx(70.0, abs=3.0) and 25 < res["avg_mm_s"] < 70
    assert res["speed_mm_s"] == round(res["avg_mm_s"])
    assert len(res["kept"]) == 3 and len(res["raw_mm"]) >= 2
    r = cmd(s, "teach_simplify", pos_tol_mm=50, rot_tol_deg=45)
    assert r["teach"]["result"]["points"] == 2
    with pytest.raises(CommandError, match="tolerance"):
        cmd(s, "teach_simplify", pos_tol_mm=0)
    with pytest.raises(CommandError, match="speed"):
        cmd(s, "teach_insert", kind="movel", pos_tol_mm=1, rot_tol_deg=2, speed_mm_s=9999)
    r = cmd(s, "teach_insert", kind="movel", pos_tol_mm=1, rot_tol_deg=2, speed_mm_s=40, path=[0], where="after")
    assert r["selected"] == [1] and s.teach is None
    folder = prog(s).steps[1]
    assert folder.type == "folder" and len(folder.children) == 3
    first, *rest = folder.children
    assert first.params["cartesian"] is False and "speed_mm_s" not in first.params
    assert all(c.params["cartesian"] is True and c.params["speed_mm_s"] == 40 for c in rest)
    assert [c.params["target"] for c in folder.children] == ["taught_001", "taught_002", "taught_003"]
    end = prog(s).waypoints["taught_003"].pose
    assert end.frame == s.core.profile.base_link
    assert np.allclose(end.T()[:3, 3], s.core.geo.current_T_base_tcp(end.tcp)[:3, 3], atol=1e-3)
    assert s._undo                                              # one undoable edit
    # replay: an ordinary program run, back to the start of the path and along it
    move_by(s, dz=0.05)
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None, 30)
    assert s.core.program_result[0], s.core.program_result
    assert np.allclose(s.core.geo.current_T_base_tcp(end.tcp)[:3, 3], end.T()[:3, 3], atol=1e-3)


def test_path_teaching_cancel_and_too_short(s):
    cmd(s, "teach_start", mode="path")
    with pytest.raises(CommandError, match="finish the recording"):
        cmd(s, "teach_insert")
    s.teach["samples"][:] = s.teach["samples"][:1]
    s.teach["sampler"].stop()
    with pytest.raises(CommandError, match="fewer than 2"):
        cmd(s, "teach_finish")
    assert s.teach is None
    cmd(s, "teach_start", mode="path")
    cmd(s, "teach_cancel")
    assert s.teach is None and prog(s).steps == []


# ======================================================================= batches
def load_program(s, steps, waypoints=None, save_as=None):
    d = {"version": 2, "name": "t", "robot": "ur7e_lab", "steps": steps}
    if waypoints:
        d["waypoints"] = waypoints
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d))
    if save_as:
        cmd(s, "program_save", name=save_as)


def capture_runs(s):
    """Each run's program copy and where the vial fixture was when it started."""
    seen = []
    orig = s.core.runner.start

    def start(program, *a, **k):
        try:
            v = s.core.geo.T_base_frame("vial")[:3, 3].round(4).tolist()
        except Exception:
            v = None
        seen.append({"program": program, "vial": v})
        return orig(program, *a, **k)
    s.core.runner.start = start
    return seen


def run_batch(s, timeout=30.0, **batch):
    cmd(s, "batch_run", batch=batch)
    assert s.core.batch.active
    assert tick_until(s, lambda: not s.core.batch.active, timeout)
    return s.core.batch.view()


def test_repeat_runs_n_times_and_counts_as_one_run(s):
    load_program(s, [{"type": "wait", "seconds": 0.3}])
    cmd(s, "batch_run", batch={"kind": "repeat", "repeats": 3})
    busy = []
    for _ in range(40):
        s.tick()
        busy.append(s.core.idle())
        if s.core.runner.state.name == "IDLE" and s.core.batch.active:
            # between two runs: still busy for everything else
            with pytest.raises(CommandError, match="busy"):
                cmd(s, "program_run")
        time.sleep(0.05)
    assert tick_until(s, lambda: not s.core.batch.active, 20)
    assert not any(busy[:5])
    v = s.core.batch.view()
    assert [r["n"] for r in v["rows"]] == [1, 2, 3] and all(r["ok"] for r in v["rows"])
    assert v["result"][0] and v["result"][1].startswith("3 of 3 runs ok")
    assert all(r["duration_s"] >= 0.3 for r in v["rows"]) and v["rows"][0]["change"] == "repeat 1"
    kinds = [e["kind"] for e in s.core.history.recent(10)]
    assert kinds.count("program") >= 3 and kinds[0] == "batch"
    assert s.state()["batch"]["done"] == 3


def test_sweep_changes_one_parameter_per_run_on_a_copy(s):
    load_program(s, [{"type": "folder", "name": "f", "steps": [
        {"type": "force", "wrench": [0, 0, 5, 0, 0, 0], "frame": "tool"}, {"type": "wait", "seconds": 0.1}]}])
    before = s._program_yaml()
    seen = capture_runs(s)
    v = run_batch(s, kind="sweep", step=[0, 1], step_type="wait", param="seconds", values=[0.1, 0.3, 0.2],
                  on_fail="continue")
    assert len(v["rows"]) == 3
    assert [x["program"].steps[0].children[1].params["seconds"] for x in seen] == [0.1, 0.3, 0.2]
    assert s._program_yaml() == before                          # the open program never changed
    assert v["rows"][1]["change"] == "step 1.2 seconds = 0.3"
    # a list element (Fz of the force step) and the static check of every value before anything runs
    b = BatchDef.from_dict({"kind": "sweep", "step": [0, 0], "step_type": "force", "param": "wrench.2", "values": "3, 4"})
    assert [variant(s.program, b, r).steps[0].children[0].params["wrench"][2] for r in b.runs()] == [3.0, 4.0]
    with pytest.raises(CommandError, match=r"run 2 \(step 1.1 wrench.2 = 99\).*limit"):
        cmd(s, "batch_run", batch={"kind": "sweep", "step": [0, 0], "step_type": "force", "param": "wrench.2",
                                    "values": [3, 99]})
    with pytest.raises(CommandError, match="pick the step again"):
        cmd(s, "batch_run", batch={"kind": "sweep", "step": [0, 0], "step_type": "wait", "param": "seconds",
                                    "values": [1]})
    assert not s.core.batch.active


def test_sweep_wait_seconds_all_ok(s):
    load_program(s, [{"type": "wait", "seconds": 0.1}])
    v = run_batch(s, kind="sweep", step=[0], step_type="wait", param="seconds", values=[0.2, 0.6])
    assert all(r["ok"] for r in v["rows"])
    assert v["rows"][1]["duration_s"] >= 0.6 > v["rows"][0]["duration_s"] >= 0.2


def _vials(s, original=True):
    cmd(s, "vial_profile_save", name="A", source="typed", xyz_mm=[400, -100, 20], radius_mm=12)
    cmd(s, "vial_profile_save", name="B", source="typed", xyz_mm=[450, 120, 25], radius_mm=12)
    if original:
        s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.make([0.5, 0.0, 0.05]), {"method": "taught"}))


def test_vial_batch_uses_each_profile_then_puts_the_fixture_back(s):
    _vials(s)
    load_program(s, [{"type": "wait", "seconds": 0.1}])
    seen = capture_runs(s)
    v = run_batch(s, kind="vials", vials=["A", "B", "baseline"], fixture="vial")
    assert [x["vial"] for x in seen] == [[0.4, -0.1, 0.02], [0.45, 0.12, 0.025], [0.525, 0.016, 0.04]]
    assert all(r["ok"] for r in v["rows"]) and [r["change"] for r in v["rows"]] == ["vial A", "vial B", "vial baseline"]
    fx = s.core.store.fixtures["vial"]
    assert np.allclose(fx.T_parent_fixture[:3, 3], [0.5, 0.0, 0.05]) and fx.meta.get("method") == "taught"
    assert "vial_profile" not in fx.meta and "put back" in v["restored"]
    with pytest.raises(CommandError, match="unknown vial"):
        cmd(s, "batch_run", batch={"kind": "vials", "vials": ["A", "nope"]})


def test_vial_batch_without_a_fixture_before_removes_it_again(s):
    _vials(s, original=False)
    load_program(s, [{"type": "wait", "seconds": 0.05}])
    v = run_batch(s, kind="vials", vials=["A"], fixture="vial_2")
    assert v["rows"][0]["ok"] and "vial_2" not in s.core.store.fixtures and "removed again" in v["restored"]


def test_failure_stops_or_continues_and_the_vial_is_restored(s):
    _vials(s)
    load_program(s, [{"type": "movej", "target": "home"}])
    calls = {"n": 0}
    real = s.core.bridge.move_joints

    def flaky(*a, **k):
        calls["n"] += 1
        return False if calls["n"] == 2 else real(*a, **k)
    s.core.bridge.move_joints = flaky
    v = run_batch(s, kind="vials", vials=["A", "B", "baseline"], on_fail="stop")
    assert [r["ok"] for r in v["rows"]] == [True, False] and not v["result"][0] and "run 2 failed" in v["result"][1]
    assert np.allclose(s.core.store.fixtures["vial"].T_parent_fixture[:3, 3], [0.5, 0.0, 0.05])
    calls["n"] = 0
    v = run_batch(s, kind="vials", vials=["A", "B", "baseline"], on_fail="continue")
    assert [r["ok"] for r in v["rows"]] == [True, False, True] and "1 failed" in v["result"][1]
    assert np.allclose(s.core.store.fixtures["vial"].T_parent_fixture[:3, 3], [0.5, 0.0, 0.05])


def test_stop_ends_the_whole_batch_and_restores(s):
    _vials(s)
    load_program(s, [{"type": "wait", "seconds": 5}])
    before = s._program_yaml()
    cmd(s, "batch_run", batch={"kind": "vials", "vials": ["A", "B"], "on_fail": "continue"})
    assert tick_until(s, lambda: s.core.runner.state.name == "RUNNING", 5)
    assert np.allclose(s.core.geo.T_base_frame("vial")[:3, 3], [0.4, -0.1, 0.02])
    s.stop("test")                                              # the big STOP (anyone, no lease)
    assert tick_until(s, lambda: not s.core.batch.active, 5)
    v = s.core.batch.view()
    assert len(v["rows"]) == 1 and v["rows"][0]["ok"] is False and "stopped" in v["result"][1].lower()
    assert np.allclose(s.core.store.fixtures["vial"].T_parent_fixture[:3, 3], [0.5, 0.0, 0.05])
    assert s._program_yaml() == before and s.core.idle()
    # PROGRAMS ■ STOP (program_stop) ends the batch too, not just its run
    load_program(s, [{"type": "wait", "seconds": 5}])
    cmd(s, "batch_run", batch={"kind": "repeat", "repeats": 3, "on_fail": "continue"})
    assert tick_until(s, lambda: s.core.runner.state.name == "RUNNING", 5)
    cmd(s, "program_stop")
    assert tick_until(s, lambda: not s.core.batch.active, 5)
    assert len(s.core.batch.view()["rows"]) == 1


def test_pause_asks_before_each_next_run(s):
    _vials(s)
    load_program(s, [{"type": "wait", "seconds": 0.05}])
    cmd(s, "batch_run", batch={"kind": "vials", "vials": ["A", "B", "baseline"], "pause": True})
    assert tick_until(s, lambda: s.core.batch.phase == "waiting", 10)
    v = s.state()["batch"]
    assert v["question"].startswith("Next vial ready?") and "Run 2 of 3: vial B" in v["question"] and v["done"] == 1
    # waiting counts as busy: nothing else moves, the vial cannot be changed, nobody takes over
    with pytest.raises(CommandError, match="busy"):
        cmd(s, "move_joints", q_deg=[0, -90, 90, -90, -90, 0])
    with pytest.raises(CommandError, match="busy"):
        cmd(s, "vial_profile_use", name="A")
    with pytest.raises(CommandError, match="running"):
        cmd(s, "teach_start")
    with pytest.raises(CommandError, match="moving robot"):
        s.take_lease("b")
    with pytest.raises(CommandError, match="take control"):
        s.command("b", "batch_answer", {"go": True})
    first = v["ask_id"]
    cmd(s, "batch_answer", go=True)
    assert tick_until(s, lambda: s.core.batch.phase == "waiting" and s.core.batch.ask_id > first, 10)
    cmd(s, "batch_answer", go=False)
    assert tick_until(s, lambda: not s.core.batch.active, 5)
    v = s.core.batch.view()
    assert len(v["rows"]) == 2 and "stopped by the operator" in v["result"][1]
    assert np.allclose(s.core.store.fixtures["vial"].T_parent_fixture[:3, 3], [0.5, 0.0, 0.05])
    with pytest.raises(CommandError, match="not waiting"):
        cmd(s, "batch_answer", go=True)


def test_lost_heartbeat_during_the_pause_stops_the_batch(s):
    load_program(s, [{"type": "wait", "seconds": 0.05}])
    cmd(s, "batch_run", batch={"kind": "repeat", "repeats": 3, "pause": True})
    assert tick_until(s, lambda: s.core.batch.phase == "waiting", 10)
    s.clock = lambda: time.monotonic() + 10.0                  # no heartbeat for 10 s
    s.tick()
    assert wait(lambda: not s.core.batch.active, 5)
    assert len(s.core.batch.view()["rows"]) == 1


def test_batch_file_next_to_the_program_round_trip(s, tmp_path):
    load_program(s, [{"type": "wait", "seconds": 0.1}])
    with pytest.raises(CommandError, match="save the program"):
        cmd(s, "batch_save", batch={"kind": "repeat", "repeats": 2})
    assert cmd(s, "batch_get")["batch"] is None
    cmd(s, "program_save", name="scrape")
    b = {"kind": "sweep", "step": [0], "step_type": "wait", "param": "seconds", "values": [0.5, 1, 2],
         "pause": True, "on_fail": "continue"}
    r = cmd(s, "batch_save", batch=b)
    assert r["file"] == "scrape.batch.yaml"
    path = s.core.programs_dir / "scrape.batch.yaml"
    assert path.exists() and yaml.safe_load(path.read_text())["program"] == "scrape.yaml"
    got = cmd(s, "batch_get")["batch"]
    assert got == {"version": 1, "program": "scrape.yaml", "kind": "sweep", "step": [0], "step_type": "wait",
                   "param": "seconds", "values": [0.5, 1.0, 2.0], "pause": True, "on_fail": "continue"}
    assert "scrape.batch.yaml" not in cmd(s, "program_list")["files"]          # not a program
    for d in ({"kind": "repeat", "repeats": 4}, {"kind": "vials", "vials": ["A", "B"], "fixture": "vial_2"}, got):
        bd = BatchDef.from_dict(d)
        save(tmp_path / "x.batch.yaml", bd)
        assert load(tmp_path / "x.batch.yaml").to_dict() == bd.to_dict()
    for bad in ({"kind": "nope"}, {"kind": "repeat", "repeats": 0}, {"kind": "repeat", "repeats": 1.5},
                {"kind": "vials", "vials": []}, {"kind": "sweep", "step": [0], "param": "seconds", "values": []},
                {"kind": "sweep", "step": [0], "param": "seconds", "values": "a, b"},
                {"kind": "repeat", "repeats": 1, "on_fail": "maybe"}, {"kind": "repeat", "repeats": 500}):
        with pytest.raises(BatchError):
            BatchDef.from_dict(bad)


def test_rows_offer_the_sweepable_parameters(s):
    load_program(s, [{"type": "wait", "seconds": 2}, {"type": "force", "wrench": [0, 0, 5, 0, 0, 0]},
                     {"type": "move_relative", "offset_mm": [0, 0, 5], "cartesian": False},
                     {"type": "gripper", "command": "open"}])
    rows = s.program_view()["program"]["rows"]
    assert rows[0]["sweep"] == [{"key": "seconds", "label": "Seconds", "value": 2}]
    assert {"key": "wrench.2", "label": "Fz N", "value": 5.0} in rows[1]["sweep"]
    assert "speed_mm_s" not in [x["key"] for x in rows[2]["sweep"]]            # not a linear move
    assert Program.from_dict(yaml.safe_load(s._program_yaml()), s.core.profile).steps[0].params["seconds"] == 2


def test_a_taught_path_can_be_replayed_pressing_with_a_force(s):
    cmd(s, "teach_start", mode="path", pos_tol_mm=1.0, rot_tol_deg=2.0)
    time.sleep(0.2)
    _drive_l_path(s)
    cmd(s, "teach_finish")
    start_mm = cmd(s, "teach_simplify", pos_tol_mm=1, rot_tol_deg=2)["teach"]["result"]["kept"][0]["xyz_mm"]
    with pytest.raises(CommandError, match="profile limit"):
        cmd(s, "teach_insert", kind="movel", press_n=500)
    with pytest.raises(CommandError, match="direction"):
        cmd(s, "teach_insert", kind="movel", press_n=5, press_dir="sideways")
    r = cmd(s, "teach_insert", kind="movel", pos_tol_mm=1, rot_tol_deg=2, speed_mm_s=20, press_n=5)
    folder = prog(s).steps[r["selected"][0]]
    assert "pressing 5 N" in folder.params["name"]
    kinds = [c.type for c in folder.children]
    assert kinds == ["controller", "move_pose", "move_until_contact", "force", "move_pose", "move_pose",
                     "force_off", "move_relative", "controller"]
    sw, lift, touch, force = folder.children[:4]
    assert sw.params["mode"] == "admittance" and folder.children[-1].params["mode"] == s.core.profile.default_mode
    assert touch.params["direction"] == [0.0, 0.0, 1.0] and touch.params["frame"] == "tool"
    assert force.params["wrench"] == [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
    assert folder.children[-2].params["offset_mm"] == [-0.0, -0.0, -10.0]
    # the lift point is 10 mm back along the start's tool Z; the path's own start is not a step (contact is)
    T_lift = prog(s).waypoints[lift.params["target"]].pose.T()
    assert lift.params["cartesian"] is False
    assert all(c.params["cartesian"] and c.params["speed_mm_s"] == 20 for c in folder.children[4:6])
    d = T_lift[:3, 3] - np.array(start_mm) / 1000
    assert np.linalg.norm(d) == pytest.approx(0.010, abs=2e-4)
    assert np.allclose(d / np.linalg.norm(d), -T_lift[:3, 2], atol=0.02)



def test_press_straight_down(s):
    cmd(s, "teach_start", mode="path", pos_tol_mm=1.0, rot_tol_deg=2.0)
    time.sleep(0.2)
    _drive_l_path(s)
    cmd(s, "teach_finish")
    start_mm = cmd(s, "teach_simplify", pos_tol_mm=1, rot_tol_deg=2)["teach"]["result"]["kept"][0]["xyz_mm"]
    r = cmd(s, "teach_insert", kind="movel", press_n=4, press_dir="down")
    folder = prog(s).steps[r["selected"][0]]
    touch, force = folder.children[2:4]
    assert touch.params["frame"] == "base" and touch.params["direction"] == [0.0, 0.0, -1.0]
    assert force.params["wrench"][:3] == [0.0, 0.0, -4.0]
    assert folder.children[-2].params["offset_mm"] == [-0.0, -0.0, 10.0]
    # replay on the fake robot with a floor at the path's height: Admittance, touch, press along, let go, back off
    s.core.bridge.fake_surface = {"z_m": start_mm[2] / 1000 - 0.002, "k": 3000.0}
    move_by(s, dz=0.05)
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None, 60)
    assert s.core.program_result[0], s.core.program_result
    assert any("contact" in line for line in s.core.bridge.drain_logs() + [l for _, l in s.logs])


def test_the_ft_hard_limit_ends_the_whole_batch(s):
    """A hard F/T stop in run 1 of a continue-on-failure batch: no run 2 starts (the force may still be high)."""
    load_program(s, [{"type": "wait", "seconds": 5}])
    cmd(s, "batch_run", batch={"kind": "repeat", "repeats": 3, "on_fail": "continue"})
    assert tick_until(s, lambda: s.core.runner.state.name == "RUNNING", 5)
    s.core._ft_hard_stop("test: 55 N over 40 N")
    assert tick_until(s, lambda: not s.core.batch.active, 5)
    assert len(s.core.batch.view()["rows"]) == 1
