"""Run history (history.py) and the activity log (activity.py): what is written, the limits, the hooks."""
import json

import pytest

from moveit_gui import history as hist_mod
from moveit_gui.activity import ActivityLog, summarize_args
from moveit_gui.history import RunHistory
from moveit_gui.web.service import CommandError
from test_web import env, wait  # noqa: F401  (fixture reuse)


def lines(s):
    return [json.loads(x) for x in s.activity.path.read_text().splitlines()]


def run_program(s, tok, wait_s=0.05):
    s.command(tok, "program_new", {"name": "hist"})
    s.command(tok, "program_insert", {"type": "wait", "params": {"seconds": wait_s}})
    s.command(tok, "program_run", {})
    assert wait(lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None)


# --------------------------------------------------------------- run history
def test_history_trim_and_limit(tmp_path):
    h = RunHistory("x", tmp_path / "h.jsonl")
    for i in range(hist_mod.KEEP_RUNS + hist_mod.TRIM_SLACK + 5):
        h.record("program", f"p{i % 3}", 1000.0 + i, True, "finished", log=["a"] * 500)
    n = len((tmp_path / "h.jsonl").read_text().splitlines())
    assert hist_mod.KEEP_RUNS <= n <= hist_mod.KEEP_RUNS + hist_mod.TRIM_SLACK
    assert len(h.recent(10000)) == hist_mod.KEEP_RUNS
    assert all(e["name"] == "p1" for e in h.recent(50, "p1"))
    full = h.get(h.recent(1)[0]["id"])
    assert len(full["log"]) == hist_mod.LOG_LINES and "log" not in h.recent(1)[0]
    assert len(RunHistory("x", tmp_path / "h.jsonl").entries) == hist_mod.KEEP_RUNS      # reload from disk


def test_program_run_is_recorded_with_who_steps_and_log(env):
    s, _, _, _ = env
    tok = s.take_lease("", "alice @ pc", bid="a" * 20, key="alice@lab", role="operator")["token"]
    run_program(s, tok)
    e = s.command(tok, "history", {})["runs"][0]
    assert e["name"] == "hist" and e["operator"] == "alice @ pc" and e["outcome"] == "finished"
    assert e["steps_done"] == e["steps_total"] == 1 and e["peak_force_n"] >= 0
    full = s.command(tok, "history_get", {"id": e["id"]})["run"]
    assert any("program" in x for x in full["log"])
    with pytest.raises(CommandError):
        s.command(tok, "history_get", {"id": "nope"})


def test_stopped_run_names_who_stopped_it(env):
    s, _, _, _ = env
    tok = s.take_lease("", "alice @ pc", bid="a" * 20)["token"]
    s.command(tok, "program_new", {"name": "long"})
    s.command(tok, "program_insert", {"type": "wait", "params": {"seconds": 30}})
    s.command(tok, "program_run", {})
    assert wait(lambda: s.core.runner.state.name == "RUNNING")
    s.stop("bob @ tail", bid="b" * 20, key="bob@lab")
    assert wait(lambda: s.core.history.entries and s.core.history.entries[-1]["name"] == "long")
    e = s.core.history.entries[-1]
    assert e["outcome"] == "stopped" and e["stopped_by"].startswith("Browser") and not e["ok"]


# ------------------------------------------------------------- activity log
def test_activity_line_fields_and_no_secrets(env):
    s, _, _, _ = env
    r = s.take_lease("", "alice @ pc", bid="a" * 20, key="alice@lab", role="operator")
    ev = lines(s)[-1]
    assert ev["who"] == "alice" and ev["key"] == "alice@lab" and ev["role"] == "operator"
    assert ev["kind"] == "control" and ev["what"] == "took control" and ev["time"] and ev["t"] > 0
    s.command(r["token"], "tcp_save", {"name": "t1", "parent": "tool0", "xyz_mm": [0, 0, 1], "rpy_deg": [0, 0, 0]})
    s.stop("alice", bid="a" * 20)
    s.release(r["token"])
    raw = s.activity.path.read_text()
    for secret in (r["token"], "a" * 20):
        assert secret not in raw
    assert summarize_args({"token": "x", "bid": "y", "name": "n", "data_b64": "zz", "q_deg": [1, 2]}) == "name=n"


def test_hooks_take_over_give_stop_admin_program(env):
    s, _, _, _ = env
    s.take_lease("", "alice @ pc", bid="a" * 20, key="k1")
    s.take_lease("", "bob @ pc", bid="b" * 20, key="k2")
    s.ask_control("c" * 20, "carol")
    s.cmd_control_answer(give=False)
    kinds = [(e["kind"], e["what"], e["who"]) for e in lines(s)]
    assert ("control", "took control", "alice") in kinds
    assert ("control", "took over control", "bob") in kinds
    assert ("control", "asked for control", "carol") in kinds
    assert ("control", "kept control", "bob") in kinds
    b = s.take_lease("", "bob @ pc", presented=s.lease.token, bid="b" * 20)["token"]
    s.command(b, "roles_save", {"default": "operator", "people": {"x@y": "viewer"}})
    run_program(s, b)
    s.command(b, "record_start", {})
    s.command(b, "record_stop", {})
    s.stop("bob", bid="b" * 20)
    got = {(e["kind"], e["what"]) for e in lines(s)}
    for want in (("roles", "saved roles"), ("program", "program run"), ("record", "record start"),
                 ("record", "record stop"), ("stop", "pressed STOP")):
        assert want in got, want
    s.release(b)
    assert lines(s)[-1]["what"] == "released control"


def test_give_control_and_admin_command_are_logged(env):
    s, _, _, _ = env
    a = s.take_lease("", "alice @ pc", bid="a" * 20)["token"]
    s.ask_control("b" * 20, "bob")
    s.cmd_control_answer(give=True)
    ev = [e for e in lines(s) if e["what"] == "gave control"][0]
    assert ev["who"] == "alice" and "bob" in ev["details"]
    t = s.take_lease("", "bob @ pc", bid="b" * 20)["token"]
    s.command(t, "tcp_save", {"name": "t9", "parent": "tool0", "xyz_mm": [0, 0, 5], "rpy_deg": [0, 0, 0]})
    ev = lines(s)[-1]
    assert ev["kind"] == "setup" and ev["what"] == "tcp save" and "name=t9" in ev["details"] and ev["who"] == "bob"


def test_activity_rotation_filter_and_read(tmp_path):
    a = ActivityLog(tmp_path / "a" / "activity.jsonl", rotate_bytes=600)
    for i in range(30):
        a.add("program" if i % 2 else "stop", f"e{i}", {"name": "ann" if i % 3 else "bo", "key": "k"}, "d")
    assert (tmp_path / "a" / "activity.jsonl.1").exists()
    assert (tmp_path / "a" / "activity.jsonl").stat().st_size < 1200
    ev = a.recent(1000)
    assert ev[0]["what"] == "e29"                                  # newest first
    assert {e["kind"] for e in a.recent(1000, kind="stop")} == {"stop"}
    assert {e["who"] for e in a.recent(1000, who="bo")} == {"bo"}
    assert a.people() == ["ann", "bo"]


def test_activity_command_is_read_only_and_state_has_seq(env):
    s, _, _, _ = env
    s.take_lease("", "alice @ pc", bid="a" * 20)
    seq = s.state()["activity_seq"]
    out = s.command("", "activity", {"who": "alice", "kind": "control"})
    assert out["events"] and out["people"] == ["alice"] and "control" in out["kinds"]
    s.stop("x")
    assert s.state()["activity_seq"] > seq
