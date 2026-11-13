"""Web front end: one user per browser (even from one IP), and asking whoever has control for it."""
import json
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.web.service import CommandError  # noqa: E402
from moveit_gui.web.users import GONE_S, Users, public_id  # noqa: E402
from test_web import env, wait  # noqa: E402,F401  (fixture reuse)

A, B = "a" * 22, "b" * 22                     # two browsers' own ids (localStorage)


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


def test_two_browsers_on_one_ip_are_two_users_and_tabs_are_one():
    c = Clock()
    u = Users(c)
    ia = u.open(A, "Cenk", "127.0.0.1")
    u.open(A, "", "127.0.0.1")                # a second tab of the same browser
    ib = u.open(B, "", "127.0.0.1")
    assert ia == public_id(A) and ib != ia and A not in json.dumps(u.view())
    v = u.view(holder=ib)
    assert [x["name"] for x in v] == [f"Browser {ib[:4]}", "Cenk"]          # control first
    assert v[0]["control"] and v[1]["tabs"] == 2 and v[1]["where"] == "127.0.0.1"
    u.rename(B, "Cenk")
    assert [x["name"] for x in u.view()] == ["Cenk", "Cenk (2)"]           # same name: told apart
    assert u.open("short", "x", "y") is None and u.open("../" * 8, "x", "y") is None


def test_a_closed_browser_stays_briefly_so_a_reload_does_not_flicker():
    c = Clock()
    u = Users(c)
    u.open(A, "Cenk", "ip")
    u.close(A)
    assert [x["name"] for x in u.view()] == []                              # not shown while gone...
    c.t += GONE_S - 1
    u.open(A, "", "ip")                                                     # ...and a reload keeps its name
    assert [x["name"] for x in u.view()] == ["Cenk"]
    u.close(A)
    c.t += GONE_S + 1
    u.view()
    assert u.label(A) == f"Browser {public_id(A)[:4]}"                    # forgotten after a while


def test_ask_give_hands_control_to_the_one_who_asked_only(env):
    s, _, clock, _ = env
    s.users.open(A, "Alice", "ip"), s.users.open(B, "Bob", "ip")
    ta = s.take_lease("", "Alice @ ip", bid=A)["token"]
    assert s.state()["lease"]["uid"] == public_id(A)
    assert [u["control"] for u in s.state()["users"]] == [True, False]
    with pytest.raises(CommandError, match="already have control"):
        s.ask_control(A)
    s.ask_control(B)
    assert s.state()["lease"]["ask"]["name"] == "Bob"
    with pytest.raises(CommandError, match="take control"):            # only the holder answers
        s.command("", "control_answer", {"give": True})
    s.command(ta, "control_answer", {"give": True})
    st = s.state()["lease"]
    assert not s.heartbeat(ta) and st["client"] is None and st["offer"]["id"] == public_id(B)
    with pytest.raises(CommandError, match="being handed to Bob"):     # nobody else may grab it meanwhile
        s.take_lease("", "Carol", bid="c" * 22)
    tb = s.take_lease("", "Bob @ ip", bid=B)["token"]
    assert s.heartbeat(tb) and s.state()["lease"]["offer"] is None
    # an offer nobody takes runs out
    s.ask_control(A)
    s.command(tb, "control_answer", {"give": True})
    clock.t += s.OFFER_S + 1
    s.state()
    s.take_lease("", "Carol", bid="c" * 22)


def test_keep_tells_the_one_who_asked_and_stops_repeated_asking(env):
    s, _, clock, _ = env
    s.users.open(B, "Bob", "ip")
    ta = s.take_lease("", "Alice", bid=A)["token"]
    s.ask_control(B)
    assert "already asked" in s.ask_control(B)["msg"]
    s.command(ta, "control_answer", {"give": False})
    assert s.state()["lease"]["ask"]["kept"] and s.heartbeat(ta)
    with pytest.raises(CommandError, match="keeps control"):
        s.ask_control(B)
    with pytest.raises(CommandError, match="nobody is asking"):
        s.command(ta, "control_answer", {"give": True})
    clock.t += 21
    s.ask_control(B)
    clock.t += s.ASK_S + 1                                                 # no answer: the question expires
    assert s.state()["lease"]["ask"] is None


def test_give_is_refused_while_the_robot_moves(env):
    s, bridge, _, _ = env
    ta = s.take_lease("", "Alice", bid=A)["token"]
    s.ask_control(B)
    bridge.joint_speed = 0.5
    s.command(ta, "move_joints", {"q_deg": [30, -90, 0, -90, 0, 0], "vel": 0.1})
    assert wait(lambda: s.core.busy)
    with pytest.raises(CommandError, match="moving"):
        s.command(ta, "control_answer", {"give": True})
    assert s.heartbeat(ta)
    s.stop()
