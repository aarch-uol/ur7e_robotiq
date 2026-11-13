"""Profiles (user_profiles.py): each person's setups and screen layout, copied between people; and the
"Stop robot stack" marker with its start page (web/idle.py)."""
import json
import os
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from moveit_gui.setups import Setups
from moveit_gui.user_profiles import PREFS_MAX, ProfileError, Profiles
from moveit_gui.web.service import CommandError
from moveit_gui.web.workcell import RESTART_STACK

from test_workcell import env, run, wait  # noqa: F401  (the fixture)

ROOT = Path(__file__).resolve().parents[1]


def test_profiles_create_find_list_delete(tmp_path):
    p = Profiles(tmp_path)
    assert p.names() == [] and p.list() == []
    assert p.create("Cenk") == "Cenk"
    assert p.create("cenk") == "Cenk"                          # the same person, any case
    p.create("Ayesha")
    assert p.names() == ["Ayesha", "Cenk"] and p.find("AYESHA") == "Ayesha"
    for bad in ("", "1abc", "a b", "../x", "x" * 25):
        with pytest.raises(ProfileError):
            p.create(bad)
    with pytest.raises(ProfileError, match="no profile"):
        p.find("Bob")
    S = Setups(tmp_path, "ur7e_lab", owner="Cenk", root=p.setups_dir("Cenk"))
    S.save("mine")
    assert p.list()[1] == {"name": "Cenk", "setups": 1, "layout": False}
    with pytest.raises(ProfileError, match="still has setups"):
        p.delete("Cenk")
    p.delete("Ayesha")
    assert p.names() == ["Cenk"]


def test_prefs_keep_only_the_layout_keys(tmp_path):
    p = Profiles(tmp_path)
    p.create("Cenk")
    p.create("Ayesha")
    assert p.prefs("Cenk") == {}
    p.save_prefs("cenk", {"op-layouts1": "[1]", "v3-show": "{}", "browser-id": "secret", "operator": "x", "pg-path": 3})
    assert p.prefs("Cenk") == {"op-layouts1": "[1]", "v3-show": "{}"}
    with pytest.raises(ProfileError, match="no saved layout"):
        p.copy_prefs("Ayesha", "Cenk")
    assert p.copy_prefs("Cenk", "Ayesha") == p.prefs("Ayesha") == {"op-layouts1": "[1]", "v3-show": "{}"}
    with pytest.raises(ProfileError, match="too big"):
        p.save_prefs("Cenk", {"op-layouts1": "x" * (PREFS_MAX + 1)})


def as_(s, name):
    """The test browser ("me") takes control as `name` (a profile name, as the page sends it)."""
    return s.take_lease("me", f"{name} @ 100.64.0.1")


def test_setups_per_profile_in_use_marker_and_copy(env):
    s, _, tmp = env
    run(s, "profile_create", name="Cenk")
    run(s, "profile_create", name="Ayesha")
    assert [p["name"] for p in run(s, "profile_list")["profiles"]] == ["Ayesha", "Cenk"]
    as_(s, "Cenk")                                   # nobody's settings yet: they become Cenk's, no restart
    assert s.worksets.active() == "Cenk" and s.worksets.has("Cenk")
    run(s, "tcp_save", name="tip_a", parent="tool0", xyz_mm=[0, 0, 200], rpy_deg=[0, 0, 0])
    r = run(s, "setup_save", name="scraping", owner="Cenk")
    assert r["owner"] == "Cenk" and [x["name"] for x in r["setups"]] == ["scraping"]
    assert r["current"]["name"] == "scraping" and r["current"]["owner"] == "Cenk"
    assert (tmp / "cfg" / "moveit_gui" / "profiles" / "Cenk" / "setups" / "scraping" / "setup.yaml").is_file()
    assert run(s, "setup_list")["setups"] == [] and run(s, "setup_list", owner="Ayesha")["setups"] == []
    assert s.workcell_state()["setup"]["owner"] == "Cenk"
    # nobody writes into another person's profile
    with pytest.raises(CommandError, match="belongs to Ayesha"):
        run(s, "setup_copy", source_owner="Cenk", name="scraping", owner="Ayesha", new_name="x")
    with pytest.raises(CommandError, match="belongs to Ayesha"):
        run(s, "setup_save", name="x", owner="Ayesha")
    # Ayesha takes control: Cenk's settings are in use, so she must switch first
    s.on_restart = lambda: None
    as_(s, "Ayesha")
    assert s.state()["lease"]["needs_switch"] and s.state()["lease"]["settings_owner"] == "Cenk"
    with pytest.raises(CommandError, match="Cenk's settings in use"):
        run(s, "tcp_save", name="tip_b", parent="tool0", xyz_mm=[0, 0, 100], rpy_deg=[0, 0, 0])
    with pytest.raises(CommandError, match="Cenk's settings in use"):
        run(s, "setup_copy", source_owner="Cenk", name="scraping", owner="Ayesha", new_name="ayesha_scrape")
    run(s, "profile_activate")                       # she has none yet: a copy of Cenk's
    assert s.worksets.active() == "Ayesha" and not s.state()["lease"]["needs_switch"]
    assert s.restart_code is not None
    r = run(s, "setup_copy", source_owner="Cenk", name="scraping", owner="Ayesha", new_name="ayesha_scrape")
    assert [x["name"] for x in r["setups"]] == ["ayesha_scrape"]
    assert r["setups"][0]["copied_from"] == "Cenk/scraping"
    with pytest.raises(CommandError, match="already has"):
        run(s, "setup_copy", source_owner="Cenk", name="scraping", owner="Ayesha", new_name="ayesha_scrape")
    run(s, "setup_copy", source_owner="Cenk", name="scraping", owner="Ayesha", new_name="ayesha_scrape", replace=True)
    with pytest.raises(CommandError, match="no setup"):
        run(s, "setup_copy", source_owner="Ayesha", name="nope", owner="Ayesha")
    with pytest.raises(CommandError, match="no profile"):
        run(s, "setup_list", owner="Bob")
    # loading Cenk's setup into her settings: the backup goes to her profile, none to his
    run(s, "setup_load", name="scraping", owner="Cenk")
    cur = run(s, "setup_list")["current"]
    assert cur["name"] == "scraping" and cur["owner"] == "Cenk" and cur["changed"] == []
    assert any(x["auto"] for x in run(s, "setup_list", owner="Ayesha")["setups"])
    assert not any(x["auto"] for x in run(s, "setup_list", owner="Cenk")["setups"])
    # a shared setup (saved before profiles) still works, and its marker has no owner
    run(s, "setup_save", name="old")
    assert run(s, "setup_list")["current"]["owner"] == ""
    assert run(s, "setup_preview", name="ayesha_scrape", owner="Ayesha")["changes"] == []


def test_a_keepout_box_stays_with_the_one_who_added_it(env):
    s, _, tmp = env
    cfg = tmp / "cfg" / "moveit_gui"
    box = cfg / "calibrations" / "ur7e_lab.keepout.yaml"
    s.on_restart = lambda: None
    for n in ("Cenk", "Ayesha"):
        run(s, "profile_create", name=n)
    as_(s, "Cenk")
    assert not box.exists()
    as_(s, "Ayesha")
    run(s, "profile_activate")
    run(s, "keepout_save", name="her_box", frame="base_link", size_mm=[100, 100, 100], xyz_mm=[500, 300, 200],
        rpy_deg=[0, 0, 0])
    assert "her_box" in box.read_text()
    as_(s, "Cenk")                                   # Cenk switches back to his own: her box is not in them
    run(s, "profile_activate")
    assert s.worksets.active() == "Cenk" and not box.exists()
    assert "her_box" in (cfg / "profiles" / "Ayesha" / "live" / "files" / "calibrations" /
                         "ur7e_lab.keepout.yaml").read_text()
    as_(s, "Ayesha")                                 # and back: hers again
    run(s, "profile_activate")
    assert "her_box" in box.read_text()
    with pytest.raises(CommandError, match="in use"):
        run(s, "profile_delete", name="Ayesha")
    as_(s, "Cenk")
    with pytest.raises(CommandError, match="Ayesha's settings in use"):
        run(s, "keepout_delete", name="her_box")


def test_a_new_profile_can_start_from_a_setup(env):
    s, _, tmp = env
    cfg = tmp / "cfg" / "moveit_gui"
    s.on_restart = lambda: None
    for n in ("Cenk", "Bob"):
        run(s, "profile_create", name=n)
    as_(s, "Cenk")
    run(s, "tcp_save", name="tip_a", parent="tool0", xyz_mm=[0, 0, 200], rpy_deg=[0, 0, 0])
    run(s, "setup_save", name="plain", owner="Cenk")
    run(s, "tcp_save", name="tip_x", parent="tool0", xyz_mm=[0, 0, 50], rpy_deg=[0, 0, 0])
    as_(s, "Bob")
    with pytest.raises(CommandError, match="no setup"):
        run(s, "profile_activate", start="setup", owner="Cenk", name="nope")
    assert s.worksets.active() == "Cenk"            # a refused switch changes nothing
    run(s, "profile_activate", start="setup", owner="Cenk", name="plain")
    cal = (cfg / "calibrations" / "ur7e_lab.yaml").read_text()
    assert "tip_a" in cal and "tip_x" not in cal
    assert "tip_x" in (cfg / "profiles" / "Cenk" / "live" / "files" / "calibrations" / "ur7e_lab.yaml").read_text()
    as_(s, "nobody")                                 # a name that is no profile cannot control the robot now
    with pytest.raises(CommandError, match="pick your profile"):
        run(s, "tcp_save", name="tip_y", parent="tool0", xyz_mm=[0, 0, 50], rpy_deg=[0, 0, 0])


def test_an_old_marker_without_owner_is_a_shared_setup(tmp_path):
    S = Setups(tmp_path, "ur7e_lab")
    S.save("x")
    (tmp_path / "setups" / "current.yaml").write_text("name: x\nat: '2026-01-01 10:00'\n")
    assert S.current() == "x"
    assert Setups(tmp_path, "ur7e_lab", owner="Cenk", root=tmp_path / "profiles" / "Cenk" / "setups").current() == ""


def test_profile_commands_need_no_control_but_delete_and_copy_do(env):
    s, _, _ = env
    s.command("other", "profile_create", {"name": "Cenk"})          # "other" does not hold the lease
    s.command("other", "profile_prefs_save", {"name": "Cenk", "prefs": {"v3-show": "{}"}})
    assert s.command("other", "profile_prefs_get", {"name": "Cenk"})["prefs"] == {"v3-show": "{}"}
    with pytest.raises(CommandError, match="take control"):
        s.command("other", "setup_copy", {"source_owner": "", "name": "x", "owner": "Cenk"})
    with pytest.raises(CommandError, match="only an Admin"):
        s.command("me", "profile_delete", {"name": "Cenk"}, role="operator")


def test_stack_stop_writes_the_marker_and_restarts(env):
    s, _, tmp = env
    with pytest.raises(CommandError, match="launcher"):
        run(s, "stack_stop")
    got = []
    s.on_restart = lambda: got.append(s.restart_code)
    s.core.busy = True
    with pytest.raises(CommandError, match="busy"):
        run(s, "stack_stop", who="Cenk")
    s.core.busy = False
    run(s, "stack_stop", who="Cenk")
    m = tmp / "cfg" / "moveit_gui" / "stack_off"
    assert json.loads(m.read_text())["by"] == "Cenk"
    assert wait(lambda: got == [RESTART_STACK], 3)


def _free_port():
    with socket.socket() as k:
        k.bind(("127.0.0.1", 0))
        return k.getsockname()[1]


def test_the_start_page_removes_the_marker_and_exits_75(tmp_path):
    cfg = tmp_path / "moveit_gui"
    cfg.mkdir()
    (cfg / "stack_off").write_text(json.dumps({"by": "Cenk", "at": "2026-11-10 17:00"}))
    port = _free_port()
    envv = dict(os.environ, XDG_CONFIG_HOME=str(tmp_path), PYTHONPATH=str(ROOT))
    p = subprocess.Popen(["python3", "-m", "moveit_gui.web.idle", "--port", str(port)], env=envv, cwd=str(tmp_path),
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        url = f"http://127.0.0.1:{port}"
        for _ in range(100):
            try:
                page = urllib.request.urlopen(url + "/").read().decode()
                break
            except OSError:
                time.sleep(0.05)
        assert "Start robot stack" in page and "Stopped by Cenk" in page
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(url + "/api/info")
        assert e.value.code == 503 and json.loads(e.value.read())["stack"] == "stopped"
        bad = urllib.request.Request(url + "/api/stack_start", data=b"{}", method="POST",
                                     headers={"Content-Type": "application/json", "Origin": "http://evil.example"})
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(bad)
        assert e.value.code == 403 and (cfg / "stack_off").exists()
        ok = urllib.request.Request(url + "/api/stack_start", data=b"{}", method="POST",
                                    headers={"Content-Type": "application/json"})
        assert json.loads(urllib.request.urlopen(ok).read())["ok"]
        assert p.wait(5) == RESTART_STACK and not (cfg / "stack_off").exists()
    finally:
        if p.poll() is None:
            p.kill()


def test_programs_per_profile(env):
    s, _, tmp = env
    a = "me"
    s.on_restart = lambda: None
    s.command(a, "profile_create", {"name": "Ayesha"})
    s.command(a, "profile_create", {"name": "Cenk"})
    as_(s, "Ayesha")
    s.command(a, "program_save", {"name": "shared1"})
    r = s.command(a, "program_save", {"name": "mine", "owner": "ayesha"})
    assert (tmp / "progs" / "Ayesha" / "mine.yaml").is_file() and r["program"]["owner"] == "Ayesha"
    assert s.command(a, "program_list", {"owner": "Ayesha"})["files"] == ["mine.yaml"]
    assert "mine.yaml" not in s.command(a, "program_list", {})["files"]
    assert "shared1.yaml" in s.command(a, "program_list", {})["files"]
    s.command(a, "program_save", {})                              # saves where it is open
    assert not (tmp / "progs" / "mine.yaml").exists()
    with pytest.raises(CommandError, match="belongs to Cenk"):     # not into another person's programs
        s.command(a, "program_copy", {"source_owner": "", "name": "shared1.yaml", "owner": "Cenk"})
    as_(s, "Cenk")
    s.command(a, "profile_activate", {})
    with pytest.raises(CommandError, match="belongs to Ayesha"):   # her program is open: he cannot save over it
        s.command(a, "program_save", {})
    r = s.command(a, "program_copy", {"source_owner": "", "name": "shared1.yaml", "owner": "Cenk"})
    assert r["files"] == ["shared1.yaml"] and (tmp / "progs" / "shared1.yaml").is_file()
    with pytest.raises(CommandError, match="already exists"):
        s.command(a, "program_copy", {"source_owner": "", "name": "shared1.yaml", "owner": "Cenk"})
    s.command(a, "program_copy", {"source_owner": "Ayesha", "name": "mine.yaml", "owner": "Cenk", "new_name": "x"})
    assert (tmp / "progs" / "Cenk" / "x.yaml").is_file()
    s.command(a, "program_open", {"name": "x.yaml", "owner": "Cenk"})
    assert s.program_path == (tmp / "progs" / "Cenk" / "x.yaml").resolve()
    for bad in ("../shared1.yaml", "a/b.yaml"):
        with pytest.raises(CommandError):
            s.command(a, "program_open", {"name": bad, "owner": "Cenk"})
    with pytest.raises(CommandError, match="no profile"):
        s.command(a, "program_list", {"owner": "../etc"})
