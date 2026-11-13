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

from moveit_gui import setups as setups_mod
from moveit_gui.setups import Setups
from moveit_gui.user_profiles import PREFS_MAX, ProfileError, Profiles
from moveit_gui.worksets import WorkSets
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


def test_put_in_place_restores_live_settings_after_mid_copy_failure(tmp_path, monkeypatch):
    cfg, src = tmp_path / "cfg", tmp_path / "incoming"
    old_cal = cfg / "calibrations" / "ur7e_lab.yaml"
    old_bias = cfg / "ft_bias" / "ur7e_lab.json"
    new_cal = src / "calibrations" / "ur7e_lab.yaml"
    new_bias = src / "ft_bias" / "ur7e_lab.json"
    for pth, text in ((old_cal, "old calibration"), (old_bias, "old bias"),
                      (new_cal, "new calibration"), (new_bias, "new bias")):
        pth.parent.mkdir(parents=True, exist_ok=True)
        pth.write_text(text)
    (src / ".strict").write_text("")

    original = setups_mod.shutil.copy2
    failed = [False]

    def fail_once(a, b, *args, **kwargs):
        if not failed[0] and Path(a) == new_bias:
            failed[0] = True
            raise OSError("simulated copy failure")
        return original(a, b, *args, **kwargs)

    monkeypatch.setattr(setups_mod.shutil, "copy2", fail_once)
    with pytest.raises(OSError, match="simulated copy failure"):
        setups_mod.put_in_place(cfg, "ur7e_lab", src)
    assert old_cal.read_text() == "old calibration"
    assert old_bias.read_text() == "old bias"
    assert not list(cfg.glob(".settings-rollback-*"))


def test_profiles_notice_external_directory_changes(tmp_path):
    p = Profiles(tmp_path)
    p.create("Cenk")
    assert p.names() == ["Cenk"]
    (p.dir / "Ayesha" / "setups").mkdir(parents=True)
    assert p.names() == ["Ayesha", "Cenk"]
    (p.dir / "Ayesha" / "setups").rmdir()
    (p.dir / "Ayesha").rmdir()
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


def test_profile_switch_restores_old_owner_if_new_profile_save_fails(env, monkeypatch):
    s, _, tmp = env
    cfg = tmp / "cfg" / "moveit_gui"
    box = cfg / "calibrations" / "ur7e_lab.keepout.yaml"
    s.on_restart = lambda: None
    for n in ("Cenk", "Ayesha"):
        run(s, "profile_create", name=n)
    as_(s, "Cenk")
    as_(s, "Ayesha")
    run(s, "profile_activate")
    run(s, "keepout_save", name="her_box", frame="base_link", size_mm=[100, 100, 100],
        xyz_mm=[500, 300, 200], rpy_deg=[0, 0, 0])
    as_(s, "Cenk")
    run(s, "profile_activate")
    assert s.worksets.active() == "Cenk" and not box.exists()

    original = s.worksets.save

    def fail_new(name, *args, **kwargs):
        if str(name).lower() == "ayesha":
            raise OSError("simulated profile save failure")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(s.worksets, "save", fail_new)
    as_(s, "Ayesha")
    with pytest.raises(CommandError, match="could not switch settings"):
        run(s, "profile_activate")
    assert s.worksets.active() == "Cenk"
    assert not box.exists()


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
    with pytest.raises(CommandError, match="take control"):
        s.command("other", "profile_prefs_save", {"name": "Cenk", "prefs": {"v3-show": "{}"}})
    assert s.command("other", "profile_prefs_get", {"name": "Cenk"})["prefs"] == {}
    with pytest.raises(CommandError, match="take control"):
        s.command("other", "setup_copy", {"source_owner": "", "name": "x", "owner": "Cenk"})
    with pytest.raises(CommandError, match="only an Admin"):
        s.command("me", "profile_delete", {"name": "Cenk"}, role="operator")


def test_profile_prefs_save_only_writes_the_callers_profile(env):
    s, _, _ = env
    run(s, "profile_create", name="Cenk")
    run(s, "profile_create", name="Ayesha")
    s.take_lease("me", "Cenk @ 100.64.0.1")
    run(s, "profile_prefs_save", name="Cenk", prefs={"v3-show": "mine"})
    with pytest.raises(CommandError, match="your own profile"):
        run(s, "profile_prefs_save", name="Ayesha", prefs={"v3-show": "not mine"})


def test_profile_switch_ignores_faulted_gripper_and_stale_none_mode(env):
    s, bridge, _ = env
    run(s, "profile_create", name="alpha")
    run(s, "profile_create", name="beta")
    as_(s, "alpha")
    as_(s, "beta")
    s.on_restart = lambda: None

    bridge.gripper_fault = 5
    bridge.gripper_controllers[bridge.profile.gripper["controller"]] = "inactive"
    s.core.mode = "none"                         # stale cached mode; arm controllers remain in Position

    out = run(s, "profile_activate")
    assert out["ok"] and s.worksets.active() == "beta"


def test_profile_switch_none_mode_requires_fresh_controllers_and_standstill(env, monkeypatch):
    s, bridge, _ = env
    run(s, "profile_create", name="alpha")
    run(s, "profile_create", name="beta")
    as_(s, "alpha")
    as_(s, "beta")
    s.on_restart = lambda: None
    s.core.mode = "none"

    real = bridge.snapshot

    def stale():
        snap = real()
        snap.controllers_age = 10.0
        return snap

    monkeypatch.setattr(bridge, "snapshot", stale)
    with pytest.raises(CommandError, match="controller list"):
        run(s, "profile_activate")
    monkeypatch.setattr(bridge, "snapshot", real)
    bridge._qd = [0.1] * len(bridge._qd)
    with pytest.raises(CommandError, match="moving"):
        run(s, "profile_activate")


def test_profile_switch_is_pinned_to_the_callers_lease(env, monkeypatch):
    s, _, _ = env
    for n in ("alpha", "beta", "gamma"):
        run(s, "profile_create", name=n)
    as_(s, "alpha")
    as_(s, "beta")
    s.on_restart = lambda: None
    original = s.worksets.save

    def take_over_before_commit(name, *args, **kwargs):
        if str(name).lower() == "beta":
            s.take_lease("other", "gamma @ 100.64.0.2")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(s.worksets, "save", take_over_before_commit)
    with pytest.raises(CommandError, match="control changed"):
        run(s, "profile_activate")


def test_profile_switch_keeps_refusing_a_known_non_moveit_mode(env):
    s, _, _ = env
    run(s, "profile_create", name="alpha")
    run(s, "profile_create", name="beta")
    as_(s, "alpha")
    as_(s, "beta")
    s.on_restart = lambda: None
    s.core.mode = "impedance"
    with pytest.raises(CommandError, match="change to Position"):
        run(s, "profile_activate")


def test_refused_commands_are_written_to_the_gui_log(env):
    s, bridge, _ = env
    run(s, "profile_create", name="alpha")
    run(s, "profile_create", name="beta")
    as_(s, "alpha")
    as_(s, "beta")
    with pytest.raises(CommandError, match="launcher"):
        run(s, "profile_activate")
    assert any("REFUSED profile_activate:" in line and "launcher" in line
               for line in bridge.drain_logs())


def test_stack_stop_does_not_require_profile_settings_switch(env):
    s, _, tmp = env
    run(s, "profile_create", name="Cenk")
    run(s, "profile_create", name="Ayesha")
    as_(s, "Cenk")
    as_(s, "Ayesha")
    assert s.state()["lease"]["needs_switch"]
    s.on_restart = lambda: None
    out = run(s, "stack_stop", who="Ayesha")
    assert "stopping" in out["msg"]
    m = tmp / "cfg" / "moveit_gui" / "stack_off"
    assert json.loads(m.read_text())["by"] == "Ayesha"


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
    again = run(s, "stack_stop", who="Cenk")
    assert "already stopping" in again["msg"]
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
        assert "Start robot" in page and "Stopped by Cenk at 17:00" in page
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
        first = json.loads(urllib.request.urlopen(ok).read())
        assert first["ok"] and first["starting"]
        again = urllib.request.Request(url + "/api/stack_start", data=b"{}", method="POST",
                                       headers={"Content-Type": "application/json"})
        second = json.loads(urllib.request.urlopen(again).read())
        assert second["ok"] and second["starting"] and "already" in second["msg"].lower()
        try:
            urllib.request.urlopen(url + "/api/info")
        except urllib.error.HTTPError as e:
            assert e.code == 503 and json.loads(e.read())["stack"] == "starting"
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
    with pytest.raises(CommandError, match="belongs to Ayesha"):   # nor the batch kept next to it
        s.command(a, "batch_save", {"batch": {"kind": "repeat", "repeats": 2}})
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


def test_a_failed_first_switch_puts_the_settings_back(env, monkeypatch):
    """Nobody's settings in use yet (from before profiles): a failed switch has no owner to load, so the snapshot
    taken before the switch is put back."""
    s, _, tmp = env
    cfg = tmp / "cfg" / "moveit_gui"
    box = cfg / "calibrations" / "ur7e_lab.keepout.yaml"
    s.on_restart = lambda: None
    run(s, "profile_create", name="Cenk")
    as_(s, "Cenk")
    run(s, "keepout_save", name="old_box", frame="base_link", size_mm=[100, 100, 100], xyz_mm=[500, 0, 200],
        rpy_deg=[0, 0, 0])
    run(s, "setup_save", name="with_old_box")                       # a shared setup
    run(s, "keepout_save", name="new_box", frame="base_link", size_mm=[100, 100, 100], xyz_mm=[500, 300, 200],
        rpy_deg=[0, 0, 0])
    (cfg / "profiles" / "active.yaml").unlink(missing_ok=True)       # the settings from before profiles: no owner
    before = box.read_text()
    assert s.worksets.active() == "" and "new_box" in before

    def fail(name):
        raise OSError("simulated marker failure")
    monkeypatch.setattr(s.worksets, "set_active", fail)
    with pytest.raises(CommandError, match="could not switch settings"):
        run(s, "profile_activate", start="setup", owner="", name="with_old_box")
    assert box.read_text() == before and s.worksets.active() == ""
    assert not list((cfg / "profiles").glob(".switch-*"))


def test_settings_switch_recovery_rolls_back_before_publish(tmp_path):
    cfg = tmp_path / "cfg"
    p = Profiles(cfg)
    p.create("old")
    p.create("new")
    live = cfg / "calibrations" / "ur7e_lab.yaml"
    live.parent.mkdir(parents=True, exist_ok=True)
    live.write_text("old")
    ws = WorkSets(cfg, "ur7e_lab", p)
    ws.set_active("old")
    target = tmp_path / "target"
    (target / "calibrations").mkdir(parents=True)
    (target / "calibrations" / "ur7e_lab.yaml").write_text("new")
    (target / ".strict").write_text("")

    tx = ws.begin_switch("new")
    ws.stage_switch(tx, target, {})
    # A killed process leaves the journal and the old active marker; startup must undo the staged switch.
    ws.recover()
    assert live.read_text() == "old" and ws.active() == "old"
    assert not list((cfg / "profiles").glob(".switch-*"))


def test_settings_switch_recovery_finishes_after_active_publish(tmp_path):
    cfg = tmp_path / "cfg"
    p = Profiles(cfg)
    p.create("old")
    p.create("new")
    live = cfg / "calibrations" / "ur7e_lab.yaml"
    live.parent.mkdir(parents=True, exist_ok=True)
    live.write_text("old")
    ws = WorkSets(cfg, "ur7e_lab", p)
    ws.set_active("old")
    target = tmp_path / "target"
    (target / "calibrations").mkdir(parents=True)
    (target / "calibrations" / "ur7e_lab.yaml").write_text("new")
    (target / ".strict").write_text("")

    tx = ws.begin_switch("new")
    ws.stage_switch(tx, target, {})
    ws.apply_switch(tx)
    ws.set_active("new")             # killed after the final rename, before journal cleanup
    ws = WorkSets(cfg, "ur7e_lab", p)
    ws.recover()
    assert live.read_text() == "new" and ws.active() == "new"
    assert not list((cfg / "profiles").glob(".switch-*"))


def test_two_restart_requests_at_once_restart_once(env):
    s, _, _ = env
    calls = []
    s.on_restart = lambda: calls.append(1)
    import threading as th
    go, got = th.Barrier(8), []

    def ask(i):
        go.wait()
        got.append(s._restart(RESTART_STACK if i % 2 else 76, f"t{i}"))
    ts = [th.Thread(target=ask, args=(i,)) for i in range(8)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert got.count(True) == 1
    assert wait(lambda: calls == [1], 3) and s.restart_code in (RESTART_STACK, 76)


def test_the_progress_page_shows_how_far_the_start_is(tmp_path):
    cfg = tmp_path / "moveit_gui"
    cfg.mkdir()
    (cfg / "stack_progress").write_text("stage=driver\nlog=launch_20261111_080000.log\n")
    port = _free_port()
    envv = dict(os.environ, XDG_CONFIG_HOME=str(tmp_path), PYTHONPATH=str(ROOT))
    p = subprocess.Popen(["python3", "-m", "moveit_gui.web.idle", "--port", str(port), "--progress"], env=envv,
                         cwd=str(tmp_path), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        url = f"http://127.0.0.1:{port}"

        def info():
            with pytest.raises(urllib.error.HTTPError) as e:
                urllib.request.urlopen(url + "/api/info")
            return e.value.code, json.loads(e.value.read())
        for _ in range(100):
            try:
                page = urllib.request.urlopen(url + "/").read().decode()
                break
            except OSError:
                time.sleep(0.05)
        assert 'id="st-driver"' in page and "initiallyStarting=true" in page
        assert info() == (503, {"ok": False, "stack": "starting", "starting": True, "stage": "driver",
                                "log": "launch_20261111_080000.log"})
        (cfg / "stack_progress").write_text("stage=moveit\nlog=../../etc/passwd\n")
        assert info()[1]["stage"] == "moveit" and "log" not in info()[1]
        ok = urllib.request.Request(url + "/api/stack_start", data=b"{}", method="POST",
                                    headers={"Content-Type": "application/json"})
        assert "already" in json.loads(urllib.request.urlopen(ok).read())["msg"].lower()
        (cfg / "stack_progress").write_text("stage=gui\n")             # the launcher starts the GUI next
        assert p.wait(5) == 0
    finally:
        if p.poll() is None:
            p.kill()
