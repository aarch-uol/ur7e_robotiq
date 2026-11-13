"""Roles: Viewer watches, Operator runs the robot, Admin also changes the setup. This PC is always Admin."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.web.roles import Roles, RolesError  # noqa: E402
from moveit_gui.web.service import ADMIN_COMMANDS, CommandError  # noqa: E402
from test_web import env, wait  # noqa: E402,F401  (fixture reuse)


def test_no_file_means_everyone_admin(tmp_path):
    r = Roles(tmp_path / "roles.yaml")
    assert r.role("bob@x", "100.1.1.1", local=False) == "admin" and not r.view()["saved"]


def test_saved_roles_by_login_or_ip(tmp_path):
    r = Roles(tmp_path / "roles.yaml")
    r.save("viewer", {"alice@x": "admin", "100.1.1.2": "operator"})
    r2 = Roles(tmp_path / "roles.yaml")                          # read back from the file
    assert r2.role("alice@x", "127.0.0.1", False) == "admin"
    assert r2.role("", "100.1.1.2", False) == "operator"
    assert r2.role("mallory@x", "100.9.9.9", False) == "viewer"
    assert r2.role("mallory@x", "127.0.0.1", local=True) == "admin"   # the robot PC: never locked out


def test_bad_input_and_broken_file(tmp_path):
    r = Roles(tmp_path / "roles.yaml")
    with pytest.raises(RolesError):
        r.save("boss", {})
    with pytest.raises(RolesError):
        r.save("operator", {"a b": "admin"})
    with pytest.raises(RolesError):
        r.save("operator", {"a@x": "god"})
    (tmp_path / "roles.yaml").write_text("default: [unclosed")
    assert Roles(tmp_path / "roles.yaml").role("x", "1.2.3.4", False) == "viewer"   # broken: not everyone admin


def test_admin_commands_need_admin(env):
    s, _, _, _ = env
    t = s.take_lease("", "op", role="operator")["token"]
    with pytest.raises(CommandError, match="only an Admin"):
        s.command(t, "keepout_delete", {"name": "x"}, role="operator")
    s.command(t, "program_list", {}, role="operator")                # running things is fine
    assert "roles_save" in ADMIN_COMMANDS and "move_joints" not in ADMIN_COMMANDS and "restart" not in ADMIN_COMMANDS


def test_policy_import_sources_and_upload_need_admin(env):
    s, _, _, _ = env
    token = s.take_lease("", "operator", role="operator")["token"]
    for command in ("policy_sources", "policy_upload"):
        with pytest.raises(CommandError, match="only an Admin"):
            s.command(token, command, {"folder": "x", "files": []} if command == "policy_upload" else {},
                      role="operator")


def test_viewer_cannot_take_or_ask_for_control(env):
    s, _, _, _ = env
    with pytest.raises(CommandError, match="Viewer"):
        s.take_lease("", "v", role="viewer")
    s.take_lease("", "op", role="operator")
    with pytest.raises(CommandError, match="Viewer"):
        s.ask_control("v" * 22, role="viewer")
    assert s.stop("v")["ok"]                                         # STOP: anyone


def test_admin_commands_exist():
    from moveit_gui.web.service import WebService
    missing = [c for c in ADMIN_COMMANDS if not hasattr(WebService, f"cmd_{c}")]
    assert not missing, missing


def test_admin_lists_policy_sources_without_control(env):
    s, _, _, _ = env
    out = s.command("", "policy_sources", {}, role="admin")
    assert out["ok"] and out["roots"]
