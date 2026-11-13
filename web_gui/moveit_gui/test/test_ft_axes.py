"""SETUP > F/T + payload > Reading axes: the broadcaster publishes a turned copy of the sensor (a sensor mounted
turned), and the zero offset follows the turn."""
import numpy as np
import pytest

from moveit_gui import ft_axes as FA
from moveit_gui.ft_bias import PARAMS
from moveit_gui.web.service import CommandError
from moveit_gui.web.workcell import RESTART_STACK

from test_ft_bias import NODE, cmd, reading, result, s, wait  # noqa: F401  (s: the fixture)

TURN = {"sensor_name": "", **{f"interface_names.{k}.{a}": f"tcp_fts_sensor/{k}.{src}"
                              for k in FA.KINDS for a, src in zip("xyz", "yxz")},
        **{f"multiplier.{k}.{a}": m for k in FA.KINDS for a, m in zip("xyz", (-1.0, 1.0, 1.0))}}


def test_mapping_picks_x_and_y_and_z_follows():
    assert FA.label(FA.mapping("-y", "+x")) == {"x": "-y", "y": "+x", "z": "+z"}
    assert FA.label(FA.mapping("+z", "+x")) == {"x": "+z", "y": "+x", "z": "+y"}
    assert FA.is_identity(FA.mapping("+x", "+y"))
    for bad in (("+x", "-x"), ("x", "+y"), ("+w", "+y")):
        with pytest.raises(FA.FtAxesError):
            FA.mapping(*bad)
    m = FA.mapping("-y", "+x")
    # (sensor + offset) * multiplier, as the broadcaster does (checked on ros2_controllers 4.42.1)
    assert FA.apply(m, [1, 2, 3, .1, .2, .3], [10, 0, 0, 0, 0, 0]) == [-12, 1, 3, -0.2, 0.1, 0.3]
    b = [1, 2, 3, .1, .2, .3]
    assert np.allclose(FA.convert(FA.convert(b, None, m), m, None), b)


def test_the_file_is_the_broadcasters_parameters_and_none_for_the_sensors_own_axes(tmp_path):
    m = FA.mapping("-y", "+x")
    FA.write_next(tmp_path, m, NODE)
    import yaml
    p = yaml.safe_load((tmp_path / FA.FILE).read_text())["ft/force_torque_sensor_broadcaster"]["ros__parameters"]
    assert p["sensor_name"] == "" and p["interface_names"]["force"]["x"] == "tcp_fts_sensor/force.y"
    assert p["multiplier"]["force"] == {"x": -1.0, "y": 1.0, "z": 1.0} and p["frame_id"] == "ft_fts_link"
    assert FA.read_next(tmp_path, NODE) == m
    FA.write_next(tmp_path, FA.IDENTITY, NODE)
    assert not (tmp_path / FA.FILE).exists() and FA.is_identity(FA.read_next(tmp_path, NODE))


def test_zero_on_turned_axes_reads_zero_and_the_saved_bias_turns_with_the_axes(s):
    c, b = s.core, s.core.bridge
    raw = reading(s)                                          # the sensor's own axes (tool weight + its offset)
    b.node_params[NODE].update(TURN)                          # the stack started with the turned axes
    c.ft_bias.read()
    assert c.ft_bias.view()["axes"] == {"x": "-y", "y": "+x", "z": "+z"}
    assert np.allclose(reading(s)[:3], [-raw[1], raw[0], raw[2]], atol=0.02)   # published X = -(sensor Y)
    n = len(s.results)
    cmd(s, "ft_bias_zero", plain=True)
    r = result(s, n)
    assert r["ok"], r["msg"]
    f = c.ft_bias.view()
    # published Fx = -(sensor Fy): the bias is -RAW[1]; the offset that cancels it is on the sensor's sign
    assert np.allclose(f["bias"][:2], [-raw[1], raw[0]], atol=0.03)
    off = [b.node_params[NODE][p] for p in PARAMS]
    assert np.allclose(off[:2], [-raw[1], -raw[0]], atol=0.03)               # offset = -bias * multiplier
    assert np.allclose(reading(s), 0.0, atol=0.02)                           # Zero cancels, not doubles
    assert c.ft_bias.last["axes"] == FA.encode(FA.mapping("-y", "+x"))
    # the next stack start runs the sensor's own axes again: the saved bias is turned back before it is restored
    for k in FA.KINDS:
        for a in "xyz":
            b.node_params[NODE][f"interface_names.{k}.{a}"] = ""
            b.node_params[NODE][f"multiplier.{k}.{a}"] = 1.0
    b.node_params[NODE].update({"sensor_name": "tcp_fts_sensor", **{p: 0.0 for p in PARAMS}})
    c.ft_bias.refusal = lambda: None
    c.ft_bias._last_poll = 0.0
    c.ft_bias.poll(True)
    assert wait(lambda: not c.ft_bias._polling and not c.ft_bias.lost())
    assert np.allclose(c.ft_bias.live[:3], raw[:3], atol=0.03)               # the same zero, in the sensor's axes
    assert np.allclose(reading(s), 0.0, atol=0.02)
    assert any("turned to the new reading axes" in line for line in b._logs)


def test_save_for_the_next_start_and_with_a_restart(s):
    cfg = s.core.ft_bias.settings_path.parent.parent
    v = s.state()["ft_axes"]
    assert v["available"] and v["running"] == {"x": "+x", "y": "+y", "z": "+z"} and not v["pending"]
    r = cmd(s, "ft_axes_save", x="-y", y="+x")
    assert "next robot stack restart" in r["msg"] and r["ft_axes"]["pending"]
    assert r["ft_axes"]["next"] == {"x": "-y", "y": "+x", "z": "+z"} and r["ft_axes"]["next_name"] == "turned +90° about Z"
    assert (cfg / FA.FILE).exists()
    with pytest.raises(CommandError, match="launcher"):
        cmd(s, "ft_axes_save", x="-y", y="+x", restart=True, confirm=True)
    restarts = []
    s.on_restart = lambda: restarts.append(s.restart_code)
    with pytest.raises(CommandError, match="confirm"):
        cmd(s, "ft_axes_save", x="+x", y="+y", restart=True)
    assert (cfg / FA.FILE).exists()                                          # refused: nothing written
    with pytest.raises(CommandError, match="different"):
        cmd(s, "ft_axes_save", x="+x", y="-x")
    r = cmd(s, "ft_axes_save", x="+x", y="+y", restart=True, confirm=True)
    assert r["restart"] == "stack" and not (cfg / FA.FILE).exists()
    assert wait(lambda: restarts == [RESTART_STACK], 3)


def test_only_an_admin_changes_the_reading_axes(s):
    cfg = s.core.ft_bias.settings_path.parent.parent
    op = s.take_lease("", "op", role="operator")["token"]
    for kw in ({}, {"restart": True, "confirm": True}):
        with pytest.raises(CommandError, match="Admin"):
            s.command(op, "ft_axes_save", {"x": "-y", "y": "+x", **kw}, role="operator")
    assert not (cfg / FA.FILE).exists()
