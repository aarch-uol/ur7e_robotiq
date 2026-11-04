"""Pausing the robot program (OPERATE > Robot, or the pendant) pauses a GUI program run instead of ending it,
power and brakes untouched; Resume carries on with the step it was on."""
import os

import numpy as np
import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from test_program_force import cmd, logs, published, s, tcp_base, tick_until  # noqa: E402,F401
from moveit_gui.web.service import CommandError  # noqa: E402


def start(s, steps):
    cmd(s, "program_set_yaml", text=yaml.safe_dump({"version": 2, "name": "t", "robot": "ur7e_lab", "steps": steps}))
    cmd(s, "program_run")


def finish(s, timeout=40):
    assert tick_until(s, lambda: s.core.runner.state.name == "IDLE" and s.core.program_result is not None, timeout)
    return s.core.program_result


def state(s):
    return s.state()


def test_pause_from_the_robot_card_pauses_the_run_and_play_resumes_both(s):
    T0 = tcp_base(s)
    start(s, [{"type": "move_relative", "offset_mm": [0, 0, 60], "frame": "base", "velocity": 0.05}])
    assert tick_until(s, lambda: np.linalg.norm(tcp_base(s)[:3, 3] - T0[:3, 3]) > 0.005, 10)
    cmd(s, "robot_pause")
    assert tick_until(s, lambda: s.core.runner.state.name == "PAUSED", 5)
    st = state(s)
    assert st["program_state"] == "PAUSED" and st["robot_mode"] == "RUNNING"          # no power off
    assert st["runner"]["pause_reason"]
    with pytest.raises(CommandError, match="cannot resume"):
        cmd(s, "program_resume")                                   # the robot program must play first
    cmd(s, "robot_play", program=s.core.bridge.pendant_program)    # ▶ Resume program
    assert finish(s) == (True, "finished")
    # the relative move ends where it was meant to from where it STARTED, not 60 mm further from the pause
    assert np.allclose(tcp_base(s)[:3, 3], T0[:3, 3] + [0, 0, 0.06], atol=2e-4)
    assert ("dashboard", "power_off") not in s.core.bridge.calls


def test_pendant_pause_mid_run_pauses_and_gui_resume_continues(s):
    start(s, [{"type": "move_relative", "offset_mm": [0, 0, 40], "frame": "base", "velocity": 0.05},
              {"type": "wait", "seconds": 0.1}])
    assert tick_until(s, lambda: s.core.runner.current_path == (0,), 5)
    s.core.bridge.dashboard("pause")                               # pressed on the pendant, not through the GUI
    assert tick_until(s, lambda: s.core.runner.state.name == "PAUSED", 5)
    assert s.core.program_result is None                           # not ended
    s.core.bridge.dashboard("play")
    assert tick_until(s, lambda: s.core.robot_refusal() is None, 5)
    assert s.core.runner.state.name == "PAUSED"                    # paused on the pendant: the GUI waits for Resume
    cmd(s, "program_resume")
    assert finish(s) == (True, "finished")


def test_stop_program_keeps_power_and_a_pressed_force_comes_back_on_resume(s):
    start(s, [{"type": "controller", "mode": "admittance"},
              {"type": "force", "wrench": [0, 0, 3, 0, 0, 0]},
              {"type": "wait", "seconds": 1.5},
              {"type": "force_off"}])
    assert tick_until(s, lambda: s.core.runner.current_path == (2,), 30)
    cmd(s, "robot_stop_program")
    assert tick_until(s, lambda: s.core.runner.state.name == "PAUSED", 5)
    assert not s.core.force_active and published(s)[-1] == [0.0] * 6          # nothing pushes while stopped
    assert state(s)["program_state"] == "STOPPED" and state(s)["robot_mode"] == "RUNNING"
    with pytest.raises(CommandError, match="not running"):
        cmd(s, "robot_pause")
    cmd(s, "robot_play", program=s.core.bridge.pendant_program)
    assert tick_until(s, lambda: s.core.runner.state.name == "RUNNING", 5)    # resumes with the robot program
    assert tick_until(s, lambda: s.core.force_active, 5)
    assert finish(s) == (True, "finished")
    assert any("force re-applied after the pause" in line for line in logs(s))

