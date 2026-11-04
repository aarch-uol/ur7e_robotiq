"""F/T bias after a robot stack restart and a GUI restart, with the robot stack's REAL F/T broadcaster
(ft_controllers.yaml on a mock sensor), a mock UR arm and the real GUI core (RobotCore + WebService + RosBridge).

Opt-in (needs docker and the robot stack image): MOVEIT_GUI_DOCKER_TESTS=1 pytest test/test_ft_bias_restart_docker.py
Image: $MOVEIT_GUI_TEST_IMAGE (default ur7e_robotiq:buildtest). No network (--network none), its own ROS_DOMAIN_ID.

What it guards: the broadcaster restarts with offsets 0 (the sensor's own offset is back, Fz ~10 N on the UR7e
tool), so the GUI must put the saved bias back by itself in every order the stack and the GUI come up."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
IMAGE = os.environ.get("MOVEIT_GUI_TEST_IMAGE", "ur7e_robotiq:buildtest")

pytestmark = pytest.mark.skipif(os.environ.get("MOVEIT_GUI_DOCKER_TESTS") != "1" or shutil.which("docker") is None,
                                reason="opt-in: MOVEIT_GUI_DOCKER_TESTS=1 (needs docker and the robot stack image)")


@pytest.fixture(scope="module")
def results():
    suite = ROOT / "test" / "ft_bias_restart"
    cmd = ["docker", "run", "--rm", "--network", "none", "-e", "ROS_DOMAIN_ID=77",
           "-v", f"{suite}:/suite:ro", "-v", f"{ROOT}:/mg:ro", IMAGE, "bash", "/suite/run.sh"]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=400).stdout
    assert "DONE" in out, out[-3000:]
    return {l.split()[1]: l for l in out.splitlines() if l.startswith("RESULT ")}


@pytest.mark.parametrize("case", ["stack_restart_gui_running", "stack_and_gui_restart", "gui_first_then_stack"])
def test_the_bias_comes_back_after_a_restart(results, case):
    assert case in results and results[case].split()[2] == "ok", results.get(case)
