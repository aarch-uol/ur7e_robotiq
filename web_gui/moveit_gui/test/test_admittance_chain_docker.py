"""Force + motion cycles on the REAL admittance controller chain (ros2_control mock UR arm, admittance_controller,
admittance_trajectory_controller, simulated F/T), driven through the GUI's RosBridge.

Opt-in (needs docker and the robot stack image): MOVEIT_GUI_DOCKER_TESTS=1 pytest test/test_admittance_chain_docker.py
Image: $MOVEIT_GUI_TEST_IMAGE (default ur7e_robotiq:buildtest). The container has no network (--network none) and its
own ROS_DOMAIN_ID, so it can never reach a robot.

What it guards: under an applied force the arm stands off the trajectory controller's reference (5 N at 250 N/m:
~20 mm). A move started at the measured joints, or a pause / STOP that cancels the goal (the controller then holds
at the measured joints), jumped the reference there and the push doubled, again at every cycle."""
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
def metrics():
    suite = ROOT / "test" / "admittance_chain"
    cmd = ["docker", "run", "--rm", "--network", "none", "-e", "ROS_DOMAIN_ID=77",
           "-v", f"{suite}:/suite:ro", "-v", f"{ROOT}:/mg:ro", IMAGE, "bash", "/suite/run.sh"]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=300).stdout
    assert "DONE" in out, out[-3000:]
    return {l.split()[1]: float(l.split()[2]) for l in out.splitlines() if l.startswith("METRIC ")}


def test_the_force_pushes_the_arm_off_its_reference(metrics):
    assert 15.0 < metrics["push_mm"] < 25.0                    # 5 N / 250 N/m, the setup works


def test_hold_and_pause_cycles_do_not_walk_the_arm_out(metrics):
    assert metrics["walk_after_5_hold_cycles_mm"] < 3.0        # was ~20 mm more per cycle


def test_a_pause_stops_the_arm_where_it_is(metrics):
    assert metrics["worst_move_after_pause_mm"] < 3.0          # was ~20 mm


def test_a_move_starts_without_a_jump(metrics):
    assert metrics["worst_move_first_0p25s_mm"] < 4.0          # was ~20 mm


def test_stop_releases_the_push_smoothly(metrics):
    assert 15.0 < metrics["stop_back_mm"] < 25.0               # back to the reference, by the push
    assert metrics["stop_biggest_20ms_step_mm"] < 2.0          # eased back, no jump
