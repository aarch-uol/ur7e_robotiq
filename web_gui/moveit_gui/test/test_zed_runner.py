"""The ZED container's runner (scripts/zed_runner.py) and the GUI side (services.RemoteProc), with a stand-in ros2."""
import importlib.util
import os
import time
from pathlib import Path

import pytest

from moveit_gui import services as SV

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("zed_runner", ROOT / "scripts" / "zed_runner.py")
ZR = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ZR)

ROS2 = """#!/bin/sh
# stand-in for `ros2 launch zed_wrapper ...`: runs until SIGINT
trap 'echo "SIGINT: shutting down"; exit 0' INT
echo "zed driver: $*"
while true; do sleep 0.1; done
"""


def wait(cond, s=15.0, step=None):
    end = time.time() + s
    while time.time() < end:
        if step:
            step()
        if cond():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def rig(tmp_path, monkeypatch):
    """A runner (stepped by hand) and a Services manager that routes the ZED to it."""
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    (bin_ / "ros2").write_text(ROS2)
    (bin_ / "ros2").chmod(0o755)
    monkeypatch.setenv("PATH", f"{bin_}:{os.environ['PATH']}")
    logs = tmp_path / "data" / "logs" / "services"
    monkeypatch.setattr(ZR, "LOGS", str(tmp_path / "data") + "/")
    root = tmp_path / "data" / "run" / "zed"
    runner = ZR.Runner(root)
    runner.step()                                     # alive

    def make():
        return SV.Services(tmp_path / "s.yaml", tmp_path / "run", logs, route=lambda s: (
            lambda argv, log: SV.RemoteProc.start(root, s.name, argv, log)), remote_root=root,
            have=lambda p: SV.remote_alive(root), publishers=lambda t: 1)
    m = make()
    m.save({"name": "zed", "type": "zed", "params": {"model": "zedxnano"}})
    yield m, runner, root, make
    runner.stop_all()


def test_runner_runs_only_the_zed_driver():
    good = ["ros2", "launch", "zed_wrapper", "zed_camera.launch.py", "camera_model:=zedxnano", "camera_name:=zed",
            "param_overrides:=pos_tracking.pos_tracking_enabled:=false;depth.depth_mode:=NONE"]
    assert ZR.ok({"argv": good, "log": "/data/logs/services/zed_1.log"}) == ""
    assert "only the ZED driver" in ZR.ok({"argv": ["bash", "-c", "id"], "log": "/data/logs/x.log"})
    assert "not allowed" in ZR.ok({"argv": good + ["camera_name:=a b"], "log": "/data/logs/x.log"})
    assert "not allowed" in ZR.ok({"argv": good + ["svo_path:=$(id)"], "log": "/data/logs/x.log"})
    assert "under /data/" in ZR.ok({"argv": good, "log": "/etc/passwd"})
    assert "under /data/" in ZR.ok({"argv": good, "log": "/data/../etc/x"})


def test_the_gui_starts_and_stops_the_zed_through_the_runner(rig):
    m, runner, root, _ = rig
    assert m.state("zed") == "stopped"
    m.start("zed")
    assert wait(lambda: m.state("zed") == "running", step=lambda: (runner.step(), m.tick()))
    run = m.runs["zed"]
    assert isinstance(run.proc, SV.RemoteProc) and run.proc.pid in [p.pid for _, p in runner.runs.values()]
    assert "publish_tf:=false" in run.argv and "pos_tracking.pos_tracking_enabled:=false" in " ".join(run.argv)
    m.stop("zed")
    assert wait(lambda: m.state("zed") == "stopped", step=lambda: (runner.step(), m.tick()))
    assert not runner.runs and "SIGINT: shutting down" in m.log_tail("zed")["text"]


def test_a_restarted_gui_adopts_the_running_zed(rig):
    m, runner, root, make = rig
    m.start("zed")
    assert wait(lambda: m.state("zed") == "running", step=lambda: (runner.step(), m.tick()))
    m2 = make()                                       # the GUI restarted; the container kept the driver
    assert isinstance(m2.runs["zed"].proc, SV.RemoteProc)
    m2.stop("zed")
    assert wait(lambda: m2.state("zed") == "stopped", step=lambda: (runner.step(), m2.tick()))


def test_the_zed_fails_when_its_container_goes(rig, monkeypatch):
    m, runner, root, _ = rig
    monkeypatch.setattr(SV, "REMOTE_TIMEOUT_S", 0.3)
    m.start("zed")
    assert wait(lambda: m.state("zed") == "running", step=lambda: (runner.step(), m.tick()))
    (root / "alive").write_text(str(time.time() - 60))  # the runner stops answering
    assert wait(lambda: "zed" not in m.runs, step=m.tick)
    assert m.state("zed") == "missing" and "ZED container is not running" in m.view()["services"][0]["msg"]
    assert m.last["zed"]["msg"] == "the ZED container stopped"


def test_no_zed_container_no_start(tmp_path):
    root = tmp_path / "zed"                           # never started: no alive file
    m = SV.Services(tmp_path / "s.yaml", tmp_path / "run", tmp_path / "logs", route=lambda s: (
        lambda argv, log: SV.RemoteProc.start(root, s.name, argv, log)), have=lambda p: SV.remote_alive(root))
    m.save({"name": "zed", "type": "zed", "params": {}})
    assert m.state("zed") == "missing" and "docker/zed/build.sh" in m.view()["services"][0]["msg"]
    with pytest.raises(SV.ServiceError):
        m.start("zed")


def test_an_alive_file_caught_mid_write_still_counts(tmp_path):
    (tmp_path / "alive").write_text("")              # an older runner truncates, then writes
    assert SV.remote_alive(tmp_path)
    os.utime(tmp_path / "alive", (time.time() - 60, time.time() - 60))
    assert not SV.remote_alive(tmp_path)
    ZR.Runner(tmp_path).step()
    assert SV.remote_alive(tmp_path) and not (tmp_path / "alive.tmp").exists()
