"""SETUP > Drivers & cameras: services.py (definitions, start/stop with real child processes, needs, adopt)."""
import os
import time

import pytest
import yaml

from moveit_gui import services as SV


def wait(cond, t=8.0):
    end = time.monotonic() + t
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def mgr(tmp_path):
    pubs = set()
    made = []

    def make(**kw):
        m = SV.Services(tmp_path / "s.yaml", tmp_path / "run", tmp_path / "logs", spawn=SV.spawn_fake,
                        have=kw.pop("have", lambda p: True), publishers=lambda t: 1 if t in pubs else 0,
                        on_change=lambda s, up: [(pubs.add if up else pubs.discard)(o["topic"]) for o in s.outputs()],
                        **kw)
        made.append(m)
        return m
    yield make, pubs
    for m in made:
        m.stop_all(wait=5)


def test_definitions_and_commands(tmp_path):
    s = SV.make_service({"name": "wrist", "type": "realsense", "params": {"serial": "123", "depth": False}})
    a = s.argv()
    assert a[:4] == ["ros2", "launch", "realsense2_camera", "rs_launch.py"]
    assert "camera_name:=wrist" in a and "serial_no:='123'" in a and "enable_depth:=false" in a
    assert [o["topic"] for o in s.outputs()] == ["/camera/wrist/color/image_raw", "/camera/wrist/color/camera_info"]
    u = SV.make_service({"name": "web", "type": "usb_cam", "params": {"size": "1280,720,30"}})
    assert "image_width:=1280" in u.argv() and u.outputs()[0]["topic"] == "/web/image_raw"
    r = SV.make_service({"name": "midpoints", "type": "run", "params": {
        "package": "scraping_vision", "exe": "node", "args": "--ros-args -p rate:=10",
        "outputs": "/scraping_vision/midpoints /scraping_vision/annotated_image"}})
    assert r.kind == "pipeline" and r.argv()[-3:] == ["--ros-args", "-p", "rate:=10"]
    assert [o["image"] for o in r.outputs()] == [False, True]
    for bad in ({"name": "Bad", "type": "usb_cam"}, {"name": "x", "type": "nope"},
                {"name": "x", "type": "usb_cam", "params": {"device": "; rm"}},
                {"name": "x", "type": "run", "params": {"package": "a b", "exe": "n"}},
                {"name": "x", "type": "launch", "params": {"package": "p", "file": "f", "args": "'open"}},
                {"name": "x", "type": "run", "params": {"package": "p", "exe": "n", "outputs": "nope"}},
                {"name": "x", "type": "realsense", "params": {"size": "1x1"}}):
        with pytest.raises(SV.ServiceError):
            SV.make_service(bad)


def test_start_ready_stop_and_log(mgr):
    make, pubs = mgr
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    assert m.state("cam") == "stopped"
    assert "starting cam" in m.start("cam")
    assert wait(lambda: (m.tick(), m.state("cam"))[1] == "running")
    assert m.ready("cam")
    pid = m.runs["cam"].proc.pid
    assert wait(lambda: "fake driver" in m.log_tail("cam")["text"])
    with pytest.raises(SV.ServiceError, match="stop 'cam' first"):
        m.delete("cam")
    m.stop("cam")
    assert m.state("cam") == "stopping"
    assert wait(lambda: (m.tick(), m.state("cam"))[1] == "stopped")
    assert not SV._alive(pid)
    log = m.log_tail("cam")["text"]
    assert "fake driver: ros2 run usb_cam" in log and "SIGINT" in log
    v = m.view()["services"][0]
    assert v["state"] == "stopped" and v["outputs"][0]["topic"] == "/cam/image_raw"
    assert yaml.safe_load((m.path).read_text())["services"][0]["name"] == "cam"


def test_crash_is_reported(mgr, tmp_path):
    make, _ = mgr
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    m.start("cam")
    os.killpg(m.runs["cam"].proc.pid, 9)
    assert wait(lambda: (m.tick(), m.state("cam"))[1] == "failed")
    assert "stopped by itself (exit code -9)" in m.view()["services"][0]["msg"]


def test_needs_start_first_and_stop_after(mgr):
    make, pubs = mgr
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    m.save({"name": "pipe", "type": "run", "needs": ["cam"],
            "params": {"package": "vis", "exe": "node", "outputs": "/pipe/out"}})
    msg = m.start("pipe")
    assert "after cam" in msg and "pipe" in m.waiting and "cam" in m.runs
    assert wait(lambda: (m.tick(), m.state("pipe"))[1] == "running")
    assert "and pipe, which need it" in m.stop("cam")
    assert wait(lambda: (m.tick(), not m.runs)[1])
    with pytest.raises(SV.ServiceError, match="pipe needs 'cam'"):
        m.delete("cam")
    with pytest.raises(SV.ServiceError, match="circle"):
        m.save({"name": "cam", "type": "usb_cam", "needs": ["pipe"]}, replace="cam")
    m.save({"name": "cam2", "type": "usb_cam"}, replace="cam")    # a rename keeps the need
    assert m.services["pipe"].needs == ["cam2"]


def test_missing_package_and_autostart(mgr):
    make, _ = mgr
    m = make(have=lambda p: p != "zed_wrapper")
    m.save({"name": "zed", "type": "zed", "autostart": True})
    m.save({"name": "cam", "type": "usb_cam", "autostart": True})
    assert m.state("zed") == "missing"
    with pytest.raises(SV.ServiceError, match="zed_wrapper is not installed"):
        m.start("zed")
    m.tick()                                                      # autostart: cam starts, zed reports why not
    assert "cam" in m.runs and "zed" not in m.runs
    assert "autostart" in m.view()["services"][0]["msg"] or m.state("zed") == "missing"


def test_a_new_gui_adopts_running_drivers(mgr):
    make, _ = mgr
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    m.start("cam")
    pid = m.runs["cam"].proc.pid
    m2 = make()                                                   # the GUI restarted: same files
    assert isinstance(m2.runs["cam"].proc, SV.Adopted) and m2.runs["cam"].proc.pid == pid
    m.runs.clear()                                                # the old GUI is gone
    m2.stop("cam")
    assert wait(lambda: (m2.tick(), m2.state("cam"))[1] == "stopped")
    assert not (m2.run_dir / "cam.json").exists()
    m3 = make()
    assert not m3.runs
