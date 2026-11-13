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


def test_resolution_frame_rate_and_display_name(tmp_path):
    rs = SV.make_service({"name": "d405", "label": " D405  wrist ", "type": "realsense",
                          "params": {"resolution": "848x480", "fps": "60"}})
    assert rs.title == "D405 wrist" and "depth_module.depth_profile:=848,480,60" in rs.argv()
    old = SV.make_service({"name": "web", "type": "usb_cam", "params": {"size": "1280,720,15"}})   # saved before
    assert old.params["resolution"] == "1280x720" and "framerate:=15.0" in old.argv() and old.title == "web"
    with pytest.raises(SV.ServiceError, match="30 fps"):
        SV.make_service({"name": "x", "type": "realsense", "params": {"resolution": "1280x720", "fps": "60"}})
    z = SV.make_service({"name": "zed_x_nano", "type": "zed",
                         "params": {"model": "zedxnano", "resolution": "SVGA", "fps": "120", "publish": "2"}})
    ov = next(a for a in z.argv() if a.startswith("param_overrides:="))
    assert "general.grab_resolution:=SVGA" in ov and "general.grab_frame_rate:=120" in ov
    assert "general.pub_resolution:=CUSTOM;general.pub_downscale_factor:=2.0" in ov
    assert "grab_resolution" not in " ".join(SV.make_service({"name": "z", "type": "zed", "params": {}}).argv())
    with pytest.raises(SV.ServiceError, match="15, 30, 60 fps"):
        SV.make_service({"name": "z", "type": "zed", "params": {"model": "zedx", "resolution": "HD1200", "fps": "120"}})
    with pytest.raises(SV.ServiceError, match="resolution one of"):
        SV.make_service({"name": "z", "type": "zed", "params": {"model": "zed2i", "resolution": "HD1200"}})


def test_rename_while_running(mgr):
    m = mgr[0]()
    m.save({"name": "cam", "type": "usb_cam", "params": {}})
    m.start("cam")
    assert m.view()["services"][0]["label"] == "cam"
    m.save({"name": "cam", "label": "Wrist webcam", "type": "usb_cam", "params": {}}, replace="cam")
    assert m.view()["services"][0]["label"] == "Wrist webcam"
    with pytest.raises(SV.ServiceError, match="stop 'cam' first"):
        m.save({"name": "cam", "label": "Wrist webcam", "type": "usb_cam", "params": {"fps": "15"}}, replace="cam")


def test_the_process_command_line_matches_a_ros2_command():
    argv = ["ros2", "launch", "realsense2_camera", "rs_launch.py", "camera_name:=d405"]
    assert SV._matches(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", *argv[1:]], argv)
    assert SV._matches(argv, argv)
    assert not SV._matches(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", *argv[1:-1], "camera_name:=d435"], argv)
    assert not SV._matches(argv[1:], argv)


def test_a_new_gui_adopts_a_driver_without_pid_file(mgr):
    make, _ = mgr
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    m.start("cam")
    proc = m.runs["cam"].proc
    m.runs.clear()
    (m.run_dir / "cam.json").unlink()                              # an earlier GUI lost track of it
    m2 = make()
    assert isinstance(m2.runs["cam"].proc, SV.Adopted) and m2.runs["cam"].proc.pid == proc.pid
    m2.stop("cam")
    assert wait(lambda: (m2.tick(), m2.state("cam"))[1] == "stopped") and wait(lambda: proc.poll() is not None)


def test_two_copies_of_a_driver_are_both_stopped_and_one_is_started(mgr):
    make, _ = mgr
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    m.start("cam")
    first = m.runs["cam"].proc
    m.runs.clear()
    m.start("cam")                                                 # the pid file now names only the second
    second = m.runs["cam"].proc
    m.runs.clear()
    m2 = make()
    assert "cam" not in m2.runs
    assert wait(lambda: first.poll() is not None and second.poll() is not None)
    assert wait(lambda: (m2.tick(), "cam" in m2.runs)[1])
    fresh = m2.runs["cam"].proc
    assert fresh.pid not in (first.pid, second.pid) and fresh.poll() is None
    m2.stop("cam")
    assert wait(lambda: (m2.tick(), m2.state("cam"))[1] == "stopped")


def test_a_log_cut_to_its_start_and_end(tmp_path):
    p = tmp_path / "d405_x.log"
    p.write_bytes(b"started ok\n" + b"ERROR No such device\n" * 5000 + b"last line\n")
    assert SV.trim_log(p, max_bytes=10 ** 6) == 0                     # small: untouched
    with open(p, "ab") as f:                                        # the driver keeps appending (O_APPEND)
        cut = SV.trim_log(p, max_bytes=20000, head=100, tail=300)
        assert cut > 90000
        f.write(b"after the cut\n")
    t = p.read_bytes()
    assert t.startswith(b"started ok\n") and b"MB cut from this log" in t and t.endswith(b"last line\nafter the cut\n")
    assert len(t) < 1000


def test_a_flooding_driver_log_is_cut_and_shown(mgr, monkeypatch):
    make, pubs = mgr
    monkeypatch.setattr(SV, "LOG_MAX_BYTES", 50000)
    monkeypatch.setattr(SV.trim_log, "__defaults__", (50000, 1000, 2000))
    m = make()
    m.save({"name": "cam", "type": "usb_cam"})
    old = m.log_dir / "cam_20200101_000000.log"                      # flooded before this check existed
    old.parent.mkdir(parents=True, exist_ok=True)
    old.write_bytes(b"x" * 200000)
    m.start("cam")
    assert old.stat().st_size < 5000
    assert wait(lambda: (m.tick(), m.state("cam"))[1] == "running")
    with open(m.runs["cam"].log, "ab") as f:
        f.write(b"ERROR No such device\n" * 10000)
    assert wait(lambda: (m.tick(), m.runs["cam"].log_cut)[1] > 0, 5)
    assert m.runs["cam"].log.stat().st_size < 10000
    assert "its log floods" in m.view()["services"][0]["msg"]
