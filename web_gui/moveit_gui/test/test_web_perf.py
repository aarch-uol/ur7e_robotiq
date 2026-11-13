"""web/perf.py: the GUI's own profiler and late-update report."""
import threading
import time

from moveit_gui.web import perf


def test_sample_names_the_busy_thread_and_its_place():
    stop = []

    def busy_loop():
        while not stop:
            sum(i * i for i in range(2000))
    t = threading.Thread(target=busy_loop, name="burner", daemon=True)
    t.start()
    try:
        lines = perf.sample(0.5, hz=200)
    finally:
        stop.append(1)
        t.join()
    text = "\n".join(lines)
    assert "thread burner" in text and "busy_loop" in text


def test_late_updates_are_logged_on_time_ones_are_not(monkeypatch):
    logs = []
    monkeypatch.setattr(perf, "LOG_EVERY_S", 0.2)
    s = perf.TickStats(0.1, logs.append)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:                   # 1 update in 0.3 s: late
        s.add({"tick": 0.25})
        time.sleep(0.25)
    assert logs and "late" in logs[0] and "tick 250" in logs[0]


def test_have_package_is_cached_but_follows_a_new_prefix_path(tmp_path, monkeypatch):
    from moveit_gui import services as SV
    idx = tmp_path / "a" / "share" / "ament_index" / "resource_index" / "packages"
    idx.mkdir(parents=True)
    monkeypatch.setattr(SV, "_have_cache", {})
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path / "a"))
    assert not SV.have_package("pkg_x")
    (idx / "pkg_x").write_text("")
    assert not SV.have_package("pkg_x")                  # cached: no file checks on every state update
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"{tmp_path / 'a'}:{tmp_path / 'b'}")
    assert SV.have_package("pkg_x")                      # another prefix path: looked up again
