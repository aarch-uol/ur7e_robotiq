import threading
import time

from moveit_gui.cpu_sample import sample


def test_busy_thread_is_found():
    stop = threading.Event()

    def spin_here():
        while not stop.is_set():
            sum(range(1000))
    t = threading.Thread(target=spin_here, name="busy-test", daemon=True)
    t.start()
    try:
        out = sample(0.6)
    finally:
        stop.set()
        t.join()
    busy = next(d for d in out if d["thread"] == "busy-test")
    assert busy["cpu_pct"] > 20
    assert any("spin_here" in s["at"] for s in busy["stacks"])
    assert out[0]["cpu_pct"] >= out[-1]["cpu_pct"]
