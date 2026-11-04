"""Policy subprocess lifecycle: dies with the GUI, non-blocking stop, failed starts always end as a failure."""
import os
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

from moveit_gui.policies import PolicyLibrary, PolicyManifest
from moveit_gui.policy_action import RunPolicyGoal, SubprocessPolicyRunner


def wait(cond, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def make(tmp_path, script, name="p", **kw):
    f = tmp_path / f"{name}.py"
    f.write_text(script)
    lib = PolicyLibrary(tmp_path / "none")
    lib.manifests[name] = PolicyManifest(name=name, command=[sys.executable, str(f)], required_mode="x", **kw)
    modes = SimpleNamespace(profile=SimpleNamespace(modes={"x": 1}), current_mode=lambda: "x",
                            bridge=SimpleNamespace(snapshot=lambda: SimpleNamespace(connected={})))
    return lib, modes


def start_run(lib, modes, name="p", **kw):
    out, ev = [], threading.Event()
    t = threading.Thread(target=lambda: (out.append(lib.run(name, modes, stop_event=ev, log=lambda m: None, **kw))),
                         daemon=True)
    t.start()
    return t, out, ev


def test_child_has_pdeathsig_and_its_own_process_group(tmp_path):
    res = tmp_path / "res.txt"
    lib, modes = make(tmp_path, f"""
import ctypes, os
v = ctypes.c_int(0)
ctypes.CDLL("libc.so.6").prctl(2, ctypes.byref(v))      # PR_GET_PDEATHSIG
open({str(res)!r}, "w").write(f"{{v.value}} {{os.getpgrp() == os.getpid()}}")
""")
    assert lib.run("p", modes, log=lambda m: None)
    assert res.read_text() == f"{int(signal.SIGTERM)} True"


def test_stop_does_not_block_and_run_returns(tmp_path):
    lib, modes = make(tmp_path, "import time\nwhile True: time.sleep(0.1)\n")
    t, out, _ = start_run(lib, modes)
    assert wait(lambda: lib.running)
    t0 = time.monotonic()
    lib.stop()
    assert time.monotonic() - t0 < 0.5
    t.join(5)
    assert out == [False]


def test_stubborn_child_is_escalated_in_the_background(tmp_path):
    marker = tmp_path / "up"
    lib, modes = make(tmp_path, f"""
import signal, time
signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN)
open({str(marker)!r}, "w").write("1")
while True: time.sleep(0.1)
""")
    t, out, _ = start_run(lib, modes)
    assert wait(marker.exists)
    proc = lib._proc
    t0 = time.monotonic()
    lib._terminate(lib.manifests["p"], grace_s=0.3)               # SIGINT now; SIGTERM, SIGKILL later
    assert time.monotonic() - t0 < 0.2 and proc.poll() is None
    assert wait(lambda: proc.poll() is not None, 5)
    assert proc.returncode == -signal.SIGKILL
    t.join(5)
    assert out == [False]


def test_failed_start_is_a_failure_not_an_exception(tmp_path):
    lib = PolicyLibrary(tmp_path / "none")
    lib.manifests["p"] = PolicyManifest(name="p", command=["/nonexistent/runner"], required_mode="x")
    modes = SimpleNamespace(profile=SimpleNamespace(modes={"x": 1}), current_mode=lambda: "x",
                            bridge=SimpleNamespace(snapshot=lambda: SimpleNamespace(connected={})))
    msgs = []
    assert lib.run("p", modes, log=msgs.append) is False
    assert any("failed to start" in m for m in msgs) and not lib.running


def test_runner_reports_done_with_failure_when_the_child_exits_at_once_or_cannot_start(tmp_path):
    for script in ("import sys; sys.exit(3)", None):
        lib, modes = make(tmp_path, script or "")
        if script is None:
            lib.manifests["p"].command = ["/nonexistent/runner"]
        runner = SubprocessPolicyRunner(lib, modes)
        done = []
        runner.run(RunPolicyGoal(policy="p", backend="", duration=None, robot_config=""), 5.0,
                   lambda fb: None, done.append)
        assert wait(lambda: done)
        assert done[0].success is False


def test_runner_cancel_returns_at_once(tmp_path):
    lib, modes = make(tmp_path, "import time\nwhile True: time.sleep(0.1)\n")
    runner = SubprocessPolicyRunner(lib, modes)
    done = []
    runner.run(RunPolicyGoal(policy="p", backend="", duration=None, robot_config=""), 5.0, lambda fb: None,
               done.append)
    assert wait(lambda: lib.running)
    t0 = time.monotonic()
    runner.cancel()
    assert time.monotonic() - t0 < 0.5
    assert wait(lambda: done) and done[0].success is False
