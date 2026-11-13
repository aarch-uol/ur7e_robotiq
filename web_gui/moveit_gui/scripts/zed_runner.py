#!/usr/bin/env python3
"""Starts and stops the ZED camera driver for the GUI, inside the ZED container (docker/zed, ur7e_gui.sh).

The GUI has no Docker access, so it cannot start a container. Instead the launcher starts this container next to
the robot stack, and the GUI asks this runner through files in a folder both can see (~/ur7e_data/run/zed):

    req/<name>.json    the GUI: {"id", "argv", "log"}: start this (a new id: a new start)
    sig/<name>         the GUI: "<id> INT|TERM|KILL": signal that run
    state/<name>.json  this runner: {"id", "pid", "rc"} (rc null while it runs)
    alive              this runner: touched every second

Only the ZED driver's own launch file runs, with key:=value arguments made of plain characters (no spaces,
quotes or $): the folder is writable by the GUI, and this container can reach the cameras. Stdlib only.
"""
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

PREFIX = ["ros2", "launch", "zed_wrapper", "zed_camera.launch.py"]
ARG = re.compile(r"^[a-z_]{1,40}:=[A-Za-z0-9_.\-/:=;]{0,400}$")   # param_overrides:=a.b:=x;c.d:=y
NAME = re.compile(r"^[a-z][a-z0-9_]{0,39}$")
LOGS = "/data/"                                   # the ~/ur7e_data volume
SIGS = {"INT": signal.SIGINT, "TERM": signal.SIGTERM, "KILL": signal.SIGKILL}


def ok(req: dict) -> str:
    """'' when the request may run, else why not."""
    argv, log = req.get("argv"), str(req.get("log", ""))
    if not isinstance(argv, list) or argv[:4] != PREFIX:
        return "only the ZED driver (zed_wrapper zed_camera.launch.py) runs here"
    bad = [a for a in argv[4:] if not isinstance(a, str) or not ARG.match(a)]
    if bad:
        return f"argument not allowed: {bad[0]}"
    if not log.startswith(LOGS) or ".." in log:
        return f"the log goes under {LOGS}"
    return ""


def write(p: Path, d: dict):
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(d))
    tmp.replace(p)


class Runner:
    def __init__(self, root: Path):
        self.root = root
        for d in ("req", "sig", "state"):
            (root / d).mkdir(parents=True, exist_ok=True)
        self.runs = {}                                   # name -> (id, Popen)
        for p in (root / "state").glob("*.json"):        # an earlier container's runs are gone with it
            try:
                d = json.loads(p.read_text())
            except (OSError, ValueError):
                continue
            if d.get("rc") is None:
                write(p, {**d, "rc": -1, "msg": "the ZED container restarted"})

    def say(self, m):
        print(time.strftime("%H:%M:%S"), m, flush=True)

    def step(self):
        tmp = self.root / "alive.tmp"                    # replaced whole: the GUI never reads it half written
        tmp.write_text(str(time.time()))
        tmp.replace(self.root / "alive")
        for p in sorted((self.root / "req").glob("*.json")):
            name = p.stem
            if not NAME.match(name):
                continue
            try:
                req = json.loads(p.read_text())
            except (OSError, ValueError):
                continue
            rid = str(req.get("id", ""))
            cur = self.runs.get(name)
            if not rid or (cur and cur[0] == rid) or self._done(name, rid):
                continue
            if cur:                                      # a new start while the old one runs: keep the old one
                continue
            why = ok(req)
            if why:
                self.say(f"{name}: refused: {why}")
                write(self.root / "state" / f"{name}.json", {"id": rid, "pid": None, "rc": -1, "msg": why})
                continue
            Path(req["log"]).parent.mkdir(parents=True, exist_ok=True)
            with open(req["log"], "ab") as f:
                proc = subprocess.Popen(req["argv"], stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                        start_new_session=True,
                                        env={**os.environ, "PYTHONUNBUFFERED": "1", "RCUTILS_COLORIZED_OUTPUT": "0"})
            self.runs[name] = (rid, proc)
            write(self.root / "state" / f"{name}.json", {"id": rid, "pid": proc.pid, "rc": None})
            self.say(f"{name}: started pid {proc.pid}: {' '.join(req['argv'])}")
        for p in (self.root / "sig").glob("*"):
            name = p.name
            try:
                rid, _, sig = p.read_text().strip().partition(" ")
                p.unlink()
            except OSError:
                continue
            cur = self.runs.get(name)
            if cur and cur[0] == rid and sig in SIGS:
                self._kill(cur[1], SIGS[sig])
                self.say(f"{name}: {sig}")
        for name, (rid, proc) in list(self.runs.items()):
            rc = proc.poll()
            if rc is not None:
                del self.runs[name]
                write(self.root / "state" / f"{name}.json", {"id": rid, "pid": proc.pid, "rc": rc})
                self.say(f"{name}: ended ({rc})")

    def _done(self, name, rid) -> bool:
        try:
            d = json.loads((self.root / "state" / f"{name}.json").read_text())
        except (OSError, ValueError):
            return False
        return d.get("id") == rid and d.get("rc") is not None

    @staticmethod
    def _kill(proc, sig):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass

    def stop_all(self):
        for _, proc in self.runs.values():
            self._kill(proc, signal.SIGINT)
        end = time.time() + 10
        while time.time() < end and any(p.poll() is None for _, p in self.runs.values()):
            time.sleep(0.2)
        for _, proc in self.runs.values():
            if proc.poll() is None:
                self._kill(proc, signal.SIGKILL)
        self.step()


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "/data/run/zed")
    r = Runner(root)
    stop = []
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.append(1))
    r.say(f"ZED runner: waiting for the GUI in {root}")
    while not stop:
        r.step()
        time.sleep(0.3)
    r.say("stopping the ZED driver")
    r.stop_all()


if __name__ == "__main__":
    main()
