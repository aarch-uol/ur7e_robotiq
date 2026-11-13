"""Where the GUI process spends its CPU, per Python thread (DIAGNOSTICS, command cpu_profile).

For a few seconds: the CPU time of every thread from /proc/self/task/<tid>/stat, and samples of every thread's
Python stack (sys._current_frames) every few ms. A thread that only waits shows its wait; the CPU column says
which threads matter. Linux only; no tool to install and no access to other processes.
"""
from __future__ import annotations

import collections
import os
import sys
import threading
import time
from typing import Dict, List

TICK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


def _cpu_ticks(tid: int) -> int:
    try:
        with open(f"/proc/self/task/{tid}/stat") as f:
            s = f.read()
    except OSError:
        return 0
    rest = s[s.rindex(")") + 2:].split()
    return int(rest[11]) + int(rest[12])               # utime + stime


def _where(frame, depth: int) -> str:
    out = []
    while frame is not None and len(out) < depth:
        c = frame.f_code
        fn = c.co_filename
        short = fn[fn.rfind("site-packages/") + 14:] if "site-packages/" in fn else "/".join(fn.rsplit("/", 2)[-2:])
        out.append(f"{short}:{c.co_name}:{frame.f_lineno}")
        frame = frame.f_back
    return " < ".join(out)


def _tids() -> List[int]:
    try:
        return [int(t) for t in os.listdir("/proc/self/task")]
    except OSError:
        return []


def _comm(tid: int) -> str:
    try:
        with open(f"/proc/self/task/{tid}/comm") as f:
            return f.read().strip()
    except OSError:
        return "?"


def sample(seconds: float = 5.0, interval: float = 0.005, depth: int = 4, top: int = 6) -> List[dict]:
    """Threads sorted by CPU (% of one core over the window). Threads without Python (DDS, C++) are listed by
    their OS name, without stacks."""
    seconds = max(0.5, min(float(seconds), 20.0))
    me = threading.get_ident()
    names = {t.ident: (t.name, t.native_id) for t in threading.enumerate()}
    t0 = {tid: _cpu_ticks(tid) for tid in _tids()}
    hits: Dict[int, collections.Counter] = collections.defaultdict(collections.Counter)
    n = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for ident, frame in sys._current_frames().items():
            if ident != me:
                hits[ident][_where(frame, depth)] += 1
        n += 1
        time.sleep(interval)
    out = []
    for ident, (name, nid) in names.items():
        if ident == me or not nid:
            continue
        cpu = (_cpu_ticks(nid) - t0.get(nid, 0)) / TICK / seconds * 100
        c = hits.get(ident, collections.Counter())
        out.append({"thread": name, "tid": nid, "cpu_pct": round(cpu, 1),
                    "stacks": [{"pct": round(100 * k / max(n, 1)), "at": w} for w, k in c.most_common(top)]})
    py = {nid for _, nid in names.values()}
    for tid in _tids():
        if tid not in py and tid in t0:
            cpu = (_cpu_ticks(tid) - t0[tid]) / TICK / seconds * 100
            if cpu >= 0.5:
                out.append({"thread": _comm(tid) + " (no Python)", "tid": tid, "cpu_pct": round(cpu, 1), "stacks": []})
    out.sort(key=lambda d: -d["cpu_pct"])
    return out
