#!/usr/bin/env python3
"""PC setup for the MoveIt GUI robot stack: desktop PCs (amd64) and NVIDIA Jetsons (arm64).

    scripts/pc_setup.py doctor                 # checks this PC, changes nothing
    scripts/pc_setup.py doctor --robot-ip 10.6.203.101 --json

doctor only reads: no sudo, no installs, nothing sent to the robot except one ping. Each line is
    ok    fine
    warn  works, but worth a look
    FAIL  the GUI or a driver will not work until this is fixed
with a short hint on how to fix it. Python standard library only, so it runs before anything is installed.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

HOME = Path.home()
ROBOT_IP = "10.6.203.101"
IMAGE_REPO = "ur7e_robotiq"
# JetPack 6 releases by L4T version (the jetpack apt package is often not installed, L4T always is)
JETPACK = {"36.2": "6.0 DP", "36.3": "6.0", "36.4.0": "6.1", "36.4.3": "6.2", "36.4.4": "6.2.1", "36.4.7": "6.2.1"}
USB_IDS = {"8086": "Intel RealSense", "2b03": "Stereolabs ZED (USB)", "0403:6015": "Robotiq gripper (USB serial)"}


def run(cmd, timeout=5.0) -> tuple:
    """(exit code, stdout); 127 when the program is missing."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.strip()
    except FileNotFoundError:
        return 127, ""
    except subprocess.TimeoutExpired:
        return 124, ""


def read(path, default="") -> str:
    try:
        return Path(path).read_text(errors="replace").replace("\0", "").strip()
    except OSError:
        return default


# ------------------------------------------------------------------ what this machine is
def detect() -> dict:
    m = {"arch": platform.machine(), "kernel": platform.release()}
    osr = dict(re.findall(r'^(\w+)="?([^"\n]*)"?', read("/etc/os-release"), re.M))
    m["os"] = osr.get("PRETTY_NAME", "?")
    m["os_version"] = osr.get("VERSION_ID", "")
    m["model"] = read("/proc/device-tree/model") or read("/sys/class/dmi/id/product_name") or "?"
    tegra = read("/etc/nv_tegra_release")
    if tegra:
        r = re.search(r"R(\d+).*REVISION:\s*([\d.]+)", tegra)
        l4t = f"{r.group(1)}.{r.group(2)}" if r else "?"
        m.update(platform_kind="jetson", l4t=l4t,
                 jetpack=JETPACK.get(l4t) or JETPACK.get(".".join(l4t.split(".")[:2]), "?"), gpu="jetson")
    else:
        m["platform_kind"] = "desktop"
        code, out = run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
        m["gpu"] = out.splitlines()[0] if code == 0 and out else "none"
    m["realtime"] = read("/sys/kernel/realtime") == "1" or "PREEMPT_RT" in read("/proc/version") \
        or "-rt" in m["kernel"]
    m["mem_gb"] = round(int(re.search(r"MemTotal:\s+(\d+)", read("/proc/meminfo", "MemTotal: 0")).group(1)) / 2**20, 1)
    m["image"] = image_tag(m)
    return m


def image_tag(m: dict) -> str:
    """The robot image this machine runs. amd64 keeps the old tag, so existing PCs are unchanged."""
    return f"{IMAGE_REPO}:jazzy" if m["arch"] in ("x86_64", "amd64") else f"{IMAGE_REPO}:jazzy-arm64"


# ------------------------------------------------------------------ checks
class Report:
    def __init__(self):
        self.rows = []

    def add(self, group, level, what, hint=""):
        self.rows.append({"group": group, "level": level, "what": what, "hint": hint})

    def worst(self) -> str:
        levels = {r["level"] for r in self.rows}
        return "FAIL" if "FAIL" in levels else "warn" if "warn" in levels else "ok"


def check_machine(r: Report, m: dict):
    g = "machine"
    desc = f"{m['model']}, {m['arch']}, {m['os']}"
    if m["platform_kind"] == "jetson":
        desc += f", JetPack {m['jetpack']} (L4T {m['l4t']})"
    r.add(g, "ok", desc)
    if m["arch"] not in ("x86_64", "amd64", "aarch64", "arm64"):
        r.add(g, "FAIL", f"{m['arch']} is not supported", "use an amd64 PC or an arm64 Jetson")
    if m["platform_kind"] == "jetson" and not m["l4t"].startswith("36"):
        r.add(g, "warn", f"L4T {m['l4t']}: only JetPack 6 (L4T 36) is tested", "flash JetPack 6.x")
    r.add(g, "ok", f"{m['mem_gb']} GB RAM, GPU: {m['gpu']}")
    if m["realtime"]:
        r.add(g, "ok", f"real-time kernel ({m['kernel']})")
    else:
        r.add(g, "warn", f"no real-time kernel ({m['kernel']})",
              "the UR driver runs at 500 Hz; it works without RT, with more jitter (lowlatency kernel helps)")
    if m["platform_kind"] == "jetson":
        code, out = run(["nvpmodel", "-q"])
        mode = out.splitlines()[0].replace("NV Power Mode:", "").strip() if code == 0 and out else "?"
        r.add(g, "ok" if "MAXN" in mode else "warn", f"power mode {mode}",
              "" if "MAXN" in mode else "sudo nvpmodel -m 0 (MAXN) for full speed")
    else:
        gov = read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "?")
        r.add(g, "ok" if gov in ("performance", "?") else "warn", f"CPU governor {gov}",
              "" if gov in ("performance", "?") else "'performance' gives steadier timing for the robot driver")


def check_docker(r: Report, m: dict) -> bool:
    g = "docker"
    code, out = run(["docker", "--version"])
    if code == 127:
        r.add(g, "FAIL", "Docker is not installed", "https://docs.docker.com/engine/install/ubuntu/")
        return False
    r.add(g, "ok", out)
    code, out = run(["docker", "info", "--format", "{{json .}}"], timeout=10)
    if code != 0:
        groups = run(["id", "-nG"])[1].split()
        r.add(g, "FAIL", "cannot talk to the Docker daemon",
              "sudo usermod -aG docker $USER, then log in again" if "docker" not in groups
              else "is the daemon running? sudo systemctl start docker")
        return False
    info = json.loads(out)
    runtimes = sorted((info.get("Runtimes") or {}).keys())
    if m["gpu"] != "none":
        if "nvidia" in runtimes:
            r.add(g, "ok", "NVIDIA container runtime: containers can use the GPU")
        else:
            r.add(g, "warn", "no NVIDIA container runtime: containers cannot use the GPU (torch policies)",
                  "install nvidia-container-toolkit (pc_setup install will do it)")
    root = info.get("DockerRootDir", "/var/lib/docker")
    try:
        du = shutil.disk_usage(root if os.path.exists(root) else "/")
        free = du.free / 2**30
        r.add(g, "ok" if free > 30 else "warn", f"{free:.0f} GB free for images ({root})",
              "" if free > 30 else "the robot image needs ~15 GB; a ZED/CUDA image more")
    except OSError:
        pass
    code, out = run(["docker", "image", "inspect", m["image"], "--format", "{{.Architecture}} {{.Created}}"])
    if code == 0:
        arch, created = out.split(" ", 1)
        want = "arm64" if m["arch"] in ("aarch64", "arm64") else "amd64"
        r.add(g, "ok" if arch == want else "FAIL", f"image {m['image']} ({arch}, built {created[:10]})",
              "" if arch == want else f"the image is {arch}, this PC is {want}: rebuild it here")
    else:
        r.add(g, "warn", f"image {m['image']} not built yet", "the launcher builds it on its first start (slow)")
    return True


def check_network(r: Report, robot_ip: str):
    g = "network"
    code, out = run(["ip", "-o", "route", "get", robot_ip])
    src = re.search(r"\bsrc (\S+)", out)
    dev = re.search(r"\bdev (\S+)", out)
    via = re.search(r"\bvia (\S+)", out)
    if code != 0 or not src:
        r.add(g, "FAIL", f"no route to the robot {robot_ip}", "connect this PC to the robot network")
    else:
        r.add(g, "warn" if via else "ok", f"robot {robot_ip} through {dev.group(1) if dev else '?'}, "
              f"this PC is {src.group(1)}" + (f" (via router {via.group(1)})" if via else ""),
              "the robot should be on the same network as this PC (External Control connects back to it)"
              if via else "")
        code, _ = run(["ping", "-c", "1", "-W", "1", robot_ip], timeout=3)
        r.add(g, "ok" if code == 0 else "warn", f"robot {robot_ip} " + ("answers ping" if code == 0 else "does not answer ping"),
              "" if code == 0 else "robot off, wrong IP (--robot-ip), or ping blocked")
    code, out = run(["tailscale", "ip", "-4"])
    if code == 127:
        r.add(g, "warn", "Tailscale not installed: only this PC's browser can open the GUI",
              "optional: https://tailscale.com/download, then tailscale up")
    elif code == 0 and out:
        r.add(g, "ok", f"Tailscale {out.splitlines()[0]}: the GUI is reachable on your tailnet")
    else:
        r.add(g, "warn", "Tailscale installed but not up", "tailscale up")


def check_code(r: Report):
    g = "code"
    for path, what, hint in (
            (HOME / "ur7e_robotiq", "robot stack", "git clone git@github.com:aarch-uol/ur7e_robotiq.git ~/ur7e_robotiq"),
            (HOME / "ws/src/moveit_gui", "GUI", "git clone -b v2-ur-jazzy https://github.com/mrceki/MoveIt-GUI.git "
             "~/ws/src/moveit_gui"),
            (HOME / "ws/src/pymoveit2", "pymoveit2", "git clone https://github.com/AndrejOrsula/pymoveit2.git "
             "~/ws/src/pymoveit2"),
            (HOME / "ur7e_gui.sh", "launcher", "cp ~/ws/src/moveit_gui/scripts/ur7e_gui.sh ~/")):
        r.add(g, "ok" if path.exists() else "FAIL", f"{what}: {path}" if path.exists() else f"{what} missing",
              "" if path.exists() else hint)
    data = HOME / "ur7e_data"
    r.add(g, "ok" if data.is_dir() else "warn", f"data folder {data}" + ("" if data.is_dir() else " not made yet"),
          "" if data.is_dir() else "the launcher makes it on the first start")


def check_devices(r: Report, m: dict):
    g = "devices"
    code, out = run(["lsusb"])
    found = []
    for line in out.splitlines() if code == 0 else []:
        ids = re.search(r"ID ([0-9a-f]{4}):([0-9a-f]{4})", line)
        if not ids:
            continue
        for key, name in USB_IDS.items():
            if key == ids.group(1) or key == f"{ids.group(1)}:{ids.group(2)}":
                found.append(name)
    for name in sorted(set(found)):
        r.add(g, "ok", f"{name} plugged in")
    zed = Path("/usr/local/zed/include/sl/Camera.hpp")
    if zed.exists():
        v = dict(re.findall(r"#define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION (\d+)", read(zed)))
        r.add(g, "ok", f"ZED SDK {v.get('MAJOR', '?')}.{v.get('MINOR', '?')}.{v.get('PATCH', '?')} on the host "
              "(the ZED container uses it)")
        if m["platform_kind"] == "jetson":
            code, out = run(["systemctl", "is-active", "zed_x_daemon"])
            if out:
                r.add(g, "ok" if out == "active" else "warn", f"ZED X daemon {out}",
                      "" if out == "active" else "sudo systemctl start zed_x_daemon (ZED X / GMSL cameras)")
            img = os.environ.get("UR7E_ZED_IMAGE", "moveit_gui_zed:humble")
            code, _ = run(["docker", "image", "inspect", img])
            r.add(g, "ok" if code == 0 else "warn", f"ZED driver image {img}" + ("" if code == 0 else " missing"),
                  "" if code == 0 else "~/ws/src/moveit_gui/docker/zed/build.sh (once; the GUI starts the ZED with it)")
    vids = sorted(p.name for p in Path("/dev").glob("video*"))
    if vids:
        r.add(g, "ok", f"{len(vids)} video devices (/dev/{vids[0]}…)")
    if not found and not vids and not zed.exists():
        r.add(g, "ok", "no cameras or USB devices found (fine for the arm alone)")


def doctor(robot_ip: str) -> tuple:
    m = detect()
    r = Report()
    check_machine(r, m)
    check_docker(r, m)
    check_network(r, robot_ip)
    check_code(r)
    check_devices(r, m)
    return m, r


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("doctor", help="check this PC (changes nothing)")
    d.add_argument("--robot-ip", default=ROBOT_IP)
    d.add_argument("--json", action="store_true", help="machine-readable output")
    a = ap.parse_args(argv)
    m, r = doctor(a.robot_ip)
    if a.json:
        print(json.dumps({"machine": m, "checks": r.rows, "worst": r.worst()}, indent=1))
    else:
        color = sys.stdout.isatty()
        paint = {"ok": "32", "warn": "33", "FAIL": "31"}
        group = None
        for row in r.rows:
            if row["group"] != group:
                group = row["group"]
                print(f"\n{group.upper()}")
            tag = f"{row['level']:<4}"
            if color:
                tag = f"\033[{paint[row['level']]}m{tag}\033[0m"
            print(f"  {tag}  {row['what']}")
            if row["hint"]:
                print(f"        → {row['hint']}")
        print(f"\nImage for this PC: {m['image']}.  Result: {r.worst()}")
    return 1 if r.worst() == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main())
