"""The robot this PC runs: which arm, real or fake, its IP, and what is on it (SETUP > Setups > Set up…).

Like a MoveIt Pro robot config, one place says what the hardware is; here it is a small catalog (ROBOTS, GRIPPERS,
FT_SENSORS) instead of a YAML package per robot. The arm, mode and IPs are kept in config_dir()/robot as plain
`key=value` lines, so the launcher (ur7e_gui.sh, bash) can read it:

    robot=ur7e
    mode=real
    ip=10.6.203.101
    ft_ip=192.168.1.1
    domain=42
    net=lab
    servo=on

`domain` is the ROS_DOMAIN_ID of the stack, the GUI and the cameras: other ROS 2 programs need the same one.
`net` is who can see their topics: `lab` (any PC on the network) or `local` (this PC only).
`servo` starts MoveIt Servo with the UR stack (`on`, the default, or `off`): the hold-to-jog buttons need it.

The gripper and F/T sensor become the end-effector base (bases.py, config_dir()/base). The launcher reads both at
every robot stack start and tells the GUI what it started (UR7E_ROBOT, UR7E_ROBOT_MODE, UR7E_ROBOT_IP, UR7E_FT_IP).
The GUI only writes the files and asks for a restart: it never starts a driver.
Command-line options (--fake, --robot-ip) win for that start only.
"""
from __future__ import annotations

import ipaddress
import os
from pathlib import Path
from typing import Dict, List, Optional

# The robots the GUI knows: each one's GUI profile (profiles/<profile>.yaml), and what can be on it. Whether this PC
# can start one (its image and workspace are there) is the launcher's to say: UR7E_ROBOT_STATUS, see status().
ROBOTS = {
    "ur7e": {"label": "UR7e", "text": "Universal Robots UR7e (ur7e_robotiq stack)", "profile": "ur7e_lab",
             "grippers": ["robotiq_2f85", "none"], "ft_sensors": ["ati", "none"]},
    "fr3": {"label": "Franka FR3", "text": "Franka Research 3 (fr3_scraping_ws stack)", "profile": "fr3",
            "grippers": ["franka_hand", "none"], "ft_sensors": ["ati", "none"]},
}
GRIPPERS = {"robotiq_2f85": ("Robotiq 2F-85", "on the flange or on the F/T sensor; USB through the tool connector"),
            "franka_hand": ("Franka Hand", "Franka's own two-finger gripper"),
            "none": ("None", "")}
FT_SENSORS = {"ati": ("ATI F/T", "on the flange, read over the network (Net F/T box)"),
              "none": ("None", "")}
NETS = {
    "lab": ("Lab network", "ROS 2 programs on other PCs on the network see the robot's topics"),
    "local": ("This PC only", "only programs on this PC see them (ROS 2 discovery on localhost)"),
}
MODES = {
    "real": ("Real", "the robot at this IP"),
    "fake": ("Fake", "no robot: a simulated arm, nothing is sent anywhere"),
}
DEFAULT = {"robot": "ur7e", "mode": "real", "ip": "10.6.203.101", "ft_ip": "192.168.1.1", "domain": "42", "net": "lab", "servo": "on"}
KEYS = ("robot", "mode", "ip", "ft_ip", "domain", "net", "servo")
SERVO = ("on", "off")
DOMAIN_MAX = 101                                  # 0-101: the IDs every ROS 2 PC can use (ROS 2 docs, "Domain ID")
FILE = "robot"


class RobotError(ValueError):
    pass


def status() -> Dict[str, str]:
    """What the launcher found on this PC per robot: UR7E_ROBOT_STATUS="ur7e=ok|fr3=<what is missing>"."""
    out = {}
    for item in os.environ.get("UR7E_ROBOT_STATUS", "").split("|"):
        k, sep, v = item.partition("=")
        if sep and k.strip() in ROBOTS:
            out[k.strip()] = v.strip()
    return out


def missing(robot: str) -> List[str]:
    """What this PC lacks to start `robot` (empty: ready, or the launcher did not say)."""
    v = status().get(robot, "ok")
    return [] if v == "ok" else [v]


def _ip(a: str, what: str) -> str:
    try:
        addr = ipaddress.IPv4Address(a)
    except ValueError:
        raise RobotError(f"{what}: '{a}' is not an IP address like 10.6.203.101") from None
    if addr.is_unspecified or addr.is_multicast:
        raise RobotError(f"{what}: {a} cannot be a device's address")
    return a


def _domain(d) -> str:
    t = str(DEFAULT["domain"] if d is None or str(d).strip() == "" else d).strip()
    if not t.isdigit() or int(t) > DOMAIN_MAX:
        raise RobotError(f"the ROS domain is a number from 0 to {DOMAIN_MAX}, not '{t}'")
    return str(int(t))


def check(robot, mode, ip, ft_ip=None, domain=None, net=None, servo=None) -> Dict[str, str]:
    r, m = str(robot or "").strip(), str(mode or "").strip()
    n = str(net or DEFAULT["net"]).strip()
    if n not in NETS:
        raise RobotError(f"the network is one of {', '.join(NETS)}, not '{n}'")
    if isinstance(servo, bool):
        servo = "on" if servo else "off"
    sv = str(servo or DEFAULT["servo"]).strip().lower()
    if sv not in SERVO:
        raise RobotError(f"servo is on or off, not '{sv}'")
    if r not in ROBOTS:
        raise RobotError(f"the robot is one of {', '.join(ROBOTS)}, not '{r}'")
    if m not in MODES:
        raise RobotError(f"the mode is one of {', '.join(MODES)}, not '{m}'")
    return {"robot": r, "mode": m, "ip": _ip(str(ip or "").strip(), "robot IP"),
            "ft_ip": _ip(str(ft_ip or DEFAULT["ft_ip"]).strip(), "F/T sensor IP"), "domain": _domain(domain), "net": n,
            "servo": sv}


def base_of(robot: str, gripper: str, ft: str) -> str:
    """The end-effector base (bases.py) for this gripper + F/T sensor on this robot."""
    r = ROBOTS[robot]
    if gripper not in r["grippers"]:
        raise RobotError(f"{r['label']} takes the gripper {' or '.join(GRIPPERS[g][0] for g in r['grippers'])}")
    if ft not in r["ft_sensors"]:
        raise RobotError(f"{r['label']} takes the F/T sensor {' or '.join(FT_SENSORS[f][0] for f in r['ft_sensors'])}")
    return {(True, True): "ft_gripper", (False, True): "gripper", (True, False): "ft",
            (False, False): "none"}[(ft != "none", gripper != "none")]


def parts_of(robot: str, base: str) -> Dict[str, str]:
    """The gripper and F/T sensor of a base on this robot (the inverse of base_of)."""
    r = ROBOTS.get(robot, ROBOTS[DEFAULT["robot"]])
    return {"gripper": r["grippers"][0] if base in ("gripper", "ft_gripper") else "none",
            "ft": r["ft_sensors"][0] if base in ("ft", "ft_gripper") else "none"}


def read_next(cfg: Path) -> Dict[str, str]:
    """What the launcher starts next time: the file, with defaults for what it lacks or gets wrong."""
    out = dict(DEFAULT)
    try:
        text = (Path(cfg) / FILE).read_text()
    except OSError:
        return out
    got = {}
    for line in text.splitlines():
        k, sep, v = line.partition("=")
        if sep and k.strip() in DEFAULT:
            got[k.strip()] = v.strip()
    try:
        return check(*(got.get(k, out[k]) for k in KEYS))
    except RobotError:
        return out


def write_next(cfg: Path, robot, mode, ip, ft_ip=None, domain=None, net=None, servo=None) -> Dict[str, str]:
    v = check(robot, mode, ip, ft_ip, domain, net, servo)
    p = Path(cfg) / FILE
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text("".join(f"{k}={v[k]}\n" for k in KEYS))
    tmp.replace(p)
    return v


def running(fake: bool) -> Optional[Dict[str, str]]:
    """What the launcher started (None: this GUI was not started by ur7e_gui.sh)."""
    mode = os.environ.get("UR7E_ROBOT_MODE", "").strip()
    if mode not in MODES:
        return None
    return {"robot": os.environ.get("UR7E_ROBOT", "").strip() or DEFAULT["robot"],
            "mode": "fake" if fake else mode, "ip": os.environ.get("UR7E_ROBOT_IP", "").strip(),
            "ft_ip": os.environ.get("UR7E_FT_IP", "").strip() or DEFAULT["ft_ip"],
            "domain": os.environ.get("ROS_DOMAIN_ID", "").strip() or "0",
            "net": "local" if os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "").strip().upper() == "LOCALHOST"
            else "lab",
            # an older launcher never started Servo and does not say
            "servo": "on" if os.environ.get("UR7E_SERVO", "").strip() == "on" else "off"}


def chosen(cfg: Path) -> bool:
    """Has anyone picked a robot on this PC yet (the file exists)?"""
    return (Path(cfg) / FILE).exists()


def view(cfg: Path, fake: bool) -> dict:
    return {"running": running(fake), "next": read_next(cfg), "chosen": chosen(cfg),
            "robots": [{"name": k, "label": v["label"], "text": v["text"], "missing": missing(k),
                        "grippers": v["grippers"], "ft_sensors": v["ft_sensors"]} for k, v in ROBOTS.items()],
            "grippers": [{"name": k, "label": v[0], "text": v[1]} for k, v in GRIPPERS.items()],
            "ft_sensors": [{"name": k, "label": v[0], "text": v[1]} for k, v in FT_SENSORS.items()],
            "modes": [{"name": k, "label": v[0], "text": v[1]} for k, v in MODES.items()],
            "nets": [{"name": k, "label": v[0], "text": v[1]} for k, v in NETS.items()], "domain_max": DOMAIN_MAX}
