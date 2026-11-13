"""The robot choice (robots.py), the robot_setup command, and the launcher reading it (with a stub docker)."""
import os
import subprocess
import textwrap
from pathlib import Path

import pytest

from moveit_gui import robots as R
from moveit_gui.web.service import CommandError
from moveit_gui.web.workcell import RESTART_STACK
from test_workcell import env, run, wait  # noqa: F401  (the fixture)

ROOT = Path(__file__).parents[1]


def test_file_round_trip_and_defaults(tmp_path):
    assert R.read_next(tmp_path) == R.DEFAULT                      # no file: the lab robot, real
    R.write_next(tmp_path, "ur7e", "fake", "10.0.0.5")
    assert (tmp_path / "robot").read_text() == "robot=ur7e\nmode=fake\nip=10.0.0.5\nft_ip=192.168.1.1\ndomain=42\nnet=lab\nservo=on\n"
    assert R.read_next(tmp_path) == {"robot": "ur7e", "mode": "fake", "ip": "10.0.0.5", "ft_ip": "192.168.1.1",
                                     "domain": "42", "net": "lab", "servo": "on"}
    R.write_next(tmp_path, "ur7e", "real", "10.0.0.5", domain="07", net="local")
    assert R.read_next(tmp_path)["domain"] == "7" and R.read_next(tmp_path)["net"] == "local"
    (tmp_path / "robot").write_text("robot=ur7e\nmode=flying\n")    # a broken file falls back to the defaults
    assert R.read_next(tmp_path) == R.DEFAULT


def test_missing_is_what_the_launcher_found(monkeypatch):
    monkeypatch.delenv("UR7E_ROBOT_STATUS", raising=False)
    assert R.missing("fr3") == []                                                  # the launcher did not say
    monkeypatch.setenv("UR7E_ROBOT_STATUS", "ur7e=ok|fr3=the image franka_scraping:jazzy is not on this PC")
    assert R.missing("ur7e") == [] and R.missing("fr3") == ["the image franka_scraping:jazzy is not on this PC"]


def test_fr3_profile_and_parts():
    from moveit_gui.profile import RobotProfile
    p = RobotProfile.load(str(ROOT / "profiles" / "fr3.yaml"))
    assert len(p.joint_names) == 7 and p.moveit["group"] == "fr3_arm" and p.gripper["model"] == "Franka Hand"
    assert len(p.waypoints["ready"].joints) == 7
    assert R.base_of("fr3", "franka_hand", "none") == "gripper"
    assert R.parts_of("fr3", "gripper") == {"gripper": "franka_hand", "ft": "none"}
    with pytest.raises(R.RobotError, match="takes the gripper"):
        R.base_of("fr3", "robotiq_2f85", "none")


def test_a_hand_started_gui_never_asks(env, monkeypatch):
    s, _, _ = env
    monkeypatch.delenv("UR7E_ROBOT_MODE", raising=False)
    assert s.state()["workcell"]["robot_chosen"] is True


@pytest.mark.parametrize("args,msg", [(("ur99", "real", "10.0.0.1"), "robot is one of"),
                                      (("ur7e", "sim", "10.0.0.1"), "mode is one of"),
                                      (("ur7e", "real", "robot.lab"), "not an IP"),
                                      (("ur7e", "real", "10.0.0.1", "nope"), "F/T sensor IP"),
                                      (("ur7e", "real", "0.0.0.0"), "cannot be"),
                                      (("ur7e", "real", "10.0.0.1", None, "102"), "ROS domain"),
                                      (("ur7e", "real", "10.0.0.1", None, "-1"), "ROS domain"),
                                      (("ur7e", "real", "10.0.0.1", None, "42", "wifi"), "network is one of")])
def test_check_refuses(args, msg):
    with pytest.raises(R.RobotError, match=msg):
        R.check(*args)


UR = dict(robot="ur7e", ip="10.6.203.101", ft_ip="192.168.1.1")


def test_robot_setup_command(env, monkeypatch):
    s, _, tmp = env
    cfg = tmp / "cfg" / "moveit_gui"
    monkeypatch.delenv("UR7E_ROBOT_MODE", raising=False)
    monkeypatch.setenv("UR7E_EE_CONFIG", "ft_gripper")
    with pytest.raises(CommandError, match="launcher"):            # a GUI started by hand cannot set up
        run(s, "robot_setup", **UR, mode="real", gripper="robotiq_2f85", ft="ati", confirm=True)
    monkeypatch.setenv("UR7E_ROBOT", "ur7e")
    monkeypatch.setenv("UR7E_ROBOT_MODE", "fake")
    monkeypatch.setenv("UR7E_ROBOT_IP", "10.6.203.101")
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    monkeypatch.setenv("UR7E_SERVO", "on")
    monkeypatch.delenv("ROS_AUTOMATIC_DISCOVERY_RANGE", raising=False)
    monkeypatch.setenv("UR7E_ROBOT_STATUS", "ur7e=ok|fr3=the image franka_scraping:jazzy is not on this PC")
    v = run(s, "robot_view")["robot"]
    assert (v["gripper"], v["ft"], v["cameras"]) == ("robotiq_2f85", "ati", [])
    assert {t["type"] for t in v["camera_types"]} == {"realsense", "usb_cam", "zed"}
    assert not v["chosen"] and s.state()["workcell"]["robot_chosen"] is False     # the page asks once
    fr3 = next(r for r in v["robots"] if r["name"] == "fr3")
    assert fr3["missing"] == ["the image franka_scraping:jazzy is not on this PC"]  # listed, with what is missing
    assert fr3["grippers"] == ["franka_hand", "none"]
    with pytest.raises(CommandError, match="Franka FR3 is not set up on this PC"):
        run(s, "robot_setup", **{**UR, "robot": "fr3"}, mode="fake", gripper="none", ft="none", confirm=True)
    with pytest.raises(CommandError, match="F/T sensor IP"):
        run(s, "robot_setup", **{**UR, "ft_ip": "x"}, mode="fake", gripper="none", ft="ati", confirm=True)
    with pytest.raises(CommandError, match="takes the gripper"):
        run(s, "robot_setup", **UR, mode="fake", gripper="franka_hand", ft="ati", confirm=True)
    # the first pick, same as what runs, plus a camera: kept, camera added, nothing restarts
    r = run(s, "robot_setup", **UR, mode="fake", gripper="robotiq_2f85", ft="ati", cameras=["realsense"])
    assert "nothing to restart" in r["msg"] and r["robot"]["chosen"] and s.restart_code is None
    assert [c["name"] for c in r["robot"]["cameras"]] == ["Intel RealSense"]
    # ticked again: no second RealSense (it would fight the first for the camera)
    r = run(s, "robot_setup", **UR, mode="fake", gripper="robotiq_2f85", ft="ati", cameras=["realsense"])
    assert [c["name"] for c in r["robot"]["cameras"]] == ["Intel RealSense"]
    # a different gripper / F/T: confirm, then the base file and a restart
    with pytest.raises(CommandError, match="confirm"):
        run(s, "robot_setup", **UR, mode="fake", gripper="none", ft="ati")
    s.core.busy = True
    with pytest.raises(CommandError, match="busy"):
        run(s, "robot_setup", **UR, mode="real", gripper="none", ft="ati", confirm=True)
    s.core.busy = False
    restarts = []
    s.on_restart = lambda: restarts.append(s.restart_code)
    r = run(s, "robot_setup", **{**UR, "ft_ip": "192.168.1.7"}, mode="real", gripper="none", ft="ati", confirm=True)
    assert r["restart"] == "stack" and "UR7e, real, f/t sensor" in r["msg"]
    assert (cfg / "robot").read_text() == "robot=ur7e\nmode=real\nip=10.6.203.101\nft_ip=192.168.1.7\ndomain=42\nnet=lab\nservo=on\n"
    assert (cfg / "base").read_text().strip() == "ft"
    assert wait(lambda: restarts == [RESTART_STACK], 3)


def test_ros_domain_and_network_restart_the_stack(env, monkeypatch):
    s, _, tmp = env
    for k, v in {"UR7E_ROBOT": "ur7e", "UR7E_ROBOT_MODE": "fake", "UR7E_ROBOT_IP": "10.6.203.101",
                 "UR7E_EE_CONFIG": "ft_gripper", "ROS_DOMAIN_ID": "42", "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
                 "UR7E_SERVO": "on"}.items():
        monkeypatch.setenv(k, v)
    v = run(s, "robot_view")["robot"]
    assert (v["running"]["domain"], v["running"]["net"]) == ("42", "lab") and [n["name"] for n in v["nets"]] == ["lab", "local"]
    args = dict(**UR, mode="fake", gripper="robotiq_2f85", ft="ati")
    assert "nothing to restart" in run(s, "robot_setup", **args, domain="42", net="lab")["msg"]
    with pytest.raises(CommandError, match="confirm"):
        run(s, "robot_setup", **args, domain="7", net="lab")
    r = run(s, "robot_setup", **args, domain="7", net="local", confirm=True)
    assert r["restart"] == "stack" and "ROS domain 7, this pc only" in r["msg"]
    assert "domain=7\nnet=local\n" in (tmp / "cfg" / "moveit_gui" / "robot").read_text()
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    assert run(s, "robot_view")["robot"]["running"]["net"] == "local"


# ------------------------------------------------------------------ the launcher, with a stub docker
STUB = r"""#!/usr/bin/env bash
# a stand-in for docker: logs what the launcher asks, starts nothing
printf '%s\n' "$*" | tr '\n' ' ' >> "$STUB_LOG"; echo >> "$STUB_LOG"     # one line per call
case "$1" in
  image)                                    # the robot image: STUB_CREATED, STUB_LABEL (deps hash), STUB_IMAGE_ID
    [ -n "$STUB_NO_IMAGE" ] && exit 1
    case "$*" in
      *Labels*) echo "${STUB_LABEL-<no value>}" ;;
      *.Id*)    echo "sha256:${STUB_IMAGE_ID:-1}" ;;
      *)        echo "${STUB_CREATED:-2099-01-01T00:00:00Z}" ;;
    esac ;;
  ps)    [ "$2" = "-a" ] || { [ -f "$STUB_DIR/up" ] && echo ur7e_robotiq; } ;;
  run)   [ "$2" = "-d" ] && touch "$STUB_DIR/up"
         case "$*" in                       # the overlay build: leaves its stamp, as the real one does
           *"--base-paths /ur7e_robotiq"*)
             for a in "$@"; do case "$a" in ID=*) id=${a#ID=} ;; esac; done
             mkdir -p "$HOME/ur7e_data/overlay"; printf '%s' "$id" > "$HOME/ur7e_data/overlay/.built_for" ;;
         esac ;;
  stop)  rm -f "$STUB_DIR/up" ;;
  exec)
    case "$*" in
      *list_controllers*) echo "joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster  active" ;;
      *moveit_gui_web*)                     # the GUI: the first one asks for a restart (after STUB_GUI_FIRST)
        n=$(cat "$STUB_DIR/guis" 2>/dev/null || echo 0); echo $((n + 1)) > "$STUB_DIR/guis"
        if [ "$n" = 0 ]; then eval "$STUB_GUI_FIRST"; exit ${STUB_GUI_CODE:-75}; fi
        exit 0 ;;
    esac ;;
esac
exit 0
"""


def launch(tmp_path, args, gui_first="", env_extra=None):
    home, stub = tmp_path / "home", tmp_path / "stub"
    for d in (home / "ur7e_robotiq", home / "ws", home / "ur7e_data" / "config" / "moveit_gui", stub):
        d.mkdir(parents=True, exist_ok=True)
    (stub / "docker").write_text(STUB)
    (stub / "docker").chmod(0o755)
    log = tmp_path / "docker.log"
    env = {"HOME": str(home), "PATH": f"{stub}:/usr/bin:/bin", "STUB_LOG": str(log), "STUB_DIR": str(stub),
           "STUB_GUI_FIRST": gui_first, "UR7E_IMAGE": "stub:img", **(env_extra or {})}
    p = subprocess.run(["bash", str(ROOT / "scripts" / "ur7e_gui.sh"), *args], env=env, capture_output=True,
                       text=True, timeout=60)
    lines = log.read_text().splitlines()
    starts = [l for l in lines if l.startswith("run -d")]
    guis = [l for l in lines if "moveit_gui_web" in l]
    return p, starts, guis, home / "ur7e_data" / "config" / "moveit_gui" / "robot"


def test_launcher_follows_the_gui_switch(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    # started with --fake; the GUI switches to the real robot at 10.6.203.7 and asks for a stack restart
    p, starts, guis, _ = launch(tmp_path, ["--fake"],
                                gui_first=f"printf 'robot=ur7e\\nmode=real\\nip=10.6.203.7\\n' > {f}")
    assert p.returncode == 0, p.stdout + p.stderr
    assert len(starts) == 2 and len(guis) == 2
    assert "UR7E_ROBOT_MODE=fake" in starts[0] and "sleep infinity" in starts[0] and "--fake" in guis[0]
    assert "UR7E_ROBOT_MODE=real" in starts[1] and "robot_ip:=10.6.203.7" in starts[1]
    assert "UR7E_ROBOT_IP=10.6.203.7" in starts[1] and "--fake" not in guis[1]
    assert "ft_ip_address:=192.168.1.1" in starts[1]                  # the default, no ft_ip= line


def test_launcher_keeps_its_options_over_a_plain_restart(tmp_path):
    p, starts, guis, f = launch(tmp_path, ["--fake"])              # restart (75) without a robot switch
    assert p.returncode == 0, p.stdout + p.stderr
    assert len(starts) == 2 and all("UR7E_ROBOT_MODE=fake" in s for s in starts)


def test_launcher_reads_the_file(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("robot=ur7e\nmode=fake\nip=10.1.2.3\nft_ip=10.1.2.4\n")
    p, starts, guis, _ = launch(tmp_path, [])
    assert p.returncode == 0, p.stdout + p.stderr
    assert "UR7E_ROBOT_MODE=fake" in starts[0] and "UR7E_ROBOT_IP=10.1.2.3" in starts[0]
    assert "UR7E_FT_IP=10.1.2.4" in starts[0]
    f.write_text("mode=real\n")                                     # ip missing: the lab robot's
    (tmp_path / "docker.log").unlink()
    (tmp_path / "stub" / "guis").unlink()
    p, starts, guis, _ = launch(tmp_path, ["--robot-ip", "10.9.9.9"])
    assert "robot_ip:=10.9.9.9" in starts[0]                        # the option wins for this start


def test_launcher_ros_domain_and_network(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("robot=ur7e\nmode=fake\ndomain=7\nnet=local\n")
    p, starts, guis, _ = launch(tmp_path, [])
    assert p.returncode == 0, p.stdout + p.stderr
    assert "ROS_DOMAIN_ID=7 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in starts[0]
    f.write_text("mode=fake\ndomain=500\nnet=wifi\n")             # wrong values: the defaults, and a word why
    (tmp_path / "docker.log").unlink()
    (tmp_path / "stub" / "guis").unlink()
    p, starts, guis, _ = launch(tmp_path, [])
    assert "ROS_DOMAIN_ID=42 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in starts[0]
    assert "not 0-101" in p.stderr and "not lab or local" in p.stderr
    (tmp_path / "docker.log").unlink()
    (tmp_path / "stub" / "guis").unlink()
    p, starts, guis, _ = launch(tmp_path, [], env_extra={"UR7E_ROS_DOMAIN_ID": "77"})
    assert "ROS_DOMAIN_ID=77" in starts[0]                          # the variable wins for this start
    p, _, _, _ = launch(tmp_path, [], env_extra={"UR7E_ROS_DOMAIN_ID": "abc"})
    assert p.returncode == 1 and "0-101" in p.stdout


def test_launcher_starts_the_fr3_in_its_own_image(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("robot=fr3\nmode=real\nip=172.16.0.2\nft_ip=192.168.1.1\n")
    (f.parent / "base").write_text("gripper\n")                    # Franka Hand, no F/T sensor
    fr3ws = tmp_path / "fr3ws"
    (fr3ws / "install").mkdir(parents=True)
    (fr3ws / "install" / "setup.bash").write_text("")
    p, starts, guis, _ = launch(tmp_path, [], env_extra={"UR7E_FR3_WS": str(fr3ws), "UR7E_FR3_IMAGE": "fr3:img"})
    assert p.returncode == 0, p.stdout + p.stderr
    log = (tmp_path / "docker.log").read_text()
    assert "/ws -w /ws fr3:img" in log                                  # the GUI workspace built in the FR3 image
    st = starts[0]
    assert "fr3:img" in st and "-u root" in st and f"{fr3ws}:/ros2_ws:ro" in st and "rtprio=99" in st
    assert "franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2" in st
    assert "load_gripper:=true" in st and "load_ft_sensor:=false" in st
    assert "franka_gripper gripper.launch.py robot_ip:=172.16.0.2" in st
    assert "UR7E_ROBOT_STATUS=ur7e=ok|fr3=ok" in st
    assert "--profile fr3" in guis[0]


def test_launcher_refuses_an_fr3_it_cannot_start(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("robot=fr3\nmode=real\nip=172.16.0.2\n")
    p, starts, guis, _ = launch(tmp_path, [], env_extra={"UR7E_FR3_WS": str(tmp_path / "nowhere")})
    assert p.returncode != 0 and not starts and "Cannot start fr3" in p.stdout and "not built" in p.stdout


def test_launcher_starts_the_zed_container_on_a_jetson(tmp_path):
    stub = tmp_path / "stub"
    stub.mkdir(parents=True)
    (stub / "uname").write_text("#!/bin/sh\necho aarch64\n")
    (stub / "uname").chmod(0o755)
    sdk = tmp_path / "zed"
    (sdk / "lib").mkdir(parents=True)
    (sdk / "lib" / "libsl_zed.so").write_text("")
    p, starts, guis, _ = launch(tmp_path, ["--fake"], env_extra={"UR7E_ZED_SDK": str(sdk)})
    assert p.returncode == 0, p.stdout + p.stderr
    zed, robot = starts[0], starts[1]
    assert "--name ur7e_robotiq_zed" in zed and "moveit_gui_zed:humble python3 /zed_runner.py /data/run/zed" in zed
    assert f"{sdk}:/usr/local/zed:ro" in zed and "ROS_DOMAIN_ID=42" in zed
    assert "MOVEIT_GUI_ZED_DIR=/data/run/zed" in robot and "c 189:* rmw" in robot      # USB cameras too
    assert "--fake --real-cameras" in guis[0]
    log = (tmp_path / "docker.log").read_text()
    assert "stop ur7e_robotiq_zed" in log                              # stopped with the stack


def test_launcher_has_no_zed_container_on_a_desktop(tmp_path):
    sdk = tmp_path / "zed"
    (sdk / "lib").mkdir(parents=True)
    (sdk / "lib" / "libsl_zed.so").write_text("")
    stub = tmp_path / "stub"
    stub.mkdir(parents=True)
    (stub / "uname").write_text("#!/bin/sh\necho x86_64\n")
    (stub / "uname").chmod(0o755)
    p, starts, guis, _ = launch(tmp_path, ["--fake"], env_extra={"UR7E_ZED_SDK": str(sdk)})
    assert p.returncode == 0, p.stdout + p.stderr
    assert not any("--name ur7e_robotiq_zed" in s for s in starts) and "MOVEIT_GUI_ZED_DIR" not in starts[0]


def test_launcher_lan_also_listens_on_the_lab_address(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("mode=fake\n")
    stub = tmp_path / "stub"
    stub.mkdir(parents=True, exist_ok=True)
    (stub / "ip").write_text("#!/bin/sh\necho '1.1.1.1 via 10.6.203.1 dev eth0 src 10.6.203.38 uid 1000'\n")
    (stub / "tailscale").write_text("#!/bin/sh\necho 100.79.241.116\n")
    for n in ("ip", "tailscale"):
        (stub / n).chmod(0o755)
    p, starts, guis, _ = launch(tmp_path, ["--lan"])
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--host 100.79.241.116 --also-host 10.6.203.38 --port 8180" in guis[0]


def test_launcher_starts_servo_unless_the_file_says_off(tmp_path):
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("robot=ur7e\nmode=real\n")                         # no servo= line: on
    p, starts, guis, _ = launch(tmp_path, [])
    assert p.returncode == 0, p.stdout + p.stderr
    assert "servo:=true" in starts[0] and "UR7E_SERVO=on" in starts[0]
    f.write_text("robot=ur7e\nmode=real\nservo=off\n")
    (tmp_path / "docker.log").unlink()
    (tmp_path / "stub" / "guis").unlink()
    p, starts, guis, _ = launch(tmp_path, [])
    assert "servo:=false" in starts[0] and "UR7E_SERVO=off" in starts[0]


def test_servo_setting_round_trip(tmp_path):
    assert R.write_next(tmp_path, "ur7e", "real", "10.0.0.5", servo="off")["servo"] == "off"
    assert R.read_next(tmp_path)["servo"] == "off"
    assert R.check("ur7e", "real", "10.0.0.5", servo=True)["servo"] == "on"
    with pytest.raises(R.RobotError):
        R.check("ur7e", "real", "10.0.0.5", servo="maybe")


def test_launcher_lets_the_control_loop_run_real_time(tmp_path):
    p, starts, guis, _ = launch(tmp_path, ["--fake"])
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--ulimit rtprio=99" in starts[0]


def test_launcher_runs_only_the_start_page_while_the_stack_is_stopped(tmp_path):
    cfg = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui"
    cfg.mkdir(parents=True)
    (cfg / "robot").write_text("robot=ur7e\nmode=real\nip=10.6.203.7\n")
    # the GUI's Stop robot stack: the marker, then exit 75
    p, starts, guis, _ = launch(tmp_path, [], gui_first=f"echo '{{}}' > {cfg / 'stack_off'}")
    assert p.returncode == 0, p.stdout + p.stderr
    log = (tmp_path / "docker.log").read_text()
    assert len(starts) == 2 and "ros2 launch ur7e_robotiq_bringup" in starts[0]
    assert "sleep infinity" in starts[1] and "ros2 launch" not in starts[1]          # nothing of the robot runs
    assert len(guis) == 1 and "moveit_gui_idle" in log and "Robot stack stopped" in p.stdout



def test_launcher_shows_the_start_progress_while_the_driver_comes_up(tmp_path):
    cfg = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui"
    cfg.mkdir(parents=True)
    (cfg / "robot").write_text("robot=ur7e\nmode=real\nip=10.6.203.7\n")
    p, starts, guis, _ = launch(tmp_path, [], gui_first="true", env_extra={"STUB_GUI_CODE": "0"})
    assert p.returncode == 0, p.stdout + p.stderr
    log = (tmp_path / "docker.log").read_text().splitlines()
    page = [i for i, l in enumerate(log) if "moveit_gui_idle" in l and "--progress" in l]
    gui = [i for i, l in enumerate(log) if "moveit_gui_web" in l]
    kill = [i for i, l in enumerate(log) if "stack_progress.pid" in l and "kill" in l]
    assert len(page) == 1 and kill and page[0] < kill[0] < gui[0]       # stopped before the GUI takes the port
    assert (cfg / "stack_progress").read_text().startswith("stage=moveit\n")

# ------------------------------------------------------------------ the image rebuild and the overlay
def ur_repo(tmp_path, deps="h1"):
    """A ~/ur7e_robotiq with docker/deps_hash.sh (prints `deps`), a build.sh that only logs, and one package."""
    repo = tmp_path / "home" / "ur7e_robotiq"
    (repo / "docker").mkdir(parents=True, exist_ok=True)
    (repo / "pkg_a" / "launch").mkdir(parents=True, exist_ok=True)
    (repo / "docker" / "Dockerfile").write_text("FROM x\n")
    (repo / "docker" / "deps_hash.sh").write_text(
        f'#!/bin/sh\n[ "$1" = --files ] && {{ echo docker/Dockerfile; echo pkg_a/package.xml; exit 0; }}\necho {deps}\n')
    (repo / "docker" / "build.sh").write_text('#!/bin/sh\necho "BUILD $UR7E_IMAGE" >> "$STUB_LOG"\n')
    for n in ("deps_hash.sh", "build.sh"):
        (repo / "docker" / n).chmod(0o755)
    (repo / "pkg_a" / "package.xml").write_text("<package><name>pkg_a</name></package>\n")
    (repo / "pkg_a" / "launch" / "a.launch.py").write_text("")
    return repo


def builds(tmp_path):
    return [l for l in (tmp_path / "docker.log").read_text().splitlines() if l.startswith("BUILD")]


def overlay_builds(tmp_path):
    return [l for l in (tmp_path / "docker.log").read_text().splitlines() if "--base-paths /ur7e_robotiq" in l]


def fresh(tmp_path):
    for f in (tmp_path / "docker.log", tmp_path / "stub" / "guis"):
        f.unlink(missing_ok=True)


@pytest.mark.parametrize("label,created,rebuild", [
    ("h1", None, False),                              # the image was built from these dependencies
    ("h0", None, True),                               # a package.xml / the Dockerfile / a patch changed
    ("<no value>", None, False),                      # built before the labels, and nothing it needs is newer
    ("<no value>", "2000-01-01T00:00:00Z", True),     # built before the labels, and the Dockerfile is newer
])
def test_launcher_rebuilds_the_image_only_for_its_dependencies(tmp_path, label, created, rebuild):
    ur_repo(tmp_path)
    extra = {"STUB_LABEL": label, **({"STUB_CREATED": created} if created else {})}
    p, starts, guis, _ = launch(tmp_path, ["--fake"], env_extra=extra)
    assert p.returncode == 0, p.stdout + p.stderr
    assert builds(tmp_path) == (["BUILD stub:img"] if rebuild else [])


def test_launcher_does_not_rebuild_for_a_launch_file(tmp_path):
    # an image from before the labels, older than a launch file but not than the dependency files
    repo = ur_repo(tmp_path)
    for f in ("docker/Dockerfile", "pkg_a/package.xml"):
        os.utime(repo / f, (946684800, 946684800))                  # 2000-01-01
    p, *_ = launch(tmp_path, ["--fake"], env_extra={"STUB_CREATED": "2010-01-01T00:00:00Z"})
    assert p.returncode == 0, p.stdout + p.stderr
    assert builds(tmp_path) == [] and len(overlay_builds(tmp_path)) == 1


def test_launcher_builds_a_missing_image(tmp_path):
    ur_repo(tmp_path)
    p, *_ = launch(tmp_path, ["--fake"], env_extra={"STUB_NO_IMAGE": "1"})
    assert builds(tmp_path) == ["BUILD stub:img"]


def test_launcher_keeps_the_old_rule_for_an_old_checkout(tmp_path):
    # ~/ur7e_robotiq without docker/deps_hash.sh: any newer file rebuilds the image, and its packages come from
    # the image (no overlay)
    repo = tmp_path / "home" / "ur7e_robotiq"
    (repo / "docker").mkdir(parents=True)
    (repo / "docker" / "build.sh").write_text('#!/bin/sh\necho "BUILD $UR7E_IMAGE" >> "$STUB_LOG"\n')
    (repo / "docker" / "build.sh").chmod(0o755)
    (repo / "x.yaml").write_text("")
    p, starts, guis, _ = launch(tmp_path, ["--fake"], env_extra={"STUB_CREATED": "2000-01-01T00:00:00Z"})
    assert p.returncode == 0, p.stdout + p.stderr
    assert builds(tmp_path) == ["BUILD stub:img"] and not overlay_builds(tmp_path)
    assert "/data/overlay" not in starts[0] and "/ur7e_robotiq" not in starts[0]


def test_launcher_runs_the_stack_on_the_overlay(tmp_path):
    repo = ur_repo(tmp_path)
    p, starts, guis, _ = launch(tmp_path, [], gui_first="")                   # real robot, restarts the stack once
    assert p.returncode == 0, p.stdout + p.stderr
    ov = overlay_builds(tmp_path)
    assert len(ov) == 1                               # the restart finds it current
    assert f"{repo}:/ur7e_robotiq:ro" in ov[0] and "--network none" in ov[0] and "ID=sha256:1" in ov[0]
    assert "--symlink-install" in ov[0] and "--packages-select pkg_a" in ov[0] and "--allow-overriding pkg_a" in ov[0]
    data = tmp_path / "home" / "ur7e_data"
    assert (data / "overlay" / ".built_for").read_text() == "sha256:1"
    for st in starts:                                 # sourced after the image's workspace, the source mounted
        assert "source /opt/ur7e_ws/install/setup.bash && source /data/overlay/install/setup.bash" in st
        assert f"{repo}:/ur7e_robotiq:ro" in st
    assert all("source /data/overlay/install/setup.bash" in g for g in guis)
    # nothing changed: no build; an edited launch file or a new image: built again
    fresh(tmp_path)
    old = 946684800
    os.utime(data / "overlay" / ".built_for", (old + 100, old + 100))
    for f in [repo, *repo.rglob("*")]:            # (a new or deleted file changes its directory)
        os.utime(f, (old, old))
    launch(tmp_path, ["--fake"])
    assert overlay_builds(tmp_path) == []
    fresh(tmp_path)
    (repo / "pkg_a" / "launch" / "a.launch.py").write_text("# edited")
    launch(tmp_path, ["--fake"])
    assert len(overlay_builds(tmp_path)) == 1 and builds(tmp_path) == []
    fresh(tmp_path)
    launch(tmp_path, ["--fake"], env_extra={"STUB_IMAGE_ID": "2"})
    assert len(overlay_builds(tmp_path)) == 1


def test_launcher_stops_when_the_overlay_does_not_build(tmp_path):
    ur_repo(tmp_path)
    stub = tmp_path / "stub"
    stub.mkdir(parents=True)
    real = stub / "docker"
    # a docker whose overlay build fails (the stub, renamed, does the rest)
    p, *_ = launch(tmp_path, ["--fake"])                               # writes the stub
    real.rename(stub / "docker_stub")
    real.write_text('#!/bin/bash\ncase "$*" in *"--base-paths /ur7e_robotiq"*) echo "CMake Error: boom"; exit 2 ;; esac\n'
                    'exec "$(dirname "$0")/docker_stub" "$@"\n')
    real.chmod(0o755)
    (tmp_path / "home" / "ur7e_data" / "overlay" / ".built_for").unlink()
    fresh(tmp_path)
    env = {"HOME": str(tmp_path / "home"), "PATH": f"{stub}:/usr/bin:/bin", "STUB_LOG": str(tmp_path / "docker.log"),
           "STUB_DIR": str(stub), "STUB_GUI_FIRST": "", "UR7E_IMAGE": "stub:img"}
    p = subprocess.run(["bash", str(ROOT / "scripts" / "ur7e_gui.sh"), "--fake"], env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode != 0 and "boom" in p.stdout and "overlay_build.log" in p.stdout
    assert not [l for l in (tmp_path / "docker.log").read_text().splitlines() if l.startswith("run -d")]


def test_launcher_builds_no_overlay_for_the_fr3(tmp_path):
    ur_repo(tmp_path)
    f = tmp_path / "home" / "ur7e_data" / "config" / "moveit_gui" / "robot"
    f.parent.mkdir(parents=True)
    f.write_text("robot=fr3\nmode=fake\n")
    fr3ws = tmp_path / "fr3ws"
    (fr3ws / "install").mkdir(parents=True)
    (fr3ws / "install" / "setup.bash").write_text("")
    p, starts, guis, _ = launch(tmp_path, [], env_extra={"UR7E_FR3_WS": str(fr3ws), "UR7E_FR3_IMAGE": "fr3:img"})
    assert p.returncode == 0, p.stdout + p.stderr
    assert not overlay_builds(tmp_path) and "/data/overlay" not in starts[0]


def test_a_gui_restart_builds_the_gui_workspace_again(tmp_path):
    # the GUI asks for a GUI-only restart (76): ~/ws is built again, so pulled code runs
    p, starts, guis, _ = launch(tmp_path, ["--fake"], env_extra={"STUB_GUI_CODE": "76"})
    assert p.returncode == 0, p.stdout + p.stderr
    log = (tmp_path / "docker.log").read_text()
    assert len(starts) == 1 and len(guis) == 2
    assert log.count("colcon build --packages-select pymoveit2 moveit_gui") == 2
