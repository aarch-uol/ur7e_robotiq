#!/usr/bin/env bash
# One command for the UR7e + MoveIt GUI:
#   1. rebuilds the ur7e_robotiq:jazzy image if the dependencies in ~/ur7e_robotiq changed (docker/deps_hash.sh)
#   2. builds ~/ur7e_robotiq's packages (~/ur7e_data/overlay, seconds) and the GUI workspace (~/ws: moveit_gui + pymoveit2)
#   3. starts the robot stack (gui.launch.py) in the container
#   4. starts the GUI. Ctrl-C stops the GUI and the robot stack
#      gracefully (GUI: SIGINT, 10 s grace; stack: SIGINT, 30 s grace; nothing is SIGKILLed unless it hangs).
#   The GUI can ask for a restart (SETUP > Parts / Setups): exit code 76 restarts the GUI, 75 the robot stack too.
#   Parts made in the GUI (~/ur7e_data/config/moveit_gui/parts/generated) go into the robot model at stack start.
#
#   ur7e_gui.sh                         # real robot, web GUI on http://<tailscale ip>:8180/
#   ur7e_gui.sh --desktop               # real robot, desktop (Qt) GUI on this PC
#   ur7e_gui.sh --fake                  # no robot: simulated arm (web; add --desktop for Qt)
#   ur7e_gui.sh --robot-ip 10.6.203.101 -- rviz:=true servo:=true    # extra gui.launch.py args after --
#   ur7e_gui.sh --base gripper          # switch the end-effector base (kept for the next starts; see below)
#   ur7e_gui.sh --lan                   # web GUI on Tailscale and on this PC's lab network address too
#                                       # (or UR7E_GUI_LAN=1): anyone on that network can open it
#
# Robot = which arm, real or fake, its IP and the F/T sensor's, the ROS domain and network. Kept in
# ~/ur7e_data/config/moveit_gui/robot (robot=, mode=, ip=, ft_ip=, domain=, net=, servo= lines); SETUP > Setups > Set up… changes it with the base (and restarts the stack).
# --fake and --robot-ip win for this start only.
#
# Base = what is on the flange, a fixed part of the robot model the GUI cannot edit (parts go on top of it):
#   ft_gripper (default)  F/T sensor + Robotiq gripper     ft        F/T sensor only (your own tool on it)
#   gripper               Robotiq gripper on tool0          none      the bare arm
# Kept in ~/ur7e_data/config/moveit_gui/base; SETUP > Setups switches it too (and restarts the stack).
#
# Cameras: the robot container can open USB cameras (RealSense, webcams); SETUP > Drivers & cameras starts and
# stops their drivers. A ZED X on a Jetson runs in its own container (moveit_gui_zed:humble, built once with
# ~/ws/src/moveit_gui/docker/zed/build.sh), started here next to the robot stack. With --fake the robot is
# simulated but the cameras are real.
#
# Power on / Release brakes work from any browser that has control (after "I can see the robot");
# MOVEIT_GUI_LOCAL_POWER_ONLY=1 ur7e_gui.sh limits them to a browser on this PC (and the pendant).
# Saved data (programs, TCPs, fixtures, payload/vial results, logs) goes to ~/ur7e_data.
set -eo pipefail

CLI_IP=""
CLI_FAKE=0
LAN=${UR7E_GUI_LAN:-0}
MODE=web
PORT=8180
# Our own ROS domain: other PCs on the lab network run ROS on the default domain 0 (e.g. a Gazebo UR sim on
# 10.6.203.36), and a second /controller_manager on the same domain breaks controller switching.
# Anything that must talk to this stack (policy runner, ros2 CLI) needs  export ROS_DOMAIN_ID=42  too.
# The domain and who can see the topics (net=lab: the network, net=local: this PC only) are kept with the robot
# below (SETUP > Setups > Hardware…); UR7E_ROS_DOMAIN_ID=<n> wins for this start, like --fake.
CLI_DOMAIN=${UR7E_ROS_DOMAIN_ID:-}
if [ -n "$CLI_DOMAIN" ] && ! { [[ "$CLI_DOMAIN" =~ ^[0-9]{1,3}$ ]] && [ "$((10#$CLI_DOMAIN))" -le 101 ]; }; then
  echo "UR7E_ROS_DOMAIN_ID must be 0-101"; exit 1
fi
DOMAIN=42
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --robot-ip) CLI_IP="$2"; shift 2 ;;
    --desktop)  MODE=desktop; shift ;;
    --fake)     CLI_FAKE=1; shift ;;
    --port)     PORT="$2"; shift 2 ;;
    --lan)      LAN=1; shift ;;
    --base)     NEW_BASE="$2"; shift 2 ;;
    --)         shift; LAUNCH_ARGS=("$@"); break ;;
    -h|--help)  sed -n 2,35p "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)"; exit 1 ;;
  esac
done

# The robot image for this PC's CPU (~/ur7e_robotiq/docker/build.sh builds the same tag): amd64 desktop PCs use
# ur7e_robotiq:jazzy, arm64 Jetsons ur7e_robotiq:jazzy-arm64. UR7E_IMAGE=name:tag overrides it.
case "$(uname -m)" in
  aarch64|arm64) IMAGE=${UR7E_IMAGE:-ur7e_robotiq:jazzy-arm64} ;;
  *)             IMAGE=${UR7E_IMAGE:-ur7e_robotiq:jazzy} ;;
esac
export UR7E_IMAGE=$IMAGE
NAME=${UR7E_CONTAINER:-ur7e_robotiq}
ZED_NAME=${NAME}_zed
ZED_IMAGE=${UR7E_ZED_IMAGE:-moveit_gui_zed:humble}
ZED_SDK=${UR7E_ZED_SDK:-/usr/local/zed}              # the host's ZED SDK (Jetson)
REPO="$HOME/ur7e_robotiq"
WS="$HOME/ws"
DATA="$HOME/ur7e_data"
mkdir -p "$DATA/logs"
say() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 1. image
# The image holds the dependencies; ~/ur7e_robotiq's own packages are built from the source at each stack start
# (build_overlay below), so changing a launch file, a yaml or a script needs no new image. The image is rebuilt only
# when docker/deps_hash.sh (in ~/ur7e_robotiq) gives another hash than the one build.sh stored in the image: the
# docker/ files, the workspace script, the patches or a package.xml changed. An image built before these labels
# counts as current unless one of those files is newer than it. A ~/ur7e_robotiq without deps_hash.sh (older
# checkout) keeps the old rule: rebuild when any file is newer than the image, and no overlay.
DEPS_LABEL=ur7e_robotiq.deps
OVERLAY=0
image_rebuild_reason() {                            # prints why the image must be (re)built; nothing if it is current
  local created have want newest
  created=$(docker image inspect "$IMAGE" --format '{{.Created}}' 2>/dev/null || true)
  if [ -z "$created" ]; then echo "Image $IMAGE missing: building it (takes a while)"; return 0; fi
  if [ -x "$REPO/docker/deps_hash.sh" ]; then
    want=$("$REPO/docker/deps_hash.sh")
    have=$(docker image inspect "$IMAGE" --format "{{index .Config.Labels \"$DEPS_LABEL\"}}" 2>/dev/null || true)
    [ "$have" = "<no value>" ] && have=""
    if [ -n "$have" ]; then
      [ "$have" = "$want" ] || echo "The image's dependencies changed in ~/ur7e_robotiq: rebuilding $IMAGE"
      return 0
    fi
    newest=$(cd "$REPO" && "$REPO/docker/deps_hash.sh" --files | tr '\n' '\0' | xargs -0 -r stat -c %Y | sort -n | tail -1)
  else
    newest=$(find "$REPO" -path "$REPO/.git" -prune -o -name __pycache__ -prune -o -type f -printf '%T@\n' \
              | sort -n | tail -1 | cut -d. -f1)
  fi
  [ "${newest:-0}" -gt "$(date -d "$created" +%s)" ] && echo "~/ur7e_robotiq changed since the image was built: rebuilding $IMAGE"
  return 0
}
reason=$(image_rebuild_reason)
if [ -n "$reason" ]; then
  say "$reason"
  "$REPO/docker/build.sh"
fi
[ -x "$REPO/docker/deps_hash.sh" ] && OVERLAY=1

# ---------------------------------------------------------------- 2. GUI workspace (built in the robot's image)
WS_BUILT_FOR=""
build_ws() {                                        # $1: the image the stack and the GUI run in
  [ "$WS_BUILT_FOR" = "$1" ] && return 0
  say "Building the GUI workspace (~/ws) in $1"
  docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$WS:/ws" -w /ws "$1" \
    bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select pymoveit2 moveit_gui' \
    > "$DATA/logs/ws_build.log" 2>&1 || { tail -30 "$DATA/logs/ws_build.log"; return 1; }
  WS_BUILT_FOR=$1
}

# ---------------------------------------------------------------- ~/ur7e_robotiq's packages (overlay on the image)
# Built from ~/ur7e_robotiq (mounted read-only at /ur7e_robotiq) into ~/ur7e_data/overlay, incrementally, with
# --symlink-install: launch files, yaml, URDF and Python scripts are links to the source, so editing one takes
# effect at the next stack start; new files and C++ are built here in seconds. The stack sources it after the
# image's /opt/ur7e_ws (whose copies of these packages it overrides). Skipped when nothing changed since the last
# build for this image (the stamp has the time that build started); a new image starts it from scratch.
OVERLAY_DIR=/data/overlay                           # $DATA/overlay on this PC
OVERLAY_STAMP="$DATA/overlay/.built_for"            # the image id it was built for
overlay_pkgs() { sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$REPO"/*/package.xml | paste -sd' '; }
build_overlay() {
  [ "$OVERLAY" = 1 ] || return 0
  local id pkgs
  id=$(docker image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null || true)
  if [ -f "$OVERLAY_STAMP" ] && [ "$(cat "$OVERLAY_STAMP" 2>/dev/null)" = "$id" ] && \
     [ -z "$(find "$REPO" -path "$REPO/.git" -prune -o -newer "$OVERLAY_STAMP" -print -quit)" ]; then
    return 0
  fi
  pkgs=$(overlay_pkgs)
  say "Building ~/ur7e_robotiq's packages (overlay on $IMAGE): $pkgs"
  docker run --rm --network none -v "$REPO:/ur7e_robotiq:ro" -v "$DATA:/data" -e ID="$id" "$IMAGE" bash -c "
    mkdir -p $OVERLAY_DIR && cd $OVERLAY_DIR || exit 1
    [ \"\$(cat .built_for 2>/dev/null)\" = \"\$ID\" ] || rm -rf build install log .built_for
    touch .started && source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash &&
    ov=''; colcon build --help 2>/dev/null | grep -q -- --allow-overriding && ov='--allow-overriding $pkgs'
    colcon build --symlink-install --base-paths /ur7e_robotiq --packages-select $pkgs \$ov &&
    printf '%s' \"\$ID\" > .built_for && touch -r .started .built_for && rm -f .started" \
    > "$DATA/logs/overlay_build.log" 2>&1 || { tail -30 "$DATA/logs/overlay_build.log"; echo "Building ~/ur7e_robotiq failed (log: ~/ur7e_data/logs/overlay_build.log)"; return 1; }
}

# ---------------------------------------------------------------- robots (moveit_gui/robots.py ROBOTS)
# Franka FR3: the aarch-uol/fr3_scraping_ws stack, in its own image with the workspace mounted (built there).
FR3_IMAGE=${UR7E_FR3_IMAGE:-franka_scraping:jazzy}
FR3_WS=${UR7E_FR3_WS:-$HOME/franka_scraping_ws}
robot_status() {                                    # what this PC can start, for the GUI's Set up pop-up
  local fr3=ok
  if ! docker image inspect "$FR3_IMAGE" >/dev/null 2>&1; then
    fr3="the image $FR3_IMAGE is not on this PC: build it from aarch-uol/fr3_scraping_ws (cd docker; docker compose build)"
  elif [ ! -f "$FR3_WS/install/setup.bash" ]; then
    fr3="$FR3_WS is not built: clone aarch-uol/fr3_scraping_ws there and run bash scripts/build in its container"
  fi
  printf 'ur7e=ok|fr3=%s' "$fr3"
}
robot_config() {                                    # ROBOT -> R_IMAGE, UNDERLAY, PROFILE, R_RUN (docker run options)
  case "$ROBOT" in
    fr3)
      R_IMAGE=$FR3_IMAGE; UNDERLAY=/ros2_ws/install/setup.bash; PROFILE=fr3
      # root like the UR stack (the GUI and the stack must be the same user for shared memory); libfranka's 1 kHz
      # loop needs real-time priority, as in fr3_scraping_ws's docker-compose.yml
      R_RUN=(-u root -v "$FR3_WS:/ros2_ws:ro" --cap-add SYS_NICE --ulimit rtprio=99 --ulimit rttime=-1
             --ulimit memlock=8428281856) ;;
    *)
      R_IMAGE=$IMAGE; UNDERLAY=/opt/ur7e_ws/install/setup.bash; PROFILE=ur7e_lab; R_RUN=()
      if [ "$OVERLAY" = 1 ]; then                   # ~/ur7e_robotiq's packages, built by build_overlay
        UNDERLAY="$UNDERLAY && source $OVERLAY_DIR/install/setup.bash"; R_RUN=(-v "$REPO:/ur7e_robotiq:ro")
      fi ;;
  esac
}

# ---------------------------------------------------------------- 3. robot stack
BASES="none gripper ft ft_gripper"
BASE_FILE="$DATA/config/moveit_gui/base"
valid_base() { case " $BASES " in *" $1 "*) return 0 ;; esac; return 1; }
if [ -n "$NEW_BASE" ]; then
  valid_base "$NEW_BASE" || { echo "--base must be one of: $BASES"; exit 1; }
  # ~/ur7e_data belongs to root (the containers write it): write the file the same way
  docker run --rm --network none -v "$DATA:/data" "$IMAGE" \
    bash -c "mkdir -p /data/config/moveit_gui && printf '%s\n' '$NEW_BASE' > /data/config/moveit_gui/base"
fi
read_base() {                                       # the base for this start (file written by --base or the GUI)
  local b
  [ -f "$BASE_FILE" ] && b=$(tr -d ' \t\r\n' < "$BASE_FILE" 2>/dev/null || true)
  if [ -z "$b" ]; then b=ft_gripper
  elif ! valid_base "$b"; then echo "$BASE_FILE says '$b' (not one of: $BASES): using ft_gripper" >&2; b=ft_gripper
  fi
  printf '%s' "$b"
}
PARTS=/data/config/moveit_gui/parts/generated      # in the container; GUI parts for the robot's URDF / SRDF
parts_env() {                                       # -e options for the parts files that exist now
  local f
  for f in "URDF robot_parts.urdf.xacro" "SRDF robot_parts.srdf.xacro"; do
    set -- $f
    [ -f "$DATA/config/moveit_gui/parts/generated/$2" ] && printf -- '-e UR7E_USER_PARTS_%s=%s/%s ' "$1" "$PARTS" "$2"
  done
  return 0
}
ROBOTS="ur7e fr3"                                   # moveit_gui/robots.py ROBOTS
ROBOT_FILE="$DATA/config/moveit_gui/robot"
# SETUP > Setups > Robot > Stop robot stack: while this file exists only a small start page runs (no driver, no
# MoveIt, no ROS), until someone presses Start there
STACK_OFF_FILE="$DATA/config/moveit_gui/stack_off"
# what the start page shows while the stack starts (web/idle.py --progress): stage=driver|moveit|slow|gui, log=<file>
STACK_PROGRESS_FILE="$DATA/config/moveit_gui/stack_progress"
PROGRESS_JOB=""
IDLE=0
OVERRIDE=1                                          # --fake / --robot-ip count until the GUI switches the robot
read_robot() {                                      # sets ROBOT, FAKE, ROBOT_IP, FT_IP, DOMAIN, NET, SERVO for this start
  local k v robot=ur7e mode=real ip=10.6.203.101 ft_ip=192.168.1.1 domain=42 net=lab servo=on
  if [ -f "$ROBOT_FILE" ]; then
    while IFS='=' read -r k v; do
      v=$(printf '%s' "$v" | tr -d ' \t\r')
      case "$k" in
        robot) case " $ROBOTS " in *" $v "*) robot=$v ;; *) echo "$ROBOT_FILE: unknown robot '$v', using ur7e" >&2 ;; esac ;;
        mode)  case "$v" in real|fake) mode=$v ;; *) echo "$ROBOT_FILE: mode '$v' is not real or fake, using real" >&2 ;; esac ;;
        ip)    if [[ "$v" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then ip=$v; else echo "$ROBOT_FILE: '$v' is not an IP" >&2; fi ;;
        ft_ip) if [[ "$v" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then ft_ip=$v; else echo "$ROBOT_FILE: '$v' is not an IP" >&2; fi ;;
        domain) if [[ "$v" =~ ^[0-9]{1,3}$ ]] && [ "$((10#$v))" -le 101 ]; then domain=$((10#$v)); else echo "$ROBOT_FILE: domain '$v' is not 0-101, using 42" >&2; fi ;;
        net)   case "$v" in lab|local) net=$v ;; *) echo "$ROBOT_FILE: net '$v' is not lab or local, using lab" >&2 ;; esac ;;
        servo) case "$v" in on|off) servo=$v ;; *) echo "$ROBOT_FILE: servo '$v' is not on or off, using on" >&2 ;; esac ;;
      esac
    done < "$ROBOT_FILE"
  fi
  if [ "$OVERRIDE" = 1 ]; then
    [ "$CLI_FAKE" = 1 ] && mode=fake
    [ -n "$CLI_IP" ] && ip=$CLI_IP
    [ -n "$CLI_DOMAIN" ] && domain=$CLI_DOMAIN
  fi
  ROBOT=$robot; ROBOT_IP=$ip; FT_IP=$ft_ip; DOMAIN=$domain; NET=$net; SERVO=$servo
  # net=local: ROS 2 discovery stays on this PC. Jazzy reads ROS_AUTOMATIC_DISCOVERY_RANGE; the ZED container
  # (Humble) has only ROS_LOCALHOST_ONLY.
  if [ "$net" = local ]; then
    NET_ENV=(-e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST); ZED_NET_ENV=(-e ROS_LOCALHOST_ONLY=1)
  else
    NET_ENV=(-e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET); ZED_NET_ENV=()
  fi
  if [ "$mode" = fake ]; then FAKE=1; else FAKE=0; fi
  ROBOT_SEEN=$(cat "$ROBOT_FILE" 2>/dev/null || true)
}
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "A container called $NAME is already running (another terminal?). Stop it first:"
  echo "    docker stop $NAME"
  exit 1
fi
# Graceful shutdown, in this order: (1) the GUI gets SIGINT (it runs stop_all), waits up to 10 s, then SIGTERM, then
# SIGKILL; (2) the container is stopped with SIGINT (--stop-signal), so `ros2 launch` shuts the driver and the
# recorders down cleanly (MCAP files finalised), with 30 s before docker's SIGKILL (--stop-timeout).
GUI_PIDFILE=/tmp/moveit_gui.pid
stop_gui() {
  docker exec "$NAME" bash -c '
    p=$(cat '"$GUI_PIDFILE"' 2>/dev/null); [ -n "$p" ] || exit 0
    for step in "INT 10" "TERM 5" "KILL 2"; do
      set -- $step
      kill -$1 "$p" 2>/dev/null || exit 0
      for _ in $(seq 1 $(( $2 * 10 ))); do kill -0 "$p" 2>/dev/null || exit 0; sleep 0.1; done
    done' >/dev/null 2>&1 || true
}
cleanup() {
  trap '' INT TERM            # a second Ctrl-C must not cut the graceful stop short
  say "Stopping the GUI, then the robot stack (up to ~45 s; please wait)"
  stop_gui
  docker stop "$NAME" >/dev/null 2>&1 || true     # uses the container's --stop-signal / --stop-timeout
  stop_zed
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# ---------------------------------------------------------------- ZED camera container (Jetson)
# The ZED SDK, CUDA and the Jetson libraries come from the host, read-only (docker/zed/Dockerfile). The container
# only runs scripts/zed_runner.py, which starts / stops the ZED driver when the GUI asks (files in
# ~/ur7e_data/run/zed): the GUI never gets Docker. Privileged because the ZED X needs the Jetson's camera devices.
ZED_ENV=()
start_zed() {
  ZED_ENV=()
  case "$(uname -m)" in aarch64|arm64) ;; *) return 0 ;; esac     # the image is for Jetsons (JetPack 6)
  [ -f "$ZED_SDK/lib/libsl_zed.so" ] || return 0
  if ! docker image inspect "$ZED_IMAGE" >/dev/null 2>&1; then
    echo "ZED SDK found, but no ZED camera image: build it once with  $WS/src/moveit_gui/docker/zed/build.sh"
    return 0
  fi
  docker rm -f "$ZED_NAME" >/dev/null 2>&1 || true
  local a=/usr/lib/aarch64-linux-gnu
  docker run -d --rm --init --stop-signal SIGINT --stop-timeout 15 --name "$ZED_NAME" --privileged --net=host --ipc=host \
    -e ROS_DOMAIN_ID="$DOMAIN" "${ZED_NET_ENV[@]}" -v /dev:/dev -v /tmp:/tmp \
    -v "$ZED_SDK:/usr/local/zed:ro" -v "$ZED_SDK/resources:/usr/local/zed/resources" \
    -v "$ZED_SDK/settings:/usr/local/zed/settings" \
    -v /usr/local/cuda-12.6:/usr/local/cuda-12.6:ro -v $a/nvidia:$a/nvidia:ro -v $a/tegra-egl:$a/tegra-egl:ro \
    -v $a:/host/lib:ro -v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d:ro \
    -v /var/nvidia/nvcam/settings:/var/nvidia/nvcam/settings \
    -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service:ro \
    -v "$DATA:/data" -v "$WS/src/moveit_gui/scripts/zed_runner.py:/zed_runner.py:ro" \
    "$ZED_IMAGE" python3 /zed_runner.py /data/run/zed >/dev/null || { echo "The ZED container did not start"; return 0; }
  ZED_ENV=(-e MOVEIT_GUI_ZED_DIR=/data/run/zed)
  say "ZED camera container up (start the camera in SETUP > Drivers & cameras)"
}
stop_zed() { docker stop "$ZED_NAME" >/dev/null 2>&1 || true; }

# ---------------------------------------------------------------- 4. GUI
build_gui_cmd() {                                   # GUI_PRE, GUI_CMD for this start (fake or real)
  GUI_ARGS="--profile $PROFILE --programs-dir /data/programs --policies-dir /data/policies"
  [ "$FAKE" = 1 ] && GUI_ARGS="$GUI_ARGS --fake --real-cameras"     # a simulated arm, the real cameras
  if [ "$FAKE" = 1 ] && [ "$ROBOT" = fr3 ]; then
    # the 3D view gets the FR3 model with the hand / F/T sensor this setup has
    local hand=false ft=false
    case "$BASE" in gripper|ft_gripper) hand=true ;; esac
    case "$BASE" in ft|ft_gripper) ft=true ;; esac
    FAKE_URDF="xacro \$(ros2 pkg prefix franka_description)/share/franka_description/robots/fr3/fr3.urdf.xacro hand:=$hand ee_id:=franka_hand robot_ip:=0.0.0.0 use_fake_hardware:=true ros2_control:=false load_ft_sensor:=$ft > /tmp/fake_robot.urdf"
    GUI_PRE="{ $FAKE_URDF || echo 'robot model for the 3D view failed'; } && "
    GUI_ARGS="$GUI_ARGS --urdf /tmp/fake_robot.urdf"
  elif [ "$FAKE" = 1 ]; then
    # the 3D view gets the real robot model, with the GUI parts in it (as the stack would after a restart)
    FAKE_URDF='xacro $(ros2 pkg prefix ur7e_robotiq_bringup)/share/ur7e_robotiq_bringup/urdf/ur7e_robotiq.urdf.xacro name:=ur7e_robotiq ur_type:=ur7e use_mock_hardware:=true robot_ip:=0.0.0.0 > /tmp/fake_robot.urdf'
    GUI_PRE="{ [ -f /data/config/moveit_gui/parts/generated/robot_parts.urdf.xacro ] && export UR7E_USER_PARTS_URDF=$PARTS/robot_parts.urdf.xacro; $FAKE_URDF || echo 'robot model for the 3D view failed'; } && "
    GUI_ARGS="$GUI_ARGS --urdf /tmp/fake_robot.urdf"
  else
    GUI_PRE=""
  fi
  if [ "$MODE" = web ]; then
    HOST=$(tailscale ip -4 2>/dev/null | head -1)
    if [ -z "$HOST" ]; then
      HOST=127.0.0.1
      echo "Tailscale is not up: the web GUI is only reachable from this PC."
    fi
    local also="" lan_ip=""
    if [ "$LAN" = 1 ]; then                         # the address of the default route: the lab network
      lan_ip=$(ip -4 route get 1.1.1.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -1)
      if [ -n "$lan_ip" ] && [ "$lan_ip" != "$HOST" ]; then also="--also-host $lan_ip"
      else echo "--lan: no lab network address found, Tailscale / this PC only"; lan_ip=""; fi
    fi
    say "Web GUI: http://$HOST:$PORT/${lan_ip:+  and http://$lan_ip:$PORT/}   (Ctrl-C here stops the GUI and the robot stack)"
    GUI_CMD="ros2 run moveit_gui moveit_gui_web --host $HOST $also --port $PORT $GUI_ARGS"
    IDLE_CMD="ros2 run moveit_gui moveit_gui_idle --host $HOST $also --port $PORT"
  else
    say "Desktop GUI (closing the window stops the robot stack)"
    GUI_CMD="ros2 run moveit_gui moveit_gui $GUI_ARGS"
  fi
}

start_stack() {
  xhost +local:root >/dev/null 2>&1 || true
  BASE=$(read_base)
  read_robot
  robot_config
  STATUS=$(robot_status)
  case "|$STATUS|" in
    *"|$ROBOT=ok|"*) ;;
    *) echo "Cannot start $ROBOT: $(printf '%s' "$STATUS" | tr '|' '\n' | sed -n "s/^$ROBOT=//p")"
       echo "Pick another robot:  printf 'robot=ur7e\\n' > ~/ur7e_data/config/moveit_gui/robot  (or fix the above)"
       return 1 ;;
  esac
  [ "$ROBOT" = fr3 ] || build_overlay || return 1
  build_ws "$R_IMAGE" || return 1
  build_gui_cmd
  stamp=$(date +%Y%m%d_%H%M%S)
  IDLE=0
  if [ "$MODE" = web ] && [ -f "$STACK_OFF_FILE" ]; then
    IDLE=1
    say "Robot stack stopped (from the GUI): only the start page runs. Press Start there to bring it back."
    STACK='exec sleep infinity'
  elif [ "$FAKE" = 1 ]; then
    say "Fake robot ($ROBOT): no driver, nothing is sent to a robot (ROS domain $DOMAIN, net $NET)"
    STACK='exec sleep infinity'
  else
    # exec: ros2 launch is the container's main process (under --init), so the stop signal reaches it directly.
    # The log goes through a process substitution for the same reason (a `| tee` pipeline would keep bash in between).
    local hand=false ft=false
    if [ "$ROBOT" = fr3 ]; then
      case "$BASE" in gripper|ft_gripper) hand=true ;; esac
      case "$BASE" in ft|ft_gripper) ft=true ;; esac
      LAUNCH="franka_fr3_moveit_config moveit.launch.py robot_ip:=$ROBOT_IP use_fake_hardware:=false controller_type:=position load_gripper:=$hand load_gripper_model:=$hand launch_gripper_node:=$hand load_ft_sensor:=$ft ft_ip_address:=$FT_IP"
    else
      # servo: MoveIt Servo for the hold-to-jog buttons (servo=on|off in the robot file; `-- servo:=...` still wins)
      LAUNCH="ur7e_robotiq_bringup gui.launch.py robot_ip:=$ROBOT_IP ft_ip_address:=$FT_IP servo:=$([ "$SERVO" = on ] && echo true || echo false)"
    fi
    STACK="exec > >(tee /data/logs/launch_$stamp.log) 2>&1; exec ros2 launch $LAUNCH ${LAUNCH_ARGS[*]}"
    if [ "$ROBOT" = fr3 ] && [ "$hand" = true ]; then
      # the Franka Hand's node (fr3_scraping_ws's moveit.launch.py does not start it) next to the arm's launch;
      # the stop signal (SIGINT) reaches both
      STACK="exec > >(tee /data/logs/launch_$stamp.log) 2>&1; ros2 launch franka_gripper gripper.launch.py robot_ip:=$ROBOT_IP & G=\$!; ros2 launch $LAUNCH ${LAUNCH_ARGS[*]} & M=\$!; trap 'kill -INT \$M \$G 2>/dev/null' INT TERM; wait \$M; kill -INT \$G 2>/dev/null; wait"
    fi
  fi
  [ "$IDLE" = 1 ] || start_zed
  # USB cameras (RealSense, webcams): /dev from the host so a re-plugged camera shows up again; only video (81) and
  # USB (189) devices may be opened
  # --ulimit rtprio: the controller managers may run their control loop at real-time priority (SCHED_FIFO 50).
  # Without it they log "Could not enable FIFO RT scheduling policy" and the loop overruns on a busy PC (seen on
  # the Jetson: thousands of overruns, TF up to 0.4 s late).
  # SETUP > Policies: when the policy runner is installed in ~/ws/scraping (/ws/scraping in here), import from its
  # policies folder, make cards with its exporter and run imported policies with its run_policy.py
  POLICY_ENV=()
  if [ -f "$WS/scraping/run_policy.py" ]; then
    POLICY_ENV=(-e MOVEIT_GUI_POLICY_IMPORT_ROOTS=/ws/scraping/policies
      -e "MOVEIT_GUI_POLICY_RUN_COMMAND=python3 /ws/scraping/run_policy.py --model {model} --normalization {normalization} --config {training_config} --robot-config {robot_config} --robot-controller servo_admittance")
    [ -f "$WS/scraping/export_policy_card.py" ] && \
      POLICY_ENV+=(-e "MOVEIT_GUI_POLICY_CARD_COMMAND=python3 /ws/scraping/export_policy_card.py")
  fi
  docker run -d --rm --init --stop-signal SIGINT --stop-timeout 30 --name "$NAME" --net=host --ulimit rtprio=99 -e ROS_DOMAIN_ID="$DOMAIN" "${NET_ENV[@]}" -e UR7E_EE_CONFIG="$BASE" $(parts_env) \
    -v /dev:/dev --device-cgroup-rule 'c 81:* rmw' --device-cgroup-rule 'c 189:* rmw' "${ZED_ENV[@]}" \
    -e UR7E_ROBOT="$ROBOT" -e UR7E_ROBOT_MODE="$([ "$FAKE" = 1 ] && echo fake || echo real)" -e UR7E_ROBOT_IP="$ROBOT_IP" -e UR7E_FT_IP="$FT_IP" \
    -e UR7E_SERVO="$SERVO" \
    -e UR7E_ROBOT_STATUS="$STATUS" "${R_RUN[@]}" \
    -e MOVEIT_GUI_LOCAL_POWER_ONLY="${MOVEIT_GUI_LOCAL_POWER_ONLY:-}" \
    "${POLICY_ENV[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$DATA:/data" -v "$WS:/ws" \
    "$R_IMAGE" bash -c "source /opt/ros/jazzy/setup.bash && source $UNDERLAY && $STACK" >/dev/null

  ENV="source /opt/ros/jazzy/setup.bash && source $UNDERLAY && source /ws/install/setup.bash && export XDG_CONFIG_HOME=/data/config"

  if [ "$FAKE" = 0 ] && [ "$IDLE" = 0 ]; then
    say "Robot stack starting ($ROBOT at $ROBOT_IP, base $BASE, ROS domain $DOMAIN, net $NET). Log: ~/ur7e_data/logs/launch_$stamp.log"
    progress driver
    start_progress_page
    ok=0
    for _ in $(seq 1 45); do
      docker ps --format '{{.Names}}' | grep -qx "$NAME" || { echo "The robot stack exited:"; tail -30 "$DATA/logs/launch_$stamp.log"; return 1; }
      if docker exec "$NAME" bash -c "$ENV && timeout 5 ros2 control list_controllers" 2>/dev/null | grep -qE "^joint_state_broadcaster +[^ ]+ +active"; then ok=1; break; fi
      sleep 2
    done
    if [ "$ok" = 1 ]; then
      say "Controllers are up"
      progress moveit
    else
      progress slow
      echo "Controllers not up after 90 s: starting the GUI anyway (it shows what is missing)."
      echo "Last launch log lines:"; tail -15 "$DATA/logs/launch_$stamp.log"
    fi
    [ "$ROBOT" = fr3 ] && echo "    Franka Desk: joints unlocked, FCI activated, the arm in execution mode (blue light)." \
      || echo "    On the pendant: load and PLAY the External Control program, unless you passed headless_mode:=true."
  fi
}

# The start page with the stack's progress, while the driver comes up (the GUI is not there yet to answer on the
# port). It runs in the new container (the one with ROS and the GUI's code) and is stopped before the GUI starts.
progress() {
  [ "$MODE" = web ] || return 0
  mkdir -p "$(dirname "$STACK_PROGRESS_FILE")"
  printf 'stage=%s\nlog=launch_%s.log\n' "$1" "$stamp" > "$STACK_PROGRESS_FILE.tmp" && mv -f "$STACK_PROGRESS_FILE.tmp" "$STACK_PROGRESS_FILE"
}
start_progress_page() {
  [ "$MODE" = web ] && [ -n "${IDLE_CMD:-}" ] || return 0
  docker exec "$NAME" bash -c "$ENV && echo \$\$ > /tmp/stack_progress.pid && exec $IDLE_CMD --progress" >/dev/null 2>&1 &
  PROGRESS_JOB=$!
}
# stage=gui: the page exits by itself (it polls the file); SIGINT too, as for the GUI: ros2 run passes SIGINT on to
# the page (SIGTERM would only end ros2 run and leave the page on the port)
stop_progress_page() {
  [ -n "$PROGRESS_JOB" ] || return 0
  progress gui
  docker exec "$NAME" bash -c 'kill -INT "$(cat /tmp/stack_progress.pid)"' >/dev/null 2>&1 || true
  for _ in $(seq 1 40); do kill -0 "$PROGRESS_JOB" 2>/dev/null || break; sleep 0.25; done
  kill "$PROGRESS_JOB" 2>/dev/null || true
  wait "$PROGRESS_JOB" 2>/dev/null || true
  PROGRESS_JOB=""
}

# In the background + wait, so Ctrl-C reaches the cleanup at once. The GUI records its pid and is exec'd, so
# cleanup (stop_gui) can signal it by itself before the container is stopped.
# Its output also goes to ~/ur7e_data/logs/gui_<time>.log (what the GUI did after a restart: F/T bias, gains, ...).
run_gui() {
  local gstamp; gstamp=$(date +%Y%m%d_%H%M%S)
  say "GUI log: ~/ur7e_data/logs/gui_$gstamp.log"
  local pre="$GUI_PRE" cmd="$GUI_CMD"
  [ "$IDLE" = 1 ] && { pre=""; cmd="$IDLE_CMD"; }
  stop_progress_page
  docker exec "$NAME" bash -c "$ENV && $pre echo \$\$ > $GUI_PIDFILE && exec > >(tee -a /data/logs/gui_$gstamp.log) 2>&1 && exec $cmd" &
  local rc=0
  wait $! || rc=$?
  return $rc
}
start_stack
while true; do
  rc=0
  run_gui || rc=$?
  case "$rc" in
    76) say "Restarting the GUI (asked from the GUI)"
        # ~/ws is installed as copies: build it again so a GUI restart runs the code pulled since (seconds)
        WS_BUILT_FOR=""; build_ws "$R_IMAGE" || echo "GUI workspace build failed: starting the GUI as it was" ;;
    75) # fake robot: the container is restarted too, so a new base (UR7E_EE_CONFIG) reaches the robot model
        # the GUI switched the robot: its choice wins over --fake / --robot-ip from now on
        [ "$(cat "$ROBOT_FILE" 2>/dev/null || true)" != "$ROBOT_SEEN" ] && OVERRIDE=0
        if [ -f "$STACK_OFF_FILE" ]; then say "Stopping the robot stack (asked from the GUI; up to ~45 s)"
        else say "Restarting the robot stack and the GUI (asked from the GUI; up to ~45 s)"; fi
        docker stop "$NAME" >/dev/null 2>&1 || true
        stop_zed
        for _ in $(seq 1 40); do docker ps -a --format '{{.Names}}' | grep -qx "$NAME" || break; sleep 1; done
        start_stack ;;
    *) exit "$rc" ;;
  esac
done
