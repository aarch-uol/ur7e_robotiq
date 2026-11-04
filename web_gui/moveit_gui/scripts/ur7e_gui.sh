#!/usr/bin/env bash
# One command for the UR7e + MoveIt GUI:
#   1. rebuilds the ur7e_robotiq:jazzy image if ~/ur7e_robotiq changed since it was built
#   2. builds the GUI workspace (~/ws: moveit_gui + pymoveit2)
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
#
# Base = what is on the flange, a fixed part of the robot model the GUI cannot edit (parts go on top of it):
#   ft_gripper (default)  F/T sensor + Robotiq gripper     ft        F/T sensor only (your own tool on it)
#   gripper               Robotiq gripper on tool0          none      the bare arm
# Kept in ~/ur7e_data/config/moveit_gui/base; SETUP > Setups switches it too (and restarts the stack).
#
# Power on / Release brakes work from any browser that has control (after "I can see the robot");
# MOVEIT_GUI_LOCAL_POWER_ONLY=1 ur7e_gui.sh limits them to a browser on this PC (and the pendant).
# Saved data (programs, TCPs, fixtures, payload/vial results, logs) goes to ~/ur7e_data.
set -eo pipefail

ROBOT_IP=10.6.203.101
MODE=web
FAKE=0
PORT=8180
# Our own ROS domain: other PCs on the lab network run ROS on the default domain 0 (e.g. a Gazebo UR sim on
# 10.6.203.36), and a second /controller_manager on the same domain breaks controller switching.
# Anything that must talk to this stack (policy runner, ros2 CLI) needs  export ROS_DOMAIN_ID=42  too.
DOMAIN=${UR7E_ROS_DOMAIN_ID:-42}
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --robot-ip) ROBOT_IP="$2"; shift 2 ;;
    --desktop)  MODE=desktop; shift ;;
    --fake)     FAKE=1; shift ;;
    --port)     PORT="$2"; shift 2 ;;
    --base)     NEW_BASE="$2"; shift 2 ;;
    --)         shift; LAUNCH_ARGS=("$@"); break ;;
    -h|--help)  sed -n 2,24p "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)"; exit 1 ;;
  esac
done

IMAGE=ur7e_robotiq:jazzy
NAME=${UR7E_CONTAINER:-ur7e_robotiq}
REPO="$HOME/ur7e_robotiq"
WS="$HOME/ws"
DATA="$HOME/ur7e_data"
mkdir -p "$DATA/logs"
say() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 1. image
image_time=$(docker image inspect "$IMAGE" --format '{{.Created}}' 2>/dev/null || true)
if [ -z "$image_time" ]; then
  say "Image $IMAGE missing: building it (takes a while)"
  "$REPO/docker/build.sh"
else
  image_epoch=$(date -d "$image_time" +%s)
  src_epoch=$(find "$REPO" -path "$REPO/.git" -prune -o -name __pycache__ -prune -o -type f -printf '%T@\n' \
              | sort -n | tail -1 | cut -d. -f1)
  if [ "${src_epoch:-0}" -gt "$image_epoch" ]; then
    say "~/ur7e_robotiq changed since the image was built: rebuilding $IMAGE"
    "$REPO/docker/build.sh"
  fi
fi

# ---------------------------------------------------------------- 2. GUI workspace
say "Building the GUI workspace (~/ws)"
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$WS:/ws" -w /ws "$IMAGE" \
  bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select pymoveit2 moveit_gui' \
  > "$DATA/logs/ws_build.log" 2>&1 || { tail -30 "$DATA/logs/ws_build.log"; exit 1; }

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
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start_stack() {
  xhost +local:root >/dev/null 2>&1 || true
  BASE=$(read_base)
  stamp=$(date +%Y%m%d_%H%M%S)
  if [ "$FAKE" = 1 ]; then
    STACK='exec sleep infinity'
  else
    # exec: ros2 launch is the container's main process (under --init), so the stop signal reaches it directly.
    # The log goes through a process substitution for the same reason (a `| tee` pipeline would keep bash in between).
    STACK="exec > >(tee /data/logs/launch_$stamp.log) 2>&1; exec ros2 launch ur7e_robotiq_bringup gui.launch.py robot_ip:=$ROBOT_IP ${LAUNCH_ARGS[*]}"
  fi
  docker run -d --rm --init --stop-signal SIGINT --stop-timeout 30 --name "$NAME" --net=host -e ROS_DOMAIN_ID="$DOMAIN" -e UR7E_EE_CONFIG="$BASE" $(parts_env) \
    -e MOVEIT_GUI_LOCAL_POWER_ONLY="${MOVEIT_GUI_LOCAL_POWER_ONLY:-}" \
    -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$DATA:/data" -v "$WS:/ws" \
    "$IMAGE" bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash && $STACK" >/dev/null

  ENV='source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash && source /ws/install/setup.bash && export XDG_CONFIG_HOME=/data/config'

  if [ "$FAKE" = 0 ]; then
    say "Robot stack starting (robot $ROBOT_IP, base $BASE). Log: ~/ur7e_data/logs/launch_$stamp.log"
    ok=0
    for _ in $(seq 1 45); do
      docker ps --format '{{.Names}}' | grep -qx "$NAME" || { echo "The robot stack exited:"; tail -30 "$DATA/logs/launch_$stamp.log"; return 1; }
      if docker exec "$NAME" bash -c "$ENV && timeout 5 ros2 control list_controllers" 2>/dev/null | grep -qE "^joint_state_broadcaster +[^ ]+ +active"; then ok=1; break; fi
      sleep 2
    done
    if [ "$ok" = 1 ]; then
      say "Controllers are up"
    else
      echo "Controllers not up after 90 s: starting the GUI anyway (it shows what is missing)."
      echo "Last launch log lines:"; tail -15 "$DATA/logs/launch_$stamp.log"
    fi
    echo "    On the pendant: load and PLAY the External Control program, unless you passed headless_mode:=true."
  fi
}

# ---------------------------------------------------------------- 4. GUI
GUI_ARGS="--programs-dir /data/programs --policies-dir /data/policies"
[ "$FAKE" = 1 ] && GUI_ARGS="$GUI_ARGS --fake"
if [ "$FAKE" = 1 ]; then
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
  say "Web GUI: http://$HOST:$PORT/   (Ctrl-C here stops the GUI and the robot stack)"
  GUI_CMD="ros2 run moveit_gui moveit_gui_web --host $HOST --port $PORT $GUI_ARGS"
else
  say "Desktop GUI (closing the window stops the robot stack)"
  GUI_CMD="ros2 run moveit_gui moveit_gui $GUI_ARGS"
fi
# In the background + wait, so Ctrl-C reaches the cleanup at once. The GUI records its pid and is exec'd, so
# cleanup (stop_gui) can signal it by itself before the container is stopped.
# Its output also goes to ~/ur7e_data/logs/gui_<time>.log (what the GUI did after a restart: F/T bias, gains, ...).
run_gui() {
  local gstamp; gstamp=$(date +%Y%m%d_%H%M%S)
  say "GUI log: ~/ur7e_data/logs/gui_$gstamp.log"
  docker exec "$NAME" bash -c "$ENV && $GUI_PRE echo \$\$ > $GUI_PIDFILE && exec > >(tee -a /data/logs/gui_$gstamp.log) 2>&1 && exec $GUI_CMD" &
  local rc=0
  wait $! || rc=$?
  return $rc
}
start_stack
while true; do
  rc=0
  run_gui || rc=$?
  case "$rc" in
    76) say "Restarting the GUI (asked from the GUI)" ;;
    75) # fake robot: the container is restarted too, so a new base (UR7E_EE_CONFIG) reaches the robot model
        say "Restarting the robot stack and the GUI (asked from the GUI; up to ~45 s)"
        docker stop "$NAME" >/dev/null 2>&1 || true
        for _ in $(seq 1 40); do docker ps -a --format '{{.Names}}' | grep -qx "$NAME" || break; sleep 1; done
        start_stack ;;
    *) exit "$rc" ;;
  esac
done
