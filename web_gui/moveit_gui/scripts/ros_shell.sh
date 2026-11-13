#!/usr/bin/env bash
# Your own ROS 2 programs on the robot PC (the Jetson), next to the robot stack started by ur7e_gui.sh.
# The host has no ROS 2: this starts a throwaway container from the robot's image with ROS 2 Jazzy, the robot's
# packages and pymoveit2, on the stack's ROS domain and network setting. The current folder is /work inside.
#
#   ros_shell.sh                    # a shell: ros2 topic list, python3 my_script.py, ...
#   ros_shell.sh my_script.py -x    # run one Python script (from the current folder) and exit
#
# Needs Docker (the docker group) and the image the launcher uses. It never touches the running stack.
set -eo pipefail

case "$(uname -m)" in
  aarch64|arm64) IMAGE=${UR7E_IMAGE:-ur7e_robotiq:jazzy-arm64} ;;
  *)             IMAGE=${UR7E_IMAGE:-ur7e_robotiq:jazzy} ;;
esac
STACK=${UR7E_CONTAINER:-ur7e_robotiq}
WS="$HOME/ws"

# The domain and network of the running stack (else 42, lab network)
DOMAIN=42; RANGE=SUBNET
if docker ps --format '{{.Names}}' | grep -qx "$STACK"; then
  while IFS='=' read -r k v; do
    case "$k" in
      ROS_DOMAIN_ID) DOMAIN=$v ;;
      ROS_AUTOMATIC_DISCOVERY_RANGE) RANGE=$v ;;
    esac
  done < <(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$STACK")
else
  echo "The robot stack ($STACK) is not running: using ROS domain $DOMAIN." >&2
fi
echo "ROS domain $DOMAIN, discovery $RANGE. This folder is /work." >&2

# ~/ur7e_robotiq's packages as the launcher built them (its overlay on the image), when there is one
OVERLAY=(); [ -f "$HOME/ur7e_data/overlay/install/setup.bash" ] && \
  OVERLAY=(-v "$HOME/ur7e_data/overlay:/data/overlay:ro" -v "$HOME/ur7e_robotiq:/ur7e_robotiq:ro")
SRC="source /opt/ros/jazzy/setup.bash; for f in /opt/ur7e_ws/install/setup.bash /data/overlay/install/setup.bash /ws/install/setup.bash; do [ -f \$f ] && source \$f; done"
TTY=(-i); [ -t 0 ] && [ -t 1 ] && TTY=(-it)
if [ $# -gt 0 ]; then
  CMD=(bash -c "$SRC; exec python3 \"\$@\"" ros_shell "$@")
else
  CMD=(bash -c "$SRC; echo 'ROS 2 Jazzy: ros2 topic list, python3 my_script.py, exit to leave.'; exec bash")
fi
# --ipc=host like the ZED container: without it the topics show but no messages arrive (ROS 2 on one PC uses
# shared memory; tested on the Jetson)
exec docker run --rm "${TTY[@]}" --net=host --ipc=host -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_AUTOMATIC_DISCOVERY_RANGE="$RANGE" \
  -v "$PWD:/work" -w /work -v "$WS:/ws:ro" "${OVERLAY[@]}" "$IMAGE" "${CMD[@]}"
