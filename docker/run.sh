#!/usr/bin/env bash
# Starts the container (first terminal). Open more terminals with:
#   docker exec -it ur7e_robotiq bash
#
# --rm means every start is a fresh container, so no old ROS processes
# are left behind between sessions. Files you want to keep (like the
# robot calibration) go in /data, which is ~/ur7e_data on the host.
set -e
mkdir -p "$HOME/ur7e_data"
xhost +local:root >/dev/null 2>&1 || true
docker run -it --rm \
  --name ur7e_robotiq \
  --net=host \
  -e ROS_DOMAIN_ID="${UR7E_ROS_DOMAIN_ID:-42}" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/ur7e_data:/data" \
  -v "$HOME/ws:/ws" \
  ur7e_robotiq:jazzy bash
