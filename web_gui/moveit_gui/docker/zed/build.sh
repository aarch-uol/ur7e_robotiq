#!/bin/bash
# Build the ZED camera image on a Jetson (JetPack 6 + ZED SDK installed): moveit_gui_zed:humble.
#   docker/zed/build.sh            # or UR7E_ZED_IMAGE=name:tag docker/zed/build.sh
# Needs no sudo: the SDK, CUDA and the Jetson libraries are read from the host while building.
set -euo pipefail
cd "$(dirname "$0")"
IMAGE=${UR7E_ZED_IMAGE:-moveit_gui_zed:humble}
for d in /usr/local/zed/lib /usr/local/cuda-12.6 /usr/lib/aarch64-linux-gnu/nvidia; do
  [ -d "$d" ] || { echo "missing $d: this needs a Jetson with JetPack 6 and the ZED SDK"; exit 1; }
done
docker buildx build --load -t "$IMAGE" \
  --build-context zed=/usr/local/zed \
  --build-context cuda=/usr/local/cuda-12.6 \
  .
echo "built $IMAGE"
