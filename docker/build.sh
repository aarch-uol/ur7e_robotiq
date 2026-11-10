#!/usr/bin/env bash
# Builds the robot image for this PC's CPU:
#   amd64 (desktop PCs)  ur7e_robotiq:jazzy        from osrf/ros:jazzy-desktop
#   arm64 (Jetson)       ur7e_robotiq:jazzy-arm64  from ros:jazzy-ros-base + ros-jazzy-desktop (osrf/ros is amd64 only)
# UR7E_IMAGE=name:tag docker/build.sh  builds under another tag (e.g. for a test build).
set -e
cd "$(dirname "$0")/.."
case "$(uname -m)" in
  x86_64|amd64)
    TAG=${UR7E_IMAGE:-ur7e_robotiq:jazzy}
    docker build -f docker/Dockerfile -t "$TAG" . ;;
  aarch64|arm64)
    TAG=${UR7E_IMAGE:-ur7e_robotiq:jazzy-arm64}
    docker build -f docker/Dockerfile -t "$TAG" \
      --build-arg BASE_IMAGE=ros:jazzy-ros-base --build-arg EXTRA_APT=ros-jazzy-desktop . ;;
  *) echo "unsupported CPU $(uname -m): amd64 or arm64 only"; exit 1 ;;
esac
