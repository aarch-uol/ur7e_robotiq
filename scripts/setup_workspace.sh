#!/usr/bin/env bash
# Does the README "Build" step in one go: workspace with this repo + the
# patched Robotiq driver, dependencies, build. Used by the Docker image.
#
#   scripts/setup_workspace.sh              # workspace in ~/ros2_ws
#   scripts/setup_workspace.sh /some/dir    # workspace somewhere else
#
# Safe to run again: it skips steps that are already done.
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$HOME/ros2_ws}"
ROBOTIQ_URL="https://github.com/robotiq/ros.git"
ROBOTIQ_COMMIT="8d7b8412ad685ffe1db5719da6e8fce6c1896e5e"
ROBOTIQ_DIR="$WS/src/robotiq_ros"
PATCH="$REPO_DIR/patches/robotiq_driver_tcp_transport.patch"

source /opt/ros/jazzy/setup.bash

echo "==> Workspace: $WS"
mkdir -p "$WS/src"

case "$REPO_DIR/" in
  "$(realpath "$WS")"/src/*) ;; # Cloned inside the workspace already.
  *)
    echo "==> Linking this repo into $WS/src"
    ln -sfn "$REPO_DIR" "$WS/src/ur7e_robotiq"
    ;;
esac

if [ ! -d "$ROBOTIQ_DIR/.git" ]; then
  echo "==> Cloning Robotiq ROS driver at $ROBOTIQ_COMMIT"
  git clone "$ROBOTIQ_URL" "$ROBOTIQ_DIR"
  git -C "$ROBOTIQ_DIR" checkout --quiet "$ROBOTIQ_COMMIT"
  git -C "$ROBOTIQ_DIR" submodule update --init --recursive
fi

if git -C "$ROBOTIQ_DIR" apply --reverse --check "$PATCH" 2>/dev/null; then
  echo "==> TCP transport patch already applied"
else
  echo "==> Applying TCP transport patch to robotiq_driver"
  git -C "$ROBOTIQ_DIR" apply "$PATCH"
fi

echo "==> Installing dependencies with rosdep"
rosdep update --rosdistro jazzy >/dev/null || true
rosdep install -y --rosdistro jazzy --ignore-src \
  -t buildtool -t buildtool_export -t build -t build_export -t exec \
  --from-paths "$REPO_DIR" "$ROBOTIQ_DIR/grippers"

echo "==> Building"
cd "$WS"
colcon build --symlink-install --packages-up-to ur7e_robotiq_bringup

echo
echo "Done. In every new terminal run:"
echo "  source $WS/install/setup.bash"
