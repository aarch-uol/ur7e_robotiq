#!/usr/bin/env bash
# Does the README "Build" step in one go: workspace with this repo + the
# Robotiq PTY support, dependencies, build. Used by the Docker image.
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
ROBOTIQ_PATCH="$REPO_DIR/patches/robotiq_driver_pty_transport.patch"
NET_FT_URL="https://github.com/gbartyzel/ros2_net_ft_driver.git"
NET_FT_COMMIT="dbe9916c7221247810cd21b741acd48de458bac2"
NET_FT_DIR="$WS/src/ros2_net_ft_driver"
NET_FT_PATCH="$REPO_DIR/patches/net_ft_gamma_runtime.patch"
CARTESIAN_IMPEDANCE_URL="https://github.com/mrceki/Cartesian-Impedance-Controller.git"
CARTESIAN_IMPEDANCE_COMMIT="0a1c635a921b931ae1e9d2f3357f4b34bf28b4ed"
CARTESIAN_IMPEDANCE_DIR="$WS/src/cartesian_impedance_controller"
CARTESIAN_IMPEDANCE_PATCH="$REPO_DIR/patches/cartesian_impedance_controller_ur6dof.patch"
SPACEVECALG_COMMIT="c0885d038b8b0cfac320c472f6d837bdd16d0312"
RBDYN_COMMIT="9888f20c7f79a10ffc61a74901caf982c42361da"
MC_RBDYN_URDF_COMMIT="c7eb72b46b3753ed3542793778e0cc59496574ef"

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

if git -C "$ROBOTIQ_DIR" apply --reverse --check "$ROBOTIQ_PATCH" 2>/dev/null; then
  echo "==> Robotiq PTY transport patch already applied"
else
  echo "==> Applying Robotiq PTY transport patch"
  git -C "$ROBOTIQ_DIR" apply "$ROBOTIQ_PATCH"
fi

if [ ! -d "$NET_FT_DIR/.git" ]; then
  echo "==> Cloning Net F/T driver at $NET_FT_COMMIT"
  git clone "$NET_FT_URL" "$NET_FT_DIR"
  git -C "$NET_FT_DIR" checkout --quiet "$NET_FT_COMMIT"
fi

if git -C "$NET_FT_DIR" apply --reverse --check "$NET_FT_PATCH" 2>/dev/null; then
  echo "==> ATI Gamma runtime patch already applied"
else
  echo "==> Applying ATI Gamma runtime patch"
  git -C "$NET_FT_DIR" apply "$NET_FT_PATCH"
fi

if [ ! -d "$CARTESIAN_IMPEDANCE_DIR/.git" ]; then
  echo "==> Cloning Cartesian impedance controller at $CARTESIAN_IMPEDANCE_COMMIT"
  git clone "$CARTESIAN_IMPEDANCE_URL" "$CARTESIAN_IMPEDANCE_DIR"
  git -C "$CARTESIAN_IMPEDANCE_DIR" checkout --quiet "$CARTESIAN_IMPEDANCE_COMMIT"
fi

if git -C "$CARTESIAN_IMPEDANCE_DIR" apply --reverse --check "$CARTESIAN_IMPEDANCE_PATCH" 2>/dev/null; then
  echo "==> UR 6-DOF Cartesian impedance patch already applied"
else
  echo "==> Applying UR 6-DOF Cartesian impedance patch"
  git -C "$CARTESIAN_IMPEDANCE_DIR" apply "$CARTESIAN_IMPEDANCE_PATCH"
fi

# These CMake libraries are required by the Cartesian controller but are not
# declared in its package.xml and are not available from Ubuntu/ROS apt repos.
# Install pinned revisions only when their CMake package files are absent.
if [ ! -f /usr/local/lib/cmake/SpaceVecAlg/SpaceVecAlgConfig.cmake ] || \
   [ ! -f /usr/local/lib/cmake/RBDyn/RBDynConfig.cmake ] || \
   [ ! -f /usr/local/lib/cmake/mc_rbdyn_urdf/mc_rbdyn_urdfConfig.cmake ]; then
  echo "==> Building Cartesian impedance kinematics dependencies"
  DEPENDS_DIR="$WS/depends"
  mkdir -p "$DEPENDS_DIR"
  touch "$DEPENDS_DIR/COLCON_IGNORE"

  build_cmake_dependency() {
    local name="$1"
    local url="$2"
    local commit="$3"
    local source_dir="$DEPENDS_DIR/$name"

    if [ ! -d "$source_dir/.git" ]; then
      git clone --recursive "$url" "$source_dir"
      git -C "$source_dir" checkout --quiet "$commit"
      git -C "$source_dir" submodule update --init --recursive
    fi

    cmake -S "$source_dir" -B "$source_dir/build" \
      -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF -DBUILD_TESTING=OFF
    cmake --build "$source_dir/build" --parallel
    if [ "$(id -u)" -eq 0 ]; then
      cmake --install "$source_dir/build"
    else
      sudo cmake --install "$source_dir/build"
    fi
  }

  build_cmake_dependency SpaceVecAlg \
    https://github.com/jrl-umi3218/SpaceVecAlg.git "$SPACEVECALG_COMMIT"
  build_cmake_dependency RBDyn \
    https://github.com/jrl-umi3218/RBDyn.git "$RBDYN_COMMIT"
  build_cmake_dependency mc_rbdyn_urdf \
    https://github.com/jrl-umi3218/mc_rbdyn_urdf.git "$MC_RBDYN_URDF_COMMIT"

  if [ "$(id -u)" -eq 0 ]; then
    ldconfig
  else
    sudo ldconfig
  fi
fi

echo "==> Installing dependencies with rosdep"
rosdep update --rosdistro jazzy >/dev/null || true
rosdep install -y --rosdistro jazzy --ignore-src \
  -t buildtool -t buildtool_export -t build -t build_export -t exec \
  --from-paths "$REPO_DIR" "$ROBOTIQ_DIR/grippers" "$NET_FT_DIR" "$CARTESIAN_IMPEDANCE_DIR"

echo "==> Building"
cd "$WS"
colcon build --symlink-install --packages-up-to ur7e_robotiq_bringup

echo
echo "Done. In every new terminal run:"
echo "  source $WS/install/setup.bash"
