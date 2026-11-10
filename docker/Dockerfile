# docker/build.sh picks the base for this PC: osrf/ros:jazzy-desktop on amd64 (the default), and on arm64
# (Jetson) the official ros:jazzy-ros-base plus ros-jazzy-desktop, because osrf/ros is amd64 only.
ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}
ARG EXTRA_APT=""

RUN apt-get update && apt-get install -y --no-install-recommends ${EXTRA_APT} \
      git \
      libserialport-dev \
      python3-colcon-common-extensions \
      ros-jazzy-moveit \
      ros-jazzy-parallel-gripper-controller \
      ros-jazzy-realsense2-camera \
      ros-jazzy-ur \
      ros-jazzy-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# The dependencies: third-party drivers (cloned, patched, compiled) and rosdep from the package.xml files. Only
# the files docker/deps_hash.sh lists go into this layer, so it stays cached while the packages change. Keep the
# package.xml list in step with the packages (a new package's dependencies are still installed by the last step).
COPY scripts/setup_workspace.sh /opt/ur7e_robotiq/scripts/
COPY patches/ /opt/ur7e_robotiq/patches/
COPY topic_sensor_bridge/package.xml /opt/ur7e_robotiq/topic_sensor_bridge/
COPY ur7e_robotiq_bringup/package.xml /opt/ur7e_robotiq/ur7e_robotiq_bringup/
COPY ur7e_robotiq_moveit_config/package.xml /opt/ur7e_robotiq/ur7e_robotiq_moveit_config/
COPY ur7e_safety/package.xml /opt/ur7e_robotiq/ur7e_safety/
RUN apt-get update \
    && /opt/ur7e_robotiq/scripts/setup_workspace.sh --deps-only /opt/ur7e_ws \
    && rm -rf /var/lib/apt/lists/*

# This repo's packages, so the image also works on its own (docker/run.sh). moveit_gui's ur7e_gui.sh does not
# rebuild the image for them: it builds them from ~/ur7e_robotiq at each start into an overlay
# (~/ur7e_data/overlay) that is sourced after this workspace and wins over it.
COPY . /opt/ur7e_robotiq
RUN apt-get update \
    && /opt/ur7e_robotiq/scripts/setup_workspace.sh /opt/ur7e_ws \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ur7e_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /opt/ur7e_ws
# ros2 launch shuts down cleanly on SIGINT. Run with --init (docker/run.sh does).
STOPSIGNAL SIGINT
CMD ["bash"]
