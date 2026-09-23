FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      libserialport-dev \
      python3-colcon-common-extensions \
      ros-jazzy-moveit \
      ros-jazzy-parallel-gripper-controller \
      ros-jazzy-ur \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/ur7e_robotiq
RUN apt-get update \
    && /opt/ur7e_robotiq/scripts/setup_workspace.sh /opt/ur7e_ws \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ur7e_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /opt/ur7e_ws
CMD ["bash"]
