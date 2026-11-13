# ZED camera driver (zed-ros2-wrapper) for a Jetson with JetPack 6 (L4T 36.x, Ubuntu 22.04) and the ZED SDK on the host.
#
# The image holds ROS 2 Humble and the wrapper only. The ZED SDK, CUDA, TensorRT and the Jetson's own libraries
# stay on the host and are mounted read-only when the container runs (docker/zed/build.sh, ur7e_gui.sh), so the
# image always matches the host's SDK and nothing is installed on the host. Humble because the ZED SDK for JetPack 6
# is built for Ubuntu 22.04; the GUI (Jazzy) reads its topics over DDS.
#
# Build on the Jetson:  docker/zed/build.sh
FROM ros:humble-ros-base

ARG WRAPPER_TAG=v5.5.0
ENV DEBIAN_FRONTEND=noninteractive

# what the ZED SDK needs from Ubuntu (the Jetson's own libraries come from the host)
RUN apt-get update && apt-get install -y --no-install-recommends \
      libturbojpeg libusb-1.0-0 libv4l-0 libegl1 libgles2 libarchive13 libpng16-16 libjpeg8 libudev1 \
      libx11-6 libxml2 libicu70 wget git python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# where the host's libraries are found at run time (entry.sh links TensorRT and cuDNN into /opt/hostlibs)
RUN mkdir -p /opt/hostlibs && ln -s /usr/local/cuda-12.6 /usr/local/cuda \
 && printf '%s\n' /usr/local/zed/lib /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu/nvidia \
      /usr/lib/aarch64-linux-gnu/tegra-egl /opt/hostlibs > /etc/ld.so.conf.d/zed-host.conf

WORKDIR /zed_ws
RUN git clone --depth 1 -b ${WRAPPER_TAG} https://github.com/stereolabs/zed-ros2-wrapper.git src/zed-ros2-wrapper \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*

# build against the host's SDK and CUDA (bind-mounted for this step only, see build.sh); the libraries those
# link to are only needed at run time
RUN --mount=type=bind,from=zed,target=/usr/local/zed \
    --mount=type=bind,from=cuda,target=/usr/local/cuda-12.6 \
    . /opt/ros/humble/setup.sh && ldconfig \
 && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXE_LINKER_FLAGS="-Wl,--allow-shlib-undefined" -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
 && rm -rf build log

COPY entry.sh /entry.sh
ENTRYPOINT ["/entry.sh"]
CMD ["bash"]
