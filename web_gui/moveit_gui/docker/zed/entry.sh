#!/bin/bash
# The host's libraries are mounted at run time (/host/lib = the host's /usr/lib/aarch64-linux-gnu, read-only).
# Link the ones the ZED SDK needs from there (TensorRT, cuDNN, DLA), so the image follows the host's JetPack.
for f in /host/lib/libnvinfer*.so.* /host/lib/libnvonnxparser*.so.* /host/lib/libcudnn*.so.* /host/lib/libnvcudla.so; do
  [ -e "$f" ] && ln -sf "$f" /opt/hostlibs/
done
ldconfig
export LD_LIBRARY_PATH=/opt/hostlibs:/usr/lib/aarch64-linux-gnu/tegra-egl${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}   # dlopen by file name
source /opt/ros/humble/setup.bash
source /zed_ws/install/setup.bash
exec "$@"
