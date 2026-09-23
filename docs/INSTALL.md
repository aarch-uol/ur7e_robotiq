# Installation notes

The [README](../README.md#setup) covers the standard installation. This page covers adding the repo to a workspace you already have, and updating.

## Existing workspace

Use your workspace path instead of `~/ros2_ws` in the README steps. The rest works unchanged:
- `--packages-up-to ur7e_robotiq_bringup` only builds this repo and what it needs. Your other packages are left alone.
- `rosdep install` only looks at the paths given.

Check these first:

| You already have... | Do this |
|---|---|
| Robotiq's driver (`robotiq/ros`) in `src/` | Skip cloning it and apply the patch to your copy: `git -C <your robotiq dir> apply <path to>/ur7e_robotiq/patches/robotiq_driver_tcp_transport.patch`. If it doesn't apply, your copy is a different version: check out `8d7b8412` as in the README. |
| The UR driver built from source | Nothing to do. It's used instead of the apt version. |
| Another package named `ur7e_robotiq_bringup` or `ur7e_robotiq_moveit_config` | Remove or rename it. colcon won't build two packages with the same name. |

Without the patch the gripper can't connect. The unpatched driver ignores the `tcp_host` parameter and looks for a USB serial port instead.

## Updating

```bash
cd ~/ros2_ws/src/ur7e_robotiq && git pull
cd ~/ros2_ws && colcon build --symlink-install --packages-up-to ur7e_robotiq_bringup
```

If the pull changed `patches/robotiq_driver_tcp_transport.patch`, re-apply it to a clean Robotiq checkout, then build again:
```bash
cd ~/ros2_ws/src/robotiq_ros
git checkout -- . && git clean -fd grippers/robotiq_driver    # discards local changes in robotiq_ros
git apply ../ur7e_robotiq/patches/robotiq_driver_tcp_transport.patch
```

With Docker, run `git pull` and then `./docker/build.sh` again.
