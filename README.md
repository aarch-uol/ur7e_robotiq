# UR7e + Robotiq 2F-85 ROS 2 Integration

ROS 2 **Jazzy** bringup and MoveIt configuration for a **UR7e** arm with a **Robotiq 2F-85** gripper.

## Table of Contents
- [About](#about)
- [Setup](#setup)
  - [Robot Setup](#robot-setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Launch Files](#launch-files)
- [Troubleshooting](#troubleshooting)

## About

The gripper is plugged into the robot's **wrist connector**, so no USB adapter or extra cable is needed. One launch file starts the arm driver, the gripper driver, MoveIt and RViz.

| Package | Contents |
|---|---|
| `ur7e_robotiq_bringup` | Launch files, combined arm + gripper model, gripper helper scripts |
| `ur7e_robotiq_moveit_config` | MoveIt config with `ur_manipulator` (arm) and `gripper` planning groups |

Robotiq's driver needs a small patch (`patches/robotiq_driver_tcp_transport.patch`) to reach the gripper through the robot. The steps below apply it. [Why?](docs/HOW_IT_WORKS.md#why-the-robotiq-driver-is-patched)

## Setup

### Robot Setup

Configure the teach pendant once: URCaps, network, tool I/O and the External Control program. Follow **[docs/ROBOT_SETUP.md](docs/ROBOT_SETUP.md)** carefully. The gripper won't work reliably if a setting is wrong.

### Local Machine Installation

1. **Install ROS 2 Jazzy** (Ubuntu 24.04) by following the [official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including `ros-dev-tools`.

2. **Clone this repository into a workspace.** A new workspace or one you already have both work ([notes for existing workspaces](docs/INSTALL.md#existing-workspace)).
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/aarch-uol/ur7e_robotiq.git
   ```

3. **Clone Robotiq's driver and apply the patch:**
   ```bash
   git clone https://github.com/robotiq/ros.git robotiq_ros
   cd robotiq_ros
   git checkout 8d7b8412ad685ffe1db5719da6e8fce6c1896e5e
   git submodule update --init --recursive
   git apply ../ur7e_robotiq/patches/robotiq_driver_tcp_transport.patch
   ```

4. **Install the dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src/ur7e_robotiq src/robotiq_ros/grippers --ignore-src --rosdistro jazzy -y
   ```

5. **Build:**
   ```bash
   colcon build --symlink-install --packages-up-to ur7e_robotiq_bringup
   ```

6. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

> **Shortcut:** after step 2, `~/ros2_ws/src/ur7e_robotiq/scripts/setup_workspace.sh ~/ros2_ws` runs steps 3–5 for you.

### Docker Container Installation

The image contains ROS 2, all dependencies and the built workspace. Nothing else is installed on your machine. It's built with the same `scripts/setup_workspace.sh`.

1. **Clone this repository:**
   ```bash
   git clone https://github.com/aarch-uol/ur7e_robotiq.git
   cd ur7e_robotiq
   ```

2. **Build the image:**
   ```bash
   ./docker/build.sh
   ```

3. **Run the container:**
   ```bash
   ./docker/run.sh
   ```
   Each run starts a fresh container. Files in `/data` are kept in `~/ur7e_data` on your machine.

4. **Open more shells inside the container:**
   ```bash
   docker exec -it ur7e_robotiq bash
   ```

## Test the Setup

1. **Extract your robot's calibration** (once). Without it, the tool pose in ROS is off by a few millimetres. In Docker, save it to `/data/ur7e_calibration.yaml` instead.
   ```bash
   export ROBOT_IP=xxx.xxx.xxx.xxx    # your robot's IP
   ros2 launch ur_calibration calibration_correction.launch.py \
     robot_ip:=$ROBOT_IP target_filename:=$HOME/ur7e_calibration.yaml
   ```

2. **Start the drivers, MoveIt and RViz:**
   ```bash
   ros2 launch ur7e_robotiq_bringup moveit.launch.py robot_ip:=$ROBOT_IP \
     kinematics_params_file:=$HOME/ur7e_calibration.yaml
   ```

3. **Press ▶ Play** on the External Control program on the teach pendant.

4. **Move the robot in RViz.** In the MotionPlanning panel, set velocity/acceleration scaling to `0.1`, pick a planning group and goal, then click **Plan & Execute**:
   - `ur_manipulator`: `ready` (arm in front, gripper pointing down), `home`, `up`, or drag the marker
   - `gripper`: `open`, `closed`

5. **Or from the terminal:**
   ```bash
   ros2 run ur7e_robotiq_bringup gripper.sh close        # open, close, or a position 0.0-0.79
   ros2 run ur7e_robotiq_bringup moveit_test.py ur_manipulator wrist_3_joint 1.0
   ```

## Launch Files

| Command | Starts |
|---|---|
| `ros2 launch ur7e_robotiq_bringup moveit.launch.py` | Drivers + MoveIt + RViz |
| `ros2 launch ur7e_robotiq_bringup drivers.launch.py` | Arm and gripper drivers only |
| `ros2 launch ur7e_robotiq_moveit_config move_group.launch.py` | MoveIt only |
| `ros2 launch ur7e_robotiq_moveit_config moveit_rviz.launch.py` | RViz only |

Arguments: `robot_ip` (required), `kinematics_params_file`, `rviz:=false`, `headless_mode`, `tool_tcp_port` (default `54321`).

## Troubleshooting

- **Arm doesn't move (`CONTROL_FAILED`):** External Control isn't playing. Press Play on the pendant again.
- **Gripper log repeats `TCP serial bridge closed the connection`:** something on the robot is also using the gripper port. Check the URCaps and tool I/O in [docs/ROBOT_SETUP.md](docs/ROBOT_SETUP.md).

More in **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)**. To see how the pieces fit together, read **[docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md)**.
