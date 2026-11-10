# UR7e + ATI Gamma + Robotiq 2F-85 ROS 2 Integration

ROS 2 **Jazzy** bringup and MoveIt configuration for a **UR7e** arm with an **ATI Gamma** F/T sensor and a **Robotiq 2F-85** gripper.

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

| Package | Contents |
|---|---|
| `ur7e_robotiq_bringup` | Launch files, robot model, drivers |
| `ur7e_robotiq_moveit_config` | MoveIt config: `ur_manipulator` (arm) and `gripper` groups |
| `topic_sensor_bridge` | Feeds the ATI reading to the arm's controllers |
| `ur7e_safety` | Wrench watchdog (see below) |

The gripper is plugged into the robot's wrist connector. The ATI Gamma runs over Ethernet in its own controller
manager, so a sensor problem does not stop the arm or the gripper. Details: [docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md).

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

3. **Set up the external drivers, install dependencies and build:**
   ```bash
   ~/ros2_ws/src/ur7e_robotiq/scripts/setup_workspace.sh ~/ros2_ws
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

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
   On a desktop PC this builds `ur7e_robotiq:jazzy`. On an NVIDIA Jetson (arm64) it builds `ur7e_robotiq:jazzy-arm64`
   from `ros:jazzy-ros-base`, because `osrf/ros` has no arm64 image. That takes about an hour.

   The expensive part (third-party drivers, rosdep) is a layer made only from the files `docker/deps_hash.sh`
   lists: `docker/`, `scripts/setup_workspace.sh`, `patches/` and the `package.xml` files. Changing anything else
   rebuilds only the last, short step. The image is labelled `ur7e_robotiq.deps=<that hash>`; moveit_gui's
   `ur7e_gui.sh` rebuilds it only when the hash changes and builds this repo's packages from the checkout at each
   start (an overlay in `~/ur7e_data/overlay`), so launch, yaml and script changes need no new image there.

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

6. **Check the ATI Gamma:**
   ```bash
   ros2 control list_hardware_components -c /ft/controller_manager
   ros2 topic echo /ft/force_torque_sensor_broadcaster/wrench --once
   ```

## Launch Files

```bash
ros2 launch ur7e_robotiq_bringup <launch file> robot_ip:=$ROBOT_IP
```

Then press Play on the External Control program on the pendant.

| Launch file | Starts |
|---|---|
| `ur7e_gripper_moveit.launch.py` | Arm, gripper, MoveIt, RViz |
| `moveit.launch.py` | Same, plus the ATI F/T sensor |
| `gui.launch.py` | Everything the MoveIt GUI needs |
| `admittance.launch.py` | MoveIt with admittance (force) control |
| `cartesian_impedance.launch.py` | Cartesian impedance (experimental) |
| `drivers.launch.py` | Drivers only, no MoveIt |

Common options: `kinematics_params_file:=$HOME/ur7e_calibration.yaml`, `rviz:=false`, `load_ft_sensor:=false`.

**`ur7e_gripper_moveit.launch.py`:** the ATI stays in the robot model (MoveIt avoids it), but its driver is not
started, so the sensor does not need to be connected.

### End-effector bases

Pick what is on the flange with `UR7E_EE_CONFIG` (or `ur7e_gui.sh --base <name>`):

| Base | On the flange |
|---|---|
| `ft_gripper` (default) | ATI Gamma + Robotiq 2F-85 |
| `gripper` | Robotiq 2F-85 only |
| `ft` | ATI Gamma only |
| `none` | Nothing |

### Admittance

Only the tool X axis is compliant by default. Calibrate the payload (the MoveIt GUI does this) before making more
axes compliant.

### Wrench watchdog

`gui`, `admittance` and `cartesian_impedance` start a watchdog: if the program sending a force stops, the force is
set to zero after 0.3 s. Anything that sends forces must keep sending them faster than that.

### Parts from the MoveIt GUI

Parts added in the GUI (SETUP › Parts) are added to the robot model by `ur7e_gui.sh`. Without the GUI nothing
changes.

More detail on all of these: [docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md).

## Troubleshooting

- **Arm doesn't move (`CONTROL_FAILED`):** External Control isn't playing. Press Play on the pendant again.
- **Gripper communication is intermittent:** confirm the Robotiq URCap is disabled, Tool I/O is controlled by User, and only the RS485 forwarding URCap owns the wrist serial port. Check [docs/ROBOT_SETUP.md](docs/ROBOT_SETUP.md).

More in **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)**. To see how the pieces fit together, read **[docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md)**.
