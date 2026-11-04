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

The gripper is plugged into the robot's **wrist connector**. The ATI Gamma runs over Ethernet in its own controller manager, so a sensor/network failure does not take down the UR arm or gripper controller managers.

| Package | Contents |
|---|---|
| `ur7e_robotiq_bringup` | Launch files, combined arm + Gamma + gripper model, F/T and gripper drivers |
| `ur7e_robotiq_moveit_config` | MoveIt config with `ur_manipulator` (arm) and `gripper` planning groups |
| `topic_sensor_bridge` | ros2_control hardware interface that exposes the ATI wrench topic as `ati_fts_sensor/*` state interfaces |
| `ur7e_safety` | `wrench_watchdog`: zeroes the wrench target if its publisher dies mid-force (see [Wrench watchdog](#wrench-watchdog)) |

Robotiq uses the UR driver's supported tool-communication forwarder: the robot's wrist RS-485 port is exposed locally as `/tmp/ttyUR`. A small PTY compatibility patch lets the Robotiq driver open that device with POSIX termios because its bundled libserialport rejects `/dev/pts/*`. The ATI driver carries the activation fix in `patches/net_ft_gamma_runtime.patch`.

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
   The wrench is labelled with `ft_fts_link`. The combined robot model also publishes `ati_sensor` as a zero-offset alias of that same sensing frame.

## Launch Files

| Command | Starts |
|---|---|
| `ros2 launch ur7e_robotiq_bringup moveit.launch.py` | Drivers + MoveIt + RViz |
| `ros2 launch ur7e_robotiq_bringup admittance.launch.py` | Drivers + ATI Gamma + MoveIt through the admittance controller chain + RViz |
| `ros2 launch ur7e_robotiq_bringup gui.launch.py` | Drivers + MoveIt with every mode (position, admittance, Cartesian impedance) loaded; Position active |
| `ros2 launch ur7e_robotiq_bringup cartesian_impedance.launch.py` | Experimental direct-torque Cartesian impedance (controller left inactive unless `activate_controller:=true`) |
| `ros2 launch ur7e_robotiq_bringup drivers.launch.py` | Arm and gripper drivers only |
| `ros2 launch ur7e_robotiq_moveit_config move_group.launch.py` | MoveIt only |
| `ros2 launch ur7e_robotiq_moveit_config moveit_rviz.launch.py` | RViz only |

Arguments: `robot_ip` (required), `kinematics_params_file`, `rviz:=false`, `headless_mode`, `tool_tcp_port` (default `54321`), `load_ft_sensor`, `ft_ip_address` (default `192.168.1.1`), `ft_sampling_rate` (default `1000`) and `ft_hardware_biasing`.

### Wrench watchdog

`/admittance_controller/wrench_reference` and `/cartesian_impedance_controller/set_cartesian_wrench` (`geometry_msgs/WrenchStamped`) have no input timeout in the upstream controllers: if the program streaming the target (the GUI) crashes or is killed mid-force, the last force stays applied. `gui.launch.py`, `admittance.launch.py` and `cartesian_impedance.launch.py` therefore start `ur7e_safety/wrench_watchdog`. For each watched topic, if the last message was non-zero and nothing arrives for `timeout_s` (default 0.3 s), it publishes a zero wrench (same `frame_id`) 3 times and logs one warning. It publishes nothing during normal traffic or after a zero message. Parameters: `topics` (list), `timeout_s`, `zero_repeats` (3), `check_rate_hz` (50). Timing logic is unit tested: `cd ur7e_safety && python3 -m pytest test`.

Anything else that sends a wrench target (a policy, a script) must therefore stream it faster than every `timeout_s`, like the GUI does (50 Hz): a single non-zero message is zeroed after 0.3 s. Raise `timeout_s` for slower publishers.

Note the watchdog only helps while it is running. Controllers started by other launch files (`moveit.launch.py`, `drivers.launch.py`) do not use wrench targets.

### Parts from the GUI (optional URDF / SRDF include)

`ur7e_robotiq.urdf.xacro` and `ur7e_robotiq.srdf.xacro` include one extra file each when an environment variable
names it: `UR7E_USER_PARTS_URDF` (fixed links on the robot's links) and `UR7E_USER_PARTS_SRDF` (their
`disable_collisions` pairs). The MoveIt GUI writes both (SETUP › Parts, under
`~/ur7e_data/config/moveit_gui/parts/generated/`) and `ur7e_gui.sh` sets the variables when the files exist. Without
them the robot model is exactly as before. The variables reach every launch that runs the xacro (driver, MoveIt,
Servo, RViz), because they are set on the container.

### Admittance mode

The admittance launch uses this position-control chain:

`MoveIt -> admittance_trajectory_controller -> admittance_controller -> UR joint position interfaces`

The ATI Gamma still runs in the separate `/ft/controller_manager`. Its wrench topic is bridged into the UR controller manager as `ati_fts_sensor/*` state interfaces; this name intentionally differs from the UR driver's built-in `tcp_fts_sensor`.

Start it with:

```bash
ros2 launch ur7e_robotiq_bringup admittance.launch.py \
  robot_ip:=$ROBOT_IP \
  kinematics_params_file:=$HOME/ur7e_calibration.yaml
```

The migration default enables compliance only on tool0 X. The gravity-compensation payload and CoG are deliberately set to zero until the real Robotiq + adapter + tool payload is measured. Calibrate those values before enabling vertical or rotational compliance.

## Troubleshooting

- **Arm doesn't move (`CONTROL_FAILED`):** External Control isn't playing. Press Play on the pendant again.
- **Gripper communication is intermittent:** confirm the Robotiq URCap is disabled, Tool I/O is controlled by User, and only the RS485 forwarding URCap owns the wrist serial port. Check [docs/ROBOT_SETUP.md](docs/ROBOT_SETUP.md).

More in **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)**. To see how the pieces fit together, read **[docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md)**.
