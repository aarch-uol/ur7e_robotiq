# How it works

## The big picture

```
                         ┌──────────────── drivers.launch.py ────────────────┐
                         │                                                   │
  MoveIt (move_group) ───┼─► /controller_manager            (official UR driver)
    │                    │     scaled_joint_trajectory_controller ──► arm joints
    │                    │                                                   │
    └────────────────────┼─► /robotiq_controller_manager    (gripper only)
                         │     robotiq_gripper_controller ──► robotiq_driver
                         │                                      │  /tmp/ttyUR
                         └──────────────────────────────────────┼────────────┘
                                                                ▼
                                 official UR tool forwarder ─► wrist ─► 2F-85
```

- **Two controller managers.** The arm and gripper run separately, so a gripper connection problem can never stop the arm.
- **One robot model.** `urdf/ur7e_robotiq.urdf.xacro` = the official UR7e model + the 2F-85 mounted on `tool0`. It's used for TF, RViz and MoveIt.
- **Both publish to `/joint_states`.** MoveIt sees all arm and gripper joints together.

## Why the Robotiq driver is patched

The UR driver runs its supported tool-communication forwarder and creates
`/tmp/ttyUR`. Robotiq's bundled libserialport rejects PTYs under
`/dev/pts/*`, so `patches/robotiq_driver_pty_transport.patch` replaces only
the serial-device opener with POSIX `open()`/termios. The Robotiq protocol,
controllers, and SDK exchange loop remain upstream. There is no direct
Robotiq TCP client.

## Tool-port ownership

The UR driver configures 24 V, 115200 baud, no parity, one stop bit, and the
1.5/3.5-character idle times. The Robotiq Grippers URCap must remain disabled;
only the RS485 forwarding URCap may own the wrist serial device.

## Startup order (drivers.launch.py)

1. The UR driver starts. The arm controllers load but stay **inactive** until you press **Play** on the pendant.
2. The gripper's controller manager starts with the gripper **not connected**.
3. `wait_for_gripper` waits for `/tmp/ttyUR` and for the gripper controller manager to load the hardware.
4. The gripper hardware is activated. It connects, resets and calibrates (the fingers move once). If activation fails it is retried every 2 s for up to 60 s (stops as soon as the component is `active`).
5. The gripper controllers start.

## Controller safety

- **Joint states:** every `joint_state_broadcaster` config (`ur_controllers_*.yaml`) lists only the six arm joints and `position/velocity/effort`. Without the list the broadcaster would also claim the F/T bridge's `ati_fts_sensor/*` interfaces, and an F/T dropout would deactivate it and stop `/joint_states`.
- **Wrench targets:** `ur7e_safety/wrench_watchdog` zeroes the wrench target when the publisher goes silent while non-zero (see README).

## MoveIt config

| Planning group | Joints | Controller |
|---|---|---|
| `ur_manipulator` | 6 arm joints | `scaled_joint_trajectory_controller` (FollowJointTrajectory) |
| `gripper` | `robotiq_85_left_knuckle_joint` (the other finger joints follow it) | `robotiq_gripper_controller` (ParallelGripperCommand) |

Named states:
- arm: `ready` (arm in front at shoulder_pan 90°, gripper pointing straight down, fingertips about 39 cm above the base), plus UR's own `home`, `up` and `test_configuration`
- gripper: `open` (0.0) and `closed` (0.79)

To add your own pose: move the arm there, read the joints with `ros2 topic echo /joint_states --once`, and add a `<group_state>` to `ur7e_robotiq.srdf.xacro` like `ready`.

Files in `ur7e_robotiq_moveit_config/config/`:
- `ur7e_robotiq.srdf.xacro`: groups, named states, which link pairs to skip in collision checks
- `moveit_controllers.yaml`: which ROS controller executes each group
- `kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml`: standard UR values

## Mounting offset

The gripper model is mounted directly on `tool0` with no offset. If you have a coupling plate between the robot and gripper, add its thickness in `ur7e_robotiq_bringup/urdf/ur7e_robotiq.urdf.xacro`:
```xml
<origin xyz="0 0 0.011" rpy="0 0 0"/>   <!-- e.g. 11 mm Robotiq coupling -->
```

## Tool communication and patches

Robotiq uses the UR driver's supported tool-communication forwarder: the robot's wrist RS-485 port is exposed locally as `/tmp/ttyUR`. A small PTY compatibility patch lets the Robotiq driver open that device with POSIX termios because its bundled libserialport rejects `/dev/pts/*`. The ATI driver carries the activation fix in `patches/net_ft_gamma_runtime.patch`.

## Wrench watchdog

`/admittance_controller/wrench_reference` and `/cartesian_impedance_controller/set_cartesian_wrench` (`geometry_msgs/WrenchStamped`) have no input timeout in the upstream controllers: if the program streaming the target (the GUI) crashes or is killed mid-force, the last force stays applied. `gui.launch.py`, `admittance.launch.py` and `cartesian_impedance.launch.py` therefore start `ur7e_safety/wrench_watchdog`. For each watched topic, if the last message was non-zero and nothing arrives for `timeout_s` (default 0.3 s), it publishes a zero wrench (same `frame_id`) 3 times and logs one warning. It publishes nothing during normal traffic or after a zero message. Parameters: `topics` (list), `timeout_s`, `zero_repeats` (3), `check_rate_hz` (50). Timing logic is unit tested: `cd ur7e_safety && python3 -m pytest test`.

Anything else that sends a wrench target (a policy, a script) must therefore stream it faster than every `timeout_s`, like the GUI does (50 Hz): a single non-zero message is zeroed after 0.3 s. Raise `timeout_s` for slower publishers.

Note the watchdog only helps while it is running. Controllers started by other launch files (`moveit.launch.py`, `drivers.launch.py`) do not use wrench targets.

## Parts from the GUI (optional URDF / SRDF include)

`ur7e_robotiq.urdf.xacro` and `ur7e_robotiq.srdf.xacro` include one extra file each when an environment variable
names it: `UR7E_USER_PARTS_URDF` (fixed links on the robot's links) and `UR7E_USER_PARTS_SRDF` (their
`disable_collisions` pairs). The MoveIt GUI writes both (SETUP › Parts, under
`~/ur7e_data/config/moveit_gui/parts/generated/`) and `ur7e_gui.sh` sets the variables when the files exist. Without
them the robot model is exactly as before. The variables reach every launch that runs the xacro (driver, MoveIt,
Servo, RViz), because they are set on the container.

## End-effector bases (`UR7E_EE_CONFIG`)

The URDF and SRDF xacro build one of four bases from `UR7E_EE_CONFIG` (xacro arg `ee_config`). Anything else is
rejected with an error.

| Base | On the flange | Controllers loaded besides Position |
|---|---|---|
| `ft_gripper` (default) | ATI Gamma, Robotiq 2F-85 on it | admittance, cartesian impedance; gripper + F/T managers |
| `gripper` | Robotiq 2F-85 coupling straight on `tool0` | cartesian impedance; gripper manager |
| `ft` | ATI Gamma only; your own tool on `ft_tool_mount_link` | admittance; F/T manager |
| `none` | nothing; your own tool on `tool0` | none |

The SRDF follows the same variable: the gripper group, its states and passive joints, and the collision pairs
of the hardware that is there. `gui.launch.py` reads it too and passes `load_ft_sensor` / `load_gripper` to
`drivers.launch.py`. `ft_gripper` gives exactly the URDF and SRDF from before this option.

`ur7e_gui.sh --base <name>` sets it, and it stays set for later starts (`~/ur7e_data/config/moveit_gui/base`). The
GUI switches it too, in SETUP › Setups. Tools you build in the GUI go on top of a base as parts. They are never
edited into the base itself.

## Admittance mode

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
