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
4. The gripper hardware is activated. It connects, resets and calibrates (the fingers move once).
5. The gripper controllers start.

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
