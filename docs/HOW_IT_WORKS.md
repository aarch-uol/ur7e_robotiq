# How it works

## The big picture

```
                         ┌──────────────── drivers.launch.py ────────────────┐
                         │                                                   │
  MoveIt (move_group) ───┼─► /controller_manager            (official UR driver)
    │                    │     scaled_joint_trajectory_controller ──► arm joints
    │                    │                                                   │
    └────────────────────┼─► /robotiq_controller_manager    (gripper only)
                         │     robotiq_gripper_controller ──► patched robotiq_driver
                         │                                      │  TCP robot_ip:54321
                         └──────────────────────────────────────┼────────────┘
                                                                ▼
                                             robot's RS485 URCap ─► wrist ─► 2F-85
```

- **Two controller managers.** The arm and gripper run separately, so a gripper connection problem can never stop the arm.
- **One robot model.** `urdf/ur7e_robotiq.urdf.xacro` = the official UR7e model + the 2F-85 mounted on `tool0`. It's used for TF, RViz and MoveIt.
- **Both publish to `/joint_states`.** MoveIt sees all arm and gripper joints together.

## Why the Robotiq driver is patched

The Robotiq driver normally opens a serial port like `/dev/ttyUSB0`. Here the gripper sits behind the robot's wrist, and the robot exposes that wire as a **TCP port (54321)**. The usual approach is a tool that turns that TCP port into a fake serial device (`/tmp/ttyUR`), but:

1. **Robotiq's serial library (`libserialport`) refuses fake serial devices** (`/dev/pts/*`). So the gripper could never connect that way.
2. **The robot drops the TCP connection** now and then, for example while the gripper is quiet during its activation sweep. The fake-serial bridge doesn't recover when that happens.

`patches/robotiq_driver_tcp_transport.patch` adds one small connection type to `robotiq_driver`, **`TcpSerial`** (`src/tcp_serial.cpp`). When the `tcp_host` parameter is set, the driver uses it instead of a serial port. It:

- talks to `robot_ip:54321` directly
- waits for each full gripper reply, even when the robot forwards it in several pieces
- **reconnects automatically** (at most every 200 ms) when the robot drops the connection

Without `tcp_host` the driver behaves exactly like upstream (USB serial). The gripper protocol, controllers and topics are all upstream.

The install steps clone Robotiq's repo at the exact tested commit (`8d7b841`) and apply the patch.

## Why ROS doesn't configure the tool port

The UR driver *can* set the tool voltage and RS-485 settings itself (`use_tool_communication`). Here it's turned **off**, and the settings come from the pendant (**Installation → Tool I/O**) instead. This way there's one place to configure it, and ROS never starts a second, competing connection to the gripper port.

## Startup order (drivers.launch.py)

1. The UR driver starts. The arm controllers load but stay **inactive** until you press **Play** on the pendant.
2. The gripper's controller manager starts with the gripper **not connected**.
3. `wait_for_gripper` waits until `robot_ip:54321` accepts connections **and** the gripper's controller manager has loaded the gripper.
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
