# Troubleshooting

Every problem below actually happened while this setup was being built.

**First, three quick checks:**
```bash
ros2 control list_controllers | grep scaled                 # arm: must say "active"
ros2 control list_hardware_components -c /robotiq_controller_manager   # gripper: "active"
ros2 node list 2>&1 | head -1                               # must NOT warn about duplicate names
```

---

## Arm

### Arm won't move: `CONTROL_FAILED` (error_code -4), "Controller is not running"

`scaled_joint_trajectory_controller` is **inactive**. The UR driver turns it off automatically whenever the External Control program isn't running on the robot.

**Fix:** press **▶ Play** on the pendant, then re-check `ros2 control list_controllers | grep scaled`. You have to press Play again **after every bringup restart**.

### It worked, then stopped mid-motion: "Connection to reverse interface dropped"

The robot stopped the program. Look at the pendant for a popup, for example a **protective stop**, joint limit, or someone pressed Stop. Clear it and press **Play**.

Tip: keep **Velocity/Accel. Scaling** low in RViz (`0.1`) while testing.

### `PATH_TOLERANCE_VIOLATED`

The controller started, but the real arm didn't follow the trajectory. This is usually the same cause as above: the program was paused or stopped. Check the pendant, press Play, and try a smaller motion.

## Gripper

### Gripper never became `active` after launch

`drivers.launch.py` retries activation every 2 s for 60 s and prints `gripper activation attempt N/30`. If it still gives up, check the tool-port items below, then relaunch (a driver `ERROR` finalizes the component; only a relaunch recovers it).

### Log keeps printing `TCP serial bridge closed the connection` / `link faulted`

The robot keeps dropping the gripper's connection. A few of these right after startup are normal while the gripper activates. If they **don't stop**, something on the robot is also using the gripper port. Check [ROBOT_SETUP.md](ROBOT_SETUP.md):

1. **Robotiq Grippers URCap** must be **disabled**, and the robot **restarted** afterwards.
2. **Tool I/O → Controlled by** must be **User**.
3. The program must not contain any Robotiq gripper nodes.

This was the fix for the gripper link dropping every ~150 ms.

### `robotiq_joint_state_broadcaster: No state interfaces found to publish` / MoveIt says `Missing robotiq_85_left_knuckle_joint`

The gripper hardware didn't activate, so its controllers can't start. Check the bringup log above this error for the reason. Once it's fixed, you can activate the gripper without restarting anything:
```bash
ros2 run controller_manager hardware_spawner robotiq_2f85 -c /robotiq_controller_manager --activate
ros2 run controller_manager spawner robotiq_joint_state_broadcaster robotiq_gripper_controller \
  robotiq_activation_controller robotiq_gripper_status_broadcaster -c /robotiq_controller_manager \
  --param-file $(ros2 pkg prefix ur7e_robotiq_bringup)/share/ur7e_robotiq_bringup/config/robotiq_controllers.yaml
```

### Stuck at `waiting for the tool TCP port <robot_ip>:54321`

The PC can't reach the gripper port on the robot.
- Is the **RS485 URCap** installed and active?
- Can you `ping` the robot?
- Is the robot powered on (it doesn't need to be playing yet)?

### Gripper moves but the reported position looks stale

The log says `link is Faulted; the reported position may be stale` while it's reconnecting. It clears by itself (`link recovered`). If it never clears, see the first gripper entry above.

## MoveIt / RViz

### `START_STATE_IN_COLLISION` (error_code -10)

MoveIt thinks two links of the robot are touching. The log names them, for example `contact(s) detected : robotiq_85_base_link - wrist_3_link`. If it's two parts that are *supposed* to touch, add a line to `ur7e_robotiq_moveit_config/config/ur7e_robotiq.srdf.xacro`:
```xml
<disable_collisions link1="LINK_A" link2="LINK_B" reason="Adjacent"/>
```
Rebuild and restart move_group.

### MoveIt error codes

| Code | Meaning |
|---|---|
| `1` | success |
| `-1` | no plan found (goal unreachable?) |
| `-4` | controller failed (see *Arm* above) |
| `-6` | timed out |
| `-10` | start state in collision |
| `-12` | goal in collision |

### Harmless messages you can ignore

- move_group: `No 3D sensor plugin(s) defined for octomap updates` (no depth camera is configured)
- RViz: `Action server: /recognize_objects not available`
- RViz: `No active joints or end effectors found for group 'gripper'` (there's just no drag marker for the gripper; use the `open`/`closed` goal states)
- RViz: `glx: failed to create dri3 screen` / `failed to load driver: nvidia-drm` (RViz falls back to software rendering)

### RViz: `cannot open display`

Run `export DISPLAY=:1` (or `:0`) before launching. With Docker, `./docker/run.sh` already passes your display through. If RViz still can't open, run `xhost +local:root` on the host.

## General

<a id="two-copies-running"></a>
### Two copies running / weird random failures

**Symptoms:**
- `ros2 node list` warns *"nodes in the graph that share an exact name"*
- a spawner fails with `Failed loading controller`
- the gripper works but the arm doesn't, or the other way round
- `ros2` commands hang or time out

This means an old bringup / move_group is still running next to the new one. It happens when a terminal is closed without **Ctrl+C**, or `ros2 launch` is run twice.

**Fix:**
- **Docker:** exit all container terminals and start again with `./docker/run.sh`. Each start is a fresh container. `run.sh` uses `--init`, SIGINT as stop signal and a 15 s stop timeout, so `docker stop ur7e_robotiq` shuts the launch down cleanly.
- **Native:**
  ```bash
  pkill -f "ros2 launch"; pkill -f ros2_control_node; pkill -f move_group; pkill -f rviz2
  ros2 daemon stop
  ```
  Then start again.

Old leftover processes can also eat memory over time. One leftover launch process grew to 6.5 GB and made everything time out. Restarting cleanly fixes it.

### `ros2 service call ...` hangs or says `rcl node's context is invalid`

Run `ros2 daemon stop` and try again.
