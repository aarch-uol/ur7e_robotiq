# ur7e_robotiq_bringup

Starts the UR7e arm driver, ATI Gamma F/T driver and Robotiq 2F-85 gripper driver.

See the [top-level README](../README.md) for setup and usage.

## Experimental direct-torque Cartesian impedance

`cartesian_impedance.launch.py` is a separate experimental mode. It does not
start MoveIt or the position-interface admittance chain. It requires the
official UR direct-torque interface and therefore PolyScope 5.25.1 or newer on
an e-Series robot.

Start it with the impedance controller configured but inactive:

```bash
ros2 launch ur7e_robotiq_bringup cartesian_impedance.launch.py \
  robot_ip:=10.6.203.101
```

Before activation, confirm the console reports `Calibration checked
successfully`, the UR program is running, the payload/TCP are correct, the
workspace is clear, and no joint position or velocity controller is active:

```bash
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager | grep effort
```

Only after those checks, activate direct torque mode explicitly:

```bash
ros2 control set_controller_state cartesian_impedance_controller active \
  -c /controller_manager
```

Deactivate it before stopping the program or changing control modes:

```bash
ros2 control set_controller_state cartesian_impedance_controller inactive \
  -c /controller_manager
```

The initial configuration makes TCP X softer than the other Cartesian axes,
limits torque change to 0.10 Nm per 500 Hz update, and sets nullspace stiffness
and damping to zero. The ATI Gamma continues to publish for monitoring, but it
is not a feedback input to this impedance law.
