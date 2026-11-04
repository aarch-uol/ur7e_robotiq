# MoveIt-GUI v2.3: visual system and real-stack notes

Navigation is unchanged: OPERATE, PROGRAMS, AUTOMATION, SETUP, DIAGNOSTICS. v2.2 screens are in `ux/v2.2/`; regenerate
the current ones with `QT_QPA_PLATFORM=offscreen python3 tools/screenshots.py` (fake robot, temporary config).

## Design system (`moveit_gui/gui/theme.py`)
One token table and one stylesheet. Widgets set a `role` or `kind` property instead of styling themselves.
Grouping comes from background levels, not outlines. The accent is for the one main action and for selection; green is
healthy state, amber is caution or direct torque, red is real stop, delete or fault.

## MoveIt routing (Position and Admittance)
`moveit_controllers_gui.yaml` lists both trajectory controllers for MoveItSimpleControllerManager, with
`scaled_joint_trajectory_controller` as default. The GUI plans, then sends `ExecuteTrajectory` with
`controller_names=[<controller of the active mode>]`, read fresh from the controller manager. There is no routing state and no
MoveIt restart. A goal from RViz names no controller, goes to the default and fails closed in Admittance, because that
controller is inactive. Ros2ControlManager was rejected: the gripper has its own controller manager.

## Mode switching
Record previous mode, cancel, verify stationary, STRICT switch, verify controllers, verify interface claims, verify the
target (trajectory action or impedance reference). After a failed post-switch check the target is deactivated and the previous
mode restored and verified. If rollback fails the state latches as `CONTROL_STATE_UNKNOWN`, GUI motion is disabled and
OPERATE offers Re-verify after manual recovery.

The GUI never publishes an impedance reference. The controller seeds it from hardware state in `on_activate`
(`patches/cartesian_impedance_controller_ur6dof.patch`); the GUI reads `controller_state` and fails closed on a mismatch.

## Bringup
`ros2 launch ur7e_robotiq_bringup gui.launch.py robot_ip:=...` loads every arm mode with Position active. The older
commissioning launches are untouched. `servo:=true` starts MoveIt Servo with `ur7e_servo.yaml`.

## Policies
Manifests produce the real runner CLI. `PolicyExecutor` is the single path for AUTOMATION and the PROGRAM policy step.
`ur7e_policy_deployment.commissioning.yaml` (`ready: false`) blocks real runs until the listed values are measured. It is a
sidecar because the runner rejects unknown keys in its own config. Real runs have no live feedback until
`docs/RunPolicy.action` is built; `/policy_runner/status` is declared by the runner but never published.
