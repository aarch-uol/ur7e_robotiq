"""UR7e + ATI Gamma + Robotiq bringup for MoveIt GUI: every arm mode loaded, Position active.

  active    scaled_joint_trajectory_controller (the driver's default)
  inactive  admittance_controller, admittance_trajectory_controller (with the F/T sensor),
            cartesian_impedance_controller (with the gripper: its end effector is robotiq_tcp)
  separate  /robotiq_controller_manager, /ft/controller_manager (drivers.launch.py), each only with its hardware

UR7E_EE_CONFIG picks the end-effector base: ft_gripper (default), gripper, ft or none. The URDF / SRDF xacro read
the same variable, so the model, MoveIt and the controllers always agree on what is on the flange.

MoveIt knows both trajectory controllers (moveit_controllers_gui.yaml); the GUI names the active one
per goal, so Position <-> Admittance needs no MoveIt restart. The commissioning launches are unchanged.
Direct torque stays off unless a real activation is requested: the impedance controller is only configured.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


EE_CONFIGS = ("none", "gripper", "ft", "ft_gripper")


def end_effector():
    """(has_ft, has_gripper) from UR7E_EE_CONFIG, the variable the robot's URDF / SRDF xacro read too."""
    ee = os.environ.get("UR7E_EE_CONFIG", "ft_gripper")
    if ee not in EE_CONFIGS:
        raise RuntimeError(f"UR7E_EE_CONFIG must be one of {', '.join(EE_CONFIGS)}, not '{ee}'")
    return ee in ("ft", "ft_gripper"), ee in ("gripper", "ft_gripper")


def generate_launch_description():
    has_ft, has_gripper = end_effector()
    def launch_file(package, name):
        return PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), "launch", name]))

    kinematics = {"kinematics_params_file": LaunchConfiguration("kinematics_params_file")}.items()
    drivers = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_bringup", "drivers.launch.py"),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "activate_joint_controller": "true",
            "ur_controllers_file": PathJoinSubstitution(
                [FindPackageShare("ur7e_robotiq_bringup"), "config", "ur_controllers_gui.yaml"]
            ),
            "load_ft_sensor": "true" if has_ft else "false",
            "load_gripper": "true" if has_gripper else "false",
            "ft_ip_address": LaunchConfiguration("ft_ip_address"),
            "ft_sampling_rate": LaunchConfiguration("ft_sampling_rate"),
            "ft_hardware_biasing": LaunchConfiguration("ft_hardware_biasing"),
        }.items(),
    )

    inactive_modes = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        # admittance needs the ATI (ati_fts_sensor), the impedance controller's end effector is robotiq_tcp
        arguments=(["admittance_controller", "admittance_trajectory_controller"] if has_ft else []) + (
            ["cartesian_impedance_controller"] if has_gripper else []) + [
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    move_group = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "move_group_gui.launch.py"),
        launch_arguments=kinematics,
    )
    servo = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "servo.launch.py"),
        launch_arguments=kinematics,
        condition=IfCondition(LaunchConfiguration("servo")),
    )
    rviz = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "moveit_rviz.launch.py"),
        launch_arguments=kinematics,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # The wrench controllers have no input timeout: zero their target if the
    # (GUI) publisher dies while a force is commanded. See ur7e_safety.
    wrench_watchdog = Node(
        package="ur7e_safety",
        executable="wrench_watchdog",
        name="wrench_watchdog",
        output="screen",
        parameters=[{"topics": ["/admittance_controller/wrench_reference", "/cartesian_impedance_controller/set_cartesian_wrench"], "timeout_s": 0.3}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the UR7e controller"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur7e_robotiq_bringup"), "config", "ur7e_calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("ft_ip_address", default_value="192.168.1.1"),
            DeclareLaunchArgument("ft_sampling_rate", default_value="1000"),
            DeclareLaunchArgument("ft_hardware_biasing", default_value="false"),
            DeclareLaunchArgument("servo", default_value="false", description="Start MoveIt Servo"),
            DeclareLaunchArgument("rviz", default_value="false"),
            drivers,
            *([inactive_modes] if (has_ft or has_gripper) else []),
            move_group,
            servo,
            rviz,
            wrench_watchdog,
        ]
    )
