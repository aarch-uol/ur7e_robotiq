"""Arm + Robotiq gripper + MoveIt + RViz, with the ATI Gamma mounted but NOT used.

    ros2 launch ur7e_robotiq_bringup ur7e_gripper_moveit.launch.py robot_ip:=<ROBOT_IP>

Then press Play on the External Control program on the pendant.

For a robot that has the Gamma and the 2F-85 on the flange (the default ft_gripper base) when only the gripper and
MoveIt are needed. The Gamma stays in the robot model, so MoveIt plans around it and the TCP is right, but its
Ethernet driver, the /ft controller manager and the arm-side wrench bridge are not started. No force sensing, no
admittance. It is moveit.launch.py with load_ft_sensor:=false and the base fixed to ft_gripper.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur7e_robotiq_bringup"), "launch", "moveit.launch.py"])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "rviz": LaunchConfiguration("rviz"),
            "load_ft_sensor": "false",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the UR7e controller"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur7e_robotiq_bringup"), "config", "ur7e_calibration.yaml"]
                ),
                description="Calibration YAML extracted from your robot (see README)",
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz with the MotionPlanning panel"),
            # The Gamma is physically on the flange: keep it in the model whatever UR7E_EE_CONFIG says in this shell
            SetEnvironmentVariable("UR7E_EE_CONFIG", "ft_gripper"),
            moveit,
        ]
    )
