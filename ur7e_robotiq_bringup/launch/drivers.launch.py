"""UR7e arm + Robotiq 2F-85 gripper drivers only (no MoveIt).
moveit.launch.py starts this plus MoveIt and RViz.

Starts:
  * the official UR driver (arm) on /controller_manager
  * a second, gripper-only controller manager on /robotiq_controller_manager
    that talks to the gripper through the robot's wrist RS-485 port, exposed
    by the robot at robot_ip:tool_tcp_port.

The two are separate so a gripper link problem never takes the arm down.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    pkg_share = FindPackageShare("ur7e_robotiq_bringup")
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "robotiq_controllers.yaml"])

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": "ur7e",
            "robot_ip": robot_ip,
            "description_launchfile": PathJoinSubstitution([pkg_share, "launch", "combined_rsp.launch.py"]),
            "kinematics_params_file": kinematics_params_file,
            "launch_rviz": launch_rviz,
            "headless_mode": headless_mode,
            # The tool port is configured on the pendant (Installation > Tool I/O)
            # and the gripper driver connects to it over TCP directly, so the
            # UR driver's own tool-communication bridge stays off.
            "use_tool_communication": "false",
        }.items(),
    )

    gripper_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([pkg_share, "urdf", "robotiq_2f85_control.urdf.xacro"]),
                " tcp_host:=",
                robot_ip,
                " tcp_port:=",
                tool_tcp_port,
            ]
        ),
        value_type=str,
    )

    # Jazzy's controller_manager reads the robot_description topic, so the
    # gripper-only description is published on its own topic.
    robotiq_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robotiq_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": gripper_description}],
        remappings=[
            ("robot_description", "/robotiq/robot_description"),
            ("/tf", "/robotiq/tf"),
            ("/tf_static", "/robotiq/tf_static"),
        ],
    )

    gripper_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="robotiq_controller_manager",
        output="screen",
        parameters=[controllers_yaml],
        remappings=[("robot_description", "/robotiq/robot_description")],
    )

    # Before activating the gripper, wait until:
    #  1. the robot's tool port accepts connections. The first connect must
    #     succeed (later drops reconnect automatically). The probe only opens a
    #     socket and never sends bytes to the gripper.
    #  2. the gripper controller manager has loaded the gripper from its
    #     description. hardware_spawner exits "successfully" without doing
    #     anything if it runs before that.
    wait_for_gripper = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            [
                'echo "waiting for the tool TCP port ', robot_ip, ":", tool_tcp_port, '"',
                "; until (exec 3<>/dev/tcp/", robot_ip, "/", tool_tcp_port,
                ") 2>/dev/null; do sleep 0.2; done",
                "; until ros2 control list_hardware_components -c /robotiq_controller_manager 2>/dev/null"
                " | grep -q robotiq_2f85; do sleep 0.5; done",
                "; echo tool port reachable and gripper loaded",
            ],
        ],
        name="wait_for_gripper",
        output="screen",
    )

    gripper_hardware_spawner = Node(
        package="controller_manager",
        executable="hardware_spawner",
        output="screen",
        arguments=[
            "robotiq_2f85",
            "--controller-manager", "/robotiq_controller_manager",
            "--controller-manager-timeout", "30",
            "--activate",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "robotiq_joint_state_broadcaster",
            "robotiq_gripper_controller",
            "robotiq_activation_controller",
            "robotiq_gripper_status_broadcaster",
            "--controller-manager", "/robotiq_controller_manager",
            "--controller-manager-timeout", "30",
            "--param-file", controllers_yaml,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the UR7e controller"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "config", "ur7e", "default_kinematics.yaml"]
                ),
                description="Calibration YAML extracted from your robot (see docs/INSTALL.md)",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument(
                "tool_tcp_port",
                default_value="54321",
                description="TCP port where the robot exposes the wrist RS-485 port",
            ),
            ur_driver,
            robotiq_robot_state_publisher,
            gripper_manager,
            wait_for_gripper,
            RegisterEventHandler(
                OnProcessExit(target_action=wait_for_gripper, on_exit=[gripper_hardware_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=gripper_hardware_spawner, on_exit=[gripper_controller_spawner])
            ),
        ]
    )
