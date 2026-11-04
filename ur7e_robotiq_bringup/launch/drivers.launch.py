"""UR7e arm + Robotiq 2F-85 gripper drivers only (no MoveIt).
moveit.launch.py starts this plus MoveIt and RViz.

Starts:
  * the official UR driver (arm) on /controller_manager
  * a second, gripper-only controller manager on /robotiq_controller_manager
    that talks to the gripper through /tmp/ttyUR, created by the official UR
    tool-communication forwarder for the wrist RS-485 port.

The two are separate so a gripper link problem never takes the arm down.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
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
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    load_ft_sensor = LaunchConfiguration("load_ft_sensor")
    ft_ip_address = LaunchConfiguration("ft_ip_address")
    ft_sampling_rate = LaunchConfiguration("ft_sampling_rate")
    ft_hardware_biasing = LaunchConfiguration("ft_hardware_biasing")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    ur_controllers_file = LaunchConfiguration("ur_controllers_file")

    pkg_share = FindPackageShare("ur7e_robotiq_bringup")
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "robotiq_controllers.yaml"])
    ft_controllers_yaml = PathJoinSubstitution([pkg_share, "config", "ft_controllers.yaml"])

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": "ur7e",
            "robot_ip": robot_ip,
            "description_launchfile": PathJoinSubstitution([pkg_share, "launch", "combined_rsp.launch.py"]),
            "kinematics_params_file": kinematics_params_file,
            "controllers_file": ur_controllers_file,
            "launch_rviz": launch_rviz,
            "headless_mode": headless_mode,
            "activate_joint_controller": activate_joint_controller,
            "use_tool_communication": "true",
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage": "24",
            "tool_baud_rate": "115200",
            "tool_parity": "0",
            "tool_stop_bits": "1",
            "tool_rx_idle_chars": "1.5",
            "tool_tx_idle_chars": "3.5",
        }.items(),
    )

    gripper_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([pkg_share, "urdf", "robotiq_2f85_control.urdf.xacro"]),
                " com_port:=",
                tool_device_name,
                " baudrate:=115200",
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

    # ATI Gamma runs in its own controller manager. This keeps F/T network or
    # sensor faults isolated from the UR arm and Robotiq controller managers.
    # The combined robot model already contains the physical Gamma frames; this
    # small description contains only the ros2_control sensor hardware.
    ft_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([pkg_share, "urdf", "ati_gamma_control.urdf.xacro"]),
                " ip_address:=",
                ft_ip_address,
                " rdt_sampling_rate:=",
                ft_sampling_rate,
                " use_hardware_biasing:=",
                ft_hardware_biasing,
            ]
        ),
        value_type=str,
    )

    ft_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ft",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ft_description}],
        remappings=[
            ("/tf", "/ft/discard_tf"),
            ("/tf_static", "/ft/discard_tf_static"),
        ],
        condition=IfCondition(load_ft_sensor),
    )

    ft_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="ft",
        name="controller_manager",
        output="screen",
        parameters=[ft_controllers_yaml],
        remappings=[("robot_description", "/ft/robot_description")],
        condition=IfCondition(load_ft_sensor),
    )

    ft_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager",
            "/ft/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--param-file",
            ft_controllers_yaml,
        ],
        condition=IfCondition(load_ft_sensor),
    )

    # Before activating the gripper, wait until:
    #  1. the official UR tool-communication process has created the PTY.
    #  2. the gripper controller manager has loaded the gripper from its
    #     description. hardware_spawner exits "successfully" without doing
    #     anything if it runs before that.
    wait_for_gripper = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            [
                'echo "waiting for the UR tool device ', tool_device_name, '"',
                "; until test -e ", tool_device_name, "; do sleep 0.2; done",
                "; until ros2 control list_hardware_components -c /robotiq_controller_manager 2>/dev/null"
                " | grep -q robotiq_2f85; do sleep 0.5; done",
                "; echo UR tool device ready and gripper loaded",
            ],
        ],
        name="wait_for_gripper",
        output="screen",
    )

    # Activation connects to the gripper, resets and calibrates it, and can fail
    # (RS-485 forwarder not ready, gripper still powering up, link glitch). The
    # plain hardware_spawner tries once and the launch moves on, leaving the
    # gripper dead until someone restarts the stack. Retry every 2 s for up to
    # 60 s, stop as soon as the component reports "active". After the last try
    # the launch continues as before (the controller spawner then fails visibly).
    # Note: if the driver returns ERROR (not FAILURE) the component is finalized
    # and only a relaunch recovers it; the retries then just time out.
    gripper_hardware_spawner = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "is_active() { ros2 control list_hardware_components -c /robotiq_controller_manager 2>/dev/null"
            " | sed 's/\\x1b\\[[0-9;]*m//g' | grep -q 'label=active'; }"
            "; for i in $(seq 1 30); do"
            " if is_active; then echo 'gripper hardware active'; exit 0; fi"
            "; echo \"gripper activation attempt $i/30\""
            "; ros2 run controller_manager hardware_spawner robotiq_2f85"
            " --controller-manager /robotiq_controller_manager"
            " --controller-manager-timeout 30 --activate"
            "; is_active && { echo 'gripper hardware active'; exit 0; }"
            "; sleep 2; done"
            "; echo 'WARNING: gripper hardware not active after 60 s of retries' >&2; exit 1",
        ],
        name="gripper_hardware_activation",
        output="screen",
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
                    [
                        FindPackageShare("ur7e_robotiq_bringup"),
                        "config",
                        "ur7e_calibration.yaml",
                    ]
                ),
                description="Calibration YAML extracted from your robot (see docs/INSTALL.md)",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument(
                "activate_joint_controller",
                default_value="true",
                description="Activate the UR driver's normal initial joint controller",
            ),
            DeclareLaunchArgument(
                "ur_controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
                ),
                description="Controller parameter file for the UR arm controller manager",
            ),
            DeclareLaunchArgument(
                "tool_device_name",
                default_value="/tmp/ttyUR",
                description="Local PTY created by the official UR tool-communication forwarder",
            ),
            DeclareLaunchArgument(
                "tool_tcp_port",
                default_value="54321",
                description="TCP port where the robot exposes the wrist RS-485 port",
            ),
            DeclareLaunchArgument(
                "load_ft_sensor",
                default_value="true",
                description="Start the ATI Gamma Net F/T sensor driver and broadcaster",
            ),
            DeclareLaunchArgument(
                "ft_ip_address",
                default_value="192.168.1.1",
                description="ATI Gamma Net F/T sensor IP address",
            ),
            DeclareLaunchArgument(
                "ft_sampling_rate",
                default_value="1000",
                description="ATI Gamma RDT sampling rate in Hz",
            ),
            DeclareLaunchArgument(
                "ft_hardware_biasing",
                default_value="false",
                description="Use ATI hardware biasing on activation",
            ),
            ur_driver,
            robotiq_robot_state_publisher,
            gripper_manager,
            ft_robot_state_publisher,
            ft_controller_manager,
            ft_broadcaster_spawner,
            wait_for_gripper,
            RegisterEventHandler(
                OnProcessExit(target_action=wait_for_gripper, on_exit=[gripper_hardware_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=gripper_hardware_spawner, on_exit=[gripper_controller_spawner])
            ),
        ]
    )
