from copy import deepcopy
from typing import Optional, Tuple

from control_msgs.msg import JointJog
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from std_srvs.srv import Trigger

class MoveIt2ServoJoints:
    def __init__(
        self,
        node: Node,
        frame_id: str,
        joint_names: str,
        #speed_setter: float = 1.0,
        callback_group: Optional[CallbackGroup] = None,
        enable_at_init: bool = True,
        
    ):

        self._node=node

        # Create publisher
        self.__joint_pub = self._node.create_publisher(
            msg_type=JointJog,
            topic="/servo_node/delta_joint_cmds",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            ),
            callback_group=callback_group,
        )

        # Create service clients
        self.__start_service = self._node.create_client(
            srv_type=Trigger,
            srv_name="/servo_node/start_servo",
            callback_group=callback_group,
        )
        self.__stop_service = self._node.create_client(
            srv_type=Trigger,
            srv_name="/servo_node/stop_servo",
            callback_group=callback_group,
        )
        self.__trigger_req = Trigger.Request()
        self.__is_enabled = True

        # Initialize message based on passed arguments
        self.__joint_msg = JointJog()
        self.__joint_msg.header.frame_id = frame_id
        self.__joint_msg.joint_names = joint_names
        
        if enable_at_init:
            self.enable()

    def __del__(self):
        """
        Try to stop MoveIt 2 Servo during destruction.
        """

        try:
            if self.is_enabled:
                self.__stop_service.call_async(self.__trigger_req)
        except:
            pass

    def __call__(
        self,
        velocities: Tuple[float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ):
        """
        Callable that is identical to `MoveIt2Servo.servo()`.
        """
        self.servoJ(velocities=velocities)

    def servoJ(
        self,
        velocities: Tuple[float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        enable_if_disabled: bool = True,
    ):

        if not self.is_enabled:
            self._node.get_logger().warn(
                "Command failed because MoveIt Servo is not yet enabled."
            )
            if enable_if_disabled:
                self._node.get_logger().warn(
                    f"Calling '{self.__start_service.srv_name}' service to enable MoveIt Servo..."
                )
                if not self.enable():
                    return
            else:
                return

        joint_msg = deepcopy(self.__joint_msg)
        joint_msg.header.stamp = self._node.get_clock().now().to_msg()
        joint_msg.velocities = [velocities[0], velocities[1],velocities[2], velocities[3],velocities[4],velocities[5]]     #velocities[5]
        self.__joint_pub.publish(joint_msg)

    def enable(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False
    ) -> bool:
        """
        Enable MoveIt 2 Servo server via async service call.
        """

        while not self.__start_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__start_service.srv_name}' is not yet available..."
            )
            return False

        if sync:
            result = self.__start_service.call(self.__trigger_req)
            if not result.success:
                self._node.get_logger().error(
                    f"MoveIt Servo could not be enabled. ({result.message})"
                )
            self.__is_enabled = result.success
            return result.success
        else:
            start_service_future = self.__start_service.call_async(self.__trigger_req)
            start_service_future.add_done_callback(self.__enable_done_callback)
            return True

    def disable(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False
    ) -> bool:
        """
        Disable MoveIt 2 Servo server via async service call.
        """

        while not self.__stop_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__stop_service.srv_name}' is not yet available..."
            )
            return False

        if sync:
            result = self.__stop_service.call(self.__trigger_req)
            if not result.success:
                self._node.get_logger().error(
                    f"MoveIt Servo could not be disabled. ({result.message})"
                )
            self.__is_enabled = not result.success
            return result.success
        else:
            stop_service_future = self.__stop_service.call_async(self.__trigger_req)
            stop_service_future.add_done_callback(self.__disable_done_callback)
            return True

    def __enable_done_callback(self, future: Future):

        result: Trigger.Response = future.result()

        if not result.success:
            self._node.get_logger().error(
                f"MoveIt Servo could not be enabled. ({result.message})"
            )

        self.__is_enabled = result.success

    def __disable_done_callback(self, future: Future):

        result: Trigger.Response = future.result()

        if not result.success:
            self._node.get_logger().error(
                f"MoveIt Servo could not be disabled. ({result.message})"
            )

        self.__is_enabled = not result.success

    @property
    def is_enabled(self) -> bool:

        return self.__is_enabled

    @property
    def frame_id(self) -> str:

        return self.__joint_msg.header.frame_id

    @frame_id.setter
    def frame_id(self, value: str):

        self.__joint_msg.header.frame_id = value

    @property
    def joint_names(self) -> str:
        return self.__joint_msg.joint_names

    @joint_names.setter
    def joint_names(self, value: str):

        self.__joint_msg.joint_names = value

    @property
    def speed_setter(self) -> float:

        return self.__joint_msg.velocities[0]

    @speed_setter.setter
    def speed_setter(self, value: float):

        self.__joint_msg.velocities[0] = value
        self.__joint_msg.velocities[1] = value
        self.__joint_msg.velocities[2] = value
        self.__joint_msg.velocities[3] = value
        self.__joint_msg.velocities[4] = value
        self.__joint_msg.velocities[5] = value
