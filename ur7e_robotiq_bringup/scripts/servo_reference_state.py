#!/usr/bin/env python3
"""Republish /joint_states with the admittance JTC's commanded reference as arm positions.

The admittance controller displaces the real arm from the JTC reference by (residual force / stiffness),
easily 1-2 cm. MoveIt Servo takes its start state from ``joint_topic`` (measured /joint_states) whenever a
new command burst starts, so its first output would jump the JTC reference by that offset and the arm
jumps at every jog start. Feeding Servo this topic (reference instead of measured arm positions) makes the
first trajectory point start exactly at the current reference.

If no reference arrived within ``max_reference_age`` (JTC inactive, e.g. Position mode) or a message has
no joint in the reference (gripper-only), the measured message is passed through unchanged: as the raw bytes,
without decoding it. Every /joint_states message comes through here (500 Hz arm + the gripper), and decoding,
copying and encoding each one in Python took ~75 % of a Jetson core.
"""

import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


class ServoReferenceState(Node):
    def __init__(self):
        super().__init__("servo_reference_state")
        self.declare_parameter("max_reference_age", 0.05)
        self.declare_parameter("output_topic", "~/joint_states")
        self.declare_parameter("controller_state_topic", "/admittance_trajectory_controller/controller_state")
        self._ref = {}  # joint name -> (position, velocity or None)
        self._ref_time = None  # time.monotonic() of the last reference
        self._age_limit = float(self.get_parameter("max_reference_age").value)
        self._pub = self.create_publisher(JointState, self.get_parameter("output_topic").value, 10)
        # Reliable subscription: matches JTC's default (reliable) publisher; a best-effort one would also match.
        self.create_subscription(
            JointTrajectoryControllerState,
            self.get_parameter("controller_state_topic").value,
            self._on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, qos_profile_sensor_data, raw=True)

    def _on_state(self, msg):
        pos, vel = msg.reference.positions, msg.reference.velocities
        self._ref = {
            n: (pos[i], vel[i] if i < len(vel) else None)
            for i, n in enumerate(msg.joint_names)
            if i < len(pos)
        }
        self._ref_time = time.monotonic()

    def _on_joint_states(self, data):
        if self._ref_time is None or time.monotonic() - self._ref_time >= self._age_limit:
            self._pub.publish(data)  # no reference (not admittance): pass the message on as it came
            return
        out = deserialize_message(data, JointState)
        for i, name in enumerate(out.name):
            ref = self._ref.get(name)
            if ref is None or i >= len(out.position):
                continue
            out.position[i] = ref[0]
            if ref[1] is not None and i < len(out.velocity):
                out.velocity[i] = ref[1]
        self._pub.publish(out)


def make_executor():
    """rclpy's EventsExecutor (Jazzy: rclpy.experimental) costs about a third of the default executor per message
    (measured on the Jetson at 500 Hz: 10 % vs 32 % of a core); the default if it is not there."""
    try:
        from rclpy.experimental import EventsExecutor
        return EventsExecutor()
    except Exception:
        from rclpy.executors import SingleThreadedExecutor
        return SingleThreadedExecutor()


def main():
    rclpy.init()
    node = ServoReferenceState()
    executor = make_executor()
    executor.add_node(node)
    node.get_logger().info(f"spinning on {type(executor).__name__}")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
