#!/usr/bin/env python3
"""Zero the wrench target when its publisher disappears mid-force.

The admittance (wrench_reference) and Cartesian impedance (set_cartesian_wrench)
controllers have no input timeout, so the last wrench stays applied if the GUI
crashes. This node watches each configured WrenchStamped topic; when the last
message was non-zero and nothing arrives for timeout_s, it publishes a zero
wrench (same frame_id) zero_repeats times and logs a warning once per event.
It publishes nothing while traffic is normal or the last message was zero.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ur7e_safety.watchdog_logic import WrenchWatchdog, is_zero

DEFAULT_TOPICS = [
    "/admittance_controller/wrench_reference",
    "/cartesian_impedance_controller/set_cartesian_wrench",
]


def _wrench_values(msg):
    w = msg.wrench
    return (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)


class _Channel:
    def __init__(self, node, topic, timeout_s):
        self.node = node
        self.topic = topic
        self.logic = WrenchWatchdog(timeout_s)
        self.frame_id = ""
        self.pending = 0
        # Best-effort subscription matches reliable and best-effort publishers alike.
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = node.create_publisher(WrenchStamped, topic, pub_qos)
        self.sub = node.create_subscription(WrenchStamped, topic, self.on_msg, sub_qos)

    def on_msg(self, msg):
        # Our own zero messages come back through the subscription; they are
        # zero, so they just keep the watchdog disarmed.
        self.frame_id = msg.header.frame_id
        self.logic.on_message(self.node.now_s(), not is_zero(_wrench_values(msg)))

    def tick(self, repeats):
        if self.logic.poll(self.node.now_s()):
            self.pending = repeats
            self.node.get_logger().warn(
                f"No wrench on {self.topic} for {self.logic.timeout_s:.2f} s while non-zero: publishing zero wrench"
            )
        if self.pending > 0:
            zero = WrenchStamped()
            zero.header.frame_id = self.frame_id
            zero.header.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(zero)
            self.pending -= 1


class WrenchWatchdogNode(Node):
    def __init__(self):
        super().__init__("wrench_watchdog")
        topics = self.declare_parameter("topics", DEFAULT_TOPICS).value
        timeout_s = self.declare_parameter("timeout_s", 0.3).value
        self.repeats = int(self.declare_parameter("zero_repeats", 3).value)
        rate = self.declare_parameter("check_rate_hz", 50.0).value
        self.channels = [_Channel(self, t, float(timeout_s)) for t in topics]
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Watching {list(topics)} (timeout {timeout_s} s)")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        for c in self.channels:
            c.tick(self.repeats)


def main(args=None):
    rclpy.init(args=args)
    node = WrenchWatchdogNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
