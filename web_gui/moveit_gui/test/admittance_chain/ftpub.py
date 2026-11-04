import rclpy
from geometry_msgs.msg import WrenchStamped
rclpy.init(); n = rclpy.create_node("fake_ft")
p = n.create_publisher(WrenchStamped, "/ft/force_torque_sensor_broadcaster/wrench", 10)
def cb():
    m = WrenchStamped(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = "ft_fts_link"; p.publish(m)
n.create_timer(0.002, cb); rclpy.spin(n)
