import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import json
import logging

logging.basicConfig(level=logging.DEBUG, format='%(message)s')


class PrimitiveNode(Node):
    def __init__(self):
        super().__init__('primitive_node')
        self.sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(json.dumps({"event": "node_start", "node": "primitive_node"}))

    def on_odom(self, msg):
        v = msg.twist.twist.linear.x
        self.get_logger().debug(json.dumps({
            "event": "odom", "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y, "vx": v
        }))

    def tick(self):
        t = Twist()
        t.linear.x = 0.2
        self.pub.publish(t)
        self.get_logger().trace if hasattr(self.get_logger(), 'trace') else self.get_logger().debug(
            json.dumps({"event": "cmd_vel", "linear_x": 0.2})
        )


def main():
    rclpy.init()
    node = PrimitiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
