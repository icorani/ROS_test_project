import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class RobotSim(Node):
    def __init__(self):
        super().__init__('robot_sim')
        self.L = 0.22  # wheel_separation
        self.r = 0.05  # wheel_radius
        self.K = 1.1  # калибровка правого колеса

        self.pub_left = self.create_publisher(Float64, '/model/turtle/joint/joint_left/cmd_vel', 10)
        self.pub_right = self.create_publisher(Float64, '/model/turtle/joint/joint_right/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

    def cmd_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        v_left = (v - w * self.L / 2.0) / self.r
        v_right = (v + w * self.L / 2.0) / self.r * self.K

        # Публикуем в одном тике, максимально синхронно
        self.pub_left.publish(Float64(data=v_left))
        self.pub_right.publish(Float64(data=v_right))


def main():
    rclpy.init()
    node = RobotSim()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()