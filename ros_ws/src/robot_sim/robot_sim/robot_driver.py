import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
import math
import json
import os
import yaml


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.robot_id = os.getenv('ROBOT_ID', 'robot_1')

        # Загрузка конфига (карта)
        map_name = os.getenv('MAP', 'simple')
        cfg_path = f"/configs/{map_name}.yaml"
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        # Координаты узлов
        self.node_coords = {**cfg['nodes'], **cfg.get('parking', {})}

        # Стартовые координаты (из конфига)
        robot_cfg = cfg['robots'][self.robot_id]
        self.start_x = robot_cfg['start_x']
        self.start_y = robot_cfg['start_y']

        # Параметры робота
        self.L = 0.22
        self.r = 0.05
        self.K = 1.1

        # Текущая позиция (относительно старта)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Миссия
        self.mission = []
        self.mission_idx = 0
        self.status = 'idle'

        # Ручное управление
        self.manual_linear = 0.0
        self.manual_angular = 0.0
        self.manual_active = False

        # Публикаторы в Gazebo (через мост)
        self.pub_left = self.create_publisher(
            Float64, f'/model/turtle_{self.robot_id}/joint/joint_left/cmd_vel', 10)
        self.pub_right = self.create_publisher(
            Float64, f'/model/turtle_{self.robot_id}/joint/joint_right/cmd_vel', 10)

        # Публикатор состояния
        self.pub_state = self.create_publisher(String, f'/{self.robot_id}/state', 10)

        # Подписки
        self.create_subscription(Odometry, f'/odom_{self.robot_id}', self.odom_cb, 10)
        self.create_subscription(String, f'/{self.robot_id}/order', self.order_cb, 10)
        self.create_subscription(Twist, f'/{self.robot_id}/cmd_vel', self.cmd_vel_cb, 10)

        # Таймер управления (20 Гц)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Robot {self.robot_id} started at ({self.start_x}, {self.start_y})")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def order_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.mission = data.get('path', [])
            self.mission_idx = 0
            self.status = 'moving'
            self.manual_active = False
            self.get_logger().info(f"New mission: {self.mission}")
        except Exception as e:
            self.get_logger().error(f"Bad order: {e}")

    def cmd_vel_cb(self, msg):
        # Ручное управление
        self.manual_linear = msg.linear.x
        self.manual_angular = msg.angular.z
        self.manual_active = True
        self.status = 'manual'

    def control_loop(self):
        # Публикуем состояние
        state = {
            'id': self.robot_id,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'status': self.status,
            'mission': self.mission,
            'mission_idx': self.mission_idx,
        }
        self.pub_state.publish(String(data=json.dumps(state)))

        # Ручное управление
        if self.manual_active:
            v = self.manual_linear
            w = self.manual_angular
            v_left = (v - w * self.L / 2.0) / self.r
            v_right = (v + w * self.L / 2.0) / self.r * self.K
            self.pub_left.publish(Float64(data=v_left))
            self.pub_right.publish(Float64(data=v_right))
            return

        # Автономное движение по миссии
        if self.status != 'moving' or not self.mission:
            self.pub_left.publish(Float64(data=0.0))
            self.pub_right.publish(Float64(data=0.0))
            return

        # Целевой узел
        target_id = self.mission[self.mission_idx]
        if target_id not in self.node_coords:
            self.get_logger().warn(f"Unknown node {target_id}")
            self.status = 'error'
            return

        tx = self.node_coords[target_id]['x']
        ty = self.node_coords[target_id]['y']

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.1:
            self.mission_idx += 1
            if self.mission_idx >= len(self.mission):
                self.status = 'done'
                self.pub_left.publish(Float64(data=0.0))
                self.pub_right.publish(Float64(data=0.0))
            return

        target_yaw = math.atan2(dy, dx)
        angle_err = target_yaw - self.yaw
        while angle_err > math.pi: angle_err -= 2 * math.pi
        while angle_err < -math.pi: angle_err += 2 * math.pi

        if abs(angle_err) > 0.3:
            v = 0.0
            w = 1.5 if angle_err > 0 else -1.5
        else:
            v = 0.3
            w = 2.0 * angle_err

        v_left = (v - w * self.L / 2.0) / self.r
        v_right = (v + w * self.L / 2.0) / self.r * self.K

        self.pub_left.publish(Float64(data=v_left))
        self.pub_right.publish(Float64(data=v_right))


def main():
    rclpy.init()
    node = RobotDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()