import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
import math
import json
import os
import yaml
import time

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

        # Стартовые координаты (из конфига секции robots, либо из parking по индексу)
        robots_cfg = cfg.get('robots', {})
        if self.robot_id in robots_cfg:
            robot_cfg = robots_cfg[self.robot_id]
            self.start_x = robot_cfg['start_x']
            self.start_y = robot_cfg['start_y']
            self.start_yaw = robot_cfg.get('start_yaw', 0)
        else:
            # Вывести из parking: robot_1 -> P1, robot_2 -> P2, ...
            parking = sorted(cfg.get('parking', {}).items(), key=lambda item: item[0])
            idx = int(self.robot_id.split('_')[-1]) - 1
            if 0 <= idx < len(parking):
                _, pos = parking[idx]
                self.start_x = pos['x']
                self.start_y = pos['y']
                self.start_yaw = 0
            else:
                self.start_x = 0.0
                self.start_y = 0.0
                self.start_yaw = 0

        # Параметры робота
        self.L = 0.22
        self.r = 0.05
        self.K = 1.1

        # Текущая позиция (абсолютная, из /odom)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_movement_time = time.time()
        self.odom_received = False

        # Миссия
        self.mission = []
        self.mission_idx = 0
        self.status = 'idle'

        # Ручное управление
        self.manual_linear = 0.0
        self.manual_angular = 0.0
        self.manual_active = False
        self.last_cmd_time = 0.0

        # Ошибки
        self.error = None

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
        old_x, old_y = self.x, self.y
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_received = True
        # Track movement
        if math.hypot(self.x - old_x, self.y - old_y) > 0.001:
            self.last_movement_time = time.time()

    def order_cb(self, msg):
        try:
            data = json.loads(msg.data)
            path = data.get('path', [])
            if not path:
                self.error = "no_path"
                self.status = 'error'
                return
            # Check if any node exists
            for node in path:
                if node not in self.node_coords:
                    self.error = "node_not_found"
                    self.status = 'error'
                    self.get_logger().error(f"Unknown node {node} not in {list(self.node_coords.keys())}")
                    return
            self.mission = path
            self.mission_idx = 0
            self.status = 'moving'
            self.manual_active = False
            self.error = None
            self.last_movement_time = time.time()
            self.get_logger().info(f"New mission: {self.mission}")
        except Exception as e:
            self.get_logger().error(f"Bad order: {e}")
            self.error = "bad_order"
            self.status = 'error'

    def cmd_vel_cb(self, msg):
        # Ручное управление
        self.manual_linear = msg.linear.x
        self.manual_angular = msg.angular.z
        self.manual_active = True
        self.last_cmd_time = time.time()
        self.status = 'manual'
        self.error = None

    def control_loop(self):
        now = time.time()
        # Таймаут ручного управления (0.5 сек без cmd_vel → idle)
        if self.manual_active and (now - self.last_cmd_time) > 0.5:
            self.manual_active = False
            self.status = 'idle'

        # Check if too far from any node/edge (only after we have odom)
        if self.odom_received and self.status in ['moving', 'manual']:
            nearest = min(self.node_coords.keys(), key=lambda n: math.hypot(self.node_coords[n]['x'] - self.x, self.node_coords[n]['y'] - self.y))
            dist_to_nearest = math.hypot(self.node_coords[nearest]['x'] - self.x, self.node_coords[nearest]['y'] - self.y)
            if dist_to_nearest > 3.0:  # Too far from graph
                self.error = "too_far_from_graph"
                self.status = 'error'

        # Check if stuck (no movement for long time while supposed to move)
        if self.status == 'moving' and (now - self.last_movement_time) > 3.0:
            self.error = "stuck_in_obstacle"
            self.status = 'error'

        # Публикуем состояние
        state = {
            'id': self.robot_id,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'status': self.status,
            'mission': self.mission,
            'mission_idx': self.mission_idx,
            'linear': self.manual_linear if self.manual_active else 0.0,
            'angular': self.manual_angular if self.manual_active else 0.0,
            'error': self.error,
        }
        self.pub_state.publish(String(data=json.dumps(state)))

        # Ручное управление всегда имеет приоритет
        if self.manual_active:
            v = self.manual_linear
            w = self.manual_angular
            v_left = (v - w * self.L / 2.0) / self.r
            v_right = (v + w * self.L / 2.0) / self.r * self.K
            self.pub_left.publish(Float64(data=v_left))
            self.pub_right.publish(Float64(data=v_right))
            return

        # Автономное движение по миссии
        if self.status != 'moving' or not self.mission or self.status == 'error':
            self.pub_left.publish(Float64(data=0.0))
            self.pub_right.publish(Float64(data=0.0))
            return

        # Целевой узел
        target_id = self.mission[self.mission_idx]
        if target_id not in self.node_coords:
            self.get_logger().warn(f"Unknown node {target_id} in coords {list(self.node_coords.keys())}")
            self.error = "node_not_found"
            self.status = 'error'
            self.pub_left.publish(Float64(data=0.0))
            self.pub_right.publish(Float64(data=0.0))
            return

        tx = self.node_coords[target_id]['x']
        ty = self.node_coords[target_id]['y']

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.12:
            self.mission_idx += 1
            if self.mission_idx >= len(self.mission):
                self.status = 'done'
                self.error = None
                self.pub_left.publish(Float64(data=0.0))
                self.pub_right.publish(Float64(data=0.0))
            return

        target_yaw = math.atan2(dy, dx)
        angle_err = target_yaw - self.yaw
        while angle_err > math.pi:
            angle_err -= 2 * math.pi
        while angle_err < -math.pi:
            angle_err += 2 * math.pi

        if abs(angle_err) > 0.4:
            v = 0.0
            w = 1.8 if angle_err > 0 else -1.8
        else:
            v = 0.35
            w = 2.2 * angle_err

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
