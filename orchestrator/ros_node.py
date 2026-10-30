"""ROS 2 нода оркестратора: публикаторы order/cmd_vel, подписка state."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("orchestrator")
        self.publishers_order = {}
        self.publishers_cmd = {}
        self.on_state_cb = None

    def bind_robots(self, robots):
        """Создать публикаторы/подписки для активных роботов."""
        for rid, r in robots.items():
            if not getattr(r, "is_active", True):
                continue
            self.publishers_order[rid] = self.create_publisher(
                String, f"/{rid}/order", 10)
            self.publishers_cmd[rid] = self.create_publisher(
                Twist, f"/{rid}/cmd_vel", 10)
            self.create_subscription(
                String, f"/{rid}/state", self.make_state_cb(rid), 10)

    def make_state_cb(self, rid):
        def cb(msg):
            try:
                data = json.loads(msg.data)
                now = time.time()
                if self.on_state_cb is not None:
                    self.on_state_cb(rid, data)
            except Exception as e:
                self.get_logger().error(f"Bad state from {rid}: {e}")
        return cb

    def send_order(self, rid, path):
        if rid not in self.publishers_order:
            return
        msg = String()
        msg.data = json.dumps({"path": path})
        self.publishers_order[rid].publish(msg)

    def send_cmd(self, rid, linear, angular):
        self.get_logger().info(f"send_cmd: rid={rid}, linear={linear}, angular={angular}")
        if rid not in self.publishers_cmd:
            self.get_logger().warn(f"No publisher for {rid}")
            return
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publishers_cmd[rid].publish(msg)
