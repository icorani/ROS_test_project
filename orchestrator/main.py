import asyncio
import json
import math
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# Глобальное состояние робота
robot_state = {"x": 0.0, "y": 0.0, "yaw": 0.0, "status": "idle"}


class OdomListener(Node):
    def __init__(self):
        super().__init__("odom_listener")
        self.create_subscription(Odometry, "/odom", self.callback, 10)

    def callback(self, msg):
        robot_state["x"] = msg.pose.pose.position.x
        robot_state["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # кватернион → yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_state["yaw"] = math.atan2(siny, cosy)


@app.on_event("startup")
async def startup():
    rclpy.init()
    node = OdomListener()
    asyncio.create_task(spin(node))


async def spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        await asyncio.sleep(0.01)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_text(json.dumps({"robots": [{"id": "r1", **robot_state}]}))
            await asyncio.sleep(0.1)  # 10 Гц
    except Exception:
        pass


@app.get("/health")
async def health():
    return {"status": "ok"}