import asyncio
import json
import math
import os
import yaml
import heapq
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# ── Загрузка конфига ──────────────────────────────────
MAP_NAME = os.getenv("MAP", "simple")
MAP_PATH = f"/configs/{MAP_NAME}.yaml"
with open(MAP_PATH) as f:
    CFG = yaml.safe_load(f)

NODES = CFG["nodes"]
PARKING = CFG.get("parking", {})
EDGES = CFG["edges"]
BOUNDS = CFG["bounds"]

# Все узлы (включая парковки)
ALL_NODES = {**NODES, **PARKING}
POS = {**NODES, **PARKING}

# Граф: adjacency list (включая парковки)
ADJ = {n: [] for n in ALL_NODES}
for a, b in EDGES:
    if a in ADJ and b in ADJ:
        ADJ[a].append(b)
        ADJ[b].append(a)

# ── Dijkstra ──────────────────────────────────────────
def find_path(start: str, goal: str) -> list[str] | None:
    if start not in POS or goal not in POS:
        return None
    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, float("inf")):
            continue
        for v in ADJ.get(u, []):
            w = math.dist((POS[u]["x"], POS[u]["y"]), (POS[v]["x"], POS[v]["y"]))
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        return None
    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = prev[cur]
    path.append(start)
    return list(reversed(path))

# ── Состояние робота ──────────────────────────────────
robot_state = {
    "id": "robot_1",
    "x": 0.0, "y": 0.0, "yaw": 0.0,
    "status": "idle",
    "mission": [],
}

class OdomListener(Node):
    def __init__(self):
        super().__init__("odom_listener")
        self.create_subscription(Odometry, "/odom", self.cb, 10)

    def cb(self, msg):
        robot_state["x"] = msg.pose.pose.position.x
        robot_state["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
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
    await ws.send_text(json.dumps({"type": "map", "data": {
        "name": CFG["name"],
        "bounds": BOUNDS,
        "nodes": POS,
        "edges": EDGES,
    }}))
    try:
        while True:
            await ws.send_text(json.dumps({"type": "state", "robots": [robot_state]}))
            await asyncio.sleep(0.1)
    except Exception:
        pass

@app.post("/order")
async def order(payload: dict):
    robot_id = payload.get("robot_id", "robot_1")
    target = payload.get("target")
    start = payload.get("start", "C1")
    path = find_path(start, target)
    if path is None:
        return {"ok": False, "error": "no path"}
    robot_state["mission"] = path
    robot_state["status"] = "moving"
    return {"ok": True, "path": path}

@app.get("/health")
async def health():
    return {"status": "ok", "map": MAP_NAME}