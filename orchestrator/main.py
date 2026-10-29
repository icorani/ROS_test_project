import asyncio
import json
import math
import os
import yaml
import heapq
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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
ROBOTS_CFG = CFG.get("robots", {})

ALL_NODES = {**NODES, **PARKING}
POS = {**NODES, **PARKING}

ADJ = {n: [] for n in ALL_NODES}
for a, b in EDGES:
    if a in ADJ and b in ADJ:
        ADJ[a].append(b)
        ADJ[b].append(a)

# ── Loading walls ──────────────────────────────────────────
import json as jsonlib

WALLS_PATH = f"/configs/walls_{MAP_NAME}.json"
try:
    with open(WALLS_PATH) as f:
        WALLS_JSON = jsonlib.load(f)
except FileNotFoundError:
    WALLS_JSON = []

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

def nearest_node(x, y):
    return min(POS.keys(), key=lambda n: math.dist((POS[n]["x"], POS[n]["y"]), (x, y)))

# ── Состояния роботов ─────────────────────────────────
robots = {}
for rid, rcfg in ROBOTS_CFG.items():
    robots[rid] = {
        "id": rid,
        "x": rcfg.get("start_x", 0),
        "y": rcfg.get("start_y", 0),
        "yaw": 0.0,
        "status": "idle",
        "mission": [],
        "target": None,
    }

# ── ROS 2 нода ────────────────────────────────────────
class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("orchestrator")
        self.publishers_order = {}
        self.publishers_cmd = {}
        for rid in robots:
            self.publishers_order[rid] = self.create_publisher(String, f"/{rid}/order", 10)
            self.publishers_cmd[rid] = self.create_publisher(Twist, f"/{rid}/cmd_vel", 10)
            self.create_subscription(String, f"/{rid}/state", self.make_state_cb(rid), 10)

    def make_state_cb(self, rid):
        def cb(msg):
            try:
                data = json.loads(msg.data)
                robots[rid].update({
                    "x": data.get("x", 0),
                    "y": data.get("y", 0),
                    "yaw": data.get("yaw", 0),
                    "status": data.get("status", "idle"),
                    "mission": data.get("mission", []),
                    "mission_idx": data.get("mission_idx", 0),
                })
            except Exception as e:
                self.get_logger().error(f"Bad state from {rid}: {e}")
        return cb

    def send_order(self, rid, path):
        if rid in self.publishers_order:
            msg = String()
            msg.data = json.dumps({"path": path})
            self.publishers_order[rid].publish(msg)

    def send_cmd(self, rid, linear, angular):
        self.get_logger().info(f"send_cmd: rid={rid}, linear={linear}, angular={angular}")
        if rid in self.publishers_cmd:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.publishers_cmd[rid].publish(msg)
        else:
            self.get_logger().warn(f"No publisher for {rid}")

rclpy.init()
node = OrchestratorNode()

@app.on_event("startup")
async def startup():
    asyncio.create_task(spin())

async def spin():
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
            "robots": ROBOTS_CFG,
            "walls": WALLS_JSON,   
        }}))
    try:
        while True:
            # Принимаем сообщения от Canvas
            try:
                msg_text = await asyncio.wait_for(ws.receive_text(), timeout=0.1)
                msg = json.loads(msg_text)
                if msg.get("type") == "cmd_vel":
                    rid = msg.get("robot_id")
                    node.send_cmd(rid, msg.get("linear", 0), msg.get("angular", 0))
                if msg.get("type") == "resume":
                    rid = msg.get("robot_id")
                    target = robots.get(rid, {}).get("target")
                    if target:
                        cur = robots[rid]
                        start = nearest_node(cur["x"], cur["y"])
                        path = find_path(start, target)
                        if path:
                            robots[rid]["mission"] = path
                            robots[rid]["status"] = "moving"
                            node.send_order(rid, path)
                    else:
                        robots[rid]["status"] = "idle"
            except asyncio.TimeoutError:
                pass
            # Отправляем состояния всех роботов
            await ws.send_text(json.dumps({"type": "state", "robots": list(robots.values())}))
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass

@app.post("/order")
async def order(payload: dict):
    rid = payload.get("robot_id", "robot_1")
    target = payload.get("target")
    start = payload.get("start")
    if start is None:
        start = robots.get(rid, {}).get("mission", [None])[0] or list(POS.keys())[0]
    path = find_path(start, target)
    if path is None:
        return {"ok": False, "error": "no path"}
    robots[rid]["mission"] = path
    robots[rid]["status"] = "moving"
    robots[rid]["target"] = target
    node.send_order(rid, path)
    return {"ok": True, "path": path}

@app.get("/state")
async def state(pretty: bool = False):
    data = {"robots": list(robots.values())}
    if pretty:
        import json
        return Response(
            content=json.dumps(data, indent=2, ensure_ascii=False),
            media_type="application/json; charset=utf-8"
        )
    return data

@app.get("/health")
async def health():
    return {"status": "ok", "map": MAP_NAME, "robots": list(robots.keys())}