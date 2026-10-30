import asyncio
import json
import math
import os
import yaml
import heapq
import random
import time
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
WALLS_PATH = f"/configs/walls_{MAP_NAME}.json"
try:
    with open(WALLS_PATH) as f:
        WALLS_JSON = json.load(f)
except FileNotFoundError:
    WALLS_JSON = []

# ── Dijkstra ──────────────────────────────────────────
def find_path(start: str, goal: str, blocked_nodes=None, blocked_edges=None, robot_id=None) -> list[str] | None:
    if start not in POS or goal not in POS:
        return None
    if blocked_nodes is None:
        blocked_nodes = set()
    if blocked_edges is None:
        blocked_edges = set()
    now = time.time()
    # Clean expired reservations
    expired = [k for k, v in path_reservations.items() if v["expires"] < now]
    for k in expired:
        del path_reservations[k]
    # Goal cannot be blocked (except by the requesting robot itself)
    if goal in blocked_nodes and blocked_nodes_owner.get(goal) != robot_id:
        return None
    # Goal cannot be reserved by another robot
    goal_res_key = f"node:{goal}"
    if goal_res_key in path_reservations and path_reservations[goal_res_key]["robot_id"] != robot_id:
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
        # Don't expand from blocked intermediate nodes (but start may be blocked)
        if u != start and u in blocked_nodes and blocked_nodes_owner.get(u) != robot_id:
            continue
        u_res_key = f"node:{u}"
        if u != start and u_res_key in path_reservations and path_reservations[u_res_key]["robot_id"] != robot_id:
            continue
        for v in ADJ.get(u, []):
            if v in blocked_nodes and blocked_nodes_owner.get(v) != robot_id:
                continue
            v_res_key = f"node:{v}"
            if v_res_key in path_reservations and path_reservations[v_res_key]["robot_id"] != robot_id:
                continue
            edge_key = tuple(sorted((u, v)))
            if edge_key in blocked_edges and blocked_edges_owner.get(edge_key) != robot_id:
                continue
            e_res_key = f"edge:{edge_key[0]},{edge_key[1]}"
            if e_res_key in path_reservations and path_reservations[e_res_key]["robot_id"] != robot_id:
                continue
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


def reserve_path(robot_id: str, path: list[str]):
    """Reserve all nodes and edges on the path for the robot."""
    now = time.time()
    expires = now + RESERVATION_TTL
    # Reserve nodes
    for node in path:
        key = f"node:{node}"
        path_reservations[key] = {"robot_id": robot_id, "expires": expires}
    # Reserve edges
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        edge_key = tuple(sorted((a, b)))
        rkey = f"edge:{edge_key[0]},{edge_key[1]}"
        path_reservations[rkey] = {"robot_id": robot_id, "expires": expires}


def release_reservations(robot_id: str):
    """Release all path reservations for a robot."""
    keys_to_remove = [k for k, v in path_reservations.items() if v["robot_id"] == robot_id]
    for k in keys_to_remove:
        del path_reservations[k]


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
        "mission_idx": 0,
        "target": None,
        "linear": 0.0,
        "angular": 0.0,
        "error": None,
        "last_update": time.time(),
        "stuck_count": 0,
        "last_x": rcfg.get("start_x", 0),
        "last_y": rcfg.get("start_y", 0),
        "stuck_since": None,
    }

# ── Блокировки ────────────────────────────────────────
blocked_nodes = set()  # permanently blocked (stuck robot)
blocked_edges = set()
stuck_robots = set()  # robots marked as stuck/unrecoverable
robot_blocked_nodes = {}  # robot_id -> set of nodes it blocked (stuck)
robot_blocked_edges = {}  # robot_id -> set of edges it blocked (stuck)
blocked_nodes_owner = {}  # node -> robot_id that blocked it
blocked_edges_owner = {}  # edge -> robot_id that blocked it
path_reservations = {}  # key ("node:C1" or "edge:C1,B2") -> {robot_id, expires}
RESERVATION_TTL = 15.0  # seconds

# ── Dark shop (Тёмный цех) ────────────────────────────
dark_shop_enabled = False
DARK_SHOP_INTERVAL = 3.0  # seconds
last_dark_shop = time.time()

# ── Connection loss tracking ─────────────────────────
CONNECTION_TIMEOUT = 2.5  # seconds without update
THROTTLE_THRESHOLD = 0.5  # warn if updates come too fast

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
                now = time.time()
                prev = robots.get(rid, {})
                robots[rid].update({
                    "x": data.get("x", prev.get("x", 0)),
                    "y": data.get("y", prev.get("y", 0)),
                    "yaw": data.get("yaw", prev.get("yaw", 0)),
                    "status": data.get("status", prev.get("status", "idle")),
                    "mission": data.get("mission", prev.get("mission", [])),
                    "mission_idx": data.get("mission_idx", prev.get("mission_idx", 0)),
                    "linear": data.get("linear", 0.0),
                    "angular": data.get("angular", 0.0),
                    "error": data.get("error"),
                    "last_update": now,
                })
                # Check for stuck
                if robots[rid]["status"] == "moving" and robots[rid].get("mission"):
                    last_x = prev.get("x", robots[rid]["x"])
                    last_y = prev.get("y", robots[rid]["y"])
                    dist_moved = math.hypot(robots[rid]["x"] - last_x, robots[rid]["y"] - last_y)
                    if dist_moved < 0.01:
                        robots[rid]["stuck_count"] = robots[rid].get("stuck_count", 0) + 1
                        if robots[rid].get("stuck_since") is None:
                            robots[rid]["stuck_since"] = now
                    else:
                        robots[rid]["stuck_count"] = 0
                        robots[rid]["stuck_since"] = None
                        if rid in stuck_robots:
                            stuck_robots.discard(rid)
                            for n in robot_blocked_nodes.pop(rid, []):
                                blocked_nodes.discard(n)
                            for e in robot_blocked_edges.pop(rid, []):
                                blocked_edges.discard(e)
                    # If stuck for too long while moving -> mark as stuck/unrecoverable
                    if robots[rid]["stuck_count"] > 40:  # ~2 seconds at 20Hz
                        if rid not in stuck_robots:
                            stuck_robots.add(rid)
                            # Block the node/area where it's stuck
                            stuck_node = nearest_node(robots[rid]["x"], robots[rid]["y"])
                            blocked_nodes.add(stuck_node)
                            for neighbor in ADJ.get(stuck_node, []):
                                edge_key = tuple(sorted((stuck_node, neighbor)))
                                blocked_edges.add(edge_key)
                            self.get_logger().warn(f"Robot {rid} stuck at {stuck_node} - blocking area")
                elif robots[rid]["status"] == "error" and rid not in stuck_robots:
                    # Robot reported itself as stuck (error status)
                    stuck_robots.add(rid)
                    robot_blocked_nodes[rid] = set()
                    robot_blocked_edges[rid] = set()
                    stuck_node = nearest_node(robots[rid]["x"], robots[rid]["y"])
                    blocked_nodes.add(stuck_node)
                    blocked_nodes_owner[stuck_node] = rid
                    robot_blocked_nodes[rid].add(stuck_node)
                    for neighbor in ADJ.get(stuck_node, []):
                        edge_key = tuple(sorted((stuck_node, neighbor)))
                        blocked_edges.add(edge_key)
                        blocked_edges_owner[edge_key] = rid
                        robot_blocked_edges[rid].add(edge_key)
                    self.get_logger().warn(f"Robot {rid} self-reported stuck at {stuck_node} - blocking area")
                elif rid in stuck_robots and robots[rid]["status"] != "error":
                    # Robot recovered from stuck state (moving/idle/done without error)
                    stuck_robots.discard(rid)
                    robots[rid]["stuck_count"] = 0
                    robots[rid]["stuck_since"] = None
                    # Release this robot's blocked nodes/edges
                    for n in robot_blocked_nodes.pop(rid, []):
                        blocked_nodes.discard(n)
                    for e in robot_blocked_edges.pop(rid, []):
                        blocked_edges.discard(e)
                elif robots[rid]["status"] in ["idle", "done"] and rid in stuck_robots and not robots[rid].get("error"):
                    stuck_robots.discard(rid)
                    robots[rid]["stuck_count"] = 0
                    robots[rid]["stuck_since"] = None
                    for n in robot_blocked_nodes.pop(rid, []):
                        blocked_nodes.discard(n)
                    for e in robot_blocked_edges.pop(rid, []):
                        blocked_edges.discard(e)
                # Release path reservations when mission completes
                if robots[rid]["status"] in ["done", "idle"] and prev.get("status") == "moving":
                    release_reservations(rid)
                else:
                    robots[rid]["stuck_count"] = 0
                    robots[rid]["stuck_since"] = None
            except Exception as e:
                self.get_logger().error(f"Bad state from {rid}: {e}")
        return cb

    def send_order(self, rid, path):
        r = robots.get(rid, {})
        if r.get("status") in ["idle", "done"] and rid in stuck_robots:
            stuck_robots.discard(rid)
            robots[rid]["stuck_count"] = 0
            robots[rid]["stuck_since"] = None
            for n in robot_blocked_nodes.pop(rid, []):
                blocked_nodes.discard(n)
            for e in robot_blocked_edges.pop(rid, []):
                blocked_edges.discard(e)
        # Always send order to allow recovery from stuck state
        if rid in self.publishers_order:
            msg = String()
            msg.data = json.dumps({"path": path})
            self.publishers_order[rid].publish(msg)

    def send_cmd(self, rid, linear, angular):
        self.get_logger().info(f"send_cmd: rid={rid}, linear={linear}, angular={angular}")
        if rid in stuck_robots:
            self.get_logger().warn(f"Cannot send cmd to stuck robot {rid}")
            return
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
    asyncio.create_task(monitor_tasks())

async def spin():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        await asyncio.sleep(0.01)

async def monitor_tasks():
    global dark_shop_enabled, last_dark_shop
    while True:
        try:
            now = time.time()
            # Check connection loss
            for rid, r in robots.items():
                if now - r.get("last_update", 0) > CONNECTION_TIMEOUT:
                    if r.get("status") != "disconnected":
                        r["status"] = "disconnected"
                        r["error"] = "connection_lost"
                    # Release path reservations on disconnect
                    release_reservations(rid)
            # Clean expired path reservations
            expired_keys = [k for k, v in path_reservations.items() if v["expires"] < now]
            for k in expired_keys:
                del path_reservations[k]
            # Dark shop
            if dark_shop_enabled and now - last_dark_shop > DARK_SHOP_INTERVAL:
                await do_dark_shop()
                last_dark_shop = now
        except Exception as e:
            print(f"Monitor error: {e}")
        await asyncio.sleep(0.5)

async def do_dark_shop():
    global dark_shop_enabled
    # Give random long-distance orders to idle robots
    parking_nodes = list(PARKING.keys()) if PARKING else list(ALL_NODES.keys())
    for rid, r in robots.items():
        if rid in stuck_robots:
            continue
        if r.get("status") in ["idle", "done"] and (time.time() - r.get("last_update", 0) < CONNECTION_TIMEOUT):
            # Pick random distant target
            candidates = [n for n in parking_nodes + list(NODES.keys()) if n != nearest_node(r["x"], r["y"])]
            if not candidates:
                continue
            target = random.choice(candidates)
            cur = nearest_node(r["x"], r["y"])
            path = find_path(cur, target, blocked_nodes=blocked_nodes, blocked_edges=blocked_edges, robot_id=rid)
            if path and len(path) > 2:  # Make it a long trip
                r["mission"] = path
                r["status"] = "moving"
                r["target"] = target
                reserve_path(rid, path)
                node.send_order(rid, path)
                break  # Send to one robot at a time

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    global dark_shop_enabled
    await ws.accept()
    await ws.send_text(json.dumps({"type": "map", "data": {
                    "name": CFG["name"],
                    "bounds": BOUNDS,
                    "nodes": POS,
                    "edges": EDGES,
                    "robots": ROBOTS_CFG,
                    "walls": WALLS_JSON,
                    "blocked_nodes": list(blocked_nodes),
                    "blocked_edges": [list(e) for e in blocked_edges],
                    "dark_shop": dark_shop_enabled,
                    "stuck_robots": list(stuck_robots),
                    "reserved_nodes": {k[5:]: v["robot_id"] for k, v in path_reservations.items() if k.startswith("node:")},
                    "reserved_edges": {k[5:]: v["robot_id"] for k, v in path_reservations.items() if k.startswith("edge:")},
                }}))
    try:
        while True:
            try:
                msg_text = await asyncio.wait_for(ws.receive_text(), timeout=0.1)
                msg = json.loads(msg_text)
                if msg.get("type") == "cmd_vel":
                    rid = msg.get("robot_id")
                    node.send_cmd(rid, msg.get("linear", 0), msg.get("angular", 0))
                elif msg.get("type") == "resume":
                    rid = msg.get("robot_id")
                    target = robots.get(rid, {}).get("target")
                    if target and robots[rid].get("status") != "disconnected":
                        cur = nearest_node(robots[rid]["x"], robots[rid]["y"])
                        path = find_path(cur, target, blocked_nodes=blocked_nodes, blocked_edges=blocked_edges, robot_id=rid)
                        if path:
                            robots[rid]["mission"] = path
                            robots[rid]["status"] = "moving"
                            reserve_path(rid, path)
                            node.send_order(rid, path)
                    else:
                        robots[rid]["status"] = "idle"
                elif msg.get("type") == "dark_shop":
                    dark_shop_enabled = bool(msg.get("enabled", False))
                    await ws.send_text(json.dumps({"type": "dark_shop_status", "enabled": dark_shop_enabled}))
                elif msg.get("type") == "clear_stuck":
                    rid = msg.get("robot_id")
                    if rid in stuck_robots:
                        stuck_robots.discard(rid)
                    # Don't auto-clear blocked nodes/edges as per requirement
            except asyncio.TimeoutError:
                pass
            await ws.send_text(json.dumps({
                "type": "state", 
                "robots": list(robots.values()),
                "blocked_nodes": list(blocked_nodes),
                "blocked_edges": [list(e) for e in blocked_edges],
                "dark_shop": dark_shop_enabled,
                "stuck_robots": list(stuck_robots),
                "reserved_nodes": {k[5:]: v["robot_id"] for k, v in path_reservations.items() if k.startswith("node:")},
                "reserved_edges": {k[5:]: v["robot_id"] for k, v in path_reservations.items() if k.startswith("edge:")},
            }))
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
    # Release stuck robot's blocked resources before pathfinding so it can recover
    if rid in stuck_robots:
        stuck_robots.discard(rid)
        robots[rid]["stuck_count"] = 0
        robots[rid]["stuck_since"] = None
        robots[rid]["error"] = None
        for n in robot_blocked_nodes.pop(rid, []):
            blocked_nodes.discard(n)
        for e in robot_blocked_edges.pop(rid, []):
            blocked_edges.discard(e)
    if time.time() - robots.get(rid, {}).get("last_update", 0) > CONNECTION_TIMEOUT:
        return {"ok": False, "error": "connection_lost"}
    if start is None:
        r = robots.get(rid, {})
        if r.get("mission"):
            start = r["mission"][min(r.get("mission_idx", 0), len(r["mission"])-1)]
        else:
            start = nearest_node(r.get("x", 0), r.get("y", 0))
    path = find_path(start, target, blocked_nodes=blocked_nodes, blocked_edges=blocked_edges, robot_id=rid)
    if path is None:
        return {"ok": False, "error": "no_path"}
    robots[rid]["mission"] = path
    robots[rid]["status"] = "moving"
    robots[rid]["target"] = target
    release_reservations(rid)
    reserve_path(rid, path)
    node.send_order(rid, path)
    return {"ok": True, "path": path}

@app.get("/state")
async def state(pretty: bool = False):
    data = {"robots": list(robots.values()), "blocked_nodes": list(blocked_nodes), "dark_shop": dark_shop_enabled}
    if pretty:
        return Response(
            content=json.dumps(data, indent=2, ensure_ascii=False),
            media_type="application/json; charset=utf-8"
        )
    return data

@app.get("/health")
async def health():
    return {"status": "ok", "map": MAP_NAME, "robots": list(robots.keys()), "dark_shop": dark_shop_enabled}
