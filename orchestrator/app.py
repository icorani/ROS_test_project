"""FastAPI приложение orkestrators: маршруты, WS, цикл движения и блокировок."""

from __future__ import annotations

import asyncio
import json
import math
import os
import random
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware

import rclpy

from graph import load_map, load_walls, Graph
from locks import LockManager
from pathfinder import PathFinder
from robots import RobotState, RobotRegistry
from ros_node import OrchestratorNode
import stuck as stuck_mod

app = FastAPI()
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

# ── Константы ────────────────────────────────────────────────────────
CONNECTION_TIMEOUT = 2.5  # сек без обновления -> disconnected
WAIT_TIMEOUT = 30.0  # сек ожидания освобождения ресурса
NODE_ARRIVAL_THRESHOLD = 0.3  # м, считаем роботом у узла
DARK_SHOP_INTERVAL = 3.0  # сек

# ── Загрузка карты ───────────────────────────────────────────────────
CFG = load_map()
MAP_NAME = CFG.get("name", os.getenv("MAP", "simple"))
BOUNDS = CFG.get("bounds", {})
WALLS_JSON = load_walls(MAP_NAME)

GRAPH = Graph(CFG)
POS = GRAPH.pos
ADJ = GRAPH.adj
ALL_NODES = GRAPH.all_nodes()
ROBOTS_CFG = GRAPH.robots_cfg
PARKING = GRAPH.parking

# Если секции robots нет или пуста — генерируем из parking
if not ROBOTS_CFG:
    sorted_parking = sorted(PARKING.items(), key=lambda item: item[0])
    for i, (pid, pos_) in enumerate(sorted_parking):
        ROBOTS_CFG[f"robot_{i+1}"] = {
            "start_node": pid,
            "start_x": pos_["x"],
            "start_y": pos_["y"],
            "start_yaw": 0,
            "color": "#4af",
            "home_parking": pid,
        }

NUM_ROBOTS_ENV = os.getenv("NUM_ROBOTS", "")
if NUM_ROBOTS_ENV:
    try:
        NUM_ROBOTS_LIMIT = int(NUM_ROBOTS_ENV)
    except ValueError:
        NUM_ROBOTS_LIMIT = len(ROBOTS_CFG)
else:
    NUM_ROBOTS_LIMIT = len(ROBOTS_CFG)
NUM_ROBOTS_LIMIT = min(max(NUM_ROBOTS_LIMIT, 0), len(ROBOTS_CFG))

rclpy.init()
NODE = OrchestratorNode()

# ── Единые структуры ─────────────────────────────────────────────────
LOCKS = LockManager()
PATHFINDER = PathFinder(GRAPH)
REGISTRY = RobotRegistry()

for i, (rid, rcfg) in enumerate(ROBOTS_CFG.items()):
    is_active = i < NUM_ROBOTS_LIMIT
    REGISTRY.add(
        RobotState(
            id=rid,
            x=rcfg.get("start_x", 0),
            y=rcfg.get("start_y", 0),
            yaw=rcfg.get("start_yaw", 0),
            status="idle" if is_active else "inactive",
            mission=[],
            mission_idx=0,
            target=None,
            home_parking=rcfg.get("home_parking"),
            is_active=is_active,
            color=rcfg.get("color", "#4af"),
            start_node=rcfg.get("start_node"),
            start_x=rcfg.get("start_x", 0),
            start_y=rcfg.get("start_y", 0),
            start_yaw=rcfg.get("start_yaw", 0),
        )
    )

NODE.bind_robots(dict(REGISTRY.items()))

# ── Темный цех ───────────────────────────────────────────────────────
dark_shop_enabled = False
last_dark_shop = time.time()

# ── Контекст движения роботов ────────────────────────────────────────
# rid -> {"seg_cur": str|None, "seg_nxt": str|None, "at_node": bool, "waiting": bool, "wait_since": float|None}
MOVEMENT_CTX: dict[str, dict] = {}

# ── Вспомогательные ──────────────────────────────────────────────────


def nearest_node(x: float, y: float) -> str:
    return GRAPH.nearest_node(x, y)


def node_dist(x: float, y: float, node: str) -> float:
    p = POS[node]
    return math.hypot(x - p["x"], y - p["y"])


def release_all_for_robot(rid: str) -> None:
    LOCKS.release_all_for_robot(rid)
    MOVEMENT_CTX.pop(rid, None)


def ws_blocked_snapshot():
    """Совместимый с WS/HTTP снимок блокировок из LockManager."""
    nodes: list[str] = []
    edges: list[list[str]] = []
    reserved_nodes: dict[str, str] = {}
    reserved_edges: dict[str, str] = {}
    occupied_nodes: dict[str, str] = {}
    now = time.time()
    for key, lock in list(LOCKS._locks.items()):
        if lock.expires is not None and now > lock.expires:
            del LOCKS._locks[key]
            continue
        owner = lock.owner or ""
        if lock.resource_type == "node":
            name = key[len("node:"):]
            nodes.append(name)
            reserved_nodes[name] = owner
            occupied_nodes[name] = owner
        elif lock.resource_type == "edge":
            a, b = LOCKS.edge_from_key(key)
            edges.append([a, b])
            reserved_edges[f"{a},{b}"] = owner
    return nodes, edges, reserved_nodes, reserved_edges, occupied_nodes


# ── Callbacks ROS ────────────────────────────────────────────────────


def on_state(rid: str, state: dict) -> None:
    """Вызывается из ros_node при получении /{rid}/state."""
    r = REGISTRY.get(rid)
    if not r:
        return
    now = time.time()
    r.x = state.get("x", r.x)
    r.y = state.get("y", r.y)
    r.yaw = state.get("yaw", r.yaw)
    r.status = state.get("status", r.status)
    r.mission = list(state.get("mission", r.mission))
    r.mission_idx = int(state.get("mission_idx", r.mission_idx))
    r.target = state.get("target", r.target)
    r.linear = state.get("linear", r.linear)
    r.angular = state.get("angular", r.angular)
    r.error = state.get("error", r.error)
    r.last_update = now

    # Застрявший робот (самоошибка) — блокируем узел где стоит (постоянно)
    if r.status == "error":
        cur = nearest_node(r.x, r.y)
        LOCKS.lock_node(cur, rid, reason="stuck")
        MOVEMENT_CTX[rid] = {
            "seg_cur": cur,
            "seg_nxt": None,
            "at_node": True,
            "waiting": False,
            "wait_since": None,
        }
        return

    # Восстановление после ошибки/ручного управления -> сброс блокировок
    if r.status in ("idle", "done") and not r.mission:
        release_all_for_robot(rid)
        r.error = None
        return

    # Движение: reconciliate блокировки
    if r.status == "moving" and r.mission:
        _reconcile_movement(rid, r, now)

NODE.on_state_cb = on_state


def _reconcile_movement(rid: str, r: RobotState, now: float) -> None:
    """Выставить/освободить блокировки для робота в движении.

    Семитика (по плану):
      * На узле: блокируем узел + ребро к следующему + целевой узел.
      * На ребре: узел отправления освобождается, держим ребро + целевой узел.
      * При въезде в узел: освобождаем предыдущее ребро, блокируем узел и следующее ребро.
    """
    mission = r.mission
    idx = r.mission_idx
    if not mission:
        return

    x, y = r.x, r.y
    nearest = nearest_node(x, y)
    at_node = node_dist(x, y, nearest) < NODE_ARRIVAL_THRESHOLD

    # Определяем текущий сегмент движения: (from_node -> to_node)
    if at_node and idx < len(mission) and mission[idx] == nearest:
        # Стоим на текущем целевом узле — едем к следующему
        seg_cur = mission[idx]
        seg_nxt = mission[idx + 1] if idx + 1 < len(mission) else mission[idx]
    elif at_node:
        # Стоим на узле, но цель — mission[idx] (индекс не синхронизирован)
        seg_cur = nearest
        seg_nxt = mission[idx] if idx < len(mission) else nearest
    else:
        # Едем по ребру к mission[idx], приехали с mission[idx-1] (или с nearest)
        seg_cur = mission[idx - 1] if idx > 0 else nearest
        seg_nxt = mission[idx] if idx < len(mission) else nearest

    # Миссия завершена: доехали до последнего узла и стоим на нём
    if idx >= len(mission) - 1 and at_node and nearest == mission[-1]:
        release_all_for_robot(rid)
        r.status = "done"
        r.mission = []
        r.mission_idx = 0
        r.target = None
        MOVEMENT_CTX.pop(rid, None)
        return

    ctx = MOVEMENT_CTX.get(
        rid, {"seg_cur": None, "seg_nxt": None, "at_node": False, "waiting": False, "wait_since": None}
    )

    # Ждём освобождения ресурса?
    if ctx.get("waiting"):
        ok = _acquire_segment_locks(rid, seg_cur, seg_nxt, lock_cur_node=at_node)
        if ok:
            ctx["waiting"] = False
            ctx["wait_since"] = None
        else:
            if now - (ctx.get("wait_since") or now) > WAIT_TIMEOUT:
                new_path = PATHFINDER.find_path(seg_cur, r.target or seg_nxt, LOCKS, rid)
                if new_path:
                    _adopt_path(rid, r, new_path)
                    ctx = MOVEMENT_CTX.get(rid, ctx)
                else:
                    ctx["wait_since"] = ctx.get("wait_since") or now
            MOVEMENT_CTX[rid] = ctx
            return

    # Освобождаем ресурсы предыдущего сегмента
    prev_cur = ctx.get("seg_cur")
    prev_nxt = ctx.get("seg_nxt")
    prev_at_node = ctx.get("at_node", False)
    if prev_cur and prev_nxt:
        # Ребро освобождаем при смене сегмента
        if prev_cur != seg_cur or prev_nxt != seg_nxt:
            LOCKS.release_edge(prev_cur, prev_nxt)
        # Узел освобождаем, как только уехали с него на ребро
        if prev_at_node and not at_node:
            LOCKS.release_node(prev_cur)

    ctx["seg_cur"] = seg_cur
    ctx["seg_nxt"] = seg_nxt
    ctx["at_node"] = at_node

    if at_node and seg_cur == seg_nxt:
        # Стоим на узле, без перехода (последний узел или старт)
        ok = LOCKS.lock_node(seg_cur, rid, reason="node")
    elif at_node:
        # На узле: узел + ребро к следующему + целевой узел
        ok = _acquire_segment_locks(rid, seg_cur, seg_nxt, lock_cur_node=True)
    else:
        # На ребре: узел отправления уже освобождён, только ребро + целевой узел
        ok = _acquire_segment_locks(rid, seg_cur, seg_nxt, lock_cur_node=False)
    MOVEMENT_CTX[rid] = ctx
    if not ok:
        # Ресурс занят другим -> перестроить или ждать
        new_path = PATHFINDER.find_path(seg_cur, r.target or seg_nxt, LOCKS, rid)
        if new_path:
            _adopt_path(rid, r, new_path)
        else:
            ctx["waiting"] = True
            ctx["wait_since"] = now
            MOVEMENT_CTX[rid] = ctx


def _acquire_segment_locks(rid: str, frm: str, to: str, lock_cur_node: bool = True) -> bool:
    """Заблокировать [узел frm,] ребро frm-to, целевой узел to для rid."""
    if lock_cur_node:
        if not LOCKS.lock_node(frm, rid, reason="node"):
            return False
    if not LOCKS.lock_edge(frm, to, rid, reason="edge"):
        if lock_cur_node:
            LOCKS.release_node(frm)
        return False
    if not LOCKS.lock_target(to, rid, reason="target"):
        if lock_cur_node:
            LOCKS.release_node(frm)
        LOCKS.release_edge(frm, to)
        return False
    return True


def _adopt_path(rid: str, r: RobotState, path: list[str]) -> None:
    # Не сбрасываем блокировки полностью: стартовый узел может быть уже
    # заблокирован роботом в order(); он перезапишется тем же владельцем
    # при первом _reconcile_movement. Сбрасываем только контекст движения.
    MOVEMENT_CTX.pop(rid, None)
    r.mission = path
    r.mission_idx = 0
    r.status = "moving"
    if path:
        r.target = path[-1]
    NODE.send_order(rid, path)


# ── HTTP API ─────────────────────────────────────────────────────────


@app.get("/health")
async def health():
    return {
        "status": "ok",
        "map": MAP_NAME,
        "robots": list(REGISTRY.keys()),
        "dark_shop": dark_shop_enabled,
    }


@app.get("/state")
async def state(pretty: bool = False):
    nodes, edges, reserved_nodes, reserved_edges, occupied_nodes = ws_blocked_snapshot()
    data = {
        "robots": [r.to_dict() for r in REGISTRY.values()],
        "blocked_nodes": nodes,
        "blocked_edges": edges,
        "dark_shop": dark_shop_enabled,
        "stuck_robots": [rid for rid, r in REGISTRY.items() if r.status == "error"],
        "reserved_nodes": reserved_nodes,
        "reserved_edges": reserved_edges,
        "occupied_nodes": occupied_nodes,
    }
    if pretty:
        return Response(
            content=json.dumps(data, indent=2, ensure_ascii=False),
            media_type="application/json; charset=utf-8",
        )
    return data


@app.post("/order")
async def order(payload: dict):
    rid = payload.get("robot_id", "robot_1")
    target = payload.get("target")
    start = payload.get("start")
    r = REGISTRY.get(rid)
    if not r or not r.is_active:
        return {"ok": False, "error": "robot_inactive"}

    # Новая миссия -> сбрасываем старые блокировки робота (в т.ч. застрявшего)
    release_all_for_robot(rid)
    r.error = None

    if time.time() - r.last_update > CONNECTION_TIMEOUT:
        return {"ok": False, "error": "connection_lost"}

    if start is None:
        if r.mission:
            start = r.mission[min(r.mission_idx, len(r.mission) - 1)]
        else:
            start = nearest_node(r.x, r.y)

    # Блокируем только стартовый узел (цель и ребра — по мере движения)
    LOCKS.lock_node(start, rid, reason="start")

    path = PATHFINDER.find_path(start, target, LOCKS, rid)
    if path is None:
        LOCKS.release_node(start)
        return {"ok": False, "error": "no_path"}

    _adopt_path(rid, r, path)
    return {"ok": True, "path": path}


@app.post("/go_home")
async def go_home(payload: dict):
    rid = payload.get("robot_id", "robot_1")
    r = REGISTRY.get(rid)
    if not r or not r.is_active:
        return {"ok": False, "error": "robot_inactive"}
    home = r.home_parking
    if not home:
        return {"ok": False, "error": "no_home_parking"}

    release_all_for_robot(rid)
    r.error = None

    if time.time() - r.last_update > CONNECTION_TIMEOUT:
        return {"ok": False, "error": "connection_lost"}

    cur = nearest_node(r.x, r.y)
    path = PATHFINDER.find_path(cur, home, LOCKS, rid)
    if path is None:
        return {"ok": False, "error": "no_path"}

    _adopt_path(rid, r, path)
    return {"ok": True, "path": path, "home": home}


# ── WebSocket ────────────────────────────────────────────────────────


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    global dark_shop_enabled, last_dark_shop
    await ws.accept()
    nodes, edges, reserved_nodes, reserved_edges, occupied_nodes = ws_blocked_snapshot()
    await ws.send_text(
        json.dumps(
            {
                "type": "map",
                "data": {
                    "name": CFG.get("name", MAP_NAME),
                    "bounds": BOUNDS,
                    "nodes": POS,
                    "edges": [list(e) for e in GRAPH.edges],
                    "robots": ROBOTS_CFG,
                    "walls": WALLS_JSON,
                    "blocked_nodes": nodes,
                    "blocked_edges": edges,
                    "dark_shop": dark_shop_enabled,
                    "stuck_robots": [rid for rid, r in REGISTRY.items() if r.status == "error"],
                    "reserved_nodes": reserved_nodes,
                    "reserved_edges": reserved_edges,
                    "occupied_nodes": occupied_nodes,
                },
            }
        )
    )
    try:
        while True:
            try:
                msg_text = await asyncio.wait_for(ws.receive_text(), timeout=0.1)
                msg = json.loads(msg_text)
                mtype = msg.get("type")

                if mtype == "cmd_vel":
                    rid = msg.get("robot_id")
                    r = REGISTRY.get(rid)
                    if not r or not r.is_active:
                        continue
                    NODE.send_cmd(rid, msg.get("linear", 0), msg.get("angular", 0))
                    r.status = "manual"
                    r.error = None
                    # Ручное управление -> сброс блокировок
                    release_all_for_robot(rid)

                elif mtype == "resume":
                    rid = msg.get("robot_id")
                    r = REGISTRY.get(rid)
                    if not r:
                        continue
                    release_all_for_robot(rid)
                    r.status = "idle"
                    r.error = None
                    r.mission = []
                    r.mission_idx = 0
                    r.target = None

                elif mtype == "dark_shop":
                    dark_shop_enabled = bool(msg.get("enabled", False))
                    await ws.send_text(
                        json.dumps(
                            {"type": "dark_shop_status", "enabled": dark_shop_enabled}
                        )
                    )

                elif mtype == "clear_stuck":
                    rid = msg.get("robot_id")
                    r = REGISTRY.get(rid)
                    if not r:
                        continue
                    release_all_for_robot(rid)
                    r.status = "idle"
                    r.error = None
                    r.mission = []
                    r.mission_idx = 0
                    r.target = None

                elif mtype == "go_home":
                    rid = msg.get("robot_id")
                    r = REGISTRY.get(rid)
                    if not r or not r.is_active:
                        continue
                    home = r.home_parking
                    if not home:
                        continue
                    release_all_for_robot(rid)
                    r.error = None
                    cur = nearest_node(r.x, r.y)
                    path = PATHFINDER.find_path(cur, home, LOCKS, rid)
                    if path:
                        _adopt_path(rid, r, path)

            except asyncio.TimeoutError:
                pass

            # Отправка состояния
            nodes, edges, reserved_nodes, reserved_edges, occupied_nodes = ws_blocked_snapshot()
            await ws.send_text(
                json.dumps(
                    {
                        "type": "state",
                        "robots": [r.to_dict() for r in REGISTRY.values()],
                        "blocked_nodes": nodes,
                        "blocked_edges": edges,
                        "dark_shop": dark_shop_enabled,
                        "stuck_robots": [rid for rid, r in REGISTRY.items() if r.status == "error"],
                        "reserved_nodes": reserved_nodes,
                        "reserved_edges": reserved_edges,
                        "occupied_nodes": occupied_nodes,
                    }
                )
            )
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


# ── Фоновые задачи ───────────────────────────────────────────────────


async def spin():
    while rclpy.ok():
        rclpy.spin_once(NODE, timeout_sec=0.05)
        await asyncio.sleep(0.01)


async def monitor_tasks():
    global dark_shop_enabled, last_dark_shop
    while True:
        try:
            now = time.time()
            # Потеря связи
            for rid, r in REGISTRY.items():
                if now - r.last_update > CONNECTION_TIMEOUT:
                    if r.status != "disconnected":
                        r.status = "disconnected"
                        r.error = "connection_lost"
                    release_all_for_robot(rid)

            # Застревание (автообнаружение по отсутствию движения)
            for rid, r in REGISTRY.items():
                if r.status == "moving" and r.mission:
                    result = stuck_mod.update_stuck(r, now)
                    if result == "stuck":
                        cur = nearest_node(r.x, r.y)
                        LOCKS.lock_node(cur, rid, reason="stuck")
                        MOVEMENT_CTX[rid] = {
                            "seg_cur": cur,
                            "seg_nxt": None,
                            "at_node": True,
                            "waiting": False,
                            "wait_since": None,
                        }

            # Темный цех
            if dark_shop_enabled and now - last_dark_shop > DARK_SHOP_INTERVAL:
                await do_dark_shop()
                last_dark_shop = now
        except Exception as e:
            print(f"Monitor error: {e}")
        await asyncio.sleep(0.5)


async def do_dark_shop():
    parking_nodes = list(PARKING.keys()) if PARKING else ALL_NODES
    for rid, r in REGISTRY.items():
        if not r.is_active:
            continue
        if r.status == "error":
            continue
        if r.status in ("idle", "done") and (
            time.time() - r.last_update < CONNECTION_TIMEOUT
        ):
            candidates = [n for n in parking_nodes + ALL_NODES if n != nearest_node(r.x, r.y)]
            if not candidates:
                continue
            target = random.choice(candidates)
            cur = nearest_node(r.x, r.y)
            path = PATHFINDER.find_path(cur, target, LOCKS, rid)
            if path and len(path) > 2:
                r.mission = path
                r.status = "moving"
                r.target = target
                NODE.send_order(rid, path)
                break  # одного робота за раз


@app.on_event("startup")
async def startup():
    asyncio.create_task(spin())
    asyncio.create_task(monitor_tasks())
