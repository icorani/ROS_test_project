"""Поиск пути (Dijkstra) с учётом блокировок LockManager.
Изолированный, импортируемый алгоритм. Не depende от FastAPI/ROS.
"""

from __future__ import annotations

import heapq
from typing import Optional

from graph import Graph
from locks import LockManager


class PathFinder:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def find_path(self, start: str, goal: str, lock_manager: LockManager,
                  robot_id: str) -> Optional[list[str]]:
        """Возвращает список узлов от start до goal или None.

        - Не расширяется из промежуточного узла, залоченного другим роботом.
        - Не заходит в целевой узел, если он залочен другим роботом.
        - Не проходит через рёбра, залоченные другим роботом.
        - Собственные локи робота игнорируются.
        """
        pos = self.graph.pos
        adj = self.graph.adj
        if start not in pos or goal not in pos:
            return None
        if start == goal:
            return [start]

        dist: dict[str, float] = {start: 0.0}
        prev: dict[str, str] = {}
        pq: list[tuple[float, str]] = [(0.0, start)]
        visited: set[str] = set()

        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            if u == goal:
                break
            # Не расширяемся из заблокированного промежуточного узла другого робота.
            if u != start and lock_manager.is_locked_by_other(
                    LockManager.node_key(u), robot_id):
                continue
            for v in adj.get(u, []):
                if v == start:
                    continue
                # Целевой узел занят другим роботом.
                if v == goal and lock_manager.is_locked_by_other(
                        LockManager.node_key(v), robot_id):
                    continue
                # Промежуточный узел занят другим роботом.
                if v != goal and lock_manager.is_locked_by_other(
                        LockManager.node_key(v), robot_id):
                    continue
                # Ребро заблокировано другим роботом.
                if lock_manager.is_locked_by_other(
                        LockManager.edge_key(u, v), robot_id):
                    continue
                w = self.graph.distance(u, v)
                nd = d + w
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if goal not in dist:
            return None
        path: list[str] = []
        cur = goal
        while cur != start:
            path.append(cur)
            cur = prev[cur]
        path.append(start)
        return list(reversed(path))
