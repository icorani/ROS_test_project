"""Единая структура блокировок узлов/рёбер/целевых узлов.

"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class Lock:
    resource_type: str  # 'node' | 'edge' | 'target'
    resource_key: str   # 'node:C1' | 'edge:A,B' | 'target:C5'
    owner: Optional[str]  # robot_id, None = свободен
    reason: str
    expires: Optional[float] = None  # None = постоянная блокировка (застрявший)


class LockManager:
    def __init__(self) -> None:
        self._locks: dict[str, Lock] = {}

    # ---------------------------------------------------------------- ключи
    @staticmethod
    def node_key(node: str) -> str:
        return f"node:{node}"

    @staticmethod
    def edge_key(a: str, b: str) -> str:
        # Каноничный (отсортированный) ключ неориентированного ребра
        a, b = sorted((a, b))
        return f"edge:{a},{b}"

    @staticmethod
    def target_key(node: str) -> str:
        return f"target:{node}"

    @staticmethod
    def edge_from_key(key: str) -> tuple[str, str]:
        _, rest = key.split(":", 1)
        a, b = rest.split(",", 1)
        return a, b

    # ---------------------------------------------------------------- взятие
    def lock_node(self, node: str, robot_id: str, reason: str = "node",
                  expires: Optional[float] = None) -> bool:
        return self._take(self.node_key(node), "node", robot_id, reason, expires)

    def lock_edge(self, a: str, b: str, robot_id: str, reason: str = "edge",
                  expires: Optional[float] = None) -> bool:
        return self._take(self.edge_key(a, b), "edge", robot_id, reason, expires)

    def lock_target(self, node: str, robot_id: str, reason: str = "target",
                    expires: Optional[float] = None) -> bool:
        return self._take(self.target_key(node), "target", robot_id, reason, expires)

    def _take(self, key: str, rtype: str, robot_id: str, reason: str,
              expires: Optional[float]) -> bool:
        existing = self._locks.get(key)
        if existing is not None:
            if existing.owner == robot_id:
                existing.reason = reason
                existing.expires = expires
                return True
            return False  # занят другим роботом
        self._locks[key] = Lock(rtype, key, robot_id, reason, expires)
        return True

    # ---------------------------------------------------------------- снятие
    def release_node(self, node: str) -> None:
        self._locks.pop(self.node_key(node), None)

    def release_edge(self, a: str, b: str) -> None:
        self._locks.pop(self.edge_key(a, b), None)

    def release_target(self, node: str) -> None:
        self._locks.pop(self.target_key(node), None)

    def release_all_for_robot(self, robot_id: str) -> None:
        keys = [k for k, v in self._locks.items() if v.owner == robot_id]
        for k in keys:
            del self._locks[k]

    def release_all(self) -> None:
        self._locks.clear()

    def _drop_expired(self, key: str) -> bool:
        lock = self._locks.get(key)
        if lock is None:
            return False
        if lock.expires is not None and time.time() > lock.expires:
            del self._locks[key]
            return False
        return True

    # ---------------------------------------------------------------- запросы
    def is_locked(self, resource_key: str) -> bool:
        return self._drop_expired(resource_key)

    def lock_owner(self, resource_key: str) -> Optional[str]:
        if not self._drop_expired(resource_key):
            return None
        return self._locks[resource_key].owner

    def is_locked_by(self, resource_key: str, robot_id: str) -> bool:
        return self.lock_owner(resource_key) == robot_id

    def is_locked_by_other(self, resource_key: str, robot_id: str) -> bool:
        owner = self.lock_owner(resource_key)
        return owner is not None and owner != robot_id

    def get_locks_by_robot(self, robot_id: str) -> list[Lock]:
        return [v for v in self._locks.values() if v.owner == robot_id]

    # ---------------------------------------------------------------- экспорт
    def locked_nodes(self) -> dict[str, str]:
        """{имя узла: owner} для всех активных блокировок узлов."""
        result: dict[str, str] = {}
        now = time.time()
        for key, lock in list(self._locks.items()):
            if lock.resource_type != "node":
                continue
            if lock.expires is not None and now > lock.expires:
                del self._locks[key]
                continue
            result[key[len("node:"):]] = lock.owner
        return result

    def locked_edges(self) -> dict[tuple[str, str], str]:
        """{(a, b): owner} для всех активных блокировок рёбер."""
        result: dict[tuple[str, str], str] = {}
        now = time.time()
        for key, lock in list(self._locks.items()):
            if lock.resource_type != "edge":
                continue
            if lock.expires is not None and now > lock.expires:
                del self._locks[key]
                continue
            result[self.edge_from_key(key)] = lock.owner
        return result

    def all_blocked(self) -> tuple[list[str], list[list[str]]]:
        """(blocked_nodes, blocked_edges) для совместимого WS/HTTP формата."""
        nodes: list[str] = []
        edges: list[list[str]] = []
        now = time.time()
        for key, lock in list(self._locks.items()):
            if lock.expires is not None and now > lock.expires:
                del self._locks[key]
                continue
            if lock.resource_type == "node":
                nodes.append(key[len("node:"):])
            elif lock.resource_type == "edge":
                a, b = self.edge_from_key(key)
                edges.append([a, b])
        return nodes, edges
