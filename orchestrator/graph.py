"""Загрузка карты (YAML): узлы, рёбра, смежность, координаты, расстояние."""
from __future__ import annotations

import json
import math
import os
from typing import Any

import yaml

MAP_NAME = os.getenv("MAP", "simple")
MAP_PATH = f"/configs/{MAP_NAME}.yaml"


def load_map(path: str = MAP_PATH) -> dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


def load_walls(map_name: str) -> list[Any]:
    path = f"/configs/walls_{map_name}.json"
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []


class Graph:
    def __init__(self, cfg: dict[str, Any]):
        self.name: str = cfg.get("name", MAP_NAME)
        self.bounds: dict[str, Any] = cfg.get("bounds", {})
        self.nodes: dict[str, dict[str, float]] = cfg.get("nodes", {})
        self.parking: dict[str, dict[str, float]] = cfg.get("parking", {})
        self.edges: list[tuple[str, str]] = [tuple(e) for e in cfg.get("edges", [])]
        self.robots_cfg: dict[str, Any] = cfg.get("robots", {}) or {}
        # Узлы + парковка — общая система координат
        self.pos: dict[str, dict[str, float]] = {**self.nodes, **self.parking}
        self.adj: dict[str, list[str]] = {n: [] for n in self.pos}
        for a, b in self.edges:
            if a in self.adj and b in self.adj:
                self.adj[a].append(b)
                self.adj[b].append(a)

    def distance(self, a: str, b: str) -> float:
        if a not in self.pos or b not in self.pos:
            return float("inf")
        pa = self.pos[a]
        pb = self.pos[b]
        return math.dist((pa["x"], pa["y"]), (pb["x"], pb["y"]))

    def nearest_node(self, x: float, y: float) -> str:
        return min(
            self.pos.keys(),
            key=lambda n: math.dist((self.pos[n]["x"], self.pos[n]["y"]), (x, y)),
        )

    def all_nodes(self) -> list[str]:
        return list(self.pos.keys())
