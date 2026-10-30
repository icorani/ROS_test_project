"""Состояния роботов: RobotState (dataclass) + RobotRegistry."""

from __future__ import annotations

from dataclasses import dataclass, field
import time


@dataclass
class RobotState:
    id: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    status: str = "idle"          # idle | moving | done | manual | error | disconnected
    mission: list[str] = field(default_factory=list)
    mission_idx: int = 0
    target: str | None = None
    linear: float = 0.0
    angular: float = 0.0
    error: str | None = None
    last_update: float = field(default_factory=time.time)
    stuck_count: int = 0
    stuck_since: float | None = None
    last_x: float = 0.0
    last_y: float = 0.0
    home_parking: str | None = None
    is_active: bool = True
    color: str = "#4af"
    start_node: str | None = None
    start_x: float = 0.0
    start_y: float = 0.0
    start_yaw: float = 0.0

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "status": self.status,
            "mission": list(self.mission),
            "mission_idx": self.mission_idx,
            "target": self.target,
            "linear": self.linear,
            "angular": self.angular,
            "error": self.error,
            "last_update": self.last_update,
            "stuck_count": self.stuck_count,
            "stuck_since": self.stuck_since,
            "last_x": self.last_x,
            "last_y": self.last_y,
            "home_parking": self.home_parking,
            "is_active": self.is_active,
        }


class RobotRegistry:
    """Хранилище состояний роботов с dict-подобным интерфейсом."""

    def __init__(self) -> None:
        self._robots: dict[str, RobotState] = {}

    def add(self, state: RobotState) -> None:
        self._robots[state.id] = state

    def get(self, rid: str) -> RobotState | None:
        return self._robots.get(rid)

    def __contains__(self, rid: str) -> bool:
        return rid in self._robots

    def __getitem__(self, rid: str) -> RobotState:
        return self._robots[rid]

    def __setitem__(self, rid: str, state: RobotState) -> None:
        self._robots[rid] = state

    def __delitem__(self, rid: str) -> None:
        del self._robots[rid]

    def __iter__(self):
        return iter(self._robots.values())

    def items(self):
        return self._robots.items()

    def values(self):
        return self._robots.values()

    def keys(self):
        return self._robots.keys()

    def __len__(self) -> int:
        return len(self._robots)
