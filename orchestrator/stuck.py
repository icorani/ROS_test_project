"""Детекция застревания робота и сброс блокировок."""

from __future__ import annotations

import math

# Порог застревания: счётчик тиков без движения при активной миссии.
# При ~20 Гц и пороге 40 — порядка 2 секунд.
STUCK_COUNT_THRESHOLD = 40
STUCK_MOVE_THRESHOLD = 0.01  # м, меньше чего движения не считаем


def update_stuck(robot, now: float) -> str | None:
    """Обновляет счётчик застревания.

    Возвращает ``"stuck"`` когда робот только что помечен застрявшим,
    ``"recovered"`` когда возобновил движение, иначе ``None``.
    """
    moving = robot.status == "moving" and bool(robot.mission)
    if moving:
        dist = math.hypot(robot.x - robot.last_x, robot.y - robot.last_y)
        if dist < STUCK_MOVE_THRESHOLD:
            robot.stuck_count += 1
            if robot.stuck_since is None:
                robot.stuck_since = now
        else:
            robot.stuck_count = 0
            robot.stuck_since = None
    else:
        robot.stuck_count = 0
        robot.stuck_since = None

    if moving and robot.stuck_count > STUCK_COUNT_THRESHOLD:
        return "stuck"
    return None
