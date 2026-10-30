"""Точка входа оркестратора: rclpy.init, FastAPI app, startup задачи."""

from __future__ import annotations

import uvicorn

from app import app  # noqa: F401  (при импорте запускаются startup-события и rclpy.init)

if __name__ == "__main__":
    import os

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("ORCHESTRATOR_PORT", "8191")),
    )
