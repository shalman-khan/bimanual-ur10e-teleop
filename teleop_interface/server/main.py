"""
FastAPI application — entry point for the bimanual teleop server.

Serves:
  GET  /                      → index.html (web UI)
  GET  /static/*              → UI assets
  WS   /ws                    → real-time state broadcast (10 Hz)
  REST /api/*                 → control & settings endpoints
"""

import asyncio
import logging
import os
import sys
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional, Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ── Suppress noisy health-check access logs ───────────────────────────────────
class _SuppressStatusLog(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return "GET /api/status" not in record.getMessage()

logging.getLogger("uvicorn.access").addFilter(_SuppressStatusLog())

# ── Path setup ────────────────────────────────────────────────────────────────
SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
UI_DIR     = os.path.join(os.path.dirname(SERVER_DIR), "ui")

for p in [SERVER_DIR, "/workspace/gello_software"]:
    if p not in sys.path:
        sys.path.insert(0, p)

from settings_manager import SettingsManager
from state_machine    import StateMachine, SystemState
from robot_manager    import RobotManager

# ── Singletons ────────────────────────────────────────────────────────────────
settings      = SettingsManager()
state_machine = StateMachine()
robot_manager = RobotManager(state_machine, settings)

# ── WebSocket registry ────────────────────────────────────────────────────────
_ws_clients: Set[WebSocket] = set()
_ws_lock = asyncio.Lock()


async def _broadcast(payload: dict) -> None:
    async with _ws_lock:
        dead: Set[WebSocket] = set()
        for ws in _ws_clients:
            try:
                await ws.send_json(payload)
            except Exception:
                dead.add(ws)
        _ws_clients.difference_update(dead)


async def _status_loop() -> None:
    """Push state snapshots to all WS clients at 10 Hz."""
    while True:
        try:
            snap = robot_manager.shared.snapshot()
            snap["system_state"] = state_machine.state.value
            await _broadcast({"type": "state_update", **snap})
        except Exception as exc:
            print(f"[WS] broadcast error: {exc}")
        await asyncio.sleep(0.1)


# ── App lifecycle ─────────────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Start ROS2 node (non-fatal if rclpy unavailable)
    ros2_ok = False
    try:
        from ros2_node import BimanualTeleopROS2Node
        ros2_node = BimanualTeleopROS2Node(robot_manager, state_machine, settings)
        ros2_node.start()
        app.state.ros2_node = ros2_node
        ros2_ok = True
    except Exception as exc:
        print(f"[Main] ROS2 node init error: {exc}")
        app.state.ros2_node = None

    broadcast_task = asyncio.create_task(_status_loop())
    print(f"[Main] Server ready — ROS2: {'enabled' if ros2_ok else 'disabled'}")
    yield
    broadcast_task.cancel()


app = FastAPI(title="Bimanual UR10e Teleop Interface", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

# Static UI files
app.mount("/static", StaticFiles(directory=UI_DIR), name="static")


# ── Root ─────────────────────────────────────────────────────────────────────
@app.get("/")
async def root():
    return FileResponse(os.path.join(UI_DIR, "index.html"))


# ── WebSocket ─────────────────────────────────────────────────────────────────
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    async with _ws_lock:
        _ws_clients.add(ws)
    try:
        # Send immediate snapshot on connect
        snap = robot_manager.shared.snapshot()
        snap["system_state"] = state_machine.state.value
        await ws.send_json({"type": "state_update", **snap})
        # Keep alive — actual data comes via broadcast loop
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        async with _ws_lock:
            _ws_clients.discard(ws)


# ═══════════════════════════════════════════════════════════════════════════════
# REST API
# ═══════════════════════════════════════════════════════════════════════════════

# ── Status ────────────────────────────────────────────────────────────────────
@app.get("/api/status")
async def api_status():
    snap = robot_manager.shared.snapshot()
    snap["system_state"] = state_machine.state.value
    snap["ros2_available"] = (
        getattr(app.state, "ros2_node", None) is not None
        and app.state.ros2_node.is_available()
    )
    return snap


# ── Connection ────────────────────────────────────────────────────────────────
@app.post("/api/connect/robots")
async def api_connect_robots():
    loop   = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, robot_manager.connect_robots)
    if robot_manager.shared.robot_left_connected or robot_manager.shared.robot_right_connected:
        state_machine.transition(SystemState.TELEOP_PASSIVE)
    return {"result": result, "state": state_machine.state.value}


@app.post("/api/connect/gello")
async def api_connect_gello():
    loop   = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, robot_manager.connect_gello)
    return {"result": result}


@app.post("/api/disconnect")
async def api_disconnect():
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, robot_manager.disconnect)
    return {"state": state_machine.state.value}


# ── Mode Switching ────────────────────────────────────────────────────────────
class ModeRequest(BaseModel):
    mode: str  # "teleop" | "motion_plan"


@app.post("/api/mode")
async def api_set_mode(req: ModeRequest):
    current = state_machine.state

    if current is SystemState.IDLE:
        return {"error": "Connect to robots first", "state": current.value}

    if current is SystemState.MOTION_PLAN_EXECUTING:
        return {"error": "Cannot switch mode while executing", "state": current.value}

    if req.mode == "teleop":
        # From motion_plan or teleop_active → teleop_passive
        if current is SystemState.TELEOP_ACTIVE:
            state_machine.transition(SystemState.TELEOP_PASSIVE)
        elif current is SystemState.MOTION_PLAN_IDLE:
            if not state_machine.transition(SystemState.TELEOP_PASSIVE):
                return {"error": "Transition failed", "state": state_machine.state.value}
        # already TELEOP_PASSIVE → ok

    elif req.mode == "motion_plan":
        if current is SystemState.TELEOP_ACTIVE:
            state_machine.transition(SystemState.TELEOP_PASSIVE)
            current = state_machine.state
        if current is SystemState.TELEOP_PASSIVE:
            if not state_machine.transition(SystemState.MOTION_PLAN_IDLE):
                return {"error": "Transition failed", "state": state_machine.state.value}
        # already MOTION_PLAN_IDLE → ok

    else:
        return {"error": f"Unknown mode: {req.mode}"}

    return {"state": state_machine.state.value}


# ── Teleop Sub-state ──────────────────────────────────────────────────────────
class TeleopStateRequest(BaseModel):
    state: str  # "active" | "passive"


@app.post("/api/teleop/state")
async def api_set_teleop_state(req: TeleopStateRequest):
    if req.state == "active":
        if (not robot_manager.shared.gello_left_connected
                and not robot_manager.shared.gello_right_connected):
            return {"error": "No GELLO device connected", "state": state_machine.state.value}
        ok = state_machine.transition(SystemState.TELEOP_ACTIVE)
    elif req.state == "passive":
        ok = state_machine.transition(SystemState.TELEOP_PASSIVE)
    else:
        return {"error": f"Unknown teleop state: {req.state}"}

    return {"state": state_machine.state.value, "ok": ok}


# ── Motion Plan — Move To Target ──────────────────────────────────────────────
class MoveToRequest(BaseModel):
    left_joints:   List[float]          # 6 values (radians)
    left_gripper:  float                # 0.0–1.0
    right_joints:  List[float]          # 6 values (radians)
    right_gripper: float                # 0.0–1.0
    speed:         Optional[float] = None
    acceleration:  Optional[float] = None


@app.post("/api/motion_plan/move_to")
async def api_move_to(req: MoveToRequest):
    current = state_machine.state
    if current not in (SystemState.MOTION_PLAN_IDLE, SystemState.TELEOP_PASSIVE):
        return {"error": f"Must be in motion_plan or passive mode (current: {current.value})"}

    if current is SystemState.TELEOP_PASSIVE:
        state_machine.transition(SystemState.MOTION_PLAN_IDLE)

    s     = settings.get()
    speed = req.speed        if req.speed        is not None else s.get("movej_speed",        0.5)
    accel = req.acceleration if req.acceleration is not None else s.get("movej_acceleration", 0.3)

    robot_manager.execute_move_to_async(
        req.left_joints,  req.left_gripper,
        req.right_joints, req.right_gripper,
        speed, accel,
    )
    return {"status": "executing"}


# ── Motion Plan — Execute Trajectory ──────────────────────────────────────────
class TrajectoryPoint(BaseModel):
    left_joints:     List[float]
    left_gripper:    float
    right_joints:    List[float]
    right_gripper:   float
    time_from_start: float   # seconds from trajectory start


class TrajectoryRequest(BaseModel):
    points: List[TrajectoryPoint]


@app.post("/api/motion_plan/execute_trajectory")
async def api_execute_trajectory(req: TrajectoryRequest):
    current = state_machine.state
    if current not in (SystemState.MOTION_PLAN_IDLE, SystemState.TELEOP_PASSIVE):
        return {"error": f"Must be in motion_plan or passive mode (current: {current.value})"}

    if not req.points:
        return {"error": "Empty trajectory"}

    if current is SystemState.TELEOP_PASSIVE:
        state_machine.transition(SystemState.MOTION_PLAN_IDLE)

    points = [p.model_dump() for p in req.points]
    robot_manager.execute_trajectory_async(points)
    return {"status": "executing", "num_points": len(points)}


# ── Gripper Control ──────────────────────────────────────────────────────────
class GripperRequest(BaseModel):
    side:     str    # "left" | "right" | "both"
    position: float  # 0.0 = open, 1.0 = closed


@app.post("/api/gripper")
async def api_gripper(req: GripperRequest):
    current = state_machine.state
    if current is SystemState.IDLE:
        return {"error": "Connect to robots first"}
    if current is SystemState.MOTION_PLAN_EXECUTING:
        return {"error": "Cannot control gripper while executing"}

    loop   = asyncio.get_event_loop()
    result = await loop.run_in_executor(
        None, robot_manager.set_gripper, req.side, req.position
    )
    return {"result": result}


# ── Settings ─────────────────────────────────────────────────────────────────
@app.get("/api/settings")
async def api_get_settings():
    return settings.get()


@app.post("/api/settings")
async def api_update_settings(data: Dict[str, Any]):
    settings.update(data)
    return {"status": "saved", "settings": settings.get()}


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8080,
        log_level="info",
        reload=False,
    )
