"""
FastAPI application — entry point for the bimanual teleop server.

Serves:
  GET  /                      → index.html (web UI)
  GET  /static/*              → UI assets
  WS   /ws                    → real-time state broadcast (10 Hz)
  REST /api/*                 → control & settings endpoints
"""

import asyncio
import datetime
import logging
import os
import shlex
import subprocess
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


# ── Keyboard TCP Teleop ───────────────────────────────────────────────────────
class KeyboardKeyRequest(BaseModel):
    side:      str    # "left" | "right"
    axis:      int    # 0=X 1=Y 2=Z 3=Rx 4=Ry 5=Rz 6=gripper
    direction: float  # -1, 0, +1


class KeyboardSpeedRequest(BaseModel):
    linear:   float   # m/s  (default 0.05)
    rotation: float   # rad/s (default 0.3)


@app.post("/api/keyboard_teleop/activate")
async def api_kb_activate():
    current = state_machine.state
    if current is SystemState.IDLE:
        return {"error": "Connect to robots first", "state": current.value}
    if current is SystemState.MOTION_PLAN_EXECUTING:
        return {"error": "Cannot activate while executing", "state": current.value}
    if current is SystemState.TELEOP_ACTIVE:
        state_machine.transition(SystemState.TELEOP_PASSIVE)
    if state_machine.state is not SystemState.KEYBOARD_TELEOP:
        ok = state_machine.transition(SystemState.KEYBOARD_TELEOP)
        if not ok:
            return {"error": "Transition failed", "state": state_machine.state.value}
    return {"state": state_machine.state.value}


@app.post("/api/keyboard_teleop/deactivate")
async def api_kb_deactivate():
    if state_machine.state is SystemState.KEYBOARD_TELEOP:
        state_machine.transition(SystemState.TELEOP_PASSIVE)
    return {"state": state_machine.state.value}


@app.post("/api/keyboard_teleop/key")
async def api_kb_key(req: KeyboardKeyRequest):
    if state_machine.state is not SystemState.KEYBOARD_TELEOP:
        return {"error": "Not in keyboard teleop mode"}
    robot_manager.set_keyboard_key(req.side, req.axis, req.direction)
    return {"ok": True}


@app.post("/api/keyboard_teleop/speed")
async def api_kb_speed(req: KeyboardSpeedRequest):
    robot_manager.set_keyboard_speed(req.linear, req.rotation)
    return {"ok": True}


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


# ── Gripper Override ──────────────────────────────────────────────────────────
class GripperOverrideRequest(BaseModel):
    side:    str   # "left" | "right" | "both"
    enabled: bool


@app.get("/api/gripper/override")
async def api_gripper_override_get():
    return robot_manager.get_gripper_overrides()


@app.post("/api/gripper/override")
async def api_gripper_override_set(req: GripperOverrideRequest):
    robot_manager.set_gripper_override(req.side, req.enabled)
    return robot_manager.get_gripper_overrides()


# ── Rosbag Recording ──────────────────────────────────────────────────────────
RECORD_TOPICS = [
    "/robot1/joint_states",
    "/robot2/joint_states",
    "/gripper1/joint_states",
    "/gripper2/joint_states",
    "/zed/zed_node/depth/depth_registered",
    "/zed/zed_node/rgb/color/rect/image",
    "/robot1/wrench",
    "/robot2/wrench",
]

_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source /ros2_ws/install/setup.bash 2>/dev/null; "
    "source /workspace/install/setup.bash 2>/dev/null; "
)

_rosbag_proc: Optional[subprocess.Popen] = None
_rosbag_path: Optional[str] = None
_rosbag_start_time: Optional[datetime.datetime] = None


def _check_topic_publishers(topic: str) -> int:
    """Return publisher count for a topic, or -1 on error."""
    try:
        env = os.environ.copy()
        result = subprocess.run(
            ["bash", "-c", f"{_ROS_SETUP}ros2 topic info {shlex.quote(topic)}"],
            capture_output=True, text=True, timeout=6, env=env,
        )
        for line in result.stdout.splitlines():
            if "Publisher count:" in line:
                return int(line.split(":")[-1].strip())
    except Exception:
        pass
    return -1


@app.get("/api/rosbag/status")
async def api_rosbag_status():
    recording = _rosbag_proc is not None and _rosbag_proc.poll() is None
    elapsed = None
    if recording and _rosbag_start_time:
        elapsed = (datetime.datetime.now() - _rosbag_start_time).seconds
    return {
        "recording": recording,
        "bag_path":  _rosbag_path,
        "elapsed_s": elapsed,
    }


@app.post("/api/rosbag/check_topics")
async def api_rosbag_check_topics():
    loop = asyncio.get_event_loop()
    results: Dict[str, int] = {}
    tasks = {
        topic: loop.run_in_executor(None, _check_topic_publishers, topic)
        for topic in RECORD_TOPICS
    }
    for topic, fut in tasks.items():
        results[topic] = await fut
    return {"topics": results}


class RosbagStartRequest(BaseModel):
    bag_dir: str = "/workspace/bags"


@app.post("/api/rosbag/start")
async def api_rosbag_start(req: RosbagStartRequest):
    global _rosbag_proc, _rosbag_path, _rosbag_start_time

    if _rosbag_proc is not None and _rosbag_proc.poll() is None:
        return {"error": "Already recording"}

    bag_dir = req.bag_dir.rstrip("/") or "/workspace/bags"
    os.makedirs(bag_dir, exist_ok=True)

    ts       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = f"{bag_dir}/session_{ts}"
    topics   = " ".join(RECORD_TOPICS)

    env = os.environ.copy()
    cmd = f"{_ROS_SETUP}ros2 bag record -o {shlex.quote(bag_path)} {topics}"

    try:
        _rosbag_proc       = subprocess.Popen(["bash", "-c", cmd], env=env)
        _rosbag_path       = bag_path
        _rosbag_start_time = datetime.datetime.now()
        return {"status": "started", "bag_path": bag_path}
    except Exception as exc:
        return {"error": str(exc)}


def _kill_rosbag_proc() -> Optional[str]:
    """Stop the recording process and return the bag path. Does not delete files."""
    global _rosbag_proc, _rosbag_path, _rosbag_start_time
    path = _rosbag_path
    if _rosbag_proc is not None:
        _rosbag_proc.terminate()
        try:
            _rosbag_proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            _rosbag_proc.kill()
    _rosbag_proc       = None
    _rosbag_start_time = None
    return path


@app.post("/api/rosbag/stop")
async def api_rosbag_stop():
    if _rosbag_proc is None or _rosbag_proc.poll() is not None:
        return {"error": "Not recording"}
    path = _kill_rosbag_proc()
    return {"status": "stopped", "bag_path": path}


@app.post("/api/rosbag/discard")
async def api_rosbag_discard():
    import shutil
    if _rosbag_proc is None or _rosbag_proc.poll() is not None:
        return {"error": "Not recording"}
    path = _kill_rosbag_proc()
    deleted = False
    if path and os.path.exists(path):
        try:
            shutil.rmtree(path)
            deleted = True
        except Exception as exc:
            return {"status": "stopped_not_deleted", "bag_path": path, "error": str(exc)}
    return {"status": "discarded", "bag_path": path, "deleted": deleted}


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8080,
        log_level="info",
        reload=False,
    )
