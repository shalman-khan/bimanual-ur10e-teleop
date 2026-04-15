"""
Persistent settings manager — loads/saves JSON to a Docker volume.
Defaults match the existing PORT_CONFIG_MAP entries in gello_agent.py.
"""

import json
import math
import os
from typing import Any, Dict

SETTINGS_FILE = os.environ.get(
    "SETTINGS_FILE", "/workspace/settings/settings.json"
)

DEFAULT_SETTINGS: Dict[str, Any] = {
    # ── Robot IPs ──────────────────────────────────────────────────────────
    "robot_left_ip": "192.168.1.20",   # robot2 in bimanual_ur10e / left GELLO
    "robot_right_ip": "192.168.1.10",  # robot1 in bimanual_ur10e / right GELLO

    # ── GELLO Serial Ports ─────────────────────────────────────────────────
    "gello_left_port": (
        "/dev/serial/by-id/"
        "usb-FTDI_USB__-__Serial_Converter_FTAO5209-if00-port0"
    ),
    "gello_right_port": (
        "/dev/serial/by-id/"
        "usb-FTDI_USB__-__Serial_Converter_FTAO528D-if00-port0"
    ),

    # ── GELLO Left Config  (matches PORT_CONFIG_MAP LEFT UR entry) ─────────
    "gello_left_config": {
        "joint_ids":     [1, 2, 3, 4, 5, 6],
        "joint_offsets": [
            round(4 * math.pi / 2, 6),   # 6.283185
            round(5 * math.pi / 2, 6),   # 7.853982
            round(math.pi,         6),   # 3.141593
            round(1 * math.pi / 2, 6),   # 1.570796
            round(2 * math.pi,     6),   # 6.283185
            round(2 * math.pi,     6),   # 6.283185
        ],
        "joint_signs":   [1, 1, -1, 1, 1, 1],
        "gripper_config": [7, 16, -25],  # (motor_id, open_deg, close_deg)
    },

    # ── GELLO Right Config (matches PORT_CONFIG_MAP RIGHT UR entry) ────────
    "gello_right_config": {
        "joint_ids":     [1, 2, 3, 4, 5, 6],
        "joint_offsets": [
            round(2 * math.pi / 2, 6),   # 3.141593
            round(2 * math.pi / 2, 6),   # 3.141593
            round(2 * math.pi / 2, 6),   # 3.141593
            round(5 * math.pi / 2, 6),   # 7.853982
            round(1 * math.pi,     6),   # 3.141593
            round(1 * math.pi / 2, 6),   # 1.570796
        ],
        "joint_signs":   [1, 1, -1, 1, 1, 1],
        "gripper_config": [7, 197, 155],
    },

    # ── Control Loop ───────────────────────────────────────────────────────
    "control_rate_hz": 50,

    # ── ServoJ Parameters (used in TELEOP ACTIVE mode) ────────────────────
    "servoj_velocity":      0.5,
    "servoj_acceleration":  0.5,
    "servoj_lookahead_time": 0.2,
    "servoj_gain":          300,

    # ── MoveJ Parameters (used in MOTION PLAN mode) ───────────────────────
    "movej_speed":        0.5,
    "movej_acceleration": 0.3,
}


class SettingsManager:
    def __init__(self) -> None:
        self._data: Dict[str, Any] = dict(DEFAULT_SETTINGS)
        self._load()

    # ── Persistence ────────────────────────────────────────────────────────

    def _load(self) -> None:
        if not os.path.exists(SETTINGS_FILE):
            return
        try:
            with open(SETTINGS_FILE, "r") as f:
                saved = json.load(f)
            self._data.update(saved)
            print(f"[Settings] Loaded from {SETTINGS_FILE}")
        except Exception as exc:
            print(f"[Settings] Load failed ({exc}), using defaults")

    def save(self) -> None:
        os.makedirs(os.path.dirname(SETTINGS_FILE), exist_ok=True)
        with open(SETTINGS_FILE, "w") as f:
            json.dump(self._data, f, indent=2)

    # ── Access ─────────────────────────────────────────────────────────────

    def get(self) -> Dict[str, Any]:
        data = dict(self._data)
        # Env-var overrides — useful for sim/CI without touching persisted settings
        if ip := os.environ.get("ROBOT_LEFT_IP"):
            data["robot_left_ip"] = ip
        if ip := os.environ.get("ROBOT_RIGHT_IP"):
            data["robot_right_ip"] = ip
        return data

    def update(self, updates: Dict[str, Any]) -> None:
        self._data.update(updates)
        self.save()

    def __getitem__(self, key: str) -> Any:
        return self._data[key]

    def get_value(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)
