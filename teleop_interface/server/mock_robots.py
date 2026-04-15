"""
Mock implementations of URRobot and GelloAgent for testing without hardware.

Activated automatically when MOCK_ROBOTS=1 env var is set.
Simulates:
  - URRobot:    returns sinusoidal joint motion, accepts all commands silently
  - GelloAgent: returns slowly-sweeping joint values
"""

import math
import time
from typing import Optional

import numpy as np


# ── Mock RTDE control interface ───────────────────────────────────────────────

class _MockRTDEControl:
    _DT = 1.0 / 500

    def initPeriod(self):
        return time.monotonic()

    def servoJ(self, q, velocity, acceleration, dt, lookahead_time, gain):
        pass  # silently accept

    def waitPeriod(self, t_start):
        elapsed   = time.monotonic() - t_start
        remaining = self._DT - elapsed
        if remaining > 0:
            time.sleep(remaining)

    def servoStop(self, decelerate=True):
        pass

    def moveJ(self, q, speed=0.5, acceleration=0.3, asynchronous=False):
        # simulate motion time proportional to speed
        time.sleep(min(0.5 / max(speed, 0.01), 2.0))
        return True

    def freedriveMode(self):  pass
    def endFreedriveMode(self): pass


class _MockRTDEReceive:
    def __init__(self):
        self._t0 = time.monotonic()

    def getActualQ(self):
        t = time.monotonic() - self._t0
        return [math.sin(t + i * 0.5) * 0.3 for i in range(6)]


class _MockGripper:
    def __init__(self):
        self._pos = 0
        self._t0  = time.monotonic()

    def connect(self, hostname, port, socket_timeout=10.0): pass
    def activate(self, auto_calibrate=True): pass
    def is_active(self): return True

    def get_current_position(self) -> int:
        t = time.monotonic() - self._t0
        return int((math.sin(t * 0.5) * 0.5 + 0.5) * 200)

    def move(self, position, speed, force):
        self._pos = int(position)
        time.sleep(0.002)          # tiny sleep to match real gripper latency shape
        return True, self._pos

    def move_and_wait_for_pos(self, position, speed, force):
        self.move(position, speed, force)
        from gello.robots.robotiq_gripper import RobotiqGripper
        return int(position), RobotiqGripper.ObjectStatus.AT_DEST


# ── Mock URRobot ──────────────────────────────────────────────────────────────

class MockURRobot:
    """Drop-in replacement for gello.robots.ur.URRobot."""

    def __init__(self, robot_ip: str = "0.0.0.0", no_gripper: bool = False):
        print(f"[Mock] URRobot connected (simulated) @ {robot_ip}")
        self.robot    = _MockRTDEControl()
        self.r_inter  = _MockRTDEReceive()
        self.gripper  = _MockGripper()
        self._use_gripper = not no_gripper
        self._t0      = time.monotonic()

    def num_dofs(self) -> int:
        return 7 if self._use_gripper else 6

    def get_joint_state(self) -> np.ndarray:
        t      = time.monotonic() - self._t0
        joints = np.array([math.sin(t + i * 0.5) * 0.3 for i in range(6)])
        if self._use_gripper:
            gripper = (math.sin(t * 0.5) * 0.5 + 0.5)
            return np.append(joints, gripper)
        return joints

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        pass  # accept silently

    def get_observations(self):
        state = self.get_joint_state()
        return {
            "joint_positions":  state,
            "joint_velocities": np.zeros_like(state),
            "ee_pos_quat":      np.zeros(7),
            "gripper_position": np.array([state[-1]]) if self._use_gripper else np.array([0.0]),
        }

    def set_freedrive_mode(self, enable: bool): pass
    def freedrive_enabled(self) -> bool: return False


# ── Mock GelloAgent ───────────────────────────────────────────────────────────

class MockGelloAgent:
    """Drop-in replacement for gello.agents.gello_agent.GelloAgent."""

    def __init__(self, port: str = "/dev/mock", **kwargs):
        print(f"[Mock] GelloAgent connected (simulated) @ {port}")
        self._t0   = time.monotonic()
        self._port = port

    def act(self, obs: dict) -> np.ndarray:
        t      = time.monotonic() - self._t0
        joints = np.array([math.sin(t * 0.4 + i * 0.7) * 0.5 for i in range(6)])
        gripper = (math.sin(t * 0.3) * 0.5 + 0.5)
        return np.append(joints, gripper)
