"""
Robot manager — owns the URRobot and GelloAgent instances and
runs the control loop and motion-plan threads.

Joint array layout (14 DOF bimanual):
  index 0-5  : left arm joints  (radians)
  index 6    : left gripper     (0.0 = open, 1.0 = closed)
  index 7-12 : right arm joints (radians)
  index 13   : right gripper    (0.0 = open, 1.0 = closed)

Gripper protocol: normalized float (0-1) ↔ raw 0-255 via socket to UR.
"""

import os
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from state_machine import StateMachine, SystemState

_MOCK       = os.environ.get("MOCK_ROBOTS", "").lower() in ("1", "true")
_MOCK_GELLO = _MOCK or os.environ.get("MOCK_GELLO", "").lower() in ("1", "true")


# ─── Shared state (thread-safe snapshot for WebSocket broadcasts) ────────────

class SharedRobotState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.left_joints:  List[float] = [0.0] * 6
        self.left_gripper: float       = 0.0
        self.right_joints:  List[float] = [0.0] * 6
        self.right_gripper: float       = 0.0
        self.robot_left_connected:  bool = False
        self.robot_right_connected: bool = False
        self.gello_left_connected:  bool = False
        self.gello_right_connected: bool = False
        self.motion_plan_status: Optional[str] = None

    def update_left(self, joints: Any, gripper: float) -> None:
        with self._lock:
            self.left_joints  = [float(v) for v in joints[:6]]
            self.left_gripper = float(gripper)

    def update_right(self, joints: Any, gripper: float) -> None:
        with self._lock:
            self.right_joints  = [float(v) for v in joints[:6]]
            self.right_gripper = float(gripper)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "left_joints":            list(self.left_joints),
                "left_gripper":           self.left_gripper,
                "right_joints":           list(self.right_joints),
                "right_gripper":          self.right_gripper,
                "robot_left_connected":   self.robot_left_connected,
                "robot_right_connected":  self.robot_right_connected,
                "gello_left_connected":   self.gello_left_connected,
                "gello_right_connected":  self.gello_right_connected,
                "motion_plan_status":     self.motion_plan_status,
            }


# ─── Robot Manager ────────────────────────────────────────────────────────────

class RobotManager:
    def __init__(self, state_machine: StateMachine, settings) -> None:
        self._sm       = state_machine
        self._settings = settings
        self.shared    = SharedRobotState()

        # Robot + GELLO handles
        self._robot_left  = None  # gello.robots.ur.URRobot
        self._robot_right = None
        self._gello_left  = None  # gello.agents.gello_agent.GelloAgent
        self._gello_right = None

        # Relative-teleop reference snapshots (captured when Active is pressed)
        self._gello_ref_left:  Optional[np.ndarray] = None
        self._gello_ref_right: Optional[np.ndarray] = None
        self._robot_ref_left:  Optional[np.ndarray] = None
        self._robot_ref_right: Optional[np.ndarray] = None

        # Control loop
        self._ctl_thread:  Optional[threading.Thread] = None
        self._ctl_running: bool = False

        self._sm.add_listener(self._on_state_change)

    # ─── State-change hook ────────────────────────────────────────────────

    def _on_state_change(self, old: SystemState, new: SystemState) -> None:
        """Stop servoJ when leaving ACTIVE; capture reference when entering ACTIVE."""
        if old is SystemState.TELEOP_ACTIVE:
            self._stop_servoj()
        if new is SystemState.TELEOP_ACTIVE:
            self._capture_teleop_reference()

    def _capture_teleop_reference(self) -> None:
        """
        Record GELLO and robot joint positions the moment Active is pressed.
        The control loop will then send (robot_ref + delta) so the robot only
        moves when the GELLO moves, never jumping on mode activation.
        """
        self._gello_ref_left  = None
        self._gello_ref_right = None
        self._robot_ref_left  = None
        self._robot_ref_right = None

        for gello, attr in [(self._gello_left, '_gello_ref_left'),
                            (self._gello_right, '_gello_ref_right')]:
            if gello is not None:
                for _attempt in range(5):
                    try:
                        setattr(self, attr, np.array(gello.act({})[:6]))
                        break
                    except Exception:
                        time.sleep(0.01)

        for robot, attr in [(self._robot_left, '_robot_ref_left'),
                            (self._robot_right, '_robot_ref_right')]:
            if robot is not None:
                try:
                    joints = np.array(robot.r_inter.getActualQ())
                    if np.any(np.abs(joints) > 1e-6):
                        setattr(self, attr, joints)
                    else:
                        # Fallback: shared state has the last good read
                        fallback = (self.shared.left_joints
                                    if robot is self._robot_left
                                    else self.shared.right_joints)
                        setattr(self, attr, np.array(fallback))
                except Exception:
                    pass

    def _stop_servoj(self) -> None:
        for robot in (self._robot_left, self._robot_right):
            if robot is not None:
                try:
                    robot.robot.servoStop(True)
                except Exception:
                    pass

    # ─── Connection ───────────────────────────────────────────────────────

    def connect_robots(self) -> Dict[str, str]:
        """Connect both URRobot instances. Returns per-arm status strings."""
        if _MOCK:
            from mock_robots import MockURRobot as URRobot
        else:
            from gello.robots.ur import URRobot

        result: Dict[str, str] = {}
        s = self._settings.get()

        for side, ip_key, attr, update_fn in [
            ("left",  "robot_left_ip",  "_robot_left",  self.shared.update_left),
            ("right", "robot_right_ip", "_robot_right", self.shared.update_right),
        ]:
            try:
                robot = URRobot(robot_ip=s[ip_key], no_gripper=False)
                setattr(self, attr, robot)

                # RTDE receive buffer may return all-zeros on the first few
                # calls after connection.  Retry until we get real data.
                joints = np.zeros(6)
                for _ in range(20):
                    joints = np.array(robot.r_inter.getActualQ())
                    if np.any(np.abs(joints) > 1e-6):
                        break
                    time.sleep(0.05)

                # Activate gripper if not already active (required before move() works)
                if robot._use_gripper:
                    try:
                        if not robot.gripper.is_active():
                            print(f"[Connect] activating {side} gripper...")
                            robot.gripper.activate(auto_calibrate=False)
                            print(f"[Connect] {side} gripper activated")
                    except Exception as exc:
                        print(f"[Connect] {side} gripper activation failed: {exc}")

                gripper = 0.0
                if robot._use_gripper:
                    try:
                        gripper = robot._get_gripper_pos()
                    except Exception:
                        pass

                update_fn(joints, gripper)
                setattr(self.shared, f"robot_{side}_connected", True)
                result[side] = "connected"
            except Exception as exc:
                setattr(self.shared, f"robot_{side}_connected", False)
                result[side] = f"error: {exc}"

        if self.shared.robot_left_connected or self.shared.robot_right_connected:
            self._start_control_loop()

        return result

    def connect_gello(self) -> Dict[str, str]:
        """Connect both GELLO Dynamixel devices. Returns per-arm status strings."""
        if _MOCK_GELLO:
            from mock_robots import MockGelloAgent as GelloAgent
            for side, port_key, attr in [
                ("left",  "gello_left_port",  "_gello_left"),
                ("right", "gello_right_port", "_gello_right"),
            ]:
                port = self._settings.get()[port_key]
                setattr(self, attr, GelloAgent(port=port))
                setattr(self.shared, f"gello_{side}_connected", True)
            return {"left": "connected (mock)", "right": "connected (mock)"}

        from gello.agents.gello_agent import DynamixelRobotConfig, GelloAgent

        result: Dict[str, str] = {}
        s = self._settings.get()

        def make_cfg(key: str) -> DynamixelRobotConfig:
            cfg = s[key]
            return DynamixelRobotConfig(
                joint_ids     = tuple(cfg["joint_ids"]),
                joint_offsets = tuple(cfg["joint_offsets"]),
                joint_signs   = tuple(cfg["joint_signs"]),
                gripper_config= tuple(cfg["gripper_config"]),
            )

        for side, port_key, cfg_key, attr in [
            ("left",  "gello_left_port",  "gello_left_config",  "_gello_left"),
            ("right", "gello_right_port", "gello_right_config", "_gello_right"),
        ]:
            port = s[port_key]
            try:
                cfg   = make_cfg(cfg_key)
                agent = GelloAgent(port=port, dynamixel_config=cfg)
                agent.act({})          # probe read to verify connectivity
                setattr(self, attr, agent)
                setattr(self.shared, f"gello_{side}_connected", True)
                result[side] = "connected"
            except Exception as exc:
                setattr(self.shared, f"gello_{side}_connected", False)
                result[side] = f"error: {exc}"

        return result

    def disconnect(self) -> None:
        self._stop_control_loop()
        self._robot_left  = None
        self._robot_right = None
        self._gello_left  = None
        self._gello_right = None
        self.shared.robot_left_connected  = False
        self.shared.robot_right_connected = False
        self.shared.gello_left_connected  = False
        self.shared.gello_right_connected = False
        self._sm.transition(SystemState.IDLE, force=True)

    # ─── Control Loop ─────────────────────────────────────────────────────

    def _start_control_loop(self) -> None:
        if self._ctl_running:
            return
        self._ctl_running = True
        self._ctl_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="control_loop"
        )
        self._ctl_thread.start()

    def _stop_control_loop(self) -> None:
        self._ctl_running = False
        if self._ctl_thread:
            self._ctl_thread.join(timeout=2.0)
            self._ctl_thread = None

    def _control_loop(self) -> None:
        """
        Runs at `control_rate_hz` (default 50 Hz).

        TELEOP_ACTIVE  → reads GELLO, sends servoJ + gripper commands
                         left and right arms run in parallel threads.
        All other states → reads robot joint state to keep UI live.
        """
        s            = self._settings.get()
        hz           = s.get("control_rate_hz", 50)
        period       = 1.0 / hz
        sv_vel       = s.get("servoj_velocity",       0.5)
        sv_acc       = s.get("servoj_acceleration",   0.5)
        sv_look      = s.get("servoj_lookahead_time", 0.1)
        sv_gain      = s.get("servoj_gain",           300)
        # sv_dt must match the actual control period so each servoJ command
        # stays valid until the next one arrives (not the UR's 500 Hz internal rate).
        sv_dt        = period

        while self._ctl_running:
            t0 = time.monotonic()

            if self._sm.state is SystemState.TELEOP_ACTIVE:

                def _cmd_arm(robot, gello, update_fn, gello_ref, robot_ref):
                    try:
                        obs          = gello.act({})   # (7,) = 6 arm + 1 gripper
                        gello_now    = obs[:6]
                        gripper      = float(obs[6]) if len(obs) > 6 else 0.0

                        if gello_ref is not None and robot_ref is not None:
                            # Relative mode: only the delta from the reference
                            # is applied to the robot's starting position.
                            # Robot stays still when GELLO is static.
                            delta = gello_now - gello_ref
                            arm_q = (robot_ref + delta).tolist()
                        else:
                            # Reference not yet captured — hold current robot
                            # position rather than jumping to absolute GELLO pose.
                            arm_q = robot.r_inter.getActualQ()

                        robot.robot.servoJ(
                            arm_q, sv_vel, sv_acc, sv_dt, sv_look, sv_gain
                        )
                        if robot._use_gripper:
                            robot.gripper.move(int(gripper * 255), 255, 10)

                        update_fn(arm_q, gripper)
                    except Exception as exc:
                        print(f"[Control] Arm command error: {exc}")

                threads = []
                if self._gello_left and self._robot_left:
                    threads.append(threading.Thread(
                        target=_cmd_arm,
                        args=(self._robot_left, self._gello_left,
                              self.shared.update_left,
                              self._gello_ref_left, self._robot_ref_left),
                        daemon=True,
                    ))
                if self._gello_right and self._robot_right:
                    threads.append(threading.Thread(
                        target=_cmd_arm,
                        args=(self._robot_right, self._gello_right,
                              self.shared.update_right,
                              self._gello_ref_right, self._robot_ref_right),
                        daemon=True,
                    ))

                for t in threads:
                    t.start()
                for t in threads:
                    t.join(timeout=period * 3)

            else:
                # Passive / other states: keep UI joint display live.
                # Use r_inter.getActualQ() directly — avoids the 10ms gripper
                # sleep in get_joint_state() and prevents the gripper socket
                # from blocking the loop.  Guard against all-zero reads that
                # the RTDE buffer can return before its first data packet.
                for robot, update_fn in [
                    (self._robot_left,  self.shared.update_left),
                    (self._robot_right, self.shared.update_right),
                ]:
                    if robot is not None:
                        try:
                            joints = np.array(robot.r_inter.getActualQ())
                            if np.any(np.abs(joints) > 1e-6):
                                update_fn(joints, self.shared.left_gripper
                                          if robot is self._robot_left
                                          else self.shared.right_gripper)
                        except Exception as exc:
                            print(f"[Control] passive read error: {exc}")

            # Maintain loop rate (servoJ blocks for sv_dt, so remaining ≈ 0
            # in TELEOP_ACTIVE; in passive mode we still pace at hz)
            elapsed   = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ─── Motion Planning ──────────────────────────────────────────────────

    def execute_move_to(
        self,
        left_joints:   List[float],
        left_gripper:  float,
        right_joints:  List[float],
        right_gripper: float,
        speed:         float = 0.5,
        accel:         float = 0.3,
    ) -> bool:
        """
        Blocking: move both arms to target joint state via moveJ (UR's own
        joint-space planner — shortest path, no collision checking).
        Both arms move simultaneously in parallel threads.
        """
        if not self._sm.transition(SystemState.MOTION_PLAN_EXECUTING):
            return False

        self.shared.motion_plan_status = "executing"

        try:
            results: Dict[str, bool] = {}

            def move(robot, joints, gripper_val, key):
                try:
                    if robot is None:
                        results[key] = True
                        return
                    success = robot.robot.moveJ(joints, speed, accel)
                    if robot._use_gripper:
                        robot.gripper.move(int(gripper_val * 255), 255, 10)
                    results[key] = bool(success)
                except Exception as exc:
                    print(f"[Motion] {key} moveJ error: {exc}")
                    results[key] = False

            tl = threading.Thread(
                target=move,
                args=(self._robot_left,  left_joints,  left_gripper,  "left"),
                daemon=True,
            )
            tr = threading.Thread(
                target=move,
                args=(self._robot_right, right_joints, right_gripper, "right"),
                daemon=True,
            )
            tl.start(); tr.start()
            tl.join();  tr.join()

            ok_l = results.get("left",  True)
            ok_r = results.get("right", True)
            success = ok_l and ok_r

            if success:
                self.shared.update_left(left_joints,   left_gripper)
                self.shared.update_right(right_joints, right_gripper)

        except Exception as exc:
            print(f"[Motion] execute_move_to error: {exc}")
            success = False
        finally:
            self.shared.motion_plan_status = "completed" if success else "failed"
            self._sm.transition(SystemState.MOTION_PLAN_IDLE)

        return success

    def execute_trajectory(self, points: List[Dict[str, Any]]) -> bool:
        """
        Blocking: stream a pre-planned joint trajectory via servoJ.

        Each point is a dict:
            left_joints:     list[float]  (6 values, radians)
            left_gripper:    float        (0.0-1.0)
            right_joints:    list[float]
            right_gripper:   float
            time_from_start: float        (seconds from trajectory start)
        """
        if not self._sm.transition(SystemState.MOTION_PLAN_EXECUTING):
            return False

        self.shared.motion_plan_status = "executing_trajectory"
        s        = self._settings.get()
        sv_vel   = s.get("servoj_velocity",       0.5)
        sv_acc   = s.get("servoj_acceleration",   0.5)
        sv_look  = s.get("servoj_lookahead_time", 0.2)
        sv_gain  = s.get("servoj_gain",           300)
        sv_dt    = 1.0 / 500
        success  = False

        try:
            t_start = time.monotonic()

            for i, pt in enumerate(points):
                if self._sm.state is not SystemState.MOTION_PLAN_EXECUTING:
                    break   # aborted via mode switch

                lj  = pt["left_joints"]
                lg  = pt.get("left_gripper",  0.0)
                rj  = pt["right_joints"]
                rg  = pt.get("right_gripper", 0.0)

                if self._robot_left is not None:
                    ts = self._robot_left.robot.initPeriod()
                    self._robot_left.robot.servoJ(lj, sv_vel, sv_acc, sv_dt, sv_look, sv_gain)
                    if self._robot_left._use_gripper:
                        self._robot_left.gripper.move(int(lg * 255), 255, 10)
                    self._robot_left.robot.waitPeriod(ts)

                if self._robot_right is not None:
                    ts = self._robot_right.robot.initPeriod()
                    self._robot_right.robot.servoJ(rj, sv_vel, sv_acc, sv_dt, sv_look, sv_gain)
                    if self._robot_right._use_gripper:
                        self._robot_right.gripper.move(int(rg * 255), 255, 10)
                    self._robot_right.robot.waitPeriod(ts)

                self.shared.update_left(lj,  lg)
                self.shared.update_right(rj, rg)

                # Sleep until next waypoint's scheduled time
                if i < len(points) - 1:
                    next_t    = points[i + 1].get("time_from_start", 0.0)
                    wake_at   = t_start + next_t
                    remaining = wake_at - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)

            self._stop_servoj()
            success = True

        except Exception as exc:
            print(f"[Motion] execute_trajectory error: {exc}")
            success = False
        finally:
            self.shared.motion_plan_status = "completed" if success else "failed"
            self._sm.transition(SystemState.MOTION_PLAN_IDLE)

        return success

    # ─── Async wrappers (fire-and-forget for API handlers) ────────────────

    def execute_move_to_async(self, *args, **kwargs) -> None:
        threading.Thread(
            target=self.execute_move_to, args=args, kwargs=kwargs, daemon=True
        ).start()

    def execute_trajectory_async(self, points: List[Dict]) -> None:
        threading.Thread(
            target=self.execute_trajectory, args=(points,), daemon=True
        ).start()

    # ─── Direct gripper control ───────────────────────────────────────────

    def set_gripper(self, side: str, position: float) -> Dict[str, str]:
        """
        Send a direct gripper command without moving the arm joints.

        side     : "left" | "right" | "both"
        position : 0.0 (fully open) – 1.0 (fully closed)
        """
        raw    = int(max(0.0, min(1.0, position)) * 255)
        result: Dict[str, str] = {}

        pairs = []
        if side in ("left", "both") and self._robot_left is not None:
            pairs.append(("left",  self._robot_left,  self.shared.update_left,
                          self.shared.left_joints))
        if side in ("right", "both") and self._robot_right is not None:
            pairs.append(("right", self._robot_right, self.shared.update_right,
                          self.shared.right_joints))

        for name, robot, update_fn, joints in pairs:
            if not robot._use_gripper:
                result[name] = "no gripper"
                continue
            try:
                robot.gripper.move(raw, 255, 10)
                update_fn(joints, position)
                result[name] = "ok"
            except Exception as exc:
                result[name] = f"error: {exc}"

        return result

    # ─── Direct robot read (slow — avoid in tight loops) ─────────────────

    def read_robot_joint_states(self) -> Dict[str, Any]:
        """Read actual joint states from robots. Used at connect time."""
        out: Dict[str, Any] = {}
        for side, robot in [("left", self._robot_left), ("right", self._robot_right)]:
            if robot is None:
                continue
            try:
                state = robot.get_joint_state()
                out[f"{side}_joints"]  = state[:6].tolist()
                out[f"{side}_gripper"] = float(state[6]) if len(state) > 6 else 0.0
            except Exception:
                pass
        return out
