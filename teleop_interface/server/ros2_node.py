"""
ROS2 interface for the bimanual teleop server.

Provides
--------
Action server : /bimanual_teleop/follow_joint_trajectory
                (control_msgs/action/FollowJointTrajectory)
                Single-point goal  → moveJ (motion plan)
                Multi-point goal   → servoJ trajectory

Topic sub     : /bimanual_teleop/joint_trajectory
                (trajectory_msgs/JointTrajectory)   fire-and-forget trajectory

Topic sub     : /bimanual_teleop/target_joint_state
                (sensor_msgs/JointState)             fire-and-forget moveJ

Topic pub     : /bimanual_teleop/status
                (std_msgs/String)  JSON snapshot at 2 Hz

Topic pub     : /robot1/joint_states
                (sensor_msgs/JointState)  right arm joints at 10 Hz
                Joint names: robot1_shoulder_pan_joint … robot1_wrist_3_joint

Topic pub     : /robot2/joint_states
                (sensor_msgs/JointState)  left arm joints at 10 Hz
                Joint names: robot2_shoulder_pan_joint … robot2_wrist_3_joint

Topic pub     : /gripper1/joint_states
                (sensor_msgs/JointState)  right gripper at 10 Hz
                Joint name: gripper1_finger_joint  (0.0 open, 1.0 closed)

Topic pub     : /gripper2/joint_states
                (sensor_msgs/JointState)  left gripper at 10 Hz
                Joint name: gripper2_finger_joint  (0.0 open, 1.0 closed)

Robot / topic mapping (matches online.launch.py convention)
-----------------------------------------------------------
robot1 = 192.168.1.10  (right arm in teleop server)  → /robot1/*
robot2 = 192.168.1.20  (left arm  in teleop server)  → /robot2/*

Joint naming convention (for /bimanual_teleop/* interfaces)
-----------------------------------------------------------
Left arm  : left_shoulder_pan, left_shoulder_lift, left_elbow,
            left_wrist_1, left_wrist_2, left_wrist_3, left_gripper
Right arm : right_shoulder_pan, right_shoulder_lift, right_elbow,
            right_wrist_1, right_wrist_2, right_wrist_3, right_gripper

Gripper value: 0.0 = fully open, 1.0 = fully closed.
"""

import json
import threading
from typing import Any, Dict, List, Optional

LEFT_ARM_NAMES = [
    "left_shoulder_pan", "left_shoulder_lift", "left_elbow",
    "left_wrist_1", "left_wrist_2", "left_wrist_3",
]
RIGHT_ARM_NAMES = [
    "right_shoulder_pan", "right_shoulder_lift", "right_elbow",
    "right_wrist_1", "right_wrist_2", "right_wrist_3",
]


def _traj_to_points(traj_msg) -> List[Dict[str, Any]]:
    """Convert trajectory_msgs/JointTrajectory into internal point dicts."""
    names = list(traj_msg.joint_names)
    points: List[Dict[str, Any]] = []

    for pt in traj_msg.points:
        pos = dict(zip(names, pt.positions))
        t   = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9

        lj = [pos.get(n, 0.0) for n in LEFT_ARM_NAMES]
        lg = pos.get("left_gripper", 0.0)
        rj = [pos.get(n, 0.0) for n in RIGHT_ARM_NAMES]
        rg = pos.get("right_gripper", 0.0)

        points.append({
            "left_joints":     lj,
            "left_gripper":    lg,
            "right_joints":    rj,
            "right_gripper":   rg,
            "time_from_start": t,
        })

    return points


class BimanualTeleopROS2Node:
    """
    Manages the rclpy node in a daemon background thread.
    Gracefully does nothing if rclpy is unavailable.
    """

    def __init__(self, robot_manager, state_machine, settings) -> None:
        self._rm       = robot_manager
        self._sm       = state_machine
        self._settings = settings
        self._available = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._spin, daemon=True, name="ros2_node"
        )
        self._thread.start()

    def is_available(self) -> bool:
        return self._available

    # ─── ROS2 spin loop ───────────────────────────────────────────────────

    def _spin(self) -> None:
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            from rclpy.action import ActionServer, GoalResponse, CancelResponse
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String

            rclpy.init()

            rm       = self._rm
            sm       = self._sm
            settings = self._settings

            class _TeleopNode(Node):
                def __init__(self_n):
                    super().__init__("bimanual_teleop")

                    # ── Publishers ────────────────────────────────────────
                    self_n._status_pub = self_n.create_publisher(
                        String, "/bimanual_teleop/status", 10
                    )

                    # Joint state publishers — topic names match online.launch.py:
                    #   robot1 = right arm (192.168.1.10)
                    #   robot2 = left  arm (192.168.1.20)
                    self_n._robot1_js_pub  = self_n.create_publisher(
                        JointState, "/robot1/joint_states", 10
                    )
                    self_n._robot2_js_pub  = self_n.create_publisher(
                        JointState, "/robot2/joint_states", 10
                    )
                    self_n._gripper1_pub   = self_n.create_publisher(
                        JointState, "/gripper1/joint_states", 10
                    )
                    self_n._gripper2_pub   = self_n.create_publisher(
                        JointState, "/gripper2/joint_states", 10
                    )

                    # ── Subscribers ───────────────────────────────────────
                    self_n.create_subscription(
                        JointState,
                        "/bimanual_teleop/target_joint_state",
                        self_n._cb_target_joint_state,
                        10,
                    )
                    self_n.create_subscription(
                        JointTrajectory,
                        "/bimanual_teleop/joint_trajectory",
                        self_n._cb_joint_trajectory,
                        10,
                    )

                    # ── Action Server ─────────────────────────────────────
                    self_n._action_server = ActionServer(
                        self_n,
                        FollowJointTrajectory,
                        "/bimanual_teleop/follow_joint_trajectory",
                        execute_callback=self_n._execute_action,
                        goal_callback=lambda _: GoalResponse.ACCEPT,
                        cancel_callback=lambda _: CancelResponse.ACCEPT,
                    )

                    # ── Status timer (2 Hz) ───────────────────────────────
                    self_n.create_timer(0.5, self_n._pub_status)

                    # ── Joint state timers ────────────────────────────────
                    # 50 Hz — matches the RTDE control loop rate so every
                    # publish carries a genuinely new reading.
                    self_n.create_timer(1.0 / 50, self_n._pub_arm_joint_states)
                    self_n.create_timer(1.0 / 50, self_n._pub_gripper_joint_states)

                    self_n.get_logger().info(
                        "BimanualTeleop ROS2 node ready.\n"
                        "  Action : /bimanual_teleop/follow_joint_trajectory\n"
                        "  Sub    : /bimanual_teleop/target_joint_state\n"
                        "  Sub    : /bimanual_teleop/joint_trajectory\n"
                        "  Pub    : /bimanual_teleop/status          (2 Hz)\n"
                        "  Pub    : /robot1/joint_states             (50 Hz, right arm)\n"
                        "  Pub    : /robot2/joint_states             (50 Hz, left arm)\n"
                        "  Pub    : /gripper1/joint_states           (50 Hz, right gripper)\n"
                        "  Pub    : /gripper2/joint_states           (50 Hz, left gripper)"
                    )

                # ── Callbacks ─────────────────────────────────────────────

                def _guard_motion_plan(self_n, caller: str) -> bool:
                    """Return True if we are in a state that allows motion commands."""
                    from state_machine import SystemState as S
                    state = sm.state
                    if state in (S.MOTION_PLAN_IDLE, S.TELEOP_PASSIVE):
                        if state is S.TELEOP_PASSIVE:
                            sm.transition(S.MOTION_PLAN_IDLE)
                        return True
                    self_n.get_logger().warn(
                        f"[{caller}] Ignored: current state is {state.value}"
                    )
                    return False

                def _cb_target_joint_state(self_n, msg: JointState):
                    if not self_n._guard_motion_plan("target_joint_state"):
                        return
                    n2p = dict(zip(msg.name, msg.position))
                    lj  = [n2p.get(n, 0.0) for n in LEFT_ARM_NAMES]
                    lg  = n2p.get("left_gripper",  0.0)
                    rj  = [n2p.get(n, 0.0) for n in RIGHT_ARM_NAMES]
                    rg  = n2p.get("right_gripper", 0.0)
                    s   = settings.get()
                    rm.execute_move_to_async(
                        lj, lg, rj, rg,
                        s.get("movej_speed",        0.5),
                        s.get("movej_acceleration", 0.3),
                    )

                def _cb_joint_trajectory(self_n, msg: JointTrajectory):
                    if not self_n._guard_motion_plan("joint_trajectory"):
                        return
                    pts = _traj_to_points(msg)
                    if pts:
                        rm.execute_trajectory_async(pts)

                def _execute_action(self_n, goal_handle):
                    """Blocking callback — called in action server thread pool."""
                    from control_msgs.action import FollowJointTrajectory as FJT
                    from state_machine import SystemState as S

                    self_n.get_logger().info("FollowJointTrajectory goal received")

                    traj = goal_handle.request.trajectory
                    pts  = _traj_to_points(traj)

                    if not pts:
                        goal_handle.abort()
                        res = FJT.Result()
                        res.error_code   = FJT.Result.INVALID_GOAL
                        res.error_string = "Empty trajectory"
                        return res

                    # Guard: must be in a valid state
                    state = sm.state
                    if state not in (S.MOTION_PLAN_IDLE, S.TELEOP_PASSIVE,
                                     S.TELEOP_ACTIVE):
                        goal_handle.abort()
                        res = FJT.Result()
                        res.error_code   = FJT.Result.INVALID_GOAL
                        res.error_string = f"Invalid state: {state.value}"
                        return res

                    if state in (S.TELEOP_ACTIVE, S.TELEOP_PASSIVE):
                        sm.transition(S.MOTION_PLAN_IDLE)

                    s = settings.get()

                    # Single point → moveJ; multi-point → trajectory
                    if len(pts) == 1:
                        p  = pts[0]
                        ok = rm.execute_move_to(
                            p["left_joints"],  p["left_gripper"],
                            p["right_joints"], p["right_gripper"],
                            s.get("movej_speed",        0.5),
                            s.get("movej_acceleration", 0.3),
                        )
                    else:
                        ok = rm.execute_trajectory(pts)

                    res = FJT.Result()
                    if ok:
                        res.error_code = FJT.Result.SUCCESSFUL
                        goal_handle.succeed()
                    else:
                        res.error_code   = FJT.Result.PATH_TOLERANCE_VIOLATED
                        res.error_string = "Execution failed or aborted"
                        goal_handle.abort()
                    return res

                def _pub_status(self_n):
                    snap = rm.shared.snapshot()
                    snap["system_state"] = sm.state.value
                    msg      = String()
                    msg.data = json.dumps(snap)
                    self_n._status_pub.publish(msg)

                def _pub_arm_joint_states(self_n):
                    """50 Hz — matches the RTDE control loop so every message has fresh data.
                    robot1 (192.168.1.10) = right arm → /robot1/joint_states
                    robot2 (192.168.1.20) = left arm  → /robot2/joint_states
                    """
                    snap = rm.shared.snapshot()
                    now  = self_n.get_clock().now().to_msg()

                    msg1 = JointState()
                    msg1.header.stamp = now
                    msg1.name         = [
                        "robot1_shoulder_pan_joint", "robot1_shoulder_lift_joint",
                        "robot1_elbow_joint", "robot1_wrist_1_joint",
                        "robot1_wrist_2_joint", "robot1_wrist_3_joint",
                    ]
                    msg1.position     = [float(v) for v in snap["right_joints"]]
                    self_n._robot1_js_pub.publish(msg1)

                    msg2 = JointState()
                    msg2.header.stamp = now
                    msg2.name         = [
                        "robot2_shoulder_pan_joint", "robot2_shoulder_lift_joint",
                        "robot2_elbow_joint", "robot2_wrist_1_joint",
                        "robot2_wrist_2_joint", "robot2_wrist_3_joint",
                    ]
                    msg2.position     = [float(v) for v in snap["left_joints"]]
                    self_n._robot2_js_pub.publish(msg2)

                def _pub_gripper_joint_states(self_n):
                    """50 Hz — matches the RTDE control loop so every message has fresh data.
                    gripper1 = right gripper → /gripper1/joint_states  gripper1_finger_joint
                    gripper2 = left  gripper → /gripper2/joint_states  gripper2_finger_joint
                    """
                    snap = rm.shared.snapshot()
                    now  = self_n.get_clock().now().to_msg()

                    g1 = JointState()
                    g1.header.stamp = now
                    g1.name         = ["gripper1_finger_joint"]
                    g1.position     = [float(snap["right_gripper"])]
                    self_n._gripper1_pub.publish(g1)

                    g2 = JointState()
                    g2.header.stamp = now
                    g2.name         = ["gripper2_finger_joint"]
                    g2.position     = [float(snap["left_gripper"])]
                    self_n._gripper2_pub.publish(g2)

            node = _TeleopNode()
            self._available = True

            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.spin()

        except ImportError as exc:
            print(f"[ROS2] rclpy / control_msgs not available: {exc}")
            print("[ROS2] ROS2 interface disabled (server still functional)")
        except Exception as exc:
            print(f"[ROS2] Node crashed: {exc}")
            import traceback
            traceback.print_exc()
