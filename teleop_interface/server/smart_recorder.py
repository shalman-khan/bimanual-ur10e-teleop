#!/usr/bin/env python3
"""
smart_recorder.py — Pause-filtered rosbag recorder for bimanual UR10e teleop.

Usage: python3 smart_recorder.py <bag_path>

Subscribes to the configured topics, applies a dual-condition pause filter
(arm joints + depth vision), writes kept frames to a rosbag2 sqlite3 bag,
prints a live ANSI HUD, and writes stats to /tmp/smart_recorder_stats.json.
"""

import json
import os
import signal
import struct
import sys
import time
import threading
from collections import deque

# ── Validate imports early ────────────────────────────────────────────────────
try:
    import numpy as np
except ImportError:
    print("[smart_recorder] ERROR: numpy not available. Install it first.", flush=True)
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState, Image, CameraInfo
    from geometry_msgs.msg import WrenchStamped
except ImportError as e:
    print(f"[smart_recorder] ERROR: rclpy not importable: {e}", flush=True)
    sys.exit(1)

try:
    import rosbag2_py
except ImportError as e:
    print(f"[smart_recorder] ERROR: rosbag2_py not importable: {e}", flush=True)
    sys.exit(1)

# ── Configuration ─────────────────────────────────────────────────────────────
# Slow cable manipulation moves ~0.003 rad/step — threshold must be below that
MOTION_THRESHOLD = 0.003   # rad — arm joint delta (was 0.015, too high for slow teleop)
VISION_THRESHOLD = 0.003   # m   — mean depth delta threshold
MIN_MOTION_PCT   = 70      # %   — quality gate (was 80)
MIN_VALID_PIX    = 5_000   # minimum finite pixels needed for vision check
FRAME_DT         = 0.05    # s   — nominal period per kept frame (50 ms)
STATS_FILE       = "/tmp/smart_recorder_stats.json"

DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
ARM_TOPIC   = '/robot2/joint_states'   # only active arm monitored for motion

# Camera topics — ALWAYS written at full native rate, NEVER filtered.
# DP3 needs point cloud at 28Hz regardless of arm motion.
CAMERA_TOPICS = {
    DEPTH_TOPIC,
    '/zed/zed_node/depth/camera_info',
    '/zed/zed_node/rgb/color/rect/image',
}

# Robot state topics — written only when motion detected (filtered).
ROBOT_TOPICS = {
    '/robot1/joint_states',
    '/robot2/joint_states',
    '/gripper1/joint_states',
    '/gripper2/joint_states',
    '/robot1/wrench',
    '/robot2/wrench',
}

TOPIC_TYPES = [
    ('/robot1/joint_states',                'sensor_msgs/msg/JointState'),
    ('/robot2/joint_states',                'sensor_msgs/msg/JointState'),
    ('/gripper1/joint_states',              'sensor_msgs/msg/JointState'),
    ('/gripper2/joint_states',              'sensor_msgs/msg/JointState'),
    ('/robot1/wrench',                      'geometry_msgs/msg/WrenchStamped'),
    ('/robot2/wrench',                      'geometry_msgs/msg/WrenchStamped'),
    (DEPTH_TOPIC,                           'sensor_msgs/msg/Image'),
    ('/zed/zed_node/depth/camera_info',     'sensor_msgs/msg/CameraInfo'),
    ('/zed/zed_node/rgb/color/rect/image',  'sensor_msgs/msg/Image'),
]

# ── ANSI helpers ──────────────────────────────────────────────────────────────
_G   = '\033[32m'       # green
_Y   = '\033[33m'       # yellow/amber
_R   = '\033[31m'       # red
_B   = '\033[1m'        # bold
_DIM = '\033[2m'        # dim
_RST = '\033[0m'        # reset
_CLR = '\033[K'         # clear to EOL

HUD_LINES = 4  # maximum HUD lines printed


def _cursor_up(n: int) -> str:
    return f'\033[{n}A' if n > 0 else ''


# ── Filter helpers ────────────────────────────────────────────────────────────
def arm_is_static(prev, curr) -> bool:
    if prev is None or curr is None:
        return False
    try:
        return max(abs(c - p) for c, p in zip(curr, prev)) < MOTION_THRESHOLD
    except Exception:
        return False


def vision_is_static(prev_arr, curr_arr) -> bool:
    if prev_arr is None or curr_arr is None:
        return False
    try:
        valid = np.isfinite(prev_arr) & np.isfinite(curr_arr)
        if int(valid.sum()) < MIN_VALID_PIX:
            return False
        return float(np.abs(curr_arr[valid] - prev_arr[valid]).mean()) < VISION_THRESHOLD
    except Exception:
        return False


def _depth_to_array(depth_msg) -> np.ndarray:
    """Convert a sensor_msgs/Image depth message to a float32 numpy array."""
    raw = bytes(depth_msg.data)
    h, w = depth_msg.height, depth_msg.width
    enc = depth_msg.encoding.lower()
    if enc in ('32fc1', '32fc'):
        arr = np.frombuffer(raw, dtype='<f4')
    elif enc in ('16uc1', '16uc', 'mono16'):
        arr = np.frombuffer(raw, dtype='<u2').astype(np.float32) * 0.001
    else:
        # Attempt raw float32 regardless
        arr = np.frombuffer(raw, dtype='<f4')
    return arr.reshape(h, w) if arr.size == h * w else arr


def _msg_stamp_ns(msg, node) -> int:
    """Return message timestamp in nanoseconds."""
    try:
        s = msg.header.stamp
        return int(s.sec) * 1_000_000_000 + int(s.nanosec)
    except AttributeError:
        return node.get_clock().now().nanoseconds


# ── Quality report builder ────────────────────────────────────────────────────
def _build_report(frames_kept: int, frames_skipped: int, elapsed_s: float) -> dict:
    total = frames_kept + frames_skipped
    quality_pct = (frames_kept / total * 100.0) if total > 0 else 0.0
    filtered_s  = frames_kept * FRAME_DT
    return {
        'pass':               quality_pct >= MIN_MOTION_PCT,
        'raw_duration_s':     elapsed_s,
        'filtered_duration_s': filtered_s,
        'frames_kept':        frames_kept,
        'frames_skipped':     frames_skipped,
        'quality_pct':        quality_pct,
    }


# ── Operator tips banner ──────────────────────────────────────────────────────
def _print_tips(bag_path: str) -> None:
    bar = '━' * 50
    print(f"""
{bar}
  RECORDING TIPS — read before starting
{bar}
  {_B}Setup:{_RST}
  {_G}✓{_RST}  Robot 1 (left)  — stays STATIC, holds cable
  {_G}✓{_RST}  Robot 2 (right) — YOU control this arm only

  {_B}How to teleoperate:{_RST}
  {_G}✓{_RST}  Plan the full motion BEFORE pressing record
  {_G}✓{_RST}  Approach cable → grasp → pull → release
  {_G}✓{_RST}  Move robot 2 smoothly and continuously
  {_G}✓{_RST}  Cable swinging while arm is still = OK (kept)
  {_R}✗{_RST}  Do NOT stop mid-task to think or reposition
  {_R}✗{_RST}  Do NOT freeze robot 2 for more than 0.5s

  {_B}Filter behaviour:{_RST}
  System monitors robot 2 joints + depth camera.
  Frames dropped only when BOTH arm AND scene frozen.
  Camera always recorded at full rate (28 Hz).
  Target motion: > 70%  (aim for > 85%)
{bar}

  Recording to: {bag_path}
""", flush=True)


# ── Quality report printer ────────────────────────────────────────────────────
def _print_report(report: dict) -> None:
    ok  = report['pass']
    pct = report['quality_pct']
    if ok:
        verdict_lines = [f"  {_G}{_B}✓  KEEP — clean demo{_RST}                       "]
    else:
        verdict_lines = [
            f"  {_R}{_B}✗  DISCARD — below 70% motion threshold{_RST}   ",
            f"  {_R}   This bag will NOT produce a working{_RST}         ",
            f"  {_R}   policy. DELETE IT and re-record.{_RST}            ",
        ]

    def row(label, value):
        lbl = f"  {label}"
        combined = f"{lbl:<25}: {value}"
        return f"║  {combined:<44}║"

    lines = [
        '╔══════════════════════════════════════════════╗',
        '║  BAG QUALITY REPORT                          ║',
        '║                                              ║',
        row('Raw duration',      f"{report['raw_duration_s']:.1f}s"),
        row('Filtered duration', f"{report['filtered_duration_s']:.1f}s  (frames kept)"),
        row('Frames recorded',   str(report['frames_kept'])),
        row('Frames skipped',    str(report['frames_skipped'])),
        row('Motion',            f"{pct:.1f}%"),
        '║                                              ║',
    ]
    for vl in verdict_lines:
        lines.append(f'║{vl}║')
    lines.append('╚══════════════════════════════════════════════╝')

    print('\n'.join(lines), flush=True)


# ── HUD printer ───────────────────────────────────────────────────────────────
_hud_printed = 0  # how many HUD lines were last printed


def _fmt_elapsed(s: float) -> str:
    m  = int(s) // 60
    ss = s - m * 60
    return f"{m:02d}:{ss:04.1f}"


def _print_hud(state: str, elapsed_s: float, frames_kept: int, frames_skipped: int,
               arm_moving: bool, vision_moving: bool) -> None:
    global _hud_printed

    total     = frames_kept + frames_skipped
    skip_pct  = (frames_skipped / total * 100) if total > 0 else 0
    qual_pct  = (frames_kept    / total * 100) if total > 0 else 100.0
    filt_s    = frames_kept * FRAME_DT
    elapsed_f = _fmt_elapsed(elapsed_s)

    # Line 1 — state / counters
    if state == 'recording':
        icon, label, col = '●', 'REC ', _G
    elif state == 'cable_inertia':
        icon, label, col = '◉', 'REC ', _Y
    else:
        icon, label, col = '◌', 'SKIP', _DIM

    line1 = (f"{col}{_B}{icon} {label}{_RST}  │  {elapsed_f}"
             f"  │  Motion: {frames_kept} frames"
             f"  │  Skipped: {frames_skipped} ({skip_pct:.0f}%)"
             f"  │  Filtered time: {filt_s:.1f}s")

    # Line 2 — ARM / VISION bars
    arm_bar    = f"{_G}████{_RST}" if arm_moving    else f"{_DIM}░░░░{_RST}"
    vision_bar = f"{_G}████{_RST}" if vision_moving else f"{_DIM}░░░░{_RST}"
    note = ''
    if state == 'cable_inertia':
        note = f'  {_Y}(cable inertia){_RST}'
    elif state == 'paused':
        note = f'  {_DIM}(paused){_RST}'
    line2 = f"  ARM {arm_bar}  VISION {vision_bar}{note}"

    # Line 3 — quality bar
    filled = max(0, min(10, round(qual_pct / 10)))
    q_col  = _G if qual_pct >= 70 else (_Y if qual_pct >= 55 else _R)
    q_bar  = f"{q_col}{'█' * filled}{'░' * (10 - filled)}{_RST}"
    line3  = f"  Quality {q_bar}  {q_col}{qual_pct:.0f}%{_RST}  target > 70%"

    # Line 4 (optional warning)
    line4 = None
    if qual_pct < 70:
        line4 = f"  {_R}{_B}⚠  QUALITY TOO LOW — STOP AND RESTART THIS DEMO{_RST}"

    lines   = [line1, line2, line3]
    if line4:
        lines.append(line4)

    # Move cursor up to overwrite previous HUD
    if _hud_printed > 0:
        sys.stdout.write(_cursor_up(_hud_printed))

    for ln in lines:
        sys.stdout.write(f"\r{_CLR}{ln}\n")

    _hud_printed = len(lines)
    sys.stdout.flush()


# ── Main recorder node ────────────────────────────────────────────────────────
class SmartRecorder(Node):
    def __init__(self, bag_path: str):
        super().__init__('smart_recorder')
        self._bag_path    = bag_path
        self._lock        = threading.Lock()
        self._stopping    = False
        self._first_frame = True
        self._start_time  = time.monotonic()

        # Shared state (protected by _lock)
        self._latest_joints: list | None = None
        self._prev_joints:   list | None = None
        self._prev_depth:    np.ndarray | None = None
        self._frames_kept    = 0
        self._frames_skipped = 0
        self._arm_moving     = False
        self._vision_moving  = False
        self._cur_state      = 'recording'

        # Per-topic message buffers
        self._buffers: dict[str, deque] = {t: deque() for t, _ in TOPIC_TYPES}

        # Open bag writer
        os.makedirs(os.path.dirname(os.path.abspath(bag_path)), exist_ok=True)
        storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        conv    = rosbag2_py.ConverterOptions('', '')
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(storage, conv)
        for topic, type_str in TOPIC_TYPES:
            self._writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_str, serialization_format='cdr'))

        # Subscriptions
        self._subs = []
        for topic, type_str in TOPIC_TYPES:
            msg_type = self._type_str_to_class(type_str)
            if msg_type is None:
                self.get_logger().warn(f"Unknown type for topic {topic}: {type_str}")
                continue
            if topic == ARM_TOPIC:
                cb = self._make_arm_cb(topic)
            elif topic == DEPTH_TOPIC:
                cb = self._depth_cb
            else:
                cb = self._make_generic_cb(topic)
            sub = self.create_subscription(msg_type, topic, cb, 10)
            self._subs.append(sub)

        self.get_logger().info(f"SmartRecorder ready — bag: {bag_path}")

    @staticmethod
    def _type_str_to_class(type_str: str):
        mapping = {
            'sensor_msgs/msg/JointState':    JointState,
            'sensor_msgs/msg/Image':         Image,
            'sensor_msgs/msg/CameraInfo':    CameraInfo,
            'geometry_msgs/msg/WrenchStamped': WrenchStamped,
        }
        return mapping.get(type_str)

    # ── Generic buffer callback ───────────────────────────────────────────────
    def _make_generic_cb(self, topic: str):
        def cb(msg):
            with self._lock:
                self._buffers[topic].append(msg)
        return cb

    # ── ARM callback: buffer + update latest joints ────────────────────────────
    def _make_arm_cb(self, topic: str):
        def cb(msg):
            with self._lock:
                self._buffers[topic].append(msg)
                self._latest_joints = list(msg.position)
        return cb

    # ── DEPTH callback: frame trigger ─────────────────────────────────────────
    def _depth_cb(self, msg):
        with self._lock:
            self._buffers[DEPTH_TOPIC].append(msg)
            self._process_frame(msg)

    def _process_frame(self, depth_msg):
        """Called under lock. Decides keep/skip and writes to bag."""
        if self._stopping:
            return

        elapsed = time.monotonic() - self._start_time

        # Convert depth image
        try:
            curr_depth = _depth_to_array(depth_msg)
        except Exception as e:
            self.get_logger().warn(f"Depth convert error: {e}")
            curr_depth = None

        # Valid pixel count
        valid_pix = int(np.isfinite(curr_depth).sum()) if curr_depth is not None else 0

        # Evaluate filters
        arm_static    = arm_is_static(self._prev_joints, self._latest_joints)
        vis_static    = vision_is_static(self._prev_depth, curr_depth)
        force_keep    = self._first_frame or (valid_pix < MIN_VALID_PIX)

        self._arm_moving    = not arm_static
        self._vision_moving = not vis_static

        keep = (not arm_static) or (not vis_static) or force_keep

        # Determine display state
        if keep and not arm_static and not vis_static:
            self._cur_state = 'recording'
        elif keep and arm_static and not vis_static:
            self._cur_state = 'cable_inertia'
        elif keep and force_keep:
            self._cur_state = 'recording'
        else:
            self._cur_state = 'paused'

        # Camera topics: ALWAYS write at full native rate (28 Hz).
        # DP3 needs continuous point cloud regardless of arm motion.
        # Robot state topics: only write when motion detected.
        self._write_camera_buffers()

        if keep:
            self._write_robot_buffers()
            self._prev_joints = list(self._latest_joints) if self._latest_joints else None
            self._prev_depth  = curr_depth
            self._frames_kept += 1
            self._first_frame  = False
        else:
            # Discard only robot state buffers — camera already written above
            for topic, q in self._buffers.items():
                if topic in ROBOT_TOPICS:
                    q.clear()
            self._frames_skipped += 1

        # Update stats file
        total       = self._frames_kept + self._frames_skipped
        quality_pct = (self._frames_kept / total * 100.0) if total > 0 else 100.0
        self._write_stats(elapsed, quality_pct, quality_report=None)

        # Print HUD
        _print_hud(
            self._cur_state, elapsed,
            self._frames_kept, self._frames_skipped,
            self._arm_moving, self._vision_moving,
        )

    def _write_camera_buffers(self):
        """Flush camera topic buffers — called every depth frame regardless of motion."""
        for topic, q in self._buffers.items():
            if topic in CAMERA_TOPICS:
                while q:
                    msg = q.popleft()
                    ts  = _msg_stamp_ns(msg, self)
                    try:
                        self._writer.write(topic, serialize_message(msg), ts)
                    except Exception as e:
                        print(f"\n[smart_recorder] Warning: write error for {topic}: {e}", flush=True)

    def _write_robot_buffers(self):
        """Flush robot state topic buffers — called only when motion detected."""
        for topic, q in self._buffers.items():
            if topic in ROBOT_TOPICS:
                while q:
                    msg = q.popleft()
                    ts  = _msg_stamp_ns(msg, self)
                    try:
                        self._writer.write(topic, serialize_message(msg), ts)
                    except Exception as e:
                        print(f"\n[smart_recorder] Warning: write error for {topic}: {e}", flush=True)

    def _write_buffers(self):
        """Flush all topic buffers (used on final stop)."""
        for topic, q in self._buffers.items():
            while q:
                msg = q.popleft()
                ts  = _msg_stamp_ns(msg, self)
                try:
                    self._writer.write(topic, serialize_message(msg), ts)
                except Exception as e:
                    print(f"\n[smart_recorder] Warning: write error for {topic}: {e}", flush=True)

    def _write_stats(self, elapsed: float, quality_pct: float, quality_report):
        filtered_s = self._frames_kept * FRAME_DT
        data = {
            'recording':          quality_report is None,
            'bag_path':           self._bag_path,
            'state':              self._cur_state,
            'elapsed_s':          round(elapsed, 2),
            'frames_kept':        self._frames_kept,
            'frames_skipped':     self._frames_skipped,
            'quality_pct':        round(quality_pct, 2),
            'filtered_duration_s': round(filtered_s, 2),
            'arm_moving':         self._arm_moving,
            'vision_moving':      self._vision_moving,
            'quality_report':     quality_report,
        }
        try:
            tmp = STATS_FILE + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, STATS_FILE)
        except Exception as e:
            pass  # Non-fatal

    def stop(self):
        """Graceful stop: flush last frame, write report, close bag."""
        with self._lock:
            if self._stopping:
                return
            self._stopping = True

            elapsed = time.monotonic() - self._start_time

            # Flush whatever is still buffered (last partial frame)
            self._write_buffers()

            # Build quality report
            report = _build_report(self._frames_kept, self._frames_skipped, elapsed)

            total       = self._frames_kept + self._frames_skipped
            quality_pct = report['quality_pct']

            # Write final stats with quality report
            self._write_stats(elapsed, quality_pct, report)

        # Close writer outside lock to avoid deadlock
        try:
            del self._writer
        except Exception:
            pass

        # Print final report
        print('\n', flush=True)
        _print_report(report)


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 smart_recorder.py <bag_path>", flush=True)
        sys.exit(1)

    bag_path = sys.argv[1]

    rclpy.init()

    recorder = SmartRecorder(bag_path)

    # SIGTERM / SIGINT handler (idempotent via _stopping flag)
    def _handle_signal(signum, frame):
        recorder.stop()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT,  _handle_signal)

    _print_tips(bag_path)

    try:
        rclpy.spin(recorder)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
