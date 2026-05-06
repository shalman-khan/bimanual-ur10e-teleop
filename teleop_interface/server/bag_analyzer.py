"""
bag_analyzer.py — Post-recording quality analyzer for bimanual DP3 bags.

Reads a recorded bag, applies the dual-condition pause filter on the stored
joint and depth data, and returns quality metrics without modifying the bag.
"""

import os
from typing import Any, Dict

import numpy as np

MOTION_THRESHOLD = 0.015   # rad — max joint delta across either arm's 6 joints
VISION_THRESHOLD = 0.003   # m   — mean absolute depth change over valid pixels
MIN_MOTION_PCT   = 80      # %   — pass threshold
MIN_VALID_PIX    = 5_000
FRAME_DT         = 0.05    # s   — assumed per-frame duration (20 Hz)

ARM1_TOPIC  = '/robot1/joint_states'   # right arm
ARM2_TOPIC  = '/robot2/joint_states'   # left arm
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'


def analyze_bag(bag_path: str) -> Dict[str, Any]:
    """
    Read a rosbag and compute motion-quality metrics.

    Returns a dict with:
        raw_duration_s, filtered_duration_s, total_frames,
        frames_kept, frames_skipped, quality_pct, pass,
        has_depth, bag_path, error (optional)
    """
    base = {
        'raw_duration_s':      0.0,
        'filtered_duration_s': 0.0,
        'total_frames':        0,
        'frames_kept':         0,
        'frames_skipped':      0,
        'quality_pct':         0.0,
        'pass':                False,
        'has_depth':           False,
        'bag_path':            bag_path,
    }

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import JointState, Image
    except ImportError as exc:
        return {**base, 'error': f'ROS2 unavailable: {exc}'}

    if not os.path.exists(bag_path):
        return {**base, 'error': f'Bag path not found: {bag_path}'}

    try:
        storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        conv    = rosbag2_py.ConverterOptions('', '')
        reader  = rosbag2_py.SequentialReader()
        reader.open(storage, conv)

        available  = {t.name for t in reader.get_all_topics_and_types()}
        has_arm1   = ARM1_TOPIC  in available
        has_arm2   = ARM2_TOPIC  in available
        has_depth  = DEPTH_TOPIC in available

        read_topics = [t for t in [ARM1_TOPIC, ARM2_TOPIC, DEPTH_TOPIC] if t in available]
        if read_topics:
            reader.set_filter(rosbag2_py.StorageFilter(topics=read_topics))

        arm1_msgs  = []   # robot1 (right arm): (timestamp_ns, [float …])
        arm2_msgs  = []   # robot2 (left arm):  (timestamp_ns, [float …])
        depth_msgs = []   # (timestamp_ns, np.ndarray)

        while reader.has_next():
            topic, data, ts = reader.read_next()
            try:
                if topic == ARM1_TOPIC:
                    msg = deserialize_message(data, JointState)
                    arm1_msgs.append((ts, list(msg.position)))
                elif topic == ARM2_TOPIC:
                    msg = deserialize_message(data, JointState)
                    arm2_msgs.append((ts, list(msg.position)))
                elif topic == DEPTH_TOPIC:
                    msg = deserialize_message(data, Image)
                    arr = np.frombuffer(bytes(msg.data),
                                        dtype=np.float32).reshape(msg.height, msg.width)
                    depth_msgs.append((ts, arr))
            except Exception:
                continue

        del reader

    except Exception as exc:
        return {**base, 'error': str(exc)}

    joint_msgs = arm1_msgs or arm2_msgs   # prefer whichever is available
    has_arm    = bool(arm1_msgs or arm2_msgs)

    if not has_arm and not depth_msgs:
        return {**base, 'error': 'No joint or depth data found in bag.'}

    all_ts         = ([t for t, _ in arm1_msgs] + [t for t, _ in arm2_msgs]
                      + [t for t, _ in depth_msgs])
    raw_duration_s = (max(all_ts) - min(all_ts)) / 1e9 if len(all_ts) > 1 else 0.0

    if has_depth and depth_msgs:
        kept, skipped = _filter_with_depth(arm1_msgs, arm2_msgs, depth_msgs)
    elif has_arm:
        kept, skipped = _filter_arm_only(arm1_msgs, arm2_msgs)
    else:
        kept, skipped = 0, 0

    total       = kept + skipped
    quality_pct = kept / max(1, total) * 100

    return {
        'raw_duration_s':      raw_duration_s,
        'filtered_duration_s': kept * FRAME_DT,
        'total_frames':        total,
        'frames_kept':         kept,
        'frames_skipped':      skipped,
        'quality_pct':         quality_pct,
        'pass':                quality_pct >= MIN_MOTION_PCT,
        'has_arm1':            bool(arm1_msgs),
        'has_arm2':            bool(arm2_msgs),
        'has_depth':           bool(depth_msgs),
        'bag_path':            bag_path,
    }


# ── Filter implementations ────────────────────────────────────────────────────

def _nearest_joints(msgs_sorted, ts):
    """Return joint positions from msgs_sorted closest to (and at or before) ts."""
    idx = 0
    for i, (t, _) in enumerate(msgs_sorted):
        if t <= ts:
            idx = i
        else:
            break
    return msgs_sorted[idx][1] if msgs_sorted else None


def _filter_with_depth(arm1_msgs, arm2_msgs, depth_msgs):
    """
    Skip a depth frame only when BOTH arms AND vision are simultaneously static.
    Either arm moving is enough to keep the frame.
    Compare against the last KEPT frame so slow continuous motion accumulates.
    """
    arm1_msgs  = sorted(arm1_msgs,  key=lambda x: x[0])
    arm2_msgs  = sorted(arm2_msgs,  key=lambda x: x[0])
    depth_msgs = sorted(depth_msgs, key=lambda x: x[0])

    kept = skipped = 0
    prev_arm1 = prev_arm2 = prev_depth = None
    first = True

    for d_ts, depth_arr in depth_msgs:
        curr_arm1 = _nearest_joints(arm1_msgs, d_ts)
        curr_arm2 = _nearest_joints(arm2_msgs, d_ts)

        # Arm is static only if BOTH arms are static (or unavailable)
        arm1_static = _arm_is_static(prev_arm1, curr_arm1)
        arm2_static = _arm_is_static(prev_arm2, curr_arm2)
        arm_static  = arm1_static and arm2_static

        vis_static  = _vision_is_static(prev_depth, depth_arr)
        skip        = arm_static and vis_static and not first

        if skip:
            skipped += 1
        else:
            kept     += 1
            prev_arm1 = curr_arm1
            prev_arm2 = curr_arm2
            prev_depth = depth_arr
            first      = False

    return kept, skipped


def _filter_arm_only(arm1_msgs, arm2_msgs):
    """Arm-only fallback when depth is unavailable. Keep if either arm moves."""
    # Merge both arm streams sorted by timestamp
    all_msgs = sorted(arm1_msgs + arm2_msgs, key=lambda x: x[0])

    # Build per-arm timeseries for joint lookup
    arm1_sorted = sorted(arm1_msgs, key=lambda x: x[0])
    arm2_sorted = sorted(arm2_msgs, key=lambda x: x[0])

    kept = skipped = 0
    prev_arm1 = prev_arm2 = None
    first = True
    seen_ts = set()

    # Iterate at depth-equivalent rate: sample every ~50ms window
    if not all_msgs:
        return 0, 0

    t_start = all_msgs[0][0]
    t_end   = all_msgs[-1][0]
    step_ns = int(0.05 * 1e9)   # 50 ms

    t = t_start
    while t <= t_end:
        curr_arm1 = _nearest_joints(arm1_sorted, t)
        curr_arm2 = _nearest_joints(arm2_sorted, t)

        arm1_static = _arm_is_static(prev_arm1, curr_arm1)
        arm2_static = _arm_is_static(prev_arm2, curr_arm2)
        skip        = arm1_static and arm2_static and not first

        if skip:
            skipped += 1
        else:
            kept     += 1
            prev_arm1 = curr_arm1
            prev_arm2 = curr_arm2
            first      = False

        t += step_ns

    return kept, skipped


def _arm_is_static(prev, curr) -> bool:
    if prev is None or curr is None:
        return False
    return max(abs(c - p) for c, p in zip(curr, prev)) < MOTION_THRESHOLD


def _vision_is_static(prev: np.ndarray, curr: np.ndarray) -> bool:
    if prev is None or curr is None:
        return False
    valid = np.isfinite(prev) & np.isfinite(curr)
    if valid.sum() < MIN_VALID_PIX:
        return False
    return float(np.abs(curr[valid] - prev[valid]).mean()) < VISION_THRESHOLD
