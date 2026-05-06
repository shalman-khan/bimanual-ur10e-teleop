"""
bag_analyzer.py — Post-recording quality analyzer for bimanual DP3 bags.

Reads a recorded bag, applies the dual-condition pause filter on the stored
joint and depth data, and returns quality metrics without modifying the bag.
"""

import os
from typing import Any, Dict

import numpy as np

MOTION_THRESHOLD = 0.015   # rad — max joint delta across robot2's 6 joints
VISION_THRESHOLD = 0.003   # m   — mean absolute depth change over valid pixels
MIN_MOTION_PCT   = 80      # %   — pass threshold
MIN_VALID_PIX    = 5_000
FRAME_DT         = 0.05    # s   — assumed per-frame duration (20 Hz)

ARM_TOPIC   = '/robot2/joint_states'
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
        has_arm    = ARM_TOPIC   in available
        has_depth  = DEPTH_TOPIC in available

        read_topics = [t for t in [ARM_TOPIC, DEPTH_TOPIC] if t in available]
        if read_topics:
            reader.set_filter(rosbag2_py.StorageFilter(topics=read_topics))

        joint_msgs = []   # (timestamp_ns, [float …])
        depth_msgs = []   # (timestamp_ns, np.ndarray)

        while reader.has_next():
            topic, data, ts = reader.read_next()
            try:
                if topic == ARM_TOPIC:
                    msg = deserialize_message(data, JointState)
                    joint_msgs.append((ts, list(msg.position)))
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

    if not joint_msgs and not depth_msgs:
        return {**base, 'error': 'No joint or depth data found in bag.'}

    all_ts         = [t for t, _ in joint_msgs] + [t for t, _ in depth_msgs]
    raw_duration_s = (max(all_ts) - min(all_ts)) / 1e9 if len(all_ts) > 1 else 0.0

    if has_depth and depth_msgs:
        kept, skipped = _filter_with_depth(joint_msgs, depth_msgs)
    elif has_arm and joint_msgs:
        kept, skipped = _filter_arm_only(joint_msgs)
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
        'has_depth':           bool(depth_msgs),
        'bag_path':            bag_path,
    }


# ── Filter implementations ────────────────────────────────────────────────────

def _filter_with_depth(joint_msgs, depth_msgs):
    joint_msgs = sorted(joint_msgs, key=lambda x: x[0])
    depth_msgs = sorted(depth_msgs, key=lambda x: x[0])

    kept = skipped = 0
    prev_joints = prev_depth = None
    j_idx = 0

    for d_ts, depth_arr in depth_msgs:
        while j_idx + 1 < len(joint_msgs) and joint_msgs[j_idx + 1][0] <= d_ts:
            j_idx += 1
        curr_joints = joint_msgs[j_idx][1] if joint_msgs else None

        arm_static = _arm_is_static(prev_joints, curr_joints)
        vis_static = _vision_is_static(prev_depth, depth_arr)
        skip       = arm_static and vis_static and prev_joints is not None

        if skip:
            skipped += 1
        else:
            kept       += 1
            prev_joints = curr_joints
            prev_depth  = depth_arr

    return kept, skipped


def _filter_arm_only(joint_msgs):
    kept = skipped = 0
    prev = None
    for _, joints in sorted(joint_msgs, key=lambda x: x[0]):
        if not _arm_is_static(prev, joints):
            kept += 1
        else:
            skipped += 1
        prev = joints
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
