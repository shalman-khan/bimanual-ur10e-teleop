"""
bag_analyzer.py — Post-recording quality analyzer and filter for bimanual DP3.

Reads the original bag, applies the dual-condition pause filter, and writes a
new filtered bag that has static (thinking-pause) frames removed.

Filter logic (skip a frame only when ALL are simultaneously true):
  - Both robot1 AND robot2 joints are static relative to last kept frame
  - Depth scene is static relative to last kept frame
  - Not the first frame

Motion quality (%) = frames_kept / total_depth_frames × 100
This measures what fraction of the original demo had actual motion.
"""

import os
from typing import Any, Dict, Tuple

import numpy as np

MOTION_THRESHOLD = 0.003   # rad — slow cable manipulation is ~0.003 rad/step (was 0.015)
VISION_THRESHOLD = 0.003   # m   — mean absolute depth change over valid pixels
MIN_MOTION_PCT   = 70      # %   — pass threshold (was 80)
MIN_VALID_PIX    = 5_000

ARM1_TOPIC  = '/robot1/joint_states'   # fixed arm (static by design)
ARM2_TOPIC  = '/robot2/joint_states'   # active arm (monitor this for motion)
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'


def analyze_bag(bag_path: str) -> Dict[str, Any]:
    """
    Read the original bag, apply the pause filter, and write a filtered bag.

    The filtered bag is written to <bag_path>_filtered and contains only
    frames where at least one arm OR the visual scene was moving.

    Returns a dict with quality metrics and the filtered bag path.
    """
    base = {
        'raw_duration_s':        0.0,
        'filtered_duration_s':   0.0,
        'total_frames':          0,
        'frames_kept':           0,
        'frames_skipped':        0,
        'quality_pct':           0.0,
        'pass':                  False,
        'has_arm1':              False,
        'has_arm2':              False,
        'has_depth':             False,
        'bag_path':              bag_path,
        'filtered_bag_path':     None,
    }

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import JointState, Image
    except ImportError as exc:
        return {**base, 'error': f'ROS2 unavailable: {exc}'}

    if not os.path.exists(bag_path):
        return {**base, 'error': f'Bag path not found: {bag_path}'}

    # metadata.yaml is written by the recorder on clean shutdown.  If it's
    # missing the recorder was likely force-killed (SIGKILL after timeout).
    meta_path = os.path.join(bag_path, 'metadata.yaml')
    if not os.path.exists(meta_path):
        return {**base, 'error': (
            f'Bag metadata missing — recorder was likely killed before it could '
            f'finalise the bag ({meta_path}). '
            f'Try increasing the stop timeout or re-recording.'
        )}

    filtered_bag_path = bag_path + '_filtered'

    try:
        # ── Open reader (all topics, no filter) ───────────────────────────
        # storage_id='' lets rosbag2 auto-detect the format from metadata.yaml
        # (avoids hard-coding 'sqlite3' which fails when bags are recorded as mcap)
        storage_r = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
        conv      = rosbag2_py.ConverterOptions('', '')
        reader    = rosbag2_py.SequentialReader()
        reader.open(storage_r, conv)

        topic_meta = reader.get_all_topics_and_types()
        available  = {t.name for t in topic_meta}
        has_arm1   = ARM1_TOPIC  in available
        has_arm2   = ARM2_TOPIC  in available
        has_depth  = DEPTH_TOPIC in available

        # ── Open writer for filtered bag ──────────────────────────────────
        os.makedirs(os.path.dirname(os.path.abspath(filtered_bag_path)), exist_ok=True)
        storage_w = rosbag2_py.StorageOptions(uri=filtered_bag_path, storage_id='sqlite3')
        writer    = rosbag2_py.SequentialWriter()
        writer.open(storage_w, conv)
        for t in topic_meta:
            writer.create_topic(t)

        # ── Streaming filter ──────────────────────────────────────────────
        # Buffer messages between depth frames; flush to writer on keep,
        # discard on skip.  Only deserialize arm joints and depth images.
        buffer     = []          # (topic, raw_data, timestamp_ns)
        latest_a1  = None        # most recent robot1 joints in buffer window
        latest_a2  = None        # most recent robot2 joints in buffer window
        prev_a1    = None        # joints at last KEPT frame
        prev_a2    = None
        prev_depth = None        # depth array at last KEPT frame
        first      = True
        kept       = 0
        skipped    = 0
        all_ts     = []

        while reader.has_next():
            topic, data, ts = reader.read_next()
            all_ts.append(ts)

            if topic == ARM1_TOPIC:
                try:
                    msg = deserialize_message(data, JointState)
                    latest_a1 = list(msg.position)
                except Exception:
                    pass
                buffer.append((topic, data, ts))

            elif topic == ARM2_TOPIC:
                try:
                    msg = deserialize_message(data, JointState)
                    latest_a2 = list(msg.position)
                except Exception:
                    pass
                buffer.append((topic, data, ts))

            elif topic == DEPTH_TOPIC:
                # Frame decision point
                try:
                    msg = deserialize_message(data, Image)
                    depth_arr = np.frombuffer(bytes(msg.data),
                                              dtype=np.float32).reshape(
                                              msg.height, msg.width)
                except Exception:
                    depth_arr = None

                buffer.append((topic, data, ts))

                arm1_static = _arm_is_static(prev_a1, latest_a1)
                arm2_static = _arm_is_static(prev_a2, latest_a2)
                # Skip only when BOTH arms AND vision are simultaneously static
                arm_static  = arm1_static and arm2_static
                vis_static  = _vision_is_static(prev_depth, depth_arr)
                skip        = arm_static and vis_static and not first

                if skip:
                    skipped += 1
                    buffer.clear()
                else:
                    kept += 1
                    for t_item, d_item, ts_item in buffer:
                        try:
                            writer.write(t_item, d_item, ts_item)
                        except Exception:
                            pass
                    buffer.clear()
                    prev_a1    = latest_a1
                    prev_a2    = latest_a2
                    prev_depth = depth_arr
                    first      = False

            else:
                buffer.append((topic, data, ts))

        # Write any remaining buffered messages (non-depth tail)
        if not first and buffer:
            for t_item, d_item, ts_item in buffer:
                try:
                    writer.write(t_item, d_item, ts_item)
                except Exception:
                    pass

        del reader
        del writer

    except Exception as exc:
        return {**base, 'error': str(exc)}

    # ── Quality metrics ───────────────────────────────────────────────────
    raw_duration_s = (max(all_ts) - min(all_ts)) / 1e9 if len(all_ts) > 1 else 0.0
    total          = kept + skipped
    quality_pct    = kept / max(1, total) * 100
    frame_dt       = raw_duration_s / max(1, total)   # actual per-frame duration
    filtered_dur   = kept * frame_dt

    return {
        'raw_duration_s':      raw_duration_s,
        'filtered_duration_s': filtered_dur,
        'total_frames':        total,
        'frames_kept':         kept,
        'frames_skipped':      skipped,
        'quality_pct':         quality_pct,
        'pass':                quality_pct >= MIN_MOTION_PCT,
        'has_arm1':            has_arm1,
        'has_arm2':            has_arm2,
        'has_depth':           has_depth,
        'bag_path':            bag_path,
        'filtered_bag_path':   filtered_bag_path,
    }


# ── Filter helpers ────────────────────────────────────────────────────────────

def _arm_is_static(prev, curr) -> bool:
    """True if max joint delta across all 6 joints is below threshold."""
    if prev is None or curr is None:
        return False
    return max(abs(c - p) for c, p in zip(curr, prev)) < MOTION_THRESHOLD


def _vision_is_static(prev: np.ndarray, curr: np.ndarray) -> bool:
    """True if mean absolute depth change over valid pixels is below threshold."""
    if prev is None or curr is None:
        return False
    valid = np.isfinite(prev) & np.isfinite(curr)
    if valid.sum() < MIN_VALID_PIX:
        return False   # sparse depth → assume moving (keep frame)
    return float(np.abs(curr[valid] - prev[valid]).mean()) < VISION_THRESHOLD
