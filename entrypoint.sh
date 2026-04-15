#!/bin/bash
set -e

# ── Source ROS2 Humble ───────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── Source workspace overlay (bimanual_ur10e + robotiq packages) ─────────────
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# ── ROS domain ───────────────────────────────────────────────────────────────
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── Create settings directory (Docker volume mount) ──────────────────────────
mkdir -p /workspace/settings

echo "================================================================"
echo "  Bimanual UR10e Teleop Interface"
echo "  State: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  Web UI: http://localhost:8080"
echo "================================================================"

CMD="${1:-server}"

case "$CMD" in
    server)
        echo "[Entrypoint] Starting web server..."
        exec python3 /workspace/teleop_interface/server/main.py
        ;;
    ros2)
        # Drop into ROS2 shell (useful for rosbag recording etc.)
        echo "[Entrypoint] ROS2 shell — use 'ros2 launch bimanual_ur10e ...'"
        exec bash
        ;;
    bash|shell)
        exec bash
        ;;
    python*|pytest*|ros2*)
        exec "$@"
        ;;
    *)
        # Allow running arbitrary commands: docker run ... ros2 bag record ...
        exec "$@"
        ;;
esac
