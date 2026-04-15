# ════════════════════════════════════════════════════════════════════════════
#  Bimanual UR10e Teleop — Docker Image
#  Base: ros:humble-ros-base (Ubuntu 22.04)
#
#  Contains:
#    - gello_software          (ur-rtde teleoperation via Dynamixel GELLO)
#    - bimanual_ur10e          (ROS2 package for visualization / rosbag)
#    - robotiq packages        (description + driver, cloned from GitHub)
#    - teleop_interface        (FastAPI server + Web UI on :8080)
#    - ROS2 action interface   (control_msgs/action/FollowJointTrajectory)
# ════════════════════════════════════════════════════════════════════════════

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ── System packages ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    # Core ROS2 packages
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    # UR driver (pulls ur_description as dep)
    ros-humble-ur-robot-driver \
    # ros2_control (needed by robotiq_description's rosdep)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    # Message packages needed by teleop interface
    ros-humble-trajectory-msgs \
    ros-humble-control-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-action-msgs \
    # Rosbag
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbag2-transport \
    # Image (optional, used in bimanual_ur10e launch)
    ros-humble-image-transport \
    # USB/HID for GELLO Dynamixel
    libhidapi-dev \
    usbutils \
    # Port diagnostics (used by GELLO driver)
    lsof \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ───────────────────────────────────────────────────────
# Install core Python packages needed by gello_software and the teleop server
RUN pip3 install --no-cache-dir \
    # Web server
    fastapi \
    "uvicorn[standard]" \
    # gello_software core deps
    numpy \
    tyro \
    omegaconf \
    pyzmq \
    termcolor \
    pyquaternion \
    # Dynamixel SDK (for GELLO device)
    dynamixel-sdk \
    # UR robot control (ur-rtde wraps C++ RTDE)
    ur-rtde \
    # Testing (use pip version to match uvicorn/anyio, not the old apt version)
    "pytest>=7.0" \
    httpx

# ── Copy source repositories ──────────────────────────────────────────────────
COPY gello_software  /workspace/gello_software
COPY bimanual_ur10e  /workspace/bimanual_ur10e
COPY teleop_interface /workspace/teleop_interface

# ── Install gello_software as editable package (no extra deps — installed above) ──
RUN pip3 install --no-cache-dir -e /workspace/gello_software --no-deps

# ── Clone ONLY robotiq_description (not the C++ driver — we use socket control) ──
# Cloning the full repo then isolating just the description package prevents
# colcon from trying to build robotiq_driver (which needs extra C++ deps).
RUN git clone --depth=1 \
        https://github.com/PickNikRobotics/ros2_robotiq_gripper.git \
        /tmp/robotiq_gripper && \
    mkdir -p /ros2_ws/src && \
    cp -r /tmp/robotiq_gripper/robotiq_description /ros2_ws/src/ && \
    rm -rf /tmp/robotiq_gripper

# ── Link bimanual_ur10e into ros2_ws ─────────────────────────────────────────
RUN ln -s /workspace/bimanual_ur10e/src/bimanual_ur10e \
          /ros2_ws/src/bimanual_ur10e

# ── Initialize rosdep & install deps for ros2_ws ─────────────────────────────
# apt-get update is required here: the lists were cleaned after the initial
# package install step, but rosdep may need to install additional packages.
RUN apt-get update && \
    rosdep init 2>/dev/null || true && \
    rosdep update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths /ros2_ws/src \
        --ignore-src --rosdistro=humble -r -y 2>&1 | tail -20 && \
    rm -rf /var/lib/apt/lists/*

# ── Build ros2_ws ─────────────────────────────────────────────────────────────
# Build only robotiq_description and bimanual_ur10e (both are install-only CMake,
# no C++ to compile, so this is fast).
# robotiq_driver is skipped here (C++ plugin) — not needed for our socket-based
# gripper control; it is available at runtime if needed.
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build \
        --packages-select robotiq_description bimanual_ur10e \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        2>&1 | tail -30

# ── Environment ──────────────────────────────────────────────────────────────
ENV PYTHONPATH=/workspace/gello_software:/workspace/teleop_interface/server:${PYTHONPATH}
ENV ROS_DOMAIN_ID=0

# ── Entrypoint & expose ───────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

EXPOSE 8080

ENTRYPOINT ["/entrypoint.sh"]
# Default command: start the web server
CMD ["server"]
