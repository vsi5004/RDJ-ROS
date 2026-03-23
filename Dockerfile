# RDJ Vinyl Robot — Docker image
# Base: ros:humble-ros-base (Ubuntu 22.04 Jammy, ROS 2 Humble)
# Target: Toradex Verdin iMX8M Plus (linux/arm64)
# Dev:    linux/amd64 with QEMU or native (see docs/DEPLOYMENT.md)
#
# Build (x86 dev, fast):
#   docker build -t vinyl-robot:dev .
#
# Build (ARM64 for Toradex, slow via QEMU buildx):
#   docker buildx build --platform linux/arm64 -t vinyl-robot:arm64 --load .
#
# Run mock simulation on laptop:
#   docker run -it --rm --network host vinyl-robot:dev \
#       ros2 launch vinyl_robot mock_bringup.launch.py

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
# Suppress rosdep warning about running as root (expected in Docker)
ENV ROSDEP_SUPPRESS_ROOT_WARNING=1

# ── System dependencies ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # CycloneDDS RMW (ros:humble-ros-base ships FastDDS only)
    ros-humble-rmw-cyclonedds-cpp \
    # CAN bus utilities
    iproute2 \
    can-utils \
    # Camera and computer vision
    ros-humble-usb-cam \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-opencv \
    # SICK LiDAR driver
    ros-humble-sick-scan-xd \
    # CANopen master + proxy drivers
    ros-humble-canopen-core \
    ros-humble-canopen-master-driver \
    ros-humble-canopen-proxy-driver \
    # Web interface (rosbridge)
    ros-humble-rosbridge-suite \
    # Diagnostics
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator \
    # Behavior tree (state machine)
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
    # Python build tools
    python3-colcon-common-extensions \
    python3-pip \
    python3-yaml \
    # Debug tools (remove in production — see docs/DEPLOYMENT.md)
    vim \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────────
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# ── Source code and default config ────────────────────────────────────────────
COPY src/ src/
COPY config/ config/

# ── ROS dependency resolution ─────────────────────────────────────────────────
RUN . /opt/ros/humble/setup.sh && \
    rosdep update --rosdistro humble && \
    rosdep install --from-paths src --ignore-src -r -y

# ── Bundle JS libraries (served statically — no CDN dependency at runtime) ────
RUN python3 -c "\
import urllib.request; \
urllib.request.urlretrieve(\
  'https://unpkg.com/roslib@1/build/roslib.min.js', \
  'src/web_interface/web/roslib.min.js'); \
urllib.request.urlretrieve(\
  'https://unpkg.com/three@0.128.0/build/three.min.js', \
  'src/web_interface/web/three.min.js'); \
urllib.request.urlretrieve(\
  'https://unpkg.com/three@0.128.0/examples/js/controls/OrbitControls.js', \
  'src/web_interface/web/OrbitControls.js')"

# ── Build ─────────────────────────────────────────────────────────────────────
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --symlink-install

# ── Auto-source workspace on interactive shells ────────────────────────────────
RUN echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

# ── Default command ───────────────────────────────────────────────────────────
# Override at runtime:
#   real hardware:  ros2 launch vinyl_robot bringup.launch.py
#   simulation:     ros2 launch vinyl_robot mock_bringup.launch.py
CMD ["/bin/bash", "-c", \
     "source /ros_ws/install/setup.bash && \
      ros2 launch vinyl_robot mock_bringup.launch.py"]
