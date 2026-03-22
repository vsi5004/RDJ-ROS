# Deployment and Development Workflow

## Target Platform

- **SoM**: Toradex Verdin iMX8M Plus 4GB WB IT
- **Carrier**: Mallow carrier board
- **OS**: Torizon OS (latest stable)
- **Container runtime**: Docker (bundled with Torizon OS)

## Base Docker Image

```dockerfile
FROM ros:humble-ros-base
```

The `ros:humble-ros-base` image provides:
- ROS 2 Humble on Ubuntu 22.04 (Jammy)
- Core ROS 2 libraries (rclcpp, rclpy, launch, message types)
- Build tools (colcon, rosdep, vcstools)
- Multi-arch: arm64v8 pulled automatically on the Toradex

It does NOT include GUI tools (rviz2, rqt) — these are unnecessary on a headless robot.

## Full Dockerfile

```dockerfile
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # CAN bus
    iproute2 \
    can-utils \
    # Camera and CV
    ros-humble-usb-cam \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-opencv \
    # SICK LiDAR
    ros-humble-sick-scan-xd \
    # CANopen
    ros-humble-canopen-core \
    ros-humble-canopen-master-driver \
    ros-humble-canopen-proxy-driver \
    ros-humble-canopen-402-driver \
    # Behavior tree
    ros-humble-behaviortree-cpp \
    # ros2_control (FakeSystem for simulation, real hardware interface for production)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # Web interface
    ros-humble-rosbridge-suite \
    # Diagnostics
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator \
    # Build tools
    python3-colcon-common-extensions \
    python3-pip \
    # Debug (remove in production)
    vim \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# Source code
COPY src/ src/

# Dependencies
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Auto-source workspace
RUN echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

CMD ["ros2", "launch", "vinyl_robot", "bringup.launch.py"]
```

## Docker Compose

```yaml
version: "3.8"

services:
  can-setup:
    image: torizon/debian:4
    network_mode: host
    cap_add:
      - NET_ADMIN
    command: >
      sh -c "
        ip link set can0 type can bitrate 1000000 &&
        ip link set can0 up &&
        echo 'CAN0 up at 1Mbit' &&
        sleep infinity
      "
    restart: unless-stopped

  vinyl-robot:
    build: .
    network_mode: host
    cap_add:
      - NET_ADMIN
    devices:
      - /dev/video0:/dev/video0
    volumes:
      - ./config:/ros_ws/config:ro
      - ./logs:/root/.ros/log
    depends_on:
      - can-setup
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    restart: unless-stopped
```

### Key Docker flags

- `network_mode: host` — container shares host network namespace, sees `can0` socketcan interface directly
- `cap_add: NET_ADMIN` — required for CAN interface configuration (can be removed if CAN is configured via systemd on host)
- `devices: /dev/video0` — camera access
- `volumes: ./config` — YAML parameters mounted read-only, editable without rebuild

## CAN Setup on Torizon Host

Preferred production approach — systemd service ensures CAN is up at boot:

```ini
# /etc/systemd/system/can0.service
[Unit]
Description=CAN0 interface at 1Mbit
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/sbin/ip link set can0 type can bitrate 1000000
ExecStart=/sbin/ip link set can0 up
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```

Enable: `systemctl enable can0.service`

With this in place, the Docker container only needs `--network host` (no `NET_ADMIN` needed since CAN is already configured).

### Verify CAN transceiver on Mallow

The Mallow carrier's onboard CAN transceivers should support CAN-FD (most do — typically TCAN330 or similar). The node boards use TCAN332 (CAN-FD capable), and the iMX8M Plus FlexCAN peripheral supports FD. The system currently runs classic CAN at 1Mbit, but the entire chain is CAN-FD ready if higher throughput is needed later. Check the Mallow datasheet to confirm the transceiver model and maximum bitrate.

## Development Workflow

### Prerequisites

- Windows 11 laptop with VS Code
- Torizon IDE VS Code extension installed
- Toradex Verdin iMX8M Plus flashed with Torizon OS
- Both devices on the same network

### Daily Development Loop

1. **Edit** code in VS Code on Windows laptop
2. **Sync** — Torizon IDE extension pushes files to SoM over SSH
3. **Build** — `colcon build --packages-select <changed_package>` runs inside container on SoM (incremental, few seconds)
4. **Run** — `ros2 launch vinyl_robot bringup.launch.py` in container
5. **Debug** — SSH into Toradex, `docker exec` into container, use `ros2 topic echo`, `ros2 node list`, etc.

### Mock Simulation (no hardware)

Swap launch file to start mock nodes instead of real drivers:

```bash
ros2 launch vinyl_robot mock_bringup.launch.py
```

Mock nodes use `vcan0` (virtual CAN) which can be created inside the container:

```bash
ip link add dev vcan0 type vcan
ip link set up vcan0
```

The mock `canopen_master` replacement publishes fake TPDO data. When the motion coordinator writes a target, the mock ramps XACTUAL toward it over time and sets in-position. This tests the entire ROS 2 graph without hardware.

### Windows-Only Development (No SBC Hardware)

The full ROS 2 stack can be developed and tested on a Windows laptop using Docker with QEMU emulation. No Toradex board or any physical hardware required.

**Prerequisites:**
- Docker Desktop for Windows (with WSL2 backend)
- QEMU user-space emulation registered

```bash
# Register QEMU static binaries for ARM64 emulation (run once)
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

**Building the ARM64 container on x86:**

```bash
# Create a buildx builder
docker buildx create --name vinyl-builder --use
docker buildx inspect --bootstrap

# Build for ARM64 (cross-compile on x86 via QEMU)
docker buildx build --platform linux/arm64 -t vinyl-robot:arm64 --load .

# Or build natively for x86 during development (faster, no QEMU)
docker build -t vinyl-robot:x86-dev .
```

For day-to-day development, build the **x86 native** image — it's much faster since there's no QEMU overhead. The ROS 2 code is architecture-independent. Build the ARM64 image only when deploying to the Toradex.

**Running the full stack locally on x86:**

```bash
# Start with mock hardware (no CAN, no camera, no LiDAR)
docker run -it --rm --network host vinyl-robot:x86-dev \
    ros2 launch vinyl_robot mock_bringup.launch.py
```

This launches all nodes with mock substitutes for hardware-dependent drivers. The behavior tree, motion coordinator, homing logic, safety system, and web UI all run exactly as they would on the real robot.

**ros2_control FakeSystem for actuator simulation:**

Instead of custom mock nodes, `ros2_control` provides a `FakeSystem` (also called `mock_hardware`) plugin that simulates joints. The controller code is identical between simulation and real hardware — only the YAML config differs:

```yaml
# config/mock_ros2_control.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

vinyl_robot:
  ros__parameters:
    type: fake_components/GenericSystem
    fake_sensor_commands: true
    joints:
      - x_axis
      - z_axis
      - a_axis
    command_interfaces:
      - position
      - velocity
    state_interfaces:
      - position
      - velocity
```

With `FakeSystem`, commanding a joint position immediately reports it as achieved (optionally with configurable dynamics). This lets you test the full controller stack — joint trajectory actions, velocity scaling, position feedback — without any hardware or even mock node code.

**Switching between simulated and real hardware:**

The only difference is the launch file and YAML config:

```bash
# Simulated (laptop, no hardware)
ros2 launch vinyl_robot mock_bringup.launch.py

# Real hardware (on Toradex)
ros2 launch vinyl_robot bringup.launch.py
```

Both launch the same nodes with the same topics. The behavior tree, web UI, and all application logic are identical in both environments.

**Web UI development on laptop:**

The `rosbridge_websocket` node runs in the simulated stack too. Open a browser on your laptop pointing at `http://localhost:9090` and the web UI works against the simulated robot. You can develop the full frontend experience without hardware.

### Useful ROS 2 CLI Commands

```bash
# See the node graph
ros2 node list

# Watch axis status
ros2 topic echo /canopen/node_1/tpdo1

# Check motion state
ros2 topic echo /motion/status

# Monitor safety
ros2 topic echo /safety/velocity_scale

# Trigger homing manually
ros2 action send_goal /motion/home_all vinyl_robot_msgs/action/HomeAll {}

# Record all topics for replay
ros2 bag record -a -o test_session

# Play back recorded session
ros2 bag play test_session
```

## ROS 2 Workspace Structure

```
ros-torizon-repo/
├── docs/
│   ├── SYSTEM_ARCHITECTURE.md
│   ├── MOTION.md
│   ├── BEHAVIOR_TREE.md
│   ├── DEPLOYMENT.md
│   └── HMI_AND_IDENTIFICATION.md
├── config/
│   ├── robot_params.yaml
│   ├── bus.yml
│   ├── mock_ros2_control.yaml     # FakeSystem config for simulation
│   ├── stepper_node.eds
│   ├── a_axis_node.eds
│   └── servo_node.eds
├── src/
│   ├── vinyl_robot/              # Main package — launch files
│   │   ├── launch/
│   │   │   ├── bringup.launch.py
│   │   │   └── mock_bringup.launch.py
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── vinyl_robot_msgs/         # Custom message/action definitions
│   │   ├── action/
│   │   │   ├── MoveToPosition.action
│   │   │   ├── HomeAll.action
│   │   │   ├── Grip.action
│   │   │   └── ...
│   │   ├── msg/
│   │   │   ├── MotionStatus.msg
│   │   │   └── AlbumMetadata.msg
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── motion_coordinator/       # Motion planning and coordination
│   ├── lidar_safety/             # Safety zone monitoring
│   ├── turntable_monitor/        # Camera CV tonearm tracking
│   ├── state_machine/            # BehaviorTree.CPP orchestrator
│   ├── led_controller/           # DotStar pattern generation
│   ├── diagnostics_aggregator/   # Health monitoring
│   ├── web_interface/            # rosbridge + static web UI files
│   │   ├── launch/
│   │   │   └── web_ui.launch.py
│   │   ├── web/                  # Frontend (HTML/JS/CSS)
│   │   │   ├── index.html
│   │   │   └── app.js
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── record_identifier/        # Visual + audio record identification
│   │   ├── record_identifier/
│   │   │   ├── visual_identifier.py   # Camera OCR → Discogs lookup
│   │   │   └── audio_identifier.py    # Audio fingerprint → AudD/ACRCloud
│   │   ├── package.xml
│   │   └── setup.py
│   └── mock_nodes/               # Simulation stubs for testing
├── Dockerfile
├── docker-compose.yml
└── README.md
```

## Firmware Flashing (EE repo)

The STM32 firmware is built and flashed separately from the ROS workspace. Firmware lives in the EE repo. Flashing is done via SWD (Tag-Connect TC2030) using an ST-Link programmer, not from the Toradex.

The ROS workspace does not contain or build firmware. The two repos are independent. The CANopen object dictionary (EDS files) is the contract between them.

## Production Hardening (Future)

- Remove debug packages (vim, net-tools) from Dockerfile
- Set container restart policy to `always`
- Enable Torizon OTA for remote updates
- Add healthcheck to docker-compose for automatic container restart on crash
- Pin all apt package versions for reproducible builds
- Use multi-stage Docker build to reduce image size
