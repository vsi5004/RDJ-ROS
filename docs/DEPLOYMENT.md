# Deployment and Development Workflow

## Docker Image

**Base image**: `ros:humble-ros-base` (ROS 2 Humble on Ubuntu 22.04, multi-arch)

**Dockerfile** installs:
- CycloneDDS RMW (`ros-humble-rmw-cyclonedds-cpp`) — base image ships FastDDS only
- CAN tools (`iproute2`, `can-utils`)
- Camera/CV (`ros-humble-usb-cam`, `ros-humble-cv-bridge`, `python3-opencv`)
- SICK LiDAR (`ros-humble-sick-scan-xd`)
- CANopen (`ros-humble-canopen-core`, `-master-driver`, `-proxy-driver`)
- rosbridge (`ros-humble-rosbridge-suite`)
- Diagnostics packages
- Build tools (`colcon`, `rosdep`, `python3-yaml`)
- Debug tools (`vim`, `iputils-ping`, `net-tools`) — remove in production
- roslibjs bundled: `roslib.min.js` downloaded from unpkg at build time, served alongside web UI

**Build sequence**:
1. `COPY src/ src/` and `COPY config/ config/`
2. `rosdep install` resolves Python/ROS dependencies
3. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install`
4. Config installed into `vinyl_robot` share directory during build (`install(DIRECTORY ../../config/ ...)` in CMakeLists.txt)

**Why `ROSDEP_SUPPRESS_ROOT_WARNING=1`**: rosdep warns when run as root because on a multi-user system it could corrupt shared files. In a single-purpose Docker container this concern doesn't apply; the env var is the correct mechanism for CI/Docker use.

## Docker Compose

```bash
# Simulation (laptop, no hardware):
docker compose up

# Real hardware (Toradex, after CAN is up):
docker compose --profile hardware up
```

### Simulation profile (default)
- Bridge networking with explicit port mappings — required for Docker Desktop on Windows/Mac where `network_mode: host` does not expose ports to the host OS
- `8080:8080` — web UI
- `9090:9090` — rosbridge WebSocket
- All ROS nodes share one container so CycloneDDS discovery works without host networking
- Config volume: `./config:/ros_ws/install/vinyl_robot/share/vinyl_robot/config:ro` — overlays the baked-in defaults, editable without rebuild

### Hardware profile
- `vinyl-robot` service: `network_mode: host`, `cap_add: NET_ADMIN`, `/dev/video0` device passthrough
- `can-setup` service: configures `can0` at 1 Mbit/s (can be removed if systemd handles CAN at boot)
- Depends on `can-setup` completing before starting

### Environment
Both profiles set `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## Development Workflows

### Simulation on Windows laptop (no hardware)

```bash
# Build and run full mock stack:
docker compose build
docker compose up

# Web UI:        http://localhost:8080
# rosbridge WS:  ws://localhost:9090

# Trigger homing manually from exec terminal:
docker exec -it <container> bash -c "source /ros_ws/install/setup.bash && ros2 action send_goal /motion/home_all vinyl_robot_msgs/action/HomeAll {}"
```

To exec into a running container via Docker Desktop: use the Exec tab or run `docker exec -it <container> bash`. Note that Docker Desktop's terminal is `sh` not `bash` — prefix commands with `bash -c "..."` or use `bash` explicitly.

### Incremental rebuild (faster during development)

The config volume mount means config changes take effect immediately. For code changes to Python nodes, rebuild the image. With `--symlink-install`, Python source is symlinked from `src/` into `build/` so changes to source files in `src/` take effect inside the container without rebuild — but only if the container was started with the `src/` directory also mounted (not the default production setup).

### Cross-compile for Toradex ARM64

```bash
# Register QEMU (once per machine):
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Build ARM64 image:
docker buildx build --platform linux/arm64 -t vinyl-robot:arm64 --load .

# Or using compose:
docker compose build  # add platform: linux/arm64 to compose for ARM target
```

### Deploying to Toradex via Torizon IDE

1. Connect VS Code + Torizon IDE extension to the SOM
2. Transfer the ARM64 image or build directly on the board
3. `docker compose --profile hardware up`

## CAN Interface (Hardware)

Preferred production setup — systemd service on the Torizon host configures CAN at boot, eliminating the need for the `can-setup` container:

```
# /etc/systemd/system/can0.service
[Unit]
Description=CAN bus configuration
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 type can bitrate 1000000 restart-ms 100
ExecStart=/sbin/ip link set can0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

If using `can0.service`, remove the `can-setup` service from `docker-compose.yml` and drop `cap_add: NET_ADMIN` from the `vinyl-robot` service.

## Workspace Structure

```
RDJ-ROS/
├── config/               ← robot_params.yaml, bus.yml, mock_ros2_control.yaml, 3× EDS files
├── docs/                 ← architecture, motion, behavior tree, HMI, deployment docs
├── src/
│   ├── vinyl_robot_msgs/ ← custom msgs/actions (CMake)
│   ├── vinyl_robot/      ← launch files only (CMake)
│   ├── motion_coordinator/  ← core node: homing, actions, jog, servo raw, estop
│   ├── mock_nodes/          ← 50 Hz CAN simulation
│   ├── lidar_safety/        ← safety zones (mock_mode param)
│   ├── turntable_monitor/   ← tonearm tracking (mock_mode param)
│   ├── state_machine/       ← orchestrator stub (HomeAll wired)
│   ├── led_controller/      ← stub
│   ├── diagnostics_aggregator/ ← DiagnosticArray publisher
│   └── web_interface/       ← rosbridge + Python HTTP + web UI
├── Dockerfile
└── docker-compose.yml
```

## Production Hardening (Future)

- Remove debug packages (`vim`, `iputils-ping`, `net-tools`) from Dockerfile
- Set `restart: always` (currently `unless-stopped`)
- Multi-stage Docker build to reduce image size
- Enable Torizon OTA for over-the-air updates
- Add Docker healthcheck based on `/motion/status` publish rate
- Pin all package versions for reproducible builds
