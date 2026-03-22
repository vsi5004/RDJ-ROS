# Vinyl Robot — ROS 2 Control Software

ROS 2 Humble control system for a CANopen-based vinyl record handling robot, deployed on Toradex Verdin iMX8M Plus via Torizon OS Docker containers.

## Documentation

- [System Architecture](docs/SYSTEM_ARCHITECTURE.md) — ROS 2 node graph, action servers, configuration
- [Motion Coordinator & Homing](docs/MOTION.md) — Stack-aware homing sequence, safety integration, coordinate systems
- [Behavior Tree](docs/BEHAVIOR_TREE.md) — BehaviorTree.CPP operational workflow, error handling
- [HMI & Record Identification](docs/HMI_AND_IDENTIFICATION.md) — Web interface, voice control, Discogs/audio fingerprint ID
- [Deployment](docs/DEPLOYMENT.md) — Torizon/Docker setup, Windows-only dev workflow, workspace structure

## Quick Start

```bash
# On the Toradex (via SSH)
docker-compose up --build

# Or for mock simulation (no hardware, works on Windows laptop)
docker build -t vinyl-robot:x86-dev .
docker run -it --rm --network host vinyl-robot:x86-dev \
    ros2 launch vinyl_robot mock_bringup.launch.py
```

## Architecture

9 ROS 2 nodes: CANopen master, motion coordinator, LiDAR safety, turntable monitor, behavior tree state machine, LED controller, diagnostics, web interface (rosbridge), and record identifier. All operational intelligence on the SBC — MCU nodes are dumb I/O. All HMI paths (web UI, display, voice) publish to the same ROS 2 topics — robot logic is decoupled from input source.

## Platform

- **SBC**: Toradex Verdin iMX8M Plus (4GB) on Mallow carrier
- **OS**: Torizon OS + Docker
- **ROS 2**: Humble on Ubuntu 22.04
- **CAN**: Native FlexCAN, socketcan, 1Mbit, 5 nodes
- **LiDAR**: SICK TIM571 (Ethernet)
- **Camera**: USB/CSI for tonearm tracking + label recognition
- **Web UI**: rosbridge_websocket, phone-accessible

## Development

Two workflows supported:

- **With SBC**: Windows 11 + VS Code + Torizon IDE extension → SSH to Toradex
- **Without SBC**: Docker Desktop on Windows laptop — full stack runs on x86 with ros2_control FakeSystem, no hardware needed

## Related

Firmware and PCB design live in a separate repository. The CANopen object dictionary (EDS files in `config/`) is the interface contract between them.
