# CLAUDE.md — Project Context for AI Assistant

## What This Project Is

ROS 2 Humble control software for a CANopen vinyl record handling robot, deployed on a Toradex Verdin iMX8M Plus via Torizon OS Docker containers. This is where ALL operational intelligence lives — the MCU firmware (separate repo) is just dumb I/O.

## Key Design Documents

Read these before making changes:

- `docs/SYSTEM_ARCHITECTURE.md` — ROS 2 node graph (9 nodes), action servers, YAML config, CANopen master
- `docs/MOTION.md` — Stack-aware homing sequence (critical safety logic), coordinate systems, safety integration
- `docs/BEHAVIOR_TREE.md` — BehaviorTree.CPP workflow, blackboard state, error recovery
- `docs/HMI_AND_IDENTIFICATION.md` — Web interface (rosbridge), record ID (Discogs/audio fingerprint), voice control
- `docs/DEPLOYMENT.md` — Dockerfile, docker-compose, CAN setup, Windows-only dev workflow, workspace structure

## Critical Design Decisions

1. **All intelligence on the SBC.** MCUs are dumb I/O nodes. The motion coordinator converts mm/degrees to microsteps. The behavior tree sequences all operations. MCU firmware just executes CAN commands.
2. **Stack-aware homing.** The record queue is a shelved structure with floors between slots. When the arm is inside a slot, Z movement is DANGEROUS — shelves above and below. The homing sequence must detect this via X position + A angle and extract the arm horizontally first. See MOTION.md Phase 0 / IN_STACK regime.
3. **SYNC-gated multi-axis coordination.** Motion coordinator writes RPDOs for all axes, then SYNC fires — all axes start simultaneously (~130μs of each other).
4. **Safety is reactive, not procedural.** The ReactiveSequence root re-checks SafetyCheck every tick. Safety interrupt can preempt any running action. The motion coordinator multiplies all velocities by the safety scaling factor before sending.
5. **TMC5160 microstep counter is position authority.** Not the pot. The pot is only for homing direction and sanity checks.
6. **Config-driven.** All mechanical geometry, safety zones, and homing parameters live in YAML. No hardcoded magic numbers in code.
7. **Docker on Torizon.** The entire ROS 2 stack runs in a single container based on `ros:humble-ros-base`. CAN interface configured on host via systemd.
8. **HMI decoupled from robot logic.** All input paths (web UI, onboard display, voice) publish to the same `/user/*` ROS 2 topics. The behavior tree doesn't know or care where a command came from. rosbridge_websocket bridges the web frontend.
9. **Develop entirely on Windows.** Docker + QEMU buildx for cross-architecture builds. ros2_control FakeSystem for actuator simulation. Full stack runs on x86 laptop without any hardware.

## Tech Stack

- ROS 2 Humble (Ubuntu 22.04 Docker container)
- CycloneDDS middleware
- ros2_canopen (Fraunhofer IPA) — CANopen master
- ros2_control + FakeSystem — hardware abstraction and simulation
- sick_scan_xd — SICK TIM571 LiDAR driver
- BehaviorTree.CPP — state machine
- rosbridge_suite — WebSocket bridge for web UI
- OpenCV (via cv_bridge) — tonearm tracking and label recognition
- Discogs API / AudD / ACRCloud — record identification
- Torizon OS + Docker on Toradex Verdin iMX8M Plus

## Hardware Interface

- **CAN bus**: socketcan `can0` at 1Mbit, native FlexCAN on iMX8M Plus
- **LiDAR**: SICK TIM571 on Ethernet (TCP/IP, `sick_scan_xd` driver)
- **Camera**: USB or CSI camera at `/dev/video0`
- **5 CANopen nodes**: X (0x01), Z (0x02), A (0x03), Pincher (0x04), Player (0x05)

## What NOT to Do

- Don't put motor control logic in the MCU firmware — it stays here
- Don't change OD indices without updating firmware EDS files AND the bus.yml
- Don't assume Z-up is always safe — it's not when inside the record stack
- Don't use blocking I/O in the motion coordinator — all CAN interactions are async via ros2_canopen topics/services
- Don't hardcode positions — use the YAML config
- Don't skip the safety velocity scale check before sending any RPDO
- Don't use rviz2/rqt in the production container — this is headless

## Development

- **With SBC**: Windows 11 + VS Code + Torizon IDE extension → SSH to Toradex
- **Without SBC**: Docker Desktop + QEMU on Windows laptop — full stack runs on x86
- Mock simulation via `mock_bringup.launch.py` with ros2_control FakeSystem
- `vcan0` virtual CAN for simulated bus
- Web UI development works against simulated robot (rosbridge on localhost)
- Docker buildx for ARM64 cross-compilation when deploying to Toradex
