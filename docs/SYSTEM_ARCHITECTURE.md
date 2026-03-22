# ROS 2 System Architecture

## Platform

- **SBC**: Toradex Verdin iMX8M Plus (4GB) on Mallow carrier
- **OS**: Torizon OS with Docker containers
- **ROS 2**: Humble (Ubuntu 22.04 Jammy base) — or Jazzy if `ros-jazzy-sick-scan-xd` is available as binary
- **DDS**: CycloneDDS (lower overhead than FastDDS for single-machine)
- **CAN interface**: Native FlexCAN on iMX8M Plus → socketcan `can0` at 1Mbit

## Node Graph

Seven core ROS 2 nodes plus optional HMI and identification nodes:

### 1. canopen_master (from ros2_canopen package)

Bridge between ROS 2 and the CAN bus. Manages CANopen stack: NMT lifecycle, SYNC generation at 50Hz, SDO client, PDO mapping. Configured via `bus.yml` and EDS files.

**Publishes**: `/canopen/node_N/tpdo1`, `/canopen/node_N/nmt_state`, `/canopen/node_N/emcy`
**Subscribes**: `/canopen/node_N/rpdo1`

### 2. motion_coordinator

Translates high-level move commands into coordinated TMC5160 register writes. Knows mechanical geometry (mm/deg to microsteps conversion). Orchestrates homing sequence. Enforces safety velocity scaling. Hosts action servers for the behavior tree.

**Subscribes**: `/canopen/node_N/tpdo1`, `/safety/velocity_scale`, `/safety/estop`
**Publishes**: `/motion/status`, `/canopen/node_N/rpdo1`
**Action servers**: `/motion/move_to_position`, `/motion/home_all`, `/motion/grip`, `/motion/flip_record`, `/motion/press_play`, `/motion/set_speed`

### 3. lidar_safety

Subscribes to SICK TIM571 laser scan, defines dynamic safety zones that rotate with the A axis angle. Publishes velocity scaling factor and emergency stop signal.

**Subscribes**: `/scan`, `/canopen/node_3/tpdo1` (A axis angle from pot)
**Publishes**: `/safety/velocity_scale` (Float32: 1.0=full, 0.25=slow, 0.0=halt), `/safety/estop` (Bool)

Safety zones:
- Warning zone (400mm radius): scale to 25%
- Stop zone (200mm radius): halt all motion
- Exclusion angles: static environment baseline, only trigger on new objects
- Zones rotate with A axis angle — knows arm direction from TPDO data

### 4. turntable_monitor

Processes camera feed to track tonearm position. Simple CV pipeline: crop → HSV threshold → find contour → compute centroid → map to radial progress.

**Subscribes**: `/camera/image_raw` (sensor_msgs/Image)
**Publishes**: `/turntable/progress` (Float32: 0.0=outer groove, 1.0=run-out)

Update rate: 1–2Hz is sufficient. The robot needs ~10 seconds warning before run-out, not frame-accurate tracking.

### 5. state_machine (BehaviorTree.CPP)

Top-level orchestrator. Commands the full record-handling workflow. Uses reactive sequences so safety checks can interrupt any operation. Also listens to user commands from the web UI.

**Subscribes**: `/motion/status`, `/turntable/progress`, `/safety/velocity_scale`, `/safety/estop`, `/user/command`, `/user/play_mode`, `/user/select_record`
**Calls**: Action servers on motion_coordinator

### 6. led_controller

Maps axis positions and targets to DotStar LED patterns. Writes to A axis and Player nodes via canopen_master.

**Subscribes**: `/motion/status`, `/canopen` TPDOs

### 7. diagnostics

Aggregates health data from all nodes. Publishes standard `diagnostic_msgs/DiagnosticArray`.

### 8. web_interface (rosbridge_websocket + HTTP server)

Bridges ROS 2 topics to WebSocket connections for the phone-accessible web UI. Serves static HTML/JS/CSS files. All HMI paths (web, display, voice) publish to the same ROS 2 topics — robot logic is fully decoupled from input source.

**Provides**: WebSocket bridge at `ws://robot-ip:9090`, HTTP server for frontend files
**Bridges**: All `/motion/*`, `/turntable/*`, `/safety/*`, `/record/*`, `/user/*` topics

See [HMI_AND_IDENTIFICATION.md](./HMI_AND_IDENTIFICATION.md) for full details.

### 9. record_identifier (optional)

Identifies records using visual label recognition (camera → OCR → Discogs API) and/or audio fingerprinting (audio capture → AudD/ACRCloud). Publishes album metadata for display in the web UI.

**Subscribes**: `/camera/image_raw`
**Publishes**: `/record/metadata` (AlbumMetadata)

See [HMI_AND_IDENTIFICATION.md](./HMI_AND_IDENTIFICATION.md) for full details.

## Action Server Definitions

### /motion/move_to_position

```
# Goal
float64 x_mm
float64 z_mm
float64 a_deg
float64 velocity_scale   # 0.0-1.0, multiplied against configured max
---
# Result
bool success
string error_msg
---
# Feedback
float64 x_actual_mm
float64 z_actual_mm
float64 a_actual_deg
bool x_in_position
bool z_in_position
bool a_in_position
```

The coordinator converts mm/degrees to microsteps, applies safety velocity scaling, writes RPDOs for all axes, and publishes feedback at 50Hz until all report in-position.

Cancellation: writes current XACTUAL as new XTARGET to each axis → immediate decel stop.

### /motion/home_all

```
# Goal
(empty)
---
# Result
bool success
string error_msg
---
# Feedback
string current_phase      # "assessing", "z_clearing", "x_to_safe", "homing_a", etc.
uint8 axes_homed          # count of completed axes
```

See [MOTION.md](./MOTION.md) for the full homing sequence with stack-awareness.

### /motion/grip

```
# Goal
bool close          # true=close, false=open
---
# Result
bool success
float32 tof_mm      # ToF reading after grip (confirmation)
```

### /motion/flip_record, /motion/press_play, /motion/set_speed

Simple servo commands with completion confirmation.

## Configuration

All mechanical geometry, safety zones, and operational parameters live in YAML, loaded at launch:

```yaml
# config/robot_params.yaml
geometry:
  x_steps_per_mm: 80.0       # microsteps per mm
  z_steps_per_mm: 400.0      # lead screw pitch dependent
  a_steps_per_deg: 142.2     # belt/pulley ratio dependent
  
positions:
  turntable:
    x_mm: 950.0
    z_platter_mm: 45.0
  queue_stack:
    x_mm: 50.0
    slot_z_mm: [20, 60, 100, 140, 180]  # Z heights for each slot
  safe_park:
    x_mm: 500.0
    z_mm: 120.0
    a_deg: 180.0

homing:
  safe_zone:
    x_min_mm: 300
    x_max_mm: 700
    z_clear_mm: 120
    a_park_deg: 180
  turntable:
    x_center_mm: 950
    x_danger_mm: 800
    platter_height_mm: 45
  record_stack:
    x_center_mm: 50
    x_danger_mm: 200
    x_clear_mm: 250
    a_center_deg: 0
    a_tolerance_deg: 15
    shelf_count: 5
    shelf_pitch_mm: 40
    shelf_clearance_mm: 20
  velocities:
    z_initial_up: 10
    x_to_safe_zone: 20
    a_emergency_park: 5
    a_normal_home: 30
    x_home: 40
    z_home: 20
    stack_retract: 5

safety:
  lidar:
    warning_radius_mm: 400
    stop_radius_mm: 200
  a_axis:
    min_safe_deg: 30
    max_safe_deg: 330
  stallguard:
    homing_threshold: 10    # very sensitive
    operating_threshold: 4  # less sensitive (carrying record)
```

## CANopen Master Configuration

```yaml
# config/bus.yml
bus:
  interface: can0
  bitrate: 1000000
  sync_period: 20000  # microseconds = 50Hz

nodes:
  x_axis:
    node_id: 1
    eds_file: stepper_node.eds
  z_axis:
    node_id: 2
    eds_file: stepper_node.eds
  a_axis:
    node_id: 3
    eds_file: a_axis_node.eds
  pincher:
    node_id: 4
    eds_file: servo_node.eds
  player:
    node_id: 5
    eds_file: servo_node.eds
```

## Launch Sequence

Nodes start in dependency order via ROS 2 launch file:

1. `canopen_master` — waits for CAN interface, discovers nodes
2. `sick_scan_xd` — connects to TIM571 over Ethernet
3. `usb_cam` — starts camera
4. `lidar_safety` — waits for first `/scan` message
5. `motion_coordinator` — subscribes to safety topics, issues home_all
6. `turntable_monitor` — starts when camera is publishing
7. `led_controller` — starts anytime
8. `state_machine` — starts last, waits for all axes homed

## Simulation / Mock Testing

### Level 1 — Mock CAN nodes (no hardware)

A single Python node pretends to be `canopen_master`. When motion_coordinator writes a target, the mock ramps XACTUAL toward it using linear interpolation, then sets in-position status. ~50 lines per mock axis.

Sufficient to develop and test: behavior tree, motion coordinator, homing sequence, safety zones, LED controller, all error handling paths.

### Level 2 — Gazebo (3D physics)

URDF model of the robot. Visual collision checking, realistic timing, simulated LiDAR. 2–3 days setup investment.

### Level 3 — Hardware-in-the-loop

Real Nucleo boards on CAN bus, no motors attached. Tests actual CAN communication and timing.
