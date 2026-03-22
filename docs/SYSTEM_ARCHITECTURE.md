# ROS 2 System Architecture

The ROS 2 system runs on a Toradex Verdin iMX8M Plus with Torizon OS and Docker. It uses CycloneDDS middleware, native FlexCAN at 1 Mbit for the CAN bus, and coordinates 5 CANopen nodes (X, Z, A axes plus Pincher and Player servos).

## Implementation Status

| Package | Status | Notes |
|---|---|---|
| `vinyl_robot_msgs` | ✅ Complete | 6 actions, 2 messages |
| `motion_coordinator` | ✅ Complete | Full homing, action servers, jog, servo raw, hw+sw estop |
| `mock_nodes` | ✅ Complete | 50 Hz physics simulation of all 5 CAN nodes |
| `lidar_safety` | ✅ Functional stub | mock_mode only; real LiDAR CV pipeline future work |
| `turntable_monitor` | ✅ Functional stub | mock_mode only; real camera CV pipeline future work |
| `web_interface` | ✅ Complete | rosbridge + dark UI with jog panel, servo calibration, estop |
| `state_machine` | 🔄 Partial stub | HomeAll wired up; BehaviorTree.CPP implementation pending |
| `led_controller` | 🔄 Stub | Logging only |
| `diagnostics_aggregator` | ✅ Functional | Publishes DiagnosticArray at 1 Hz |
| `record_identifier` | ❌ Not started | Optional future package |

## Node Graph

1. **mock_canopen_master** *(sim only)* — Simulates all 5 CAN nodes at 50 Hz with ramp physics. Publishes `/canopen/<name>/tpdo1`, subscribes `/canopen/<name>/rpdo1`. Replaced at hardware time by `canopen_master_driver` + `canopen_proxy_driver`.

2. **motion_coordinator** — Core intelligence node. Converts mm/deg to microsteps, enforces safety scaling, orchestrates homing, hosts action servers, handles jog and raw servo commands.

3. **lidar_safety** — SICK TIM571 laser scan → dynamic safety zones (warning 400 mm, stop 200 mm). Publishes `/safety/velocity_scale` (0.0–1.0) and `/safety/estop`. In mock_mode always publishes scale=1.0, estop=false.

4. **turntable_monitor** — Camera CV pipeline tracks tonearm radial position. Publishes `/turntable/progress` (0.0–1.0). In mock_mode holds at 0.0.

5. **state_machine** — Orchestrator. Currently routes `/user/command: home` to the HomeAll action server. Full BehaviorTree.CPP implementation pending (see BEHAVIOR_TREE.md).

6. **led_controller** — Stub. Will map robot state to DotStar LED patterns.

7. **diagnostics_aggregator** — Aggregates motion and safety health, publishes `DiagnosticArray`.

8. **web_interface** — rosbridge_websocket on port 9090 + Python HTTP server on port 8080. Full operator panel: axis status, ToF sensors, manual jog, servo calibration sliders, e-stop.

9. **record_identifier** *(future)* — Visual label recognition + audio fingerprinting.

## Topics Reference

### CAN Interface (mock or real hardware bridge)
| Topic | Type | Direction |
|---|---|---|
| `/canopen/<name>/tpdo1` | `std_msgs/UInt8MultiArray` | Node → motion_coordinator |
| `/canopen/<name>/rpdo1` | `std_msgs/UInt8MultiArray` | motion_coordinator → Node |

Node names: `x_axis`, `z_axis`, `a_axis`, `pincher`, `player`

### Safety
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/safety/velocity_scale` | `std_msgs/Float32` | lidar_safety | motion_coordinator |
| `/safety/estop` | `std_msgs/Bool` | lidar_safety | motion_coordinator, web_interface |
| `/user/estop` | `std_msgs/Bool` | web_interface | motion_coordinator |

**Hardware vs software estop**: `/safety/estop` is sensor-driven (LiDAR). `/user/estop` is operator-driven (web UI button or `Escape` key). motion_coordinator tracks them independently — clearing a software estop does not override an active hardware estop.

### Motion
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/motion/status` | `vinyl_robot_msgs/MotionStatus` | motion_coordinator | state_machine, web_interface, diagnostics |
| `/motion/jog` | `std_msgs/String` | web_interface | motion_coordinator |
| `/motion/servo_raw` | `std_msgs/String` | web_interface | motion_coordinator |

`/motion/jog` format: `"<axis> <delta>"` — e.g. `"x 10.0"`, `"z -5.0"`, `"a 90.0"`

`/motion/servo_raw` format: `"<node> <s1_us> <s2_us>"` — e.g. `"pincher 1500 1200"`. Clamps to 500–2500 µs. Used for servo endpoint calibration; bypasses action abstraction.

### User Commands
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/user/command` | `std_msgs/String` | web_interface | state_machine |
| `/user/play_mode` | `std_msgs/String` | web_interface | state_machine |
| `/user/select_record` | `std_msgs/UInt8` | web_interface | state_machine |
| `/user/estop` | `std_msgs/Bool` | web_interface | motion_coordinator |

### Sensors
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/turntable/progress` | `std_msgs/Float32` | turntable_monitor | state_machine, web_interface |
| `/scan` | `sensor_msgs/LaserScan` | sick_scan_xd | lidar_safety |
| `/camera/image_raw` | `sensor_msgs/Image` | usb_cam | turntable_monitor |

## Action Servers (on motion_coordinator)

| Server | Action Type | Purpose |
|---|---|---|
| `/motion/home_all` | `vinyl_robot_msgs/HomeAll` | Stack-aware 6-phase homing sequence |
| `/motion/move_to_position` | `vinyl_robot_msgs/MoveToPosition` | Coordinated XZA move with velocity scaling |
| `/motion/grip` | `vinyl_robot_msgs/Grip` | Open/close pincher servo with ToF confirmation |
| `/motion/flip_record` | `vinyl_robot_msgs/FlipRecord` | Rotate flip servo to side B |
| `/motion/press_play` | `vinyl_robot_msgs/PressPlay` | Press/lift player servo |
| `/motion/set_speed` | `vinyl_robot_msgs/SetSpeed` | Select 33/45 RPM via speed servo |

**Note on rosbridge + ROS 2 actions**: `ROSLIB.ActionClient` (roslibjs) uses ROS 1 message type naming and is incompatible with ROS 2 rosbridge. The web UI does not call actions directly — instead it publishes to `/user/command` (for state_machine) and `/motion/jog`/`/motion/servo_raw` (for direct motion_coordinator control) using plain String topics.

## Configuration

All mechanical geometry, safety zones, servo pulse widths, and homing parameters live in `config/robot_params.yaml`. During Docker build, config is installed into the vinyl_robot share directory (`/ros_ws/install/vinyl_robot/share/vinyl_robot/config/`). The docker-compose volume mount overlays `./config/` on top of the installed copy at runtime — config is editable without an image rebuild.

CANopen config in `config/bus.yml`: interface, bitrate (1 Mbit), sync period (20 ms = 50 Hz), node IDs, EDS file paths.

EDS files (`stepper_node.eds`, `a_axis_node.eds`, `servo_node.eds`) are the contract between ROS 2 and MCU firmware. Changing OD indices requires updating both.
