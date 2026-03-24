# Motion Coordinator and Homing

## Responsibilities

- Convert mm/degrees to microsteps using ratios from `robot_params.yaml`
- Enforce safety velocity scaling (multiply all velocities by `/safety/velocity_scale`)
- Write coordinated RPDOs for multi-axis moves, gated by SYNC
- Monitor RAMPSTAT and status words for completion
- Orchestrate homing with spatial awareness
- Host action servers for behavior tree and direct commands
- Handle incremental jog and raw servo commands from web UI

## Coordinate Systems

- **X**: millimetres (0 = stack endstop, ~950 = turntable)
- **Z**: millimetres (0 = lowest, ~150 = endstop)
- **A**: degrees (0 = endstop, 180 = park/away, 300 = max)
- **Pincher/Player**: microseconds pulse width (500–2500 µs)

## Manual Control Topics

These topics allow the web UI to drive the robot directly without going through the behavior tree. They are intended for calibration and debugging.

### `/motion/jog` (std_msgs/String)
Format: `"<axis> <delta>"` — e.g. `"x 10.0"`, `"z -5.0"`, `"a 90.0"`

motion_coordinator adds the delta to the current `actual_pos` in steps and sends an RPDO immediately. Jog speed: 50 mm/s for linear axes, 30 °/s for A axis, with safety scaling applied. E-stop blocks jog.

### `/motion/servo_raw` (std_msgs/String)
Format: `"<node_name> <s1_us> <s2_us>"` — e.g. `"pincher 1500 1200"`

Sets both servo channels on a node simultaneously. Pulse widths clamped to 500–2500 µs. Bypasses the action abstraction (Grip, FlipRecord, etc.) — intended for finding servo endpoints during initial calibration. See HMI_AND_IDENTIFICATION.md for the calibration workflow.

## E-Stop Architecture

Two independent estop sources, tracked separately in motion_coordinator:

| Source | Topic | Managed by | Behaviour |
|---|---|---|---|
| Hardware (LiDAR) | `/safety/status` (`estop` field) | `lidar_safety` | Published continuously; auto-clears when object leaves zone |
| Software (operator) | `/user/estop` | Web UI / operator | Latching; must be explicitly cleared |

`motion_coordinator.estop` property returns `_hw_estop OR _sw_estop`. On either activating: `_halt_all()` sets XTARGET = XACTUAL on all stepper axes. Clearing software estop when hardware estop is still active has no effect on motion.

## Homing Sequence (Stack-Aware)

See `src/motion_coordinator/motion_coordinator/homing.py` for the full implementation.

### Phase 0 — Assess (no motion)
Read A angle, X position, Z height. Classify regime:

```
if X < x_danger_mm AND A pointing at stack (within 90° of stack angle):
    IN_STACK    ← arm may be between shelf floors
elif X > turntable.x_danger_mm:
    NEAR_TURNTABLE
else:
    OPEN_AIR
```

### IN_STACK Extraction (critical safety path)

When arm is between shelf floors, Z movement is dangerous in both directions (shelf above and below). Rotation sweeps into shelf walls.

**Only safe action: retract X along the rail.**

Precondition: A must be aligned with the rail (within ±`a_tolerance_deg`). If misaligned, **ABORT** — manual intervention required. This is a hard failure; the homing sequence returns False.

Execute: retract X only (Z and A frozen) at 5 mm/s until X > `x_clear_mm`, then halt. Fall through to standard sequence.

### Standard Sequence (OPEN_AIR or post-extraction)

**Phase 1 — Z safe height**: If Z < `z_clear_mm`, move Z up at 10 mm/s until Z > `z_clear_mm`.

**Phase 2 — X to safe zone**: Check if A is pointing toward a wall (outside `min_safe_deg`–`max_safe_deg`). If unsafe AND X not in safe zone: move Z to max height, rotate A to park (180°) at 5°/s. Then move X to safe zone centre at 20 mm/s.

**Phase 3 — Home A axis**: X in safe zone, Z high. Send home command, wait for `homed` status bit, then move directly to `a_x_homing_deg` (90°, faces turntable). Skips the intermediate 180° park to avoid an unnecessary extra move.

**Phase 4 — Home X axis**: A is at 90° — arm points toward the turntable, clearing the stack structure as X homes to endstop at 40 mm/s. A=180° (park) risks clipping the stack. After, move X to safe zone centre.

**Phase 5 — Home Z axis**: Rotate A back to 180° (park). Home Z upward to endstop at 20 mm/s, lower to `z_clear_mm`.

**Phase 6 — Done**: Publish `HomeAll.Feedback(current_phase='done', axes_homed=0b111)`, return True.

### Homing Velocities

| Move | Speed | Notes |
|---|---|---|
| X retract from stack | 5 mm/s | Most cautious — shelves on both sides |
| Z initial up | 10 mm/s | High StallGuard |
| A emergency park | 5°/s | Max StallGuard |
| X to safe zone | 20 mm/s | High StallGuard |
| A home to endstop | 30°/s | Normal StallGuard |
| Z home to endstop | 20 mm/s | Normal StallGuard |
| X home to endstop | 40 mm/s | Normal StallGuard |

All velocities are multiplied by `safety_scale` before being sent to the CAN nodes.

### Cancellation and Error Recovery

Every polling loop checks `goal_handle.is_cancel_requested` and `node.estop`. If either fires, all axes halt and homing returns False.

If safety fires estop during homing, all axes halt immediately. The next homing attempt starts Phase 0 fresh — re-reads sensors, makes new regime decisions. Never assumes previous state.

## Safety Integration

motion_coordinator subscribes `/safety/status` (`vinyl_robot_msgs/SafetyStatus`), which carries `velocity_scale`, `estop`, and `reason` in a single message.

- Before any RPDO: multiply velocity by `safety_scale`. If scale = 0.0, move is blocked.
- On estop: write XTARGET = XACTUAL on all axes (immediate deceleration stop).
- During an active move: if scale drops to 0.25, active VMAX updates to 25%. If scale drops to 0.0, XTARGET = XACTUAL. When scale returns to 1.0, node resumes toward original target from current position.

## Stack Slot Z-Envelope Safety

Each queue stack slot has `shelf_clearance_mm` (20mm) of usable vertical space. Moving Z outside this envelope while X is inside the stack (X < `x_clear_mm` = 250mm) will collide with the shelf above or below.

### Slot geometry

| Slot | `slot_z` (grip height) | `slot_top_z` (entry/exit) |
|------|------------------------|--------------------------|
| 0    | 20mm                   | 40mm                     |
| 1    | 60mm                   | 80mm                     |
| 2    | 100mm                  | 120mm                    |
| 3    | 140mm                  | 160mm                    |
| 4    | 180mm                  | 200mm                    |

`slot_top_z = slot_z + shelf_clearance_mm`. Shelf material occupies [slot_top_z[i], slot_z[i+1]] (20mm thick between each pair).

### Invariants enforced by the behavior tree

1. **Entry** — X approaches stack_x only after Z is set to `slot_top_z` for the target slot (top of envelope). Stack entry is always at the top.
2. **Descent** — Z lowers to `slot_z` (grip height) only after X=stack_x is confirmed.
3. **Ascent** — Z raises back to `slot_top_z` before X retracts.
4. **X retract** — X moves to `x_clear_mm` (250mm) before any Z change that crosses a slot boundary.
5. **A rotation** — A must not rotate while X < `x_clear_mm`. Stack vertical pillars constrain A in that zone. A rotates to `arrive_a` during the transit (X > 250mm) and is frozen (`skip_a=True`) for all stack entry, descent, ascent, and retract moves.

### Action servers involved

- `DynamicTransitToSlotAction` — 3-waypoint velocity-mode trajectory: Z+A early (X far), Z descend to `slot_top_z`, X final stop. Arrives at `X=stack_x, Z=slot_top_z, A=arrive_a`.
- `DynamicMoveToSlotAction` — Z-only move (`skip_x, skip_a`). `slot_top=False` targets `slot_z`; `slot_top=True` targets `slot_top_z`.
- `_stack_approach_x` / `_stack_retract_x` — X-only moves to stack_x / x_clear_mm (`skip_z, skip_a`).

## ExecuteTrajectory Action Server

`vinyl_robot_msgs/action/ExecuteTrajectory` accepts a list of `Waypoint` messages and executes them sequentially with 50 Hz polling. Advance conditions per waypoint:

- **Velocity-mode WP** (`ramp_mode=1` vel+ or `ramp_mode=2` vel−): X runs at cruise speed; advance when X crosses the `x_mm` threshold (and Z+A `in_position` if not skipped)
- **Position-mode intermediate WP**: advance when all active axes are within `blend_radius_mm` / `blend_radius_deg` — arm redirects mid-deceleration, no stop
- **Final WP**: waits for full `in_position` on all active axes

### RPDO byte 7 bitfield

Firmware decodes RPDO byte 7 as a packed bitfield:

```
bits [1:0]: ramp_mode (0=position, 1=vel+, 2=vel-)
bits [7:2]: dmax_factor (0=use cfg_dmax; 1-63 = factor/64 × cfg_dmax)
```

`dmax_factor > 0` reduces the deceleration for intermediate waypoints so the axis is still at high speed when the next XTARGET is sent (smoother blending). The mock canopen master masks bits [7:2] before storing `self.ramp_mode`.

## Pot Sanity Check (Future)

motion_coordinator/diagnostics will periodically read the A axis pot angle from TPDO bytes 6–7, compare to expected angle derived from XACTUAL. Divergence > 5° triggers an EMCY "A axis position divergence" and halts all axes — indicates belt slip or mechanical failure.
