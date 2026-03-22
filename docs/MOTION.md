# Motion Coordinator and Homing

## Motion Coordinator Responsibilities

- Convert mm/degrees to microsteps using mechanical ratios from YAML config
- Enforce safety velocity scaling from the LiDAR safety node
- Write coordinated RPDOs for multi-axis moves, gated by SYNC
- Monitor RAMPSTAT and status words to determine move completion
- Orchestrate homing sequence order with spatial awareness
- Host action servers for the behavior tree

## Coordinate Systems

The SBC works in physical units. The MCUs work in microsteps. Conversion happens in the motion coordinator.

- **X axis**: millimeters (0mm = stack endstop, 1000mm = turntable end)
- **Z axis**: millimeters (0mm = lowest, 150mm = highest/endstop)
- **A axis**: degrees (0° = endstop, 300° = maximum rotation, 180° = park)
- **Pincher/Player**: microseconds pulse width (500–2500μs)

## Homing Sequence — Stack-Aware

### Phase 0 — Assess (no motors move)

Read all sensors over CAN:
- A axis pot → coarse angle (from TPDO or SDO read)
- X axis ToF → approximate rail position
- Z axis ToF → approximate height

Classify starting regime:

```python
def classify_regime(x_pos, a_angle):
    stack_angle = config.record_stack.a_center_deg
    tolerance = config.record_stack.a_tolerance_deg
    pointing_at_stack = abs(normalize_angle(a_angle - stack_angle)) < 90
    
    if x_pos < config.record_stack.x_danger_mm and pointing_at_stack:
        return IN_STACK  # Arm may be inside shelved structure
    elif x_pos > config.turntable.x_danger_mm:
        return NEAR_TURNTABLE
    else:
        return OPEN_AIR
```

### IN_STACK Regime — Stack Extraction

**Critical constraint**: When the arm is between shelf floors, Z movement is dangerous in both directions (shelves above and below with ~20mm clearance). A rotation is dangerous (pincher sweeps into shelf walls). The ONLY safe degree of freedom is X retract along the rail.

**Precondition**: A axis must be roughly aligned with X rail (within ±`a_tolerance_deg` of the stack-facing angle). If not aligned, the pincher will clip shelf edges during retract → ABORT, require manual intervention.

```python
if regime == IN_STACK:
    stack_angle = config.record_stack.a_center_deg
    if abs(normalize_angle(a_angle - stack_angle)) > config.record_stack.a_tolerance_deg:
        return abort("Arm inside stack but A axis misaligned — manual repositioning required")
    
    # Retract X only — Z and A FROZEN
    await move_axis_velocity(node_x, direction=AWAY_FROM_STACK,
                             speed=config.velocities.stack_retract,  # 5 mm/s
                             stallguard_sensitivity=MAX)
    await wait_until(lambda: read_xactual_mm(node_x) > config.record_stack.x_clear_mm)
    await halt_axis(node_x)
    # Now safe — fall through to standard sequence
```

### Standard Sequence (OPEN_AIR or after extraction)

**Phase 1 — Z safe height**: Move Z up to clear turntable and stack. Always safe when not inside stack — nothing above the robot.

```python
if z_pos < config.safe_zone.z_clear_mm:
    await move_axis_velocity(node_z, direction=UP, speed=config.velocities.z_initial_up)
    await wait_until(lambda: read_tof(node_z) > config.safe_zone.z_clear_mm)
    await halt_axis(node_z)
```

**Phase 2 — X to safe middle zone**: Check A safety first. If A is pointing toward a wall/furniture (outside safe window), move Z to max height first, then rotate A to park at very slow speed with max StallGuard sensitivity. Then move X to safe zone center.

```python
a_safe = config.safety.a_axis.min_safe_deg < a_angle < config.safety.a_axis.max_safe_deg

if not a_safe and x_zone != SAFE_MIDDLE:
    # Worst case: A dangerous AND X near obstacle
    await move_axis_velocity(node_z, UP, SLOW)
    await wait_until_endstop(node_z)
    await rotate_a_to_park(a_angle, speed=config.velocities.a_emergency_park)
elif not a_safe:
    await rotate_a_to_park(a_angle, speed=config.velocities.a_emergency_park)

if x_zone != SAFE_MIDDLE:
    await move_axis_velocity(node_x, toward_middle(x_pos), config.velocities.x_to_safe_zone)
    await wait_until(lambda: in_safe_zone(read_xactual_mm(node_x)))
    await halt_axis(node_x)
```

**Phase 3 — Home A axis**: X is safe, Z is high. Read pot → determine shortest path to endstop. Send home command to A axis MCU. Wait for "homed" status. Move to park angle (180°).

**Phase 4 — Home X axis**: A is parked, Z is high. Send home command to X MCU. Endstop at x=0. After homing, move to safe zone center.

**Phase 5 — Home Z axis**: All axes in known positions. Home Z upward to endstop. After homing, lower to clearance height.

**Phase 6 — Report all homed.** Behavior tree can begin operational loop.

### Homing Velocities

All homing moves use reduced speeds for safety:

| Move | Speed | StallGuard |
|------|-------|------------|
| Stack retract (X) | 5 mm/s | Maximum sensitivity |
| Z initial up | 10 mm/s | High sensitivity |
| X to safe zone | 20 mm/s | High sensitivity |
| A emergency park | 5 °/s | Maximum sensitivity |
| A normal home | 30 °/s | High sensitivity |
| X home to endstop | 40 mm/s | Normal |
| Z home to endstop | 20 mm/s | Normal |

### Error Recovery

Every `await` in the homing sequence is cancellable. If safety system fires estop during homing, all axes halt, homing fails. Next attempt starts fresh from Phase 0 — re-reads all sensors, makes new decisions. Never assumes previous attempt left things in a known state.

## Safety Integration

The motion coordinator subscribes to `/safety/velocity_scale` and `/safety/estop`.

Before sending any RPDO, the coordinator multiplies the requested velocity by the safety scale factor. If scale is 0.0, the move is blocked. If estop is true, all axes receive immediate halt (write current XACTUAL as XTARGET).

During an active move, if the safety scale drops:
- To 0.25: update VMAX on all active axes to 25% of their configured maximum
- To 0.0: write XTARGET = XACTUAL on all axes (decel stop)

When safety clears (scale returns to 1.0), the move resumes from the current position toward the original target.

## Pot Sanity Check

The motion coordinator (or diagnostics node) periodically reads the A axis pot value from the TPDO and compares it to the expected angle derived from XACTUAL:

```python
expected_angle = xactual_to_degrees(a_tpdo.actual_position)
pot_angle = a_tpdo.pot_angle / 10.0  # 0.1° units
if abs(normalize_angle(expected_angle - pot_angle)) > 5.0:
    # Belt slip or mechanical failure
    publish_emcy("A axis position divergence")
    halt_all_axes()
```
