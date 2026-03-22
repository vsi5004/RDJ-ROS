# Behavior Tree — Operational Workflow

## Framework

BehaviorTree.CPP (ROS 2 compatible). Chosen over SMACC2 because behavior trees handle interruptible, conditional sequences better — a safety event mid-pick needs to pause the tree, not crash a state machine.

## Tree Structure

```
ReactiveSequence (root)
├── SafetyCheck (condition) — re-evaluated every tick
├── Sequence (main operational loop)
│   ├── WaitForRecordFinished
│   ├── MoveToPlayer (X, Z, A coordinated move)
│   ├── LiftRecord (grip + Z up)
│   ├── DecideAction (flip or swap?)
│   │   ├── FlipSubtree (flip servo → replace on platter)
│   │   └── SwapSubtree (move to queue → exchange → return to player)
│   ├── PlaceRecord (Z down + release)
│   ├── PressPlay (player servo)
│   ├── SetSpeed (33 or 45 rpm)
│   └── WaitForSide (monitor tonearm via turntable_monitor)
└── Fallback (recovery)
```

## ReactiveSequence at Root

A standard Sequence ticks each child in order. A ReactiveSequence re-evaluates its condition children on every tick, even while later children are running.

This means SafetyCheck runs continuously. If the LiDAR safety node reports someone in the stop zone while the robot is mid-move, the ReactiveSequence immediately halts — the running action gets preempted. Safety is orthogonal to the operational flow.

## Blackboard State

The behavior tree uses a BehaviorTree.CPP Blackboard (shared key-value store) for persistent state:

| Key | Type | Description |
|-----|------|-------------|
| `current_record_index` | int | Which record is on the platter (0–5) |
| `current_side` | enum | A or B |
| `queue_size` | int | Number of records in the queue |
| `play_mode` | enum | SEQUENTIAL, SINGLE_REPEAT, SIDE_REPEAT |
| `playback_progress` | float | 0.0–1.0 from turntable_monitor |
| `all_axes_homed` | bool | From motion_coordinator status |
| `safety_ok` | bool | From lidar_safety |

## Key Nodes

### SafetyCheck (Condition)

Returns SUCCESS if safety_ok is true (velocity_scale > 0.0 and estop is false). Returns FAILURE otherwise, which causes the ReactiveSequence to halt all running children.

### WaitForRecordFinished (Condition/Action)

Monitors `/turntable/progress`. Returns RUNNING while progress < threshold (e.g., 0.95). Returns SUCCESS when the tonearm reaches the run-out groove. Also triggers on the tonearm returning to rest position (auto-return turntables).

### MoveToPlayer

Calls `/motion/move_to_position` action server with the turntable coordinates from YAML config. Returns RUNNING while the action is executing, SUCCESS when all axes report in-position, FAILURE on fault.

### LiftRecord

Sequence of:
1. Move Z down to record surface (using Pincher ToF to confirm height)
2. Close grip (call `/motion/grip` with close=true)
3. Verify ToF confirms grip contact
4. Move Z up to clearance height

### DecideAction

Reads `current_side` and `play_mode` from blackboard:
- If `current_side == A` and `play_mode != SIDE_REPEAT`: choose FlipSubtree, set `current_side = B`
- If `current_side == B` or `play_mode == SIDE_REPEAT`: choose SwapSubtree, increment `current_record_index`, set `current_side = A`
- If `current_record_index >= queue_size` and `play_mode == SEQUENTIAL`: halt (all records played)

### FlipSubtree

1. Command flip servo (rotate record 180°)
2. Wait for servo completion
3. (Record is now side B up, still in gripper)

### SwapSubtree

1. Move to queue stack (X to queue position, Z to correct slot height, A to stack-facing angle)
2. Place current record in its slot (Z down, release grip)
3. Move Z up to clearance
4. Move to next record's slot (adjust Z for different slot height)
5. Pick next record (Z down, grip close, Z up)
6. Move back to turntable (X to turntable, A to platter-facing angle)

### PlaceRecord

1. Move Z down toward platter (use Pincher ToF to confirm approach)
2. Release grip (call `/motion/grip` with close=false)
3. Move Z up slightly to clear record surface
4. Retract (Z to clearance height, A to park if desired)

### PressPlay / SetSpeed

Simple servo commands via action servers. Press play engages the turntable motor. Set speed selects 33 or 45 RPM.

## Action Preemption

All motion actions are preemptable. When the behavior tree cancels a running action (due to safety interrupt or tree abort):

1. Action server receives cancel request
2. Motion coordinator writes XTARGET = XACTUAL to all active axes
3. Axes decelerate to stop using their configured DMAX
4. Action reports cancelled result
5. Tree can re-attempt or enter recovery

## Error Handling

### StallGuard fault during a pick/place

1. EMCY received → motion_coordinator publishes fault on `/motion/status`
2. Behavior tree's SafetyCheck or a dedicated FaultMonitor node detects the fault
3. Tree enters recovery Fallback:
   - Open gripper (release any held record)
   - Move Z up to clearance
   - Move X to safe zone
   - Report fault to user (via diagnostics/UI)
   - Wait for manual clear

### Communication loss (heartbeat timeout)

1. canopen_master detects missing heartbeat, publishes on `/canopen/node_N/nmt_state`
2. Motion coordinator halts all axes
3. Tree enters fault state
4. After node recovers (heartbeat resumes), re-home that axis before resuming

### Record dropped

If the Pincher ToF reads an unexpected distance after a grip close (too far = didn't grip properly), the tree aborts the pick and retries. After 3 failures, report fault.

## Operational Startup Sequence

1. Behavior tree starts, first tick checks `all_axes_homed`
2. If not homed → call `/motion/home_all` action, wait for completion
3. If homing fails → report error, do not enter operational loop
4. Once homed → enter main ReactiveSequence operational loop
5. First iteration: pick record 0 side A from queue, place on turntable, press play
6. Loop: wait for side to finish → flip or swap → play → repeat

## User Interface (Future)

The behavior tree does not implement a UI. A future web dashboard could:
- Publish to `/user/play_mode` to change sequential/repeat modes
- Publish to `/user/skip` to advance to next record
- Subscribe to `/motion/status` and `/turntable/progress` for display
- Subscribe to `/diagnostics` for health monitoring

The behavior tree would add condition nodes checking for these user commands.
