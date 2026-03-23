# Behavior Tree — Operational Workflow

## Framework

**py_trees** + **py_trees_ros** (Python, ROS 2 Humble compatible).
All tree construction lives in `src/state_machine/state_machine/tree_builder.py`.
The node that owns the tree, manages subscriptions, and drives the tick timer is
`src/state_machine/state_machine/state_machine_node.py`.

## Tree Topology

```
Selector("RootSelector")                    ← failure → fault recovery
├── Sequence("ReactiveRoot", memory=False)  ← SafetyCheck re-evaluated every tick
│   ├── SafetyCheck
│   └── Sequence("MainFlow", memory=False)
│       ├── Selector("HomingSubtree")
│       │   ├── IsAllHomed
│       │   └── Sequence("DoHoming")
│       │       ├── LED:HOMING
│       │       ├── HomeAllAction
│       │       ├── ClearForceRehomeAction
│       │       └── LED:OPERATIONAL
│       └── Selector("OperationalOrHalt")
│           ├── IsHaltRequested
│           └── LoopDecorator
│               └── Sequence("OneCycle", memory=True)
│                   ├── WaitForRecordFinished
│                   ├── PressStop
│                   ├── Lift_Approach          (MoveToPosition)
│                   ├── Sequence("LiftRecord")
│                   │   ├── LiftRecord_ZDown
│                   │   ├── Retry(GripClose, n=3)
│                   │   └── LiftRecord_ZUp
│                   ├── DecideAction
│                   ├── Selector("FlipOrSwap")
│                   │   ├── Sequence("FlipSubtree")
│                   │   │   ├── IsFlipAction
│                   │   │   ├── FlipStage_SwingA    (A→180°, skip X/Z)
│                   │   │   ├── FlipStage_BackX     (X→700mm, skip Z/A)
│                   │   │   ├── FlipStage_RaiseZ    (Z→80mm, skip X/A)
│                   │   │   ├── Selector("EnsureFlipClearance")
│                   │   │   │   ├── FlipClearanceCheck
│                   │   │   │   └── Sequence("RaiseForClearance")
│                   │   │   │       ├── RaiseForClearance_ZUp
│                   │   │   │       └── FlipClearanceCheck2
│                   │   │   ├── FlipRecordAction
│                   │   │   └── FlipReturnToPlayer  (X→950, Z→60, A→90°)
│                   │   └── Sequence("SwapSubtree")
│                   │       ├── IsSwapAction
│                   │       ├── Swap_ZClear
│                   │       ├── Sequence("Swap_TransitToStack")
│                   │       │   ├── Swap_TransitToStack_Trail  (X→500, skip A)
│                   │       │   └── Swap_TransitToStack_Arrive (X→50, A→270°)
│                   │       ├── Swap_ZToOldSlot     (DynamicMoveToSlotAction)
│                   │       ├── Swap_GripOpen
│                   │       ├── Swap_ZClearAfterDeposit
│                   │       ├── Swap_ZToNewSlot     (DynamicMoveToSlotAction)
│                   │       ├── Retry(Swap_GripClose, n=3)
│                   │       ├── Swap_ZClearWithRecord
│                   │       └── Sequence("Swap_ReturnToPlayer")
│                   │           ├── Swap_ReturnToPlayer_Trail  (X→500, skip A)
│                   │           └── Swap_ReturnToPlayer_Arrive (X→950, A→90°)
│                   ├── Sequence("PlaceRecord")
│                   │   ├── PlaceRecord_ZDown
│                   │   ├── PlaceRecord_GripOpen
│                   │   └── PlaceRecord_ZUp
│                   ├── PressPlay
│                   ├── SetSpeed33RPM
│                   └── LED:PLAYING
└── FaultRecovery   (built by recovery.py)
```

## ReactiveRoot — Safety Preemption

`ReactiveRoot` uses `memory=False`, meaning `SafetyCheck` is re-evaluated on every
tick even while a motion action is RUNNING.  If the LiDAR safety node reports someone
in the stop zone (velocity_scale = 0.0) or an e-stop fires, `SafetyCheck` returns
FAILURE and the running action is cancelled via the py_trees_ros action client's
`terminate()` hook, which sends a ROS 2 cancel request to the motion coordinator.
When safety clears, the sequence restarts from `SafetyCheck` and motion resumes.

## Blackboard

Namespace: `/rdj`.  Written by `StateMachineNode` subscription callbacks; read by tree behaviours.

| Key | Type | Written by | Description |
|---|---|---|---|
| `safety_ok` | bool | `_on_status`, `_on_scale` | velocity_scale > 0 AND no fault |
| `estop_active` | bool | `_on_hw_estop`, `_on_sw_estop`, `_on_user_cmd` | HW or SW e-stop asserted |
| `all_homed` | bool | `_on_status` | All axes report homed bit |
| `motion_fault` | bool | `_on_status` | Fault bit from motion coordinator |
| `motion_fault_msg` | str | `_on_status` | Human-readable fault description |
| `pincher_tof_mm` | float | `_on_status` | Pincher ToF reading in mm |
| `playback_progress` | float | `_on_progress` | Tonearm position 0.0–1.0 |
| `play_mode` | str | `_on_play_mode` | SEQUENTIAL / SINGLE_REPEAT / SIDE_REPEAT |
| `current_record_index` | int | `DecideAction`, `_on_select_record` | Active record slot (0-based) |
| `prev_record_index` | int | `DecideAction` | Previous record slot (used by Swap) |
| `current_side` | str | `DecideAction` | "A" or "B" |
| `queue_size` | int | init | Number of slots in queue_stack |
| `next_action` | str | `DecideAction` | "flip", "swap", or "halt" |
| `force_rehome` | bool | `_on_user_cmd` | Set by "home" user command |

## Key Behaviours

### SafetyCheck

Returns SUCCESS if `safety_ok` is True and `estop_active` is False.
FAILURE halts the ReactiveRoot, cancelling any running action.

### WaitForRecordFinished

Returns RUNNING while `playback_progress < progress_threshold` (default 0.92, overridable
via ROS parameter `progress_threshold`).  Returns SUCCESS when the threshold is crossed.

### DecideAction

Pure logic node — reads blackboard, writes `next_action`, `current_side`, and record indices:

| `current_side` | `play_mode` | Action | Side after |
|---|---|---|---|
| A | any | flip | B |
| B | SIDE_REPEAT or SINGLE_REPEAT | flip | A |
| B | SEQUENTIAL, records remain | swap | A |
| B | SEQUENTIAL, last record | halt | — |

For swap: saves current index to `prev_record_index`, increments `current_record_index`.

### IsAllHomed / IsFlipAction / IsSwapAction / IsHaltRequested

Simple blackboard condition reads; return SUCCESS or FAILURE with no side effects.

### ClearForceRehomeAction

Clears the `force_rehome` blackboard flag after `HomeAllAction` confirms success.
The flag is set by the `"home"` user command and causes `IsAllHomed` to return FAILURE,
triggering a fresh homing sequence.

### LoopDecorator

Custom decorator that resets its child to INVALID each time it returns SUCCESS, creating
an eternal operational loop.  Propagates FAILURE unchanged (fault recovery path).

### DynamicMoveToSlotAction

Subclasses `py_trees_ros.action_clients.FromBlackboard` rather than `FromConstant`.
At `initialise()` it reads `current_record_index` (or `prev_record_index`) and the
slot Z heights from config, builds a fresh `MoveToPosition.Goal`, writes it to its own
private blackboard key, then calls `super().initialise()` which reads and sends it.

This avoids the `FromConstant` limitation where the goal is captured at construction time
and cannot be updated by overriding `self.action_goal`.

## Transit — Trailing A Rotation

Moving a record between the stack (X≈50mm) and the turntable (X≈950mm) while holding
it in the gripper risks the record swinging into obstacles if A rotates too early.

Both transit helpers use a two-phase approach:

```
Phase 1 — Trail  (skip_a=True):
    X moves to the midpoint (default 50% of travel), Z adjusts to target height.
    A stays wherever it is — the record "trails" the carriage.

Phase 2 — Arrive:
    X moves to the destination, A rotates to the arrival angle simultaneously.
    By the time A starts rotating, X is halfway across the open space.
```

`trailing_split` in `robot_params.yaml` (default 0.50) controls when the rotation begins.

**Turntable→Stack** (`_transit_to_stack`): midpoint X=500mm, arrival A=270°
**Stack→Turntable** (`_transit_to_turntable`): midpoint X=500mm, arrival A=90°

## Flip vs. Swap — Return to Player

After a flip, the arm is at X≈700mm (flip staging), close to the turntable.
`FlipReturnToPlayer` issues a single three-axis move (X→950, Z→60, A→90°) to return
directly — no backward motion.

After a swap, the arm is at X=50mm (stack) holding the new record.
`Swap_ReturnToPlayer` uses the standard `_transit_to_turntable` two-phase helper to
return safely with the trailing A logic.

Both paths deliver the arm to `X=turntable.x_mm, Z=turntable.z_approach_mm, A=turntable.a_deg`
before `PlaceRecord` runs, so `PlaceRecord` only needs ZDown → GripOpen → ZUp.

## Logging

The tick timer logs at 10 Hz but outputs sparingly:

- **Behaviour change**: logged immediately when `tip()` changes, showing name and feedback message.
- **Stuck behaviour**: repeated every 10 ticks (1 s) with elapsed time — e.g.
  `[tree] Lift_Approach (+3s): goal accepted :)`.

The motion coordinator logs a `[move]` line at the start of every
`/motion/move_to_position` goal, showing commanded axes and velocity scale:
```
[move] X→950mm, Z→60mm, A→90° (scale=1.00)
[move] Z→45mm (scale=1.00)
```
Skip flags suppress the axis from the log, so single-axis moves are easy to read.

## Homing Re-trigger

The operator can force a re-home at any time by publishing `"home"` to `/user/command`.
This sets `force_rehome=True` on the blackboard, which makes `IsAllHomed` return FAILURE,
causing the tree to fall through to `DoHoming`.  The flag is cleared only after
`HomeAllAction` succeeds (by `ClearForceRehomeAction`).

## Fault Recovery

`FaultRecovery` (built by `recovery.py`) is the second child of `RootSelector`.
It runs only when `ReactiveRoot` returns FAILURE (safety or motion fault).

Sequence:
1. Open gripper (release any held record)
2. Move Z to clearance height
3. Move X/Z/A to safe park position
4. `WaitForManualClear` — holds RUNNING until `all_homed=True` and `motion_fault=False`

The operator must clear the fault (via e-stop reset and re-home command) to resume.
