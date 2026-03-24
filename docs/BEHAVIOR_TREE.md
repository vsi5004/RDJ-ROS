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
│                   │   │   ├── FlipStage_FlipStage  (ExecuteTrajectory: A+Z concurrent WP0, X retract WP1)
│                   │   │   ├── Selector("EnsureFlipClearance")
│                   │   │   │   ├── FlipClearanceCheck
│                   │   │   │   └── Sequence("RaiseForClearance")
│                   │   │   │       ├── RaiseForClearance_ZUp
│                   │   │   │       └── FlipClearanceCheck2
│                   │   │   ├── FlipRecordAction
│                   │   │   ├── CommitFlipSideAction
│                   │   │   └── FlipReturnToPlayer  (MoveToPosition X→950, Z→60, A→90°)
│                   │   ├── Sequence("FlipThenSwapSubtree")
│                   │   │   ├── IsFlipThenSwapAction
│                   │   │   ├── FTS_FlipStage_FlipStage  (ExecuteTrajectory)
│                   │   │   ├── Selector("EnsureFlipClearance_FTS")
│                   │   │   ├── FTS_FlipRecordAction
│                   │   │   ├── FTS_CommitFlipSide
│                   │   │   ├── FTS_TransitToDeposit  (DynamicTransitToSlotAction → X=50, Z=slot_top_z[prev])
│                   │   │   ├── FTS_ZDownToDeposit    (DynamicMoveToSlot, slot_top=False → Z=slot_z[prev])
│                   │   │   ├── FTS_GripOpen
│                   │   │   ├── FTS_ZUpAfterDeposit   (DynamicMoveToSlot, slot_top=True  → Z=slot_top_z[prev])
│                   │   │   ├── FTS_RetractX          (X→x_clear_mm, skip Z/A)
│                   │   │   ├── FTS_ZToNewSlotTop     (DynamicMoveToSlot, slot_top=True  → Z=slot_top_z[curr])
│                   │   │   ├── FTS_ApproachX         (X→stack_x, skip Z/A)
│                   │   │   ├── FTS_ZDownToPickup     (DynamicMoveToSlot, slot_top=False → Z=slot_z[curr])
│                   │   │   ├── Retry(FTS_GripClose, n=3)
│                   │   │   ├── FTS_SetCurrentSide
│                   │   │   ├── FTS_ZUpAfterPickup    (DynamicMoveToSlot, slot_top=True  → Z=slot_top_z[curr])
│                   │   │   ├── FTS_RetractAfterPickup (X→x_clear_mm, skip Z/A)
│                   │   │   └── FTS_ReturnToPlayer    (ExecuteTrajectory: velocity-mode transit)
│                   │   └── Sequence("SwapSubtree")
│                   │       ├── IsSwapAction
│                   │       ├── Swap_TransitToDeposit (DynamicTransitToSlotAction → X=50, Z=slot_top_z[prev])
│                   │       ├── Swap_ZDownToDeposit   (DynamicMoveToSlot, slot_top=False → Z=slot_z[prev])
│                   │       ├── Swap_GripOpen
│                   │       ├── Swap_ZUpAfterDeposit  (DynamicMoveToSlot, slot_top=True  → Z=slot_top_z[prev])
│                   │       ├── Swap_RetractX         (X→x_clear_mm, skip Z/A)
│                   │       ├── Swap_ZToNewSlotTop    (DynamicMoveToSlot, slot_top=True  → Z=slot_top_z[curr])
│                   │       ├── Swap_ApproachX        (X→stack_x, skip Z/A)
│                   │       ├── Swap_ZDownToPickup    (DynamicMoveToSlot, slot_top=False → Z=slot_z[curr])
│                   │       ├── Retry(Swap_GripClose, n=3)
│                   │       ├── SetCurrentSideFromSlotAction
│                   │       ├── Swap_ZUpAfterPickup   (DynamicMoveToSlot, slot_top=True  → Z=slot_top_z[curr])
│                   │       ├── Swap_RetractAfterPickup (X→x_clear_mm, skip Z/A)
│                   │       └── Swap_ReturnToPlayer   (ExecuteTrajectory: velocity-mode transit)
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
| `safety_ok` | bool | `_on_status`, `_on_safety_status` | velocity_scale > 0 AND no fault |
| `estop_active` | bool | `_on_safety_status`, `_on_sw_estop` | HW or SW e-stop asserted |
| `all_homed` | bool | `_on_status` | All axes report homed bit |
| `motion_fault` | bool | `_on_status` | Fault bit from motion coordinator |
| `motion_fault_msg` | str | `_on_status` | Human-readable fault description |
| `pincher_tof_mm` | float | `_on_status` | Pincher ToF reading in mm |
| `playback_progress` | float | `_on_progress` | Tonearm position 0.0–1.0 |
| `play_mode` | str | `_on_play_mode` | SEQUENTIAL / SINGLE_REPEAT / SIDE_REPEAT |
| `current_record_index` | int | `DecideAction` | Active record slot (0-based) |
| `prev_record_index` | int | `DecideAction` | Previous record slot (used by Swap) |
| `current_side` | str | `DecideAction` | "A" or "B" |
| `queue_size` | int | init | Number of slots in queue_stack |
| `next_action` | str | `DecideAction` | "flip", "swap", or "halt" |
| `force_rehome` | bool | `_on_user_cmd` | Set by "home" user command |
| `override_slot` | int | `_on_select_record` | -1 = none; ≥0 = next slot to load (set by queue selector) |
| `force_flip` | bool | `_on_user_cmd` | Set by "flip" user command; fires immediately in DecideAction |
| `player_has_record` | bool | `SetPlayerStateAction` | True while a disc is on the turntable |
| `slot_sides` | list[str] | `DecideAction` | Per-slot last-known side ("A"/"B"); updated on deposit |
| `start_requested` | bool | `_on_user_cmd` | Set by "start" command; cleared after InitialLoad begins |
| `initial_loaded` | bool | `MarkInitialLoadedAction` | True after first disc has been placed on player |

## Key Behaviours

### SafetyCheck

Returns SUCCESS if `safety_ok` is True and `estop_active` is False.
FAILURE halts the ReactiveRoot, cancelling any running action.

### WaitForRecordFinished

Returns RUNNING while `playback_progress < progress_threshold` (default 0.92, overridable
via ROS parameter `progress_threshold`).  Returns SUCCESS when the threshold is crossed.

### DecideAction

Pure logic node — reads blackboard, writes `next_action`, `current_side`, `current_record_index`, `prev_record_index`, and `slot_sides`. Priority order (highest first):

| Priority | Condition | Action | Effect |
|---|---|---|---|
| 1 | `force_flip = True` | flip | Toggle A↔B; clear `force_flip` |
| 2 | `override_slot ≥ 0` | swap | Saves current side into `slot_sides`; loads target slot at its saved side; clears `override_slot` |
| 3 | `current_side = "A"` | flip | Sets side to B |
| 4 | `current_side = "B"`, SIDE_REPEAT or SINGLE_REPEAT | flip | Sets side to A |
| 5 | `current_side = "B"`, SEQUENTIAL | swap | Saves side B into `slot_sides`; wraps index `(current+1) % queue_size` (no halt at end) |

For swap: saves current index to `prev_record_index` before updating `current_record_index`.

### WaitForStartCommand

Returns RUNNING until `start_requested = True`. Placed as the first child of the `InitialLoad` sequence so the robot idles after homing until the operator presses "Start Playing" in the web UI. The `IsInitialLoaded` gate prevents re-entry after the first disc is loaded.

### SetPlayerStateAction

Pure blackboard write. Two instances in the tree:
- After `LiftRecord_ZUp` → writes `player_has_record = False`
- After `PlaceRecord_GripOpen` → writes `player_has_record = True`

### ResetProgressAction

Writes `playback_progress = 0.0`. Placed after `SetSpeedAction` in both `InitialLoad` and `OneCycle` to prevent stale progress values from triggering `WaitForRecordFinished` immediately at the start of the next cycle.

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

**Z-only** (`skip_x=True, skip_a=True`). X and A must be pre-positioned by the caller. At `initialise()` reads the slot index from the blackboard and builds a fresh `MoveToPosition.Goal` targeting:
- `slot_top=False` (default): `slot_z[idx]` — grip/release height (bottom of slot envelope)
- `slot_top=True`: `slot_z[idx] + shelf_clearance_mm` — entry/exit height (top of slot envelope)

### DynamicTransitToSlotAction

3-waypoint `ExecuteTrajectory` built at runtime. Arrives at `X=stack_x, Z=slot_top_z, A=arrive_a`.

- **WP0** (vel−): X cruises toward stack, Z rises to `safe_z`, **A rotates to `arrive_a`**. A rotation completes while X > 250mm (before A-collision zone). Advance when X ≤ pre_x threshold AND Z + A `in_position`.
- **WP1** (vel−, skip_a): Z descends to `slot_top_z` (top of slot envelope). Advance when X ≤ stack_x + blend_mm.
- **WP2** (position, skip_z, skip_a): X final stop at stack_x.

### ExecuteTrajectoryAction

`_FromConstant` subclass that sends a fixed `ExecuteTrajectory.Goal` (waypoints compiled at tree-build time from YAML or Python dicts). Used for flip staging and transit trajectories.

## Stack Operations — Z-Envelope Safety

Each stack slot has 20mm of usable vertical clearance. The correct sequence for any slot operation:

```
DynamicTransitToSlotAction   → arrives at X=stack_x, Z=slot_top_z (top of envelope), A=arrive_a
DynamicMoveToSlotAction      → Z → slot_z (descend to grip height)
GripAction                   → operate gripper
DynamicMoveToSlotAction(top) → Z → slot_top_z (ascend to top of envelope)
_stack_retract_x             → X → x_clear_mm (250mm) — Z now free to cross slot boundaries
DynamicMoveToSlotAction(top) → Z → slot_top_z[new_slot] (reposition Z while X safe)
_stack_approach_x            → X → stack_x (enter new slot at top of envelope)
... repeat for next slot ...
```

**A is never rotated while X < x_clear_mm (250mm)** — stack vertical pillars constrain the arm. A rotation happens during WP0 of DynamicTransitToSlotAction when X > 400mm. All `_stack_retract_x`, `_stack_approach_x`, and `DynamicMoveToSlotAction` calls are `skip_a=True`.

## Transit — Velocity-Mode Trajectories

All transits now use `ExecuteTrajectoryAction` with velocity-mode waypoints. X runs at cruise speed via TMC5160 RAMPMODE=VEL; the executor monitors X position at 50 Hz to trigger the next waypoint.

**Stack→Turntable** (`_transit_to_turntable`):
- WP0 (vel+, skip_a): X cruises, Z descends to `tt_z_approach`. Advance when X ≥ 500mm.
- WP1 (position): X to 950mm, A rotates to 90°.

**Turntable→Stack** (`_transit_to_stack`):
- WP0 (vel−): X cruises, Z rises to `safe_z`, **A rotates to 270°** (completes before X < 250mm). Advance when X ≤ 500mm AND Z + A done.
- WP1 (position, skip_z, skip_a): X to stack_x.

`trailing_split: 0.50` in `robot_params.yaml` sets the midpoint threshold (500mm).

## Flip vs. Swap — Return to Player

After a flip, the arm is at flip_staging (X≈700mm). `FlipReturnToPlayer` issues a single three-axis `MoveToPosition` (X→950, Z→60, A→90°).

After a swap or flip-then-swap, the arm is at `x_clear_mm=250mm` after the post-pickup X retract. `Swap_ReturnToPlayer` / `FTS_ReturnToPlayer` uses `_transit_to_turntable` (velocity-mode trajectory starting from X=250mm).

Both paths deliver the arm to `X=turntable.x_mm, Z=turntable.z_approach_mm, A=turntable.a_deg` before `PlaceRecord` runs.

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
