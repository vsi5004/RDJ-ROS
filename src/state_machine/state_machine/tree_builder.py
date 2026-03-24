"""
tree_builder.py — Assembles the full behavior tree for the vinyl robot.

Entry point: build_tree(node, config) → root Behaviour

Tree topology (see docs/BEHAVIOR_TREE.md for full specification):

  Selector("RootSelector")                   ← catches all failures → recovery
  ├── Sequence("ReactiveRoot", memory=False)  ← SafetyCheck re-evaluated every tick
  │   ├── SafetyCheck
  │   └── Sequence("MainFlow", memory=True)
  │       ├── Selector("HomingSubtree")
  │       │   ├── IsAllHomed
  │       │   └── Sequence("DoHoming")
  │       │       ├── LED:HOMING
  │       │       ├── HomeAllAction
  │       │       └── LED:OPERATIONAL
  │       └── Selector("OperationalOrHalt")
  │           ├── IsHaltRequested
  │           └── LoopDecorator
  │               └── Sequence("OneCycle")
  │                   ├── WaitForRecordFinished
  │                   ├── PressStop
  │                   ├── MoveToPlayerApproach
  │                   ├── Sequence("LiftRecord")
  │                   │   ├── ZDownToPlatter
  │                   │   ├── Retry(GripClose, n=3)
  │                   │   └── ZUpToClear
  │                   ├── DecideAction
  │                   ├── Selector("FlipOrSwap")
  │                   │   ├── FlipSubtree         (next_action=="flip")
  │                   │   ├── FlipThenSwapSubtree (next_action=="flip_swap": B-side SEQUENTIAL)
  │                   │   └── SwapSubtree         (next_action=="swap")
  │                   ├── Sequence("PlaceRecord")
  │                   │   ├── MoveToPlayerApproach
  │                   │   ├── ZDownToPlatter
  │                   │   ├── GripOpen
  │                   │   └── ZUpToClear
  │                   ├── PressPlay
  │                   └── SetSpeed33
  └── FaultRecovery
"""

import py_trees
from rclpy.node import Node

from . import blackboard_keys as K
from .actions import (
    ClearForceRehomeAction,
    CommitFlipSideAction,
    DecideAction,
    DynamicMoveToSlotAction,
    DynamicTransitToSlotAction,
    ExecuteTrajectoryAction,
    FlipRecordAction,
    GripAction,
    HomeAllAction,
    LoopDecorator,
    MarkInitialLoadedAction,
    MoveToPositionAction,
    PressPlayAction,
    PublishLEDAction,
    ResetProgressAction,
    SetCurrentSideFromSlotAction,
    SetPlayerStateAction,
    SetSpeedAction,
)
from .conditions import (
    FlipClearanceCheck,
    IsAllHomed,
    IsFlipAction,
    IsFlipThenSwapAction,
    IsHaltRequested,
    IsInitialLoaded,
    IsSwapAction,
    SafetyCheck,
    WaitForRecordFinished,
    WaitForStartCommand,
)
from .recovery import build_recovery_subtree


def build_tree(node: Node, config: dict) -> py_trees.behaviour.Behaviour:
    """
    Assemble and return the root behaviour.  Call BehaviourTree.setup() on
    the result to create all ROS action clients before ticking.
    """
    py_trees.blackboard.Blackboard.enable_activity_stream(100)

    pos = config["positions"]
    turntable = pos["turntable"]
    flip_st = pos["flip_staging"]
    safe_park = pos["safe_park"]
    progress_threshold = config.get("turntable_monitor", {}).get("progress_threshold", 0.92)
    clearance_min_mm = config.get("flip", {}).get("clearance_min_mm", 178.0)

    # ── Reusable position shortcuts ───────────────────────────────────────────

    def _move_to_player_approach(name: str = "MoveToPlayerApproach") -> MoveToPositionAction:
        return MoveToPositionAction(
            name=name,
            node=node,
            x_mm=turntable["x_mm"],
            z_mm=turntable["z_approach_mm"],
            a_deg=turntable["a_deg"],
        )

    def _z_down_to_platter(name: str = "ZDownToPlatter") -> MoveToPositionAction:
        return MoveToPositionAction(
            name=name,
            node=node,
            z_mm=turntable["z_platter_mm"],
            skip_x=True,
            skip_a=True,
        )

    def _z_up_to_clear(name: str = "ZUpToClear") -> MoveToPositionAction:
        return MoveToPositionAction(
            name=name,
            node=node,
            z_mm=turntable["z_approach_mm"],
            skip_x=True,
            skip_a=True,
        )

    def _z_up_to_safe(name: str = "ZUpToSafe") -> MoveToPositionAction:
        """Raise Z to full safe clearance (safe_park height). Only safe when X > x_clear_mm."""
        return MoveToPositionAction(
            name=name,
            node=node,
            z_mm=safe_park["z_mm"],
            skip_x=True,
            skip_a=True,
        )

    # ── Stack entry/exit helpers ───────────────────────────────────────────────

    _x_clear_mm = float(config.get("homing", {}).get("record_stack", {}).get("x_clear_mm", 250.0))

    def _stack_retract_x(name: str = "StackRetractX") -> MoveToPositionAction:
        """Retract X to safe clear position outside stack. Z and A are not moved."""
        return MoveToPositionAction(name, node=node, x_mm=_x_clear_mm, skip_z=True, skip_a=True)

    def _stack_approach_x(name: str = "StackApproachX") -> MoveToPositionAction:
        """Enter stack at current Z (must already be at slot_top_z). A is not moved."""
        return MoveToPositionAction(name, node=node, x_mm=stack_x, skip_z=True, skip_a=True)

    # ── Transit helpers ───────────────────────────────────────────────────────

    tran_cfg = config.get("positions", {}).get("transit", {})
    stack_x = pos["queue_stack"]["x_mm"]

    traj_cfg = config.get("trajectories", {})
    _blend_mm_default = float(traj_cfg.get("blend_radius_mm", 20.0))
    _blend_deg_default = float(traj_cfg.get("blend_radius_deg", 15.0))

    _RAMP_MAP = {"position": 0, "vel_pos": 1, "vel_neg": 2}

    def _normalise_waypoints(raw: list) -> list:
        """Convert YAML waypoint dicts: resolve string ramp_mode, drop 'dynamic' z placeholders."""
        result = []
        for wp in raw:
            wp = dict(wp)
            rm = wp.get("ramp_mode", 0)
            if isinstance(rm, str):
                wp["ramp_mode"] = _RAMP_MAP.get(rm, 0)
            if wp.get("z_mm") == "dynamic":
                wp["z_mm"] = 0.0  # placeholder; DynamicTransitToSlotAction fills at runtime
            result.append(wp)
        return result

    def _load_trajectory(node_name: str, traj_name: str = None) -> ExecuteTrajectoryAction:
        """Load a static trajectory definition from config['trajectories'][traj_name]."""
        key = traj_name or node_name
        tdef = traj_cfg.get(key, {})
        blend_mm = float(tdef.get("blend_radius_mm", _blend_mm_default))
        blend_deg = float(tdef.get("blend_radius_deg", _blend_deg_default))
        default_scale = float(tdef.get("default_velocity_scale",
                                       traj_cfg.get("default_velocity_scale", 1.0)))
        return ExecuteTrajectoryAction(
            name=node_name,
            node=node,
            waypoints=_normalise_waypoints(tdef.get("waypoints", [])),
            blend_radius_mm=blend_mm,
            blend_radius_deg=blend_deg,
            default_velocity_scale=default_scale,
            trajectory_name=key,
        )

    def _transit_to_turntable(name: str) -> ExecuteTrajectoryAction:
        """Stack→Turntable: velocity-mode X with concurrent Z descent; A rotates at trailing split."""
        return _load_trajectory(name, "transit_to_turntable")

    def _transit_to_stack(name: str) -> ExecuteTrajectoryAction:
        """Turntable→Stack: velocity-mode X with concurrent Z rise; A rotates at trailing split."""
        return _load_trajectory(name, "transit_to_stack")

    # ── Homing subtree ────────────────────────────────────────────────────────

    do_homing = py_trees.composites.Sequence(
        name="DoHoming",
        memory=True,
        children=[
            PublishLEDAction("HOMING", node._led_pub),
            HomeAllAction(node),
            ClearForceRehomeAction(),  # clears force_rehome only after success
            PublishLEDAction("OPERATIONAL", node._led_pub),
        ],
    )

    homing_subtree = py_trees.composites.Selector(
        name="HomingSubtree",
        memory=False,
        children=[
            IsAllHomed(),
            do_homing,
        ],
    )

    # ── Initial load subtree ──────────────────────────────────────────────────
    # Runs exactly once after homing to pick up slot 0 and place it on the
    # player so the main operational loop always starts with a record playing.
    # The IsInitialLoaded gate prevents re-entry during normal operation when
    # player_has_record is transiently False while the arm is in transit.

    initial_load = py_trees.composites.Sequence(
        name="InitialLoad",
        memory=True,
        children=[
            # Wait for operator to press "Start Playing" before touching any disc
            WaitForStartCommand(),
            # ── Approach stack safely ──────────────────────────────────────────
            # 1. Position outside stack at safe height with correct A angle.
            MoveToPositionAction(
                "InitialLoad_Approach",
                node=node,
                x_mm=_x_clear_mm,
                z_mm=safe_park["z_mm"],
                a_deg=pos["queue_stack"]["a_deg"],
            ),
            # 2. Lower Z to slot TOP envelope (X is at x_clear_mm — safe to move Z freely).
            DynamicMoveToSlotAction(
                name="InitialLoad_ZToSlotTop",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
                slot_top=True,
            ),
            # 3. Enter stack at slot top Z (A stays, Z stays).
            _stack_approach_x("InitialLoad_ApproachX"),
            # ── Inside slot: descend → grip → ascend ──────────────────────────
            # 4. Descend to grip height (bottom of slot envelope).
            DynamicMoveToSlotAction(
                name="InitialLoad_ZToSlot",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
            ),
            # 5. Pick up (with retry).
            py_trees.decorators.Retry(
                child=GripAction(node, close=True, name="InitialLoad_GripClose"),
                num_failures=config.get("grip", {}).get("retry_max", 3),
                name="InitialLoad_GripRetry",
            ),
            # 6. Raise back to top of slot envelope before retracting X.
            DynamicMoveToSlotAction(
                name="InitialLoad_ZToSlotTop2",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
                slot_top=True,
            ),
            # ── Exit stack ────────────────────────────────────────────────────
            # 7. Retract X to safe position (Z now free to move past shelf boundaries).
            _stack_retract_x("InitialLoad_RetractX"),
            # ── Transit to turntable ──────────────────────────────────────────
            _transit_to_turntable("InitialLoad_TransitToPlayer"),
            # Place on player
            _z_down_to_platter("InitialLoad_ZDown"),
            GripAction(node, close=False, name="InitialLoad_GripOpen"),
            SetPlayerStateAction(has_record=True, name="InitialLoad_MarkPlayerFull"),
            _z_up_to_clear("InitialLoad_ZUpClear"),
            # Start playing
            PressPlayAction(node, press=True, name="InitialLoad_PressPlay"),
            SetSpeedAction(node, rpm=33.0, name="InitialLoad_SetSpeed"),
            ResetProgressAction("InitialLoad_ResetProgress"),
            PublishLEDAction("PLAYING", node._led_pub),
            # Mark done so this subtree never runs again this session
            MarkInitialLoadedAction(),
        ],
    )

    maybe_initial_load = py_trees.composites.Selector(
        name="MaybeInitialLoad",
        memory=False,
        children=[
            IsInitialLoaded(),  # gate: SUCCESS once done, FAILURE before
            initial_load,
        ],
    )

    # ── Lift record subtree ───────────────────────────────────────────────────

    grip_close_with_retry = py_trees.decorators.Retry(
        child=GripAction(node, close=True, name="GripClose"),
        num_failures=config.get("grip", {}).get("retry_max", 3),
        name="GripCloseRetry",
    )

    lift_record = py_trees.composites.Sequence(
        name="LiftRecord",
        memory=True,
        children=[
            _z_down_to_platter("LiftRecord_ZDown"),
            grip_close_with_retry,
            _z_up_to_clear("LiftRecord_ZUp"),
        ],
    )

    # ── Flip subtree ──────────────────────────────────────────────────────────
    # Sequential moves: swing A away → back X → raise Z → check clearance → flip

    def _ensure_flip_clearance(suffix: str = "") -> py_trees.composites.Selector:
        """Build an EnsureFlipClearance selector.  Call twice to get two independent instances."""
        return py_trees.composites.Selector(
            name=f"EnsureFlipClearance{suffix}",
            memory=False,
            children=[
                FlipClearanceCheck(clearance_min_mm, name=f"FlipClearanceCheck{suffix}"),
                py_trees.composites.Sequence(
                    name=f"RaiseForClearance{suffix}",
                    memory=True,
                    children=[
                        _z_up_to_safe(f"RaiseForClearance_ZUp{suffix}"),
                        FlipClearanceCheck(clearance_min_mm, name=f"FlipClearanceCheck2{suffix}"),
                    ],
                ),
            ],
        )

    def _flip_staging_steps(prefix: str) -> ExecuteTrajectoryAction:
        """
        Single trajectory: A swings + Z rises simultaneously (WP0), then X retracts (WP1).
        Returns one ExecuteTrajectoryAction node — callers use it directly, no unpacking.
        """
        return _load_trajectory(f"{prefix}_FlipStage", "flip_staging")

    flip_subtree = py_trees.composites.Sequence(
        name="FlipSubtree",
        memory=True,
        children=[
            IsFlipAction(),
            # Steps 1-2: concurrent A+Z, then X retract (2-waypoint trajectory)
            _flip_staging_steps("FlipStage"),
            # Step 4: verify clearance below arm
            _ensure_flip_clearance(),
            # Step 5: run flip servos
            FlipRecordAction(node),
            # Step 5b: apply the side change now that the record is physically flipped —
            # this is what drives the 3D model colour update in the web UI.
            CommitFlipSideAction(),
            # Step 6: return directly to player approach (arm is at X=700mm, no transit needed)
            _move_to_player_approach("FlipReturnToPlayer"),
        ],
    )

    # ── Flip-then-swap subtree ────────────────────────────────────────────────
    # Used when a B-side record finishes in SEQUENTIAL mode.  The arm flips the
    # record back to A at the flip staging area, then goes straight to the stack
    # to deposit it and pick up the next record — no wasted trip back to the player.

    flip_then_swap_subtree = py_trees.composites.Sequence(
        name="FlipThenSwapSubtree",
        memory=True,
        children=[
            IsFlipThenSwapAction(),
            # Steps 1-2: concurrent A+Z then X retract (single trajectory)
            _flip_staging_steps("FTS_FlipStage"),
            # Step 3: clearance check
            _ensure_flip_clearance("_FTS"),
            # Step 4: run flip servos — record is now A-side up
            FlipRecordAction(node, name="FTS_FlipRecordAction"),
            # Step 4b: apply side change (current_side → "A")
            CommitFlipSideAction(name="FTS_CommitFlipSide"),
            # Step 5: transit to stack — arrives at X=stack_x, Z=slot_top_z[prev], A=arrive_a.
            DynamicTransitToSlotAction(
                name="FTS_TransitToDeposit",
                node=node,
                config=config,
                idx_key=K.PREV_RECORD_IDX,
            ),
            # ── Deposit old record ────────────────────────────────────────────
            DynamicMoveToSlotAction(
                name="FTS_ZDownToDeposit",
                node=node,
                config=config,
                idx_key=K.PREV_RECORD_IDX,
            ),
            GripAction(node, close=False, name="FTS_GripOpen"),
            DynamicMoveToSlotAction(
                name="FTS_ZUpAfterDeposit",
                node=node,
                config=config,
                idx_key=K.PREV_RECORD_IDX,
                slot_top=True,
            ),
            _stack_retract_x("FTS_RetractX"),
            # ── Reposition for new slot ───────────────────────────────────────
            DynamicMoveToSlotAction(
                name="FTS_ZToNewSlotTop",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
                slot_top=True,
            ),
            _stack_approach_x("FTS_ApproachX"),
            # ── Pick up new record ────────────────────────────────────────────
            DynamicMoveToSlotAction(
                name="FTS_ZDownToPickup",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
            ),
            py_trees.decorators.Retry(
                child=GripAction(node, close=True, name="FTS_GripClose"),
                num_failures=config.get("grip", {}).get("retry_max", 3),
                name="FTS_GripCloseRetry",
            ),
            # Update current_side to new record's side now that it's in the gripper.
            SetCurrentSideFromSlotAction(name="FTS_SetCurrentSide"),
            DynamicMoveToSlotAction(
                name="FTS_ZUpAfterPickup",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
                slot_top=True,
            ),
            _stack_retract_x("FTS_RetractAfterPickup"),
            # ── Return to turntable ───────────────────────────────────────────
            _transit_to_turntable("FTS_ReturnToPlayer"),
        ],
    )

    # ── Swap subtree ──────────────────────────────────────────────────────────
    # Place old record back in its slot, then pick up the new record.

    swap_subtree = py_trees.composites.Sequence(
        name="SwapSubtree",
        memory=True,
        children=[
            IsSwapAction(),
            # ── Arrive at old slot (top of envelope) ──────────────────────────
            # Transit: X vel-, Z→safe_z, A→arrive_a (all concurrent, A done before X<250mm).
            # Arrives: X=stack_x, Z=slot_top_z[prev], A=arrive_a.
            DynamicTransitToSlotAction(
                name="Swap_TransitToDeposit",
                node=node,
                config=config,
                idx_key=K.PREV_RECORD_IDX,
            ),
            # ── Deposit old record ────────────────────────────────────────────
            # Descend to grip/release height (bottom of slot envelope).
            DynamicMoveToSlotAction(
                name="Swap_ZDownToDeposit",
                node=node,
                config=config,
                idx_key=K.PREV_RECORD_IDX,
            ),
            GripAction(node, close=False, name="Swap_GripOpen"),
            # Ascend to top of slot envelope before retracting X.
            DynamicMoveToSlotAction(
                name="Swap_ZUpAfterDeposit",
                node=node,
                config=config,
                idx_key=K.PREV_RECORD_IDX,
                slot_top=True,
            ),
            # Retract X — Z is now free to move past shelf boundaries.
            _stack_retract_x("Swap_RetractX"),
            # ── Reposition for new slot ───────────────────────────────────────
            # Set Z to new slot's top envelope (X at x_clear_mm, safe).
            DynamicMoveToSlotAction(
                name="Swap_ZToNewSlotTop",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
                slot_top=True,
            ),
            # Enter new slot at top of envelope.
            _stack_approach_x("Swap_ApproachX"),
            # ── Pick up new record ────────────────────────────────────────────
            # Descend to grip height.
            DynamicMoveToSlotAction(
                name="Swap_ZDownToPickup",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
            ),
            py_trees.decorators.Retry(
                child=GripAction(node, close=True, name="Swap_GripClose"),
                num_failures=config.get("grip", {}).get("retry_max", 3),
                name="Swap_GripCloseRetry",
            ),
            # Ascend to top of slot envelope before retracting.
            DynamicMoveToSlotAction(
                name="Swap_ZUpAfterPickup",
                node=node,
                config=config,
                idx_key=K.CURRENT_RECORD_IDX,
                slot_top=True,
            ),
            # Retract X — ready for transit.
            _stack_retract_x("Swap_RetractAfterPickup"),
            # ── Return to turntable ───────────────────────────────────────────
            _transit_to_turntable("Swap_ReturnToPlayer"),
        ],
    )

    # ── Flip-or-swap selector ─────────────────────────────────────────────────

    flip_or_swap = py_trees.composites.Selector(
        name="FlipOrSwap",
        memory=False,
        children=[flip_subtree, flip_then_swap_subtree, swap_subtree],
    )

    # ── Place record subtree ──────────────────────────────────────────────────

    # Both flip_subtree and swap_subtree leave the arm at player approach position,
    # so place_record only needs to descend, release, and raise.
    place_record = py_trees.composites.Sequence(
        name="PlaceRecord",
        memory=True,
        children=[
            _z_down_to_platter("PlaceRecord_ZDown"),
            GripAction(node, close=False, name="PlaceRecord_GripOpen"),
            SetPlayerStateAction(has_record=True, name="MarkPlayerFull"),
            _z_up_to_clear("PlaceRecord_ZUp"),
        ],
    )

    # ── One full record-handling cycle ────────────────────────────────────────

    one_cycle = py_trees.composites.Sequence(
        name="OneCycle",
        memory=True,
        children=[
            WaitForRecordFinished(progress_threshold),
            PressPlayAction(node, press=False, name="PressStop"),
            _move_to_player_approach("Lift_Approach"),
            lift_record,
            SetPlayerStateAction(has_record=False, name="MarkPlayerEmpty"),
            DecideAction(),
            flip_or_swap,
            place_record,
            PressPlayAction(node, press=True, name="PressPlay"),
            SetSpeedAction(node, rpm=33.0),
            ResetProgressAction(),
            PublishLEDAction("PLAYING", node._led_pub),
        ],
    )

    # ── Operational loop (eternal) ────────────────────────────────────────────

    operational_or_halt = py_trees.composites.Selector(
        name="OperationalOrHalt",
        memory=False,
        children=[
            IsHaltRequested(),
            LoopDecorator(child=one_cycle, name="OperationalLoop"),
        ],
    )

    # ── Main flow (resumes after safety clears) ───────────────────────────────

    main_flow = py_trees.composites.Sequence(
        name="MainFlow",
        memory=False,
        children=[
            homing_subtree,
            maybe_initial_load,
            operational_or_halt,
        ],
    )

    # ── Reactive root (SafetyCheck re-evaluated every tick) ───────────────────

    reactive_root = py_trees.composites.Sequence(
        name="ReactiveRoot",
        memory=False,
        children=[
            SafetyCheck(),
            main_flow,
        ],
    )

    # ── Root selector (failure → fault recovery) ──────────────────────────────

    root = py_trees.composites.Selector(
        name="RootSelector",
        memory=False,
        children=[
            reactive_root,
            build_recovery_subtree(node, config),
        ],
    )

    return root
