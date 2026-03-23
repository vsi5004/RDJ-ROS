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
  │                   │   ├── FlipSubtree
  │                   │   └── SwapSubtree
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

from .actions import (
    ClearForceRehomeAction,
    DecideAction,
    DynamicMoveToSlotAction,
    FlipRecordAction,
    GripAction,
    HomeAllAction,
    LoopDecorator,
    MoveToPositionAction,
    PressPlayAction,
    PublishLEDAction,
    SetSpeedAction,
)
from .conditions import (
    FlipClearanceCheck,
    IsAllHomed,
    IsFlipAction,
    IsHaltRequested,
    IsSwapAction,
    SafetyCheck,
    WaitForRecordFinished,
)
from .recovery import build_recovery_subtree
from . import blackboard_keys as K


def build_tree(node: Node, config: dict) -> py_trees.behaviour.Behaviour:
    """
    Assemble and return the root behaviour.  Call BehaviourTree.setup() on
    the result to create all ROS action clients before ticking.
    """
    py_trees.blackboard.Blackboard.enable_activity_stream(100)

    pos     = config["positions"]
    turntable = pos["turntable"]
    flip_st   = pos["flip_staging"]
    safe_park = pos["safe_park"]
    progress_threshold = config.get("turntable_monitor", {}).get(
        "progress_threshold", 0.92
    )
    clearance_min_mm = config.get("flip", {}).get("clearance_min_mm", 178.0)

    # ── Reusable position shortcuts ───────────────────────────────────────────

    def _move_to_player_approach(name: str = "MoveToPlayerApproach") -> MoveToPositionAction:
        return MoveToPositionAction(
            name=name, node=node,
            x_mm=turntable["x_mm"],
            z_mm=turntable["z_approach_mm"],
            a_deg=turntable["a_deg"],
        )

    def _z_down_to_platter(name: str = "ZDownToPlatter") -> MoveToPositionAction:
        return MoveToPositionAction(
            name=name, node=node,
            z_mm=turntable["z_platter_mm"],
            skip_x=True, skip_a=True,
        )

    def _z_up_to_clear(name: str = "ZUpToClear") -> MoveToPositionAction:
        return MoveToPositionAction(
            name=name, node=node,
            z_mm=turntable["z_approach_mm"],
            skip_x=True, skip_a=True,
        )

    def _z_up_to_safe(name: str = "ZUpToSafe") -> MoveToPositionAction:
        """Raise Z to full safe clearance (safe_park height)."""
        return MoveToPositionAction(
            name=name, node=node,
            z_mm=safe_park["z_mm"],
            skip_x=True, skip_a=True,
        )

    # ── Transit helpers (record trailing safety) ──────────────────────────────
    # A=0° is perpendicular to the X rail (pointing backward).  During transit
    # the record trails the carriage: A stays at the source angle for the first
    # `trailing_split` fraction of the X move, then rotates to the destination
    # angle in the remaining travel so the record arrives correctly oriented.

    tran_cfg       = config.get("positions", {}).get("transit", {})
    trailing_split = tran_cfg.get("trailing_split", 0.70)
    stack_x        = pos["queue_stack"]["x_mm"]
    tt_x           = turntable["x_mm"]
    travel         = tt_x - stack_x

    def _transit_to_turntable(name: str) -> py_trees.composites.Sequence:
        """Stack→Turntable: A holds current angle for first 50%, rotates to 90° in last 50%."""
        waypoint_x   = tt_x - (1.0 - trailing_split) * travel
        arrive_a_deg = tran_cfg.get("to_turntable_arrive_deg", turntable["a_deg"])
        return py_trees.composites.Sequence(name=name, memory=True, children=[
            MoveToPositionAction(f"{name}_Trail", node=node,
                x_mm=waypoint_x, z_mm=turntable["z_approach_mm"],
                skip_a=True,
            ),
            MoveToPositionAction(f"{name}_Arrive", node=node,
                x_mm=tt_x, z_mm=turntable["z_approach_mm"],
                a_deg=arrive_a_deg,
                skip_z=True,
            ),
        ])

    def _transit_to_stack(name: str) -> py_trees.composites.Sequence:
        """Turntable→Stack: A holds current angle for first 50%, rotates to 270° in last 50%."""
        waypoint_x   = stack_x + (1.0 - trailing_split) * travel
        arrive_a_deg = tran_cfg.get("to_stack_arrive_deg", 270.0)
        return py_trees.composites.Sequence(name=name, memory=True, children=[
            MoveToPositionAction(f"{name}_Trail", node=node,
                x_mm=waypoint_x, z_mm=safe_park["z_mm"],
                skip_a=True,
            ),
            MoveToPositionAction(f"{name}_Arrive", node=node,
                x_mm=stack_x, z_mm=safe_park["z_mm"],
                a_deg=arrive_a_deg,
                skip_z=True,
            ),
        ])

    # ── Homing subtree ────────────────────────────────────────────────────────

    do_homing = py_trees.composites.Sequence(
        name="DoHoming",
        memory=True,
        children=[
            PublishLEDAction("HOMING", node),
            HomeAllAction(node),
            ClearForceRehomeAction(),        # clears force_rehome only after success
            PublishLEDAction("OPERATIONAL", node),
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

    ensure_flip_clearance = py_trees.composites.Selector(
        name="EnsureFlipClearance",
        memory=False,
        children=[
            FlipClearanceCheck(clearance_min_mm),
            py_trees.composites.Sequence(
                name="RaiseForClearance",
                memory=True,
                children=[
                    _z_up_to_safe("RaiseForClearance_ZUp"),
                    FlipClearanceCheck(clearance_min_mm, name="FlipClearanceCheck2"),
                ],
            ),
        ],
    )

    flip_subtree = py_trees.composites.Sequence(
        name="FlipSubtree",
        memory=True,
        children=[
            IsFlipAction(),
            # Step 1: swing A away from player
            MoveToPositionAction(
                name="FlipStage_SwingA", node=node,
                a_deg=flip_st["a_deg"],
                skip_x=True, skip_z=True,
            ),
            # Step 2: back X away from player
            MoveToPositionAction(
                name="FlipStage_BackX", node=node,
                x_mm=flip_st["x_mm"],
                skip_z=True, skip_a=True,
            ),
            # Step 3: raise Z for clearance
            MoveToPositionAction(
                name="FlipStage_RaiseZ", node=node,
                z_mm=flip_st["z_mm"],
                skip_x=True, skip_a=True,
            ),
            # Step 4: verify clearance below arm
            ensure_flip_clearance,
            # Step 5: run flip servos
            FlipRecordAction(node),
            # Step 6: return directly to player approach (arm is at X=700mm, no transit needed)
            _move_to_player_approach("FlipReturnToPlayer"),
        ],
    )

    # ── Swap subtree ──────────────────────────────────────────────────────────
    # Place old record back in its slot, then pick up the new record.

    swap_subtree = py_trees.composites.Sequence(
        name="SwapSubtree",
        memory=True,
        children=[
            IsSwapAction(),

            # Ensure Z clearance before moving X toward stack
            _z_up_to_safe("Swap_ZClear"),

            # Transit to queue stack — record trails toward turntable,
            # A rotates to arrival angle (270°) in the last 30% of X travel.
            _transit_to_stack("Swap_TransitToStack"),

            # Lower Z to old record's slot (PREV_RECORD_IDX)
            DynamicMoveToSlotAction(
                name="Swap_ZToOldSlot", node=node, config=config,
                idx_key=K.PREV_RECORD_IDX,
            ),

            # Deposit old record
            GripAction(node, close=False, name="Swap_GripOpen"),

            # Raise Z to clearance between slots
            _z_up_to_safe("Swap_ZClearAfterDeposit"),

            # Lower Z to new record's slot (CURRENT_RECORD_IDX, already incremented)
            DynamicMoveToSlotAction(
                name="Swap_ZToNewSlot", node=node, config=config,
                idx_key=K.CURRENT_RECORD_IDX,
            ),

            # Pick up new record (with retry)
            py_trees.decorators.Retry(
                child=GripAction(node, close=True, name="Swap_GripClose"),
                num_failures=config.get("grip", {}).get("retry_max", 3),
                name="Swap_GripCloseRetry",
            ),

            # Raise Z to full clearance before moving back to turntable
            _z_up_to_safe("Swap_ZClearWithRecord"),

            # Transit back to turntable — record trails toward stack,
            # A rotates to turntable angle (90°) in the last 50% of X travel.
            _transit_to_turntable("Swap_ReturnToPlayer"),
        ],
    )

    # ── Flip-or-swap selector ─────────────────────────────────────────────────

    flip_or_swap = py_trees.composites.Selector(
        name="FlipOrSwap",
        memory=False,
        children=[flip_subtree, swap_subtree],
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
            DecideAction(),
            flip_or_swap,
            place_record,
            PressPlayAction(node, press=True, name="PressPlay"),
            SetSpeedAction(node, rpm=33.0),
            PublishLEDAction("PLAYING", node),
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
