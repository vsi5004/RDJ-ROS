"""
recovery.py — Fault recovery subtree factory.

build_recovery_subtree() returns a Sequence that:
  1. Opens the gripper (idempotent — safe if already open)
  2. Raises Z to safe clearance
  3. Moves the arm to the safe park position
  4. Publishes FAULT_PARKED LED pattern
  5. Waits for the operator to manually re-home before resuming

This subtree runs as the second child of the root Selector, so it executes
whenever the ReactiveRoot sequence fails (safety interrupt, motion fault,
clearance failure, etc.).
"""

import py_trees
from rclpy.node import Node

from .actions import (
    GripAction,
    MoveToPositionAction,
    PublishLEDAction,
    WaitForManualClear,
)


def build_recovery_subtree(node: Node, config: dict) -> py_trees.behaviour.Behaviour:
    """
    Returns a Sequence(memory=True) that executes the fault recovery procedure.

    Args:
        node:   The rclpy Node (passed to all action behaviors for action clients).
        config: Parsed robot_params.yaml dict (used for safe_park coordinates).
    """
    park = config["positions"]["safe_park"]

    recovery = py_trees.composites.Sequence(
        name="FaultRecovery",
        memory=True,
        children=[
            # 1. Signal fault immediately
            PublishLEDAction("FAULT", node),

            # 2. Open gripper — always safe even if already open
            GripAction(node, close=False, name="RecoveryGripOpen"),

            # 3. Raise Z to safe height before moving X (avoid collision with stack shelves)
            MoveToPositionAction(
                name="RecoveryZToClear",
                node=node,
                z_mm=park["z_mm"],
                skip_x=True,
                skip_a=True,
            ),

            # 4. Move to safe park position (X + A together, Z already at clearance)
            MoveToPositionAction(
                name="RecoverySafePark",
                node=node,
                x_mm=park["x_mm"],
                z_mm=park["z_mm"],
                a_deg=park["a_deg"],
            ),

            # 5. Update LED to show parked-in-fault state
            PublishLEDAction("FAULT_PARKED", node),

            # 6. Hold until operator resolves fault and re-homes
            WaitForManualClear(),
        ],
    )

    return recovery
