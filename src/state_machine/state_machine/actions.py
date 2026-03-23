"""
actions.py — Action behaviors for the vinyl robot state machine.

Uses py_trees_ros.action_clients.FromConstant for all fixed-goal actions.
For DynamicMoveToSlotAction (goal depends on blackboard at runtime), we
subclass FromBlackboard directly and write the goal into the blackboard
in initialise() — FromBlackboard.initialise() reads self.blackboard.goal
which is remapped to our controlled key.
"""

import uuid as _uuid

import py_trees
import py_trees_ros.action_clients
from rclpy.node import Node
from std_msgs.msg import String

from vinyl_robot_msgs.action import (
    HomeAll,
    MoveToPosition,
    Grip,
    FlipRecord,
    PressPlay,
    SetSpeed,
)

from . import blackboard_keys as K

_NS = "/rdj"

_FromConstant = py_trees_ros.action_clients.FromConstant


# ── Non-motion behaviors ───────────────────────────────────────────────────────

class PublishLEDAction(py_trees.behaviour.Behaviour):
    """Fire-and-forget publish to /led/pattern. Returns SUCCESS immediately."""

    def __init__(self, pattern: str, node: Node, name: str = ""):
        super().__init__(name or f"LED:{pattern}")
        self._pattern = pattern
        self._pub = node.create_publisher(String, "/led/pattern", 10)

    def update(self) -> py_trees.common.Status:
        msg = String()
        msg.data = self._pattern
        self._pub.publish(msg)
        return py_trees.common.Status.SUCCESS


class DecideAction(py_trees.behaviour.Behaviour):
    """
    Pure logic — reads blackboard state, writes NEXT_ACTION and updates
    CURRENT_SIDE / record indices.  Always returns SUCCESS.

    Decision matrix:
      side A  → flip to side B
      side B, SIDE_REPEAT or SINGLE_REPEAT → flip back to side A
      side B, SEQUENTIAL, records remain   → swap to next record
      side B, SEQUENTIAL, last record      → halt
    """

    def __init__(self, name: str = "DecideAction"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.CURRENT_SIDE,       access=py_trees.common.Access.READ)
        self._bb.register_key(K.PLAY_MODE,          access=py_trees.common.Access.READ)
        self._bb.register_key(K.CURRENT_RECORD_IDX, access=py_trees.common.Access.READ)
        self._bb.register_key(K.QUEUE_SIZE,         access=py_trees.common.Access.READ)
        self._bb.register_key(K.NEXT_ACTION,        access=py_trees.common.Access.WRITE)
        self._bb.register_key(K.CURRENT_SIDE,       access=py_trees.common.Access.WRITE)
        self._bb.register_key(K.CURRENT_RECORD_IDX, access=py_trees.common.Access.WRITE)
        self._bb.register_key(K.PREV_RECORD_IDX,    access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        side = self._bb.current_side
        mode = self._bb.play_mode
        idx  = self._bb.current_record_index
        size = self._bb.queue_size

        if side == "A":
            self._bb.next_action  = "flip"
            self._bb.current_side = "B"
            self.feedback_message = "Flip to side B"
            return py_trees.common.Status.SUCCESS

        # side == "B"
        if mode in ("SIDE_REPEAT", "SINGLE_REPEAT"):
            self._bb.next_action  = "flip"
            self._bb.current_side = "A"
            self.feedback_message = f"Flip back to side A ({mode})"
            return py_trees.common.Status.SUCCESS

        # SEQUENTIAL
        next_idx = idx + 1
        if next_idx < size:
            self._bb.prev_record_index    = idx
            self._bb.current_record_index = next_idx
            self._bb.next_action          = "swap"
            self._bb.current_side         = "A"
            self.feedback_message         = f"Swap: slot {idx} → {next_idx}"
        else:
            self._bb.next_action  = "halt"
            self.feedback_message = "All records played — halting"

        return py_trees.common.Status.SUCCESS


class ClearForceRehomeAction(py_trees.behaviour.Behaviour):
    """
    Clears the force_rehome blackboard flag after HomeAllAction succeeds.
    Placed at the end of DoHoming so the flag is only cleared once the tree
    has confirmed homing completed — not from a /motion/status callback that
    could fire before homing actually starts.
    """

    def __init__(self, name: str = "ClearForceRehome"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.FORCE_REHOME, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self._bb.force_rehome = False
        return py_trees.common.Status.SUCCESS


class WaitForManualClear(py_trees.behaviour.Behaviour):
    """Holds RUNNING until the operator has re-homed after a fault."""

    def __init__(self, name: str = "WaitForManualClear"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.ALL_HOMED,    access=py_trees.common.Access.READ)
        self._bb.register_key(K.MOTION_FAULT, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            homed = self._bb.all_homed
            fault = self._bb.motion_fault
        except KeyError:
            return py_trees.common.Status.RUNNING

        if homed and not fault:
            self.feedback_message = "Fault cleared and re-homed — resuming"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = (
            f"Waiting for manual fault clear and re-home "
            f"(homed={homed}, fault={fault})"
        )
        return py_trees.common.Status.RUNNING


# ── Loop decorator ─────────────────────────────────────────────────────────────

class LoopDecorator(py_trees.decorators.Decorator):
    """
    Restarts the child each time it returns SUCCESS, creating an eternal loop.
    Propagates RUNNING while running, FAILURE if the child fails.
    """

    def update(self) -> py_trees.common.Status:
        child_status = self.decorated.status

        if child_status == py_trees.common.Status.SUCCESS:
            self.decorated.stop(py_trees.common.Status.INVALID)
            return py_trees.common.Status.RUNNING

        if child_status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


# ── Motion action behaviors (all use FromConstant) ─────────────────────────────

class HomeAllAction(_FromConstant):
    def __init__(self, node: Node, name: str = "HomeAllAction"):
        super().__init__(
            name=name,
            action_type=HomeAll,
            action_name="/motion/home_all",
            action_goal=HomeAll.Goal(),
        )


class MoveToPositionAction(_FromConstant):
    def __init__(
        self,
        name: str,
        node: Node,
        x_mm: float = 0.0,
        z_mm: float = 0.0,
        a_deg: float = 0.0,
        skip_x: bool = False,
        skip_z: bool = False,
        skip_a: bool = False,
        velocity_scale: float = 1.0,
    ):
        goal = MoveToPosition.Goal()
        goal.x_mm           = float(x_mm)
        goal.z_mm           = float(z_mm)
        goal.a_deg          = float(a_deg)
        goal.velocity_scale = float(velocity_scale)
        goal.skip_x         = skip_x
        goal.skip_z         = skip_z
        goal.skip_a         = skip_a
        super().__init__(
            name=name,
            action_type=MoveToPosition,
            action_name="/motion/move_to_position",
            action_goal=goal,
        )


class DynamicMoveToSlotAction(py_trees_ros.action_clients.FromBlackboard):
    """
    Move to a queue stack slot whose Z is read from the blackboard at runtime.

    Subclasses FromBlackboard directly.  A unique blackboard key is created per
    instance; initialise() writes the freshly-built goal there before calling
    super().initialise() which reads self.blackboard.goal (remapped to our key).
    """

    def __init__(self, name: str, node: Node, config: dict, idx_key: str):
        # Unique absolute key — avoids collisions between the two instances
        # (Swap_ZToOldSlot and Swap_ZToNewSlot).
        self._goal_key = "/dyn_slot_goal_" + _uuid.uuid4().hex

        super().__init__(
            name=name,
            action_type=MoveToPosition,
            action_name="/motion/move_to_position",
            key=self._goal_key,   # FromBlackboard registers "goal" → this key (READ)
        )

        # Register WRITE access on the same key so initialise() can update it.
        self.blackboard.register_key(
            key=self._goal_key,
            access=py_trees.common.Access.WRITE,
        )
        # Seed with a placeholder so the key exists on the blackboard.
        self.blackboard.set(name=self._goal_key, value=MoveToPosition.Goal())

        self._slot_cfg = config["positions"]["queue_stack"]
        self._idx_key  = idx_key
        self._slot_bb  = self.attach_blackboard_client(name=f"{name}_idx", namespace=_NS)
        self._slot_bb.register_key(idx_key, access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        try:
            idx = getattr(self._slot_bb, self._idx_key)
        except (KeyError, AttributeError):
            idx = 0

        slot_z_list = self._slot_cfg["slot_z_mm"]
        slot_z = slot_z_list[min(int(idx), len(slot_z_list) - 1)]

        goal = MoveToPosition.Goal()
        goal.x_mm           = float(self._slot_cfg["x_mm"])
        goal.z_mm           = float(slot_z)
        goal.a_deg          = float(self._slot_cfg["a_deg"])
        goal.velocity_scale = 1.0
        goal.skip_x         = False
        goal.skip_z         = False
        goal.skip_a         = False

        # Write goal into blackboard — FromBlackboard.initialise() reads it via "goal" remap.
        self.blackboard.set(name=self._goal_key, value=goal)
        super().initialise()


class GripAction(_FromConstant):
    def __init__(self, node: Node, close: bool, name: str = ""):
        goal = Grip.Goal()
        goal.close = close
        super().__init__(
            name=name or ("GripClose" if close else "GripOpen"),
            action_type=Grip,
            action_name="/motion/grip",
            action_goal=goal,
        )


class FlipRecordAction(_FromConstant):
    def __init__(self, node: Node, name: str = "FlipRecordAction"):
        super().__init__(
            name=name,
            action_type=FlipRecord,
            action_name="/motion/flip_record",
            action_goal=FlipRecord.Goal(),
        )


class PressPlayAction(_FromConstant):
    def __init__(self, node: Node, press: bool, name: str = ""):
        goal = PressPlay.Goal()
        goal.press = press
        super().__init__(
            name=name or ("PressPlay" if press else "PressStop"),
            action_type=PressPlay,
            action_name="/motion/press_play",
            action_goal=goal,
        )


class SetSpeedAction(_FromConstant):
    def __init__(self, node: Node, rpm: float = 33.0, name: str = ""):
        goal = SetSpeed.Goal()
        goal.rpm = float(rpm)
        super().__init__(
            name=name or f"SetSpeed{int(rpm)}RPM",
            action_type=SetSpeed,
            action_name="/motion/set_speed",
            action_goal=goal,
        )
