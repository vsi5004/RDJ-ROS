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
    ExecuteTrajectory,
    FlipRecord,
    Grip,
    HomeAll,
    MoveToPosition,
    PressPlay,
    SetSpeed,
)
from vinyl_robot_msgs.msg import Waypoint

from . import blackboard_keys as K

_NS = "/rdj"

_FromConstant = py_trees_ros.action_clients.FromConstant


# ── Non-motion behaviors ───────────────────────────────────────────────────────


class PublishLEDAction(py_trees.behaviour.Behaviour):
    """Fire-and-forget publish to /led/pattern. Returns SUCCESS immediately."""

    def __init__(self, pattern: str, led_pub, name: str = ""):
        super().__init__(name or f"LED:{pattern}")
        self._pattern = pattern
        self._pub = led_pub  # shared publisher from state_machine_node

    def update(self) -> py_trees.common.Status:
        msg = String()
        msg.data = self._pattern
        self._pub.publish(msg)
        return py_trees.common.Status.SUCCESS


class DecideAction(py_trees.behaviour.Behaviour):
    """
    Pure logic — reads blackboard state, writes NEXT_ACTION and updates
    CURRENT_SIDE / record indices.  Always returns SUCCESS.

    Decision priority (highest first):
      1. FORCE_FLIP set        → flip A↔B, clear flag
      2. OVERRIDE_SLOT >= 0    → swap to user-selected slot (any side),
                                  retrieving current record first; saves per-slot side
      3. side A                → flip to side B
      4. side B, SIDE_REPEAT or SINGLE_REPEAT → flip back to side A
      5. side B, SEQUENTIAL    → swap to (idx+1) % size  (wrap-around, never halt)
    """

    def __init__(self, name: str = "DecideAction"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        for key in [
            K.CURRENT_SIDE,
            K.PLAY_MODE,
            K.CURRENT_RECORD_IDX,
            K.QUEUE_SIZE,
            K.FORCE_FLIP,
            K.OVERRIDE_SLOT,
            K.SLOT_SIDES,
        ]:
            self._bb.register_key(key, access=py_trees.common.Access.READ)
        for key in [
            K.NEXT_ACTION,
            K.NEXT_FLIP_SIDE,
            K.CURRENT_SIDE,
            K.CURRENT_RECORD_IDX,
            K.PREV_RECORD_IDX,
            K.FORCE_FLIP,
            K.OVERRIDE_SLOT,
            K.SLOT_SIDES,
        ]:
            self._bb.register_key(key, access=py_trees.common.Access.WRITE)

    def _do_swap(self, from_idx: int, to_idx: int, current_side: str, slot_sides: list) -> None:
        """Write blackboard for a swap: deposit from_idx, pick up to_idx."""
        slot_sides = list(slot_sides)  # copy — avoid mutating in place
        slot_sides[from_idx] = current_side  # save the side the record was on
        self._bb.slot_sides = slot_sides
        self._bb.prev_record_index = from_idx
        self._bb.current_record_index = to_idx
        self._bb.current_side = slot_sides[to_idx]  # resume saved side
        self._bb.next_action = "swap"

    def update(self) -> py_trees.common.Status:
        side = self._bb.current_side
        mode = self._bb.play_mode
        idx = self._bb.current_record_index
        size = self._bb.queue_size
        force_flip = self._bb.force_flip
        override = self._bb.override_slot
        slot_sides = self._bb.slot_sides

        # ── Priority 1: manual flip request ───────────────────────────────────
        if force_flip:
            self._bb.force_flip = False
            new_side = "B" if side == "A" else "A"
            # Don't update current_side yet — CommitFlipSideAction applies it
            # after the physical servo motion so the web UI updates at the right time.
            self._bb.next_flip_side = new_side
            self._bb.next_action = "flip"
            self.feedback_message = f"Manual flip: side {side} → {new_side}"
            return py_trees.common.Status.SUCCESS

        # ── Priority 2: user slot override (any side) ─────────────────────────
        if override >= 0:
            self._bb.override_slot = -1  # consume
            if override != idx:
                self._do_swap(idx, override, side, slot_sides)
                self.feedback_message = f"Override swap: slot {idx} → {override}"
            else:
                # Same slot selected — re-flip; CommitFlipSideAction applies the side change.
                new_side = "B" if side == "A" else "A"
                self._bb.next_flip_side = new_side
                self._bb.next_action = "flip"
                self.feedback_message = f"Override: same slot {idx}, flip side"
            return py_trees.common.Status.SUCCESS

        # ── Priority 3: side A → flip to B ────────────────────────────────────
        if side == "A":
            self._bb.next_flip_side = "B"
            self._bb.next_action = "flip"
            self.feedback_message = "Flip to side B"
            return py_trees.common.Status.SUCCESS

        # ── Priority 4: side B repeat modes ───────────────────────────────────
        if mode in ("SIDE_REPEAT", "SINGLE_REPEAT"):
            self._bb.next_flip_side = "A"
            self._bb.next_action = "flip"
            self.feedback_message = f"Flip back to side A ({mode})"
            return py_trees.common.Status.SUCCESS

        # ── Priority 5: SEQUENTIAL B-side → flip back to A, then swap ─────────
        # The record has finished its B side.  Flip it back to A at the flip
        # staging area before depositing it in the stack, then pick the next record.
        next_idx = (idx + 1) % size
        slot_sides = list(slot_sides)
        slot_sides[idx] = "A"  # record will be deposited A-side up after the flip
        self._bb.slot_sides = slot_sides
        self._bb.prev_record_index = idx
        self._bb.current_record_index = next_idx
        self._bb.next_flip_side = "A"  # CommitFlipSideAction applies this after servo flip
        self._bb.next_action = "flip_swap"
        self.feedback_message = f"FlipSwap: flip slot {idx} to A, swap to slot {next_idx}"
        return py_trees.common.Status.SUCCESS


class CommitFlipSideAction(py_trees.behaviour.Behaviour):
    """
    Applies the pending flip side to CURRENT_SIDE.

    Placed AFTER FlipRecordAction in the flip subtree so the web UI 3D model
    only updates once the servo has physically turned the record over, not at
    the moment DecideAction decides to flip.
    Always returns SUCCESS.
    """

    def __init__(self, name: str = "CommitFlipSide"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.NEXT_FLIP_SIDE, access=py_trees.common.Access.READ)
        self._bb.register_key(K.CURRENT_SIDE, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self._bb.current_side = self._bb.next_flip_side
        return py_trees.common.Status.SUCCESS


class SetCurrentSideFromSlotAction(py_trees.behaviour.Behaviour):
    """
    Updates CURRENT_SIDE from SLOT_SIDES[CURRENT_RECORD_IDX].

    Placed after the new record is gripped in the FlipThenSwap subtree so
    the web UI reflects the newly held record's side rather than the "A" that
    CommitFlipSideAction wrote for the old record that was just deposited.
    Always returns SUCCESS.
    """

    def __init__(self, name: str = "SetCurrentSideFromSlot"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.SLOT_SIDES, access=py_trees.common.Access.READ)
        self._bb.register_key(K.CURRENT_RECORD_IDX, access=py_trees.common.Access.READ)
        self._bb.register_key(K.CURRENT_SIDE, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        sides = self._bb.slot_sides
        idx = self._bb.current_record_index
        self._bb.current_side = sides[idx]
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


class ResetProgressAction(py_trees.behaviour.Behaviour):
    """
    Writes 0.0 to PLAYBACK_PROGRESS immediately after a record starts playing.
    Prevents the stale progress value from the previous side/record from
    causing WaitForRecordFinished to fire instantly at the start of the next
    cycle — which would otherwise trigger an unintended immediate flip or swap.
    Always returns SUCCESS.
    """

    def __init__(self, name: str = "ResetProgress"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.PLAYBACK_PROGRESS, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self._bb.playback_progress = 0.0
        return py_trees.common.Status.SUCCESS


class MarkInitialLoadedAction(py_trees.behaviour.Behaviour):
    """
    Sets INITIAL_LOADED=True permanently.  Placed at the end of the
    InitialLoad sequence so the MaybeInitialLoad gate never fires again.
    Always returns SUCCESS.
    """

    def __init__(self, name: str = "MarkInitialLoaded"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.INITIAL_LOADED, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self._bb.initial_loaded = True
        return py_trees.common.Status.SUCCESS


class SetPlayerStateAction(py_trees.behaviour.Behaviour):
    """
    Pure blackboard write — marks whether a record is on the turntable.
    Inserted into the tree after LiftRecord (has_record=False) and after
    PlaceRecord's GripOpen (has_record=True) so the node can validate
    the Flip Now command and the web UI can show player occupancy.
    Always returns SUCCESS.
    """

    def __init__(self, has_record: bool, name: str = ""):
        super().__init__(name or ("MarkPlayerFull" if has_record else "MarkPlayerEmpty"))
        self._has_record = has_record
        self._bb = self.attach_blackboard_client(name=self.name, namespace=_NS)
        self._bb.register_key(K.PLAYER_HAS_RECORD, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self._bb.player_has_record = self._has_record
        return py_trees.common.Status.SUCCESS


class WaitForManualClear(py_trees.behaviour.Behaviour):
    """Holds RUNNING until the operator has re-homed after a fault."""

    def __init__(self, name: str = "WaitForManualClear"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.ALL_HOMED, access=py_trees.common.Access.READ)
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
            f"Waiting for manual fault clear and re-home (homed={homed}, fault={fault})"
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
        goal.x_mm = float(x_mm)
        goal.z_mm = float(z_mm)
        goal.a_deg = float(a_deg)
        goal.velocity_scale = float(velocity_scale)
        goal.skip_x = skip_x
        goal.skip_z = skip_z
        goal.skip_a = skip_a
        super().__init__(
            name=name,
            action_type=MoveToPosition,
            action_name="/motion/move_to_position",
            action_goal=goal,
        )


class DynamicMoveToSlotAction(py_trees_ros.action_clients.FromBlackboard):
    """
    Move Z to a queue stack slot height read from the blackboard at runtime.

    Z-only: skip_x=True, skip_a=True. X and A must be pre-positioned by the
    caller before this action runs (use _stack_approach_x / _stack_retract_x).

    slot_top=False  → targets slot_z[idx]                   (grip/release height)
    slot_top=True   → targets slot_z[idx] + shelf_clearance  (entry/exit height)
    """

    def __init__(self, name: str, node: Node, config: dict, idx_key: str,
                 slot_top: bool = False):
        self._goal_key = "/dyn_slot_goal_" + _uuid.uuid4().hex

        super().__init__(
            name=name,
            action_type=MoveToPosition,
            action_name="/motion/move_to_position",
            key=self._goal_key,
        )

        self.blackboard.register_key(
            key=self._goal_key,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.set(name=self._goal_key, value=MoveToPosition.Goal())

        self._slot_cfg = config["positions"]["queue_stack"]
        self._shelf_clearance = float(
            config.get("homing", {}).get("record_stack", {}).get("shelf_clearance_mm", 20.0)
        )
        self._slot_top = slot_top
        self._idx_key = idx_key
        self._slot_bb = self.attach_blackboard_client(name=f"{name}_idx", namespace=_NS)
        self._slot_bb.register_key(idx_key, access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        try:
            idx = getattr(self._slot_bb, self._idx_key)
        except (KeyError, AttributeError):
            idx = 0

        slot_z_list = self._slot_cfg["slot_z_mm"]
        slot_z = float(slot_z_list[min(int(idx), len(slot_z_list) - 1)])
        target_z = slot_z + self._shelf_clearance if self._slot_top else slot_z

        goal = MoveToPosition.Goal()
        goal.z_mm = target_z
        goal.velocity_scale = 1.0
        goal.skip_x = True
        goal.skip_z = False
        goal.skip_a = True

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


class ExecuteTrajectoryAction(_FromConstant):
    """
    Execute a named multi-waypoint trajectory.

    waypoints: list of dicts with keys matching Waypoint.msg fields.
    ramp_mode values accept 0/1/2 or strings 'position'/'vel_pos'/'vel_neg'.
    """

    _RAMP_MODE_MAP = {"position": 0, "vel_pos": 1, "vel_neg": 2}

    def __init__(
        self,
        name: str,
        node: Node,
        waypoints: list,
        blend_radius_mm: float,
        blend_radius_deg: float,
        default_velocity_scale: float = 1.0,
        trajectory_name: str = "",
    ):
        goal = ExecuteTrajectory.Goal()
        goal.default_velocity_scale = float(default_velocity_scale)
        goal.blend_radius_mm = float(blend_radius_mm)
        goal.blend_radius_deg = float(blend_radius_deg)
        goal.trajectory_name = trajectory_name or name
        goal.waypoints = []
        for wp_dict in waypoints:
            wp = Waypoint()
            wp.x_mm = float(wp_dict.get("x_mm", 0.0))
            wp.z_mm = float(wp_dict.get("z_mm", 0.0))
            wp.a_deg = float(wp_dict.get("a_deg", 0.0))
            wp.skip_x = bool(wp_dict.get("skip_x", False))
            wp.skip_z = bool(wp_dict.get("skip_z", False))
            wp.skip_a = bool(wp_dict.get("skip_a", False))
            wp.velocity_scale = float(wp_dict.get("velocity_scale", 0.0))
            rm = wp_dict.get("ramp_mode", 0)
            wp.ramp_mode = self._RAMP_MODE_MAP.get(rm, int(rm)) if isinstance(rm, str) else int(rm)
            wp.dmax_factor = int(wp_dict.get("dmax_factor", 0))
            goal.waypoints.append(wp)
        super().__init__(
            name=name,
            action_type=ExecuteTrajectory,
            action_name="/motion/execute_trajectory",
            action_goal=goal,
        )


class DynamicTransitToSlotAction(py_trees_ros.action_clients.FromBlackboard):
    """
    Transit from current position to a stack slot with concurrent Z pre-descent.

    Builds a 3-waypoint ExecuteTrajectory goal at runtime:
      WP0: X vel- toward stack, Z rises to safe height
      WP1: X vel- continues, Z descends to slot height
      WP2: X position final stop, A rotates to arrive angle
    The slot Z is read from the blackboard (idx_key) at initialise() time.
    """

    def __init__(self, name: str, node: Node, config: dict, idx_key: str):
        self._goal_key = "/dyn_transit_goal_" + _uuid.uuid4().hex
        super().__init__(
            name=name,
            action_type=ExecuteTrajectory,
            action_name="/motion/execute_trajectory",
            key=self._goal_key,
        )
        self.blackboard.register_key(self._goal_key, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name=self._goal_key, value=ExecuteTrajectory.Goal())

        self._slot_cfg = config["positions"]["queue_stack"]
        self._safe_z = float(config["positions"]["safe_park"]["z_mm"])
        self._traj_cfg = config.get("trajectories", {})
        self._shelf_clearance = float(
            config.get("homing", {}).get("record_stack", {}).get("shelf_clearance_mm", 20.0)
        )
        self._idx_key = idx_key
        self._slot_bb = self.attach_blackboard_client(name=f"{name}_idx", namespace=_NS)
        self._slot_bb.register_key(idx_key, access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        try:
            idx = getattr(self._slot_bb, self._idx_key)
        except (KeyError, AttributeError):
            idx = 0

        slot_z_list = self._slot_cfg["slot_z_mm"]
        slot_z = float(slot_z_list[min(int(idx), len(slot_z_list) - 1)])
        slot_top_z = slot_z + self._shelf_clearance
        stack_x = float(self._slot_cfg["x_mm"])
        arrive_a = float(self._slot_cfg.get("a_deg", 270.0))
        blend_mm = float(self._traj_cfg.get("blend_radius_mm", 20.0))
        blend_deg = float(self._traj_cfg.get("blend_radius_deg", 15.0))
        slot_cfg = self._traj_cfg.get("transit_to_slot", {})
        pre_x_offset = float(slot_cfg.get("stack_z_pre_descent_x_mm", 400.0))
        pre_x = stack_x + pre_x_offset

        goal = ExecuteTrajectory.Goal()
        goal.blend_radius_mm = blend_mm
        goal.blend_radius_deg = blend_deg
        goal.trajectory_name = self.name
        goal.default_velocity_scale = 1.0

        def _wp(**kwargs):
            w = Waypoint()
            for k, v in kwargs.items():
                setattr(w, k, v)
            return w

        # WP0: X vel- toward stack, Z rises to safe height, A rotates to arrive_a.
        #   A rotation completes while X >> 250mm (danger zone threshold), keeping
        #   A clear of the stack's vertical pillars. Advance when X <= pre_x AND
        #   Z in_position AND A in_position.
        # WP1: X vel- continues, Z descends to slot_top_z (top of slot envelope).
        #   skip_a — A stays at arrive_a for the rest of the transit.
        #   Advance when X <= stack_x + blend_mm.
        # WP2: X position to stack_x, skip_z, skip_a. Full stop at top of envelope.
        goal.waypoints = [
            _wp(x_mm=pre_x, z_mm=self._safe_z, skip_a=False, a_deg=arrive_a, ramp_mode=2,
                skip_x=False, skip_z=False, velocity_scale=0.0, dmax_factor=0),
            _wp(x_mm=stack_x + blend_mm, z_mm=slot_top_z, skip_a=True, ramp_mode=2,
                skip_x=False, skip_z=False, a_deg=0.0, velocity_scale=0.0, dmax_factor=0),
            _wp(x_mm=stack_x, skip_z=True, skip_a=True, ramp_mode=0, dmax_factor=0,
                skip_x=False, a_deg=0.0, z_mm=0.0, velocity_scale=0.0),
        ]
        self.blackboard.set(name=self._goal_key, value=goal)
        super().initialise()
