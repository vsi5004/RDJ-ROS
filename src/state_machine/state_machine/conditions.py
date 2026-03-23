"""
conditions.py — Condition behaviors for the vinyl robot state machine.

All conditions are stateless reads from the blackboard.  They never send
ROS messages — that is the job of action behaviors.
"""

import py_trees

from . import blackboard_keys as K

_NS = "/rdj"


class SafetyCheck(py_trees.behaviour.Behaviour):
    """
    Returns SUCCESS when the robot is safe to operate:
      - /safety/velocity_scale > 0.0  (no LiDAR stop zone)
      - /safety/estop is False         (no hardware or software e-stop)

    Returns RUNNING on the very first tick before the blackboard has been
    populated by the node's subscription callbacks (startup grace period).
    Returns FAILURE otherwise — which causes the ReactiveRoot Sequence to
    halt all running children (action clients send cancel).
    """

    def __init__(self, name: str = "SafetyCheck"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.SAFETY_OK,    access=py_trees.common.Access.READ)
        self._bb.register_key(K.ESTOP_ACTIVE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            safety_ok   = self._bb.safety_ok
            estop_active = self._bb.estop_active
        except KeyError:
            # Blackboard not yet written — give startup a grace tick
            self.feedback_message = "Waiting for safety topics…"
            return py_trees.common.Status.RUNNING

        if estop_active:
            self.feedback_message = "E-STOP ACTIVE"
            return py_trees.common.Status.FAILURE

        if not safety_ok:
            self.feedback_message = "Safety velocity scale is zero (LiDAR stop zone)"
            return py_trees.common.Status.FAILURE

        self.feedback_message = "OK"
        return py_trees.common.Status.SUCCESS


class IsAllHomed(py_trees.behaviour.Behaviour):
    """
    Returns SUCCESS only when all axes are homed AND no re-home has been
    requested by the operator.  Checking force_rehome here (rather than
    manipulating all_homed from the node) avoids the race between the
    'home' user command and /motion/status overwriting all_homed=True.
    """

    def __init__(self, name: str = "IsAllHomed"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.ALL_HOMED,    access=py_trees.common.Access.READ)
        self._bb.register_key(K.FORCE_REHOME, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            homed        = self._bb.all_homed
            force_rehome = self._bb.force_rehome
        except KeyError:
            return py_trees.common.Status.FAILURE

        if force_rehome:
            self.feedback_message = "Re-home requested by operator"
            return py_trees.common.Status.FAILURE

        if homed:
            self.feedback_message = "All axes homed"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "Not yet homed"
        return py_trees.common.Status.FAILURE


class WaitForRecordFinished(py_trees.behaviour.Behaviour):
    """
    Returns RUNNING while the record is still playing (progress < threshold).
    Returns SUCCESS when progress reaches the run-out groove.
    The threshold is read from robot_params.yaml and passed at construction.
    """

    def __init__(self, progress_threshold: float = 0.92,
                 name: str = "WaitForRecordFinished"):
        super().__init__(name)
        self._threshold = progress_threshold
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.PLAYBACK_PROGRESS,
                              access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            progress = self._bb.playback_progress
        except KeyError:
            self.feedback_message = "Waiting for turntable/progress topic…"
            return py_trees.common.Status.RUNNING

        if progress >= self._threshold:
            self.feedback_message = f"Record finished ({progress:.2f} ≥ {self._threshold:.2f})"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"Playing: {progress:.1%}"
        return py_trees.common.Status.RUNNING


class IsFlipAction(py_trees.behaviour.Behaviour):
    """Gate: returns SUCCESS only when next_action == 'flip'."""

    def __init__(self, name: str = "IsFlipAction"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.NEXT_ACTION, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            action = self._bb.next_action
        except KeyError:
            return py_trees.common.Status.FAILURE
        return (py_trees.common.Status.SUCCESS
                if action == "flip"
                else py_trees.common.Status.FAILURE)


class IsSwapAction(py_trees.behaviour.Behaviour):
    """Gate: returns SUCCESS only when next_action == 'swap'."""

    def __init__(self, name: str = "IsSwapAction"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.NEXT_ACTION, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            action = self._bb.next_action
        except KeyError:
            return py_trees.common.Status.FAILURE
        return (py_trees.common.Status.SUCCESS
                if action == "swap"
                else py_trees.common.Status.FAILURE)


class IsHaltRequested(py_trees.behaviour.Behaviour):
    """
    Returns SUCCESS when next_action == 'halt' (all records played).
    Used as the first child of Selector('OperationalOrHalt') — when this
    succeeds the LoopDecorator is never ticked and the tree idles.
    """

    def __init__(self, name: str = "IsHaltRequested"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.NEXT_ACTION, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            action = self._bb.next_action
        except KeyError:
            return py_trees.common.Status.FAILURE
        return (py_trees.common.Status.SUCCESS
                if action == "halt"
                else py_trees.common.Status.FAILURE)


class FlipClearanceCheck(py_trees.behaviour.Behaviour):
    """
    Checks that there is enough clearance below the pincher arm to safely
    rotate a record 180°.

    The pincher ToF sensor measures the distance from the bottom of the arm
    to whatever surface is below it (floor, base, etc.).  It is NOT used to
    detect record contact.  minimum clearance = 7 inches (178 mm default,
    configurable via flip.clearance_min_mm in robot_params.yaml).

    Returns SUCCESS if pincher_tof_mm ≥ clearance_min_mm, FAILURE otherwise.
    Returns RUNNING if the value hasn't been published yet.
    """

    def __init__(self, clearance_min_mm: float = 178.0,
                 name: str = "FlipClearanceCheck"):
        super().__init__(name)
        self._min = clearance_min_mm
        self._bb = self.attach_blackboard_client(name=name, namespace=_NS)
        self._bb.register_key(K.PINCHER_TOF_MM, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            tof = self._bb.pincher_tof_mm
        except KeyError:
            self.feedback_message = "Waiting for pincher_tof_mm…"
            return py_trees.common.Status.RUNNING

        if tof >= self._min:
            self.feedback_message = f"Clearance OK: {tof:.0f} mm ≥ {self._min:.0f} mm"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"Insufficient clearance: {tof:.0f} mm < {self._min:.0f} mm"
        return py_trees.common.Status.FAILURE
