"""
Stack-aware homing sequence for the RDJ vinyl robot.

Implements the six-phase sequence defined in docs/MOTION.md.
Runs as a blocking function inside the HomeAll action server callback thread.

Phase 0 — Assess:         read all sensors, classify regime
Phase 1 — Z safe height:  lift Z to clear height (skip if already high)
Phase 2 — X to safe zone: move X to middle; handle dangerous A first
Phase 3 — Home A:         run A axis homing to endstop, park at 180°
Phase 4 — Home X:         run X axis homing to endstop, move to safe zone
Phase 5 — Home Z:         run Z axis homing to endstop, lower to clear height
Phase 6 — Done
"""

import time
from enum import Enum, auto
from typing import TYPE_CHECKING

from .can_interface import (
    CW_ENABLE,
    CW_HALT,
    CW_HOME,
    RAMP_POSITION,
)

if TYPE_CHECKING:
    from .motion_coordinator_node import MotionCoordinatorNode


class Regime(Enum):
    IN_STACK = auto()
    NEAR_TURNTABLE = auto()
    OPEN_AIR = auto()


def normalize_angle(angle_deg: float) -> float:
    """Wrap angle to [-180, 180]."""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def classify_regime(x_mm: float, a_deg: float, cfg: dict) -> Regime:
    stack_cfg = cfg["record_stack"]
    turntable_cfg = cfg["turntable"]

    stack_angle = stack_cfg["a_center_deg"]
    pointing_at_stack = abs(normalize_angle(a_deg - stack_angle)) < 90.0

    if x_mm < stack_cfg["x_danger_mm"] and pointing_at_stack:
        return Regime.IN_STACK
    elif x_mm > turntable_cfg["x_danger_mm"]:
        return Regime.NEAR_TURNTABLE
    else:
        return Regime.OPEN_AIR


def run_homing(node: "MotionCoordinatorNode", goal_handle) -> bool:
    """
    Execute the full homing sequence.
    Returns True on success, False on failure/abort.
    Publishes HomeAll.Feedback throughout.
    Blocking — runs in the action server's executor thread.
    """
    from vinyl_robot_msgs.action import HomeAll

    feedback = HomeAll.Feedback()
    cfg = node.config["homing"]
    vel = cfg["velocities"]

    def publish(phase: str, axes_homed: int = 0):
        feedback.current_phase = phase
        feedback.axes_homed = axes_homed
        goal_handle.publish_feedback(feedback)
        node.get_logger().info(f"[homing] {phase}")

    def check_cancel() -> bool:
        if goal_handle.is_cancel_requested:
            node.get_logger().warn("[homing] Cancelled by client")
            goal_handle.canceled()
            return True
        if node.estop:
            node.get_logger().error("[homing] E-stop fired during homing")
            return True
        return False

    # ─── Phase 0: Assess ──────────────────────────────────────────────────────
    publish("assessing")
    time.sleep(0.05)  # let TPDO state settle

    x_mm = node.x_pos_mm()
    a_deg = node.a_pos_deg()
    z_mm = node.z_pos_mm()

    node.get_logger().info(
        f"[homing] Regime assessment: x={x_mm:.1f}mm  z={z_mm:.1f}mm  a={a_deg:.1f}°"
    )

    regime = classify_regime(x_mm, a_deg, cfg)
    node.get_logger().info(f"[homing] Regime: {regime.name}")

    # ─── IN_STACK extraction ──────────────────────────────────────────────────
    if regime == Regime.IN_STACK:
        publish("stack_extraction")

        stack_angle = cfg["record_stack"]["a_center_deg"]
        a_tolerance = cfg["record_stack"]["a_tolerance_deg"]
        misaligned = abs(normalize_angle(a_deg - stack_angle)) > a_tolerance

        if misaligned:
            node.get_logger().error(
                f"[homing] ABORT: arm inside stack but A={a_deg:.1f}° is misaligned "
                f"(tolerance ±{a_tolerance}°). Manual repositioning required."
            )
            return False

        # Retract X only — Z and A FROZEN
        x_vmax = node.mmps_to_vmax("x_axis", vel["stack_retract_mmps"])
        x_clear = cfg["record_stack"]["x_clear_mm"]

        node.send_rpdo_stepper(
            "x_axis",
            target_steps=node.mm_to_steps("x_axis", x_clear + 50),
            vmax=x_vmax,
            ctrl_word=CW_ENABLE,
            ramp_mode=RAMP_POSITION,
        )

        node.get_logger().info(f"[homing] Retracting X from stack to {x_clear}mm …")
        while node.x_pos_mm() < x_clear:
            if check_cancel():
                return False
            time.sleep(0.02)

        node.send_rpdo_stepper(
            "x_axis",
            target_steps=node.x_steps(),
            vmax=0,
            ctrl_word=CW_ENABLE | CW_HALT,
            ramp_mode=RAMP_POSITION,
        )
        time.sleep(0.1)

    # ─── Phase 1: Z to safe height ────────────────────────────────────────────
    z_clear = cfg["safe_zone"]["z_clear_mm"]
    if node.z_pos_mm() < z_clear:
        publish("z_clear")
        z_vmax = node.mmps_to_vmax("z_axis", vel["z_initial_up_mmps"])
        node.send_rpdo_stepper(
            "z_axis",
            target_steps=node.mm_to_steps("z_axis", z_clear + 20),
            vmax=z_vmax,
            ctrl_word=CW_ENABLE,
            ramp_mode=RAMP_POSITION,
        )
        while node.z_pos_mm() < z_clear:
            if check_cancel():
                return False
            time.sleep(0.02)
        node.get_logger().info(f"[homing] Z at safe height ({node.z_pos_mm():.1f}mm)")

    # ─── Phase 2: X to safe middle zone ──────────────────────────────────────
    safe_min = cfg["safe_zone"]["x_min_mm"]
    safe_max = cfg["safe_zone"]["x_max_mm"]
    safe_center = (safe_min + safe_max) / 2.0
    a_min_safe = node.config["safety"]["a_axis"]["min_safe_deg"]
    a_max_safe = node.config["safety"]["a_axis"]["max_safe_deg"]
    a_deg = node.a_pos_deg()

    a_is_dangerous = not (a_min_safe < a_deg < a_max_safe)

    if a_is_dangerous and not (safe_min < node.x_pos_mm() < safe_max):
        # Worst case — dangerous A AND X near obstacle: Z to max first
        publish("a_emergency_park")
        _rotate_a_to_park(node, cfg, vel, check_cancel)
        if not _ensure_z_high(node, cfg, vel, check_cancel):
            return False
    elif a_is_dangerous:
        publish("a_emergency_park")
        _rotate_a_to_park(node, cfg, vel, check_cancel)

    if not (safe_min < node.x_pos_mm() < safe_max):
        publish("x_to_safe")
        x_vmax = node.mmps_to_vmax("x_axis", vel["x_to_safe_zone_mmps"])
        node.send_rpdo_stepper(
            "x_axis",
            target_steps=node.mm_to_steps("x_axis", safe_center),
            vmax=x_vmax,
            ctrl_word=CW_ENABLE,
            ramp_mode=RAMP_POSITION,
        )
        while not (safe_min < node.x_pos_mm() < safe_max):
            if check_cancel():
                return False
            time.sleep(0.02)
        node.get_logger().info(f"[homing] X in safe zone ({node.x_pos_mm():.1f}mm)")

    # ─── Phase 3: Home A ──────────────────────────────────────────────────────
    publish("a_homing")
    a_vmax = node.mmps_to_vmax("a_axis", vel["a_normal_home_dps"])
    node.send_rpdo_stepper(
        "a_axis",
        target_steps=0,
        vmax=a_vmax,
        ctrl_word=CW_ENABLE | CW_HOME,
        ramp_mode=RAMP_POSITION,
    )

    # Brief settle: give the RPDO time to reach the drive and at least one TPDO
    # to come back with the homed bit cleared before we start polling.
    time.sleep(0.1)

    # Wait for homed bit
    timeout = 20.0
    elapsed = 0.1
    while not node.a_homed():
        if check_cancel():
            return False
        if elapsed >= timeout:
            node.get_logger().error("[homing] A axis homing timeout")
            return False
        time.sleep(0.05)
        elapsed += 0.05

    # Park A at 180°
    park_deg = cfg["safe_zone"]["a_park_deg"]
    a_vmax = node.mmps_to_vmax("a_axis", vel["a_normal_home_dps"])
    node.send_rpdo_stepper(
        "a_axis",
        target_steps=node.deg_to_steps(park_deg),
        vmax=a_vmax,
        ctrl_word=CW_ENABLE,
        ramp_mode=RAMP_POSITION,
    )
    _wait_in_position(node, "a_axis", timeout=10.0)
    node.get_logger().info(f"[homing] A homed and parked at {park_deg}°")
    publish("a_homing", axes_homed=0b100)

    # ─── Phase 4: Home X ──────────────────────────────────────────────────────
    publish("x_homing", axes_homed=0b100)
    x_vmax = node.mmps_to_vmax("x_axis", vel["x_home_mmps"])
    node.send_rpdo_stepper(
        "x_axis",
        target_steps=0,
        vmax=x_vmax,
        ctrl_word=CW_ENABLE | CW_HOME,
        ramp_mode=RAMP_POSITION,
    )
    time.sleep(0.1)
    elapsed = 0.1
    while not node.x_homed():
        if check_cancel():
            return False
        if elapsed >= 30.0:
            node.get_logger().error("[homing] X axis homing timeout")
            return False
        time.sleep(0.05)
        elapsed += 0.05

    # Move X to safe zone center
    node.send_rpdo_stepper(
        "x_axis",
        target_steps=node.mm_to_steps("x_axis", safe_center),
        vmax=x_vmax,
        ctrl_word=CW_ENABLE,
        ramp_mode=RAMP_POSITION,
    )
    _wait_in_position(node, "x_axis", timeout=30.0)
    node.get_logger().info("[homing] X homed")
    publish("x_homing", axes_homed=0b101)

    # ─── Phase 5: Home Z ──────────────────────────────────────────────────────
    publish("z_homing", axes_homed=0b101)
    z_vmax = node.mmps_to_vmax("z_axis", vel["z_home_mmps"])
    node.send_rpdo_stepper(
        "z_axis",
        target_steps=0,
        vmax=z_vmax,
        ctrl_word=CW_ENABLE | CW_HOME,
        ramp_mode=RAMP_POSITION,
    )
    time.sleep(0.1)
    elapsed = 0.1
    while not node.z_homed():
        if check_cancel():
            return False
        if elapsed >= 30.0:
            node.get_logger().error("[homing] Z axis homing timeout")
            return False
        time.sleep(0.05)
        elapsed += 0.05

    # Lower Z to clearance height
    node.send_rpdo_stepper(
        "z_axis",
        target_steps=node.mm_to_steps("z_axis", z_clear),
        vmax=z_vmax,
        ctrl_word=CW_ENABLE,
        ramp_mode=RAMP_POSITION,
    )
    _wait_in_position(node, "z_axis", timeout=15.0)
    node.get_logger().info("[homing] Z homed")

    # ─── Phase 6: Done ────────────────────────────────────────────────────────
    publish("done", axes_homed=0b111)
    node.get_logger().info("[homing] All axes homed successfully")
    return True


# ── Helpers ───────────────────────────────────────────────────────────────────


def _rotate_a_to_park(node, cfg, vel, check_cancel) -> None:
    park_deg = cfg["safe_zone"]["a_park_deg"]
    a_vmax = node.mmps_to_vmax("a_axis", vel["a_emergency_park_dps"])
    node.send_rpdo_stepper(
        "a_axis",
        target_steps=node.deg_to_steps(park_deg),
        vmax=a_vmax,
        ctrl_word=CW_ENABLE,
        ramp_mode=RAMP_POSITION,
    )
    _wait_in_position(node, "a_axis", timeout=60.0)


def _ensure_z_high(node, cfg, vel, check_cancel) -> bool:
    z_clear = cfg["safe_zone"]["z_clear_mm"]
    if node.z_pos_mm() >= z_clear:
        return True
    z_vmax = node.mmps_to_vmax("z_axis", vel["z_initial_up_mmps"])
    node.send_rpdo_stepper(
        "z_axis",
        target_steps=node.mm_to_steps("z_axis", z_clear + 20),
        vmax=z_vmax,
        ctrl_word=CW_ENABLE,
        ramp_mode=RAMP_POSITION,
    )
    elapsed = 0.0
    while node.z_pos_mm() < z_clear:
        if check_cancel():
            return False
        if elapsed >= 20.0:
            return False
        time.sleep(0.02)
        elapsed += 0.02
    return True


def _wait_in_position(node, axis_name: str, timeout: float = 15.0) -> bool:
    # Sleep before first poll so the RPDO has time to propagate and at least one
    # TPDO comes back reflecting the new target (avoids reading stale in_position).
    time.sleep(0.1)
    elapsed = 0.1
    while elapsed < timeout:
        state = node.node_states[axis_name]
        if state.in_position and not state.moving:
            return True
        time.sleep(0.02)
        elapsed += 0.02
    node.get_logger().warn(f"[homing] Timeout waiting for {axis_name} in-position")
    return False
