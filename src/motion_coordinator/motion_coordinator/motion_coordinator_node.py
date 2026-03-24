"""
motion_coordinator_node.py — RDJ vinyl robot motion coordinator.

Responsibilities:
  - Parse raw PDO bytes arriving from CAN nodes (or mock) and maintain internal state.
  - Convert high-level mm/deg goals into microstep RPDO commands.
  - Apply safety velocity scaling from lidar_safety before every RPDO write.
  - Host six action servers used by the behavior tree.
  - Publish /motion/status at ~50 Hz.

CAN interface topology:
  Each CAN node is represented by two ROS 2 topics:
    /canopen/<name>/tpdo1  (std_msgs/UInt8MultiArray) — incoming state from CAN node
    /canopen/<name>/rpdo1  (std_msgs/UInt8MultiArray) — outgoing commands to CAN node
  The mock_canopen_master publishes/subscribes the exact same topics.
  For real hardware, a thin ros2_canopen bridge adapts the ProxyDriver interface.
"""

import struct
import time
from typing import ClassVar

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray

from vinyl_robot_msgs.action import ExecuteTrajectory, FlipRecord, Grip, HomeAll, MoveToPosition, PressPlay, SetSpeed
from vinyl_robot_msgs.msg import MotionStatus, SafetyStatus

from .can_interface import (
    CW_ENABLE,
    CW_HALT,
    RAMP_POSITION,
    RAMP_VELOCITY_FWD,
    RAMP_VELOCITY_REV,
    AAxisTPDO,
    ServoTPDO,
    StepperTPDO,
    build_servo_rpdo,
    build_stepper_rpdo,
)
from .homing import run_homing

# ── Thin state container shared by TPDO subscribers ───────────────────────────


class StepperState:
    def __init__(self):
        self.actual_pos: int = 0  # microsteps
        self.status_word: int = 0
        self.ramp_status: int = 0
        self.tof_mm: int = 0xFFFF

    @property
    def homed(self) -> bool:
        return bool(self.status_word & 0x01)

    @property
    def moving(self) -> bool:
        return bool(self.status_word & 0x02)

    @property
    def in_position(self) -> bool:
        return bool(self.status_word & 0x04)

    @property
    def fault(self) -> bool:
        return bool(self.status_word & 0x08)


class AAxisState(StepperState):
    def __init__(self):
        super().__init__()
        self.pot_angle_raw: int = 0  # INT16, 0.1° units


class ServoState:
    def __init__(self):
        self.servo1_us: int = 1500
        self.servo2_us: int = 1500
        self.tof_mm: int = 0xFFFF
        self.status_word: int = 0


# ── Motion coordinator node ────────────────────────────────────────────────────


class MotionCoordinatorNode(Node):
    NODE_NAMES: ClassVar[list[str]] = ["x_axis", "z_axis", "a_axis", "pincher", "player"]
    STEPPER_NAMES: ClassVar[list[str]] = ["x_axis", "z_axis", "a_axis"]
    SERVO_NAMES: ClassVar[list[str]] = ["pincher", "player"]

    def __init__(self):
        super().__init__("motion_coordinator")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("motion_coordinator: config_path parameter is required")

        with open(config_path) as f:
            self.config: dict = yaml.safe_load(f)
        self.get_logger().info(f"Loaded config: {config_path}")

        # ── Internal state ────────────────────────────────────────────────────
        self.node_states: dict = {
            "x_axis": StepperState(),
            "z_axis": StepperState(),
            "a_axis": AAxisState(),
            "pincher": ServoState(),
            "player": ServoState(),
        }
        self.safety_scale: float = 1.0
        self._hw_estop: bool = False  # from lidar_safety (/safety/estop)
        self._sw_estop: bool = False  # from operator (/user/estop)
        self._fault: bool = False
        self._fault_msg: str = ""

        # Move targets (steps) — updated on every RPDO send; equals actual when idle
        self._x_target_steps: int = 0
        self._a_target_steps: int = 0

        cb = ReentrantCallbackGroup()

        # ── TPDO subscribers ──────────────────────────────────────────────────
        for name in self.NODE_NAMES:
            self.create_subscription(
                UInt8MultiArray,
                f"/canopen/{name}/tpdo1",
                lambda msg, n=name: self._on_tpdo(n, msg),
                10,
                callback_group=cb,
            )

        # ── Safety subscribers ────────────────────────────────────────────────
        self.create_subscription(
            SafetyStatus,
            "/safety/status",
            self._on_safety_status,
            10,
            callback_group=cb,
        )
        self.create_subscription(
            Bool,
            "/user/estop",
            self._on_user_estop,
            10,
            callback_group=cb,
        )
        self.create_subscription(
            String,
            "/motion/servo_raw",
            self._on_servo_raw,
            10,
            callback_group=cb,
        )
        self.create_subscription(
            String,
            "/motion/jog",
            self._on_jog,
            10,
            callback_group=cb,
        )

        # ── RPDO publishers ───────────────────────────────────────────────────
        self._rpdo_pubs: dict = {}
        for name in self.NODE_NAMES:
            self._rpdo_pubs[name] = self.create_publisher(
                UInt8MultiArray, f"/canopen/{name}/rpdo1", 10
            )

        # ── Status publisher ──────────────────────────────────────────────────
        self._status_pub = self.create_publisher(MotionStatus, "/motion/status", 10)
        self.create_timer(0.02, self._publish_status, callback_group=cb)

        # ── Action servers ────────────────────────────────────────────────────
        self._move_server = ActionServer(
            self,
            MoveToPosition,
            "/motion/move_to_position",
            execute_callback=self._move_to_position_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._home_server = ActionServer(
            self,
            HomeAll,
            "/motion/home_all",
            execute_callback=self._home_all_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._grip_server = ActionServer(
            self,
            Grip,
            "/motion/grip",
            execute_callback=self._grip_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._flip_server = ActionServer(
            self,
            FlipRecord,
            "/motion/flip_record",
            execute_callback=self._flip_record_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._play_server = ActionServer(
            self,
            PressPlay,
            "/motion/press_play",
            execute_callback=self._press_play_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._speed_server = ActionServer(
            self,
            SetSpeed,
            "/motion/set_speed",
            execute_callback=self._set_speed_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._traj_server = ActionServer(
            self,
            ExecuteTrajectory,
            "/motion/execute_trajectory",
            execute_callback=self._execute_trajectory_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )

        self.get_logger().info("motion_coordinator ready")

    # ── TPDO handlers ─────────────────────────────────────────────────────────

    def _on_tpdo(self, name: str, msg: UInt8MultiArray) -> None:
        data = bytes(msg.data)
        try:
            if name == "x_axis" or name == "z_axis":
                parsed = StepperTPDO.from_bytes(data)
                state: StepperState = self.node_states[name]
                state.actual_pos = parsed.actual_pos
                state.status_word = parsed.status_word
                state.ramp_status = parsed.ramp_status
                state.tof_mm = parsed.tof_mm
            elif name == "a_axis":
                parsed = AAxisTPDO.from_bytes(data)
                state: AAxisState = self.node_states["a_axis"]
                state.actual_pos = parsed.actual_pos
                state.status_word = parsed.status_word
                state.ramp_status = parsed.ramp_status
                state.pot_angle_raw = parsed.pot_angle_raw
            elif name in self.SERVO_NAMES:
                parsed = ServoTPDO.from_bytes(data)
                state: ServoState = self.node_states[name]
                state.servo1_us = parsed.servo1_us
                state.servo2_us = parsed.servo2_us
                state.tof_mm = parsed.tof_mm
                state.status_word = parsed.status_word

                # Check for fault on any stepper
                if name in self.STEPPER_NAMES and parsed.fault and not self._fault:
                    self._fault = True
                    self._fault_msg = f"{name} reports fault"
                    self.get_logger().error(self._fault_msg)
        except (ValueError, struct.error) as exc:
            self.get_logger().debug(f"Bad TPDO from {name}: {exc}")

    # ── Safety handlers ────────────────────────────────────────────────────────

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        self.safety_scale = float(msg.velocity_scale)
        was = self.estop
        self._hw_estop = bool(msg.estop)
        if self.estop and not was:
            self.get_logger().error("Hardware E-STOP — halting all axes")
            self._halt_all()

    @property
    def estop(self) -> bool:
        return self._hw_estop or self._sw_estop

    def _on_user_estop(self, msg: Bool) -> None:
        was = self.estop
        self._sw_estop = bool(msg.data)
        if self._sw_estop and not was:
            self.get_logger().error("Software E-STOP — halting all axes")
            self._halt_all()
        elif not self._sw_estop and was and not self._hw_estop:
            self.get_logger().info("E-STOP cleared")

    def _on_jog(self, msg: String) -> None:
        # Format: "<axis> <delta>"  e.g. "x 10.0" or "z -5.0" or "a 90.0"
        try:
            parts = msg.data.split()
            axis, delta = parts[0].lower(), float(parts[1])
        except (ValueError, IndexError):
            self.get_logger().warn(f"jog: bad format: {msg.data!r}")
            return
        if self.estop:
            return
        axis_map = {"x": "x_axis", "z": "z_axis", "a": "a_axis"}
        if axis not in axis_map:
            self.get_logger().warn(f"jog: unknown axis {axis!r}")
            return
        axis_name = axis_map[axis]
        if axis == "a":
            delta_steps = self.deg_to_steps(delta)
            vmax = self.mmps_to_vmax("a_axis", 30.0)
        else:
            delta_steps = self.mm_to_steps(axis_name, delta)
            vmax = self.mmps_to_vmax(axis_name, 50.0)
        current = self.node_states[axis_name].actual_pos
        self.send_rpdo_stepper(axis_name, current + delta_steps, vmax, CW_ENABLE, RAMP_POSITION)

    def _on_servo_raw(self, msg: String) -> None:
        # Format: "<node_name> <s1_us> <s2_us>"  e.g. "pincher 1500 1200"
        try:
            parts = msg.data.split()
            name, s1, s2 = parts[0], int(parts[1]), int(parts[2])
            if name not in self.SERVO_NAMES:
                self.get_logger().warn(f"servo_raw: unknown node {name!r}")
                return
            s1 = max(500, min(2500, s1))
            s2 = max(500, min(2500, s2))
            self.send_rpdo_servo(name, s1, s2)
        except (ValueError, IndexError):
            self.get_logger().warn(f"servo_raw: bad format: {msg.data!r}")

    # ── Coordinate conversion helpers ─────────────────────────────────────────

    def mm_to_steps(self, axis_name: str, mm: float) -> int:
        key = axis_name.replace("_axis", "")
        steps_per_mm = self.config["geometry"][f"{key}_steps_per_mm"]
        return round(mm * steps_per_mm)

    def deg_to_steps(self, deg: float) -> int:
        return round(deg * self.config["geometry"]["a_steps_per_deg"])

    def steps_to_mm(self, axis_name: str, steps: int) -> float:
        key = axis_name.replace("_axis", "")
        steps_per_mm = self.config["geometry"][f"{key}_steps_per_mm"]
        return steps / steps_per_mm

    def steps_to_deg(self, steps: int) -> float:
        return steps / self.config["geometry"]["a_steps_per_deg"]

    def mmps_to_vmax(self, axis_name: str, speed: float) -> int:
        """Convert mm/s (or deg/s for a_axis) to VMAX pps with safety scaling."""
        scale = max(0.0, min(1.0, self.safety_scale))
        if axis_name == "a_axis":
            raw = speed * self.config["geometry"]["a_steps_per_deg"] * scale
        else:
            key = axis_name.replace("_axis", "")
            raw = speed * self.config["geometry"][f"{key}_steps_per_mm"] * scale
        return max(1, min(int(raw), 65535))

    # ── Current position accessors ────────────────────────────────────────────

    def x_pos_mm(self) -> float:
        return self.steps_to_mm("x_axis", self.node_states["x_axis"].actual_pos)

    def z_pos_mm(self) -> float:
        return self.steps_to_mm("z_axis", self.node_states["z_axis"].actual_pos)

    def a_pos_deg(self) -> float:
        return self.steps_to_deg(self.node_states["a_axis"].actual_pos)

    def x_steps(self) -> int:
        return self.node_states["x_axis"].actual_pos

    def x_homed(self) -> bool:
        return self.node_states["x_axis"].homed

    def z_homed(self) -> bool:
        return self.node_states["z_axis"].homed

    def a_homed(self) -> bool:
        return self.node_states["a_axis"].homed

    # ── RPDO send helpers ─────────────────────────────────────────────────────

    def send_rpdo_stepper(
        self, name: str, target_steps: int, vmax: int, ctrl_word: int, ramp_mode: int
    ) -> None:
        if self.estop and not (ctrl_word & CW_HALT):
            return  # block all motion during e-stop except explicit halt
        if name == "x_axis":
            self._x_target_steps = target_steps
        elif name == "a_axis":
            self._a_target_steps = target_steps
        data = build_stepper_rpdo(target_steps, vmax, ctrl_word, ramp_mode)
        msg = UInt8MultiArray()
        msg.data = list(data)
        self._rpdo_pubs[name].publish(msg)

    def send_rpdo_servo(
        self, name: str, servo1_us: int, servo2_us: int, ctrl_word: int = CW_ENABLE
    ) -> None:
        data = build_servo_rpdo(servo1_us, servo2_us, ctrl_word)
        msg = UInt8MultiArray()
        msg.data = list(data)
        self._rpdo_pubs[name].publish(msg)

    def _halt_all(self) -> None:
        for name in self.STEPPER_NAMES:
            state = self.node_states[name]
            self.send_rpdo_stepper(
                name,
                target_steps=state.actual_pos,
                vmax=0,
                ctrl_word=CW_HALT,
                ramp_mode=RAMP_POSITION,
            )

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        msg = MotionStatus()
        msg.x_mm = self.x_pos_mm()
        msg.z_mm = self.z_pos_mm()
        msg.a_deg = self.a_pos_deg()
        msg.x_target_mm = self.steps_to_mm("x_axis", self._x_target_steps)
        msg.a_target_deg = self.steps_to_deg(self._a_target_steps)

        xs: StepperState = self.node_states["x_axis"]
        zs: StepperState = self.node_states["z_axis"]
        as_: AAxisState = self.node_states["a_axis"]
        ps: ServoState = self.node_states["pincher"]
        pl: ServoState = self.node_states["player"]

        msg.x_homed = xs.homed
        msg.z_homed = zs.homed
        msg.a_homed = as_.homed
        msg.all_homed = xs.homed and zs.homed and as_.homed

        msg.x_moving = xs.moving
        msg.z_moving = zs.moving
        msg.a_moving = as_.moving

        msg.x_tof_mm = float(xs.tof_mm)
        msg.z_tof_mm = float(zs.tof_mm)
        msg.pincher_tof_mm = float(ps.tof_mm)

        msg.x_status_word = xs.status_word
        msg.z_status_word = zs.status_word
        msg.a_status_word = as_.status_word
        msg.pincher_status_word = ps.status_word
        msg.player_status_word = pl.status_word

        msg.fault = self._fault
        msg.fault_msg = self._fault_msg
        msg.velocity_scale = self.safety_scale

        self._status_pub.publish(msg)

    # ── Goal / cancel callbacks (shared) ──────────────────────────────────────

    def _accept_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    # ── Action: MoveToPosition ────────────────────────────────────────────────

    def _move_to_position_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = MoveToPosition.Feedback()
        result = MoveToPosition.Result()

        # Block if e-stopped or faulted
        if self.estop:
            result.success = False
            result.error_msg = "E-stop active"
            goal_handle.abort()
            return result

        # Apply per-goal velocity scale on top of safety scale
        combined_scale = goal.velocity_scale * self.safety_scale

        if not goal.skip_x:
            self.send_rpdo_stepper(
                "x_axis",
                target_steps=self.mm_to_steps("x_axis", goal.x_mm),
                vmax=self.mmps_to_vmax("x_axis", 200) if combined_scale > 0 else 1,
                ctrl_word=CW_ENABLE,
                ramp_mode=RAMP_POSITION,
            )
        if not goal.skip_z:
            self.send_rpdo_stepper(
                "z_axis",
                target_steps=self.mm_to_steps("z_axis", goal.z_mm),
                vmax=self.mmps_to_vmax("z_axis", 100),
                ctrl_word=CW_ENABLE,
                ramp_mode=RAMP_POSITION,
            )
        if not goal.skip_a:
            self.send_rpdo_stepper(
                "a_axis",
                target_steps=self.deg_to_steps(goal.a_deg),
                vmax=self.mmps_to_vmax("a_axis", 60),
                ctrl_word=CW_ENABLE,
                ramp_mode=RAMP_POSITION,
            )

        # Log what was commanded (helps diagnose unexpected axis movements).
        axes_info = []
        if not goal.skip_x:
            axes_info.append(f"X→{goal.x_mm:.0f}mm")
        if not goal.skip_z:
            axes_info.append(f"Z→{goal.z_mm:.0f}mm")
        if not goal.skip_a:
            axes_info.append(f"A→{goal.a_deg:.0f}°")
        self.get_logger().info(
            f"[move] {', '.join(axes_info) or 'no axes'} (scale={combined_scale:.2f})"
        )

        # Brief settle: give RPDOs time to reach the mock and at least one TPDO
        # to come back with updated targets before polling in_position.
        # Without this, the first poll sees the *old* in_position (True from the
        # previous move) and returns SUCCESS before the arm moves at all.
        time.sleep(0.1)

        timeout = 60.0
        elapsed = 0.1
        while elapsed < timeout:
            if goal_handle.is_cancel_requested:
                self._halt_all()
                goal_handle.canceled()
                result.success = False
                result.error_msg = "Cancelled"
                return result

            if self.estop:
                result.success = False
                result.error_msg = "E-stop active"
                goal_handle.abort()
                return result

            xs = self.node_states["x_axis"]
            zs = self.node_states["z_axis"]
            as_ = self.node_states["a_axis"]

            x_done = goal.skip_x or xs.in_position
            z_done = goal.skip_z or zs.in_position
            a_done = goal.skip_a or as_.in_position

            feedback.x_actual_mm = self.x_pos_mm()
            feedback.z_actual_mm = self.z_pos_mm()
            feedback.a_actual_deg = self.a_pos_deg()
            feedback.x_in_position = x_done
            feedback.z_in_position = z_done
            feedback.a_in_position = a_done
            goal_handle.publish_feedback(feedback)

            if x_done and z_done and a_done:
                break

            time.sleep(0.02)
            elapsed += 0.02

        result.success = elapsed < timeout
        result.error_msg = "" if result.success else "Move timeout"
        result.x_final_mm = self.x_pos_mm()
        result.z_final_mm = self.z_pos_mm()
        result.a_final_deg = self.a_pos_deg()

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── Action: ExecuteTrajectory ─────────────────────────────────────────────

    def _send_waypoint_rpdos(self, wp, default_scale: float) -> None:
        """Send RPDOs for one trajectory waypoint.

        ramp_mode in the waypoint applies only to X (the transit axis).
        Z and A always use position mode so they stop at their targets.
        dmax_factor is packed into byte 7 bits [7:2] for all axes.
        """
        scale = (wp.velocity_scale if wp.velocity_scale > 0.0 else default_scale) * self.safety_scale
        # X gets the full bitfield: dmax_factor (bits 7:2) | ramp_mode (bits 1:0)
        x_ramp_byte = (int(wp.dmax_factor) << 2) | (int(wp.ramp_mode) & 0x03)
        # Z and A always use position mode; dmax_factor still applies for blend control
        za_ramp_byte = (int(wp.dmax_factor) << 2) | RAMP_POSITION

        if not wp.skip_x:
            self.send_rpdo_stepper(
                "x_axis",
                self.mm_to_steps("x_axis", wp.x_mm),
                self.mmps_to_vmax("x_axis", 200.0) if scale > 0 else 1,
                CW_ENABLE,
                x_ramp_byte,
            )
        if not wp.skip_z:
            self.send_rpdo_stepper(
                "z_axis",
                self.mm_to_steps("z_axis", wp.z_mm),
                self.mmps_to_vmax("z_axis", 100.0),
                CW_ENABLE,
                za_ramp_byte,
            )
        if not wp.skip_a:
            self.send_rpdo_stepper(
                "a_axis",
                self.deg_to_steps(wp.a_deg),
                self.mmps_to_vmax("a_axis", 60.0),
                CW_ENABLE,
                za_ramp_byte,
            )

    def _execute_trajectory_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = ExecuteTrajectory.Feedback()
        result = ExecuteTrajectory.Result()

        if self.estop:
            result.success = False
            result.error_msg = "E-stop active"
            goal_handle.abort()
            return result

        waypoints = goal.waypoints
        if not waypoints:
            result.success = False
            result.error_msg = "No waypoints"
            goal_handle.abort()
            return result

        blend_mm = float(goal.blend_radius_mm)
        blend_deg = float(goal.blend_radius_deg)
        default_scale = float(goal.default_velocity_scale)

        # Pre-compute final X and A step targets so LEDs track the true destination
        # throughout the trajectory, not each intermediate waypoint.
        final_x_steps = None
        final_a_steps = None
        for wp in waypoints:
            if not wp.skip_x:
                final_x_steps = self.mm_to_steps("x_axis", wp.x_mm)
            if not wp.skip_a:
                final_a_steps = self.deg_to_steps(wp.a_deg)
        if final_x_steps is not None:
            self._x_target_steps = final_x_steps
        if final_a_steps is not None:
            self._a_target_steps = final_a_steps

        # Send first waypoint RPDOs and restore LED targets
        self._send_waypoint_rpdos(waypoints[0], default_scale)
        if final_x_steps is not None:
            self._x_target_steps = final_x_steps
        if final_a_steps is not None:
            self._a_target_steps = final_a_steps

        # Log trajectory start
        last_axes = []
        for wp in waypoints[-2:]:
            if not wp.skip_x:
                last_axes.append(f"X→{wp.x_mm:.0f}mm")
            if not wp.skip_z:
                last_axes.append(f"Z→{wp.z_mm:.0f}mm")
            if not wp.skip_a:
                last_axes.append(f"A→{wp.a_deg:.0f}°")
        self.get_logger().info(
            f"[traj] {goal.trajectory_name!r}: {len(waypoints)} wps "
            f"({', '.join(last_axes)})"
        )

        # Brief settle before polling — same as MoveToPosition
        time.sleep(0.1)

        waypoints_completed = 0
        timeout_per_wp = 60.0

        for i, wp in enumerate(waypoints):
            is_last = i == len(waypoints) - 1
            elapsed = 0.0

            while elapsed < timeout_per_wp:
                if goal_handle.is_cancel_requested:
                    self._halt_all()
                    goal_handle.canceled()
                    result.success = False
                    result.error_msg = "Cancelled"
                    return result

                if self.estop:
                    result.success = False
                    result.error_msg = "E-stop active"
                    goal_handle.abort()
                    return result

                xs = self.node_states["x_axis"]
                zs = self.node_states["z_axis"]
                as_ = self.node_states["a_axis"]

                if is_last:
                    # Final waypoint: wait for full in_position on all active axes
                    x_done = wp.skip_x or xs.in_position
                    z_done = wp.skip_z or zs.in_position
                    a_done = wp.skip_a or as_.in_position
                elif wp.ramp_mode in (RAMP_VELOCITY_FWD, RAMP_VELOCITY_REV):
                    # Velocity-mode X: advance when X crosses the threshold position
                    if wp.ramp_mode == RAMP_VELOCITY_FWD:
                        x_done = wp.skip_x or (self.x_pos_mm() >= wp.x_mm)
                    else:
                        x_done = wp.skip_x or (self.x_pos_mm() <= wp.x_mm)
                    z_done = wp.skip_z or zs.in_position
                    a_done = wp.skip_a or as_.in_position
                else:
                    # Position-mode intermediate: blend radius
                    x_done = wp.skip_x or abs(self.x_pos_mm() - wp.x_mm) <= blend_mm
                    z_done = wp.skip_z or abs(self.z_pos_mm() - wp.z_mm) <= blend_mm
                    a_done = wp.skip_a or abs(self.a_pos_deg() - wp.a_deg) <= blend_deg

                feedback.x_actual_mm = self.x_pos_mm()
                feedback.z_actual_mm = self.z_pos_mm()
                feedback.a_actual_deg = self.a_pos_deg()
                feedback.current_waypoint_idx = i
                feedback.x_in_position = xs.in_position
                feedback.z_in_position = zs.in_position
                feedback.a_in_position = as_.in_position
                goal_handle.publish_feedback(feedback)

                if x_done and z_done and a_done:
                    waypoints_completed += 1
                    if not is_last:
                        self._send_waypoint_rpdos(waypoints[i + 1], default_scale)
                        # Restore final LED targets after send (which overwrites them)
                        if final_x_steps is not None:
                            self._x_target_steps = final_x_steps
                        if final_a_steps is not None:
                            self._a_target_steps = final_a_steps
                    break

                time.sleep(0.02)
                elapsed += 0.02
            else:
                result.success = False
                result.error_msg = f"Waypoint {i} timeout"
                result.waypoints_completed = waypoints_completed
                goal_handle.abort()
                return result

        result.success = True
        result.error_msg = ""
        result.x_final_mm = self.x_pos_mm()
        result.z_final_mm = self.z_pos_mm()
        result.a_final_deg = self.a_pos_deg()
        result.waypoints_completed = waypoints_completed
        goal_handle.succeed()
        return result

    # ── Action: HomeAll ───────────────────────────────────────────────────────

    def _home_all_cb(self, goal_handle):
        result = HomeAll.Result()
        try:
            success = run_homing(self, goal_handle)
        except Exception as exc:
            self.get_logger().error(f"Homing exception: {exc}")
            success = False
            result.error_msg = str(exc)

        result.success = success
        if success:
            goal_handle.succeed()
        elif not goal_handle.is_cancel_requested:
            goal_handle.abort()
        return result

    # ── Action: Grip ──────────────────────────────────────────────────────────

    def _grip_cb(self, goal_handle):
        goal = goal_handle.request
        result = Grip.Result()
        grip_cfg = self.config["grip"]

        servo1_us = grip_cfg["close_pulse_us"] if goal.close else grip_cfg["open_pulse_us"]
        servo2_us = 1500  # flip servo stays neutral

        self.send_rpdo_servo("pincher", servo1_us, servo2_us)
        time.sleep(0.5)  # wait for servo to reach position

        result.tof_mm = float(self.node_states["pincher"].tof_mm)
        result.success = (
            True  # grip is always successful — servo commanded, no confirmation needed
        )
        result.error_msg = ""
        goal_handle.succeed()
        return result

    # ── Action: FlipRecord ────────────────────────────────────────────────────

    def _flip_record_cb(self, goal_handle):
        result = FlipRecord.Result()
        flip_cfg = self.config["flip"]

        # Flip servo (servo2) to side B position
        current_tpdo: ServoState = self.node_states["pincher"]
        self.send_rpdo_servo("pincher", current_tpdo.servo1_us, flip_cfg["servo_pulse_b_us"])
        time.sleep(1.0)  # full flip takes ~1 second

        result.success = True
        goal_handle.succeed()
        return result

    # ── Action: PressPlay ─────────────────────────────────────────────────────

    def _press_play_cb(self, goal_handle):
        goal = goal_handle.request
        result = PressPlay.Result()
        player_cfg = self.config["player"]

        servo1_us = player_cfg["play_pulse_us"] if goal.press else player_cfg["stop_pulse_us"]
        self.send_rpdo_servo("player", servo1_us, 1500)
        time.sleep(0.3)

        result.success = True
        goal_handle.succeed()
        return result

    # ── Action: SetSpeed ──────────────────────────────────────────────────────

    def _set_speed_cb(self, goal_handle):
        goal = goal_handle.request
        result = SetSpeed.Result()
        player_cfg = self.config["player"]

        if abs(goal.rpm - 33.0) < 1.0:
            servo2_us = player_cfg["speed_33_pulse_us"]
        elif abs(goal.rpm - 45.0) < 1.0:
            servo2_us = player_cfg["speed_45_pulse_us"]
        else:
            result.success = False
            result.error_msg = f"Unsupported speed {goal.rpm} rpm (use 33 or 45)"
            goal_handle.abort()
            return result

        current: ServoState = self.node_states["player"]
        self.send_rpdo_servo("player", current.servo1_us, servo2_us)
        time.sleep(0.3)

        result.success = True
        goal_handle.succeed()
        return result


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = MotionCoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
