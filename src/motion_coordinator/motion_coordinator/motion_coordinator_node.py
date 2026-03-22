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

import time
import math
import struct
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32, Bool, UInt8MultiArray

from vinyl_robot_msgs.msg import MotionStatus
from vinyl_robot_msgs.action import (
    MoveToPosition, HomeAll, Grip, FlipRecord, PressPlay, SetSpeed
)

from .can_interface import (
    StepperTPDO, AAxisTPDO, ServoTPDO,
    build_stepper_rpdo, build_servo_rpdo,
    CW_ENABLE, CW_HALT, CW_HOME, CW_CLEAR_FAULT,
    RAMP_POSITION, RAMP_VELOCITY_FWD, RAMP_VELOCITY_REV,
)
from .homing import run_homing


# ── Thin state container shared by TPDO subscribers ───────────────────────────

class StepperState:
    def __init__(self):
        self.actual_pos: int = 0       # microsteps
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
        self.pot_angle_raw: int = 0   # INT16, 0.1° units


class ServoState:
    def __init__(self):
        self.servo1_us: int = 1500
        self.servo2_us: int = 1500
        self.tof_mm: int = 0xFFFF
        self.status_word: int = 0


# ── Motion coordinator node ────────────────────────────────────────────────────

class MotionCoordinatorNode(Node):

    NODE_NAMES = ['x_axis', 'z_axis', 'a_axis', 'pincher', 'player']
    STEPPER_NAMES = ['x_axis', 'z_axis', 'a_axis']
    SERVO_NAMES = ['pincher', 'player']

    def __init__(self):
        super().__init__('motion_coordinator')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError('motion_coordinator: config_path parameter is required')

        with open(config_path) as f:
            self.config: dict = yaml.safe_load(f)
        self.get_logger().info(f'Loaded config: {config_path}')

        # ── Internal state ────────────────────────────────────────────────────
        self.node_states: dict = {
            'x_axis': StepperState(),
            'z_axis': StepperState(),
            'a_axis': AAxisState(),
            'pincher': ServoState(),
            'player': ServoState(),
        }
        self.safety_scale: float = 1.0
        self.estop: bool = False
        self._fault: bool = False
        self._fault_msg: str = ''

        cb = ReentrantCallbackGroup()

        # ── TPDO subscribers ──────────────────────────────────────────────────
        for name in self.NODE_NAMES:
            self.create_subscription(
                UInt8MultiArray,
                f'/canopen/{name}/tpdo1',
                lambda msg, n=name: self._on_tpdo(n, msg),
                10,
                callback_group=cb,
            )

        # ── Safety subscribers ────────────────────────────────────────────────
        self.create_subscription(
            Float32, '/safety/velocity_scale', self._on_velocity_scale, 10,
            callback_group=cb,
        )
        self.create_subscription(
            Bool, '/safety/estop', self._on_estop, 10,
            callback_group=cb,
        )

        # ── RPDO publishers ───────────────────────────────────────────────────
        self._rpdo_pubs: dict = {}
        for name in self.NODE_NAMES:
            self._rpdo_pubs[name] = self.create_publisher(
                UInt8MultiArray, f'/canopen/{name}/rpdo1', 10
            )

        # ── Status publisher ──────────────────────────────────────────────────
        self._status_pub = self.create_publisher(MotionStatus, '/motion/status', 10)
        self.create_timer(0.02, self._publish_status, callback_group=cb)

        # ── Action servers ────────────────────────────────────────────────────
        self._move_server = ActionServer(
            self, MoveToPosition, '/motion/move_to_position',
            execute_callback=self._move_to_position_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._home_server = ActionServer(
            self, HomeAll, '/motion/home_all',
            execute_callback=self._home_all_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._grip_server = ActionServer(
            self, Grip, '/motion/grip',
            execute_callback=self._grip_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._flip_server = ActionServer(
            self, FlipRecord, '/motion/flip_record',
            execute_callback=self._flip_record_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._play_server = ActionServer(
            self, PressPlay, '/motion/press_play',
            execute_callback=self._press_play_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._speed_server = ActionServer(
            self, SetSpeed, '/motion/set_speed',
            execute_callback=self._set_speed_cb,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )

        self.get_logger().info('motion_coordinator ready')

    # ── TPDO handlers ─────────────────────────────────────────────────────────

    def _on_tpdo(self, name: str, msg: UInt8MultiArray) -> None:
        data = bytes(msg.data)
        try:
            if name == 'x_axis' or name == 'z_axis':
                parsed = StepperTPDO.from_bytes(data)
                state: StepperState = self.node_states[name]
                state.actual_pos = parsed.actual_pos
                state.status_word = parsed.status_word
                state.ramp_status = parsed.ramp_status
                state.tof_mm = parsed.tof_mm
            elif name == 'a_axis':
                parsed = AAxisTPDO.from_bytes(data)
                state: AAxisState = self.node_states['a_axis']
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
                    self._fault_msg = f'{name} reports fault'
                    self.get_logger().error(self._fault_msg)
        except (ValueError, struct.error) as exc:
            self.get_logger().debug(f'Bad TPDO from {name}: {exc}')

    # ── Safety handlers ────────────────────────────────────────────────────────

    def _on_velocity_scale(self, msg: Float32) -> None:
        self.safety_scale = float(msg.data)

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and not self.estop:
            self.get_logger().error('E-STOP received — halting all axes')
            self._halt_all()
        self.estop = bool(msg.data)

    # ── Coordinate conversion helpers ─────────────────────────────────────────

    def mm_to_steps(self, axis_name: str, mm: float) -> int:
        key = axis_name.replace('_axis', '')
        steps_per_mm = self.config['geometry'][f'{key}_steps_per_mm']
        return int(round(mm * steps_per_mm))

    def deg_to_steps(self, deg: float) -> int:
        return int(round(deg * self.config['geometry']['a_steps_per_deg']))

    def steps_to_mm(self, axis_name: str, steps: int) -> float:
        key = axis_name.replace('_axis', '')
        steps_per_mm = self.config['geometry'][f'{key}_steps_per_mm']
        return steps / steps_per_mm

    def steps_to_deg(self, steps: int) -> float:
        return steps / self.config['geometry']['a_steps_per_deg']

    def mmps_to_vmax(self, axis_name: str, speed: float) -> int:
        """Convert mm/s (or deg/s for a_axis) to VMAX pps with safety scaling."""
        scale = max(0.0, min(1.0, self.safety_scale))
        if axis_name == 'a_axis':
            raw = speed * self.config['geometry']['a_steps_per_deg'] * scale
        else:
            key = axis_name.replace('_axis', '')
            raw = speed * self.config['geometry'][f'{key}_steps_per_mm'] * scale
        return max(1, min(int(raw), 65535))

    # ── Current position accessors ────────────────────────────────────────────

    def x_pos_mm(self) -> float:
        return self.steps_to_mm('x_axis', self.node_states['x_axis'].actual_pos)

    def z_pos_mm(self) -> float:
        return self.steps_to_mm('z_axis', self.node_states['z_axis'].actual_pos)

    def a_pos_deg(self) -> float:
        return self.steps_to_deg(self.node_states['a_axis'].actual_pos)

    def x_steps(self) -> int:
        return self.node_states['x_axis'].actual_pos

    def x_homed(self) -> bool:
        return self.node_states['x_axis'].homed

    def z_homed(self) -> bool:
        return self.node_states['z_axis'].homed

    def a_homed(self) -> bool:
        return self.node_states['a_axis'].homed

    # ── RPDO send helpers ─────────────────────────────────────────────────────

    def send_rpdo_stepper(self, name: str, target_steps: int, vmax: int,
                          ctrl_word: int, ramp_mode: int) -> None:
        if self.estop and not (ctrl_word & CW_HALT):
            return   # block all motion during e-stop except explicit halt
        data = build_stepper_rpdo(target_steps, vmax, ctrl_word, ramp_mode)
        msg = UInt8MultiArray()
        msg.data = list(data)
        self._rpdo_pubs[name].publish(msg)

    def send_rpdo_servo(self, name: str, servo1_us: int, servo2_us: int,
                        ctrl_word: int = CW_ENABLE) -> None:
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

        xs: StepperState = self.node_states['x_axis']
        zs: StepperState = self.node_states['z_axis']
        as_: AAxisState = self.node_states['a_axis']
        ps: ServoState = self.node_states['pincher']
        pl: ServoState = self.node_states['player']

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
            result.error_msg = 'E-stop active'
            goal_handle.abort()
            return result

        # Apply per-goal velocity scale on top of safety scale
        combined_scale = goal.velocity_scale * self.safety_scale

        if not goal.skip_x:
            vmax = int(goal.velocity_scale * self.mmps_to_vmax(
                'x_axis', self.config.get('max_velocity_mmps', {}).get('x', 200)))
            self.send_rpdo_stepper(
                'x_axis',
                target_steps=self.mm_to_steps('x_axis', goal.x_mm),
                vmax=self.mmps_to_vmax('x_axis', 200) if combined_scale > 0 else 1,
                ctrl_word=CW_ENABLE,
                ramp_mode=RAMP_POSITION,
            )
        if not goal.skip_z:
            self.send_rpdo_stepper(
                'z_axis',
                target_steps=self.mm_to_steps('z_axis', goal.z_mm),
                vmax=self.mmps_to_vmax('z_axis', 100),
                ctrl_word=CW_ENABLE,
                ramp_mode=RAMP_POSITION,
            )
        if not goal.skip_a:
            self.send_rpdo_stepper(
                'a_axis',
                target_steps=self.deg_to_steps(goal.a_deg),
                vmax=self.mmps_to_vmax('a_axis', 60),
                ctrl_word=CW_ENABLE,
                ramp_mode=RAMP_POSITION,
            )

        timeout = 60.0
        elapsed = 0.0
        while elapsed < timeout:
            if goal_handle.is_cancel_requested:
                self._halt_all()
                goal_handle.canceled()
                result.success = False
                result.error_msg = 'Cancelled'
                return result

            if self.estop:
                result.success = False
                result.error_msg = 'E-stop active'
                goal_handle.abort()
                return result

            xs = self.node_states['x_axis']
            zs = self.node_states['z_axis']
            as_ = self.node_states['a_axis']

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
        result.error_msg = '' if result.success else 'Move timeout'
        result.x_final_mm = self.x_pos_mm()
        result.z_final_mm = self.z_pos_mm()
        result.a_final_deg = self.a_pos_deg()

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── Action: HomeAll ───────────────────────────────────────────────────────

    def _home_all_cb(self, goal_handle):
        result = HomeAll.Result()
        try:
            success = run_homing(self, goal_handle)
        except Exception as exc:
            self.get_logger().error(f'Homing exception: {exc}')
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
        grip_cfg = self.config['grip']

        servo1_us = grip_cfg['close_pulse_us'] if goal.close else grip_cfg['open_pulse_us']
        servo2_us = 1500   # flip servo stays neutral

        self.send_rpdo_servo('pincher', servo1_us, servo2_us)
        time.sleep(0.5)   # wait for servo to reach position

        tof = float(self.node_states['pincher'].tof_mm)
        contact_mm = grip_cfg['tof_contact_mm']
        open_mm = grip_cfg['tof_open_mm']

        if goal.close:
            result.success = tof <= contact_mm
            result.error_msg = '' if result.success else f'Grip failed: ToF={tof:.0f}mm (expected <{contact_mm}mm)'
        else:
            result.success = tof >= open_mm
            result.error_msg = '' if result.success else f'Open failed: ToF={tof:.0f}mm (expected >{open_mm}mm)'

        result.tof_mm = tof
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── Action: FlipRecord ────────────────────────────────────────────────────

    def _flip_record_cb(self, goal_handle):
        result = FlipRecord.Result()
        flip_cfg = self.config['flip']

        # Flip servo (servo2) to side B position
        current_tpdo: ServoState = self.node_states['pincher']
        self.send_rpdo_servo('pincher', current_tpdo.servo1_us, flip_cfg['servo_pulse_b_us'])
        time.sleep(1.0)   # full flip takes ~1 second

        result.success = True
        goal_handle.succeed()
        return result

    # ── Action: PressPlay ─────────────────────────────────────────────────────

    def _press_play_cb(self, goal_handle):
        goal = goal_handle.request
        result = PressPlay.Result()
        player_cfg = self.config['player']

        servo1_us = player_cfg['play_pulse_us'] if goal.press else player_cfg['stop_pulse_us']
        self.send_rpdo_servo('player', servo1_us, 1500)
        time.sleep(0.3)

        result.success = True
        goal_handle.succeed()
        return result

    # ── Action: SetSpeed ──────────────────────────────────────────────────────

    def _set_speed_cb(self, goal_handle):
        goal = goal_handle.request
        result = SetSpeed.Result()
        player_cfg = self.config['player']

        if abs(goal.rpm - 33.0) < 1.0:
            servo2_us = player_cfg['speed_33_pulse_us']
        elif abs(goal.rpm - 45.0) < 1.0:
            servo2_us = player_cfg['speed_45_pulse_us']
        else:
            result.success = False
            result.error_msg = f'Unsupported speed {goal.rpm} rpm (use 33 or 45)'
            goal_handle.abort()
            return result

        current: ServoState = self.node_states['player']
        self.send_rpdo_servo('player', current.servo1_us, servo2_us)
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


if __name__ == '__main__':
    main()
