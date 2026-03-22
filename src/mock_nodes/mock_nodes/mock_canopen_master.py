"""
mock_canopen_master.py — Simulated CANopen bus for development without hardware.

Publishes TPDO frames at 50 Hz (matching SYNC period) and receives RPDO
commands from the motion coordinator.  Stepper axes ramp their XACTUAL toward
XTARGET at the commanded VMAX.  Homing is simulated by instantly zeroing
XACTUAL and setting the homed bit.

Topic interface is identical to what the real ros2_canopen ProxyDriver exposes,
so the motion_coordinator can run unchanged against real or simulated hardware.

Topics:
  Publishes: /canopen/<name>/tpdo1  (UInt8MultiArray, 8 bytes, 50 Hz)
             /canopen/<name>/nmt_state  (String, "operational")
  Subscribes: /canopen/<name>/rpdo1 (UInt8MultiArray, 8 bytes)
"""

import math
import struct
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

# ── Control word bits ─────────────────────────────────────────────────────────
CW_ENABLE = 1 << 0
CW_HOME = 1 << 1
CW_HALT = 1 << 2
CW_CLEAR_FAULT = 1 << 3

# ── Status word bits ──────────────────────────────────────────────────────────
SW_HOMED = 1 << 0
SW_MOVING = 1 << 1
SW_IN_POSITION = 1 << 2
SW_FAULT = 1 << 3
SW_HOMING = 1 << 4

# ── Ramp modes ────────────────────────────────────────────────────────────────
RAMP_POSITION = 0
RAMP_VELOCITY_FWD = 1
RAMP_VELOCITY_REV = 2

SYNC_DT = 0.02   # 20 ms per SYNC cycle


class MockStepperNode:
    """Simulates one stepper axis (X, Z, or A)."""

    def __init__(self, name: str, steps_per_unit: float, initial_pos: int = 1000):
        self.name = name
        self.steps_per_unit = steps_per_unit
        self.actual_pos: int = initial_pos    # Start mid-range (not at endstop)
        self.target_pos: int = initial_pos
        self.vmax: int = 500                  # pps
        self.ramp_mode: int = RAMP_POSITION
        self.ctrl_word: int = 0
        self.homed: bool = False
        self.fault: bool = False
        self.pot_angle_raw: int = 0           # A axis only

    def apply_rpdo(self, data: bytes) -> None:
        """Parse an 8-byte stepper RPDO and update command state."""
        if len(data) < 8:
            return
        target = struct.unpack_from('<i', data, 0)[0]
        vmax = struct.unpack_from('<H', data, 4)[0]
        ctrl = data[6]
        ramp_mode = data[7]

        if ctrl & CW_CLEAR_FAULT:
            self.fault = False

        if ctrl & CW_HALT:
            self.target_pos = self.actual_pos
        else:
            self.target_pos = target

        if vmax > 0:
            self.vmax = vmax

        self.ramp_mode = ramp_mode
        self.ctrl_word = ctrl

        if ctrl & CW_HOME:
            self._start_homing()

    def _start_homing(self) -> None:
        """Simulate homing: ramp to zero and set homed bit."""
        self.target_pos = 0
        # homed bit will be set when actual_pos reaches 0

    def tick(self, dt: float) -> None:
        """Advance simulation by one SYNC tick."""
        if self.fault:
            return

        max_step = max(1, int(self.vmax * dt))

        if self.ramp_mode == RAMP_POSITION:
            delta = self.target_pos - self.actual_pos
            if abs(delta) <= max_step:
                self.actual_pos = self.target_pos
            else:
                self.actual_pos += max_step * (1 if delta > 0 else -1)
        elif self.ramp_mode == RAMP_VELOCITY_FWD:
            self.actual_pos += max_step
        elif self.ramp_mode == RAMP_VELOCITY_REV:
            self.actual_pos -= max_step

        # Simulate homing completion when reaching zero
        if self.actual_pos == 0 and (self.ctrl_word & CW_HOME):
            self.homed = True

        # Update pot angle for A axis (pot_angle_raw in 0.1° units)
        if self.name == 'a_axis':
            self.pot_angle_raw = int(self.actual_pos / self.steps_per_unit * 10)

    def build_tpdo(self, is_a_axis: bool = False) -> bytes:
        """Assemble the 8-byte TPDO from current simulated state."""
        in_pos = (self.actual_pos == self.target_pos) and self.ramp_mode == RAMP_POSITION
        moving = not in_pos or self.ramp_mode != RAMP_POSITION

        status = 0
        if self.homed:
            status |= SW_HOMED
        if moving:
            status |= SW_MOVING
        if in_pos:
            status |= SW_IN_POSITION
        if self.fault:
            status |= SW_FAULT

        data = struct.pack('<i', self.actual_pos)   # INT32 actual pos
        data += bytes([status, 0])                   # status word, ramp status

        if is_a_axis:
            # Bytes 6-7 are pot angle (INT16)
            data += struct.pack('<h', self.pot_angle_raw)
        else:
            # Bytes 6-7 are ToF distance.
            # Simulate a plausible ToF based on position (not physically accurate)
            tof = 500
            data += struct.pack('<H', tof)

        return data


class MockServoNode:
    """Simulates one servo node (Pincher or Player)."""

    def __init__(self, name: str):
        self.name = name
        self.servo1_us: int = 1500
        self.servo2_us: int = 1500
        self.tof_mm: int = 100    # Pincher ToF — simulates open gripper distance

    def apply_rpdo(self, data: bytes) -> None:
        if len(data) < 5:
            return
        self.servo1_us = struct.unpack_from('<H', data, 0)[0]
        self.servo2_us = struct.unpack_from('<H', data, 2)[0]

        # Simulate Pincher ToF based on grip position
        if self.name == 'pincher':
            grip_cfg_open = 2000   # open pulse — matches robot_params.yaml
            self.tof_mm = 10 if self.servo1_us < grip_cfg_open - 100 else 80

    def build_tpdo(self) -> bytes:
        data = struct.pack('<H', self.servo1_us)
        data += struct.pack('<H', self.servo2_us)
        data += struct.pack('<H', self.tof_mm if self.name == 'pincher' else 0)
        data += bytes([SW_HOMED | SW_IN_POSITION])   # servos are always "in position"
        return data


class MockCANopenMaster(Node):

    def __init__(self):
        super().__init__('mock_canopen_master')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        if config_path:
            with open(config_path) as f:
                cfg = yaml.safe_load(f)
            geo = cfg.get('geometry', {})
        else:
            geo = {
                'x_steps_per_mm': 80.0,
                'z_steps_per_mm': 400.0,
                'a_steps_per_deg': 142.2,
            }

        # ── Simulated nodes ───────────────────────────────────────────────────
        # Start X and Z mid-range, A at ~180° (park position)
        self._steppers: dict[str, MockStepperNode] = {
            'x_axis': MockStepperNode('x_axis', geo['x_steps_per_mm'],
                                      initial_pos=int(500 * geo['x_steps_per_mm'])),
            'z_axis': MockStepperNode('z_axis', geo['z_steps_per_mm'],
                                      initial_pos=int(80 * geo['z_steps_per_mm'])),
            'a_axis': MockStepperNode('a_axis', geo['a_steps_per_deg'],
                                      initial_pos=int(180 * geo['a_steps_per_deg'])),
        }
        self._servos: dict[str, MockServoNode] = {
            'pincher': MockServoNode('pincher'),
            'player': MockServoNode('player'),
        }

        # ── Publishers ────────────────────────────────────────────────────────
        self._tpdo_pubs: dict = {}
        self._nmt_pubs: dict = {}
        all_names = list(self._steppers.keys()) + list(self._servos.keys())
        for name in all_names:
            self._tpdo_pubs[name] = self.create_publisher(
                UInt8MultiArray, f'/canopen/{name}/tpdo1', 10
            )
            self._nmt_pubs[name] = self.create_publisher(
                String, f'/canopen/{name}/nmt_state', 10
            )

        # ── Subscribers ───────────────────────────────────────────────────────
        for name in self._steppers:
            self.create_subscription(
                UInt8MultiArray,
                f'/canopen/{name}/rpdo1',
                lambda msg, n=name: self._on_stepper_rpdo(n, msg),
                10,
            )
        for name in self._servos:
            self.create_subscription(
                UInt8MultiArray,
                f'/canopen/{name}/rpdo1',
                lambda msg, n=name: self._on_servo_rpdo(n, msg),
                10,
            )

        # ── 50 Hz SYNC timer ──────────────────────────────────────────────────
        self.create_timer(SYNC_DT, self._sync_tick)
        self.get_logger().info('mock_canopen_master running (50 Hz simulation)')

    def _on_stepper_rpdo(self, name: str, msg: UInt8MultiArray) -> None:
        self._steppers[name].apply_rpdo(bytes(msg.data))

    def _on_servo_rpdo(self, name: str, msg: UInt8MultiArray) -> None:
        self._servos[name].apply_rpdo(bytes(msg.data))

    def _sync_tick(self) -> None:
        """Simulate one 20 ms SYNC cycle: advance physics, publish TPDOs."""

        # Advance steppers
        for name, node in self._steppers.items():
            node.tick(SYNC_DT)

        # Publish stepper TPDOs
        for name, node in self._steppers.items():
            is_a = (name == 'a_axis')
            data = node.build_tpdo(is_a_axis=is_a)
            msg = UInt8MultiArray()
            msg.data = list(data)
            self._tpdo_pubs[name].publish(msg)

        # Publish servo TPDOs
        for name, node in self._servos.items():
            data = node.build_tpdo()
            msg = UInt8MultiArray()
            msg.data = list(data)
            self._tpdo_pubs[name].publish(msg)

        # NMT state — all nodes operational
        nmt_msg = String()
        nmt_msg.data = 'operational'
        for pub in self._nmt_pubs.values():
            pub.publish(nmt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCANopenMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
