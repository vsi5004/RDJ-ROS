"""
PDO parsing and assembly for the RDJ vinyl robot CAN bus.

All PDO layouts match the EDS files exactly:

STEPPER TPDO1 (8 bytes, node → master on every SYNC):
  [0:4]  INT32   XACTUAL (microsteps)
  [4]    UINT8   Status word (0x2003)
  [5]    UINT8   Ramp status (0x2105)
  [6:8]  UINT16  ToF distance mm (0x2107)  [X, Z only — 0xFFFF on A]

A-AXIS TPDO1 (8 bytes):
  [0:4]  INT32   XACTUAL (microsteps)
  [4]    UINT8   Status word
  [5]    UINT8   Ramp status
  [6:8]  INT16   Pot angle (0.1° units, signed) — A axis only

STEPPER RPDO1 (8 bytes, master → node on command):
  [0:4]  INT32   XTARGET (microsteps)
  [4:6]  UINT16  VMAX (pulses/s, 0 = keep current)
  [6]    UINT8   Control word (0x2004)
  [7]    UINT8   Ramp mode (0=position, 1=vel+, 2=vel-)

SERVO TPDO1 (7 bytes):
  [0:2]  UINT16  Servo 1 actual (µs)
  [2:4]  UINT16  Servo 2 actual (µs)
  [4:6]  UINT16  ToF mm (Pincher only, 0 on Player)
  [6]    UINT8   Status word

SERVO RPDO1 (8 bytes):
  [0:2]  UINT16  Servo 1 target (µs)
  [2:4]  UINT16  Servo 2 target (µs)
  [4]    UINT8   Control word
  [5:8]  padding / reserved

Status word bit definitions (0x2003):
  bit 0  homed
  bit 1  moving
  bit 2  in_position
  bit 3  fault
  bit 4  homing
  bit 5  stallguard_active

Control word bit definitions (0x2004):
  bit 0  enable
  bit 1  home
  bit 2  halt
  bit 3  clear_fault
"""

import struct

# ── Status word bits ──────────────────────────────────────────────────────────
SW_HOMED = 1 << 0
SW_MOVING = 1 << 1
SW_IN_POSITION = 1 << 2
SW_FAULT = 1 << 3
SW_HOMING = 1 << 4
SW_STALLGUARD = 1 << 5

# ── Control word bits ─────────────────────────────────────────────────────────
CW_ENABLE = 1 << 0
CW_HOME = 1 << 1
CW_HALT = 1 << 2
CW_CLEAR_FAULT = 1 << 3

# ── Ramp modes ────────────────────────────────────────────────────────────────
RAMP_POSITION = 0
RAMP_VELOCITY_FWD = 1   # velocity+ (increasing position)
RAMP_VELOCITY_REV = 2   # velocity- (decreasing position)


class StepperTPDO:
    """Parsed TPDO1 from a stepper node (X or Z axis)."""

    __slots__ = ('actual_pos', 'status_word', 'ramp_status', 'tof_mm')

    def __init__(self, actual_pos: int = 0, status_word: int = 0,
                 ramp_status: int = 0, tof_mm: int = 0xFFFF):
        self.actual_pos = actual_pos       # INT32  microsteps
        self.status_word = status_word     # UINT8
        self.ramp_status = ramp_status     # UINT8
        self.tof_mm = tof_mm               # UINT16

    @classmethod
    def from_bytes(cls, data: bytes) -> 'StepperTPDO':
        if len(data) < 8:
            raise ValueError(f'StepperTPDO needs 8 bytes, got {len(data)}')
        actual_pos = struct.unpack_from('<i', data, 0)[0]
        status_word = data[4]
        ramp_status = data[5]
        tof_mm = struct.unpack_from('<H', data, 6)[0]
        return cls(actual_pos, status_word, ramp_status, tof_mm)

    @property
    def homed(self) -> bool:
        return bool(self.status_word & SW_HOMED)

    @property
    def moving(self) -> bool:
        return bool(self.status_word & SW_MOVING)

    @property
    def in_position(self) -> bool:
        return bool(self.status_word & SW_IN_POSITION)

    @property
    def fault(self) -> bool:
        return bool(self.status_word & SW_FAULT)


class AAxisTPDO:
    """Parsed TPDO1 from the A axis node — same as stepper but byte 6-7 is pot angle."""

    __slots__ = ('actual_pos', 'status_word', 'ramp_status', 'pot_angle_raw')

    def __init__(self, actual_pos: int = 0, status_word: int = 0,
                 ramp_status: int = 0, pot_angle_raw: int = 0):
        self.actual_pos = actual_pos
        self.status_word = status_word
        self.ramp_status = ramp_status
        self.pot_angle_raw = pot_angle_raw   # INT16, 0.1° units

    @classmethod
    def from_bytes(cls, data: bytes) -> 'AAxisTPDO':
        if len(data) < 8:
            raise ValueError(f'AAxisTPDO needs 8 bytes, got {len(data)}')
        actual_pos = struct.unpack_from('<i', data, 0)[0]
        status_word = data[4]
        ramp_status = data[5]
        pot_angle_raw = struct.unpack_from('<h', data, 6)[0]   # signed INT16
        return cls(actual_pos, status_word, ramp_status, pot_angle_raw)

    @property
    def pot_angle_deg(self) -> float:
        """Coarse absolute angle in degrees (0.1° resolution)."""
        return self.pot_angle_raw / 10.0

    @property
    def homed(self) -> bool:
        return bool(self.status_word & SW_HOMED)

    @property
    def moving(self) -> bool:
        return bool(self.status_word & SW_MOVING)

    @property
    def in_position(self) -> bool:
        return bool(self.status_word & SW_IN_POSITION)

    @property
    def fault(self) -> bool:
        return bool(self.status_word & SW_FAULT)


class ServoTPDO:
    """Parsed TPDO1 from a servo node (Pincher or Player)."""

    __slots__ = ('servo1_us', 'servo2_us', 'tof_mm', 'status_word')

    def __init__(self, servo1_us: int = 1500, servo2_us: int = 1500,
                 tof_mm: int = 0xFFFF, status_word: int = 0):
        self.servo1_us = servo1_us
        self.servo2_us = servo2_us
        self.tof_mm = tof_mm
        self.status_word = status_word

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ServoTPDO':
        if len(data) < 7:
            raise ValueError(f'ServoTPDO needs 7 bytes, got {len(data)}')
        servo1_us = struct.unpack_from('<H', data, 0)[0]
        servo2_us = struct.unpack_from('<H', data, 2)[0]
        tof_mm = struct.unpack_from('<H', data, 4)[0]
        status_word = data[6]
        return cls(servo1_us, servo2_us, tof_mm, status_word)


def build_stepper_rpdo(target_steps: int, vmax: int,
                       ctrl_word: int, ramp_mode: int) -> bytes:
    """Assemble an 8-byte RPDO1 for a stepper node."""
    data = struct.pack('<i', target_steps)      # INT32
    data += struct.pack('<H', min(vmax, 65535))  # UINT16
    data += bytes([ctrl_word & 0xFF])            # UINT8
    data += bytes([ramp_mode & 0xFF])            # UINT8
    return data


def build_servo_rpdo(servo1_us: int, servo2_us: int,
                     ctrl_word: int = CW_ENABLE) -> bytes:
    """Assemble an 8-byte RPDO1 for a servo node."""
    data = struct.pack('<H', servo1_us)   # UINT16
    data += struct.pack('<H', servo2_us)  # UINT16
    data += bytes([ctrl_word & 0xFF])     # UINT8
    data += b'\x00\x00\x00'              # padding to 8 bytes
    return data
