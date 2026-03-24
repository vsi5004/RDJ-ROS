"""
codec.py — Pure PDO codec for canopen_bridge. No ROS dependency.

Translates between:
  - COData accumulator: dict of (OD index, subindex) → uint32 value
    (populated from ProxyDriver's <node>/rpd topic, one entry per SYNC field)
  - Raw bytes: the 8-byte UInt8MultiArray packets motion_coordinator expects

OD indices verified against config/stepper_node.eds, a_axis_node.eds, servo_node.eds.
"""

import struct

# ── OD key constants ──────────────────────────────────────────────────────────
# Stepper / A-axis TPDO  (arrive on <node>/rpd from ProxyDriver)
OD_XACTUAL = (0x2101, 0)      # INT32  — actual position in microsteps
OD_STATUS = (0x2003, 0)       # UINT8  — status word bits
OD_RAMP_STATUS = (0x2105, 0)  # UINT8  — TMC5160 RAMPSTAT[7:0]
OD_TOF = (0x2107, 0)          # UINT16 — ToF distance mm  (X, Z axes)
OD_POT_ANGLE = (0x2200, 0)    # INT16  — pot angle 0.1° units, signed  (A axis)

# Stepper / A-axis RPDO  (sent to <node>/tpd on ProxyDriver)
OD_XTARGET = (0x2100, 0)      # INT32  — target position in microsteps
OD_VMAX = (0x2102, 0)         # UINT16 — max velocity pulses/s
OD_CTRL = (0x2004, 0)         # UINT8  — control word bits
OD_RAMP_MODE = (0x2108, 0)    # UINT8  — 0=position, 1=velocity+, 2=velocity-

# Servo TPDO  (arrive on <node>/rpd from ProxyDriver)
OD_SERVO1_ACT = (0x2300, 0)   # UINT16 — servo 1 actual pulse width µs
OD_SERVO2_ACT = (0x2301, 0)   # UINT16 — servo 2 actual pulse width µs
OD_SERVO_TOF = (0x2302, 0)    # UINT16 — ToF mm (Pincher only, 0xFFFF on Player)
OD_SERVO_STS = (0x2003, 0)    # UINT8  — status word (same OD as stepper)

# Servo RPDO  (sent to <node>/tpd on ProxyDriver)
OD_SERVO1_TGT = (0x2300, 0)   # UINT16 — servo 1 target pulse width µs (same OD, RW)
OD_SERVO2_TGT = (0x2301, 0)   # UINT16 — servo 2 target pulse width µs
OD_SERVO_CTRL = (0x2004, 0)   # UINT8  — control word

# Type alias for the accumulator
Acc = dict  # {(int, int): int}

# ── Safe defaults for missing accumulator entries ──────────────────────────────
_TOF_DEFAULT = 0xFFFF
_DEFAULT = 0


def _get(acc: Acc, key: tuple, default: int = _DEFAULT) -> int:
    return acc.get(key, default)


# ── TPDO assembly: accumulator → bytes ────────────────────────────────────────

def assemble_stepper_tpdo(acc: Acc) -> bytes:
    """Pack COData accumulator → 8-byte stepper TPDO (X or Z axis).

    Layout: [INT32 XACTUAL | UINT8 status | UINT8 ramp_status | UINT16 ToF]
    """
    xactual = _get(acc, OD_XACTUAL)
    # Accumulator stores uint32; reinterpret as signed INT32 for packing.
    xactual_signed = struct.unpack("<i", struct.pack("<I", xactual & 0xFFFFFFFF))[0]
    status = _get(acc, OD_STATUS) & 0xFF
    ramp_status = _get(acc, OD_RAMP_STATUS) & 0xFF
    tof = _get(acc, OD_TOF, _TOF_DEFAULT) & 0xFFFF
    return struct.pack("<iBBH", xactual_signed, status, ramp_status, tof)


def assemble_a_axis_tpdo(acc: Acc) -> bytes:
    """Pack COData accumulator → 8-byte A-axis TPDO.

    Same as stepper but bytes 6-7 are INT16 signed pot angle instead of ToF.
    Layout: [INT32 XACTUAL | UINT8 status | UINT8 ramp_status | INT16 pot_angle]
    """
    xactual = _get(acc, OD_XACTUAL)
    xactual_signed = struct.unpack("<i", struct.pack("<I", xactual & 0xFFFFFFFF))[0]
    status = _get(acc, OD_STATUS) & 0xFF
    ramp_status = _get(acc, OD_RAMP_STATUS) & 0xFF
    pot_raw = _get(acc, OD_POT_ANGLE)
    # Accumulator stores uint32; reinterpret as signed INT16.
    pot_signed = struct.unpack("<h", struct.pack("<H", pot_raw & 0xFFFF))[0]
    return struct.pack("<iBBh", xactual_signed, status, ramp_status, pot_signed)


def assemble_servo_tpdo(acc: Acc) -> bytes:
    """Pack COData accumulator → 7-byte servo TPDO.

    Layout: [UINT16 servo1 | UINT16 servo2 | UINT16 ToF | UINT8 status]
    """
    servo1 = _get(acc, OD_SERVO1_ACT) & 0xFFFF
    servo2 = _get(acc, OD_SERVO2_ACT) & 0xFFFF
    tof = _get(acc, OD_SERVO_TOF, _TOF_DEFAULT) & 0xFFFF
    status = _get(acc, OD_SERVO_STS) & 0xFF
    return struct.pack("<HHHB", servo1, servo2, tof, status)


# ── RPDO disassembly: bytes → COData list ─────────────────────────────────────

def disassemble_stepper_rpdo(raw: bytes) -> list:
    """Unpack 8-byte stepper/A-axis RPDO → list of {index, subindex, data} dicts.

    Returns 4 entries: XTARGET, VMAX, CTRL, RAMP_MODE.
    INT32 XTARGET is reinterpreted as unsigned for COData.data (uint32).
    """
    xtarget_signed = struct.unpack_from("<i", raw, 0)[0]
    xtarget_unsigned = struct.unpack("<I", struct.pack("<i", xtarget_signed))[0]
    vmax = struct.unpack_from("<H", raw, 4)[0]
    ctrl = raw[6]
    ramp_mode = raw[7]
    return [
        {"index": OD_XTARGET[0],    "subindex": OD_XTARGET[1],    "data": xtarget_unsigned},
        {"index": OD_VMAX[0],       "subindex": OD_VMAX[1],       "data": vmax},
        {"index": OD_CTRL[0],       "subindex": OD_CTRL[1],       "data": ctrl},
        {"index": OD_RAMP_MODE[0],  "subindex": OD_RAMP_MODE[1],  "data": ramp_mode},
    ]


def disassemble_servo_rpdo(raw: bytes) -> list:
    """Unpack servo RPDO bytes 0-4 → list of {index, subindex, data} dicts.

    Returns 3 entries: servo1 target, servo2 target, control word.
    Bytes 5-7 are unmapped padding in the EDS and are ignored.
    """
    servo1 = struct.unpack_from("<H", raw, 0)[0]
    servo2 = struct.unpack_from("<H", raw, 2)[0]
    ctrl = raw[4]
    return [
        {"index": OD_SERVO1_TGT[0], "subindex": OD_SERVO1_TGT[1], "data": servo1},
        {"index": OD_SERVO2_TGT[0], "subindex": OD_SERVO2_TGT[1], "data": servo2},
        {"index": OD_SERVO_CTRL[0], "subindex": OD_SERVO_CTRL[1], "data": ctrl},
    ]
