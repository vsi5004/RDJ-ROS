"""
test_codec.py — TDD tests for canopen_bridge.codec

All tests are written before implementation. Every test should fail with
NotImplementedError until codec.py is implemented.

Cross-check helpers from existing packages validate round-trips:
  - motion_coordinator.can_interface: StepperTPDO, AAxisTPDO, ServoTPDO,
                                       build_stepper_rpdo, build_servo_rpdo,
                                       SW_HOMED, SW_IN_POSITION, CW_ENABLE
  - mock_nodes.mock_canopen_master:    MockStepperNode, MockServoNode
"""

import struct

import pytest
from motion_coordinator.can_interface import (
    SW_HOMED,
    SW_IN_POSITION,
    SW_MOVING,
    AAxisTPDO,
    CW_ENABLE,
    ServoTPDO,
    StepperTPDO,
    build_servo_rpdo,
    build_stepper_rpdo,
)
from mock_nodes.mock_canopen_master import MockServoNode, MockStepperNode

from canopen_bridge.codec import (
    OD_CTRL,
    OD_POT_ANGLE,
    OD_RAMP_MODE,
    OD_RAMP_STATUS,
    OD_SERVO1_ACT,
    OD_SERVO2_ACT,
    OD_SERVO_STS,
    OD_SERVO_TOF,
    OD_STATUS,
    OD_TOF,
    OD_VMAX,
    OD_XACTUAL,
    OD_XTARGET,
    assemble_a_axis_tpdo,
    assemble_servo_tpdo,
    assemble_stepper_tpdo,
    disassemble_servo_rpdo,
    disassemble_stepper_rpdo,
)


# ── Group 1: Stepper TPDO assembly ───────────────────────────────────────────

class TestStepperTPDOAssembly:

    def test_stepper_tpdo_is_8_bytes(self):
        result = assemble_stepper_tpdo({})
        assert len(result) == 8

    def test_stepper_tpdo_all_fields_correct_layout(self):
        acc = {
            OD_XACTUAL: 12345,
            OD_STATUS: SW_HOMED,
            OD_RAMP_STATUS: 0,
            OD_TOF: 500,
        }
        result = assemble_stepper_tpdo(acc)

        assert struct.unpack_from("<i", result, 0)[0] == 12345
        assert result[4] == SW_HOMED
        assert result[5] == 0
        assert struct.unpack_from("<H", result, 6)[0] == 500

    def test_stepper_tpdo_negative_xactual_correct_int32(self):
        neg_val = -99999
        acc = {OD_XACTUAL: neg_val & 0xFFFFFFFF}
        result = assemble_stepper_tpdo(acc)
        assert struct.unpack_from("<i", result, 0)[0] == neg_val

    def test_stepper_tpdo_missing_acc_uses_defaults(self):
        result = assemble_stepper_tpdo({})
        assert len(result) == 8  # no exception, sane length

    def test_stepper_tpdo_tof_default_is_0xffff_when_missing(self):
        result = assemble_stepper_tpdo({})
        assert struct.unpack_from("<H", result, 6)[0] == 0xFFFF

    def test_stepper_tpdo_round_trip_matches_mock(self):
        node = MockStepperNode("x_axis", 80.0)
        node.actual_pos = 5000
        node.target_pos = 5000  # in-position
        node.homed = True
        expected = node.build_tpdo(is_a_axis=False)

        status = expected[4]
        ramp_status = expected[5]
        tof = struct.unpack_from("<H", expected, 6)[0]

        acc = {
            OD_XACTUAL: 5000,
            OD_STATUS: status,
            OD_RAMP_STATUS: ramp_status,
            OD_TOF: tof,
        }
        assert assemble_stepper_tpdo(acc) == expected

    def test_stepper_tpdo_parseable_by_motion_coordinator(self):
        acc = {
            OD_XACTUAL: 12345,
            OD_STATUS: SW_HOMED | SW_IN_POSITION,
            OD_RAMP_STATUS: 3,
            OD_TOF: 750,
        }
        result = assemble_stepper_tpdo(acc)
        parsed = StepperTPDO.from_bytes(result)

        assert parsed.actual_pos == 12345
        assert parsed.status_word == (SW_HOMED | SW_IN_POSITION)
        assert parsed.ramp_status == 3
        assert parsed.tof_mm == 750


# ── Group 2: A-axis TPDO assembly ────────────────────────────────────────────

class TestAAxisTPDOAssembly:

    def test_a_axis_tpdo_is_8_bytes(self):
        result = assemble_a_axis_tpdo({})
        assert len(result) == 8

    def test_a_axis_tpdo_bytes_6_7_are_signed_pot_angle_negative(self):
        acc = {OD_XACTUAL: 0, OD_STATUS: 0, OD_RAMP_STATUS: 0, OD_POT_ANGLE: -300 & 0xFFFF}
        result = assemble_a_axis_tpdo(acc)
        assert struct.unpack_from("<h", result, 6)[0] == -300

    def test_a_axis_tpdo_bytes_6_7_are_signed_pot_angle_positive(self):
        acc = {OD_XACTUAL: 0, OD_STATUS: 0, OD_RAMP_STATUS: 0, OD_POT_ANGLE: 1800}
        result = assemble_a_axis_tpdo(acc)
        assert struct.unpack_from("<h", result, 6)[0] == 1800

    def test_a_axis_tpdo_common_fields_match_stepper(self):
        # bytes 0-5 are identical to stepper TPDO
        acc = {
            OD_XACTUAL: 7777,
            OD_STATUS: SW_HOMED,
            OD_RAMP_STATUS: 2,
            OD_POT_ANGLE: 900,
        }
        result = assemble_a_axis_tpdo(acc)
        assert struct.unpack_from("<i", result, 0)[0] == 7777
        assert result[4] == SW_HOMED
        assert result[5] == 2

    def test_a_axis_tpdo_round_trip_matches_mock(self):
        node = MockStepperNode("a_axis", 142.2)
        node.actual_pos = 2000
        node.target_pos = 2000
        node.homed = True
        node.pot_angle_raw = 140  # 14.0 degrees
        expected = node.build_tpdo(is_a_axis=True)

        status = expected[4]
        ramp_status = expected[5]
        pot_raw = struct.unpack_from("<h", expected, 6)[0]

        acc = {
            OD_XACTUAL: 2000,
            OD_STATUS: status,
            OD_RAMP_STATUS: ramp_status,
            OD_POT_ANGLE: pot_raw & 0xFFFF,
        }
        assert assemble_a_axis_tpdo(acc) == expected

    def test_a_axis_tpdo_parseable_by_motion_coordinator(self):
        acc = {
            OD_XACTUAL: 3000,
            OD_STATUS: SW_HOMED | SW_IN_POSITION,
            OD_RAMP_STATUS: 0,
            OD_POT_ANGLE: (-150) & 0xFFFF,
        }
        result = assemble_a_axis_tpdo(acc)
        parsed = AAxisTPDO.from_bytes(result)

        assert parsed.actual_pos == 3000
        assert parsed.status_word == (SW_HOMED | SW_IN_POSITION)
        assert parsed.ramp_status == 0
        assert parsed.pot_angle_raw == -150


# ── Group 3: Servo TPDO assembly ─────────────────────────────────────────────

class TestServoTPDOAssembly:

    def test_servo_tpdo_is_7_bytes(self):
        result = assemble_servo_tpdo({})
        assert len(result) == 7

    def test_servo_tpdo_all_fields_correct_layout(self):
        acc = {
            OD_SERVO1_ACT: 1000,
            OD_SERVO2_ACT: 2000,
            OD_SERVO_TOF: 150,
            OD_SERVO_STS: SW_HOMED | SW_IN_POSITION,
        }
        result = assemble_servo_tpdo(acc)

        assert struct.unpack_from("<H", result, 0)[0] == 1000
        assert struct.unpack_from("<H", result, 2)[0] == 2000
        assert struct.unpack_from("<H", result, 4)[0] == 150
        assert result[6] == (SW_HOMED | SW_IN_POSITION)

    def test_servo_tpdo_missing_tof_defaults_to_0xffff(self):
        acc = {OD_SERVO1_ACT: 1500, OD_SERVO2_ACT: 1500, OD_SERVO_STS: 0}
        result = assemble_servo_tpdo(acc)
        assert struct.unpack_from("<H", result, 4)[0] == 0xFFFF

    def test_servo_tpdo_round_trip_matches_mock_pincher(self):
        node = MockServoNode("pincher")
        node.servo1_us = 1200
        node.servo2_us = 1800
        node.tof_mm = 75
        expected = node.build_tpdo()

        status = expected[6]
        acc = {
            OD_SERVO1_ACT: 1200,
            OD_SERVO2_ACT: 1800,
            OD_SERVO_TOF: 75,
            OD_SERVO_STS: status,
        }
        assert assemble_servo_tpdo(acc) == expected

    def test_servo_tpdo_parseable_by_motion_coordinator(self):
        acc = {
            OD_SERVO1_ACT: 1100,
            OD_SERVO2_ACT: 1900,
            OD_SERVO_TOF: 200,
            OD_SERVO_STS: SW_HOMED | SW_IN_POSITION,
        }
        result = assemble_servo_tpdo(acc)
        parsed = ServoTPDO.from_bytes(result)

        assert parsed.servo1_us == 1100
        assert parsed.servo2_us == 1900
        assert parsed.tof_mm == 200
        assert parsed.status_word == (SW_HOMED | SW_IN_POSITION)


# ── Group 4: Stepper RPDO disassembly ────────────────────────────────────────

def _find_entry(entries: list, index: int) -> dict:
    """Helper: find COData dict by OD index."""
    for e in entries:
        if e["index"] == index:
            return e
    raise KeyError(f"No entry with index=0x{index:04X}")


class TestStepperRPDODisassembly:

    def test_stepper_rpdo_disassemble_produces_4_entries(self):
        raw = build_stepper_rpdo(9999, 500, CW_ENABLE, 0)
        entries = disassemble_stepper_rpdo(raw)
        assert len(entries) == 4

    def test_stepper_rpdo_disassemble_entry_schema(self):
        raw = build_stepper_rpdo(0, 0, 0, 0)
        entries = disassemble_stepper_rpdo(raw)
        for e in entries:
            assert "index" in e
            assert "subindex" in e
            assert "data" in e

    def test_stepper_rpdo_disassemble_xtarget_correct(self):
        raw = build_stepper_rpdo(9999, 500, CW_ENABLE, 0)
        entries = disassemble_stepper_rpdo(raw)
        e = _find_entry(entries, 0x2100)
        assert e["subindex"] == 0
        assert e["data"] == 9999

    def test_stepper_rpdo_disassemble_xtarget_negative_as_unsigned(self):
        # INT32 -5 → stored as unsigned 0xFFFFFFFB in COData.data
        raw = build_stepper_rpdo(-5, 500, CW_ENABLE, 0)
        entries = disassemble_stepper_rpdo(raw)
        e = _find_entry(entries, 0x2100)
        assert e["data"] == ((-5) & 0xFFFFFFFF)

    def test_stepper_rpdo_disassemble_vmax_correct(self):
        raw = build_stepper_rpdo(0, 1234, CW_ENABLE, 0)
        entries = disassemble_stepper_rpdo(raw)
        e = _find_entry(entries, 0x2102)
        assert e["subindex"] == 0
        assert e["data"] == 1234

    def test_stepper_rpdo_disassemble_ctrl_correct(self):
        raw = build_stepper_rpdo(0, 500, CW_ENABLE, 0)
        entries = disassemble_stepper_rpdo(raw)
        e = _find_entry(entries, 0x2004)
        assert e["data"] == CW_ENABLE

    def test_stepper_rpdo_disassemble_ramp_mode_correct(self):
        raw = build_stepper_rpdo(0, 500, CW_ENABLE, 2)
        entries = disassemble_stepper_rpdo(raw)
        e = _find_entry(entries, 0x2108)
        assert e["data"] == 2

    def test_stepper_rpdo_disassemble_ramp_byte_dmax_factor_preserved(self):
        # OD 0x2108 byte 7 carries a bitfield: bits[1:0]=ramp_mode, bits[7:2]=dmax_factor.
        # The codec is a pass-through — it must preserve the full byte, not just bits[1:0].
        # Firmware (app_canopen.c) decodes the bitfield; codec must not strip the upper bits.
        ramp_byte = (4 << 2) | 0  # dmax_factor=4, ramp_mode=position → 0x10
        raw = build_stepper_rpdo(0, 500, CW_ENABLE, ramp_byte)
        entries = disassemble_stepper_rpdo(raw)
        e = _find_entry(entries, 0x2108)
        assert e["data"] == ramp_byte

    def test_stepper_rpdo_disassemble_round_trip(self):
        """build_stepper_rpdo → disassemble → re-pack → equals original bytes."""
        original = build_stepper_rpdo(12000, 800, CW_ENABLE, 1)
        entries = disassemble_stepper_rpdo(original)

        xtarget = _find_entry(entries, 0x2100)["data"]
        vmax = _find_entry(entries, 0x2102)["data"]
        ctrl = _find_entry(entries, 0x2004)["data"]
        ramp = _find_entry(entries, 0x2108)["data"]

        # Reinterpret unsigned xtarget back to signed INT32 for rebuild
        xtarget_signed = struct.unpack("<i", struct.pack("<I", xtarget))[0]
        rebuilt = build_stepper_rpdo(xtarget_signed, vmax, ctrl, ramp)
        assert rebuilt == original


# ── Group 5: Servo RPDO disassembly ──────────────────────────────────────────

class TestServoRPDODisassembly:

    def test_servo_rpdo_disassemble_produces_3_entries(self):
        # bytes 5-7 are unmapped padding — only 3 COData dicts expected
        raw = build_servo_rpdo(1200, 1800, CW_ENABLE)
        entries = disassemble_servo_rpdo(raw)
        assert len(entries) == 3

    def test_servo_rpdo_disassemble_entry_schema(self):
        raw = build_servo_rpdo(1500, 1500, CW_ENABLE)
        entries = disassemble_servo_rpdo(raw)
        for e in entries:
            assert "index" in e
            assert "subindex" in e
            assert "data" in e

    def test_servo_rpdo_disassemble_servo1_correct(self):
        raw = build_servo_rpdo(1200, 1800, CW_ENABLE)
        entries = disassemble_servo_rpdo(raw)
        e = _find_entry(entries, 0x2300)
        assert e["subindex"] == 0
        assert e["data"] == 1200

    def test_servo_rpdo_disassemble_servo2_correct(self):
        raw = build_servo_rpdo(1200, 1800, CW_ENABLE)
        entries = disassemble_servo_rpdo(raw)
        e = _find_entry(entries, 0x2301)
        assert e["subindex"] == 0
        assert e["data"] == 1800

    def test_servo_rpdo_disassemble_ctrl_correct(self):
        raw = build_servo_rpdo(1200, 1800, CW_ENABLE)
        entries = disassemble_servo_rpdo(raw)
        e = _find_entry(entries, 0x2004)
        assert e["subindex"] == 0
        assert e["data"] == CW_ENABLE

    def test_servo_rpdo_disassemble_padding_bytes_ignored(self):
        """Padding bytes 5-7 must not produce extra COData entries."""
        # Send non-zero padding to confirm they're ignored
        raw = build_servo_rpdo(1500, 1500, CW_ENABLE)
        raw = raw[:5] + b"\xDE\xAD\xBE"  # inject non-zero padding
        entries = disassemble_servo_rpdo(raw)
        assert len(entries) == 3
