"""
blackboard_keys.py — Blackboard key constants for the state machine.

All behaviors use the namespace "/rdj". Import these constants instead of
bare strings to catch typos at import time.
"""

# Safety / hardware state (written by node-level subscription callbacks)
SAFETY_OK = "safety_ok"  # bool: velocity_scale > 0.0 AND not estop
ESTOP_ACTIVE = "estop_active"  # bool: hardware or software e-stop asserted
ALL_HOMED = "all_homed"  # bool: from /motion/status.all_homed
MOTION_FAULT = "motion_fault"  # bool: from /motion/status.fault
MOTION_FAULT_MSG = "motion_fault_msg"  # str:  from /motion/status.fault_msg
PINCHER_TOF_MM = "pincher_tof_mm"  # float: distance below pincher arm (mm)

# Turntable playback state
PLAYBACK_PROGRESS = "playback_progress"  # float 0.0–1.0 from /turntable/progress

# Operator-selected modes
PLAY_MODE = "play_mode"  # str: "SEQUENTIAL" | "SINGLE_REPEAT" | "SIDE_REPEAT"

# Record queue state (managed by DecideAction and node callbacks)
CURRENT_RECORD_IDX = "current_record_index"  # int: index into queue_stack.slot_z_mm
PREV_RECORD_IDX = "prev_record_index"  # int: saved before swap increment (for place-back)
CURRENT_SIDE = "current_side"  # str: "A" | "B"
QUEUE_SIZE = "queue_size"  # int: total records in queue

# Decision output (written by DecideAction, read by FlipOrSwap gates)
NEXT_ACTION = "next_action"  # str: "flip" | "swap" | "flip_swap" | "halt" | ""

# Target side after the pending flip servo motion.  Written by DecideAction for
# any "flip" or "flip_swap" decision; read and applied by CommitFlipSideAction
# AFTER FlipRecordAction completes so the web UI updates at the right time.
NEXT_FLIP_SIDE = "next_flip_side"  # str: "A" | "B"

# Set True by the "home" user command; cleared by ClearForceRehomeAction after
# HomeAllAction succeeds.  Keeps IsAllHomed returning FAILURE until homing
# actually completes inside the tree, independent of /motion/status timing.
FORCE_REHOME = "force_rehome"  # bool

# Set to ≥0 by /user/select_record; consumed by DecideAction at next decision.
# -1 = no override pending.
OVERRIDE_SLOT = "override_slot"  # int

# Set True by the "flip" user command; makes WaitForRecordFinished succeed
# immediately and forces DecideAction to flip regardless of play mode.
FORCE_FLIP = "force_flip"  # bool

# True while a record is physically on the turntable platter.
PLAYER_HAS_RECORD = "player_has_record"  # bool

# Per-slot side tracking: which side ("A" or "B") is facing up for each slot's
# record.  Allows correct side resumption after records are returned to slots.
SLOT_SIDES = "slot_sides"  # list[str]

# Set True permanently after the first record has been placed on the player by
# the InitialLoad subtree.  Used as the gate in MaybeInitialLoad so the subtree
# doesn't re-trigger when player_has_record is transiently False during normal
# arm transit between lift and place.
INITIAL_LOADED = "initial_loaded"  # bool

# Set True by the "start" user command; consumed by WaitForStartCommand so the
# InitialLoad subtree does not run until the operator explicitly triggers it.
START_REQUESTED = "start_requested"  # bool
