"""
blackboard_keys.py — Blackboard key constants for the state machine.

All behaviors use the namespace "/rdj". Import these constants instead of
bare strings to catch typos at import time.
"""

# Safety / hardware state (written by node-level subscription callbacks)
SAFETY_OK        = "safety_ok"         # bool: velocity_scale > 0.0 AND not estop
ESTOP_ACTIVE     = "estop_active"      # bool: hardware or software e-stop asserted
ALL_HOMED        = "all_homed"         # bool: from /motion/status.all_homed
MOTION_FAULT     = "motion_fault"      # bool: from /motion/status.fault
MOTION_FAULT_MSG = "motion_fault_msg"  # str:  from /motion/status.fault_msg
PINCHER_TOF_MM   = "pincher_tof_mm"   # float: distance below pincher arm (mm)

# Turntable playback state
PLAYBACK_PROGRESS = "playback_progress"  # float 0.0–1.0 from /turntable/progress

# Operator-selected modes
PLAY_MODE = "play_mode"  # str: "SEQUENTIAL" | "SINGLE_REPEAT" | "SIDE_REPEAT"

# Record queue state (managed by DecideAction and node callbacks)
CURRENT_RECORD_IDX = "current_record_index"  # int: index into queue_stack.slot_z_mm
PREV_RECORD_IDX    = "prev_record_index"     # int: saved before swap increment (for place-back)
CURRENT_SIDE       = "current_side"          # str: "A" | "B"
QUEUE_SIZE         = "queue_size"            # int: total records in queue

# Decision output (written by DecideAction, read by FlipOrSwap gates)
NEXT_ACTION = "next_action"  # str: "flip" | "swap" | "halt" | ""

# Set True by the "home" user command; cleared by ClearForceRehomeAction after
# HomeAllAction succeeds.  Keeps IsAllHomed returning FAILURE until homing
# actually completes inside the tree, independent of /motion/status timing.
FORCE_REHOME = "force_rehome"  # bool
