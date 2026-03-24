# Published by state_machine at 10 Hz on change
# Replaces the separate /state_machine/slot, /player_has_record, /current_side topics.
# /state_machine/tip is kept as a separate lightweight String for external tooling.

uint8  slot_idx          # Active record slot (index into queue_stack.slot_z_mm)
string current_side      # "A" or "B" — which side of the held/playing record is up
bool   player_has_record # True while a disc is physically on the turntable platter
bool   initial_loaded    # True once the first record has been placed on the player
