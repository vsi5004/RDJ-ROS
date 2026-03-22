# Published by motion_coordinator at ~50 Hz
# Subscribing: state_machine, led_controller, diagnostics, web_interface

# ── Axis positions (physical units) ──────────────────────────────────────────
float64 x_mm
float64 z_mm
float64 a_deg

# ── Homing state ──────────────────────────────────────────────────────────────
bool x_homed
bool z_homed
bool a_homed
bool all_homed

# ── Motion state ──────────────────────────────────────────────────────────────
bool x_moving
bool z_moving
bool a_moving

# ── Sensor readings ───────────────────────────────────────────────────────────
float32 x_tof_mm        # X axis ToF (long range — collision avoidance)
float32 z_tof_mm        # Z axis ToF (height confirmation)
float32 pincher_tof_mm  # Pincher ToF (grip confirmation)

# ── CANopen node status words ─────────────────────────────────────────────────
uint8 x_status_word
uint8 z_status_word
uint8 a_status_word
uint8 pincher_status_word
uint8 player_status_word

# ── Fault / safety ────────────────────────────────────────────────────────────
bool fault
string fault_msg
float32 velocity_scale   # Current safety scaling factor (0.0–1.0)
