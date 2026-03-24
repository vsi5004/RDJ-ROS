# Published by lidar_safety at ~10 Hz
# Consolidates velocity scaling and hardware e-stop into one message.
# Subscribers: motion_coordinator, state_machine, led_controller, web_interface

float32 velocity_scale  # 1.0 = clear, 0.25 = warning zone, 0.0 = stop zone
bool    estop           # True while an object is inside the stop zone
string  reason          # "" | "lidar_warning_zone" | "lidar_stop_zone"
