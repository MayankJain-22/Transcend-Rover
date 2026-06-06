# transcend_interfaces/msg/WheelVelocity.msg
#
# Per-wheel velocity in rad/s
# Used for:
#   /wheel_vel_cmd   : Pi → ESP32 (velocity setpoints)
#   /wheel_vel_state : ESP32 → Pi (actual measured velocities)
#
# Naming: fl=front-left, fr=front-right, rl=rear-left, rr=rear-right
# Sign convention: positive = forward rotation (REP-103)

float32 fl
float32 fr
float32 rl
float32 rr
