# BufState — motor_controller ring-buffer telemetry.
# Published periodically; forwarded by comm_bridge to PC monitor.

std_msgs/Header header

float32 velocity_fill        # velocity_buf fill ratio, 0.0–1.0
float32 joint_fill           # joint_buf    fill ratio, 0.0–1.0
float32 control_rate_hz      # measured loop frequency (target: 100.0 per REQ-38)
uint32  last_cmd_age_ms      # ms since last JointCmd pop from /onboard/safety/validated_joint
                             # (any sender). Velocity stream NOT tracked here.

# Cumulative counters since process start
uint32  underrun_count       # pop on empty buffer
uint32  overrun_count        # push on full buffer (oldest entry evicted)
