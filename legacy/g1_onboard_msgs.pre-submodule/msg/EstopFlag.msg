# EstopFlag — structured E-STOP event from safety_monitor.
# DDS mirror of the POSIX shared-memory 'safety_flag' byte that motor_controller
# polls directly. DDS path is for PC UI / logging.

std_msgs/Header header

bool active                   # true while latched

# REASON_* trigger semantics:
#   NONE           default; not a real reason (active=false)
#   JOINT_LIMIT    JointCmd q/dq/tau out of yaml range
#   VELOCITY       Twist (vx/vy/vyaw) exceeded max_linear_x/y or max_angular_z
#   PROXIMITY      Depth front-cone pixel count above threshold
#   COMMS_TIMEOUT  external input stream silent > per-stream timeout (other side stopped)
#   WATCHDOG       safety_monitor's own loop overran or heartbeat missed (we stopped)
#   MANUAL         operator-triggered (CLI / GUI / future HW button)
#   SELF_COLLISION RESERVED (no collision model on NX today)
#   MALFORMED_CMD  JointCmd schema violation (array length mismatch,
#                   mode/weight out of range, unknown joint_names)
uint8 REASON_NONE           = 0
uint8 REASON_JOINT_LIMIT    = 1
uint8 REASON_VELOCITY       = 2
uint8 REASON_PROXIMITY      = 3
uint8 REASON_COMMS_TIMEOUT  = 4
uint8 REASON_WATCHDOG       = 5
uint8 REASON_MANUAL         = 6
uint8 REASON_SELF_COLLISION = 7
uint8 REASON_MALFORMED_CMD  = 8

uint8 reason
string detail                 # e.g. "knee_pitch over 2.5rad", "18pt within 25cm"
