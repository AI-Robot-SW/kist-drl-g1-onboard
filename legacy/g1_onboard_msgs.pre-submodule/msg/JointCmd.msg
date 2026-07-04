# JointCmd — joint-space command for rt/arm_sdk (upper-body blended).
#
# Routing: /onboard/cmd/arm  → motor_controller → rt/arm_sdk (weight respected).
#          /onboard/cmd/low  → motor_controller → rt/lowcmd  (weight ignored).
#
# Wire is step-by-step. Chunk metadata travels with each step (chunk_id +
# step_index). NX consumers may detect chunk boundaries; non-chunk producers
# (teleop, CLI) set chunk_id = 0.

std_msgs/Header header

# Per-joint vectors — INVARIANT (enforced at safety_monitor, IDL cannot express):
#   len(joint_names) == len(q) == len(dq) == len(kp) == len(kd) == len(tau_ff)
# Mismatch -> EstopFlag.REASON_MALFORMED_CMD.
string[]  joint_names
float64[] q          # target position (rad)
float64[] dq         # target velocity (rad/s) — used as PD reference in POSITION + HYBRID
float64[] kp         # P gain
float64[] kd         # D gain
float64[] tau_ff     # feed-forward torque (Nm)

# Execution mode (PD law: tau = kp*(q*-q) + kd*(dq*-dq) + tau_ff)
uint8 MODE_POSITION = 0    # use q, dq, kp, kd; tau_ff = 0
uint8 MODE_TORQUE   = 1    # use tau_ff only; PD inputs ignored
uint8 MODE_HYBRID   = 2    # use q, dq, kp, kd, tau_ff (PD + feed-forward)
uint8 mode

# rt/arm_sdk blending weight (motor_cmd[29].q): actual = motion*(1-w) + user*w.
# Range [0.0, 1.0]; out-of-range -> EstopFlag.REASON_MALFORMED_CMD.
# Default 0.0 = motion controller drives. VLA Provider MUST set 1.0 for full control.
# MODE_TORQUE: weight unused; motor_controller silently coerces to 1.0.
float32 weight

# Chunk metadata (option (a') wire). chunk_id == 0 means no chunk context.
# Wrap rule: chunk_id skips 0 on overflow (2^32-1 -> 1) to preserve the sentinel.
uint32 chunk_id
uint16 step_index
