# Action chunk for /bridge/cmd/arm + /bridge/cmd/low (2026-05-26 wire reversal).
#
# Wraps action_horizon JointCmd values into a single DDS message. NX
# motor_controller unpacks into joint_buf and paces at 100 Hz; chunk
# boundaries are detected via chunk_id transitions for crossfade.
#
# Wrap rule: chunk_id skips 0 on overflow (2^32-1 -> 1) to preserve the
# sentinel. Non-chunk producers (teleop, CLI) MUST send single-step
# JointCmd on a different topic — JointCmdChunk is reserved for VLA.

std_msgs/Header header
uint32 chunk_id
JointCmd[] steps
