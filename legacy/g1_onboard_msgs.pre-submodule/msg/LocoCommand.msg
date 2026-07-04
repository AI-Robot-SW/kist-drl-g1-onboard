# LocoCommand — discrete G1 LocoClient action (anything other than continuous Move).
#
# Discrete LocoClient FSM transitions (StandUp / SitDown / BalanceStand / ZeroTorque). Continuous walking velocity is published separately as geometry_msgs/Twist on /onboard/cmd/vel by PC NavigationProvider (2026-05-26).
# This message is for one-shot LocoClient API calls that change the FSM.
# motor_controller subscribes /onboard/cmd/loco and translates each value into
# the matching unitree_sdk2_python LocoClient method call (bypasses queue).

std_msgs/Header header

uint8 ACTION_DAMP          = 0   # passive — zero torque + crouch (E-STOP path)
uint8 ACTION_ZERO_TORQUE   = 1   # zero torque without crouch (gentle hand-off)
uint8 ACTION_STAND_UP      = 2   # lying / damped → upright
uint8 ACTION_BALANCE_STAND = 3   # locked stance, ready to walk or manipulate
uint8 ACTION_LOW_STAND     = 4   # half-crouch standing posture
uint8 ACTION_HIGH_STAND    = 5   # full-height standing posture
uint8 ACTION_SIT_DOWN      = 6   # controlled sit-down

uint8 action

# Optional integer parameter (e.g. gait type, balance mode index). Most actions ignore.
# TODO(REQ-34): if a future LocoClient API needs a float param, extend the schema
#               with `float32 param_f32` instead of retyping this field.
int32 param
