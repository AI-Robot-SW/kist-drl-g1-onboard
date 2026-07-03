"""
Joint command passthrough — rt/arm_sdk and rt/lowcmd paths.

Subscriptions:
- /onboard/safety/estop (EstopFlag) E-STOP DDS context
- /onboard/cmd/arm (JointCmd)       → rt/arm_sdk  (upper body only)
- /onboard/cmd/low (JointCmd)       → rt/lowcmd   (whole-body, GearSonic planner)
- POSIX SHM byte 'safety_flag'      zero-latency E-STOP poll

Publications:
- /onboard/motor/buf_state (BufState)  telemetry → comm_bridge → PC
- rt/arm_sdk (LowCmd_)                 Unitree SDK upper-body control
- rt/lowcmd  (LowCmd_)                 Unitree SDK whole-body low-level control

Arm SDK path (/onboard/cmd/arm):
  joint_names (no _joint suffix) are mapped to Unitree G1 motor indices 0-28.
  motor_cmd[29].q = 1 enables arm_sdk mode.

Low cmd path (/onboard/cmd/low):
  All 29 joints published to rt/lowcmd. Balance is the planner's responsibility.
  mode_machine is mirrored from rt/lowstate. mode_pr = 0 (Series PR).

LocoClient removed — whole-body control via rt/lowcmd only.
"""
import gc
import struct
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from g1_onboard_msgs.msg import BufState, EstopFlag, JointCmd

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

# Pure-Python CRC32 for HG LowCmd_ — avoids crc_aarch64.so dependency.
# Matches CRC.__PackHGLowCmd + CRC._crc_py from unitree_sdk2py.
_HG_LOWCMD_FMT = '<2B2x' + 'B3x5fI' * 35 + '5I'

def _hg_lowcmd_crc(cmd) -> int:
    data = [cmd.mode_pr, cmd.mode_machine]
    for mc in cmd.motor_cmd[:35]:
        data += [mc.mode, mc.q, mc.dq, mc.tau, mc.kp, mc.kd,
                 getattr(mc, 'reserve', 0)]
    data += list(getattr(cmd, 'reserve', [0, 0, 0, 0]))
    data.append(0)  # crc placeholder
    raw = struct.pack(_HG_LOWCMD_FMT, *data)
    n = (len(raw) >> 2) - 1
    words = [
        (raw[i*4+3] << 24) | (raw[i*4+2] << 16) | (raw[i*4+1] << 8) | raw[i*4]
        for i in range(n)
    ]
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7
    for w in words:
        bit = 1 << 31
        for _ in range(32):
            crc = ((crc << 1) & 0xFFFFFFFF) ^ (poly if crc & 0x80000000 else 0)
            if w & bit:
                crc ^= poly
            bit >>= 1
    return crc


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Joint name (no _joint suffix) → Unitree G1 motor index (matches MuJoCo order).
_JOINT_TO_IDX: dict[str, int] = {
    "left_hip_pitch":       0,
    "left_hip_roll":        1,
    "left_hip_yaw":         2,
    "left_knee":            3,
    "left_ankle_pitch":     4,
    "left_ankle_roll":      5,
    "right_hip_pitch":      6,
    "right_hip_roll":       7,
    "right_hip_yaw":        8,
    "right_knee":           9,
    "right_ankle_pitch":    10,
    "right_ankle_roll":     11,
    "waist_yaw":            12,
    "waist_roll":           13,
    "waist_pitch":          14,
    "left_shoulder_pitch":  15,
    "left_shoulder_roll":   16,
    "left_shoulder_yaw":    17,
    "left_elbow":           18,
    "left_wrist_roll":      19,
    "left_wrist_pitch":     20,
    "left_wrist_yaw":       21,
    "right_shoulder_pitch": 22,
    "right_shoulder_roll":  23,
    "right_shoulder_yaw":   24,
    "right_elbow":          25,
    "right_wrist_roll":     26,
    "right_wrist_pitch":    27,
    "right_wrist_yaw":      28,
}
_ARM_SDK_ENABLE_IDX = 29  # motor_cmd[29].q = 1 enables arm_sdk


def _patch_channel_factory() -> None:
    """Join rclpy's existing DDS domain instead of creating one (shared libddsc)."""
    from cyclonedds.domain import DomainParticipant as _DDP

    def _init(self, id: int, networkInterface=None, qos=None) -> bool:
        if self.__class__._ChannelFactory__initialized:
            return True
        with self.__class__._ChannelFactory__init_lock:
            if self.__class__._ChannelFactory__initialized:
                return True
            try:
                self.__class__._ChannelFactory__participant = _DDP(id)
            except Exception:
                return False
            self.__class__._ChannelFactory__qos = qos
            self.__class__._ChannelFactory__initialized = True
            return True

    _sdk_ch.ChannelFactory.Init = _init


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__(
            'motor_controller',
            automatically_declare_parameters_from_overrides=True,
        )
        self._network_interface: str = self.get_parameter('network_interface').value
        self._domain_id: int = self.get_parameter('domain_id').value
        self._estop_shm_name: str = self.get_parameter('estop_shm_name').value
        self._dry_run: bool = self.get_parameter('dry_run').value
        self._buf_state_rate_hz: float = self.get_parameter('buf_state_rate_hz').value

        self._shm = self._open_shm(self._estop_shm_name)
        self._dds_estop = False
        self._arm_sdk_pub: ChannelPublisher | None = None
        self._lowcmd_pub: ChannelPublisher | None = None
        self._mode_machine: int = 0

        self._init_sdk()

        self._buf_pub = self.create_publisher(BufState, '/onboard/motor/buf_state', _RELIABLE_QOS)
        self.create_subscription(EstopFlag, '/onboard/safety/estop', self._on_estop, _RELIABLE_QOS)
        self.create_subscription(JointCmd, '/onboard/cmd/arm', self._on_joint_cmd, _RELIABLE_QOS)
        self.create_subscription(JointCmd, '/onboard/cmd/low', self._on_low_cmd, _RELIABLE_QOS)

        bs = self._buf_state_rate_hz if self._buf_state_rate_hz > 0 else 10.0
        self.create_timer(1.0 / bs, self._publish_buf_state)

        self.get_logger().info(
            f'motor_controller ready (dry_run={self._dry_run}, '
            f'arm_sdk={"enabled" if self._arm_sdk_pub is not None else "dry"}, '
            f'lowcmd={"enabled" if self._lowcmd_pub is not None else "dry"})')

    def _init_sdk(self) -> None:
        if self._dry_run:
            self.get_logger().warn('dry_run=true — SDK disabled, dispatch logs only')
            return
        try:
            _patch_channel_factory()
            ChannelFactory().Init(self._domain_id, self._network_interface)

            self._arm_sdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._arm_sdk_pub.Init()
            self.get_logger().info('arm_sdk publisher initialized on rt/arm_sdk')

            self._lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self._lowcmd_pub.Init()
            self.get_logger().info('lowcmd publisher initialized on rt/lowcmd')

            self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._lowstate_sub.Init(self._on_lowstate, 10)
            self.get_logger().info('lowstate subscriber initialized on rt/lowstate')

        except Exception as e:
            self.get_logger().error(f'SDK init failed ({e}) — falling back to dry_run')

    def _open_shm(self, name: str) -> shared_memory.SharedMemory:
        try:
            shm = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            shm = shared_memory.SharedMemory(name=name, create=True, size=1)
            shm.buf[0] = 0
        return shm

    def _on_estop(self, flag: EstopFlag) -> None:
        self._dds_estop = flag.active

    def _on_lowstate(self, msg: LowState_) -> None:
        """Cache mode_machine from hardware for use in lowcmd messages."""
        self._mode_machine = msg.mode_machine

    def _on_joint_cmd(self, msg: JointCmd) -> None:
        """Upper-body control via rt/arm_sdk."""
        if bool(self._shm.buf[0]) or self._dds_estop:
            return
        if self._arm_sdk_pub is None:
            self.get_logger().debug('[dry] joint_cmd received, arm_sdk not initialized')
            return

        low_cmd = unitree_hg_msg_dds__LowCmd_()
        low_cmd.motor_cmd[_ARM_SDK_ENABLE_IDX].q = 1.0  # enable arm_sdk

        unknown = []
        for i, name in enumerate(msg.joint_names):
            idx = _JOINT_TO_IDX.get(name)
            if idx is None:
                unknown.append(name)
                continue
            low_cmd.motor_cmd[idx].q   = float(msg.q[i])
            low_cmd.motor_cmd[idx].dq  = float(msg.dq[i]) if msg.dq else 0.0
            low_cmd.motor_cmd[idx].kp  = float(msg.kp[i]) if msg.kp else 0.0
            low_cmd.motor_cmd[idx].kd  = float(msg.kd[i]) if msg.kd else 0.0
            low_cmd.motor_cmd[idx].tau = float(msg.tau_ff[i]) if msg.tau_ff else 0.0

        if unknown:
            self.get_logger().warn(f'unknown joint names: {unknown}')

        low_cmd.crc = _hg_lowcmd_crc(low_cmd)
        self._arm_sdk_pub.Write(low_cmd)

    def _on_low_cmd(self, msg: JointCmd) -> None:
        """Whole-body control via rt/lowcmd — planner trajectory."""
        if bool(self._shm.buf[0]) or self._dds_estop:
            return
        if self._lowcmd_pub is None:
            self.get_logger().debug('[dry] low_cmd received, lowcmd not initialized')
            return

        low_cmd = unitree_hg_msg_dds__LowCmd_()
        low_cmd.mode_pr = 0  # Series PR (standard G1 ankle)
        low_cmd.mode_machine = self._mode_machine

        unknown = []
        for i, name in enumerate(msg.joint_names):
            idx = _JOINT_TO_IDX.get(name)
            if idx is None:
                unknown.append(name)
                continue
            low_cmd.motor_cmd[idx].mode = 1
            low_cmd.motor_cmd[idx].q   = float(msg.q[i])
            low_cmd.motor_cmd[idx].dq  = float(msg.dq[i]) if msg.dq else 0.0
            low_cmd.motor_cmd[idx].kp  = float(msg.kp[i]) if msg.kp else 0.0
            low_cmd.motor_cmd[idx].kd  = float(msg.kd[i]) if msg.kd else 0.0
            low_cmd.motor_cmd[idx].tau = float(msg.tau_ff[i]) if msg.tau_ff else 0.0

        if unknown:
            self.get_logger().warn(f'unknown joint names: {unknown}')

        low_cmd.crc = _hg_lowcmd_crc(low_cmd)
        self._lowcmd_pub.Write(low_cmd)

    def _publish_buf_state(self) -> None:
        msg = BufState()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._buf_pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            self._shm.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    gc.disable()
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
