"""把 ROS 话题封装成任务层的硬件端口。

直接实现仓库根 ``robot_interface.py`` 的抽象基类（Clock / Lidar / Chassis / Arm /
Gripper / Perception）：方法签名由 ABC 强制检查，少实现一个方法在构造对象时就会
报 TypeError，而不是跑到任务中途才 AttributeError。

感知接口已经按设计草案收敛为 ``estimate_target()`` 与 ``object_pose()``：
任务层不该关心像素，取图与检测都在 ``mm_target_localizer`` 里完成。
"""

from __future__ import annotations

import numpy as np

from robot_interface import (
    ArmInterface,
    ChassisInterface,
    Clock,
    GripperInterface,
    LidarInterface,
    PerceptionInterface,
    RobotIO,
)

from .trajectory import JointTrajectory, ScalarTrajectory

__all__ = [
    "RobotIO",
    "RosArm",
    "RosChassis",
    "RosClock",
    "RosGripper",
    "RosPerception",
]


class RosClock(Clock):
    """虚拟时钟：``step()`` 阻塞到仿真推进一帧。

    仿真时间由 mm_sim_bridge 发布，``step()`` 等到 ``/clock`` 递增即返回，
    因此任务层的每一轮"读状态 → 算指令 → 写指令"都恰好对应一个 20 ms 控制帧。

    每帧把两车最近一次轮速指令重发一遍：桥的帧同步要求每帧各收到一条
    ``wheel_command``，而任务层在机械臂动作、视��等待等阶段不会重新下发底盘指令
    （仿真里 ctrl 会一直保持，话题里不会）。重发等于把"指令保持"这个语义补齐。
    """

    def __init__(self, node) -> None:
        self._node = node

    @property
    def now(self) -> float:
        return float(self._node.sim_time)

    @property
    def dt(self) -> float:
        return float(self._node.frame_dt)

    def step(self) -> None:
        self._node.wait_next_frame()
        self._node.flush_wheel_commands()


class LidarView(LidarInterface):
    """雷达派生量的只读视图，字段名与仿真侧 ``LidarScanner`` 保持一致。

    扫描本身在桥的帧循环里按 10 Hz 做（仿真物理的一部分），任务侧只读结果，
    所以 ``scan()``/``scan_if_due()`` 是空操作。
    """

    __slots__ = (
        "front_blocked",
        "front_min",
        "front_lateral",
        "front_lateral_signed",
        "left_min",
        "right_min",
        "rear_min",
        "rear_clear",
        "last_scan_time",
        "_node",
        "_peer_avoidance",
    )

    def __init__(self, node) -> None:
        self._node = node
        self.front_blocked = False
        self.front_min = float("inf")
        self.front_lateral = float("inf")
        self.front_lateral_signed = 0.0
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.rear_min = float("inf")
        self.rear_clear = float("inf")
        self.last_scan_time = -1e9
        self._peer_avoidance = True

    def update(self, msg) -> None:
        self.front_blocked = bool(msg.front_blocked)
        self.front_min = float(msg.front_min)
        self.front_lateral = float(msg.front_lateral)
        self.front_lateral_signed = float(msg.front_lateral_signed)
        self.left_min = float(msg.left_min)
        self.right_min = float(msg.right_min)
        self.rear_min = float(msg.rear_min)
        self.rear_clear = float(msg.rear_clear)
        self.last_scan_time = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    @property
    def peer_avoidance(self) -> bool:
        return self._peer_avoidance

    @peer_avoidance.setter
    def peer_avoidance(self, value: bool) -> None:
        # 桥在帧边界读这个话题决定要不要把同伴车算进动态障碍
        self._peer_avoidance = bool(value)
        self._node.publish_peer_avoidance(self._peer_avoidance)

    def scan(self) -> None:
        """空操作：扫描由仿真桥按 10 Hz 完成。"""

    def scan_if_due(self) -> None:
        """空操作：扫描由仿真桥按 10 Hz 完成。"""


class RosChassis(ChassisInterface):
    """单台底盘的读写端口。

    ``left_cmd/right_cmd/steer_cmd`` 是控制器自己的输出缓存（与单进程实现一致），
    ``min_other_separation`` 由路径控制器逐帧刷新。
    """

    def __init__(self, node, index: int) -> None:
        self.name = f"CAR{index + 1}"
        self._node = node
        self._index = index
        self.lidar = LidarView(node)
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.steer_cmd = 0.0
        self.min_other_separation = float("inf")

    def pose(self) -> tuple[np.ndarray, float]:
        pos, yaw, _, _ = self._node.odom(self._index)
        return pos.copy(), float(yaw)

    def speed(self) -> float:
        return float(self._node.odom(self._index)[2])

    def set_wheels(self, left: float, right: float, steer: float) -> None:
        self.left_cmd = float(left)
        self.right_cmd = float(right)
        self.steer_cmd = float(steer)
        self._node.publish_wheel(self._index, left, right, steer)

    def wheel_speeds(self) -> np.ndarray:
        state = self._node.chassis_joint(self._index)
        if state is None:
            return np.zeros(4, dtype=float)
        return np.asarray(state.velocity[:4], dtype=float)

    def contacts_with(self, other) -> int:
        return int(self._node.contacts)


class RosArm(ArmInterface):
    """六轴机械臂端口：插值在任务侧完成，逆解走驱动侧服务。

    ``start_fixed`` 的入参是 **car1/arm_base_link** 坐标系，通过 TF 换算到世界系后
    再请求逆解；``start_world`` 直接吃世界坐标。姿态参数本阶段不生效
    （桥的逆解服务固定用 TARGET_QUAT，真机接 servoj 时再启用）。
    """

    def __init__(self, node, clock: RosClock) -> None:
        self._node = node
        self._clock = clock
        self._traj: JointTrajectory | None = None
        self._hold_q: np.ndarray | None = None

    def joint_positions(self) -> np.ndarray:
        q = self._node.arm_joint_positions()
        return np.zeros(6) if q is None else q.copy()

    def set_joint_targets(self, q: np.ndarray) -> None:
        self._traj = None
        self._hold_q = np.asarray(q, dtype=float).copy()
        self._node.publish_arm(self._hold_q)

    def start_fixed(self, target_pos_fixed, duration, target_quat=None) -> None:
        target_world = self._node.arm_base_to_world(np.asarray(target_pos_fixed, float))
        self.start_world(target_world, duration, target_quat)

    def start_world(self, target_pos_world, duration, target_quat=None) -> None:
        del target_quat  # 姿态由驱动侧的逆解服务固定，见类文档
        q_ik, error = self._node.solve_ik_world(np.asarray(target_pos_world, float))
        if error > 0.02:
            raise RuntimeError(f"IK 残差过大：{error * 1000:.1f} mm，目标不可达")
        self._traj = JointTrajectory(self.joint_positions(), q_ik, duration)
        self._hold_q = self._traj.q.copy()
        self._node.publish_arm(self._traj.q)

    def tick(self) -> None:
        if self._traj is None:
            return
        q = self._traj.update(self._clock.dt)
        self._hold_q = q.copy()
        self._node.publish_arm(q)
        if self._traj.done:
            self._traj = None

    def hold(self) -> None:
        if self._hold_q is None:
            self._hold_q = self.joint_positions()
        self._node.publish_arm(self._hold_q)

    @property
    def active(self) -> bool:
        return self._traj is not None

    def tool_position(self) -> np.ndarray:
        """夹持点（pinch 站点）的世界坐标，来自 ``car1/pinch`` TF。"""
        return self._node.tf_position("car1/pinch")

    def solve_ik(self, pos_world, target_quat=None) -> tuple[np.ndarray, float]:
        # 统一使用固定抓取姿态，姿态参数本阶段不生效
        del target_quat
        return self._node.solve_ik_world(np.asarray(pos_world, dtype=float))


class RosGripper(GripperInterface):
    """二指夹爪端口：开合插值在任务侧完成，指令流式下发。"""

    def __init__(self, node, clock: RosClock) -> None:
        self._node = node
        self._clock = clock
        self._traj: ScalarTrajectory | None = None
        self._value = 0.0

    def start(self, value: float, duration: float) -> None:
        # 只登记轨迹，不立刻下发目标值：否则第一帧就会从当前值跳到目标值
        self._traj = ScalarTrajectory(self._value, float(value), duration)

    def tick(self) -> None:
        if self._traj is None:
            return
        self._value = self._traj.update(self._clock.dt)
        self._node.publish_gripper(self._value)
        if self._traj.done:
            self._traj = None

    def hold(self) -> None:
        self._node.publish_gripper(self._value)

    @property
    def active(self) -> bool:
        return self._traj is not None

    def force(self) -> float:
        state = self._node.gripper_state()
        return 0.0 if state is None else float(state.effort)


class RosPerception(PerceptionInterface):
    """目标感知端口：触发一帧采集，等待感知节点给出新的估计。"""

    def __init__(self, node) -> None:
        self._node = node

    def estimate_target(self) -> np.ndarray:
        return self._node.request_estimate()

    def object_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return self._node.object_pose()
