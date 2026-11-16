"""把 ROS 话题封装成任务层的硬件端口。

与仓库根目录 ``robot_interface.py`` 的 ABC 一一对应
（Clock / ChassisInterface / ArmInterface / GripperInterface / PerceptionInterface）：
方法签名刻意保持一致，阶段 2 合并两边时只需补上基类声明。

感知接口已经按设计草案收敛为 ``estimate_target()``：任务层不该关心像素，
取图与检测都在 ``mm_target_localizer`` 里完成。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .trajectory import JointTrajectory, ScalarTrajectory


@dataclass(frozen=True)
class RobotIO:
    """任务层握有的全部输入输出端口（与 robot_interface.RobotIO 同构）。"""

    clock: "RosClock"
    chassis: tuple["RosChassis", ...]
    arm: "RosArm"
    gripper: "RosGripper"
    perception: "RosPerception"
    viewer: object | None = field(default=None, repr=False)


class RosClock:
    """虚拟时钟：``step()`` 阻塞到仿真推进一帧。

    仿真时间由 mm_sim_bridge 发布，``step()`` 等到 ``/clock`` 递增即返回，
    因此任务层的每一轮"读状态 → 算指令 → 写指令"都恰好对应一个 20 ms 控制帧。
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


class LidarView:
    """雷达派生量的只读视图，字段名与仿真侧 ``LidarScanner`` 保持一致。"""

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
    )

    def __init__(self) -> None:
        self.front_blocked = False
        self.front_min = float("inf")
        self.front_lateral = float("inf")
        self.front_lateral_signed = 0.0
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.rear_min = float("inf")
        self.rear_clear = float("inf")
        self.last_scan_time = -1e9

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


class RosChassis:
    """单台底盘的读写端口。

    ``left_cmd/right_cmd/steer_cmd`` 是控制器自己的输出缓存（与单进程实现一致），
    不属于硬件语义，真机侧不需要关心。
    """

    def __init__(self, node, index: int) -> None:
        self.name = f"CAR{index + 1}"
        self._node = node
        self._index = index
        self.lidar = LidarView()
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.steer_cmd = 0.0

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

    # 单进程实现里的历史命名，控制器沿用同一调用
    set_motion = set_wheels

    def wheel_speeds(self) -> np.ndarray:
        state = self._node.chassis_joint(self._index)
        if state is None:
            return np.zeros(4, dtype=float)
        return np.asarray(state.velocity[:4], dtype=float)

    def contacts_with(self, other) -> int:
        return int(self._node.contacts)


class RosArm:
    """六轴机械臂端口：插值在任务侧完成，逆解走驱动侧服务。

    ``start_fixed`` 的入参是 **car1/arm_base_link** 坐标系，通过 TF 换算到世界系后
    再请求逆解；``start_world`` 直接吃世界坐标。
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
        self.start_world(target_world, duration)

    def start_world(self, target_pos_world, duration: float) -> None:
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

    def solve_ik(self, pos_world, target_quat=None) -> tuple[np.ndarray, float]:
        # 统一使用固定抓取姿态，姿态参数本阶段不生效（真机接 servoj 时再启用）
        return self._node.solve_ik_world(np.asarray(pos_world, dtype=float))


class RosGripper:
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


class RosPerception:
    """目标感知端口：触发一帧采集，等待感知节点给出新的估计。"""

    def __init__(self, node) -> None:
        self._node = node

    def estimate_target(self) -> np.ndarray:
        return self._node.request_estimate()

    def object_truth(self) -> np.ndarray:
        truth = self._node.target_truth()
        if truth is None:
            return np.full(3, np.nan)
        return truth.copy()
