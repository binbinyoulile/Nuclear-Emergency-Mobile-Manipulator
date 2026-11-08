"""双车协同移动操作臂的硬件接口层（面向接口的抽象）。

文件目的：
    把「上层任务逻辑」与「底层控制实现」解耦。任务层（`DualCarMission` 及其状态机）
    只依赖本文件定义的接口；MuJoCo 实现在 `dual_car_pick_place.py` 的 `Sim*` 适配器里。
    将来接 ROS2 或真车时，只要实现同一组接口，任务层代码可以原样复用。

分层约定：
    robot_interface.py     抽象层：不 import mujoco，也不碰任何仿真全局量
    Sim* 适配器            仿真实现：把接口方法翻译成 MuJoCo 的 ctrl / qpos 读写
    Real* 适配器（后续）   真机实现：翻译成 CAN 帧 / RTDE / Modbus / 相机话题

单位约定（全库统一）：
    长度 m，角度 rad，线速度 m/s，角速度 rad/s，力 N，时间 s。

重要性等级：A"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


class Clock(ABC):
    """运行时钟与单帧推进。

    仿真实现按固定步数推进 `mj_step`；真机实现按真实时间等待到下一个控制周期。
    任务层的所有时间判断都必须通过它，不得直接读仿真全局变量。
    """

    @property
    @abstractmethod
    def now(self) -> float:
        """当前时刻，单位 s。"""

    @property
    @abstractmethod
    def dt(self) -> float:
        """单个控制帧的时长，单位 s。"""

    @abstractmethod
    def step(self) -> None:
        """推进一个控制帧（仿真里包含 viewer 同步）。"""


class ChassisInterface(ABC):
    """单台四轮底盘的读写接口。

    约定：
        轮速单位 rad/s，正数前进；前轮转角单位 rad，正数左转。
        两辆车各持有一个实现，任务层通过 `RobotIO.chassis` 下标访问。
    """

    name: str

    @abstractmethod
    def pose(self) -> tuple[np.ndarray, float]:
        """返回 (世界坐标 xyz, 偏航角 rad)。"""

    @abstractmethod
    def speed(self) -> float:
        """平面线速度，单位 m/s。"""

    @abstractmethod
    def set_wheels(self, left: float, right: float, steer: float) -> None:
        """下发左右侧轮速与前轮转角。"""

    @abstractmethod
    def wheel_speeds(self) -> np.ndarray:
        """实测四个轮子的角速度，顺序 FL/FR/RL/RR，单位 rad/s。"""

    @abstractmethod
    def contacts_with(self, other: "ChassisInterface") -> int:
        """与另一台底盘之间的物理接触对数量（安全判据用）。"""


class ArmInterface(ABC):
    """六自由度机械臂的读写接口。

    约定：
        目标位姿一律用世界坐标给出；固定基座坐标由实现内部换算。
        运动是"下发目标 + 逐帧推进"的两段式，便于替换成 ROS2 的 action 或 RTDE 的
        阻塞运动：`start_fixed(...)` 只发目标，`tick()` 每帧推进插值。
    """

    @abstractmethod
    def joint_positions(self) -> np.ndarray:
        """当前六个关节角，单位 rad，shape=(6,)。"""

    @abstractmethod
    def set_joint_targets(self, q: np.ndarray) -> None:
        """直接写关节目标（真机实现可换成关节位置指令）。"""

    @abstractmethod
    def start_fixed(
        self,
        target_pos_fixed: np.ndarray,
        duration: float,
        target_quat: np.ndarray | None = None,
    ) -> None:
        """按固定基座坐标系目标位姿启动一段平滑运动。"""

    @abstractmethod
    def tick(self) -> None:
        """推进一帧插值。"""

    @abstractmethod
    def hold(self) -> None:
        """保持当前姿态（含重力补偿）。"""

    @property
    @abstractmethod
    def active(self) -> bool:
        """是否仍在插值中。"""

    @abstractmethod
    def solve_ik(
        self, pos_fixed: np.ndarray, target_quat: np.ndarray
    ) -> tuple[np.ndarray, float]:
        """固定基座坐标系下的 IK，返回 (关节角, 位置残差 m)。"""


class GripperInterface(ABC):
    """二指夹爪接口。

    约定：
        开合值 0…255（0 = 全开，255 = 全闭），与 Robotiq 2F-85 的寄存器量纲一致，
        真机实现可直接透传，不需要在任务层做映射。
    """

    @abstractmethod
    def start(self, value: float, duration: float) -> None:
        """启动一段开合动作。"""

    @abstractmethod
    def tick(self) -> None:
        """推进一帧开合动作。"""

    @abstractmethod
    def hold(self) -> None:
        """保持当前开合指令。"""

    @property
    @abstractmethod
    def active(self) -> bool:
        """是否仍在动作中。"""

    @abstractmethod
    def force(self) -> float:
        """夹持力，单位 N（真机读寄存器，仿真返回最近测量值）。"""


class PerceptionInterface(ABC):
    """多模态感知接口：图像采集与目标定位。

    约定：
        `capture()`/`detect_target()` 是任务层唯一允许的感知入口；
        `object_truth()` 只用于仿真验收，真机实现应返回 NaN 并由视觉结果代替。
    """

    @abstractmethod
    def capture(self) -> tuple[np.ndarray, np.ndarray]:
        """采集一帧 (RGB, depth)，depth 单位 m。"""

    @abstractmethod
    def detect_target(
        self, rgb: np.ndarray, depth: np.ndarray
    ) -> np.ndarray:
        """识别目标并返回世界坐标 shape=(3,)，单位 m。"""

    @abstractmethod
    def object_truth(self) -> np.ndarray:
        """仿真真值坐标，仅用于误差验收；真机返回 NaN。"""


@dataclass(frozen=True)
class RobotIO:
    """任务层握有的全部输入输出端口。

    任务状态机只允许通过本对象访问硬件，不再直接读仿真全局量或 viewer。
    """

    clock: Clock
    chassis: tuple[ChassisInterface, ...]
    arm: ArmInterface
    gripper: GripperInterface
    perception: PerceptionInterface
    # 仅在渲染/截图场景使用；真机实现置 None。
    viewer: object | None = field(default=None, repr=False)
