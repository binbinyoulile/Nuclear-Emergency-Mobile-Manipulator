"""双车协同移动操作臂的硬件接口层（面向接口的抽象）。

文件目的：
    把「上层任务逻辑」与「底层控制实现」解耦。任务层（`mission_core.py`）
    只依赖本文件定义的接口；MuJoCo 实现在 `dual_car_pick_place.py` 的 `Sim*` 适配器里，
    ROS 2 实现在 `ros2_ws/src/mm_mission/mm_mission/adapters.py` 里。
    将来接真车时，只要实现同一组接口，任务层代码可以原样复用。

分层约定：
    robot_interface.py     抽象层：不 import mujoco，也不碰任何仿真全局量
    mission_core.py        任务层：只认端口，不认后端
    Sim* / Ros* 适配器      后端实现：把接口方法翻译成 MuJoCo 的 ctrl / qpos 读写或 ROS 话题

单位约定（全库统一）：
    长度 m，角度 rad，线速度 m/s，角速度 rad/s，力 N，时间 s。

重要性等级：A"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable

import numpy as np


class Clock(ABC):
    """运行时钟与单帧推进。

    仿真实现按固定步数推进 `mj_step`；ROS 实现阻塞等待下一条 `/clock`；
    真机实现按真实时间等待到下一个控制周期。
    任务层的所有时间判断都必须通过它，不得直接读后端全局量。
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
        """推进一个控制帧。"""


class LidarInterface(ABC):
    """单台车辆激光雷达的派生量视图与扫描节拍。

    约定：
        字段名与仿真侧 `LidarScanner`、ROS 侧 `LidarView` 完全一致，控制器代码
        因此可以在两种后端上原样运行。所有距离单位 m，值为 inf 表示该方向无回波。
    """

    # 前向警戒扇区（±LIDAR_FRONT_HALF、LIDAR_DETECT_RANGE 内）是否被占用。
    front_blocked: bool
    # 警戒区内最近回波距离、其横向偏移（带符号 = 左侧为正）。
    front_min: float
    front_lateral: float
    front_lateral_signed: float
    # 左/右侧向最近回波（不含前向警戒扇区）。
    left_min: float
    right_min: float
    # 车尾 ±LIDAR_REAR_HALF 内最近回波，以及整个后半平面（|方位| > 90°）最近回波。
    rear_min: float
    rear_clear: float
    # 上一次扫描的时刻，单位 s，供控制器自行节流。
    last_scan_time: float
    # 是否把同伴车当作动态障碍绕行。置 False 等于"只看静止障碍物"，供最后一段
    # 直线入位使用（此时两车安全间距已由停靠规划保证）；任务层会保存并恢复它。
    peer_avoidance: bool

    @abstractmethod
    def scan(self) -> None:
        """立即执行一次扫描（仿真里是射线投射，真机里是取最近一帧点云）。"""

    @abstractmethod
    def scan_if_due(self) -> None:
        """距上次扫描超过一个周期时才扫描，否则保持旧点云。"""


class ChassisInterface(ABC):
    """单台四轮底盘的读写接口。

    约定：
        轮速单位 rad/s，正数前进；前轮转角单位 rad，正数左转。
        两辆车各持有一个实现，任务层通过 `RobotIO.chassis` 下标访问（0 = CAR1）。
    """

    name: str
    lidar: LidarInterface
    # 控制器自己的输出缓存（指令斜坡用），不属于硬件状态，真机侧无需关心。
    left_cmd: float
    right_cmd: float
    steer_cmd: float
    # 与同伴车中心距的历史最小值，单位 m；路径控制器逐帧刷新，供验收判据读取。
    min_other_separation: float

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

    def set_motion(self, left: float, right: float, steer: float) -> None:
        """`set_wheels` 的历史命名，任务层的控制器沿用它。"""
        self.set_wheels(left, right, steer)


class ArmInterface(ABC):
    """六自由度机械臂的读写接口。

    约定：
        位置一律用世界坐标给出；固定基座坐标由实现内部换算。
        姿态四元数（wxyz）用**固定基座坐标系**——工具朝向与车体航向无关，
        这样同一组抓取/投放姿态常数在任意车体朝向下都成立。
        运动是"下发目标 + 逐帧推进"的两段式，便于替换成 ROS 2 的 action 或 RTDE 的
        阻塞运动：`start_world(...)` 只发目标，`tick()` 每帧推进插值。
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
    def start_world(
        self,
        target_pos_world: np.ndarray,
        duration: float,
        target_quat: np.ndarray | None = None,
    ) -> None:
        """按世界坐标系目标位置启动运动；姿态仍取固定基座坐标系四元数。

        `target_quat=None` 表示用任务约定的竖直下压姿态。
        """

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
    def tool_position(self) -> np.ndarray:
        """夹持点（pinch 站点）的世界坐标，shape=(3,)，单位 m。"""

    @abstractmethod
    def solve_ik(
        self, pos_world: np.ndarray, target_quat: np.ndarray | None = None
    ) -> tuple[np.ndarray, float]:
        """世界坐标位置的 IK，返回 (关节角, 位置残差 m)。"""


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
    """目标感知接口：只要"目标在哪"和"手里的东西现在在哪"。

    约定：
        像素级处理（取图、分割、内外参换算）不进入任务层：
        仿真侧在 `SimPerception` 里做，ROS 侧由 `mm_target_localizer` 节点做。
    """

    @abstractmethod
    def estimate_target(self) -> np.ndarray:
        """触发一帧采集并返回目标粗端中心的世界坐标 shape=(3,)，单位 m。

        异常：
            定位失败抛 RuntimeError，等待超时抛 TimeoutError——任务层据此决定
            是后退重拍还是退回目标区先验。
        """

    @abstractmethod
    def object_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """被操作对象的实时世界位姿 (位置 xyz, 四元数 wxyz)。

        用途：
            抓取成败判据、投放对中（夹持点→质心偏移）与最终验收都要用它。
            真机由腕部相机给出；仿真由 `/sim/object_pose` 给出。
        """


@dataclass(frozen=True)
class RobotIO:
    """任务层握有的全部输入输出端口。

    任务状态机只允许通过本对象访问硬件，不再直接读后端全局量。
    """

    clock: Clock
    # 两辆车，下标 0 必须是 CAR1（作业车）；任务层靠这个顺序区分主从车。
    chassis: tuple[ChassisInterface, ...]
    arm: ArmInterface
    gripper: GripperInterface
    perception: PerceptionInterface
    # 每帧渲染回调：把雷达点云叠加到观察窗、刷新画面。无渲染后端时为 None。
    frame_hook: Callable[[], None] | None = field(default=None, repr=False)
