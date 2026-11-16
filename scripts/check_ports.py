"""端口一致性检查：确认两个后端都实现了任务层要用的全部接口。

文件目的：
    `mission_core.py` 只认 `robot_interface.py` 的抽象端口。少实现一个方法时，
    抽象基类会在**构造对象**时报 TypeError——但只有真的把那个适配器实例化才会触发。
    单进程后端每次回归都会实例化（Sim* 适配器），ROS 后端在 Windows 上没有 rclpy、
    跑不起来，所以这里用一个空壳节点只做构造，把"签名漏了/改名了"挡在提交之前。

整体输入输出：
    输入为仓库根；输出为每个端口的检查结果，全部通过时退出码 0。
依赖与版本：
    Python 3.10+、numpy；不需要 ROS 2、不需要 MuJoCo（只加载适配器模块本身）。
运行方式：
    D:/Python313/python.exe scripts/check_ports.py

重要性等级：B"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
# ROS 2 包的源码根（colcon 安装前 `mm_mission` 包体就在这一层）
sys.path.insert(0, str(PROJECT / "ros2_ws" / "src" / "mm_mission"))

from robot_interface import (  # noqa: E402
    ArmInterface,
    ChassisInterface,
    Clock,
    GripperInterface,
    LidarInterface,
    PerceptionInterface,
)


class _StubNode:
    """空壳节点：只为让适配器能构造出来，不提供任何真实行为。"""

    sim_time = 0.0
    frame_dt = 0.02
    contacts = 0


class _ArmStubNode:
    """只实现 `start_fixed` 换算链与姿态透传需要的节点方法，并记录收到的请求。"""

    def __init__(self, base_pos: np.ndarray, base_rot: np.ndarray) -> None:
        self._base_pos = base_pos
        self._base_rot = base_rot
        self.world_target: np.ndarray | None = None
        self.quat: np.ndarray | None = None

    def arm_base_to_world(self, point):
        return self._base_pos + self._base_rot @ np.asarray(point, dtype=float)

    def solve_ik_world(self, point, orientation):
        self.world_target = np.asarray(point, dtype=float).copy()
        self.quat = np.asarray(orientation, dtype=float).copy()
        return np.zeros(6), 0.0

    def arm_joint_positions(self):
        return np.zeros(6)

    def publish_arm(self, q) -> None:
        pass


def check_ros_adapters() -> list[str]:
    """构造 ROS 侧全部适配器，返回失败项描述。

    能构造出来就说明抽象方法都实现了；`robot_interface` 的 ABC 会替我们检查签名。
    """
    from mm_mission.adapters import (  # noqa: PLC0415 - 需要先设好 sys.path
        RobotIO,
        RosArm,
        RosChassis,
        RosClock,
        RosGripper,
        RosPerception,
    )

    node = _StubNode()
    clock = RosClock(node)
    io = RobotIO(
        clock=clock,
        chassis=(RosChassis(node, 0), RosChassis(node, 1)),
        arm=RosArm(node, clock),
        gripper=RosGripper(node, clock),
        perception=RosPerception(node),
    )
    expected = {
        "clock": Clock,
        "arm": ArmInterface,
        "gripper": GripperInterface,
        "perception": PerceptionInterface,
    }
    failures = []
    for field, base in expected.items():
        value = getattr(io, field)
        if not isinstance(value, base):
            failures.append(f"{field}: {type(value).__name__} 不是 {base.__name__}")
    for index, car in enumerate(io.chassis):
        if not isinstance(car, ChassisInterface):
            failures.append(f"chassis[{index}]: 不是 ChassisInterface")
        if not isinstance(car.lidar, LidarInterface):
            failures.append(f"chassis[{index}].lidar: 不是 LidarInterface")
    if io.frame_hook is not None:
        failures.append("ROS 后端没有渲染回调，frame_hook 应为 None")
    return failures


def check_sim_adapters() -> list[str]:
    """构造仿真侧全部适配器（需要 MuJoCo；失败时只报告，不算致命）。"""
    import mujoco  # noqa: F401, PLC0415 - 环境里没有 MuJoCo 时跳过

    import dual_car_pick_place as legacy  # noqa: PLC0415

    arm, finger, car1, car2, _ = legacy.initialize_mission(0)
    io = legacy.build_robot_io(
        "sim",
        sim=legacy.base.Simulation(realtime=False),
        arm=arm,
        finger=finger,
        car1=car1,
        car2=car2,
    )
    failures = []
    for field, base in (
        ("clock", Clock),
        ("arm", ArmInterface),
        ("gripper", GripperInterface),
        ("perception", PerceptionInterface),
    ):
        if not isinstance(getattr(io, field), base):
            failures.append(f"{field}: 不是 {base.__name__}")
    for index, car in enumerate(io.chassis):
        if not isinstance(car, ChassisInterface):
            failures.append(f"chassis[{index}]: 不是 ChassisInterface")
        if not isinstance(car.lidar, LidarInterface):
            failures.append(f"chassis[{index}].lidar: 不是 LidarInterface")
    if io.frame_hook is None:
        failures.append("仿真后端应当提供 frame_hook（雷达点云叠加）")
    return failures


def check_start_fixed_chain() -> list[str]:
    """确认 ROS 后端的 ``start_fixed`` 与单进程后端落到同一个 IK 目标。

    ``RosArm.start_fixed`` 要经过"固定基座系 → 臂座系 → 世界系"，再由桥的逆解服务换算
    回固定基座系。这条链路里任何一处坐标系搞错都不会抛异常——残差仍然只有几毫米，机械臂
    只是安静地走到关于臂座镜像的位置上（实测 ‖残差‖ 1.65 mm vs 12.98 mm，光看残差根本
    分不出来）。所以这里做一次离线往返：喂同一个 ``CARRY_FIXED``，把 ROS 侧算出的世界点
    用单进程的 ``world_target_to_fixed`` 反算回来，必须逐位相等。
    """
    import dual_car_pick_place as legacy  # noqa: PLC0415
    import mobile_pick_place as base  # noqa: PLC0415

    from mm_mission.adapters import RosArm  # noqa: PLC0415

    legacy.initialize_mission(3)
    node = _ArmStubNode(
        base.DATA.xpos[base.ARM_BASE_BODY].copy(),
        base.DATA.xmat[base.ARM_BASE_BODY].reshape(3, 3).copy(),
    )
    RosArm(node, None).start_fixed(base.CARRY_FIXED, 1.0)
    if node.world_target is None:
        return ["start_fixed 没有下发世界目标"]
    fixed, _ = base.world_target_to_fixed(node.world_target)
    if not np.allclose(fixed, base.CARRY_FIXED, atol=1e-9):
        return [
            "start_fixed 坐标系换算不一致：ROS 落到 "
            f"{np.round(fixed, 4)}，单进程是 {np.round(base.CARRY_FIXED, 4)}"
            "（差了 FIXED_BASE_ROT 的 180°？）"
        ]
    return []


def check_orientation_passthrough() -> list[str]:
    """确认 `RosArm` 把工具姿态原样送到逆解服务。

    抓取要求工具绕自身 Z 轴转 ±90° 才能横跨杆身。姿态一旦在适配器里被丢掉，抓取不会
    报错——夹爪照常闭合、力也有读数（1.5 N，比空夹的 0.2~0.6 N 还高），但杆一根都不会
    被拿起来。单进程实测：强制 0° 时 `finger force=12.00 N`、杆原地不动、任务在 4_grasp
    失败；正常 ±90° 时力 5.10 N、杆被抬到 0.84 m。所以这条必须钉住。
    """
    from mm_mission.adapters import RosArm  # noqa: PLC0415

    # q_base * Rz(90°)：任务层实际会下发的四个候选之一
    half = 2.0**-0.5
    quat = np.array([0.0, half, 0.0, half])
    node = _ArmStubNode(np.zeros(3), np.eye(3))
    arm = RosArm(node, None)
    point = np.array([1.0, 2.0, 3.0])

    arm.start_world(point, 1.0, quat)
    if node.quat is None or not np.allclose(node.quat, quat, atol=1e-12):
        return [f"start_world 没有把工具姿态送到逆解服务（收到 {node.quat}）"]

    node.quat = None
    arm.solve_ik(point, quat)
    if node.quat is None or not np.allclose(node.quat, quat, atol=1e-12):
        return [f"solve_ik 没有把工具姿态送到逆解服务（收到 {node.quat}）"]
    return []


def main() -> int:
    failures = []
    for name, check in (
        ("ROS 适配器", check_ros_adapters),
        ("仿真适配器", check_sim_adapters),
        ("start_fixed 换算链", check_start_fixed_chain),
        ("工具姿态透传", check_orientation_passthrough),
    ):
        try:
            problems = check()
        except Exception as exc:  # noqa: BLE001 - 打印原因即可
            problems = [f"{type(exc).__name__}: {exc}"]
        if problems:
            failures.extend(f"[{name}] {item}" for item in problems)
            print(f"[FAIL] {name}")
            for item in problems:
                print(f"       {item}")
        else:
            print(f"[ OK ] {name}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
