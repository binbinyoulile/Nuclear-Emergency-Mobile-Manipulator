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


def main() -> int:
    failures = []
    for name, check in (
        ("ROS 适配器", check_ros_adapters),
        ("仿真适配器", check_sim_adapters),
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
