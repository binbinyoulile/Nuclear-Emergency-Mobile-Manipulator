"""雷达派生量纯函数的最小自检（不需要 ROS、不需要 MuJoCo）。

文件目的：
    `mm_perception/lidar_geometry.summarize` 是按仿真扫描器的定义重写的，扇区边界与
    "横向净空取距离×sin(方位角)"这类细节很容易在重写时走样。本脚本用可以手算的点集
    逐条钉住这些约定：给定这些点，8 个数必须正好是这些值。
运行方式：
    D:/Python313/python.exe scripts/check_lidar_geometry.py
    （Ubuntu 侧：python3 scripts/check_lidar_geometry.py，只依赖 numpy）
已知限制：
    只钉几何定义，不覆盖真实点云的噪声、多回波与遮挡；真车的等效性要靠同一批射线
    在仿真里做端到端对照（见 lidar_summary 节点的 check 模式）。

重要性等级：A"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "ros2_ws" / "src" / "mm_perception"))

from mission_core import (  # noqa: E402
    LIDAR_DETECT_RANGE,
    LIDAR_FRONT_HALF,
    LIDAR_REAR_HALF,
    LIDAR_SIDE_MAX,
)
from mm_perception.lidar_geometry import summarize  # noqa: E402

LIMITS = dict(
    detect_range=LIDAR_DETECT_RANGE,
    front_half=LIDAR_FRONT_HALF,
    rear_half=LIDAR_REAR_HALF,
    side_max=LIDAR_SIDE_MAX,
    max_range=100.0,
)
INF = float("inf")
CASES = 0


def polar(az_deg: float, distance: float, z: float = 0.0) -> tuple[float, float, float]:
    """按方位角/距离造一个雷达坐标系点（z 为传感器系高度）。"""
    az = np.deg2rad(az_deg)
    rho = float(np.hypot(distance, z)) if z else distance
    return (rho * np.cos(az), rho * np.sin(az), z)


def cloud(*points: tuple[float, float, float]) -> np.ndarray:
    """把若干点组成 (N, 3) 点云；空调用返回空数组。"""
    if not points:
        return np.empty((0, 3))
    return np.asarray(points, dtype=np.float64)


def check(name: str, got: dict, expected: dict, tol: float = 1e-9) -> None:
    """逐字段比对，浮点按 tol 比较、inf 与布尔按等值比较。"""
    global CASES
    CASES += 1
    for key, want in expected.items():
        have = got[key]
        if isinstance(want, bool) or want in (INF,):
            ok = have == want
        else:
            ok = abs(float(have) - float(want)) <= tol
        if not ok:
            raise AssertionError(f"{name}: {key} 期望 {want}，实际 {have}")


def main() -> int:
    """跑完全部用例并打印结论。"""
    try:
        none = dict(
            front_blocked=False,
            front_min=INF,
            front_lateral=INF,
            front_lateral_signed=0.0,
            left_min=INF,
            right_min=INF,
            rear_min=INF,
            rear_clear=INF,
        )
        check("空点云", summarize(cloud(), **LIMITS), none)

        check(
            "正前方 1 m",
            summarize(cloud(polar(0, 1.0)), **LIMITS),
            dict(none, front_blocked=True, front_min=1.0, front_lateral=0.0),
        )

        # 两条前向回波：近的那条在左 10°，横向净空必须跟着它，而不是跟远的那条。
        check(
            "前向取最近一条",
            summarize(cloud(polar(10, 1.0), polar(-25, 1.5)), **LIMITS),
            dict(
                none,
                front_blocked=True,
                front_min=1.0,
                front_lateral_signed=float(np.sin(np.deg2rad(10))),
                front_lateral=float(np.sin(np.deg2rad(10))),
            ),
        )

        # 45° 落在左右扇区（前向半角 30° 之外），不能再算进前向。
        check(
            "右 45°",
            summarize(cloud(polar(-45, 1.5)), **LIMITS),
            dict(none, right_min=1.5),
        )
        check(
            "左 45° 与左后方 100°",
            summarize(cloud(polar(45, 1.2), polar(100, 2.5)), **LIMITS),
            dict(none, left_min=1.2, rear_clear=2.5),
        )
        check(
            "正后 170°：既算 rear_min 也算 rear_clear",
            summarize(cloud(polar(170, 2.0)), **LIMITS),
            dict(none, rear_min=2.0, rear_clear=2.0),
        )
        # 超出警戒距离 2.8 m 的回波不进任何扇区（但仍在量程内，不报错）。
        check("超出警戒距离", summarize(cloud(polar(0, 5.0)), **LIMITS), none)

        # 地面过滤：雷达装在 0.2 m，回波离地 0.05 m 时剔除，0.15 m 时保留。
        ground = dict(LIMITS, mount_height=0.2, min_z=0.08)
        check(
            "地面回波被剔除",
            summarize(cloud(polar(0, 1.0, -0.15)), **ground),
            none,
        )
        check(
            "略高于地面的回波保留",
            summarize(cloud(polar(0, 1.0, -0.05)), **ground),
            dict(
                none,
                front_blocked=True,
                front_min=float(np.linalg.norm(polar(0, 1.0, -0.05))),
                front_lateral=0.0,
            ),
        )

        # 自身盲区：车体/机械臂的回波在盲区半径内要丢掉。
        check(
            "自身盲区内剔除",
            summarize(cloud((0.2, 0.0, 0.0)), **dict(LIMITS, self_radius=0.3)),
            none,
        )
        check(
            "关闭盲区后保留",
            summarize(cloud((0.2, 0.0, 0.0)), **LIMITS),
            dict(none, front_blocked=True, front_min=0.2, front_lateral=0.0),
        )

        # 横向净空 = 距离 × sin(方位角)，不是 y 坐标：z ≠ 0 时两者差 |p|/ρ。
        point = (0.8, 0.3, 0.5)
        rng = float(np.linalg.norm(point))
        rho = float(np.hypot(point[0], point[1]))
        expected_signed = rng * (point[1] / rho)
        got = summarize(cloud(point), **LIMITS)
        check("横向净空按 距离×sin", got, dict(front_lateral_signed=expected_signed))
        if abs(got["front_lateral_signed"] - point[1]) < 1e-3:
            raise AssertionError("横向净空退化成 y 坐标，z≠0 时会偏")
    except AssertionError as exc:
        print(f"FAIL {exc}")
        return 1
    print(f"全部 {CASES} 个用例通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
