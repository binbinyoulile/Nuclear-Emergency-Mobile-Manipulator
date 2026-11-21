"""雷达派生量（ObstacleSummary 的 8 个数）的纯几何实现。

文件目的：
    这 8 个数原来只在 MuJoCo 雷达扫描器里算（`dual_car_pick_place.LidarScanner`）。
    真车没有"射线"，只有 VLP-16 的点云，需要在 ROS 侧用点云重算一遍。为了让真车与
    仿真的避障判据完全同源，这里把算法抽成"输入点云、输出 8 个数"的纯函数：
    不依赖 ROS，也不依赖 MuJoCo，只有 numpy。
整体输入输出：
    输入是 (N, 3) 点云与 5 个扇区界限（全部显式传参，不给默认值）；输出是与
    `mm_msgs/ObstacleSummary` 同名同义的 8 个量，未命中时为 inf（front_blocked 为 False）。
坐标约定：
    点云在**雷达自身坐标系**（x 前、y 左、z 上，单位 m）。方位角 az = atan2(y, x)，
    取值 (-π, π]，与仿真扫描器的 azimuth 网格同一个定义；距离是三维欧氏距离。
    `front_lateral_signed` 按仿真侧的定义取 `距离 × sin(az)`——注意它**不是** y 坐标，
    z ≠ 0 时两者差一个 |p|/ρ 的因子，直接取 y 会让真车与仿真的横向净空差 1% 量级。
已知限制：
    仿真侧按 geom 名字过滤（只认 obstacle_* 与同伴车），点云里没有这个信息，真车用
    "离地高度 + 自身盲区半径"代替。两者在"除地面与自车外都算障碍"这一点上等价，但
    边界情形（绿框这类低矮结构、玻璃、强反光）会不同，只能上车复核。

重要性等级：A"""

from __future__ import annotations

import numpy as np


def summarize(
    points: np.ndarray,
    *,
    detect_range: float,
    front_half: float,
    rear_half: float,
    side_max: float,
    max_range: float,
    min_z: float | None = None,
    mount_height: float = 0.0,
    self_radius: float = 0.0,
) -> dict[str, float | bool]:
    """把一帧点云压成 8 个避障派生量。

    参数：
        points: (N, 3) 雷达坐标系点云，单位 m；允许空数组。
        detect_range: 导航警戒距离，米（超出的回波不计入任何扇区）。
        front_half: 前向扇区的半角，弧度。
        rear_half: 正后方扇区的半角，弧度。
        side_max: 左右扇区的外边界，弧度（一般 π/2）。
        max_range: 雷达量程上限，米（超出视为无效回波）。
        min_z: 离地高度门限，米；给 None 表示不做地面过滤（仿真点云已过滤过）。
        mount_height: 雷达安装高度，米（把传感器系 z 换算成离地高度）。
        self_radius: 自身盲区半径，米；0 表示不过滤。
    返回：
        front_blocked / front_min / front_lateral / front_lateral_signed /
        left_min / right_min / rear_min / rear_clear。
    重要性等级：A"""
    empty = {
        "front_blocked": False,
        "front_min": float("inf"),
        "front_lateral": float("inf"),
        "front_lateral_signed": 0.0,
        "left_min": float("inf"),
        "right_min": float("inf"),
        "rear_min": float("inf"),
        "rear_clear": float("inf"),
    }
    cloud = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if cloud.shape[0] == 0:
        return empty

    x = cloud[:, 0]
    y = cloud[:, 1]
    z = cloud[:, 2]
    rho = np.hypot(x, y)
    rng = np.sqrt(x * x + y * y + z * z)
    az = np.arctan2(y, x)

    keep = np.isfinite(rng) & (rng > 0.0) & (rng <= max_range)
    if self_radius > 0.0:
        keep &= rho >= self_radius
    if min_z is not None:
        keep &= (mount_height + z) >= min_z
    if not keep.any():
        return empty

    in_range = keep & (rng <= detect_range)
    front = in_range & (np.abs(az) <= front_half)
    rear = in_range & (np.abs(az) >= np.pi - rear_half)
    rear_wide = in_range & (np.abs(az) > np.pi / 2.0)
    left = in_range & (az > front_half) & (az <= side_max)
    right = in_range & (az >= -side_max) & (az < -front_half)

    result = dict(empty)
    front_idx = np.flatnonzero(front)
    if front_idx.size:
        # 与仿真侧一致：取最近的前向回波，横向净空跟着这一条回波走。
        nearest = front_idx[int(np.argmin(rng[front_idx]))]
        distance = float(rng[nearest])
        result["front_min"] = distance
        result["front_lateral_signed"] = distance * float(np.sin(az[nearest]))
        result["front_lateral"] = abs(result["front_lateral_signed"])
        result["front_blocked"] = distance <= detect_range
    for key, mask in (
        ("left_min", left),
        ("right_min", right),
        ("rear_min", rear),
        ("rear_clear", rear_wide),
    ):
        if mask.any():
            result[key] = float(np.min(rng[mask]))
    return result
