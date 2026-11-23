"""雷达派生量（ObstacleSummary 的 8 个数）的纯几何实现。

文件目的：
    这 8 个数原来只在 MuJoCo 雷达扫描器里算（`dual_car_pick_place.LidarScanner`）。
    真车没有"射线"，只有一帧距离数据，需要在 ROS 侧重算一遍。为了让真车与仿真的
    避障判据完全同源，这里把算法抽成"输入点云、输出 8 个数"的纯函数：不依赖 ROS，
    也不依赖 MuJoCo，只有 numpy。

两条输入路径（扇区规则只有一份）：
    `summarize()`      三维点云（仿真桥发的 `/carN/lidar/points`）；
    `summarize_scan()` 单线 2D 扫描（真车思岚 A2 的 `LaserScan`）。它把扫描点摊到
                       z=0 平面上再走 `summarize()`，所以两边判据不会有第二套实现。
整体输入输出：
    输入是 (N, 3) 点云或 (N,) 距离序列，加上 5 个扇区界限（全部显式传参，不给默认值）；
    输出是与 `mm_msgs/ObstacleSummary` 同名同义的 8 个量，未命中时为 inf
    （front_blocked 为 False）。
坐标约定：
    数据在**雷达自身坐标系**（x 前、y 左、z 上，单位 m）。方位角 az = atan2(y, x)，
    取值 (-π, π]，与仿真扫描器的 azimuth 网格同一个定义；距离是三维欧氏距离
    （2D 扫描时 z=0，退化成平面距离）。
    `front_lateral_signed` 按仿真侧的定义取 `距离 × sin(az)`——注意它**不是** y 坐标，
    z ≠ 0 时两者差一个 |p|/ρ 的因子，直接取 y 会让真车与仿真的横向净空差 1% 量级。
已知限制：
    仿真侧按 geom 名字过滤（只认 obstacle_* 与同伴车），点云里没有这个信息，真车用
    "离地高度 + 自身盲区半径"代替。两者在"除地面与自车外都算障碍"这一点上等价，但
    边界情形（绿框这类低矮结构、玻璃、强反光）会不同，只能上车复核。
    **2D 扫描只看得到雷达所在水平面**：低于该平面的障碍物（台阶、路缘、坑）完全不可见，
    这不是算法能补的，只能靠加装传感器或调整安装高度（真车另有 4 个超声波补前后来车）。

重要性等级：A"""

from __future__ import annotations

import numpy as np

# 扇区边界上的平局规则。仿真扫描器的方位角是 3° 的整数倍，30°/90°/150° 恰好有射线
# 正落在边界上；而这些射线转成 float32 点云再反算 atan2，会偏 1e-7 rad 量级，落点就会
# 在相邻扇区之间跳。这里给边界留 1e-6 rad（约 6e-5 度）的余量，按仿真侧的闭合规则
# 各让一步：30° 归前向、90° 归左右、150° 归正后方，双边的半开半闭保持一致。
AZ_EPS = 1e-6


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
    front = in_range & (np.abs(az) <= front_half + AZ_EPS)
    rear = in_range & (np.abs(az) >= np.pi - rear_half - AZ_EPS)
    rear_wide = in_range & (np.abs(az) > np.pi / 2.0 + AZ_EPS)
    left = in_range & (az > front_half + AZ_EPS) & (az <= side_max + AZ_EPS)
    right = in_range & (az >= -side_max - AZ_EPS) & (az < -front_half - AZ_EPS)

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


def scan_to_points(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_max: float | None = None,
    angle_offset: float = 0.0,
) -> np.ndarray:
    """把单线 2D 扫描的距离序列摊成 z=0 的点云。

    参数：
        ranges: (N,) 距离序列，单位 m；inf / NaN / 非正数视为无效回波。
        angle_min: 第一个采样点的方位角，弧度。
        angle_increment: 相邻采样点的角增量，弧度。
        range_max: 量程上限；给 None 表示不过滤。
        angle_offset: 雷达安装朝向补偿，弧度（默认 0 = 传感器 x 轴与车体 x 轴一致）。
    返回：
        (M, 3) 点云，落在 z=0 平面上。
    重要性等级：A"""
    values = np.asarray(ranges, dtype=np.float64).reshape(-1)
    keep = np.isfinite(values) & (values > 0.0)
    if range_max is not None:
        keep &= values <= float(range_max)
    if not keep.any():
        return np.empty((0, 3))
    index = np.flatnonzero(keep)
    dist = values[index]
    az = angle_min + angle_increment * index + angle_offset
    return np.stack(
        [dist * np.cos(az), dist * np.sin(az), np.zeros_like(dist)], axis=1
    )


def summarize_scan(ranges, angle_min: float, angle_increment: float, **limits):
    """单线 2D 扫描 → 8 个避障派生量（扇区规则复用 `summarize`）。

    参数：
        ranges: (N,) 距离序列，单位 m。
        angle_min: 第一个采样点的方位角，弧度。
        angle_increment: 相邻采样点的角增量，弧度。
        **limits: 透传给 `summarize` 的关键字参数（detect_range / front_half /
            rear_half / side_max / max_range 必填；min_z 与 mount_height 在 2D 下
            没有意义，保持默认）。
    返回：
        与 `summarize` 相同的 8 个量。
    重要性等级：A"""
    points = scan_to_points(
        ranges, angle_min, angle_increment, limits.get("max_range")
    )
    return summarize(points, **limits)
