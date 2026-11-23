"""不锈钢监测棒的深度视觉（纯函数，不依赖 ROS）。

与单进程实现（``dual_car_pick_place.py`` 的 ``depth_to_world`` + ``detect_steel_target``）
是同一套算法，只有取参方式不同：

1. 内参来自 ``sensor_msgs/CameraInfo``（``fx/fy/cx/cy``），不再由 ``cam_fovy`` 反推；
2. 相机外参来自 TF（``map → car1/camera_link`` 的平移与旋转），不再读 MuJoCo 的
   ``cam_xpos/cam_xmat``。

算法链路（四层里"感知"的三层）
--------------------------------
反投影点云 → 最小二乘拟合地面 → 取"离地 0.8~30 mm"的点 → 连通域 + 搜索窗过滤 →
二维 PCA 求轴线、1%/99% 分位定两端 → 比较两端外侧 20% 的 90% 分位高度，
粗端 Ø6 mm 的顶面比细端 Ø5 mm 高 1 mm，高的一侧判为粗端 → 从该端沿轴线向内推
16.5 mm 得到**粗端圆柱中心**。

输出语义
--------
``Detection.position`` 是**粗端中心**（粗端 Ø6×33 mm 的轴线中点），不是整根杆的
几何中心、也不是质心。它同时是单进程版的抓取点与物体真值点，三者严格同一点，
下游（抓取、落桶对准）直接拿它当"要抓的位置"用。

为什么不用颜色
--------------
不锈钢本色在中性灰地面上几乎没有色差；为什么不能直接把深度当绝对高度：渲染器
深度带约 0.15% 的系统性尺度偏差，2 m 外的地面会"高出" 1.5 mm，必须先用地平面
拟合把它吸收掉。

相机约定
--------
``car1/camera_link`` 沿用 MuJoCo 相机系：+X 右、+Y 上、+Z 与视线相反，因此像素
(u,v) 对应 ``x_cam=(u-cx)Z/fx``、``y_cam=-(v-cy)Z/fy``、``z_cam=-Z``。真机换成
光学系（+Z 朝前）时，只改 TF 里那一个固定旋转，本模块不动。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np

# 有效深度范围（m）：更近的是机械臂/夹爪自身，更远的对分割没有意义。
DEPTH_MIN = 0.05
DEPTH_MAX = 9.0

# 目标区内侧的搜索窗（map 系，m）：X 最小/最大、Y 最小/最大。
# 默认值与当前场景一致——绿框 X±0.35 / Y 20.70~21.30，内缩 5 cm / 4 cm 把贴地的
# 绿色边框挡在搜索范围外。换场景时由节点参数覆盖。
DEFAULT_ROI = (-0.30, 0.30, 20.74, 21.26)

# 离地高度带（m）：地面 z=0，杆最高处 6 mm；0.8 mm 以下留给地平面拟合残差。
STEEL_HEIGHT_MIN = 0.0008
STEEL_HEIGHT_MAX = 0.030
STEEL_MIN_PIXELS = 15
STEEL_LENGTH_RANGE = (0.10, 0.26)
# 粗端圆柱长度的一半：杆端沿轴线向内推这么多就是粗端中心。
STEEL_THICK_LENGTH = 0.033
# 杆平躺时轴心离地高度（粗端 Ø6 mm 的一半）。抓取点的高度是已知量，不用视觉给。
THICK_END_AXIS_Z = 0.003

# 拟合地平面用的近处地面点：只取 3 m 以内、世界 z 在 ±2 cm 内的像元。
PLANE_MAX_DEPTH = 3.0
PLANE_MAX_ABS_Z = 0.02
PLANE_MIN_POINTS = 500


@dataclass(frozen=True)
class Detection:
    """一次成功的检测（世界系，单位 m）。"""

    position: np.ndarray  # 粗端中心 (x, y, z)
    pixels: int           # 参与拟合的像元数
    length_m: float       # 分割出的杆长
    z_lo_mm: float        # 两端外侧 20% 的 90% 分位高度
    z_hi_mm: float
    thick_source: str     # "高度差"（正常）或 "任务约定朝向"（兜底）


def _warn(log: Callable[[str], None] | None, message: str) -> None:
    """把失败原因交给调用方（节点传 logger.warn，测试传 None）。"""
    if log is not None:
        log(message)


def depth_to_world(
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    cam_pos: np.ndarray,
    cam_rot: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """整幅深度图反投影成世界坐标点云。

    参数：
        depth: 深度图，shape=(H,W)，单位 m，沿相机光轴。
        intrinsics: (fx, fy, cx, cy)，来自 CameraInfo。
        cam_pos: 相机光心世界坐标，shape=(3,)。
        cam_rot: 相机→世界的旋转矩阵，shape=(3,3)。
    返回：
        (world, valid)：world 形状 (H,W,3)、单位 m；valid 为有效深度掩码。
    重要性等级：A"""
    fx, fy, cx, cy = intrinsics
    image_h, image_w = depth.shape[:2]
    valid = np.isfinite(depth) & (depth > DEPTH_MIN) & (depth < DEPTH_MAX)
    d = np.where(valid, depth, 1.0)
    u = np.arange(image_w, dtype=float)[None, :]
    v = np.arange(image_h, dtype=float)[:, None]
    x_cam = (u - cx) * d / fx
    y_cam = -(v - cy) * d / fy
    points_cam = np.stack([x_cam, y_cam, -d], axis=-1)
    world = points_cam @ np.asarray(cam_rot, dtype=float).T + np.asarray(
        cam_pos, dtype=float
    )
    return world, valid


def detect_steel_target(
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    cam_pos: np.ndarray,
    cam_rot: np.ndarray,
    roi: tuple[float, float, float, float] = DEFAULT_ROI,
    log: Callable[[str], None] | None = None,
) -> Detection | None:
    """在搜索窗内分割"离地细长物"，返回粗端中心。

    参数：
        depth: 深度图，shape=(H,W)，单位 m。
        intrinsics: (fx, fy, cx, cy)。
        cam_pos/cam_rot: 相机外参（map 系）。
        roi: 搜索窗 (x_min, x_max, y_min, y_max)，单位 m。
        log: 失败原因回调（节点传 logger.warn；纯计算时传 None）。
    返回：
        Detection；未分割到杆、或分割结果不像 202 mm 的杆时返回 None。
    约束：
        只使用传入的深度帧与相机位姿，不读取物体真值。
    镜像实现：
        单进程侧 `dual_car_pick_place.py` 的 `detect_steel_target()` 是同一算法的另一份
        实现（那份直接从 MuJoCo 深度帧取位姿）。**两份的阈值、滤波顺序与粗端判据必须
        一致**——改这边的常量或分割流程时要同步改那边，否则两条轨道的视觉会悄悄漂开。
    重要性等级：A"""
    world, valid = depth_to_world(depth, intrinsics, cam_pos, cam_rot)

    # 先拟合地面平面再取"离地高度"：渲染深度带约 0.15% 的系统性偏差会让 2 m 外的
    # 地面也"高出" 1.5 mm，直接把世界 z 与 0 比较会把整片地面当成目标。
    near = valid & (depth < PLANE_MAX_DEPTH) & (np.abs(world[..., 2]) < PLANE_MAX_ABS_Z)
    count = int(near.sum())
    if count < PLANE_MIN_POINTS:
        _warn(log, f"地面点太少（{count} < {PLANE_MIN_POINTS}），无法拟合地平面")
        return None
    plane = np.column_stack([world[near][:, 0], world[near][:, 1], np.ones(count)])
    coef = np.linalg.lstsq(plane, world[near][:, 2], rcond=None)[0]
    height = world[..., 2] - (
        coef[0] * world[..., 0] + coef[1] * world[..., 1] + coef[2]
    )
    band = valid & (height > STEEL_HEIGHT_MIN) & (height < STEEL_HEIGHT_MAX)
    if not band.any():
        _warn(log, "离地带内没有点")
        return None

    x_min, x_max, y_min, y_max = roi
    in_roi = (
        (world[..., 0] > x_min)
        & (world[..., 0] < x_max)
        & (world[..., 1] > y_min)
        & (world[..., 1] < y_max)
    )
    total, labels, stats, _ = cv2.connectedComponentsWithStats(
        band.astype(np.uint8), connectivity=8
    )
    best, best_area = -1, 0
    for label in range(1, total):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < STEEL_MIN_PIXELS or area <= best_area:
            continue
        if not (in_roi & (labels == label)).any():
            continue
        best, best_area = label, area
    if best < 0:
        _warn(log, f"离地带内没有落在搜索区里的连通域（候选 {total - 1} 个）")
        return None

    points = world[labels == best]
    flat = points[:, :2]
    mean = flat.mean(axis=0)
    centred = flat - mean
    _, vectors = np.linalg.eigh(centred.T @ centred)
    axis = vectors[:, -1]
    axis = axis / np.linalg.norm(axis)
    along = centred @ axis
    t_lo, t_hi = np.percentile(along, [1.0, 99.0])
    length = float(t_hi - t_lo)
    if not STEEL_LENGTH_RANGE[0] <= length <= STEEL_LENGTH_RANGE[1]:
        _warn(log, f"分割长度 {length * 100:.1f} cm 不在 10~26 cm 内，判为误检")
        return None

    ends = np.stack([mean + axis * t_lo, mean + axis * t_hi])
    span = max(length, 1e-6)
    outer_lo = along <= t_lo + 0.2 * span
    outer_hi = along >= t_hi - 0.2 * span
    z_lo = (
        float(np.percentile(points[outer_lo, 2], 90)) if outer_lo.sum() >= 5 else 0.0
    )
    z_hi = (
        float(np.percentile(points[outer_hi, 2], 90)) if outer_hi.sum() >= 5 else 0.0
    )
    if abs(z_hi - z_lo) >= 2e-4:
        thick_at_lo = z_lo > z_hi
        source = "高度差"
    else:
        # 两端高度差不足 0.2 mm（贴地或反光遮挡严重）：退回任务约定——杆生成时粗端
        # 在原点、细端沿 +X 伸展，所以粗端是更靠 -X 的那一端。
        thick_at_lo = bool(ends[0][0] < ends[1][0])
        source = "任务约定朝向"

    tip_index = 0 if thick_at_lo else 1
    tip = ends[tip_index]
    inward = ends[1 - tip_index] - tip
    inward = inward / max(float(np.linalg.norm(inward)), 1e-6)
    centre = tip + inward * (STEEL_THICK_LENGTH / 2.0)
    return Detection(
        position=np.array(
            [centre[0], centre[1], THICK_END_AXIS_Z], dtype=float
        ),
        pixels=best_area,
        length_m=length,
        z_lo_mm=z_lo * 1000.0,
        z_hi_mm=z_hi * 1000.0,
        thick_source=source,
    )
