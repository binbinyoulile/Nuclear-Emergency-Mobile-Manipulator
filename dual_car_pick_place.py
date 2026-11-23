"""双车协同核废料回收 MuJoCo 仿真入口（仿真后端 + 装配 + 指标落盘）。

文件目的：
    本文件不再持有任务逻辑——七阶段状态机、三个路径控制器与抓取/投放编排都在
    `mission_core.py`（零 MuJoCo）里。这里只做三件事：
      1. 把 `robot_interface` 的端口翻译成 MuJoCo 的 ctrl/qpos 读写（Sim* 适配器）；
      2. 场景初始化与随机生成放射物（`initialize_mission`）；
      3. 离屏渲染取图、雷达点云叠加、截图与单行 RESULT_JSON 指标。

整体输入输出：
    输入为 --seed、--snapshot/--headless 和 MJCF 场景；输出为实时画面、关键截图、
    终端日志及任务成功布尔值。
依赖与版本：
    Python 3.10+、mujoco、mink、numpy、scipy、opencv-python、Pillow；
    模型资产由 scripts/fetch_models.py 获取。
运行方式：
    D:/Python313/python.exe dual_car_pick_place.py
    D:/Python313/python.exe dual_car_pick_place.py --snapshot --seed 7
关键假设：
    XML 内相机名为 d435i，雷达站点、车辆和障碍物命名固定；物体真值只用于误差校验。
已知限制：
    规则控制与理想仿真传感器；雷达检测距离 2.8 m 小于 6.5 m/s 的制动距离 3.5 m。

重要性等级：A"""

from __future__ import annotations

import argparse
import json
import os
import random
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image

HERE = Path(os.path.abspath(__file__)).parent
SNAPSHOT_DIR = HERE / "docs" / "screenshots"
os.environ.setdefault("MOBILE_SCENE_PATH", str(HERE / "dual_car_scene_rg2.xml"))  # UR10+RG2；换回 2F-85 用 dual_car_scene.xml

import mobile_pick_place as base  # noqa: E402
import mission_core  # noqa: E402
from mission_core import (  # noqa: E402
    LIDAR_DETECT_RANGE,
    LIDAR_FRONT_HALF,
    LIDAR_PERIOD,
    LIDAR_REAR_HALF,
    LIDAR_SIDE_MAX,
    OBJECT_X_RANGE,
    OBJECT_Y_RANGE,
    OBJECT_Z,
    ROD_COM_OFFSET_M,
    STEEL_ROI_X,
    STEEL_ROI_Y,
    DualCarMission,
)
from robot_interface import (  # noqa: E402
    ArmInterface,
    ChassisInterface,
    Clock,
    GripperInterface,
    LidarInterface,
    PerceptionInterface,
    RobotIO,
)

# 雷达射线投射配置（纯建模参数，任务层不需要）。方位/俯仰解析度与显示半径只影响
# 画面和点云密度，避障判据用的四个扇区定义在 mission_core 里。
LIDAR_AZ_SAMPLES = 120
LIDAR_CHANNELS = 16
LIDAR_ELEV_MIN = np.deg2rad(-15.0)
LIDAR_ELEV_MAX = np.deg2rad(15.0)
LIDAR_MAX_RANGE = 100.0
LIDAR_VIS_RANGE = 8.0
LIDAR_GEOMGROUP = np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8)

# 调试开关：DUAL_WATCH=1 时按 0.25 s 输出等待中车辆的底盘速度与接触数。
_WATCH = os.environ.get("DUAL_WATCH") == "1"

# 监测棒 body 原点（粗端截面中心）与质心的轴向距离。任务层的投放对中判据依赖它，
# 所以这里用模型的惯性参数复核一次：数值一漂，投放余量会被悄悄吃掉一半。
_OBJ_BODY_ID = mujoco.mj_name2id(
    base.MODEL, mujoco.mjtObj.mjOBJ_BODY, "grasp_object"
)
_ROD_COM_OFFSET_MODEL = float(np.linalg.norm(base.MODEL.body_ipos[_OBJ_BODY_ID]))
if _ROD_COM_OFFSET_MODEL != ROD_COM_OFFSET_M:
    raise RuntimeError(
        "mission_core.ROD_COM_OFFSET_M 与模型的质心偏移不一致："
        f"模型={_ROD_COM_OFFSET_MODEL!r}, 常数={ROD_COM_OFFSET_M!r}"
    )

# 底盘臂座系 → 固定基座（IK 模型）系的旋转：单进程后端直接用模型算出来的，ROS 后端用
# mission_core 里的常数（它没有 MuJoCo）。两边必须逐位相同，否则同一个 CARRY_FIXED 在
# 两条轨道上会落到镜像位置。
if not np.array_equal(base.FIXED_BASE_ROT, mission_core.FIXED_BASE_ROT):
    raise RuntimeError(
        "mission_core.FIXED_BASE_ROT 与 arm_ik_scene.xml 的臂座旋转不一致："
        f"模型=\n{base.FIXED_BASE_ROT}\n常数=\n{mission_core.FIXED_BASE_ROT}"
    )


def _id(obj_type, name: str) -> int:
    """查询 MuJoCo 命名对象并校验是否存在。

参数：
    obj_type: mjtObj 对象类型，例如 mjOBJ_BODY 或 mjOBJ_ACTUATOR。
    name: MJCF 中的对象名。
返回：
    非负整数对象下标。
异常：
    名称不存在时抛 RuntimeError，便于尽早发现 XML 与控制代码不一致。
重要性等级：B"""
    value = mujoco.mj_name2id(base.MODEL, obj_type, name)
    if value < 0:
        raise RuntimeError(f"missing {obj_type}: {name}")
    return int(value)


PINCH_SITE = _id(mujoco.mjtObj.mjOBJ_SITE, "pinch")


def body_pose(body_name: str) -> tuple[np.ndarray, float]:
    """读取刚体世界位置与偏航角。

参数：
    body_name: MuJoCo body 名称。
返回：
    (pos, yaw)，pos shape=(3,) 单位 m；yaw 取世界矩阵 XY 分量，单位 rad。
副作用：
    调用 mj_forward 刷新派生量。
异常：
    名称缺失时由 _id 抛 RuntimeError。
重要性等级：B"""
    mujoco.mj_forward(base.MODEL, base.DATA)
    body_id = _id(mujoco.mjtObj.mjOBJ_BODY, body_name)
    pos = base.DATA.xpos[body_id].copy()
    rot = base.DATA.xmat[body_id].reshape(3, 3)
    yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    return pos, yaw


def yaw_quaternion(yaw: float) -> list[float]:
    """把绕 Z 轴的偏航角转换为 wxyz 四元数。

参数：
    yaw: 偏航角，单位 rad。
返回：
    [w, x, y, z] 列表，可直接写入 MuJoCo free joint 的 qpos。
重要性等级：C"""
    return [
        float(np.cos(yaw / 2.0)),
        0.0,
        0.0,
        float(np.sin(yaw / 2.0)),
    ]


def _set_scene_geom(scene, point: np.ndarray, color: np.ndarray) -> None:
    """向用户叠加场景追加一个雷达点球体。

参数：
    scene: mujoco.MjvScene，通常是 viewer.user_scn 或 Renderer 内部场景。
    point: 世界坐标，shape=(3,)，单位 m。
    color: RGBA 颜色，shape=(4,)，范围 [0, 1]。
副作用：
    scene.ngeom 未达上限时修改其 geoms 并递增 ngeom；达到上限时静默跳过。
重要性等级：C"""
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.018, 0.0, 0.0]),
        point,
        np.eye(3).reshape(-1),
        color,
    )
    scene.ngeom += 1


class LidarScanner(LidarInterface):
    """用 mj_multiRay 模拟 VLP-16 Lite 的 16 线 360 度雷达。

    关键参数：
        10 Hz 更新，水平 360 度，垂直 -15 到 +15 度，理论最大 100 m；
        导航只使用前向 LIDAR_DETECT_RANGE 内的障碍回波。
    动态障碍处理：
        排除自车，并把另一辆车固定视为动态障碍；障碍物仅接受名称以 obstacle_ 开头的 geom。
    重要性等级：A"""

    def __init__(
        self,
        site_name: str,
        own_body_name: str,
        other_body_name: str,
    ) -> None:
        """初始化雷达站点、自车根体、动态障碍与最近距离缓存。

参数：
    site_name: 雷达 site 名称。
    own_body_name: 自车 chassis body 名称，用于过滤自身回波。
    other_body_name: 另一辆车 chassis body 名称，作为动态障碍。
异常：
    任一名称缺失时由 _id 抛 RuntimeError。
重要性等级：A"""
        self.site_id = _id(mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.own_root = int(
            base.MODEL.body_rootid[_id(mujoco.mjtObj.mjOBJ_BODY, own_body_name)]
        )
        # 同伴车的刚体根索引：peer_avoidance 为 True 时算作动态障碍。
        self.peer_roots = {
            int(base.MODEL.body_rootid[
                _id(mujoco.mjtObj.mjOBJ_BODY, other_body_name)
            ])
        }
        self.peer_avoidance = True
        self.obstacle_geoms = {
            geom_id
            for geom_id in range(base.MODEL.ngeom)
            if (
                (name := mujoco.mj_id2name(
                    base.MODEL, mujoco.mjtObj.mjOBJ_GEOM, geom_id
                ))
                is not None
                and name.startswith("obstacle_")
            )
        }
        self.last_scan_time = -1e9
        self.front_blocked = False
        self.front_min = float("inf")
        self.front_lateral = float("inf")
        self.front_lateral_signed = 0.0
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.rear_min = float("inf")
        # 整个后半平面（|azimuth| > 90°）的最近回波。车头雷达本身就是 360°，
        # 只统计 ±30° 会看不到斜后方来车，倒车时存在盲区。
        self.rear_clear = float("inf")
        self.vis_points: list[np.ndarray] = []
        self.front_points: list[np.ndarray] = []
        # 雷达坐标系下的有效回波点，供 ROS 桥转成 PointCloud2 给感知节点重算摘要；
        # 单进程路径不使用它，因此不影响任何指标。
        self.hit_local = np.empty((0, 3), dtype=np.float32)

    def scan_if_due(self) -> None:
        """按 10 Hz 周期触发雷达扫描。

副作用：
    仿真时间距上次扫描至少 LIDAR_PERIOD 时调用 scan，否则保持旧点云。
重要性等级：B"""
        if base.DATA.time - self.last_scan_time >= LIDAR_PERIOD:
            self.scan()

    def scan(self) -> None:
        """执行一次全向多线射线扫描并更新避障缓存。

返回：
    无显式返回值；更新 front/rear/left/right 距离以及 vis_points。
关键参数：
    LIDAR_DETECT_RANGE 决定导航警戒距离；LIDAR_VIS_RANGE 只影响画面展示。
副作用：
    读取并刷新仿真派生状态，不推进物理时间。
重要性等级：A"""
        mujoco.mj_forward(base.MODEL, base.DATA)
        origin = base.DATA.site_xpos[self.site_id].copy()
        rot = base.DATA.site_xmat[self.site_id].reshape(3, 3).copy()

        elevations = np.linspace(LIDAR_ELEV_MIN, LIDAR_ELEV_MAX, LIDAR_CHANNELS)
        azimuths = np.linspace(-np.pi, np.pi, LIDAR_AZ_SAMPLES, endpoint=False)
        az_grid, elev_grid = np.meshgrid(azimuths, elevations)
        az_flat = az_grid.reshape(-1)
        elev_flat = elev_grid.reshape(-1)
        local_dirs = np.stack(
            [
                np.cos(elev_flat) * np.cos(az_flat),
                np.cos(elev_flat) * np.sin(az_flat),
                np.sin(elev_flat),
            ],
            axis=1,
        )
        world_dirs = local_dirs @ rot.T
        nray = int(world_dirs.shape[0])
        geomid = np.zeros(nray, dtype=np.int32)
        distances = np.zeros(nray, dtype=np.float64)
        mujoco.mj_multiRay(
            base.MODEL,
            base.DATA,
            origin,
            world_dirs.reshape(-1),
            LIDAR_GEOMGROUP,
            True,
            -1,
            geomid,
            distances,
            None,
            nray,
            LIDAR_MAX_RANGE,
        )

        dynamic_roots = self.peer_roots if self.peer_avoidance else set()
        vis_points: list[np.ndarray] = []
        front_points: list[np.ndarray] = []
        hit_local: list[np.ndarray] = []
        front_min = float("inf")
        front_lateral = float("inf")
        front_lateral_signed = 0.0
        left_min = float("inf")
        right_min = float("inf")
        rear_min = float("inf")
        rear_clear = float("inf")
        for idx in range(nray):
            distance = float(distances[idx])
            geom_id = int(geomid[idx])
            if distance < 0.0 or distance > LIDAR_MAX_RANGE:
                continue
            geom_root = int(base.MODEL.body_rootid[base.MODEL.geom_bodyid[geom_id]])
            if geom_root == self.own_root:
                continue
            if (
                geom_id not in self.obstacle_geoms
                and geom_root not in dynamic_roots
            ):
                continue
            hit = origin + distance * world_dirs[idx]
            if hit[2] < 0.08:
                continue
            hit_local.append(local_dirs[idx] * distance)
            az = float(az_flat[idx])
            is_front = (
                abs(az) <= LIDAR_FRONT_HALF
                and distance <= LIDAR_DETECT_RANGE
            )
            if is_front:
                front_points.append(hit.copy())
                if distance < front_min:
                    front_min = distance
                    front_lateral_signed = distance * np.sin(az)
                    front_lateral = abs(front_lateral_signed)
            if distance <= LIDAR_DETECT_RANGE:
                if abs(az) >= np.pi - LIDAR_REAR_HALF:
                    rear_min = min(rear_min, distance)
                if abs(az) > np.pi / 2.0:
                    rear_clear = min(rear_clear, distance)
                if LIDAR_FRONT_HALF < az <= LIDAR_SIDE_MAX:
                    left_min = min(left_min, distance)
                elif -LIDAR_SIDE_MAX <= az < -LIDAR_FRONT_HALF:
                    right_min = min(right_min, distance)
            if distance <= LIDAR_VIS_RANGE:
                vis_points.append(
                    (origin + max(0.0, distance - 0.025) * world_dirs[idx]).copy()
                )

        self.vis_points = vis_points
        self.front_points = front_points
        self.hit_local = np.asarray(hit_local, dtype=np.float32).reshape(-1, 3)
        self.front_min = front_min
        self.front_lateral = front_lateral
        self.front_lateral_signed = front_lateral_signed
        self.left_min = left_min
        self.right_min = right_min
        self.rear_min = rear_min
        self.rear_clear = rear_clear
        self.front_blocked = front_min <= LIDAR_DETECT_RANGE
        self.last_scan_time = base.DATA.time


def update_user_scene(
    viewer, cars: tuple["CarController", ...] | list["CarController"]
) -> None:
    """把两车最新雷达点云叠加到实时 viewer 画面。

参数：
    viewer: 被动 viewer；None 时直接返回。
    cars: 两辆 CarController，提供雷达点与颜色。
关键参数：
    前向警戒扇区内的点显示为红色，其余点使用各车颜色。
重要性等级：B"""
    if viewer is None:
        return
    scene = viewer.user_scn
    scene.ngeom = 0
    for car in cars:
        for point in car.lidar.vis_points:
            is_front = any(
                np.linalg.norm(point - front) < 0.04
                for front in car.lidar.front_points
            )
            color = (
                np.array([1.0, 0.15, 0.05, 1.0])
                if is_front
                else car.point_color
            )
            _set_scene_geom(scene, point, color)


def capture_snapshot(
    path: str,
    cars: tuple["CarController", ...] | list["CarController"],
    lookat: list[float],
    distance: float,
    azimuth: float,
    elevation: float,
) -> None:
    """离屏渲染并保存指定相机视角的 PNG 截图。

参数：
    path: 输出文件名，最终统一保存到 docs/screenshots。
    cars: 需要叠加雷达点的车辆。
    lookat/distance/azimuth/elevation: MuJoCo 自由相机参数。
副作用：
    创建高 480、宽 640 的 Renderer，写图片文件并打印路径。
异常：
    路径不可写或 Renderer 初始化失败时由底层库抛出。
重要性等级：B"""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    with mujoco.Renderer(base.MODEL, height=480, width=640) as renderer:
        renderer.update_scene(base.DATA, camera=cam)
        scene = renderer._scene
        for car in cars:
            for point in car.lidar.vis_points:
                point_front = any(
                    np.linalg.norm(point - front) < 0.04
                    for front in car.lidar.front_points
                )
                color = (
                    np.array([1.0, 0.15, 0.05, 1.0])
                    if point_front
                    else car.point_color
                )
                _set_scene_geom(scene, point, color)
        output_path = SNAPSHOT_DIR / Path(path).name
        output_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(output_path)
    print("  saved", output_path)


# ---------------------------------------------------------------------------
# D435i 四层视觉识别管线。
# ---------------------------------------------------------------------------
VISION_LAST: dict[str, np.ndarray] = {}


def get_camera_data() -> tuple[np.ndarray, np.ndarray]:
    """视觉第 1 层：采集腕部 D435i 的 RGB 与深度帧。

功能：
    使用 mujoco.Renderer(height=480, width=640)，显式指定 camera=d435i；先渲染 RGB，再启用深度渲染取深度并关闭。
返回：
    (rgb, depth)：RGB shape=(480,640,3)，深度 shape=(480,640)，单位 m。
副作用：
    写入 VISION_LAST 中的 rgb、depth、cam_pos、cam_mat，并打印相机局部安装位姿。
Sim2Real：
    ROS 2 里这一步由 sim_bridge 的 /car1/camera/capture 服务加图像话题完成。
重要性等级：A"""
    cam_id = _id(mujoco.mjtObj.mjOBJ_CAMERA, "d435i")
    with mujoco.Renderer(base.MODEL, height=480, width=640) as renderer:
        renderer.update_scene(base.DATA, camera="d435i")
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
    VISION_LAST["rgb"] = rgb
    VISION_LAST["depth"] = depth
    VISION_LAST["cam_pos"] = base.DATA.cam_xpos[cam_id].copy()
    VISION_LAST["cam_mat"] = base.DATA.cam_xmat[cam_id].reshape(3, 3).copy()
    print(
        f"[相机安装] local_pos={np.round(base.MODEL.cam_pos[cam_id], 4)}, "
        f"local_quat={np.round(base.MODEL.cam_quat[cam_id], 4)}"
    )
    return rgb, depth


# 不锈钢棒的视觉参数 ---------------------------------------------------------
# 不锈钢本色在中性灰地面上几乎没有颜色对比，所以视觉不再用颜色，改用"离地高度"这一
# 几何特征：地面是 z=0 的平面，棒最高处 6 mm。把反投影后落在地面上方 0.8~30 mm 的点
# 取出来，就正好是一根棒子的可见表面——对光照、反光、颜色都不敏感。
STEEL_HEIGHT_MIN = 0.0008
STEEL_HEIGHT_MAX = 0.030
STEEL_MIN_PIXELS = 15
STEEL_LENGTH_RANGE = (0.10, 0.26)
STEEL_THICK_LENGTH = 0.033


def depth_to_world(depth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """视觉第 2 层：把整幅深度图反投影成世界坐标点云。

    参数：
        depth: 深度图，shape=(H,W)，单位 m，沿相机光轴。
    返回：
        (world, valid)：world 形状 (H,W,3)、单位 m；valid 是有效深度掩码。
    算法：
        fx=fy=(H/2)/tan(fovy/2)，cx=W/2，cy=H/2；像素 (u,v) 在相机系中的坐标是
        (x,y,z)=((u-cx)d/fx, -(v-cy)d/fy, -d)，再经相机旋转换到世界系。
        MuJoCo 相机 +Z 与视线方向相反，因此深度取负。
    重要性等级：A"""
    image_h, image_w = depth.shape[:2]
    cam_id = _id(mujoco.mjtObj.mjOBJ_CAMERA, "d435i")
    fovy = np.deg2rad(float(base.MODEL.cam_fovy[cam_id]))
    fy = (image_h / 2.0) / np.tan(fovy / 2.0)
    fx = fy
    valid = np.isfinite(depth) & (depth > 0.05) & (depth < 9.0)
    d = np.where(valid, depth, 1.0)
    u = np.arange(image_w, dtype=float)[None, :]
    v = np.arange(image_h, dtype=float)[:, None]
    x_cam = (u - image_w / 2.0) * d / fx
    y_cam = -(v - image_h / 2.0) * d / fy
    points_cam = np.stack([x_cam, y_cam, -d], axis=-1)
    cam_pos = VISION_LAST.get("cam_pos", base.DATA.cam_xpos[cam_id])
    cam_mat = VISION_LAST.get(
        "cam_mat", base.DATA.cam_xmat[cam_id].reshape(3, 3)
    )
    world = points_cam @ np.asarray(cam_mat).T + np.asarray(cam_pos)
    return world, valid


def detect_steel_target(
    depth: np.ndarray,
    verbose: bool = True,
) -> np.ndarray | None:
    """视觉第 2 层：在目标区内分割"离地条状物"，拟合轴线并定位粗端。

    参数：
        depth: 深度图，shape=(H,W)，单位 m。
        verbose: 是否打印分割统计（诊断用）。
    返回：
        粗端圆柱中心的世界坐标 shape=(3,)，单位 m；未找到时返回 None。
    算法：
        ① 反投影得到点云，取 z 落在 (0.8, 30) mm 的点；
        ② 连通域分解，只保留与搜索区有交集的连通域（排除绿色边框等区域外结构）；
        ③ 对最大连通域做二维 PCA 得到轴线，投影后取 1%/99% 分位当作两端；
        ④ 两端各取外侧 20% 的点、比较 90% 分位高度：粗端 Ø6 mm 的顶面比细端高
           1 mm，高的一侧判为粗端；差异不足 0.2 mm 时退回任务约定的朝向（粗端
           在 -X 一侧）；
        ⑤ 粗端"杆端"沿轴线向内推 16.5 mm，得到粗端圆柱中心——真值点、抓取点、
           视觉点在这里三者重合。
    约束：
        只使用传入的深度帧与相机位姿，不读取物体真值。
    重要性等级：A"""
    world, valid = depth_to_world(depth)
    # 先拟合地面平面再取"离地高度"。渲染器给出的深度带约 0.15% 的系统性偏差，
    # 直接把世界 z 与 0 比较会让 2 m 外的地面也"高出 1.5 mm"，整片地面被误当成目标。
    # 最小二乘拟合可以把这个偏差连同一阶倾斜一起消掉；筷子只占千分之几的像素，
    # 不会把平面带偏。
    near = valid & (depth < 3.0) & (np.abs(world[..., 2]) < 0.02)
    if int(near.sum()) < 500:
        return None
    plane = np.column_stack(
        [world[near][:, 0], world[near][:, 1], np.ones(int(near.sum()))]
    )
    coef = np.linalg.lstsq(plane, world[near][:, 2], rcond=None)[0]
    height = world[..., 2] - (
        coef[0] * world[..., 0] + coef[1] * world[..., 1] + coef[2]
    )
    band = valid & (height > STEEL_HEIGHT_MIN) & (height < STEEL_HEIGHT_MAX)
    if not band.any():
        return None
    roi = (
        (world[..., 0] > STEEL_ROI_X[0])
        & (world[..., 0] < STEEL_ROI_X[1])
        & (world[..., 1] > STEEL_ROI_Y[0])
        & (world[..., 1] < STEEL_ROI_Y[1])
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        band.astype(np.uint8), connectivity=8
    )
    best, best_area = -1, 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < STEEL_MIN_PIXELS or area <= best_area:
            continue
        if not (roi & (labels == label)).any():
            continue
        best, best_area = label, area
    if best < 0:
        if verbose:
            print(f"  [视觉] 离地带内没有落在搜索区里的连通域（候选 {count - 1} 个）")
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
        if verbose:
            print(f"  [视觉] 分割长度 {length * 100:.1f} cm 不在筷子长度范围内，判为误检")
        return None

    ends = np.stack([mean + axis * t_lo, mean + axis * t_hi])
    span = max(length, 1e-6)
    outer_lo = along <= t_lo + 0.2 * span
    outer_hi = along >= t_hi - 0.2 * span
    z_lo = float(np.percentile(points[outer_lo, 2], 90)) if outer_lo.sum() >= 5 else 0.0
    z_hi = float(np.percentile(points[outer_hi, 2], 90)) if outer_hi.sum() >= 5 else 0.0
    if abs(z_hi - z_lo) >= 2e-4:
        thick_at_lo = z_lo > z_hi
        source = "高度差"
    else:
        # 两端高度差不足 0.2 mm（贴地或反光遮挡严重）：退回任务约定——筷子生成时
        # 粗端在原点、细端沿 +X 伸展，所以粗端是更靠 -X 的那一端。
        thick_at_lo = bool(ends[0][0] < ends[1][0])
        source = "任务约定朝向"
    tip_index = 0 if thick_at_lo else 1
    tip = ends[tip_index]
    inward = ends[1 - tip_index] - tip
    inward = inward / max(float(np.linalg.norm(inward)), 1e-6)
    centre = tip + inward * (STEEL_THICK_LENGTH / 2.0)
    if verbose:
        print(
            f"  [视觉] 分割 {best_area} 像素，长度 {length * 100:.1f} cm，"
            f"两端高度 {z_lo * 1000:.2f}/{z_hi * 1000:.2f} mm，"
            f"粗端判据={source}"
        )
    return np.array([centre[0], centre[1], OBJECT_Z], dtype=float)


def process_camera_frame(
    rgb: np.ndarray,
    depth: np.ndarray,
) -> np.ndarray:
    """在冻结帧上做分割、粗端定位与误差校验。

    参数：
        rgb: 已采集 RGB 帧，shape=(480,640,3)——本代视觉不用颜色，保留是为了与
             ROS 2 话题契约（彩色图 + 深度图）一致。
        depth: 已采集深度帧，shape=(480,640)，单位 m。
    返回：
        视觉估计的粗端世界坐标 shape=(3,)，单位 m。
    异常：
        未分割到目标或误差大于 0.03 m 时抛 RuntimeError。
    重要性等级：A"""
    del rgb
    print("[VISION] processing frozen depth frame")
    vision_world = detect_steel_target(depth)
    if vision_world is None:
        raise RuntimeError("stainless rod not found in D435i depth frame")

    truth = get_object_truth_for_validation()
    error = float(np.linalg.norm(vision_world - truth))
    print(
        f"[视觉识别] 粗端世界坐标(X_vis={vision_world[0]:.4f}, "
        f"Y_vis={vision_world[1]:.4f}, Z_vis={vision_world[2]:.4f})"
    )
    print(
        f"           仿真真值(X_true={truth[0]:.4f}, Y_true={truth[1]:.4f}, "
        f"Z_true={truth[2]:.4f}), 误差={error * 1000.0:.2f} mm"
    )
    if error > 0.03:
        raise RuntimeError(f"vision error too large: {error * 1000.0:.2f} mm")
    return vision_world


def get_object_truth_for_validation() -> np.ndarray:
    """读取物体真实坐标，仅用于视觉结果误差校验。

返回：
    物体世界坐标 shape=(3,)，单位 m。
约束：
    不参与检测或定位；必须在视觉坐标已经算完之后调用。
重要性等级：A"""
    return base.DATA.qpos[
        base.OBJ_QADR : base.OBJ_QADR + 3
    ].copy()


# ---------------------------------------------------------------------------
# 仿真侧底盘：把 ChassisInterface 翻译成 ctrl/qpos 读写
# ---------------------------------------------------------------------------
class CarController(ChassisInterface):
    """封装单辆车共用的四轮底盘、雷达和运动指令状态。

数据：
    wheel_act 按 FL/FR/RL/RR 排列；left_cmd/right_cmd 为轮角速度指令，steer_cmd 为前轮角。
接口：
    实现 `robot_interface.ChassisInterface`，任务层只依赖该接口，不关心底层是 MuJoCo 还是真车。
重要性等级：A"""

    def __init__(
        self,
        name: str,
        chassis_body: str,
        wheel_joint_names: list[str],
        steer_joint_names: list[str],
        lidar_site: str,
        point_color: np.ndarray,
        other_body: str,
        free_joint_name: str,
    ) -> None:
        """绑定一辆车的关节、执行器、雷达和动态障碍。

参数：
    name: 日志名称，如 CAR1。
    chassis_body/free_joint_name: 底盘 body 与自由关节名。
    wheel_joint_names/steer_joint_names: 轮子滚动关节和转向关节名。
    lidar_site: 雷达 site 名。
    point_color: 点云 RGBA 显示颜色。
    other_body: 另一辆车底盘 body 名。
异常：
    任一名称缺失时由 _id 抛 RuntimeError。
重要性等级：A"""
        self.name = name
        self.chassis_body = chassis_body
        self.body_id = _id(mujoco.mjtObj.mjOBJ_BODY, chassis_body)
        free_joint = _id(mujoco.mjtObj.mjOBJ_JOINT, free_joint_name)
        self.qadr = int(base.MODEL.jnt_qposadr[free_joint])
        self.dadr = int(base.MODEL.jnt_dofadr[free_joint])
        self.wheel_act = np.array(
            [
                _id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint.replace('_joint', '')}_act")
                for joint in wheel_joint_names
            ],
            dtype=int,
        )
        self.steer_act = np.array(
            [
                _id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint.replace('_joint', '')}_act")
                for joint in steer_joint_names
            ],
            dtype=int,
        )
        self.lidar = LidarScanner(
            lidar_site, chassis_body, other_body
        )
        self.point_color = np.asarray(point_color, dtype=float)
        self.other_body = other_body
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.steer_cmd = 0.0
        self.min_other_separation = float("inf")

    def pose(self) -> tuple[np.ndarray, float]:
        """读取本车世界位置和偏航角。

返回：
    (pos, yaw)，单位分别为 m 和 rad。
重要性等级：B"""
        return body_pose(self.chassis_body)

    def speed(self) -> float:
        """读取底盘自由关节的平面线速度。

返回：
    sqrt(vx^2+vy^2)，单位 m/s。
重要性等级：C"""
        return float(np.linalg.norm(base.DATA.qvel[self.dadr : self.dadr + 2]))

    # --- ChassisInterface 实现 -------------------------------------------------
    def set_wheels(self, left: float, right: float, steer: float) -> None:
        """写入左右轮速与前轮转向执行器。

参数：
    left/right: 左右轮角速度，单位 rad/s。
    steer: 前轮转向角，单位 rad。
关键参数：
    wheel_act 下标 0/2 为左轮，1/3 为右轮；左右分配错误会导致原地打转。
重要性等级：A"""
        base.DATA.ctrl[self.wheel_act[[0, 2]]] = left
        base.DATA.ctrl[self.wheel_act[[1, 3]]] = right
        if self.steer_act.size:
            base.DATA.ctrl[self.steer_act] = steer

    def wheel_speeds(self) -> np.ndarray:
        """ChassisInterface：实测四轮角速度，顺序 FL/FR/RL/RR，单位 rad/s。"""
        return np.array(
            [
                base.DATA.qvel[base.MODEL.jnt_dofadr[base.MODEL.actuator_trnid[a, 0]]]
                for a in self.wheel_act
            ],
            dtype=float,
        )

    def contacts_with(self, other: "ChassisInterface") -> int:
        """ChassisInterface：与本车相关的、与另一辆车的物理接触对数量。"""
        own_root = int(base.MODEL.body_rootid[self.body_id])
        other_root = int(base.MODEL.body_rootid[other.body_id])
        count = 0
        for index in range(base.DATA.ncon):
            contact = base.DATA.contact[index]
            roots = {
                int(base.MODEL.body_rootid[base.MODEL.geom_bodyid[contact.geom1]]),
                int(base.MODEL.body_rootid[base.MODEL.geom_bodyid[contact.geom2]]),
            }
            if roots == {own_root, other_root}:
                count += 1
        return count


# ---------------------------------------------------------------------------
# 仿真侧适配器：把 robot_interface 的抽象接口翻译成 MuJoCo 读写
# ---------------------------------------------------------------------------
def _watch_waiting_car(car: CarController) -> None:
    """采样等待中车辆的底盘线速度、轮速与接触数（仅调试用）。

参数：
    car: 被采样的车辆（通常是等待中的二号车）。
副作用：
    按 0.25 s 汇总打印一行 WATCH；数据缓存在 car._watch 上。
重要性等级：C"""
    vlin = base.DATA.qvel[car.dadr : car.dadr + 2]
    vrot = base.DATA.qvel[car.dadr + 3 : car.dadr + 6]
    pos = base.DATA.xpos[car.body_id]
    acc = getattr(car, "_watch", None) or {
        "v": 0.0, "r": 0.0, "w": 0.0, "t": base.DATA.time,
        "p0": pos[:2].copy(),
    }
    acc["v"] = max(acc["v"], float(np.linalg.norm(vlin)))
    acc["r"] = max(acc["r"], float(np.linalg.norm(vrot)))
    acc["w"] = max(
        acc["w"],
        float(np.max(np.abs([
            base.DATA.qvel[base.MODEL.jnt_dofadr[base.MODEL.actuator_trnid[a, 0]]]
            for a in car.wheel_act
        ]))),
    )
    # 记录与本车相关的接触（含滚轮/桶/臂），用于判断抽动是否由外力接触引起。
    car_root = int(base.MODEL.body_rootid[car.body_id])
    n_car = 0
    for i in range(base.DATA.ncon):
        contact = base.DATA.contact[i]
        roots = {
            int(base.MODEL.body_rootid[base.MODEL.geom_bodyid[contact.geom1]]),
            int(base.MODEL.body_rootid[base.MODEL.geom_bodyid[contact.geom2]]),
        }
        if car_root in roots:
            n_car += 1
    car._watch = acc
    if base.DATA.time - acc["t"] < 0.25:
        return
    drift = float(np.linalg.norm(pos[:2] - acc["p0"]))
    print(
        f"WATCH {car.name} t={base.DATA.time:6.2f} "
        f"max|v|={acc['v']:.6f} max|w_rot|={acc['r']:.6f} "
        f"max|wq|={acc['w']:.4f} ncon={base.DATA.ncon} carcon={n_car} "
        f"drift={drift:.4f}"
    )
    acc["v"] = 0.0
    acc["r"] = 0.0
    acc["w"] = 0.0
    acc["t"] = base.DATA.time


class SimClock(Clock):
    """MuJoCo 仿真的时钟与单帧推进。

    任务层只通过它取时间、推进物理，因此换真机时只要替换本类即可，
    上层状态机不需要认识 `mujoco`。
    重要性等级：A"""

    def __init__(self, sim: base.Simulation, cars: list[CarController]) -> None:
        self._sim = sim
        self._cars = cars

    @property
    def now(self) -> float:
        """当前仿真时间，单位 s。"""
        return float(base.DATA.time)

    @property
    def dt(self) -> float:
        """单个控制帧时长，单位 s。"""
        return float(self._sim.dt)

    def step(self) -> None:
        """推进一帧物理（含 viewer 同步）。"""
        self._sim.step()
        if _WATCH:
            _watch_waiting_car(self._cars[1])


class SimArm(ArmInterface):
    """把 `base.ArmMotion` 包装成 ArmInterface。

    关键参数：
        IK 与关节插值都走原有的 mink 求解器与重力补偿，行为与重构前完全一致。
    重要性等级：A"""

    def __init__(self, arm: base.ArmMotion, clock: Clock) -> None:
        self._arm = arm
        self._clock = clock

    def joint_positions(self) -> np.ndarray:
        return base.DATA.qpos[base.ARM_QADR].copy()

    def set_joint_targets(self, q: np.ndarray) -> None:
        self._arm.set_ctrl(q)

    def start_fixed(
        self,
        target_pos_fixed: np.ndarray,
        duration: float,
        target_quat: np.ndarray | None = None,
    ) -> None:
        self._arm.start_fixed(
            target_pos_fixed,
            duration,
            base.TARGET_QUAT if target_quat is None else target_quat,
        )

    def start_world(
        self,
        target_pos_world: np.ndarray,
        duration: float,
        target_quat: np.ndarray | None = None,
    ) -> None:
        """世界坐标目标位置 → 固定基座坐标系位置后启动运动（姿态原样透传）。"""
        target_fixed, _ = base.world_target_to_fixed(
            np.asarray(target_pos_world, dtype=float)
        )
        quat = (
            base.TARGET_QUAT
            if target_quat is None
            else np.asarray(target_quat, dtype=float)
        )
        self._arm.start_fixed(target_fixed, duration, quat)

    def tick(self) -> None:
        self._arm.update(self._clock.dt)

    def hold(self) -> None:
        self._arm.hold()

    @property
    def active(self) -> bool:
        return bool(self._arm.active)

    def tool_position(self) -> np.ndarray:
        """夹持点（pinch 站点）的世界坐标。"""
        return base.DATA.site_xpos[PINCH_SITE].copy()

    def solve_ik(
        self, pos_world: np.ndarray, target_quat: np.ndarray | None = None
    ) -> tuple[np.ndarray, float]:
        pos_fixed, quat_fixed = base.world_target_to_fixed(
            np.asarray(pos_world, dtype=float)
        )
        if target_quat is not None:
            quat_fixed = np.asarray(target_quat, dtype=float)
        return base.solve_ik_fixed(
            pos_fixed, quat_fixed, seed_q=base.DATA.qpos[base.ARM_QADR]
        )


class SimGripper(GripperInterface):
    """把 `base.FingerMotion` 包装成 GripperInterface。"""

    def __init__(self, finger: base.FingerMotion, clock: Clock) -> None:
        self._finger = finger
        self._clock = clock

    def start(self, value: float, duration: float) -> None:
        self._finger.start(value, duration)

    def tick(self) -> None:
        self._finger.update(self._clock.dt)

    def hold(self) -> None:
        self._finger.hold()

    @property
    def active(self) -> bool:
        return bool(self._finger.active)

    def force(self) -> float:
        return float(abs(base.DATA.actuator_force[base.FINGER_ACT]))


class SimPerception(PerceptionInterface):
    """仿真感知：离屏渲染取图 + 深度离地分割定位，真值仅用于验收。

    说明：
        像素级处理（渲染、反投影、连通域、PCA）都封在本类内部；
        任务层只看到 `estimate_target()` 与 `object_pose()`。
    """

    def estimate_target(self) -> np.ndarray:
        rgb, depth = get_camera_data()
        return process_camera_frame(rgb, depth)

    def object_pose(self) -> tuple[np.ndarray, np.ndarray]:
        pos = base.DATA.qpos[base.OBJ_QADR : base.OBJ_QADR + 3].copy()
        quat = base.DATA.qpos[base.OBJ_QADR + 3 : base.OBJ_QADR + 7].copy()
        return pos, quat


def build_sim_robot_io(
    sim: base.Simulation,
    arm: base.ArmMotion,
    finger: base.FingerMotion,
    car1: CarController,
    car2: CarController,
) -> RobotIO:
    """把仿真对象组装成任务层握有的 RobotIO。

参数：
    sim/arm/finger: MuJoCo 侧的执行器封装。
    car1/car2: 两辆底盘，均实现 ChassisInterface。
返回：
    任务层唯一允许使用的硬件端口集合。
重要性等级：A"""
    cars = [car1, car2]
    clock = SimClock(sim, cars)
    return RobotIO(
        clock=clock,
        chassis=(car1, car2),
        arm=SimArm(arm, clock),
        gripper=SimGripper(finger, clock),
        perception=SimPerception(),
        frame_hook=lambda: update_user_scene(sim.viewer, cars),
    )


def build_robot_io(backend: str, **kwargs) -> RobotIO:
    """按后端名构造 RobotIO；目前只实现 sim。

参数：
    backend: "sim"（默认）。
异常：
    其他后端名抛 NotImplementedError，并给出接入位置提示。
重要性等级：A"""
    if backend == "sim":
        return build_sim_robot_io(
            kwargs["sim"], kwargs["arm"], kwargs["finger"],
            kwargs["car1"], kwargs["car2"],
        )
    raise NotImplementedError(
        f"backend={backend!r} 尚未实现：真机适配器应实现 robot_interface 的 "
        f"Clock/Lidar/Chassis/Arm/Gripper/Perception 后在此注册"
    )


def make_snapshot_hook(cars: list[CarController]):
    """把 `capture_snapshot` 包装成任务层可用的一元回调。

参数：
    cars: 需要叠加雷达点的两辆车。
返回：
    签名为 (name, lookat, distance, azimuth, elevation) 的回调。
重要性等级：C"""
    def hook(
        name: str,
        lookat: list[float],
        distance: float,
        azimuth: float,
        elevation: float,
    ) -> None:
        capture_snapshot(name, cars, lookat, distance, azimuth, elevation)

    return hook


# ---------------------------------------------------------------------------
# 场景初始化与主流程
# ---------------------------------------------------------------------------
def initialize_mission(seed: int | None) -> tuple[
    base.ArmMotion,
    base.FingerMotion,
    CarController,
    CarController,
    np.ndarray,
]:
    """重置场景并随机生成放射物、初始化两车和控制器。

参数：
    seed: 随机种子；None 时每次运行随机，固定值用于可复现实验。
返回：
    (arm, finger, car1, car2, target_truth)。
异常：
    200 次采样仍无法找到远离障碍物的目标时抛 RuntimeError。
副作用：
    覆盖全局 DATA 状态并设置两车初始位姿。
重要性等级：A"""
    arm, finger = base.setup_initial_state()

    car1 = CarController(
        "CAR1",
        "mobile_chassis",
        [
            "wheel_fl_joint",
            "wheel_fr_joint",
            "wheel_rl_joint",
            "wheel_rr_joint",
        ],
        ["wheel_fl_steer_joint", "wheel_fr_steer_joint"],
        "lidar_site",
        np.array([0.1, 0.9, 1.0, 1.0]),
        "car2_chassis",
        "chassis_free",
    )
    car2 = CarController(
        "CAR2",
        "car2_chassis",
        [
            "car2_wheel_fl_joint",
            "car2_wheel_fr_joint",
            "car2_wheel_rl_joint",
            "car2_wheel_rr_joint",
        ],
        ["car2_wheel_fl_steer_joint", "car2_wheel_fr_steer_joint"],
        "car2_lidar_site",
        np.array([0.2, 1.0, 0.45, 1.0]),
        "mobile_chassis",
        "car2_chassis_free",
    )

    base.DATA.qpos[car1.qadr : car1.qadr + 3] = [
        mission_core.CAR1_START[0],
        mission_core.CAR1_START[1],
        0.0,
    ]
    base.DATA.qpos[car1.qadr + 3 : car1.qadr + 7] = yaw_quaternion(
        mission_core.CAR_START_YAW
    )

    car2_qpos = car2.qadr
    base.DATA.qpos[car2_qpos : car2_qpos + 3] = [
        mission_core.CAR2_START[0],
        mission_core.CAR2_START[1],
        0.0,
    ]
    base.DATA.qpos[car2_qpos + 3 : car2_qpos + 7] = yaw_quaternion(
        mission_core.CAR_START_YAW
    )

    rng = random.Random(seed)
    obstacle_xy = []
    for obstacle_id in range(6):
        body_id = _id(
            mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{obstacle_id}"
        )
        obstacle_xy.append(base.MODEL.body_pos[body_id, :2].copy())

    target = np.zeros(3, dtype=float)
    for _ in range(200):
        candidate = np.array(
            [
                rng.uniform(*OBJECT_X_RANGE),
                rng.uniform(*OBJECT_Y_RANGE),
                OBJECT_Z,
            ],
            dtype=float,
        )
        if all(
            np.linalg.norm(candidate[:2] - xy) >= 0.62
            for xy in obstacle_xy
        ):
            target = candidate
            break
    else:
        raise RuntimeError("could not sample a collision-free target")
    base.DATA.qpos[base.OBJ_QADR : base.OBJ_QADR + 3] = target
    base.DATA.qpos[base.OBJ_QADR + 3 : base.OBJ_QADR + 7] = [1.0, 0.0, 0.0, 0.0]
    base.DATA.qvel[:] = 0.0
    base.DATA.ctrl[:] = 0.0
    mujoco.mj_forward(base.MODEL, base.DATA)

    arm.q_hold = base.CARRY_Q.copy()
    arm.hold()
    finger.target = base.FINGER_OPEN
    finger.hold()
    return arm, finger, car1, car2, target


def run_mission(
    mission: DualCarMission, seed: int | None, started: float
) -> bool:
    """执行任务并把结构化指标写成单行 RESULT_JSON。

参数：
    mission: 已构造的 DualCarMission。
    seed: 本次随机种子，用于回归矩阵对齐。
    started: 入口处记录的 perf_counter，用于统计真实耗时。
返回：
    任务成功布尔值；任何阶段抛异常时返回 False，并把异常记入 error 字段。
副作用：
    推进长时间仿真、打印阶段日志，最后打印一行 RESULT_JSON。
重要性等级：A"""
    ok = False
    try:
        ok = bool(mission.run())
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        print(f"\n[MISSION FAILED] {type(exc).__name__}: {exc}")
        mission.metrics["error"] = f"{type(exc).__name__}: {exc}"

    metrics = dict(mission.metrics)
    metrics.update(
        seed=seed,
        wall_time_s=round(time.perf_counter() - started, 3),
        sim_time_s=round(float(mission.io.clock.now), 3),
        ok=ok,
    )
    metrics.setdefault("error", None)
    print("RESULT_JSON: " + json.dumps(metrics, ensure_ascii=False, default=float))
    return bool(metrics["ok"])


def main() -> None:
    """解析命令行参数并启动双车实时或离屏仿真。

参数：
    --headless: 无 viewer 执行。
    --snapshot: 离屏运行并保存关键截图。
    --seed: 可选随机种子。
副作用：
    初始化全局模型与任务，创建 viewer，运行完整任务；成功退出码 0，失败退出码 1。
异常：
    viewer 关闭产生 KeyboardInterrupt，由脚本入口捕获。
重要性等级：A"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--backend",
        default="sim",
        choices=("sim",),
        help="硬件后端；目前只有 sim（真机后端待接入）",
    )
    args = parser.parse_args()

    started = time.perf_counter()
    print(f"scene: {base.MOBILE_SCENE}")
    print(f"model: nq={base.MODEL.nq} nv={base.MODEL.nv} nu={base.MODEL.nu}")
    print(f"backend: {args.backend}")
    arm, finger, car1, car2, target = initialize_mission(args.seed)
    cars = [car1, car2]
    print(f"random radioactive target: {np.round(target, 4)}")

    if args.headless or args.snapshot:
        sim = base.Simulation(realtime=False)
        io = build_robot_io(
            args.backend, sim=sim, arm=arm, finger=finger, car1=car1, car2=car2
        )
        if args.snapshot:
            capture_snapshot(
                "dual_overview.png",
                cars,
                [2.0, 11.0, 0.20],
                38.0,
                135.0,
                -52.0,
            )
        mission = DualCarMission(
            io,
            target,
            snapshot=make_snapshot_hook(cars) if args.snapshot else None,
        )
        ok = run_mission(mission, args.seed, started)
        raise SystemExit(0 if ok else 1)

    import mujoco.viewer

    with mujoco.viewer.launch_passive(base.MODEL, base.DATA) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [2.0, 11.0, 0.30]
        viewer.cam.distance = 38.0
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -52.0

        sim = base.Simulation(viewer, realtime=True)
        io = build_robot_io(
            args.backend, sim=sim, arm=arm, finger=finger, car1=car1, car2=car2
        )
        mission = DualCarMission(io, target)
        ok = run_mission(mission, args.seed, started)
        print("Mission complete. Close viewer to exit.")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)
        raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.")
