"""雷达摘要节点：VLP-16 点云 → ObstacleSummary（8 个避障派生量）。

文件目的：
    任务层的避障判据（`mission_core` 里 2.8 m 前向警戒、左右净空、倒车 rear_clear）
    只认这 8 个数。仿真里它们由 MuJoCo 雷达扫描器直接算出来；真车只有点云，没有
    "射线"，因此需要本节点把点云重新压成同样定义的 8 个数。几何实现放在
    `lidar_geometry.summarize`（纯 numpy，可脱离 ROS 单测），本文件只负责 ROS 管线。

两种模式：
    ``mode:=publish``  发布 ``/{car}/obstacle_summary``，这是真车上的用法；
    ``mode:=check``    不发布到任务话题，改发 ``/{car}/obstacle_summary_check``，
                       并逐帧与仿真桥的 ``/{car}/obstacle_summary`` 对照，退出时打印
                       8 项最大偏差——用来证明"同一批点、同一套定义"两边算得一样。

为什么扇区界限直接 import mission_core：
    这 5 个数（警戒距离、前向/后方半角、侧向外边界）是任务层的定义，不是感知参数。
    在这里另存一份常量迟早会与任务层走散；因此本节点要求能 import 仓库根的
    ``mission_core``，与 `mm_mission` 用同一套注入规则（``colcon build --symlink-install``
    时由 realpath 反推，脱离源码树时用环境变量 ``MM_REPO_ROOT``）。

真车要注意的两处：
    ``mount_height`` 必须填实测的雷达安装高度（把传感器系 z 换算成离地高度），
    ``self_radius`` 要盖住车体与机械臂的回波，否则自己会被当成前方障碍。
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from mm_msgs.msg import ObstacleSummary

HERE = Path(__file__).resolve()

FLOAT_FIELDS = (
    "front_min",
    "front_lateral",
    "front_lateral_signed",
    "left_min",
    "right_min",
    "rear_min",
    "rear_clear",
)


def _inject_repo_root() -> None:
    """把仓库根加入 sys.path，使 `import mission_core` 可用（与 mm_mission 同规则）。"""
    candidates = []
    env_root = os.environ.get("MM_REPO_ROOT")
    if env_root:
        candidates.append(Path(env_root))
    candidates.append(HERE.parents[4])
    for root in candidates:
        if (root / "mission_core.py").is_file():
            text = str(root)
            if text not in sys.path:
                sys.path.insert(0, text)
            return
    raise ImportError(
        "找不到仓库根的 mission_core.py：请用 `colcon build --symlink-install` 构建，"
        "或设置环境变量 MM_REPO_ROOT=<仓库根目录>"
    )


_inject_repo_root()

from mission_core import (  # noqa: E402
    LIDAR_DETECT_RANGE,
    LIDAR_FRONT_HALF,
    LIDAR_REAR_HALF,
    LIDAR_SIDE_MAX,
)
from .lidar_geometry import summarize  # noqa: E402

MAX_RANGE = 100.0


class LidarSummary(Node):
    """把点云压成避障派生量；check 模式下与仿真桥逐帧对照。"""

    def __init__(self) -> None:
        super().__init__("mm_lidar_summary")
        self.declare_parameter("car", "car1")
        self.declare_parameter("mode", "publish")
        self.declare_parameter("cloud_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("reference_topic", "")
        self.declare_parameter("self_radius", 0.0)
        self.declare_parameter("mount_height", 0.0)
        self.declare_parameter("min_z", 0.08)
        self.declare_parameter("filter_ground", True)
        self.declare_parameter("max_range", MAX_RANGE)

        car = str(self.get_parameter("car").value)
        self._mode = str(self.get_parameter("mode").value)
        if self._mode not in ("publish", "check"):
            raise RuntimeError(f"mode 只能是 publish 或 check，收到 {self._mode!r}")
        cloud_topic = str(self.get_parameter("cloud_topic").value) or f"/{car}/lidar/points"
        output_topic = str(self.get_parameter("output_topic").value)
        if not output_topic:
            suffix = "" if self._mode == "publish" else "_check"
            output_topic = f"/{car}/obstacle_summary{suffix}"
        reference_topic = (
            str(self.get_parameter("reference_topic").value)
            or f"/{car}/obstacle_summary"
        )

        # 仿真桥发出的点云已经按离地 0.08 m 过滤过，对照时把地面门限关掉，
        # 免得因为安装高度填错而误丢点；真车上必须开，并填实测的 mount_height。
        filter_ground = bool(self.get_parameter("filter_ground").value)
        self._limits = dict(
            detect_range=LIDAR_DETECT_RANGE,
            front_half=LIDAR_FRONT_HALF,
            rear_half=LIDAR_REAR_HALF,
            side_max=LIDAR_SIDE_MAX,
            max_range=float(self.get_parameter("max_range").value),
            min_z=(
                float(self.get_parameter("min_z").value) if filter_ground else None
            ),
            mount_height=float(self.get_parameter("mount_height").value),
            self_radius=float(self.get_parameter("self_radius").value),
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(ObstacleSummary, output_topic, qos)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos)

        # 对照用：按时间戳把"本节点算的"和"桥算的"配成对，避免差一帧造成假偏差。
        self._ours: dict[tuple[int, int], ObstacleSummary] = {}
        self._refs: dict[tuple[int, int], ObstacleSummary] = {}
        self._frames = 0
        self._max_diff = {key: 0.0 for key in FLOAT_FIELDS}
        self._blocked_mismatch = 0
        self._missing_mismatch = 0
        self._empty_clouds = 0
        if self._mode == "check":
            self.create_subscription(
                ObstacleSummary, reference_topic, self._on_reference, qos
            )

        self.get_logger().info(
            f"点云 {cloud_topic} → {output_topic}；mode={self._mode}；"
            f"扇区来自 mission_core（警戒 {LIDAR_DETECT_RANGE} m/"
            f"前向 ±{math.degrees(LIDAR_FRONT_HALF):.0f}°）"
        )

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_key(msg) -> tuple[int, int]:
        """用 (sec, nanosec) 作配对键：桥对同一帧的点云与摘要用的是同一个时间戳。"""
        stamp = msg.header.stamp
        return (int(stamp.sec), int(stamp.nanosec))

    def _on_cloud(self, msg: PointCloud2) -> None:
        """收到一帧点云：算 8 个数，按模式发布或留待对照。"""
        # read_points 在 Humble 返回结构化数组，read_points_numpy 直接给 (N,3) 浮点数组。
        points = np.asarray(
            point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ),
            dtype=np.float64,
        ).reshape(-1, 3)
        if points.shape[0] == 0:
            self._empty_clouds += 1
        summary = summarize(points, **self._limits)

        output = ObstacleSummary()
        output.header = msg.header
        for key, value in summary.items():
            setattr(output, key, value)
        self._pub.publish(output)
        if self._mode == "publish":
            return

        key = self._stamp_key(msg)
        self._ours[key] = output
        self._pair(key)
        self._prune(msg.header.stamp.sec)

    def _on_reference(self, msg: ObstacleSummary) -> None:
        """收到桥的摘要（check 模式）：等着与同时间戳的点云配对。"""
        key = self._stamp_key(msg)
        self._refs[key] = msg
        self._pair(key)

    def _pair(self, key: tuple[int, int]) -> None:
        """两边都到齐就比一次，然后丢弃；缺一边就留在字典里等下一帧。"""
        ours = self._ours.get(key)
        ref = self._refs.get(key)
        if ours is None or ref is None:
            return
        del self._ours[key]
        del self._refs[key]
        self._frames += 1
        if bool(ours.front_blocked) != bool(ref.front_blocked):
            self._blocked_mismatch += 1
        for field in FLOAT_FIELDS:
            a = float(getattr(ours, field))
            b = float(getattr(ref, field))
            if math.isinf(a) and math.isinf(b):
                continue
            if math.isinf(a) or math.isinf(b):
                self._missing_mismatch += 1
                continue
            delta = abs(a - b)
            if delta > self._max_diff[field]:
                self._max_diff[field] = delta
        if self._frames % 200 == 0:
            self.report(f"已对照 {self._frames} 帧")

    def _prune(self, now_sec: int) -> None:
        """丢掉 60 s 还没配上的残留，防止节点长时间运行后字典膨胀。"""
        for store in (self._ours, self._refs):
            for key in [k for k in store if now_sec - k[0] > 60]:
                del store[key]

    # ------------------------------------------------------------------
    # 汇报
    # ------------------------------------------------------------------
    def report(self, title: str) -> None:
        """打印一次对照结论；check 模式的验收就看这一行。"""
        if self._mode != "check":
            return
        worst = max(self._max_diff.values()) if self._max_diff else 0.0
        detail = " ".join(f"{key}={self._max_diff[key]:.2e}" for key in FLOAT_FIELDS)
        self.get_logger().info(
            f"{title}：最大偏差 {worst:.3e} m，"
            f"front_blocked 不一致 {self._blocked_mismatch} 帧，"
            f"inf 不一致 {self._missing_mismatch} 次，空点云 {self._empty_clouds} 帧 | {detail}"
        )


def main() -> None:
    """入口：check 模式下退出时打印最终对照结论。"""
    rclpy.init()
    node = LidarSummary()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report("对照结束")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
