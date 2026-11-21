"""目标定位节点：深度帧 + TF → **粗端中心**的世界坐标。

订阅
----
- ``/car1/camera/color/image_raw``（rgb8，只用来触发与带时间戳；本代视觉不用颜色）
- ``/car1/camera/aligned_depth_to_color/image_raw``（32FC1，单位 m）
- ``/car1/camera/color/camera_info``（latched，提供 fx/fy/cx/cy）

发布
----
- ``/perception/target``：``mm_msgs/TargetEstimate``，``position`` 是**粗端中心**
  （粗端 Ø6×33 mm 的轴线中点，也就是单进程版的抓取点）；未检出时 ``valid=false``，
  失败原因打在日志里。

为什么不发整根杆的中心
----------------------
杆的两端直径不同、质心也不在中点；抓取点、真值点、落桶对准用的都是"粗端中心"，
所以感知只输出这一个点。整根杆的朝向若以后要用（比如给放置阶段做姿态规划），
再加字段，现在不加。

时序
----
图像是**触发式**的：任务层调用 ``/car1/camera/capture`` 后仿真才渲染一帧，
因此本节点按"彩色帧 + 深度帧都到齐"处理一次，不做固定频率推流。
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from mm_msgs.msg import TargetEstimate

from .vision import DEFAULT_ROI, detect_steel_target

MAP_FRAME = "map"
CAMERA_FRAME = "car1/camera_link"
DEPTH_ENCODING = "32FC1"


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 → 旋转矩阵。"""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


class TargetLocalizer(Node):
    """单目标定位节点：输出粗端中心。"""

    def __init__(self) -> None:
        super().__init__("mm_target_localizer")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 搜索窗：目标区内侧，用来把贴地的绿色边框与区域外结构挡在外面。
        # 默认值来自当前场景（绿框 X±0.35 / Y 20.70~21.30），换场景时用参数覆盖。
        self.declare_parameter("roi_x_min", DEFAULT_ROI[0])
        self.declare_parameter("roi_x_max", DEFAULT_ROI[1])
        self.declare_parameter("roi_y_min", DEFAULT_ROI[2])
        self.declare_parameter("roi_y_max", DEFAULT_ROI[3])
        self._roi = (
            float(self.get_parameter("roi_x_min").value),
            float(self.get_parameter("roi_x_max").value),
            float(self.get_parameter("roi_y_min").value),
            float(self.get_parameter("roi_y_max").value),
        )

        self._intrinsics: tuple[float, float, float, float] | None = None
        self._depth: np.ndarray | None = None
        self._stamp = None
        self._new_color = False
        self._new_depth = False
        self._bad_encoding = False
        self._count = 0

        self._pub = self.create_publisher(TargetEstimate, "/perception/target", qos)
        self.create_subscription(
            CameraInfo, "/car1/camera/color/camera_info", self._on_info, latched
        )
        self.create_subscription(
            Image, "/car1/camera/color/image_raw", self._on_color, qos
        )
        self.create_subscription(
            Image,
            "/car1/camera/aligned_depth_to_color/image_raw",
            self._on_depth,
            qos,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info(
            "target_localizer 就绪（深度离地分割，输出粗端中心），"
            f"搜索窗 X {self._roi[0]:.2f}~{self._roi[1]:.2f} / "
            f"Y {self._roi[2]:.2f}~{self._roi[3]:.2f}"
        )

    # ------------------------------------------------------------------
    def _on_info(self, msg: CameraInfo) -> None:
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_color(self, msg: Image) -> None:
        # 本代视觉不用颜色：彩色帧只负责触发一次处理并给出时间戳。
        self._stamp = msg.header.stamp
        self._new_color = True
        self._maybe_process()

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != DEPTH_ENCODING:
            if not self._bad_encoding:
                self._bad_encoding = True
                self.get_logger().error(
                    f"深度编码是 {msg.encoding}，本节点按 {DEPTH_ENCODING}（米）解析；"
                    "16UC1（毫米）需要先换算"
                )
            return
        self._depth = (
            np.frombuffer(msg.data, dtype=np.float32)
            .reshape(msg.height, msg.width)
            .copy()
        )
        self._new_depth = True
        self._maybe_process()

    # ------------------------------------------------------------------
    def _maybe_process(self) -> None:
        if not (self._new_color and self._new_depth and self._intrinsics):
            return
        self._new_color = False
        self._new_depth = False
        self._process()

    def _process(self) -> None:
        estimate = TargetEstimate()
        estimate.header.stamp = self._stamp
        estimate.header.frame_id = MAP_FRAME
        estimate.valid = False

        frame = self._lookup_camera()
        if frame is None:
            self.get_logger().warn(f"TF {MAP_FRAME}→{CAMERA_FRAME} 不可用")
            self._pub.publish(estimate)
            return

        detection = detect_steel_target(
            self._depth,
            self._intrinsics,
            frame[0],
            frame[1],
            roi=self._roi,
            log=self.get_logger().warn,
        )
        if detection is None:
            self._pub.publish(estimate)
            return

        position = detection.position
        estimate.position = Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        )
        estimate.confidence = 1.0
        estimate.valid = True
        self._pub.publish(estimate)

        self._count += 1
        self.get_logger().info(
            f"粗端中心 #{self._count}: ({position[0]:.4f}, {position[1]:.4f}, "
            f"{position[2]:.4f})，{detection.pixels} 像素 / "
            f"杆长 {detection.length_m * 100:.1f} cm / "
            f"两端高度 {detection.z_lo_mm:.2f}/{detection.z_hi_mm:.2f} mm / "
            f"判据 {detection.thick_source}"
        )

    def _lookup_camera(self):
        """查 map→camera_link，先按图像时刻，取不到就退回最新一帧。"""
        for stamp in (Time.from_msg(self._stamp), Time()):
            try:
                tf = self._tf_buffer.lookup_transform(MAP_FRAME, CAMERA_FRAME, stamp)
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            return (
                np.array([t.x, t.y, t.z], dtype=float),
                quat_to_matrix(q.x, q.y, q.z, q.w),
            )
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 发 SIGINT 时 rclpy 先关 context 再抛 ExternalShutdownException；
        # 不接住会以退出码 1 结束，日志里留一条假故障。
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
