"""mm_sim_bridge：独占 MuJoCo 的仿真桥接节点。

职责
----
1. 推进物理：每帧执行 ``STEPS_PER_FRAME`` 个 1 ms 子步，得到一个 20 ms 控制帧，并发布 ``/clock``；
2. 翻译传感器：把 MuJoCo 内部量发布成里程计、关节状态、雷达摘要、真值、接触、TF 与图像；
3. 接收指令：两车轮速、六轴关节目标、夹爪开合，在帧边界写回执行器。

并发模型
--------
MuJoCo 的读写**全部发生在本节点的帧循环线程里**；订阅与服务回调只写缓存或置标志，
帧循环在下一帧边界统一消费。因此不存在 ``DATA`` 的并发访问，也不需要锁住物理状态。

帧同步
------
每帧发布 ``/clock`` 之后等待两车各收到一条 ``wheel_command``（默认 50 ms 墙钟超时），
用超时而不是死等，保证上层卡住时仿真不会冻死；超时次数打印在日志里，是排查线索。

迁移说明
--------
真机阶段本节点被底盘/机械臂/夹爪/相机四个驱动节点取代，话题契约不变，
上层 ``mm_mission`` 不需要修改。
"""

from __future__ import annotations

import importlib
import os
import sys
import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from mm_msgs.msg import (
    GripperCommand,
    GripperState,
    ObstacleSummary,
    WheelCommand,
)
from mm_msgs.srv import Snapshot, SolveIK

CAR_NAMES = ("car1", "car2")
CAMERA_NAME = "d435i"
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640


def stamp_from_seconds(t: float) -> Time:
    """把仿真时刻（秒，浮点）转换成 ROS 时间戳。"""
    sec = int(np.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return Time(sec=sec, nanosec=nsec)


def qos_latest(depth: int = 1) -> QoSProfile:
    """控制/状态类话题的统一 QoS：只保留最新值，丢旧值比排队更安全。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class SimBridge(Node):
    """MuJoCo 仿真桥接节点。"""

    def __init__(self) -> None:
        super().__init__("mm_sim_bridge")

        self.declare_parameter("repo_root", "")
        self.declare_parameter("seed", -1)
        self.declare_parameter("command_timeout", 0.05)
        self.declare_parameter("max_frames", 0)

        # —— 回调与帧循环之间共享的缓存（受 _cmd_lock 保护）——
        self._cmd_lock = threading.Condition()
        self._wheel_cmd: list[tuple[float, float, float] | None] = [None, None]
        self._fresh = [False, False]
        self._arm_cmd: np.ndarray | None = None
        self._grip_cmd = 0.0
        self._capture_request = False
        self._snapshot_queue: list[str] = []
        self._peer_avoidance = True

        # —— 仅帧循环线程读写的状态 ——
        self._frame = 0
        self._timeouts = 0
        self._sim_time = 0.0
        self._arm_q = np.zeros(6)
        self._arm_base_pos = np.zeros(3)
        self._arm_base_rot = np.eye(3)
        self._shutdown = False

        self._load_scene()
        self._build_interfaces()

    # ------------------------------------------------------------------
    # 装配
    # ------------------------------------------------------------------
    def _load_scene(self) -> None:
        """把仓库根加入搜索路径并装配 MuJoCo 场景（沿用单进程实现的初始化流程）。"""
        root = str(self.get_parameter("repo_root").value or "")
        if not root:
            root = os.environ.get("MM_REPO_ROOT", "")
        if not root:
            raise RuntimeError(
                "参数 repo_root 未设置：请用 mm_bringup 的 launch 文件启动，"
                "或加 --ros-args -p repo_root:=<仓库根目录>"
            )
        root = os.path.abspath(root)
        if root not in sys.path:
            sys.path.insert(0, root)

        # 延迟导入：只有本节点需要 MuJoCo，导入即加载 30x30 m 场景（数秒）。
        self._mujoco = importlib.import_module("mujoco")
        self._legacy = importlib.import_module("dual_car_pick_place")
        self._base = self._legacy.base

        seed_param = int(self.get_parameter("seed").value)
        seed = None if seed_param < 0 else seed_param
        arm, finger, car1, car2, target = self._legacy.initialize_mission(seed)
        self._arm = arm
        self._finger = finger
        self._cars = (car1, car2)
        self._target = np.asarray(target, dtype=float)
        self._sim = self._base.Simulation(realtime=False)

        self._model = self._base.MODEL
        self._data = self._base.DATA
        self._cam_id = int(
            self._mujoco.mj_name2id(
                self._model, self._mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME
            )
        )
        self._chassis_joints = [
            self._joint_meta(list(car.wheel_act) + list(car.steer_act))
            for car in self._cars
        ]
        self.get_logger().info(
            f"scene loaded: nq={self._model.nq} nv={self._model.nv} "
            f"target=({self._target[0]:.3f}, {self._target[1]:.3f}) "
            f"seed={seed_param}"
        )

    def _joint_meta(self, actuator_ids: list[int]) -> list[tuple[str, int, int]]:
        """由执行器反查 (关节名, qpos 地址, dof 地址)。"""
        meta = []
        for act in actuator_ids:
            jid = int(self._model.actuator_trnid[act, 0])
            name = self._mujoco.mj_id2name(
                self._model, self._mujoco.mjtObj.mjOBJ_JOINT, jid
            )
            meta.append(
                (
                    str(name),
                    int(self._model.jnt_qposadr[jid]),
                    int(self._model.jnt_dofadr[jid]),
                )
            )
        return meta

    def _build_interfaces(self) -> None:
        """建立全部话题、服务与 TF 广播。"""
        qos = qos_latest()
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._clock_pub = self.create_publisher(Clock, "/clock", qos_latest(10))

        self._wheel_subs = [
            self.create_subscription(
                WheelCommand,
                f"/{name}/wheel_command",
                lambda msg, i=i: self._on_wheel_command(i, msg),
                qos,
            )
            for i, name in enumerate(CAR_NAMES)
        ]
        self._odom_pubs = [
            self.create_publisher(Odometry, f"/{name}/odom", qos)
            for name in CAR_NAMES
        ]
        self._chassis_joint_pubs = [
            self.create_publisher(JointState, f"/{name}/joint_states", qos)
            for name in CAR_NAMES
        ]
        self._obstacle_pubs = [
            self.create_publisher(ObstacleSummary, f"/{name}/obstacle_summary", qos)
            for name in CAR_NAMES
        ]

        self.create_subscription(
            Float64MultiArray,
            "/car1/arm/joint_command",
            self._on_arm_command,
            qos,
        )
        self.create_subscription(
            GripperCommand, "/car1/gripper/command", self._on_gripper_command, qos
        )
        self.create_subscription(
            Bool, "/car1/lidar/ignore_peer", self._on_peer_avoidance, qos
        )
        self._arm_joint_pub = self.create_publisher(
            JointState, "/car1/arm/joint_states", qos
        )
        self._gripper_pub = self.create_publisher(
            GripperState, "/car1/gripper/state", qos
        )

        self._image_pubs = {
            "color": self.create_publisher(
                Image, "/car1/camera/color/image_raw", qos
            ),
            "depth": self.create_publisher(
                Image, "/car1/camera/aligned_depth_to_color/image_raw", qos
            ),
        }
        self._camera_info_pub = self.create_publisher(
            CameraInfo, "/car1/camera/color/camera_info", latched
        )

        self._truth_pub = self.create_publisher(
            PoseStamped, "/sim/target_truth", qos
        )
        self._object_pub = self.create_publisher(
            PoseStamped, "/sim/object_pose", qos
        )
        self._contacts_pub = self.create_publisher(Int32, "/sim/contacts", qos)

        self._tf = TransformBroadcaster(self)

        self.create_service(SolveIK, "/car1/arm/solve_ik", self._on_solve_ik)
        self.create_service(Trigger, "/car1/camera/capture", self._on_capture)
        self.create_service(Snapshot, "/sim/snapshot", self._on_snapshot)

    # ------------------------------------------------------------------
    # 指令回调（只写缓存，不碰 MuJoCo）
    # ------------------------------------------------------------------
    def _on_wheel_command(self, index: int, msg: WheelCommand) -> None:
        with self._cmd_lock:
            self._wheel_cmd[index] = (float(msg.left), float(msg.right), float(msg.steer))
            self._fresh[index] = True
            self._cmd_lock.notify_all()

    def _on_arm_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 6:
            self.get_logger().warn(f"arm joint_command 长度 {len(msg.data)} != 6，已丢弃")
            return
        with self._cmd_lock:
            self._arm_cmd = np.asarray(msg.data, dtype=float).copy()

    def _on_gripper_command(self, msg: GripperCommand) -> None:
        with self._cmd_lock:
            self._grip_cmd = float(msg.position)

    def _on_peer_avoidance(self, msg) -> None:
        """任务层要求忽略/恢复"绕行同伴车"（最后一段直线入位用）。"""
        self._peer_avoidance = bool(msg.data)

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------
    def _on_solve_ik(self, request, response):
        """世界坐标 → 固定基座坐标系 → mink 逆解（与单进程实现的换算完全一致）。"""
        target_world = np.array(
            [request.target.x, request.target.y, request.target.z], dtype=float
        )
        local = (
            self._base.FIXED_BASE_ROT
            @ self._arm_base_rot.T
            @ (target_world - self._arm_base_pos)
        )
        q, err = self._base.solve_ik_fixed(
            local, self._base.TARGET_QUAT, seed_q=self._arm_q.copy()
        )
        response.joint_positions = [float(v) for v in q]
        response.position_error = float(err)
        return response

    def _on_capture(self, request, response):
        """触发一帧离屏渲染；真正的渲染发生在下一帧边界（避免与物理步进并发）。"""
        with self._cmd_lock:
            self._capture_request = True
        response.success = True
        response.message = "capture queued"
        return response

    def _on_snapshot(self, request, response):
        with self._cmd_lock:
            self._snapshot_queue.append(str(request.name))
        response.ok = True
        return response

    # ------------------------------------------------------------------
    # 帧循环
    # ------------------------------------------------------------------
    def run(self) -> None:
        """主帧循环，直到 rclpy 关闭或达到 max_frames。"""
        max_frames = int(self.get_parameter("max_frames").value)
        try:
            while rclpy.ok() and not self._shutdown:
                self._apply_commands()
                self._sim.step()
                self._sim_time = float(self._data.time)
                if self._peer_avoidance != self._cars[0].lidar.peer_avoidance:
                    self._cars[0].lidar.peer_avoidance = self._peer_avoidance
                for car in self._cars:
                    car.lidar.scan_if_due()
                self._refresh_arm_state()
                self._handle_capture()
                self._handle_snapshots()
                self._publish_state()
                self._publish_clock()
                self._frame += 1
                self._wait_next_commands()
                if max_frames and self._frame >= max_frames:
                    break
        finally:
            self.get_logger().info(
                f"帧循环结束：frames={self._frame} 指令超时={self._timeouts} "
                f"sim_time={self._sim_time:.2f}s"
            )

    def _apply_commands(self) -> None:
        """把上一帧收到的指令写进执行器。"""
        with self._cmd_lock:
            wheels = list(self._wheel_cmd)
            arm_cmd = None if self._arm_cmd is None else self._arm_cmd.copy()
            grip = self._grip_cmd
        for car, cmd in zip(self._cars, wheels):
            if cmd is None:
                car.set_wheels(0.0, 0.0, 0.0)
            else:
                car.set_wheels(*cmd)
        if arm_cmd is not None:
            self._data.ctrl[self._base.ARM_ACT] = self._base.mobile_gravity_ctrl(arm_cmd)
        self._data.ctrl[self._base.FINGER_ACT] = float(np.clip(grip, 0.0, 255.0))

    def _refresh_arm_state(self) -> None:
        """缓存机械臂基座位姿与关节角，供 IK 服务使用（服务线程只读这些副本）。"""
        self._arm_base_pos = self._data.xpos[self._base.ARM_BASE_BODY].copy()
        self._arm_base_rot = (
            self._data.xmat[self._base.ARM_BASE_BODY].reshape(3, 3).copy()
        )
        self._arm_q = self._data.qpos[self._base.ARM_QADR].copy()

    def _wait_next_commands(self) -> bool:
        """等待两车各一条新指令；超时返回 False 并按旧指令继续推进。"""
        timeout = float(self.get_parameter("command_timeout").value)
        with self._cmd_lock:
            ok = self._cmd_lock.wait_for(
                lambda: all(self._fresh), timeout=max(0.0, timeout)
            )
            self._fresh = [False, False]
        if not ok:
            self._timeouts += 1
            if self._timeouts <= 3 or self._timeouts % 200 == 0:
                self.get_logger().warn(
                    f"等待两车指令超时（第 {self._timeouts} 次），按旧指令推进"
                )
        return ok

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _publish_clock(self) -> None:
        msg = Clock()
        msg.clock = stamp_from_seconds(self._sim_time)
        self._clock_pub.publish(msg)

    def _publish_state(self) -> None:
        stamp = stamp_from_seconds(self._sim_time)
        self._publish_camera_info(stamp)
        for index, (name, car) in enumerate(zip(CAR_NAMES, self._cars)):
            self._publish_odom(index, name, car, stamp)
            self._publish_chassis_joints(index, name, stamp)
            self._publish_obstacle(index, name, car, stamp)
        self._publish_arm_joints(stamp)
        self._publish_gripper(stamp)
        self._publish_truth(stamp)
        self._publish_object(stamp)
        self._contacts_pub.publish(Int32(data=int(self._cars[0].contacts_with(self._cars[1]))))
        self._publish_tf(stamp)

    def _publish_odom(self, index: int, name: str, car, stamp: Time) -> None:
        pos, yaw = car.pose()
        v_world = self._data.qvel[car.dadr : car.dadr + 2]
        cy, sy = float(np.cos(yaw)), float(np.sin(yaw))
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = f"{name}/base_link"
        msg.pose.pose.position = Point(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
        )
        msg.pose.pose.orientation = Quaternion(
            z=float(np.sin(yaw / 2.0)), w=float(np.cos(yaw / 2.0))
        )
        msg.twist.twist.linear.x = float(cy * v_world[0] + sy * v_world[1])
        msg.twist.twist.linear.y = float(-sy * v_world[0] + cy * v_world[1])
        msg.twist.twist.angular.z = float(self._data.qvel[car.dadr + 5])
        self._odom_pubs[index].publish(msg)

    def _publish_chassis_joints(self, index: int, name: str, stamp: Time) -> None:
        meta = self._chassis_joints[index]
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [m[0] for m in meta]
        msg.position = [float(self._data.qpos[m[1]]) for m in meta]
        msg.velocity = [float(self._data.qvel[m[2]]) for m in meta]
        self._chassis_joint_pubs[index].publish(msg)

    def _publish_obstacle(self, index: int, name: str, car, stamp: Time) -> None:
        lidar = car.lidar
        msg = ObstacleSummary()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{name}/base_link"
        msg.front_blocked = bool(lidar.front_blocked)
        msg.front_min = float(lidar.front_min)
        msg.front_lateral = float(lidar.front_lateral)
        msg.front_lateral_signed = float(lidar.front_lateral_signed)
        msg.left_min = float(lidar.left_min)
        msg.right_min = float(lidar.right_min)
        msg.rear_min = float(lidar.rear_min)
        msg.rear_clear = float(lidar.rear_clear)
        self._obstacle_pubs[index].publish(msg)

    def _publish_arm_joints(self, stamp: Time) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self._base.ARM_JOINT_NAMES)
        msg.position = [float(v) for v in self._data.qpos[self._base.ARM_QADR]]
        msg.velocity = [float(v) for v in self._data.qvel[self._base.ARM_DADR]]
        self._arm_joint_pub.publish(msg)

    def _publish_gripper(self, stamp: Time) -> None:
        msg = GripperState()
        msg.header.stamp = stamp
        msg.position = float(
            self._data.ctrl[self._base.FINGER_ACT]
        )
        msg.effort = float(abs(self._data.actuator_force[self._base.FINGER_ACT]))
        msg.moving = bool(abs(msg.position - self._grip_cmd) > 1.0)
        self._gripper_pub.publish(msg)

    def _publish_truth(self, stamp: Time) -> None:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position = Point(
            x=float(self._target[0]), y=float(self._target[1]), z=float(self._target[2])
        )
        msg.pose.orientation.w = 1.0
        self._truth_pub.publish(msg)

    def _publish_object(self, stamp: Time) -> None:
        """被操作对象的**实时**位姿。

        与 ``/sim/target_truth`` 的区别：后者是生成时的落点（姿态恒为单位四元数），
        只用于视觉误差验收；本话题给的是夹在夹爪里、正在被搬运的那根杆，
        投放对中（夹持点→质心偏移）和投放判据都必须用它。
        """
        qpos = self._data.qpos
        adr = self._base.OBJ_QADR
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position = Point(
            x=float(qpos[adr]), y=float(qpos[adr + 1]), z=float(qpos[adr + 2])
        )
        msg.pose.orientation = Quaternion(
            w=float(qpos[adr + 3]),
            x=float(qpos[adr + 4]),
            y=float(qpos[adr + 5]),
            z=float(qpos[adr + 6]),
        )
        self._object_pub.publish(msg)

    def _publish_camera_info(self, stamp: Time) -> None:
        """按 MuJoCo 相机模型给出针孔内参（方像素，主点居中），只发一次。"""
        if getattr(self, "_camera_info_sent", False):
            return
        fovy = float(np.deg2rad(float(self._model.cam_fovy[self._cam_id])))
        f = (IMAGE_HEIGHT / 2.0) / float(np.tan(fovy / 2.0))
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "car1/camera_link"
        msg.height = IMAGE_HEIGHT
        msg.width = IMAGE_WIDTH
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [f, 0.0, IMAGE_WIDTH / 2.0, 0.0, f, IMAGE_HEIGHT / 2.0, 0.0, 0.0, 1.0]
        msg.p = [f, 0.0, IMAGE_WIDTH / 2.0, 0.0,
                 0.0, f, IMAGE_HEIGHT / 2.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self._camera_info_pub.publish(msg)
        self._camera_info_sent = True

    def _publish_tf(self, stamp: Time) -> None:
        """发布全部 TF 帧。

        这里所有位姿都直接取 MuJoCo 的世界量（``xpos``/``xmat``/``site_xpos``），
        所以父帧一律是 ``map``——见 ``_send_world_tf`` 的说明。
        """
        for name, car in zip(CAR_NAMES, self._cars):
            pos, yaw = car.pose()
            self._send_world_tf(
                stamp,
                f"{name}/base_link",
                (float(pos[0]), float(pos[1]), float(pos[2])),
                (0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))),
            )
        # 机械臂基座（任务层用它把"臂座系目标点"换算到世界系）
        arm_pos = self._data.xpos[self._base.ARM_BASE_BODY]
        arm_mat = self._data.xmat[self._base.ARM_BASE_BODY].reshape(3, 3)
        arm_quat = np.zeros(4)
        self._mujoco.mju_mat2Quat(arm_quat, arm_mat.reshape(9))
        self._send_world_tf(
            stamp,
            "car1/arm_base_link",
            (float(arm_pos[0]), float(arm_pos[1]), float(arm_pos[2])),
            (float(arm_quat[1]), float(arm_quat[2]), float(arm_quat[3]), float(arm_quat[0])),
        )

        cam_pos = self._data.cam_xpos[self._cam_id]
        cam_mat = self._data.cam_xmat[self._cam_id].reshape(3, 3)
        quat = np.zeros(4)
        self._mujoco.mju_mat2Quat(quat, cam_mat.reshape(9))
        self._send_world_tf(
            stamp,
            "car1/camera_link",
            (float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])),
            (float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])),
        )

        # 夹持点：投放对中要量"夹持点 → 质心"的实际偏移，任务层由此拿 tool_position()
        pinch_pos = self._data.site_xpos[self._legacy.PINCH_SITE]
        pinch_mat = self._data.site_xmat[self._legacy.PINCH_SITE].reshape(3, 3)
        pq = np.zeros(4)
        self._mujoco.mju_mat2Quat(pq, pinch_mat.reshape(9))
        self._send_world_tf(
            stamp,
            "car1/pinch",
            (float(pinch_pos[0]), float(pinch_pos[1]), float(pinch_pos[2])),
            (float(pq[1]), float(pq[2]), float(pq[3]), float(pq[0])),
        )

    def _send_world_tf(self, stamp, child, xyz, xyzw) -> None:
        """发布一个"父帧 = map"的 TF。

        故意不开放 parent 参数：本节点的所有位姿都是 MuJoCo 世界量，唯一正确的父帧
        就是 ``map``。曾经的写法允许传 parent，于是 ``car1/arm_base_link`` 用世界坐标
        却挂在 ``car1/base_link`` 下，TF 合成时把车体位置又加了一遍，世界目标整体偏
        ~20 m（任务卡在 4_grasp，IK 残差 18.7 m）。把参数删掉这种错误就写不出来。
        真机上这几帧由各自驱动发布真正的相对变换，届时再按需补 parent 参数。
        """
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = child
        msg.transform.translation.x = xyz[0]
        msg.transform.translation.y = xyz[1]
        msg.transform.translation.z = xyz[2]
        msg.transform.rotation.x = xyzw[0]
        msg.transform.rotation.y = xyzw[1]
        msg.transform.rotation.z = xyzw[2]
        msg.transform.rotation.w = xyzw[3]
        self._tf.sendTransform(msg)

    # ------------------------------------------------------------------
    # 图像与截图
    # ------------------------------------------------------------------
    def _handle_capture(self) -> None:
        with self._cmd_lock:
            requested = self._capture_request
            self._capture_request = False
        if not requested:
            return
        rgb, depth = self._legacy.get_camera_data()
        stamp = stamp_from_seconds(self._sim_time)
        self._publish_image("color", rgb, stamp, "rgb8", 3)
        self._publish_image("depth", depth.astype(np.float32), stamp, "32FC1", 4)

    def _publish_image(self, key, array, stamp, encoding, channels) -> None:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "car1/camera_link"
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(array.shape[1] * channels)
        msg.data = np.ascontiguousarray(array).tobytes()
        self._image_pubs[key].publish(msg)

    def _handle_snapshots(self) -> None:
        with self._cmd_lock:
            pending = list(self._snapshot_queue)
            self._snapshot_queue.clear()
        for name in pending:
            try:
                self._legacy.capture_snapshot(
                    name,
                    list(self._cars),
                    [2.0, 11.0, 0.20],
                    38.0,
                    135.0,
                    -52.0,
                )
            except Exception as exc:  # 截图失败不应影响仿真主循环
                self.get_logger().warn(f"截图失败 {name}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # 先停 executor 再销毁节点：spin 线程还在跑的时候 destroy_node() 会让 rclpy
        # 的 C++ 层直接 abort（SIGABRT: terminate called without an active exception）。
        # 顺序必须是 shutdown() → join(spin) → destroy_node()。
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
