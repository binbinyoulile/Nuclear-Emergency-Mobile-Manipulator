"""MissionNode：任务层节点。

结构
----
- 本节点持有 rclpy 侧的一切：订阅状态、发布指令、服务客户端、TF 缓冲；
- 任务逻辑跑在 ``main()`` 的主线程里，用 ``wait_next_frame()`` 阻塞等待 ``/clock``，
  因此"读状态 → 算指令 → 写指令"的写法与单进程版本完全一致；
- 回调在另一个执行器线程里跑，只负责把消息解析进缓存。

任务本体在仓库根的 ``mission_core.py``（与单进程入口共用的唯一一份实现），
``mode:=mission``（默认）跑七阶段状态机，``mode:=smoke`` 跑阶段 1 的端到端冒烟。
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from mm_msgs.msg import (
    GripperCommand,
    GripperState,
    MissionStatus,
    ObstacleSummary,
    TargetEstimate,
    WheelCommand,
)
from mm_msgs.srv import SolveIK

from .adapters import RobotIO, RosArm, RosChassis, RosClock, RosGripper, RosPerception

CAR_NAMES = ("car1", "car2")


def qos_latest(depth: int = 1) -> QoSProfile:
    """控制/状态类话题：只保留最新值。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


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


class MissionNode(Node):
    """任务决策节点。"""

    def __init__(self) -> None:
        super().__init__("mm_mission")
        self.declare_parameter("frame_dt", 0.02)
        # 墙钟超时按"慢机器"给：虚拟机降速时场景导入要 30 s 级，5 s 的帧超时会把正常运行
        # 判成失败（实测同机 2~6 倍降速时冒烟连挂 3 次，用时放宽即通过）。不会掩盖真故障：
        # 仿真真死了照样超时报错。
        self.declare_parameter("frame_timeout", 15.0)
        self.declare_parameter("ready_timeout", 60.0)
        self.declare_parameter("estimate_timeout", 5.0)
        self.declare_parameter("seed", -1)
        self.declare_parameter("mode", "mission")

        self._cond = threading.Condition()
        self._clock_seq = 0
        self._sim_time = 0.0
        self._frames = 0
        self._stage = "init"
        self._odom: list[tuple | None] = [None, None]
        self._chassis_joint: list[JointState | None] = [None, None]
        self._obstacle: list[ObstacleSummary | None] = [None, None]
        self._arm_joint: JointState | None = None
        self._gripper: GripperState | None = None
        self._estimate: np.ndarray | None = None
        self._estimate_seq = 0
        self._truth: np.ndarray | None = None
        self._object: tuple[np.ndarray, np.ndarray] | None = None
        self._contacts = 0
        # 两车最近一次轮速指令：桥的帧同步要求每帧各一条，控制器不刷新底盘时要补发。
        self._wheel_cache = [
            WheelCommand(left=0.0, right=0.0, steer=0.0) for _ in CAR_NAMES
        ]
        # 任务指标（mission_core 的 metrics 字典，逐阶段原地填充）
        self._core_metrics: dict = {}

        self.io = RobotIO(
            clock=RosClock(self),
            chassis=(RosChassis(self, 0), RosChassis(self, 1)),
            arm=RosArm(self, RosClock(self)),
            gripper=RosGripper(self, RosClock(self)),
            perception=RosPerception(self),
        )
        self._build_interfaces()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # ------------------------------------------------------------------
    # 话题
    # ------------------------------------------------------------------
    def _build_interfaces(self) -> None:
        qos = qos_latest()
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Clock, "/clock", self._on_clock, qos_latest(10))
        for index, name in enumerate(CAR_NAMES):
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, i=index: self._on_odom(i, msg),
                qos,
            )
            self.create_subscription(
                JointState,
                f"/{name}/joint_states",
                lambda msg, i=index: self._on_chassis_joint(i, msg),
                qos,
            )
            self.create_subscription(
                ObstacleSummary,
                f"/{name}/obstacle_summary",
                lambda msg, i=index: self._on_obstacle(i, msg),
                qos_latest(10),
            )
        self.create_subscription(
            JointState, "/car1/arm/joint_states", self._on_arm_joint, qos
        )
        self.create_subscription(
            GripperState, "/car1/gripper/state", self._on_gripper, qos
        )
        self.create_subscription(
            TargetEstimate, "/perception/target", self._on_estimate, qos_latest(10)
        )
        self.create_subscription(
            PoseStamped, "/sim/target_truth", self._on_truth, qos_latest(10)
        )
        self.create_subscription(
            PoseStamped, "/sim/object_pose", self._on_object_pose, qos_latest(10)
        )
        self.create_subscription(Int32, "/sim/contacts", self._on_contacts, qos)

        self._wheel_pubs = [
            self.create_publisher(WheelCommand, f"/{name}/wheel_command", qos)
            for name in CAR_NAMES
        ]
        self._arm_pub = self.create_publisher(
            Float64MultiArray, "/car1/arm/joint_command", qos
        )
        self._gripper_pub = self.create_publisher(
            GripperCommand, "/car1/gripper/command", qos
        )
        self._peer_avoid_pub = self.create_publisher(
            Bool, "/car1/lidar/ignore_peer", latched
        )
        self._status_pub = self.create_publisher(MissionStatus, "/mission/status", latched)
        self._result_pub = self.create_publisher(String, "/mission/result", latched)

        self._ik_client = self.create_client(SolveIK, "/car1/arm/solve_ik")
        self._capture_client = self.create_client(Trigger, "/car1/camera/capture")

    # ------------------------------------------------------------------
    # 回调（执行器线程）
    # ------------------------------------------------------------------
    def _on_clock(self, msg: Clock) -> None:
        with self._cond:
            self._clock_seq += 1
            self._sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
            self._cond.notify_all()

    def _on_odom(self, index: int, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        )
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        with self._cond:
            self._odom[index] = (
                np.array([p.x, p.y, p.z], dtype=float),
                yaw,
                speed,
                float(msg.twist.twist.angular.z),
            )
            self._cond.notify_all()

    def _on_chassis_joint(self, index: int, msg: JointState) -> None:
        with self._cond:
            self._chassis_joint[index] = msg

    def _on_obstacle(self, index: int, msg: ObstacleSummary) -> None:
        with self._cond:
            self._obstacle[index] = msg
        # 控制器直接读 car.lidar.front_min 这类字段，视图在这里同步
        self.io.chassis[index].lidar.update(msg)

    def _on_arm_joint(self, msg: JointState) -> None:
        with self._cond:
            self._arm_joint = msg
            self._cond.notify_all()

    def _on_gripper(self, msg: GripperState) -> None:
        with self._cond:
            self._gripper = msg

    def _on_estimate(self, msg: TargetEstimate) -> None:
        if not msg.valid:
            return
        p = msg.position
        with self._cond:
            self._estimate = np.array([p.x, p.y, p.z], dtype=float)
            self._estimate_seq += 1
            self._cond.notify_all()

    def _on_truth(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self._cond:
            self._truth = np.array([p.x, p.y, p.z], dtype=float)

    def _on_object_pose(self, msg: PoseStamped) -> None:
        """被操作对象的实时世界位姿（位置 + wxyz 四元数），供投放对中和验收用。"""
        p = msg.pose.position
        q = msg.pose.orientation
        with self._cond:
            self._object = (
                np.array([p.x, p.y, p.z], dtype=float),
                np.array([q.w, q.x, q.y, q.z], dtype=float),
            )

    def _on_contacts(self, msg: Int32) -> None:
        with self._cond:
            self._contacts = int(msg.data)

    # ------------------------------------------------------------------
    # 时钟同步
    # ------------------------------------------------------------------
    @property
    def frame_dt(self) -> float:
        return float(self.get_parameter("frame_dt").value)

    @property
    def sim_time(self) -> float:
        return float(self._sim_time)

    def wait_next_frame(self, timeout: float | None = None) -> float:
        """阻塞到仿真推进一帧（``/clock`` 递增一次）。"""
        limit = float(
            timeout if timeout is not None else self.get_parameter("frame_timeout").value
        )
        with self._cond:
            seq = self._clock_seq
            if not self._cond.wait_for(lambda: self._clock_seq > seq, timeout=limit):
                raise TimeoutError("/clock 超时：仿真节点可能已退出或未启动")
            sim_time = self._sim_time
        self._frames += 1
        if self._frames % 50 == 0:
            self._publish_status(False, False)
        return sim_time

    def wait_ready(self, timeout: float | None = None) -> None:
        """等到第一帧状态齐全（里程计、关节、夹爪、对象位姿）。"""
        limit = float(
            timeout if timeout is not None else self.get_parameter("ready_timeout").value
        )
        with self._cond:
            ready = self._cond.wait_for(
                lambda: self._clock_seq > 0
                and all(item is not None for item in self._odom)
                and self._arm_joint is not None
                and self._gripper is not None
                and self._object is not None,
                timeout=limit,
            )
        if not ready:
            raise TimeoutError(
                "状态话题未就绪：检查 mm_sim_bridge 是否已启动并发布 /clock、/sim/object_pose"
            )

    # ------------------------------------------------------------------
    # 只读访问（任务线程）
    # ------------------------------------------------------------------
    def odom(self, index: int) -> tuple[np.ndarray, float, float, float]:
        with self._cond:
            item = self._odom[index]
        if item is None:
            raise RuntimeError(f"{CAR_NAMES[index]} 里程计尚未就绪")
        return item

    def chassis_joint(self, index: int) -> JointState | None:
        with self._cond:
            return self._chassis_joint[index]

    def obstacle(self, index: int) -> ObstacleSummary | None:
        with self._cond:
            return self._obstacle[index]

    def arm_joint_positions(self) -> np.ndarray | None:
        with self._cond:
            msg = self._arm_joint
        return None if msg is None else np.asarray(msg.position, dtype=float)

    def gripper_state(self) -> GripperState | None:
        with self._cond:
            return self._gripper

    @property
    def contacts(self) -> int:
        with self._cond:
            return self._contacts

    def target_truth(self) -> np.ndarray | None:
        with self._cond:
            return None if self._truth is None else self._truth.copy()

    def object_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """被操作对象的实时位姿 (位置, 四元数 wxyz)。"""
        with self._cond:
            item = self._object
        if item is None:
            raise RuntimeError("尚未收到 /sim/object_pose")
        return item[0].copy(), item[1].copy()

    # ------------------------------------------------------------------
    # 写指令
    # ------------------------------------------------------------------
    def publish_wheel(self, index: int, left: float, right: float, steer: float) -> None:
        msg = WheelCommand(left=float(left), right=float(right), steer=float(steer))
        self._wheel_cache[index] = msg
        self._wheel_pubs[index].publish(msg)

    def flush_wheel_commands(self) -> None:
        """把两车最近一次轮速指令重发一遍。

        桥的帧同步要求每帧各收到一条 ``wheel_command``；机械臂动作、视觉等待等阶段
        任务层不刷新底盘指令（仿真里 ctrl 会保持，话题里不会），这里补齐"保持"语义。
        """
        for index, msg in enumerate(self._wheel_cache):
            self._wheel_pubs[index].publish(msg)

    def publish_arm(self, q) -> None:
        self._arm_pub.publish(Float64MultiArray(data=[float(v) for v in q]))

    def publish_gripper(self, position: float, max_effort: float = 30.0) -> None:
        self._gripper_pub.publish(
            GripperCommand(position=float(position), max_effort=float(max_effort))
        )

    def publish_peer_avoidance(self, enabled: bool) -> None:
        """告诉仿真桥要不要把同伴车算进 car1 的动态障碍。"""
        self._peer_avoid_pub.publish(Bool(data=bool(enabled)))

    def set_stage(self, stage: str) -> None:
        self._stage = stage
        self.get_logger().info(f"[阶段] {stage}")
        self._publish_status(False, False)

    def _publish_status(self, done: bool, ok: bool) -> None:
        self._status_pub.publish(
            MissionStatus(
                stage=self._stage, done=bool(done), ok=bool(ok), sim_time=self.sim_time
            )
        )

    # ------------------------------------------------------------------
    # 服务与 TF
    # ------------------------------------------------------------------
    def _call_service(self, client, request, timeout: float):
        """异步调用 + 轮询 future，超时或失败返回 None。

        rclpy Humble 的 ``Client.call()`` 没有 ``timeout_sec`` 参数（传了直接 TypeError），
        所以自己用 ``call_async`` 等 future；调用方把 ``None`` 当超时处理。

        等待期间继续推进仿真帧并补发两车轮速（与 ``request_estimate`` 同一做法）。否则
        任务线程阻塞时桥等不到指令、只能按旧指令推进：慢虚拟机上逆解一次 1~2 s，累计
        可达数秒（实测 234~601 帧，占全流程 8%），既让收尾日志的 ``指令超时`` 失去判别力，
        也破坏"仿真只由任务层驱动"这个不变量。
        """
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            try:
                self.wait_next_frame()
                self.flush_wheel_commands()
            except TimeoutError:
                # /clock 没来：不在这里放大错误，交给上面的 deadline 兜底
                time.sleep(0.005)
        try:
            return future.result()
        except Exception:  # noqa: BLE001 - 服务端异常/取消一律按失败处理
            return None

    def solve_ik_world(
        self, point: np.ndarray, orientation: np.ndarray
    ) -> tuple[np.ndarray, float]:
        """世界坐标 + 固定基座系工具姿态（wxyz）→ 关节角（驱动侧逆解）。"""
        request = SolveIK.Request()
        request.target = Point(
            x=float(point[0]), y=float(point[1]), z=float(point[2])
        )
        quat = np.asarray(orientation, dtype=float)
        request.orientation = Quaternion(
            w=float(quat[0]), x=float(quat[1]), y=float(quat[2]), z=float(quat[3])
        )
        response = self._call_service(self._ik_client, request, 5.0)
        if response is None:
            raise TimeoutError("/car1/arm/solve_ik 无响应")
        return np.asarray(response.joint_positions, dtype=float), float(
            response.position_error
        )

    def request_estimate(self) -> np.ndarray:
        """触发一帧采集，并等待感知节点给出新的有效估计。

        等待期间继续推进仿真帧（图像和 TF 都在帧边界产生），同时补发两车轮速指令；
        这样任务线程阻塞在感知上时，仿真时间仍然由任务层驱动，不会凭墙钟自由跑。
        """
        with self._cond:
            seq = self._estimate_seq
        response = self._call_service(
            self._capture_client, Trigger.Request(), 5.0
        )
        if response is None or not response.success:
            raise RuntimeError("相机触发失败：/car1/camera/capture 无响应")
        limit = float(self.get_parameter("estimate_timeout").value)
        deadline = time.monotonic() + limit
        while True:
            with self._cond:
                if self._estimate_seq > seq and self._estimate is not None:
                    return self._estimate.copy()
            if time.monotonic() > deadline:
                raise TimeoutError("等待 /perception/target 超时")
            self.wait_next_frame()
            self.flush_wheel_commands()

    def tf_position(self, frame: str) -> np.ndarray:
        """查询某个 TF 坐标系在 map 下的位置。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", frame, rclpy.time.Time()
            )
        except Exception as exc:
            raise RuntimeError(f"TF map→{frame} 查询失败: {exc}") from exc
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)

    def arm_base_to_world(self, point: np.ndarray) -> np.ndarray:
        """car1/arm_base_link 坐标 → 世界坐标。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "car1/arm_base_link", rclpy.time.Time()
            )
        except Exception as exc:
            raise RuntimeError(f"TF map→car1/arm_base_link 查询失败: {exc}") from exc
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        return np.array([t.x, t.y, t.z], dtype=float) + rot @ np.asarray(
            point, dtype=float
        )

    # ------------------------------------------------------------------
    # 任务
    # ------------------------------------------------------------------
    def _run_core_mission(self) -> bool:
        """用仓库根的 mission_core 跑七阶段状态机。"""
        import mission_core

        truth = self.target_truth()
        if truth is None:
            raise RuntimeError("未收到 /sim/target_truth，无法做视觉误差验收")
        mission = mission_core.DualCarMission(self.io, truth)
        # 与 mission 共享同一个 dict：即使中途抛异常，也能把已完成阶段的指标带出来
        self._core_metrics = mission.metrics
        return bool(mission.run())

    def run_mission(self) -> bool:
        """执行任务并输出单行 RESULT_JSON（与单进程版本的指标链路一致）。"""
        started = time.perf_counter()
        mode = str(self.get_parameter("mode").value)
        seed = int(self.get_parameter("seed").value)
        ok = False
        try:
            self.wait_ready()
            if mode == "smoke":
                from . import smoke

                self._core_metrics = smoke.run(self.io, self)
                ok = True
            else:
                ok = self._run_core_mission()
            self.set_stage("done" if ok else "failed")
        except Exception as exc:  # 任务失败要把原因写进指标，而不是直接崩掉
            message = f"{type(exc).__name__}: {exc}"
            self._core_metrics["error"] = message
            self.get_logger().error(message)
            self.set_stage("failed")
        finally:
            metrics = dict(self._core_metrics)
            metrics.update(
                seed=seed,
                backend="ros",
                mode=mode,
                sim_time_s=round(self.sim_time, 3),
                wall_time_s=round(time.perf_counter() - started, 3),
                ok=bool(ok),
            )
            metrics.setdefault("stage", self._stage)
            metrics.setdefault("error", None)
            line = "RESULT_JSON: " + json.dumps(metrics, ensure_ascii=False, default=float)
            print(line, flush=True)
            self._result_pub.publish(String(data=line))
            self._publish_status(True, ok)
        return ok


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ok = node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
