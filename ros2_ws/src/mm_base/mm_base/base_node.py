"""mm_base.base_node：真机底盘 ROS2 节点（rclpy）。

职责
----
1. 启动握手：清报警 → 使能 ×4 → 转速模式 ×4 → 零速；任一步应答超时就报错退出，
   绝不带着未知驱动器状态开始发速度；
2. 订阅 ``/cmd_vel``，按 0.61 m 轮距拆成四轮目标转速，限幅后按 125 Hz 发 0x28 速度帧；
3. 收 0x601..0x604 反馈，按 1.35 m 里程计轮距积分位姿，发布 ``/odom`` 与
   ``/joint_states``；
4. 故障码非零、/cmd_vel 超时、CAN 发送失败时停止运动并报错。

话题
----
- 订阅：``/cmd_vel``（geometry_msgs/Twist）
- 发布：``/odom``（nav_msgs/Odometry，frame ``odom``，child ``base_link``）
- 发布：``/joint_states``（sensor_msgs/JointState，4 个轮关节）

参数
----
``can_interface``、``wheel_track``、``odom_wheel_track``、``max_speed``、
``max_yaw_rate``、``publish_rate``、``backend``、``invert_side``、``cmd_timeout``、
``startup_timeout``，含义见 docs/can_driver.md。

已知限制
--------
- 不做加减速规划：/cmd_vel 的阶跃会直接变成轮速阶跃，真机首跑必须小步给；
- 只用驱动器内部速度环，没有厂家的转矩模式 + 125 Hz 上位 PI；
- 只有里程计，没有发布 odom→base_link 的 TF（导航层需要时自行加）。

重要性等级：A。
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from mm_base.backend import CanBackend, CanError, LoopbackBackend, SocketCanBackend
from mm_base.kinematics import (
    DifferentialOdometry,
    WHEEL_SIDE_SIGN,
    distance_to_wheel_angle,
    rpm_to_wheel_radps,
    twist_to_wheel_speeds,
    wheel_speeds_to_twist,
    yaw_to_quaternion,
)
from mm_base.protocol import (
    MODE_ACK_ID,
    ODOM_WHEEL_TRACK_M,
    WHEEL_TRACK_CMD_M,
    StartupError,
    disable_frame,
    driver_cmd_id,
    mps_to_rpm,
    parse_feedback,
    rpm_to_mps,
    run_startup_sequence,
    speed_value_frame,
)

#: 关节名，顺序与驱动器 1..4（左前/右前/右后/左后）一致。
JOINT_NAMES = {
    1: "wheel_fl_joint",
    2: "wheel_fr_joint",
    3: "wheel_rr_joint",
    4: "wheel_rl_joint",
}


def _qos_latest(depth: int = 1) -> QoSProfile:
    """控制/状态类话题的统一 QoS：只保留最新值。

    参数：
        depth: 队列深度。
    返回：
        RELIABLE / KEEP_LAST / VOLATILE 的 QoSProfile。
    重要性等级：B（控制话题丢旧值比排队安全）。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class BaseCanNode(Node):
    """四轮底盘 CAN 节点。

    重要性等级：A（真机上直接驱动轮毂电机）。
    """

    def __init__(self) -> None:
        """建节点、校验参数、执行启动握手、创建话题与 125 Hz 定时器。

        异常：
            ValueError: 参数非法。
            CanError: CAN 打开/发送/接收失败。
            StartupError: 使能或模式应答超时。
        副作用：
            打开 CAN 后端；启动握手期间阻塞最长约 2×startup_timeout；
            失败时关闭后端并把异常抛给 main()。
        重要性等级：A。
        """
        super().__init__("mm_base_node")

        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("wheel_track", WHEEL_TRACK_CMD_M)
        self.declare_parameter("odom_wheel_track", ODOM_WHEEL_TRACK_M)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("publish_rate", 125.0)
        self.declare_parameter("backend", "socketcan")
        self.declare_parameter("invert_side", False)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("startup_timeout", 5.0)

        self._can_interface = str(self.get_parameter("can_interface").value)
        self._wheel_track = self._float_param("wheel_track")
        self._odom_wheel_track = self._float_param("odom_wheel_track")
        self._max_speed = self._float_param("max_speed")
        self._max_yaw_rate = self._float_param("max_yaw_rate")
        self._publish_rate = self._float_param("publish_rate")
        self._backend_name = str(self.get_parameter("backend").value).strip().lower()
        self._invert_side = bool(self.get_parameter("invert_side").value)
        self._cmd_timeout = self._float_param("cmd_timeout")
        self._startup_timeout = self._float_param("startup_timeout")
        self._validate_params()

        self.get_logger().info(
            f"底盘 CAN 节点：backend={self._backend_name}, interface={self._can_interface}, "
            f"wheel_track={self._wheel_track:.3f} m, odom_track={self._odom_wheel_track:.3f} m, "
            f"max_speed={self._max_speed:.2f} m/s, publish_rate={self._publish_rate:.0f} Hz"
        )

        self._backend: CanBackend | None = self._create_backend()
        try:
            self.get_logger().info("开始启动握手：清报警 → 使能 → 转速模式")
            run_startup_sequence(
                self._backend,
                timeout_s=self._startup_timeout,
                log=lambda message: self.get_logger().error(message),
            )
        except Exception:
            # 握手失败时主动关掉后端；异常继续向上抛，由 main() 打印并退出非零。
            self._backend.close()
            self._backend = None
            raise
        self.get_logger().info("启动握手完成，进入运行")

        # —— 最新反馈缓存（单线程执行器：订阅/定时器串行执行）——
        self._latest_speed_rpm = {n: 0 for n in range(1, 5)}
        self._odom = DifferentialOdometry(self._odom_wheel_track)
        self._odom_ready = False
        self._fault_latched = False

        # —— /cmd_vel 缓存与看门狗 ——
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._got_cmd = False
        self._last_cmd_time = time.monotonic()
        self._deadman_warned = False
        self._last_tick = time.monotonic()

        self._odom_pub = self.create_publisher(Odometry, "/odom", _qos_latest(10))
        self._joint_pub = self.create_publisher(JointState, "/joint_states", _qos_latest(10))
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, _qos_latest(10))
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    # -- 参数与后端 --------------------------------------------------------

    def _float_param(self, name: str) -> float:
        """读取浮点参数并转成 float。

        参数：
            name: 参数名。
        返回：
            float 值。
        异常：
            ValueError: 参数不是数值。
        重要性等级：B（错误类型不能静默变 0）。
        """
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"参数 {name} 必须是数值，收到 {value!r}") from exc

    def _validate_params(self) -> None:
        """校验参数，避免除零、NaN 或无限速这类不可恢复的配置错误。

        异常：
            ValueError: 参数越界或非有限值。
        重要性等级：A（参数错误必须在启动时暴露，不能带病运行）。
        """
        for name in (
            "publish_rate",
            "wheel_track",
            "odom_wheel_track",
            "max_speed",
            "max_yaw_rate",
            "cmd_timeout",
            "startup_timeout",
        ):
            value = getattr(self, f"_{name}")
            if not math.isfinite(value):
                raise ValueError(f"{name} 必须是有限数值，收到 {value!r}")
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate 必须大于 0")
        if self._publish_rate > 1000.0:
            raise ValueError("publish_rate 不能超过 1000 Hz")
        if self._wheel_track <= 0.0:
            raise ValueError("wheel_track 必须大于 0")
        if self._odom_wheel_track <= 0.0:
            raise ValueError("odom_wheel_track 必须大于 0")
        if self._max_speed <= 0.0:
            raise ValueError("max_speed 必须大于 0")
        if self._max_yaw_rate < 0.0:
            raise ValueError("max_yaw_rate 不能为负")
        if self._cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout 必须大于 0")
        if self._startup_timeout <= 0.0:
            raise ValueError("startup_timeout 必须大于 0")
        if self._backend_name not in ("socketcan", "loopback"):
            raise ValueError("backend 只能是 'socketcan' 或 'loopback'")

    def _create_backend(self) -> CanBackend:
        """按参数创建传输后端。

        返回：
            SocketCanBackend 或 LoopbackBackend。
        异常：
            BackendUnavailable: SocketCAN 不可用（Windows/接口不存在）。
        重要性等级：A。
        """
        if self._backend_name == "loopback":
            self.get_logger().warn("backend=loopback：不接 CAN 硬件，仅用于台架/无硬件演练")
            return LoopbackBackend()
        return SocketCanBackend(self._can_interface)

    # -- 回调 --------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        """缓存 /cmd_vel；非有限值直接丢弃并告警。

        参数：
            msg: Twist 消息，只使用 linear.x 与 angular.z。
        副作用：
            更新目标缓存、看门狗时间戳；非法值不改变缓存。
        重要性等级：A（速度指令入口）。
        """
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.get_logger().warn(
                f"丢弃非法 /cmd_vel：linear.x={linear!r}, angular.z={angular!r}"
            )
            return
        self._cmd_linear = linear
        self._cmd_angular = angular
        self._last_cmd_time = time.monotonic()
        self._got_cmd = True
        self._deadman_warned = False

    def _on_timer(self) -> None:
        """125 Hz 主循环：推进回环时钟、收反馈、发四轮转速、发状态。

        异常：
            CanError: CAN 发送失败时不吞，向上抛给 main()。
        副作用：
            向四个驱动器写速度帧；更新里程计/关节状态并发布。
        重要性等级：A。
        """
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now
        # 回环后端用 dt 积分；真机后端 no-op。长时间卡顿后最多补 0.1 s，避免假车“瞬移”。
        if self._backend is not None:
            self._backend.advance(min(dt, 0.1))
        self._drain_feedback()
        targets = self._wheel_targets(now)
        if self._backend is not None:
            for driver in range(1, 5):
                rpm = mps_to_rpm(targets[driver])
                self._backend.send(driver_cmd_id(driver), speed_value_frame(rpm))
        self._publish_state()

    def _drain_feedback(self) -> None:
        """非阻塞收完当前所有反馈帧，更新缓存与里程计。

        副作用：
            更新转速缓存、里程计位姿与故障锁存；畸形帧记 warning 后丢弃。
        重要性等级：A。
        """
        if self._backend is None:
            return
        while True:
            frame = self._backend.recv(0.0)
            if frame is None:
                return
            can_id, data = frame
            if can_id == MODE_ACK_ID:
                continue  # 启动后的应答不再参与控制
            try:
                feedback = parse_feedback(can_id, data)
            except ValueError as exc:
                self.get_logger().warn(f"忽略畸形反馈帧 id=0x{can_id:X}: {exc}")
                continue
            if feedback is None:
                continue

            if feedback.encoder_counts is not None:
                self._odom.update_counts({feedback.driver: feedback.encoder_counts})
                if not self._odom_ready and self._odom.integrated_sets > 0:
                    self._odom_ready = True
                    self.get_logger().info("里程计已对齐四轮编码器，开始发布 /odom")
            if feedback.speed_rpm is not None:
                self._latest_speed_rpm[feedback.driver] = feedback.speed_rpm
            if feedback.fault_code is not None and feedback.fault_code != 0:
                if not self._fault_latched:
                    self._fault_latched = True
                    self.get_logger().error(
                        f"驱动器 {feedback.driver} 故障码 0x{feedback.fault_code:04X}："
                        "目标速度已清零；请断电排查后重启节点（本节点不自动清故障）"
                    )

    def _wheel_targets(self, now: float) -> dict[int, float]:
        """计算四个驱动器的目标线速度（厂家指令符号）。

        返回：
            ``{驱动器号: m/s}``；故障、未收到过 /cmd_vel、看门狗超时时全 0。
        """
        zero = {n: 0.0 for n in range(1, 5)}
        if self._fault_latched:
            return zero
        if not self._got_cmd:
            return zero
        if now - self._last_cmd_time > self._cmd_timeout:
            if not self._deadman_warned:
                self._deadman_warned = True
                self.get_logger().warn(
                    f"/cmd_vel 超过 {self._cmd_timeout:.2f} s 未更新，目标速度清零"
                )
            return zero

        left, right = twist_to_wheel_speeds(
            self._cmd_linear,
            self._cmd_angular,
            wheel_track=self._wheel_track,
            max_speed=self._max_speed,
            max_yaw_rate=self._max_yaw_rate,
            invert_side=self._invert_side,
        )
        return {1: left, 2: right, 3: right, 4: left}

    def _publish_state(self) -> None:
        """发布 /odom 与 /joint_states。

        副作用：
            里程计首次对齐四轮后才发 /odom；/joint_states 每 tick 都发。
        重要性等级：A（导航与上层监视的输入）。
        """
        stamp = self.get_clock().now().to_msg()

        if self._odom_ready:
            x, y, yaw = self._odom.pose
            qz, qw = yaw_to_quaternion(yaw)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            # 反馈转速已按厂家取负；右轮再取负还原成“前进为正”的物理量。
            left_mps = (
                rpm_to_mps(self._latest_speed_rpm[1])
                + rpm_to_mps(self._latest_speed_rpm[4])
            ) / 2.0
            right_mps = -(
                rpm_to_mps(self._latest_speed_rpm[2])
                + rpm_to_mps(self._latest_speed_rpm[3])
            ) / 2.0
            linear, angular = wheel_speeds_to_twist(
                left_mps, right_mps, self._odom_wheel_track
            )
            odom.twist.twist.linear.x = linear
            odom.twist.twist.angular.z = angular
            self._odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = "base_link"
        for driver in range(1, 5):
            joints.name.append(JOINT_NAMES[driver])
            # wheel_distance 是车轮前进距离（右轮已按符号还原）。
            joints.position.append(
                distance_to_wheel_angle(self._odom.wheel_distance(driver))
            )
            # 反馈转速是电机侧，已取负；右轮再按 WHEEL_SIDE_SIGN 还原成前进为正。
            joints.velocity.append(
                WHEEL_SIDE_SIGN[driver]
                * rpm_to_wheel_radps(self._latest_speed_rpm[driver])
            )
        self._joint_pub.publish(joints)

    # -- 关节点 ------------------------------------------------------------

    def shutdown_driver(self) -> None:
        """发失能帧并关闭后端；可重复调用。

        即使发送失败也要关闭后端；失败会写 ERROR 日志，不吞。
        """
        backend, self._backend = self._backend, None
        if backend is None:
            return
        try:
            for driver in range(1, 5):
                backend.send(driver_cmd_id(driver), disable_frame())
            self.get_logger().info("已发送四轮失能")
        except (CanError, ValueError) as exc:
            self.get_logger().error(f"关机失能失败：{exc}")
        finally:
            backend.close()

    def destroy_node(self) -> None:
        """节点销毁前确保失能并关闭 CAN。"""
        self.shutdown_driver()
        super().destroy_node()


def main(args=None) -> int:
    """节点入口；启动/运行失败返回非零，供 launch/systemd 发现。

    参数：
        args: rclpy 参数列表；None 时用 sys.argv。
    返回：
        0 正常退出；1 启动或运行失败。
    重要性等级：A（失败必须让上层看见）。
    """
    rclpy.init(args=args)
    node: BaseCanNode | None = None
    try:
        node = BaseCanNode()
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 0
    except (StartupError, CanError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(f"底盘节点退出：{exc}")
        else:
            print(f"[mm_base] 启动失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.shutdown_driver()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
