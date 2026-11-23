"""mm_base.selftest：不接 ROS2、不接 CAN 硬件的自检。

运行方式（Windows，一行）::

    D:\\Python313\\python.exe "D:\\桌面\\机械臂\\mobile_manipulator\\.dsh-worktrees\\can\\ros2_ws\\src\\mm_base\\mm_base\\selftest.py"

也可以从 ``ros2_ws/src/mm_base`` 目录跑 ``python -m mm_base.selftest``。

覆盖内容
--------
1. 七种命令帧与厂家源码逐字节一致；
2. 命令帧 CAN ID 0x101..0x104；
3. 0xCA/0xCB 正负计数、负转速、故障码、未知命令字、错误帧长；
4. 直线 1.0 m/s × 2.0 s → 里程计 ≈2.0 m、偏航 0；
5. 原地转 1.0 rad/s × 1.0 s → 按 1.35 m 里程计轮距偏航 ≈0.452 rad（并另测
   0.61 m 口径得 1.0 rad；两者冲突的说明见 docs/can_driver.md）；
6. 未使能时速度命令不产生运动（安全断言）；
7. 启动序列顺序：清报警 → 使能 → 转速模式 → 零速；
8. Windows 上 SocketCAN 只是不可用，不会 import 崩溃；
9. 协议层不 import rclpy/socket，base_node.py 至少能通过语法解析。

本文件**绝不 import rclpy**；所需数学全部来自 ``mm_base.protocol`` /
``mm_base.kinematics``。重要性等级：A。
"""

from __future__ import annotations

import ast
import math
import sys
import traceback
from pathlib import Path

# 直接按文件路径运行时，把 ros2_ws/src 加进 sys.path，让 `import mm_base` 可用；
# 用 `python -m mm_base.selftest` 运行时 __package__ 非空，不需要这一步。
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mm_base.backend import (  # noqa: E402
    BackendUnavailable,
    CanBackend,
    LoopbackBackend,
    RecordingBackend,
    SocketCanBackend,
)
from mm_base.kinematics import (  # noqa: E402
    DifferentialOdometry,
    rpm_to_wheel_radps,
    twist_to_wheel_speeds,
    wheel_speeds_to_twist,
    yaw_to_quaternion,
)
from mm_base.protocol import (  # noqa: E402
    ODOM_WHEEL_TRACK_M,
    WHEEL_TRACK_CMD_M,
    StartupError,
    clear_alarm_frame,
    counts_to_m,
    disable_frame,
    driver_cmd_id,
    driver_feedback_id,
    enable_frame,
    m_to_counts,
    mps_to_rpm,
    parse_feedback,
    parse_mode_ack,
    rpm_to_mps,
    rpm_to_speed_value,
    run_startup_sequence,
    speed_mode_frame,
    speed_value_frame,
    speed_value_to_rpm,
    torque_mode_frame,
    torque_value_frame,
)

#: 自检固定仿真步长（1 ms），保证结果可复现。
SIM_STEP_S = 0.001
#: 自检里模拟 125 Hz 上位机（每 8 ms 发一次速度帧）。
COMMAND_EVERY_STEPS = 8


class _Checker:
    """断言计数器：每条断言都留下计数，失败抛 AssertionError。"""

    def __init__(self) -> None:
        self.count = 0

    def check(self, condition: bool, message: str) -> None:
        """记录并检查一条断言。"""
        self.count += 1
        if not condition:
            raise AssertionError(message)

    def close(self, actual: float, expected: float, tol: float, label: str) -> None:
        """检查浮点数落在绝对容差内，失败信息带实际值。"""
        self.check(
            abs(actual - expected) <= tol,
            f"{label}: 期望 {expected:.6g} ± {tol:.3g}，实际 {actual:.6g}",
        )


def _drain(backend: LoopbackBackend) -> list[tuple[int, bytes]]:
    """取完回环后端当前队列里的所有帧。"""
    frames: list[tuple[int, bytes]] = []
    while True:
        frame = backend.recv(0.0)
        if frame is None:
            return frames
        frames.append(frame)


def _feed_odometry(backend: LoopbackBackend, odom: DifferentialOdometry) -> None:
    """把队列里的 0xCA 编码器帧喂给里程计。"""
    for can_id, data in _drain(backend):
        feedback = parse_feedback(can_id, data)
        if feedback is not None and feedback.encoder_counts is not None:
            odom.update_counts({feedback.driver: feedback.encoder_counts})


def _prime(backend: LoopbackBackend, odom: DifferentialOdometry) -> None:
    """先跑 10 ms 零速反馈，给里程计建立基线。"""
    backend.advance(0.01)
    _feed_odometry(backend, odom)


def _send_wheel_commands(
    backend: LoopbackBackend,
    left_mps: float,
    right_mps: float,
) -> None:
    """按厂家左右侧分配把两个轮速发给四个驱动器。"""
    left_rpm = mps_to_rpm(left_mps)
    right_rpm = mps_to_rpm(right_mps)
    for driver in (1, 4):
        backend.send(driver_cmd_id(driver), speed_value_frame(left_rpm))
    for driver in (2, 3):
        backend.send(driver_cmd_id(driver), speed_value_frame(right_rpm))


def _simulate(
    linear: float,
    angular: float,
    duration: float,
    odom_track: float = ODOM_WHEEL_TRACK_M,
    enabled: bool = True,
) -> tuple[DifferentialOdometry, LoopbackBackend]:
    """用回环后端跑一段固定步长仿真。

    参数：
        linear: /cmd_vel 线速度 m/s。
        angular: /cmd_vel 角速度 rad/s。
        duration: 仿真时长 s。
        odom_track: 里程计轮距 m。
        enabled: False 时不走启动握手、不使能，用于安全断言。
    返回：
        ``(odom, loopback)``。
    重要性等级：B。
    """
    backend = LoopbackBackend()
    if enabled:
        run_startup_sequence(backend, timeout_s=1.0)
    else:
        # 只切转速模式，不发使能：模拟“上位机配了模式但忘了使能”。
        for driver in range(1, 5):
            backend.send(driver_cmd_id(driver), speed_mode_frame())

    odom = DifferentialOdometry(odom_track)
    _prime(backend, odom)

    left, right = twist_to_wheel_speeds(
        linear,
        angular,
        wheel_track=WHEEL_TRACK_CMD_M,
        max_speed=1.0,
        max_yaw_rate=None,
    )
    steps = int(round(duration / SIM_STEP_S))
    for step in range(steps):
        if step % COMMAND_EVERY_STEPS == 0:
            _send_wheel_commands(backend, left, right)
        backend.advance(SIM_STEP_S)
        _feed_odometry(backend, odom)
    return odom, backend


def test_command_frames(checker: _Checker) -> None:
    """1. 七种命令帧逐字节对照厂家源码。"""
    cases = [
        ("使能", enable_frame(), "00 24 00 10 00 01 00 00"),
        ("失能", disable_frame(), "00 1E 00 10 00 00 00 00"),
        ("清报警", clear_alarm_frame(), "00 21 00 10 00 00 00 01"),
        ("转矩模式", torque_mode_frame(), "00 1E 00 03 00 01 00 00"),
        ("转速模式", speed_mode_frame(), "00 1E 00 03 00 02 00 00"),
        ("转矩 +1910", torque_value_frame(1910), "00 28 00 01 07 76 00 00"),
        ("转矩 -1910", torque_value_frame(-1910), "00 28 00 01 F8 8A 00 00"),
        ("转速 +2505", speed_value_frame(2505), "00 28 00 00 09 C9 00 00"),
        ("转速 -2505", speed_value_frame(-2505), "00 28 00 00 F6 37 00 00"),
    ]
    for label, actual, expected_hex in cases:
        expected = bytes.fromhex(expected_hex)
        checker.check(
            actual == expected,
            f"{label}帧不一致：期望 {expected.hex(' ').upper()}，实际 {actual.hex(' ').upper()}",
        )
    checker.check(len(enable_frame()) == 8, "命令帧 DLC 必须为 8")


def test_can_ids(checker: _Checker) -> None:
    """2. 命令/反馈 ID 映射。"""
    for driver in range(1, 5):
        checker.check(
            driver_cmd_id(driver) == 0x100 + driver,
            f"驱动器 {driver} 命令 ID 应为 0x{0x100 + driver:03X}，实际 0x{driver_cmd_id(driver):03X}",
        )
        checker.check(
            driver_feedback_id(driver) == 0x600 + driver,
            f"驱动器 {driver} 反馈 ID 应为 0x{0x600 + driver:03X}，"
            f"实际 0x{driver_feedback_id(driver):03X}",
        )
    for bad in (0, 5, -1):
        try:
            driver_cmd_id(bad)
        except ValueError:
            checker.check(True, "")
        else:
            checker.check(False, f"非法驱动器号 {bad} 应抛 ValueError")


def test_feedback_decode(checker: _Checker) -> None:
    """3. 反馈解码往返与边界情况。"""
    pos = parse_feedback(0x601, bytes.fromhex("00 CA 00 01 E2 40 05 DC"))
    checker.check(pos is not None, "0xCA 正常帧不应返回 None")
    checker.check(pos.encoder_counts == 123456, f"正计数解码错误：{pos.encoder_counts}")
    checker.check(pos.current_raw == 1500, f"电流原始值解码错误：{pos.current_raw}")
    checker.check(pos.driver == 1, f"驱动器号解码错误：{pos.driver}")

    neg = parse_feedback(0x604, bytes.fromhex("00 CA FF FE 1D C0 00 00"))
    checker.check(neg is not None and neg.encoder_counts == -123456, "负计数解码错误")

    status = parse_feedback(0x602, bytes.fromhex("00 CB 00 2A FF 9C 00 07"))
    checker.check(status is not None, "0xCB 正常帧不应返回 None")
    checker.check(status.temperature_c == 42, f"温度解码错误：{status.temperature_c}")
    checker.check(
        status.speed_rpm == 100,
        f"源码对转速取负：raw=-100 应解出 +100，实际 {status.speed_rpm}",
    )
    checker.check(status.fault_code == 7, f"故障码解码错误：{status.fault_code}")

    status2 = parse_feedback(0x603, bytes.fromhex("00 CB 00 19 00 64 00 00"))
    checker.check(
        status2 is not None and status2.speed_rpm == -100,
        "源码对转速取负：raw=+100 应解出 -100",
    )
    # 0x8000 是 int16 的最小值；C 里 -(-32768) 赋给 int16_t 会回绕成 -32768。
    status3 = parse_feedback(0x603, bytes.fromhex("00 CB 00 19 80 00 00 00"))
    checker.check(
        status3 is not None and status3.speed_rpm == -32768,
        "raw=0x8000 应按 int16_t 回绕成 -32768",
    )

    checker.check(
        parse_feedback(0x601, bytes.fromhex("00 AA 00 00 00 00 00 00")) is None,
        "未知反馈命令字应返回 None",
    )
    checker.check(parse_feedback(0x600, bytes.fromhex("00 CA 00 00 00 00 00 00")) is None,
                  "0x600 不是反馈 ID，应返回 None")
    try:
        parse_feedback(0x601, bytes.fromhex("00 CA 00 00 00 00 00"))
    except ValueError:
        checker.check(True, "")
    else:
        checker.check(False, "7 字节反馈帧应抛 ValueError")

    ack_enable = parse_mode_ack(0x700, bytes([1, 0x25, 0, 0x10, 0, 1, 0, 0]))
    checker.check(
        ack_enable is not None and ack_enable.driver == 1 and ack_enable.kind == "enable",
        "使能应答解析错误",
    )
    ack_mode = parse_mode_ack(0x700, bytes([4, 0x1F, 0, 0, 0, 0, 0, 0]))
    checker.check(
        ack_mode is not None and ack_mode.driver == 4 and ack_mode.kind == "mode",
        "模式应答解析错误",
    )
    checker.check(
        parse_mode_ack(0x700, bytes([1, 0x25, 0, 0x11, 0, 1, 0, 0])) is None,
        "非 0x10 参数的应答不应被当成使能成功",
    )


def test_units(checker: _Checker) -> None:
    """4. 单位换算、定点编码与除零保护。"""
    checker.close(rpm_to_mps(3000), 1.199, 0.002, "3000 rpm -> m/s")
    checker.close(mps_to_rpm(rpm_to_mps(2505.0)), 2505.0, 1e-6, "rpm 往返")
    checker.close(counts_to_m(417455), 1.0, 1e-12, "417455 count -> m")
    checker.close(m_to_counts(1.0), 417455, 1e-9, "1 m -> count")
    checker.check(rpm_to_speed_value(2505.0) == 2505, "转速定点编码")
    checker.close(speed_value_to_rpm(-2505), -2505.0, 1e-12, "转速定点解码")

    for bad in (40000.0, -40000.0, float("nan"), float("inf")):
        try:
            rpm_to_speed_value(bad)
        except ValueError:
            checker.check(True, "")
        else:
            checker.check(False, f"非法转速 {bad!r} 应抛 ValueError")

    try:
        wheel_speeds_to_twist(1.0, 1.0, 0.0)
    except ValueError:
        checker.check(True, "")
    else:
        checker.check(False, "轮距为 0 应抛 ValueError（不能除零）")

    left, right = twist_to_wheel_speeds(1.0, 0.0, 0.61, 1.0)
    checker.close(left, 1.0, 1e-12, "直线左轮")
    checker.close(right, -1.0, 1e-12, "直线右轮（厂家取负）")
    left, right = twist_to_wheel_speeds(0.0, 1.0, 0.61, 1.0, max_yaw_rate=1.0)
    checker.close(left, -0.305, 1e-12, "原地转左轮")
    checker.close(right, -0.305, 1e-12, "原地转右轮（厂家取负）")
    z, w = yaw_to_quaternion(math.pi)
    checker.close(w, 0.0, 1e-12, "yaw=pi 四元数 w")
    checker.close(z, 1.0, 1e-12, "yaw=pi 四元数 z")
    checker.close(rpm_to_wheel_radps(763.94), 2.0, 2e-3, "764 rpm -> 轮角速度")


def test_straight(checker: _Checker) -> None:
    """5. 直线 1.0 m/s × 2.0 s。"""
    odom, backend = _simulate(1.0, 0.0, 2.0)
    x, y, yaw = odom.pose
    print(f"       straight: x={x:.4f} m, y={y:.6f} m, yaw={yaw:.9f} rad")
    checker.close(x, 2.0, 0.04, "直线位移（±2%）")
    checker.close(y, 0.0, 1e-6, "直线横向漂移")
    checker.close(yaw, 0.0, 1e-6, "直线偏航")
    checker.check(odom.integrated_sets == 200, f"应积分 200 组，实际 {odom.integrated_sets}")
    checker.check(backend.encoder_counts(1) > 0, "左前编码器应随前进增加")
    checker.check(backend.encoder_counts(2) < 0, "右前编码器应随前进减少")
    backend.close()


def test_rotation(checker: _Checker) -> None:
    """6. 原地转 1.0 rad/s × 1.0 s。"""
    odom_135, backend_135 = _simulate(
        0.0, 1.0, 1.0, odom_track=ODOM_WHEEL_TRACK_M
    )
    x135, y135, yaw135 = odom_135.pose
    print(
        f"       rotate @ odom_track=1.35: x={x135:.6f} m, y={y135:.6f} m, "
        f"yaw={yaw135:.6f} rad"
    )
    # 0.61 的指令轮距 / 1.35 的里程计轮距 = 0.452；这是 1.35 滑移补偿的定义，
    # 与任务书 D5 的“≈1.0 rad”不能同时成立，详见 docs/can_driver.md。
    checker.close(yaw135, WHEEL_TRACK_CMD_M / ODOM_WHEEL_TRACK_M, 0.023, "原地转偏航（1.35 m 口径）")
    checker.close(x135, 0.0, 1e-6, "原地转线位移 x")
    checker.close(y135, 0.0, 1e-6, "原地转线位移 y")
    backend_135.close()

    odom_061, backend_061 = _simulate(0.0, 1.0, 1.0, odom_track=WHEEL_TRACK_CMD_M)
    _, _, yaw061 = odom_061.pose
    print(f"       rotate @ odom_track=0.61: yaw={yaw061:.6f} rad")
    checker.close(yaw061, 1.0, 0.05, "几何轮距下的偏航（±5%）")
    backend_061.close()


def test_safety_not_enabled(checker: _Checker) -> None:
    """7. 安全断言：未使能时速度命令不产生运动。"""
    odom, backend = _simulate(1.0, 0.0, 1.0, enabled=False)
    x, y, yaw = odom.pose
    print(f"       not enabled: x={x:.6f} m, yaw={yaw:.6f} rad")
    checker.close(x, 0.0, 1e-9, "未使能直线位移")
    checker.close(yaw, 0.0, 1e-9, "未使能偏航")
    for driver in range(1, 5):
        checker.check(backend.encoder_counts(driver) == 0, f"未使能时驱动器 {driver} 计数非零")
    backend.close()

    # 转矩模式下收到速度给定也不应运动（模式没切对）。
    torque_backend = LoopbackBackend()
    for driver in range(1, 5):
        torque_backend.send(driver_cmd_id(driver), enable_frame())
        torque_backend.send(driver_cmd_id(driver), speed_value_frame(mps_to_rpm(1.0)))
    torque_backend.advance(0.5)
    speeds = []
    for can_id, data in _drain(torque_backend):
        feedback = parse_feedback(can_id, data)
        if feedback is not None and feedback.speed_rpm is not None:
            speeds.append(feedback.speed_rpm)
    checker.check(all(speed == 0 for speed in speeds), f"转矩模式不应响应速度帧：{speeds}")
    checker.check(torque_backend.encoder_counts(1) == 0, "转矩模式编码器计数非零")
    torque_backend.close()

    # 运行中失能：计数必须立刻冻结。
    backend = LoopbackBackend()
    run_startup_sequence(backend, timeout_s=1.0)
    odom = DifferentialOdometry(ODOM_WHEEL_TRACK_M)
    _prime(backend, odom)
    _send_wheel_commands(backend, 1.0, -1.0)
    for _ in range(200):
        backend.advance(SIM_STEP_S)
        _feed_odometry(backend, odom)
    before = backend.encoder_counts(1)
    for driver in range(1, 5):
        backend.send(driver_cmd_id(driver), disable_frame())
    for _ in range(1000):
        backend.advance(SIM_STEP_S)
        _feed_odometry(backend, odom)
    after = backend.encoder_counts(1)
    checker.check(before != 0, "失能前应有运动")
    checker.check(after == before, f"失能后计数应冻结：{before} -> {after}")
    backend.close()


def test_startup_order(checker: _Checker) -> None:
    """8. 启动序列：清报警 → 使能 → 转速模式 → 零速。"""
    loopback = LoopbackBackend()
    recorder = RecordingBackend(loopback)
    run_startup_sequence(recorder, timeout_s=1.0)

    # 回环后端按 100 Hz 产反馈：1 s = 100 组 × 4 轮 × (0xCA+0xCB) = 800 帧。
    loopback.advance(1.0)
    feedback = _drain(loopback)
    checker.check(len(feedback) == 800, f"1 s 应产生 800 帧反馈，实际 {len(feedback)}")
    checker.check(
        sum(1 for _, data in feedback if data[1] == 0xCA) == 400,
        "0xCA 反馈帧数量应为 400",
    )
    recorder.close()

    kinds = []
    for can_id, data in recorder.frames:
        if data[1] == 0x21:
            kinds.append("clear")
        elif data[1] == 0x24:
            kinds.append("enable")
        elif data[1] == 0x1E and data[3] == 0x03:
            kinds.append("mode")
        elif data[1] == 0x28 and data[3] == 0x00:
            kinds.append("speed")
        else:
            kinds.append(f"unknown:{data.hex()}")
    expected = ["clear"] * 4 + ["enable"] * 4 + ["mode"] * 4 + ["speed"] * 4
    checker.check(kinds == expected, f"启动顺序错误：{kinds}")
    checker.check(len(recorder.frames) == 16, f"启动帧数应为 16，实际 {len(recorder.frames)}")

    for index, (can_id, data) in enumerate(recorder.frames):
        driver = index % 4 + 1
        checker.check(
            can_id == driver_cmd_id(driver),
            f"第 {index + 1} 帧 ID 应为 0x{driver_cmd_id(driver):03X}，"
            f"实际 0x{can_id:03X}",
        )
        if kinds[index] == "mode":
            checker.check(data[5] == 2, "转速模式参数应为 2")
        if kinds[index] == "speed":
            checker.check(data[4:6] == b"\x00\x00", "启动末尾的零速帧必须为 0")

    # 超时上报：后端永远不回 ACK，必须抛 StartupError 而不是静默继续。
    class _SilentBackend(CanBackend):
        def __init__(self) -> None:
            self.frames: list[tuple[int, bytes]] = []

        def send(self, can_id: int, data: bytes) -> None:
            self.frames.append((can_id, bytes(data)))

        def recv(self, timeout: float | None = None) -> tuple[int, bytes] | None:
            return None

        def close(self) -> None:
            pass

    silent = _SilentBackend()
    try:
        run_startup_sequence(silent, timeout_s=0.05)
    except StartupError as exc:
        checker.check("使能应答超时" in str(exc), f"超时错误信息不清楚：{exc}")
    else:
        checker.check(False, "无 ACK 时应抛 StartupError")
    checker.check(len(silent.frames) == 8, "超时前应已发清报警+使能共 8 帧")


def test_platform_guards(checker: _Checker) -> None:
    """9. Windows 上 SocketCAN 只是不可用；协议层不 import rclpy/socket。"""
    if sys.platform == "win32":
        try:
            SocketCanBackend("can0")
        except BackendUnavailable:
            checker.check(True, "")
        else:
            checker.check(False, "Windows 上 SocketCanBackend 应抛 BackendUnavailable")
    else:
        # 其它平台只验证构造失败类型不崩；本自检的目标环境是 Windows。
        checker.check(True, "")

    package_dir = Path(__file__).resolve().parent

    def imports_of(name: str) -> set[str]:
        tree = ast.parse((package_dir / name).read_text(encoding="utf-8"))
        modules: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules.add(node.module.split(".")[0])
        return modules

    protocol_imports = imports_of("protocol.py")
    checker.check("rclpy" not in protocol_imports, "protocol.py 不得 import rclpy")
    checker.check("socket" not in protocol_imports, "protocol.py 不得 import socket")
    checker.check("rclpy" not in imports_of("selftest.py"), "selftest.py 不得 import rclpy")
    checker.check("rclpy" not in imports_of("kinematics.py"), "kinematics.py 不得 import rclpy")
    # base_node 需要 rclpy，Windows 上没有：至少证明语法可解析（py_compile 另跑）。
    ast.parse((package_dir / "base_node.py").read_text(encoding="utf-8"))
    checker.check(True, "")


TESTS = [
    ("1 command frame bytes", test_command_frames),
    ("2 CAN ID 0x101..0x104", test_can_ids),
    ("3 feedback decode and edge cases", test_feedback_decode),
    ("4 unit conversion and zero-division guard", test_units),
    ("5 straight 1.0 m/s x 2.0 s", test_straight),
    ("6 rotate-in-place 1.0 rad/s x 1.0 s", test_rotation),
    ("7 not enabled -> no motion (safety)", test_safety_not_enabled),
    ("8 startup order and timeout reporting", test_startup_order),
    ("9 Windows SocketCAN guard and import rules", test_platform_guards),
]


def main() -> int:
    """运行全部自检并打印结果。

    返回：
        0 全部通过；1 有断言失败或异常。
    重要性等级：A（交付前必须看到 0）。
    """
    print("mm_base selftest: Windows / no ROS2 / no CAN hardware")
    print(f"Python {sys.version.split()[0]}")
    checker = _Checker()
    for name, function in TESTS:
        try:
            function(checker)
        except AssertionError as exc:
            print(f"[FAIL] {name}: {exc}")
            return 1
        except Exception:
            print(f"[ERROR] {name}")
            traceback.print_exc()
            return 1
        print(f"[PASS] {name}")
    print("-" * 64)
    print(f"ALL {checker.count} ASSERTIONS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
