"""mm_base.kinematics：四轮差速的指令分解、里程计与轮关节换算（纯数学）。

为什么单独一层
--------------
``mm_base.protocol`` 只认报文，``mm_base.base_node`` 只认 ROS；把符号、轮距、积分
放在这里，Windows 上的 ``mm_base.selftest`` 就能直接测真实节点用的同一套数学，
不需要 rclpy，也不需要 CAN 硬件。

两个轮距，别用错
----------------
- 指令分解用 ``WHEEL_TRACK_CMD_M = 0.61``：把 /cmd_vel 拆成左右轮目标转速；
- 里程计用 ``ODOM_WHEEL_TRACK_M = 1.35``：厂家用角度校准标出来的滑移补偿值，
  约为几何轮距 0.594 的 2.27 倍。同一个轮速差，按 1.35 算出的横摆只有几何模型
  的 44%，厂家说明书明说“轮距设置大约为真实轮距的 2 倍多”。

符号
----
厂家指令里右轮取负（电机镜像安装）：

    v_L = v - ω·0.61/2
    v_R = -(v + ω·0.61/2)

本模块把 ``v_L`` / ``v_R`` 都定义成“车轮前进方向为正”的物理量，因此：

- 指令帧里右轮的值 = ``-v_R``；
- 0xCA 编码器计数里，左轮随前进增加、右轮随前进减少（厂家里程计对右轮取负）；
- 0xCB 转速反馈已被厂家取负，所以右轮前进时 ``speed_rpm`` 为负。

重要性等级：A。
"""

from __future__ import annotations

import math
from typing import Mapping

from mm_base.protocol import (
    COUNTS_PER_METER,
    GEAR_RATIO,
    ODOM_WHEEL_TRACK_M,
    WHEEL_RADIUS_M,
    WHEEL_TRACK_CMD_M,
)

#: 原始计数差 → 车轮前进距离的符号：左前/左后取正，右前/右后取负。
#: 出处：``zeus_s2_driver.cpp`` 里 ``delta_left_ = +(rev-cur)/ticks_per_meter``、
#: ``delta_right_ = -(rev-cur)/ticks_per_meter``。
WHEEL_SIDE_SIGN = {1: 1.0, 2: -1.0, 3: -1.0, 4: 1.0}


def _finite(value: float, name: str) -> float:
    """把输入转成有限浮点数，非有限值抛 ``ValueError``。

    参数：
        value: 待检查值。
        name: 出错信息里的字段名。
    返回：
        float(value)。
    异常：
        ValueError: NaN/±inf。
    重要性等级：A（速度/轮距里的 NaN 会污染整条控制链）。
    """
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值，收到 {value!r}")
    return result


def wrap_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi)。

    参数：
        angle: 任意弧度角。
    返回：
        等价的最小表示角。
    重要性等级：B。
    """
    value = _finite(angle, "angle")
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def twist_to_wheel_speeds(
    linear: float,
    angular: float,
    wheel_track: float = WHEEL_TRACK_CMD_M,
    max_speed: float = 1.0,
    max_yaw_rate: float | None = None,
    invert_side: bool = False,
) -> tuple[float, float]:
    """把 /cmd_vel 分解成左右轮目标线速度（厂家指令符号）。

    公式（出处 ``send_speed_callback()``）：

        left  = v - ω·wheel_track/2
        right = -(v + ω·wheel_track/2)

    返回值里的 ``right`` 已经是**指令帧中的符号**（前进时为负）；节点直接把它
    换成 rpm 发帧即可，不要再取负。

    参数：
        linear: 前进速度 m/s。
        angular: 横摆角速度 rad/s（逆时针为正）。
        wheel_track: 指令分解轮距 m，默认 0.61。
        max_speed: 单轮线速度限幅 m/s，默认 1.0。
        max_yaw_rate: 可选横摆限幅 rad/s；None 表示只靠单轮限幅。
        invert_side: True 时左右两侧指令同时取反，用于电机镜像方向与厂家相反的车。
    返回：
        ``(left, right)``，单位 m/s。
    异常：
        ValueError: 输入非有限、轮距/限速非正、横摆限幅为负。
    重要性等级：A。
    """
    v = _finite(linear, "linear")
    w = _finite(angular, "angular")
    track = _finite(wheel_track, "wheel_track")
    limit = _finite(max_speed, "max_speed")
    if track <= 0.0:
        raise ValueError("wheel_track 必须大于 0")
    if limit <= 0.0:
        raise ValueError("max_speed 必须大于 0")

    v = max(-limit, min(limit, v))
    if max_yaw_rate is not None:
        yaw_limit = _finite(max_yaw_rate, "max_yaw_rate")
        if yaw_limit < 0.0:
            raise ValueError("max_yaw_rate 不能为负")
        w = max(-yaw_limit, min(yaw_limit, w))

    left = v - w * track / 2.0
    right = -(v + w * track / 2.0)
    # 原地转或 v/ω 组合过大时，单轮可能超过限幅；逐轮钳位，宁可少转也不超速。
    left = max(-limit, min(limit, left))
    right = max(-limit, min(limit, right))
    if invert_side:
        left, right = -left, -right
    return left, right


def wheel_speeds_to_twist(
    left_mps: float,
    right_mps: float,
    wheel_track: float = ODOM_WHEEL_TRACK_M,
) -> tuple[float, float]:
    """左右轮前进速度 → 车体线速度/角速度（与里程计同口径）。

    参数：
        left_mps: 左轮前进速度 m/s（左前/左后平均）。
        right_mps: 右轮前进速度 m/s（右前/右后平均）。
        wheel_track: 里程计轮距 m，默认 1.35。
    返回：
        ``(linear, angular)``，单位 m/s、rad/s。
    异常：
        ValueError: 输入非有限或轮距非正。
    重要性等级：A。
    """
    left = _finite(left_mps, "left_mps")
    right = _finite(right_mps, "right_mps")
    track = _finite(wheel_track, "wheel_track")
    if track <= 0.0:
        raise ValueError("wheel_track 必须大于 0")
    return (left + right) / 2.0, (right - left) / track


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """航向角 → 只绕 z 轴的四元数。

    参数：
        yaw: 航向角 rad。
    返回：
        ``(z, w)``。
    异常：
        ValueError: yaw 非有限值。
    重要性等级：B。
    """
    half = _finite(yaw, "yaw") / 2.0
    return math.sin(half), math.cos(half)


def rpm_to_wheel_radps(rpm: float) -> float:
    """电机转速 → 车轮角速度（rad/s）。

    公式：``rpm / 40 * 2π / 60``。注意编码器在电机侧，减速比在这里除。
    参数：
        rpm: 电机转速。
    返回：
        车轮角速度 rad/s。
    异常：
        ValueError: rpm 非有限值。
    重要性等级：B（/joint_states 用）。
    """
    return _finite(rpm, "rpm") / GEAR_RATIO * 2.0 * math.pi / 60.0


def distance_to_wheel_angle(distance: float) -> float:
    """车轮行驶距离 → 车轮转角。

    参数：
        distance: 车轮前进距离 m。
    返回：
        转角 rad。
    异常：
        ValueError: distance 非有限值。
    重要性等级：B（/joint_states 位置用）。
    """
    return _finite(distance, "distance") / WHEEL_RADIUS_M


def int32_delta(new: int, old: int) -> int:
    """计算 int32 计数差，正确处理 ±2^31 回绕。

    参数：
        new: 新计数。
        old: 旧计数。
    返回：
        ``new - old`` 的有符号 32 位表示，范围 [-2^31, 2^31-1]。
    重要性等级：A（跨过回绕点时不能把里程计打成负无穷）。
    """
    return ((int(new) - int(old) + 0x80000000) % 0x100000000) - 0x80000000


class DifferentialOdometry:
    """四轮差速里程计：编码器计数 → 位姿。

    规则：
        - 四个轮子都拿到新计数才积分一次；缺一个就继续等（宁可停更，不拿三轮猜）；
        - 左/右两侧各取前后轮平均；
        - 轮距用 1.35 m（厂家滑移补偿口径），不是 0.61；
        - 位姿用中点航向积分，纯旋转时线位移严格为 0。

    重要性等级：A（导航定位输入）。
    """

    def __init__(self, odom_wheel_track: float = ODOM_WHEEL_TRACK_M) -> None:
        """创建里程计。

        参数：
            odom_wheel_track: 里程计轮距 m，默认厂家 1.35。
        异常：
            ValueError: 轮距非有限或非正。
        重要性等级：A。
        """
        track = _finite(odom_wheel_track, "odom_wheel_track")
        if track <= 0.0:
            raise ValueError("odom_wheel_track 必须大于 0")
        self._track = track
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_counts: dict[int, int] = {}
        self._pending: dict[int, int] = {n: 0 for n in range(1, 5)}
        self._have: dict[int, bool] = {n: False for n in range(1, 5)}
        self._wheel_distance: dict[int, float] = {n: 0.0 for n in range(1, 5)}
        self._last_left = 0.0
        self._last_right = 0.0
        self._integrated_sets = 0

    @property
    def pose(self) -> tuple[float, float, float]:
        """当前位姿 ``(x, y, yaw)``，单位 m、m、rad。"""
        return self._x, self._y, self._yaw

    @property
    def integrated_sets(self) -> int:
        """已完成积分的四轮数据组数。"""
        return self._integrated_sets

    @property
    def last_left_distance(self) -> float:
        """最近一次积分得到的左轮前进距离 m。"""
        return self._last_left

    @property
    def last_right_distance(self) -> float:
        """最近一次积分得到的右轮前进距离 m。"""
        return self._last_right

    def wheel_distance(self, driver: int) -> float:
        """返回某轮累计前进距离。

        参数：
            driver: 1..4（左前/右前/右后/左后）。
        返回：
            累计前进距离 m（右轮已按符号还原）。
        异常：
            ValueError: 驱动器号非法。
        重要性等级：B（/joint_states 位置用）。
        """
        if int(driver) not in self._wheel_distance:
            raise ValueError(f"驱动器号必须为 1..4，收到 {driver!r}")
        return self._wheel_distance[int(driver)]

    def update_counts(self, counts_by_driver: Mapping[int, int]) -> bool:
        """喂入一个或多个驱动器的原始 0xCA 编码器计数。

        参数：
            counts_by_driver: ``{驱动器号: int32 计数}``，驱动器号 1..4。
        返回：
            True 表示四个轮子都齐了、本次完成了一次位姿积分；否则 False。
        异常：
            ValueError: 驱动器号非法。
        副作用：
            更新内部位姿与各轮累计距离。
        重要性等级：A。
        """
        for driver, counts in counts_by_driver.items():
            key = int(driver)
            if key not in self._pending:
                raise ValueError(f"驱动器号必须为 1..4，收到 {driver!r}")
            value = int(counts)
            if key not in self._last_counts:
                # 第一帧只建立基线，不贡献位移（上电计数是任意的）。
                self._last_counts[key] = value
                continue
            delta = int32_delta(value, self._last_counts[key])
            self._last_counts[key] = value
            self._pending[key] += delta
            self._have[key] = True

        if not all(self._have.values()):
            return False

        deltas = {
            n: WHEEL_SIDE_SIGN[n] * self._pending[n] / COUNTS_PER_METER
            for n in range(1, 5)
        }
        for n in range(1, 5):
            self._wheel_distance[n] += deltas[n]
        left = (deltas[1] + deltas[4]) / 2.0
        right = (deltas[2] + deltas[3]) / 2.0
        self._integrate_pose(left, right)
        self._pending = {n: 0 for n in range(1, 5)}
        self._have = {n: False for n in range(1, 5)}
        self._integrated_sets += 1
        return True

    def _integrate_pose(self, left: float, right: float) -> None:
        """用左右轮位移更新位姿。

        参数：
            left: 左轮本次前进距离 m。
            right: 右轮本次前进距离 m。
        副作用：
            更新 x/y/yaw 与 last_left/last_right；航向用中点角积分。
        重要性等级：A。
        """
        distance = (left + right) / 2.0
        delta_yaw = (right - left) / self._track
        yaw_mid = self._yaw + delta_yaw / 2.0
        self._x += distance * math.cos(yaw_mid)
        self._y += distance * math.sin(yaw_mid)
        self._yaw = wrap_angle(self._yaw + delta_yaw)
        self._last_left = left
        self._last_right = right
