"""mm_base.protocol：底盘 CAN 报文与单位换算（纯 Python，不 import rclpy / socket）。

协议来源
--------
厂家 ROS1 驱动 ``zeus_s2_driver``（2022-09-03 版，只读参考）：

- ``src/zeus_s2/zeus_s2_driver/src/zeus_s2_driver.cpp``
- ``src/zeus_s2/zeus_s2_driver/include/zeus_s2_driver/zeus_s2_driver.h``
- 查证记录：``docs/vehicle_workspace_refs.md`` 第一、二节。

帧格式
------
标准帧，DLC=8，多字节数值**大端**。命令帧 ID = ``0x101 + (n-1)``，n=1..4，
轮序为左前/右前/右后/左后；反馈帧 ID = ``0x601..0x604``，模式应答 ID = ``0x700``。

符号约定（真机最容易错的地方）
--------------------------------
厂家源码对 **0xCB 反馈转速取负号**（``motorActSpeed = -(...)``），但 **0xCA 编码器
计数不取负**；里程计里再对右轮计数取负（``delta_right = -...``）。因此本模块：

- ``DriverFeedback.speed_rpm`` 返回**已按源码取负**后的值；
- ``DriverFeedback.encoder_counts`` 返回原始 int32 计数，不做符号处理；
- 左右轮、前后轮的符号统一交给 ``mm_base.kinematics`` 处理。

安全性
------
本模块只构造/解析报文，不打开任何设备。所有命令帧长度固定 8；数值越界、帧长度
不对会抛 ``ValueError``，不会被静默截断。重要性等级：A。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

# ---------------------------------------------------------------------------
# 报文常量（全部来自厂家源码，改之前先对照 docs/vehicle_workspace_refs.md）
# ---------------------------------------------------------------------------

#: CAN 标准帧数据长度。厂家 ``can-ctrl.c`` 的 ``can_frame.can_dlc = 8``。
CAN_FRAME_LEN = 8

#: 命令帧 ID 基址；出处 ``zeus_s2_driver.h`` 的 ``DRIVER_ID_ONE_CONTROL = 0x101``。
CMD_ID_BASE = 0x101

#: 反馈帧 ID 基址；出处 ``zeus_s2_driver.h`` 的 ``DRIVER_ID_ONE_POSE_ACK = 0x601``。
FB_ID_BASE = 0x601

#: 模式/使能应答帧 ID；出处 ``zeus_s2_driver.h`` 的 ``DRIVER_ID_MODE_ACK = 0x700``。
MODE_ACK_ID = 0x700

#: 使能命令码；出处 ``enable_motor_driver()`` 的 ``tx_buff[1] = 0x24``。
CMD_ENABLE = 0x24
#: 写参数命令码；出处 ``disable_motor_driver()`` / 模式设置函数。
CMD_WRITE_PARAM = 0x1E
#: 清报警命令码；出处 ``clear_ALM_motor_driver()`` 的 ``tx_buff[1] = 0x21``。
CMD_CLEAR_ALARM = 0x21
#: 设运行值命令码；出处 ``Set_torque_value_to_motor()`` / ``Set_speed_value_to_motor()``。
CMD_SET_VALUE = 0x28

#: 参数索引：使能；出处 ``data[2..3] = 00 10``。
PARAM_ENABLE = 0x0010
#: 参数索引：运行模式；出处 ``data[2..3] = 00 03``。
PARAM_MODE = 0x0003
#: 参数索引：转矩值；出处 ``Set_torque_value_to_motor()`` 的 ``data[3] = 0x01``。
PARAM_TORQUE = 0x0001
#: 参数索引：转速值；出处 ``Set_speed_value_to_motor()`` 的 ``data[3] = 0x00``。
PARAM_SPEED = 0x0000

#: 转矩模式参数值；出处 ``Set_motor_torque_mode()`` 的 ``data[5] = 1``。
MODE_TORQUE = 1
#: 转速模式参数值；出处 ``Set_motor_speed_mode()`` 的 ``data[5] = 2``。
MODE_SPEED = 2

#: 反馈命令字：编码器计数；出处 ``updateParam()`` 的 ``data[1] == 0xCA``。
FB_ENCODER = 0xCA
#: 反馈命令字：温度/转速/故障码；出处 ``updateParam()`` 的 ``data[1] == 0xCB``。
FB_STATUS = 0xCB

#: 使能应答字；出处 ``Calculate_LeftAndRight_Meter()`` 检查 ``data[1] == 0x25``。
ACK_ENABLE_WORD = 0x25
#: 使能应答里回显的参数索引低字节；出处同一处检查 ``data[3] == 0x10``。
ACK_ENABLE_INDEX_LOW = 0x10
#: 使能应答里回显的值；出处同一处检查 ``data[5] == 1``。
ACK_ENABLE_VALUE = 1
#: 模式应答字；出处 ``mode_switch_judge()`` 检查 ``data[1] == 0x1F``。
ACK_MODE_WORD = 0x1F

# ---------------------------------------------------------------------------
# 底盘几何与单位（出处 docs/vehicle_workspace_refs.md 第二节）
# ---------------------------------------------------------------------------

#: 车轮直径 m；出处 ``zeus_params.yaml`` ``wheel_diameter_4wd``。
WHEEL_DIAMETER_M = 0.305
#: 车轮半径 m（仿真/URDF 标称 0.1525）。
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
#: 减速比；出处 ``zeus_params.yaml`` ``gear_reduction_4wd``。
GEAR_RATIO = 40.0
#: 电机编码器每转计数（2500 线 × 4 倍频）；出处 ``encoder_resolution_4wd``。
ENCODER_COUNTS_PER_MOTOR_REV = 10000.0
#: 里程计标定：417455 count/m，1 count ≈ 2.4 µm。厂家计算式
#: ``(10000 * 40) / (0.305 * pi) = 417455.4``，这里取厂家文档里的取整值。
COUNTS_PER_METER = 417455.0

#: 指令分解轮距 m。出处 ``zeus_params.yaml`` ``wheel_track: 0.61``，
#: 只用于把 /cmd_vel 拆成左右轮速。
WHEEL_TRACK_CMD_M = 0.61

#: 里程计轮距 m。出处 ``zeus_params.yaml`` ``wheel_track_4wd: 1.35``。
#: 这是厂家用角度校准 launch 标出来的滑移补偿值（约几何轮距 0.594 的 2.27 倍），
#: 不能拿它做指令分解，否则原地转会被放大 2 倍多。
ODOM_WHEEL_TRACK_M = 1.35

#: 转矩输出钳位；出处 ``zeus_s2_driver.h`` ``MAX = 1910.0 / MIN = -1910.0``。
#: 数值与额定 1.91 N·m 吻合，但源码没写单位；本包不主动使用转矩模式。
MAX_TORQUE_RAW = 1910

#: 转速给定值的 16 位定点比例：1 LSB = 1 rpm。
#:
#: **证据边界（必须如实写明）**：厂家 ``Set_speed_value_to_motor(uint16_t value)``
#: 只把 value 原样拆成 ``data[4..5]``，而唯一调用点（``mode_switching_state_machine``
#: 的 DRIVER_PARKING_MODE 分支）传的是 0。因此**源码不能证明**这个 16 位值到底是
#: rpm 整数还是 0.1 rpm 定点。这里按“与 0xCB 反馈转速同单位（rpm）”实现，
#: 理由是同一条反馈链上 ``motorActSpeed`` 直接按 rpm 参与 ``/40*πD/60`` 换算。
#: 真机首次上电必须用小转速核对；若实测只有预期的 1/10，把本常量改成 0.1 即可，
#: 其余代码不用动。未验证，详见 docs/can_driver.md。
SPEED_VALUE_RPM_PER_LSB = 1.0


class StartupError(RuntimeError):
    """启动握手失败（应答超时或缺少驱动器）。

    重要性等级：A。节点收到它必须拒绝发速度，不能继续运行。
    """


@dataclass(frozen=True)
class DriverFeedback:
    """一条驱动器反馈帧的解析结果。

    属性：
        driver: 驱动器号 1..4（0x601..0x604 对应 1..4）。
        command: 反馈命令字，``0xCA`` 或 ``0xCB``。
        encoder_counts: 0xCA 帧的 int32 编码器计数（原始值，不取负）。
        current_raw: 0xCA 帧的电流原始值，厂家注释为“千分单位”。
        temperature_c: 0xCB 帧的温度（摄氏度整数）。
        speed_rpm: 0xCB 帧的实际转速，**已按源码取负**（源码
            ``motorActSpeed = -(...)``），单位 rpm。
        fault_code: 0xCB 帧的 16 位故障码，0 表示无故障。

    重要性等级：A。字段含义直接决定控制与里程计符号。
    """

    driver: int
    command: int
    encoder_counts: int | None = None
    current_raw: int | None = None
    temperature_c: int | None = None
    speed_rpm: int | None = None
    fault_code: int | None = None


@dataclass(frozen=True)
class ModeAck:
    """0x700 模式/使能应答。

    属性：
        driver: 驱动器号 1..4（取自 ``data[0]``）。
        kind: ``"enable"`` 或 ``"mode"``。

    重要性等级：A。启动序列用它判断四个驱动器是否真的就绪。
    """

    driver: int
    kind: str


def _finite(value: float, name: str) -> float:
    """把输入转成有限浮点数，非有限值抛 ``ValueError``。

    参数：
        value: 待检查值。
        name: 出错信息里的字段名。
    返回：
        float(value)。
    异常：
        ValueError: 输入不是有限实数（NaN/±inf）。
    重要性等级：A（NaN 速度指令会让驱动器行为不可预测）。
    """
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值，收到 {value!r}")
    return result


def _payload8(data: bytes | bytearray | memoryview) -> bytes:
    """校验并返回 8 字节反馈数据。

    参数：
        data: 反馈帧数据区。
    返回：
        长度为 8 的 bytes。
    异常：
        ValueError: 长度不等于 8。
    重要性等级：A（帧长错误说明协议或接线有问题，不能猜）。
    """
    raw = bytes(data)
    if len(raw) != CAN_FRAME_LEN:
        raise ValueError(f"CAN 帧长度必须为 {CAN_FRAME_LEN} 字节，实际 {len(raw)}")
    return raw


def _encode_i16(value: int) -> bytes:
    """把整数编码成 2 字节大端有符号数。

    参数：
        value: -32768..32767 的整数。
    返回：
        2 字节 bytes。
    异常：
        ValueError: 越界或不是整数。
    重要性等级：A（越界截断会下发错误速度/转矩）。
    """
    if not -32768 <= int(value) <= 32767:
        raise ValueError(f"16 位有符号数越界：{value!r}")
    return int(value).to_bytes(2, "big", signed=True)


def _encode_u16(value: int) -> bytes:
    """把整数编码成 2 字节大端无符号数。

    参数：
        value: 0..65535 的整数。
    返回：
        2 字节 bytes。
    异常：
        ValueError: 越界。
    重要性等级：A（反馈帧的温度/故障码用它）。
    """
    if not 0 <= int(value) <= 0xFFFF:
        raise ValueError(f"16 位无符号数越界：{value!r}")
    return int(value).to_bytes(2, "big", signed=False)


def _int16_wrap(value: int) -> int:
    """把整数按 16 位二补码回绕（模拟 C 的 ``int16_t`` 赋值）。

    参数：
        value: 任意整数，通常来自 ``-raw_speed``。
    返回：
        -32768..32767 的整数。
    重要性等级：A（源码的 ``motorActSpeed`` 是 int16_t，``-(-32768)`` 会回绕成
    -32768；不按 C 语义处理会让 0x8000 这个边界值解出 +32768）。
    """
    return ((int(value) + 0x8000) % 0x10000) - 0x8000


def _encode_i32(value: int) -> bytes:
    """把整数编码成 4 字节大端有符号数。

    参数：
        value: -2^31..2^31-1 的整数。
    返回：
        4 字节 bytes。
    异常：
        ValueError: 越界。
    重要性等级：A（编码器反馈帧用它）。
    """
    if not -0x80000000 <= int(value) <= 0x7FFFFFFF:
        raise ValueError(f"32 位有符号数越界：{value!r}")
    return int(value).to_bytes(4, "big", signed=True)


def _make_command(command: int, index: int, value: int) -> bytes:
    """按厂家的 8 字节命令帧布局拼一帧。

    布局：``[0, command, index_hi, index_lo, value_hi, value_lo, 0, 0]``。
    参数：
        command: data[1] 命令码。
        index: data[2..3] 参数索引。
        value: data[4..5] 16 位有符号值。
    返回：
        8 字节 bytes。
    异常：
        ValueError: 索引或数值越界。
    重要性等级：A。
    """
    if not 0 <= int(index) <= 0xFFFF:
        raise ValueError(f"参数索引越界：{index!r}")
    payload = bytearray(CAN_FRAME_LEN)
    payload[1] = int(command) & 0xFF
    payload[2:4] = int(index).to_bytes(2, "big")
    payload[4:6] = _encode_i16(value)
    return bytes(payload)


def driver_cmd_id(driver: int) -> int:
    """返回第 n 个驱动器的命令帧 CAN ID。

    参数：
        driver: 驱动器号 1..4（左前/右前/右后/左后）。
    返回：
        ``0x101 + (driver - 1)``。
    异常：
        ValueError: driver 不在 1..4。
    重要性等级：A（发错 ID 会把指令发给另一条腿）。
    """
    if not 1 <= int(driver) <= 4:
        raise ValueError(f"驱动器号必须为 1..4，收到 {driver!r}")
    return CMD_ID_BASE + int(driver) - 1


def driver_feedback_id(driver: int) -> int:
    """返回第 n 个驱动器的反馈帧 CAN ID。

    参数：
        driver: 驱动器号 1..4（左前/右前/右后/左后）。
    返回：
        ``0x601 + (driver - 1)``。
    异常：
        ValueError: driver 不在 1..4。
    重要性等级：B（自检与诊断用）。
    """
    if not 1 <= int(driver) <= 4:
        raise ValueError(f"驱动器号必须为 1..4，收到 {driver!r}")
    return FB_ID_BASE + int(driver) - 1


def enable_frame() -> bytes:
    """构造使能帧 ``00 24 00 10 00 01 00 00``。

    返回：
        8 字节命令帧。
    重要性等级：A。
    """
    return _make_command(CMD_ENABLE, PARAM_ENABLE, 1)


def disable_frame() -> bytes:
    """构造失能帧 ``00 1E 00 10 00 00 00 00``。

    返回：
        8 字节命令帧。
    重要性等级：A（关节点必须能可靠下发）。
    """
    return _make_command(CMD_WRITE_PARAM, PARAM_ENABLE, 0)


def clear_alarm_frame() -> bytes:
    """构造清报警帧 ``00 21 00 10 00 00 00 01``。

    注意：厂家源码要求先失能再清报警；本帧最后一个字节固定为 1。
    返回：
        8 字节命令帧。
    重要性等级：A。
    """
    payload = bytearray(_make_command(CMD_CLEAR_ALARM, PARAM_ENABLE, 0))
    payload[7] = 0x01
    return bytes(payload)


def torque_mode_frame() -> bytes:
    """构造“设为转矩模式”帧 ``00 1E 00 03 00 01 00 00``。

    返回：
        8 字节命令帧。
    重要性等级：B（本包默认不用转矩模式）。
    """
    return _make_command(CMD_WRITE_PARAM, PARAM_MODE, MODE_TORQUE)


def speed_mode_frame() -> bytes:
    """构造“设为转速模式”帧 ``00 1E 00 03 00 02 00 00``。

    返回：
        8 字节命令帧。
    重要性等级：A。
    """
    return _make_command(CMD_WRITE_PARAM, PARAM_MODE, MODE_SPEED)


def torque_value_frame(value_raw: int) -> bytes:
    """构造转矩给定帧 ``00 28 00 01 hi lo 00 00``。

    参数：
        value_raw: 16 位原始转矩值；厂家钳位 ±1910。单位未验证，本包不使用。
    返回：
        8 字节命令帧。
    异常：
        ValueError: 越界。
    重要性等级：B。
    """
    return _make_command(CMD_SET_VALUE, PARAM_TORQUE, int(value_raw))


def rpm_to_speed_value(rpm: float) -> int:
    """把转速（rpm）换成速度帧的 16 位定点值并四舍五入。

    参数：
        rpm: 电机转速，单位 rpm。
    返回：
        16 位有符号整数（超出范围时抛 ValueError）。
    异常：
        ValueError: 非有限值或越界。
    重要性等级：A。
    """
    raw = int(round(_finite(rpm, "rpm") / SPEED_VALUE_RPM_PER_LSB))
    if not -32768 <= raw <= 32767:
        raise ValueError(f"转速给定 {rpm!r} rpm 超出 16 位定点范围")
    return raw


def speed_value_to_rpm(value_raw: int) -> float:
    """把速度帧的 16 位定点值还原成 rpm。

    参数：
        value_raw: ``data[4..5]`` 解出的有符号整数。
    返回：
        转速，单位 rpm。
    重要性等级：A。
    """
    return int(value_raw) * SPEED_VALUE_RPM_PER_LSB


def speed_value_frame(rpm: float) -> bytes:
    """构造转速给定帧 ``00 28 00 00 hi lo 00 00``。

    参数：
        rpm: 电机目标转速，单位 rpm；内部按 ``SPEED_VALUE_RPM_PER_LSB`` 定点化。
    返回：
        8 字节命令帧。
    异常：
        ValueError: 非有限值或超出 16 位定点范围。
    重要性等级：A。
    """
    return _make_command(CMD_SET_VALUE, PARAM_SPEED, rpm_to_speed_value(rpm))


def rpm_to_mps(rpm: float) -> float:
    """电机转速 → 车轮线速度。

    公式（出处 ``zeus_s2_driver.cpp`` 各轮 ``velocity`` 行）：
    ``rpm / 40 * π * 0.305 / 60``。
    参数：
        rpm: 电机转速，单位 rpm。
    返回：
        车轮线速度，单位 m/s。
    异常：
        ValueError: 非有限值。
    重要性等级：A。
    """
    value = _finite(rpm, "rpm")
    return value / GEAR_RATIO * math.pi * WHEEL_DIAMETER_M / 60.0


def mps_to_rpm(mps: float) -> float:
    """车轮线速度 → 电机转速（``rpm_to_mps`` 的逆）。

    参数：
        mps: 车轮线速度，单位 m/s。
    返回：
        电机转速，单位 rpm。
    异常：
        ValueError: 非有限值。
    重要性等级：A。
    """
    value = _finite(mps, "mps")
    return value * 60.0 * GEAR_RATIO / (math.pi * WHEEL_DIAMETER_M)


def counts_to_m(counts: float) -> float:
    """编码器计数 → 车轮行驶距离。

    参数：
        counts: 有符号计数差。
    返回：
        行驶距离，单位 m。
    异常：
        ValueError: 非有限值。
    重要性等级：A。
    """
    return _finite(counts, "counts") / COUNTS_PER_METER


def m_to_counts(distance: float) -> float:
    """车轮行驶距离 → 编码器计数（``counts_to_m`` 的逆）。

    参数：
        distance: 行驶距离 m。
    返回：
        有符号计数。
    异常：
        ValueError: 非有限值。
    重要性等级：B（标定与自检用）。
    """
    return _finite(distance, "distance") * COUNTS_PER_METER


def parse_feedback(can_id: int, data: bytes | bytearray | memoryview) -> DriverFeedback | None:
    """解析 0x601..0x604 的反馈帧。

    参数：
        can_id: CAN ID；不在 0x601..0x604 时返回 None。
        data: 8 字节数据区。
    返回：
        ``DriverFeedback``；未知 ``data[1]`` 返回 None。
    异常：
        ValueError: 帧长度不是 8（不能拿噪声当反馈）。
    副作用：
        无。
    重要性等级：A（里程计、故障保护都依赖它）。
    """
    if not FB_ID_BASE <= int(can_id) <= FB_ID_BASE + 3:
        return None
    raw = _payload8(data)
    driver = int(can_id) - FB_ID_BASE + 1
    command = raw[1]

    if command == FB_ENCODER:
        return DriverFeedback(
            driver=driver,
            command=command,
            encoder_counts=int.from_bytes(raw[2:6], "big", signed=True),
            current_raw=int.from_bytes(raw[6:8], "big", signed=False),
        )
    if command == FB_STATUS:
        raw_speed = int.from_bytes(raw[4:6], "big", signed=True)
        return DriverFeedback(
            driver=driver,
            command=command,
            temperature_c=int.from_bytes(raw[2:4], "big", signed=False),
            # 与厂家 updateParam() 一致：反馈转速取负号，并按 int16_t 回绕。
            speed_rpm=_int16_wrap(-raw_speed),
            fault_code=int.from_bytes(raw[6:8], "big", signed=False),
        )
    return None


def parse_mode_ack(can_id: int, data: bytes | bytearray | memoryview) -> ModeAck | None:
    """解析 0x700 的使能/模式应答。

    参数：
        can_id: CAN ID；不是 0x700 时返回 None。
        data: 8 字节数据区。
    返回：
        ``ModeAck``；不是已知应答时返回 None。
    异常：
        ValueError: 帧长度不是 8。
    重要性等级：A（启动序列用它判断驱动器是否真的切到转速模式）。
    """
    if int(can_id) != MODE_ACK_ID:
        return None
    raw = _payload8(data)
    driver = raw[0]
    if not 1 <= driver <= 4:
        return None
    if (
        raw[1] == ACK_ENABLE_WORD
        and raw[3] == ACK_ENABLE_INDEX_LOW
        and raw[5] == ACK_ENABLE_VALUE
    ):
        return ModeAck(driver=driver, kind="enable")
    if raw[1] == ACK_MODE_WORD:
        return ModeAck(driver=driver, kind="mode")
    return None


def encode_encoder_feedback(driver: int, encoder_counts: int, current_raw: int = 0) -> bytes:
    """构造 0xCA 编码器反馈帧（供回环后端与自检使用）。

    参数：
        driver: 1..4。
        encoder_counts: int32 有符号计数。
        current_raw: 16 位无符号电流原始值。
    返回：
        8 字节帧。
    异常：
        ValueError: 驱动器号或数值越界。
    重要性等级：B。
    """
    driver_cmd_id(driver)  # 复用 1..4 校验，报错信息一致
    payload = bytearray(CAN_FRAME_LEN)
    payload[1] = FB_ENCODER
    payload[2:6] = _encode_i32(encoder_counts)
    payload[6:8] = _encode_u16(current_raw)
    return bytes(payload)


def encode_status_feedback(
    driver: int,
    temperature_c: int,
    speed_rpm_raw: int,
    fault_code: int = 0,
) -> bytes:
    """构造 0xCB 温度/转速/故障反馈帧（供回环后端与自检使用）。

    参数：
        driver: 1..4。
        temperature_c: 温度，写入 16 位无符号。
        speed_rpm_raw: 反馈里的**原始** int16 转速（未取负；解析时会取负）。
        fault_code: 16 位故障码。
    返回：
        8 字节帧。
    异常：
        ValueError: 驱动器号或数值越界。
    重要性等级：B。
    """
    driver_cmd_id(driver)
    payload = bytearray(CAN_FRAME_LEN)
    payload[1] = FB_STATUS
    payload[2:4] = _encode_u16(temperature_c)
    payload[4:6] = _encode_i16(speed_rpm_raw)
    payload[6:8] = _encode_u16(fault_code)
    return bytes(payload)


def _broadcast(backend, frame: bytes) -> None:
    """把同一帧发给 1..4 号驱动器。

    参数：
        backend: 具备 ``send(can_id, data)`` 的传输后端。
        frame: 8 字节命令帧。
    副作用：
        发送 4 帧；``send`` 抛出的异常原样向上抛，不吞。
    重要性等级：A。
    """
    for driver in (1, 2, 3, 4):
        backend.send(driver_cmd_id(driver), frame)


def _wait_for_acks(backend, kind: str, timeout_s: float) -> list[int]:
    """等待 1..4 号驱动器的指定应答，返回缺失的驱动器号。

    参数：
        backend: 具备 ``recv(timeout)`` 的传输后端。
        kind: ``"enable"`` 或 ``"mode"``。
        timeout_s: 总超时秒数。
    返回：
        缺失的驱动器号列表（空列表表示四个都收到）。
    副作用：
        会消费后端里的其他帧（反馈帧在启动阶段被忽略）。
    重要性等级：A。
    """
    seen: set[int] = set()
    deadline = time.monotonic() + timeout_s
    while len(seen) < 4:
        remain = deadline - time.monotonic()
        if remain <= 0.0:
            break
        frame = backend.recv(min(0.05, remain))
        if frame is None:
            # 回环后端立即返回 None；让出 CPU，避免启动失败时忙等。
            time.sleep(min(0.001, max(0.0, remain)))
            continue
        ack = parse_mode_ack(*frame)
        if ack is not None and ack.kind == kind:
            seen.add(ack.driver)
    return sorted(set(range(1, 5)) - seen)


def run_startup_sequence(
    backend,
    timeout_s: float = 5.0,
    log: Callable[[str], None] | None = None,
) -> None:
    """执行厂家顺序的启动握手：清报警 → 使能 ×4 → 转速模式 ×4 → 零速。

    参数：
        backend: 传输后端（``CanBackend`` 的任意实现）。
        timeout_s: 每一步等待四个驱动器应答的总超时。
        log: 可选错误日志回调；超时会先调用它再抛异常。
    返回：
        无。
    异常：
        ValueError: timeout_s 非正。
        StartupError: 使能或模式应答超时（信息里列出缺失驱动器号）。
        CanError: 底层发送/接收失败（由 backend 抛出，原样向上传播）。
    副作用：
        向 CAN 总线写入 16 帧命令，并消费启动阶段的应答/反馈帧。
    重要性等级：A（顺序错了驱动器可能不进转速模式；失败必须拒绝运动）。
    """
    timeout = _finite(timeout_s, "timeout_s")
    if timeout <= 0.0:
        raise ValueError("启动超时时间必须大于 0")

    _broadcast(backend, clear_alarm_frame())

    _broadcast(backend, enable_frame())
    missing = _wait_for_acks(backend, "enable", timeout)
    if missing:
        message = (
            f"使能应答超时：未收到驱动器 {missing} 的应答（超时 {timeout:.1f} s）；"
            "请检查 CAN 接线、驱动器供电与 ID"
        )
        if log is not None:
            log(message)
        raise StartupError(message)

    _broadcast(backend, speed_mode_frame())
    missing = _wait_for_acks(backend, "mode", timeout)
    if missing:
        message = (
            f"转速模式应答超时：未收到驱动器 {missing} 的应答（超时 {timeout:.1f} s）；"
            "驱动器可能仍处于转矩模式"
        )
        if log is not None:
            log(message)
        raise StartupError(message)

    # 进入转速模式后先把目标清零，之后才允许上层 /cmd_vel 生效。
    _broadcast(backend, speed_value_frame(0.0))
