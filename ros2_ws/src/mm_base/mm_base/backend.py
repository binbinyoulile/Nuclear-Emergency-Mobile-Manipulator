"""mm_base.backend：CAN 传输层（SocketCAN / 回环 / 录制）。

三个后端
--------
- ``SocketCanBackend``：Linux ``AF_CAN`` 裸套接字，只用标准库；Windows 上**可以
  import**，只有实例化时才抛 ``BackendUnavailable``（``socket.AF_CAN`` 在函数内取）。
- ``LoopbackBackend``：不接硬件的假驱动器。收到命令就更新内部状态，按 100 Hz 产生
  0xCA/0xCB 反馈；未使能、非转速模式或故障时**不积分运动**。
- ``RecordingBackend``：把发出去的帧逐条记下来，可套在其它后端外面，供自检断言。

并发
----
三个后端都**不是线程安全的**。``mm_base.base_node`` 用单线程执行器，所有收发都在
定时器回调里完成；如果将来加接收线程，需要自己加锁。

重要性等级：A（传输层失败必须抛异常，不能静默丢帧）。
"""

from __future__ import annotations

import math
import select
import socket
import struct
from abc import ABC, abstractmethod
from collections import deque

from mm_base.protocol import (
    ACK_ENABLE_INDEX_LOW,
    ACK_ENABLE_VALUE,
    ACK_ENABLE_WORD,
    ACK_MODE_WORD,
    CAN_FRAME_LEN,
    CMD_CLEAR_ALARM,
    CMD_ENABLE,
    CMD_ID_BASE,
    CMD_SET_VALUE,
    CMD_WRITE_PARAM,
    FB_ID_BASE,
    MODE_ACK_ID,
    MODE_SPEED,
    MODE_TORQUE,
    PARAM_ENABLE,
    PARAM_MODE,
    PARAM_SPEED,
    PARAM_TORQUE,
    ENCODER_COUNTS_PER_MOTOR_REV,
    encode_encoder_feedback,
    encode_status_feedback,
    speed_value_to_rpm,
)


class CanError(RuntimeError):
    """CAN 传输失败（打开、发送、接收、短帧）。

    重要性等级：A。节点不能吞掉它继续发速度。
    """


class BackendUnavailable(CanError):
    """当前平台/接口不支持该后端（典型：Windows 上用 SocketCAN）。

    重要性等级：B。自检用它证明 Windows 上只是“不可用”，不是 import 崩溃。
    """


class CanBackend(ABC):
    """CAN 传输后端抽象基类。

    重要性等级：A（节点只依赖这层接口，换 USB-CAN 适配器时不动上层）。
    """

    @abstractmethod
    def send(self, can_id: int, data: bytes) -> None:
        """发送一帧标准 CAN 帧。

        参数：
            can_id: 11 位标准帧 ID。
            data: 0..8 字节数据区。
        异常：
            CanError: 底层发送失败。
            ValueError: ID 或长度非法。
        重要性等级：A。
        """

    @abstractmethod
    def recv(self, timeout: float | None = None) -> tuple[int, bytes] | None:
        """接收一帧。

        参数：
            timeout: 最长等待秒数；0 表示非阻塞，None 表示一直等。
        返回：
            ``(can_id, data)``；超时返回 None。
        异常：
            CanError: 底层接收失败。
        重要性等级：A。
        """

    @abstractmethod
    def close(self) -> None:
        """关闭后端；必须可重复调用。

        副作用：
            释放设备/套接字；调用后 ``send()`` 应抛 ``CanError``。
        重要性等级：A（异常退出时也要能走到这里）。
        """

    def advance(self, dt: float) -> None:
        """推进后端内部时钟（仅回环后端需要；真实后端为空操作）。

        参数：
            dt: 距上次调用的秒数。
        重要性等级：B。
        """

    def __enter__(self) -> "CanBackend":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class SocketCanBackend(CanBackend):
    """Linux SocketCAN 后端（只用标准库 ``socket``，不依赖 python-can）。

    参数：
        interface: CAN 接口名，默认 ``can0``。
    异常：
        BackendUnavailable: 非 Linux、无 ``AF_CAN``、接口不存在或被占用。
    副作用：
        打开并绑定一个 ``AF_CAN/SOCK_RAW`` 套接字；``close()`` 释放。
    重要性等级：A。
    """

    def __init__(self, interface: str = "can0") -> None:
        if not interface:
            raise ValueError("CAN 接口名不能为空")
        # 必须在函数内部取 AF_CAN：Windows 的 socket 模块没有这个常量，
        # 放到模块顶层会让 import 直接炸掉（自检就没法在 Windows 跑）。
        af_can = getattr(socket, "AF_CAN", None)
        can_raw = getattr(socket, "CAN_RAW", 1)
        if af_can is None:
            raise BackendUnavailable(
                "当前平台没有 socket.AF_CAN；SocketCAN 只能在 Linux 上使用"
            )

        sock = None
        try:
            sock = socket.socket(af_can, socket.SOCK_RAW, can_raw)
            sock.bind((interface,))
        except OSError as exc:
            if sock is not None:
                sock.close()
            raise BackendUnavailable(
                f"打开 SocketCAN 接口 {interface!r} 失败：{exc}；"
                f"先执行 `ip link set {interface} up type can bitrate 1000000`"
            ) from exc
        self._sock: socket.socket | None = sock
        self.interface = interface

    def send(self, can_id: int, data: bytes) -> None:
        """发送标准 CAN 帧；失败抛 ``CanError``，不静默。

        参数：
            can_id: 11 位标准帧 ID。
            data: 0..8 字节。
        异常：
            ValueError: ID/长度非法。
            CanError: 套接字已关闭、发送失败或短写。
        重要性等级：A。
        """
        sock = self._require_open()
        if not 0 < int(can_id) <= 0x7FF:
            raise ValueError(f"标准帧 CAN ID 必须在 1..0x7FF，收到 {can_id!r}")
        raw = bytes(data)
        if len(raw) > CAN_FRAME_LEN:
            raise ValueError(f"CAN 数据区最多 {CAN_FRAME_LEN} 字节，收到 {len(raw)}")
        frame = struct.pack("=IB3x8s", int(can_id), len(raw), raw.ljust(CAN_FRAME_LEN, b"\0"))
        try:
            sent = sock.send(frame)
        except OSError as exc:
            raise CanError(f"CAN 发送失败 id=0x{int(can_id):X}: {exc}") from exc
        if sent != len(frame):
            raise CanError(f"CAN 短写：期望 {len(frame)} 字节，实际 {sent}")

    def recv(self, timeout: float | None = None) -> tuple[int, bytes] | None:
        """接收一帧；超时返回 None。

        参数：
            timeout: 秒；0 非阻塞，None 一直等。
        异常：
            ValueError: timeout 为负。
            CanError: 套接字错误或短帧。
        重要性等级：A。
        """
        sock = self._require_open()
        if timeout is not None:
            timeout = float(timeout)
            if timeout < 0.0 or not math.isfinite(timeout):
                raise ValueError(f"timeout 必须是非负有限值，收到 {timeout!r}")
        try:
            ready, _, _ = select.select([sock], [], [], timeout)
        except OSError as exc:
            raise CanError(f"CAN select 失败：{exc}") from exc
        if not ready:
            return None
        try:
            raw = sock.recv(16)
        except OSError as exc:
            raise CanError(f"CAN 接收失败：{exc}") from exc
        if len(raw) < 16:
            raise CanError(f"CAN 短帧：预期 16 字节 can_frame，收到 {len(raw)}")
        can_id, dlc = struct.unpack("=IB3x", raw[:8])
        if dlc > CAN_FRAME_LEN:
            raise CanError(f"CAN DLC 非法：{dlc}")
        # 去掉 EFF/RTR/ERR 标志，只保留 29 位 ID；节点只认标准帧 ID。
        return can_id & 0x1FFFFFFF, raw[8 : 8 + dlc]

    def close(self) -> None:
        """关闭套接字；可重复调用。

        副作用：
            释放 ``AF_CAN`` 套接字；关闭后 ``send()``/``recv()`` 抛 ``CanError``。
        重要性等级：A。
        """
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _require_open(self) -> socket.socket:
        """返回套接字；已关闭时抛 ``CanError``。"""
        if self._sock is None:
            raise CanError("SocketCAN 后端已关闭")
        return self._sock


class LoopbackBackend(CanBackend):
    """不接硬件的假驱动器，用于 Windows 自检与无 CAN 台架演练。

    行为：
        - 收到使能/失能/清报警/模式/目标值命令，更新内部状态；
        - ``advance(dt)`` 按 100 Hz 产生 0xCA/0xCB 反馈帧；
        - 编码器计数按目标转速积分，温度恒定 25 ℃，故障码 0；
        - **未使能、非转速模式或故障码非零时，速度目标不产生运动**。

    注意：
        ``recv()`` 不做阻塞等待（假总线没有实时语义），超时参数被忽略。
        反馈帧由 ``advance()`` 驱动，而不是墙钟，所以自检结果可复现。
    重要性等级：A（安全断言“未使能不运动”就在这里实现）。
    """

    FEEDBACK_PERIOD_S = 0.01
    TEMPERATURE_C = 25

    def __init__(self) -> None:
        self._enabled = {n: False for n in range(1, 5)}
        # 厂家出厂默认转矩模式；启动序列会显式切到转速模式。
        self._mode = {n: "torque" for n in range(1, 5)}
        self._target_rpm = {n: 0.0 for n in range(1, 5)}
        self._target_torque_raw = {n: 0 for n in range(1, 5)}
        self._counts = {n: 0.0 for n in range(1, 5)}
        self._fault = {n: 0 for n in range(1, 5)}
        self._queue: deque[tuple[int, bytes]] = deque()
        self._emit_accum = 0.0
        self._sim_time = 0.0
        self._closed = False

    # -- 公共查询（自检便利，不参与节点逻辑）------------------------------

    @property
    def sim_time(self) -> float:
        """已推进的仿真时间，单位 s。"""
        return self._sim_time

    def encoder_counts(self, driver: int) -> int:
        """返回某驱动器当前编码器计数（四舍五入到 int）。

        参数：
            driver: 1..4。
        返回：
            当前计数。
        异常：
            ValueError: 驱动器号非法。
        重要性等级：B（自检便利接口，不参与节点控制）。
        """
        self._check_driver(driver)
        return int(round(self._counts[driver]))

    # -- CanBackend 接口 ---------------------------------------------------

    def send(self, can_id: int, data: bytes) -> None:
        """接收一条命令帧并按语义更新假驱动器状态。

        参数：
            can_id: 0x101..0x104。
            data: 8 字节命令帧。
        异常：
            ValueError: ID/长度/命令字/参数索引非法。
            CanError: 后端已关闭。
        重要性等级：A。
        """
        self._require_open()
        if not CMD_ID_BASE <= int(can_id) <= CMD_ID_BASE + 3:
            raise ValueError(f"命令帧 ID 必须为 0x101..0x104，收到 0x{int(can_id):X}")
        raw = bytes(data)
        if len(raw) != CAN_FRAME_LEN:
            raise ValueError(f"命令帧必须为 {CAN_FRAME_LEN} 字节，收到 {len(raw)}")
        driver = int(can_id) - CMD_ID_BASE + 1
        command = raw[1]

        if command == CMD_ENABLE:
            self._enabled[driver] = True
            self._queue_ack(driver, "enable")
            return

        if command == CMD_WRITE_PARAM:
            index = int.from_bytes(raw[2:4], "big")
            value = int.from_bytes(raw[4:6], "big", signed=True)
            if index == PARAM_ENABLE:
                self._enabled[driver] = value != 0
                if not self._enabled[driver]:
                    self._target_rpm[driver] = 0.0
                return
            if index == PARAM_MODE:
                if value == MODE_TORQUE:
                    self._mode[driver] = "torque"
                elif value == MODE_SPEED:
                    self._mode[driver] = "speed"
                else:
                    raise ValueError(f"未知运行模式 {value}（只支持 1=转矩 / 2=转速）")
                self._target_rpm[driver] = 0.0
                self._queue_ack(driver, "mode")
                return
            raise ValueError(f"未知参数索引 0x{index:04X}")

        if command == CMD_CLEAR_ALARM:
            # 厂家要求先失能再清报警；使能状态下这一帧按“无效”处理，不报错。
            if not self._enabled[driver]:
                self._fault[driver] = 0
            return

        if command == CMD_SET_VALUE:
            index = int.from_bytes(raw[2:4], "big")
            value = int.from_bytes(raw[4:6], "big", signed=True)
            if index == PARAM_SPEED:
                self._target_rpm[driver] = speed_value_to_rpm(value)
                return
            if index == PARAM_TORQUE:
                self._target_torque_raw[driver] = value
                return
            raise ValueError(f"未知运行值索引 0x{index:04X}")

        raise ValueError(f"未知命令字 0x{command:02X}")

    def recv(self, timeout: float | None = None) -> tuple[int, bytes] | None:
        """取出队首反馈帧；队列空时立即返回 None。

        参数：
            timeout: 忽略；假总线不阻塞。
        返回：
            ``(can_id, data)`` 或 None。
        重要性等级：B。
        """
        self._require_open()
        if self._queue:
            return self._queue.popleft()
        return None

    def advance(self, dt: float) -> None:
        """推进仿真时间并按 100 Hz 生成反馈帧。

        参数：
            dt: 秒；必须是非负有限值。
        异常：
            ValueError: dt 非法。
            CanError: 后端已关闭。
        副作用：
            更新编码器计数并往接收队列里追加 0xCA/0xCB 帧。
        重要性等级：A（“未使能不运动”在这里生效）。
        """
        self._require_open()
        step_s = float(dt)
        if step_s < 0.0 or not math.isfinite(step_s):
            raise ValueError(f"dt 必须是非负有限值，收到 {dt!r}")

        remaining = step_s
        while remaining > 1e-12:
            step = min(remaining, self.FEEDBACK_PERIOD_S - self._emit_accum)
            self._integrate(step)
            self._sim_time += step
            self._emit_accum += step
            remaining -= step
            if self._emit_accum >= self.FEEDBACK_PERIOD_S - 1e-12:
                self._emit_feedback()
                self._emit_accum = 0.0

    def close(self) -> None:
        """关闭后端并清空队列；可重复调用。

        副作用：
            清空未读反馈；之后 ``send()``/``recv()`` 抛 ``CanError``。
        重要性等级：A。
        """
        self._closed = True
        self._queue.clear()

    # -- 内部 --------------------------------------------------------------

    def _require_open(self) -> None:
        """后端已关闭时抛 ``CanError``。重要性等级：B。"""
        if self._closed:
            raise CanError("回环后端已关闭")

    @staticmethod
    def _check_driver(driver: int) -> None:
        """校验驱动器号 1..4，非法时抛 ``ValueError``。重要性等级：B。"""
        if not 1 <= int(driver) <= 4:
            raise ValueError(f"驱动器号必须为 1..4，收到 {driver!r}")

    def _queue_ack(self, driver: int, kind: str) -> None:
        """把 0x700 使能/模式应答放进接收队列。重要性等级：B。"""
        payload = bytearray(CAN_FRAME_LEN)
        payload[0] = driver
        if kind == "enable":
            payload[1] = ACK_ENABLE_WORD
            payload[3] = ACK_ENABLE_INDEX_LOW
            payload[5] = ACK_ENABLE_VALUE
        elif kind == "mode":
            payload[1] = ACK_MODE_WORD
        else:  # pragma: no cover - 内部调用只有两种
            raise ValueError(f"未知应答类型 {kind!r}")
        self._queue.append((MODE_ACK_ID, bytes(payload)))

    def _integrate(self, dt: float) -> None:
        """按目标转速积分编码器计数。

        参数：
            dt: 本次积分时长 s。
        副作用：
            只对已使能、转速模式、故障码为 0 的驱动器累加计数。
        重要性等级：A（“未使能不运动”在这里实现）。
        """
        for driver in range(1, 5):
            if (
                self._enabled[driver]
                and self._mode[driver] == "speed"
                and self._fault[driver] == 0
            ):
                # 编码器在电机侧：每电机转 10000 count，所以不再乘减速比。
                self._counts[driver] += (
                    self._target_rpm[driver] / 60.0 * ENCODER_COUNTS_PER_MOTOR_REV * dt
                )

    def _emit_feedback(self) -> None:
        """为四个驱动器各生成一条 0xCA + 一条 0xCB 反馈帧。

        副作用：
            往接收队列追加 8 帧；温度恒定、故障码取内部状态。
        重要性等级：B。
        """
        for driver in range(1, 5):
            moving = (
                self._enabled[driver]
                and self._mode[driver] == "speed"
                and self._fault[driver] == 0
            )
            target = self._target_rpm[driver] if moving else 0.0
            # 厂家源码对速度反馈取负号；编码器计数不取负。两者都按同一目标积分，
            # 解析后的 speed_rpm = -raw 正好等于目标值。
            raw_speed = max(-32768, min(32767, int(round(-target))))
            counts = max(-0x80000000, min(0x7FFFFFFF, int(round(self._counts[driver]))))
            self._queue.append(
                (FB_ID_BASE + driver - 1, encode_encoder_feedback(driver, counts))
            )
            self._queue.append(
                (
                    FB_ID_BASE + driver - 1,
                    encode_status_feedback(
                        driver,
                        self.TEMPERATURE_C,
                        raw_speed,
                        self._fault[driver],
                    ),
                )
            )


class RecordingBackend(CanBackend):
    """把发出去的帧记录下来的包装后端。

    参数：
        inner: 可选的真实/回环后端；所有调用转发给它。
    副作用：
        ``send()`` 每帧追加到 ``frames``，格式 ``(can_id, bytes)``。
    重要性等级：B（只用于自检与审计）。
    """

    def __init__(self, inner: CanBackend | None = None) -> None:
        self._inner = inner
        self.frames: list[tuple[int, bytes]] = []

    def send(self, can_id: int, data: bytes) -> None:
        """记录并转发一帧。

        参数：
            can_id: CAN ID。
            data: 数据区。
        副作用：
            追加到 ``frames``；有 inner 时继续发送。
        重要性等级：B。
        """
        frame = (int(can_id), bytes(data))
        self.frames.append(frame)
        if self._inner is not None:
            self._inner.send(*frame)

    def recv(self, timeout: float | None = None) -> tuple[int, bytes] | None:
        """转发接收；没有 inner 时返回 None。重要性等级：B。"""
        if self._inner is None:
            return None
        return self._inner.recv(timeout)

    def advance(self, dt: float) -> None:
        """转发时钟推进。重要性等级：B。"""
        if self._inner is not None:
            self._inner.advance(dt)

    def close(self) -> None:
        """关闭内部后端；``frames`` 保留供断言。重要性等级：B。"""
        if self._inner is not None:
            self._inner.close()
