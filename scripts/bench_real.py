"""真机首跑自检：一条命令走完底盘与传感器的基本项。

每步打印「期望 / 实测 / 合格线 / PASS-FAIL-SKIP」，最后落一份 CSV。
**不需要车也能干跑**：`--backend loopback` 用 mm_base 的回环后端把脚本自身逻辑验一遍
（它模拟四个驱动器的应答、编码器与反馈帧）。

用法：
    # Windows 干跑（不需要 ROS、不需要 CAN 硬件）
    python scripts\\bench_real.py --backend loopback
    # Ubuntu 真机（轮子必须先架空！）
    sudo python3 scripts/bench_real.py --backend socketcan --can can0
    # 已经用卷尺量过直行距离，直接填进来复核轮径
    sudo python3 scripts/bench_real.py --backend socketcan --measured 1.982

合格线出处：docs/real_machine_calibration.md 的公差表（里程计 0.5%）。
判定口径：PASS 有硬合格线；SKIP 说明缺什么（如 ROS 话题或 UR 驱动未就绪）；
记录项（死区、反馈率）只打印实测值，不给 PASS/FAIL，避免为了"通过"去调数。
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import pathlib
import shutil
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).absolute().parent.parent
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "mm_base"))

from mm_base import backend as mm_backend  # noqa: E402
from mm_base import protocol as mm_proto  # noqa: E402

# ── 合格线（都从下游判据倒推，见 docs/real_machine_calibration.md）────────────
ODOM_TOLERANCE = 0.005      # 里程计/轮径：19 m 行程 0.5% = 9.5 cm，航点容差 0.12 m
FEEDBACK_MIN_HZ = 20.0      # 反馈流只要还在动就行（厂家 125 Hz 环，实测常掉到几十 Hz）
WHEEL_CMD_MPS = 0.30        # 方向/里程计测试用的低速（架空时安全）
DEADBAND_STEPS = (0.05, 0.10, 0.20)   # 起转门限：这三个速度各给 2 s，看编码器有没有动

STATUS_PASS, STATUS_FAIL, STATUS_SKIP = "PASS", "FAIL", "SKIP"


class Step:
    """一项检查的结果。"""

    def __init__(self, key: str, name: str, status: str, expected: str = "-",
                 measured: str = "-", limit: str = "-", note: str = "") -> None:
        self.key = key
        self.name = name
        self.status = status
        self.expected = expected
        self.measured = measured
        self.limit = limit
        self.note = note


class Rig:
    """对 CAN 后端的一层薄封装：发速度、收反馈、读编码器。

    loopback 后端自己维护编码器，直接读 `encoder_counts()`；
    socketcan 后端只能从 0x601~0x604 的 0xCA 反馈帧里解，所以这里统一缓存一份。
    """

    def __init__(self, backend, real: bool) -> None:
        self.backend = backend
        self.real = real
        self._counts: dict[int, int] = {}
        self._frames = 0

    def send_speed(self, mps: float) -> None:
        frame = mm_proto.speed_value_frame(mm_proto.mps_to_rpm(mps))
        for driver in range(1, 5):
            self.backend.send(mm_proto.driver_cmd_id(driver), frame)

    def _drain(self) -> None:
        while True:
            item = self.backend.recv(timeout=0.0)
            if item is None:
                return
            can_id, data = item
            fb = mm_proto.parse_feedback(can_id, data)
            if fb is None or fb.encoder_counts is None:
                continue
            self._counts[fb.driver] = fb.encoder_counts
            self._frames += 1

    def counts(self) -> dict[int, int]:
        if not self.real:
            return {d: self.backend.encoder_counts(d) for d in range(1, 5)}
        return dict(self._counts)

    def pump(self, seconds: float, mps: float | None) -> None:
        """保持 `seconds` 秒的运动：每 20 ms 发一帧速度并收反馈。"""
        if not self.real:
            steps = max(1, int(round(seconds / 0.02)))
            for _ in range(steps):
                if mps is not None:
                    self.send_speed(mps)
                self.backend.advance(0.02)
                self._drain()
            return
        t0 = time.monotonic()
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= seconds:
                break
            if mps is not None:
                self.send_speed(mps)
            self._drain()
            time.sleep(0.01)

    def frames(self) -> int:
        return self._frames


def _send_all(backend, frame: bytes) -> None:
    for driver in range(1, 5):
        backend.send(mm_proto.driver_cmd_id(driver), frame)


def _steps(rig: Rig, args) -> list[Step]:
    results: list[Step] = []

    # 1) CAN 接口（只在真机模式有意义）
    if not rig.real:
        results.append(Step("can_iface", "CAN 接口与位速率", STATUS_SKIP,
                            "UP / 1000 kbps", "干跑模式", "-",
                            "真机用 --backend socketcan --can can0"))
    else:
        name = args.can
        try:
            out = subprocess.run(
                ["ip", "-details", "link", "show", name],
                capture_output=True, text=True, timeout=5,
            )
            text = out.stdout
            up = "state UP" in text or "state UNKNOWN" in text
            bitrate = "bitrate 1000000" in text
            ok = out.returncode == 0 and up and bitrate
            detail = "UP" if up else "DOWN"
            detail += " / 1000 kbps" if bitrate else " / 位速率不是 1000 kbps"
            results.append(Step("can_iface", "CAN 接口与位速率", STATUS_PASS if ok else STATUS_FAIL,
                                "UP / 1000 kbps", detail, "UP 且 1000 kbps",
                                "" if ok else "用 `sudo ip link set can0 up type can bitrate 1000000`"))
        except (OSError, subprocess.SubprocessError) as exc:
            results.append(Step("can_iface", "CAN 接口与位速率", STATUS_FAIL,
                                "UP / 1000 kbps", f"检查失败：{exc}", "UP 且 1000 kbps"))

    # 2) 启动握手：清报警 → 使能 ×4 → 转速模式 ×4 → 零速
    log: list[str] = []
    try:
        _send_all(rig.backend, mm_proto.clear_alarm_frame())
        rig.backend.recv(timeout=args.ack_timeout)   # 消费可能存在的应答
        mm_proto.run_startup_sequence(rig.backend, timeout_s=args.ack_timeout,
                                      log=log.append)
        results.append(Step("startup", "启动握手（清报警/使能/转速模式）", STATUS_PASS,
                            "4/4 驱动器应答", "4/4 应答", "4/4",
                            "顺序固定，先清报警再使能"))
    except (mm_proto.StartupError, mm_backend.CanError) as exc:
        results.append(Step("startup", "启动握手（清报警/使能/转速模式）", STATUS_FAIL,
                            "4/4 驱动器应答", str(exc)[:60], "4/4",
                            "查 CAN 接线、驱动器供电与 ID（0x101~0x104）"))

    # 3) 反馈流：1 s 内收到多少帧（记录项）。四路驱动器各发一份，除以 4 才是单轮频率。
    rig._frames = 0
    rig.pump(1.0, 0.0)
    hz = float(rig.frames()) / 4.0
    results.append(Step("feedback", "单轮反馈频率", STATUS_PASS if hz >= FEEDBACK_MIN_HZ else STATUS_FAIL,
                        f"≥ {FEEDBACK_MIN_HZ:.0f} Hz", f"{hz:.0f} Hz",
                        f"≥ {FEEDBACK_MIN_HZ:.0f} Hz",
                        "记录项：本机负载高时会掉，不代表驱动故障"))

    # 4) 四轮方向：给正速度，四个编码器都应变大
    start = rig.counts()
    rig.pump(1.0, WHEEL_CMD_MPS)
    end = rig.counts()
    deltas = {d: end.get(d, 0) - start.get(d, 0) for d in range(1, 5)}
    same_sign = all((v > 0) for v in deltas.values()) or all((v < 0) for v in deltas.values())
    moving = all(abs(v) > 0 for v in deltas.values())
    detail = " ".join(f"D{d}={v:+d}" for d, v in sorted(deltas.items()))
    if not moving:
        results.append(Step("wheel_dir", "四轮方向与转动", STATUS_FAIL,
                            "四轮同向且都在转", detail, "四个增量同号且非零",
                            "有车轮没动：查使能、机械卡死、编码器接线"))
    else:
        results.append(Step("wheel_dir", "四轮方向与转动", STATUS_PASS if same_sign else STATUS_FAIL,
                            "四轮同向", detail, "四个增量同号",
                            "" if same_sign else "有轮子反转：查电机相序或安装方向"))

    # 5) 死区（记录项）：逐档给很小的速度，看编码器有没有动
    marks: list[str] = []
    for mps in DEADBAND_STEPS:
        base = rig.counts()
        rig.pump(2.0, mps)
        moved = {d: rig.counts().get(d, 0) - base.get(d, 0) for d in range(1, 5)}
        turns = sum(1 for v in moved.values() if abs(v) > 0)
        marks.append(f"{mps:.2f} m/s→{turns}/4")
    rig.pump(0.3, 0.0)
    results.append(Step("deadband", "起转门限（记录项）", STATUS_SKIP,
                        "越低越好，无硬线", " / ".join(marks), "-",
                        "四轮都起转说明该速度没有落在死区"))

    # 6) 里程计：直行一段，里程计读数与期望（或卷尺实测）比
    seconds = 2.0
    base = rig.counts()
    rig.pump(seconds, WHEEL_CMD_MPS)
    rig.pump(0.3, 0.0)
    final = rig.counts()
    deltas = [final.get(d, 0) - base.get(d, 0) for d in range(1, 5)]
    counts = sum(deltas) / 4.0
    odom_m = abs(mm_proto.counts_to_m(counts))
    expected = args.measured if args.measured else WHEEL_CMD_MPS * seconds
    rel = abs(odom_m - expected) / max(expected, 1e-6)
    ok = rel <= ODOM_TOLERANCE
    results.append(Step("odom", "里程计直线吻合度", STATUS_PASS if ok else STATUS_FAIL,
                        f"{expected:.3f} m", f"{odom_m:.3f} m（偏差 {rel * 100:.2f}%）",
                        f"≤ {ODOM_TOLERANCE * 100:.1f}%",
                        "真机请用卷尺量实际距离并传 --measured；偏差大就按说明书做直线校准"))

    # 7)(8) 传感器与机械臂：没有 ROS 环境就整体跳过
    ros2 = shutil.which("ros2")
    if ros2 is None:
        for key, name, why in (
            ("lidar", "雷达 /scan", "本机没有 ros2 命令"),
            ("arm", "机械臂 /joint_states", "本机没有 ros2 命令"),
        ):
            results.append(Step(key, name, STATUS_SKIP, "有数据且频率正常", f"跳过：{why}", "-"))
    else:
        results.append(_topic_step(ros2, "lidar", "/scan", "雷达 /scan", "LaserScan"))
        results.append(_topic_step(ros2, "arm", "/joint_states", "机械臂 /joint_states", "JointState"))

    return results


def _topic_step(ros2: str, key: str, topic: str, name: str, kind: str) -> Step:
    """探测一个话题是否存在、是否真的在发（用 ros2 CLI，不依赖 rclpy）。"""
    try:
        listing = subprocess.run([ros2, "topic", "list"], capture_output=True,
                                 text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        return Step(key, name, STATUS_SKIP, "话题在发", f"ros2 CLI 失败：{exc}", "-")
    if topic not in listing.split():
        return Step(key, name, STATUS_SKIP, "话题在发", f"没有 {topic}", "-",
                    f"驱动未启动时属正常（{kind}）")
    try:
        hz_out = subprocess.run(
            [ros2, "topic", "hz", "--window", "10", topic],
            capture_output=True, text=True, timeout=15,
        ).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        return Step(key, name, STATUS_SKIP, "话题在发", f"测频率失败：{exc}", "-")
    numbers = [ln for ln in hz_out.splitlines() if "average rate" in ln]
    if not numbers:
        return Step(key, name, STATUS_FAIL, "有数据", "没有收到消息", "频率 > 0 Hz",
                    "话题在但没数据：查驱动与时间同步")
    return Step(key, name, STATUS_PASS, "频率 > 0 Hz", numbers[0].strip().split(":")[-1].strip(),
                "> 0 Hz")


def _print_table(results: list[Step]) -> None:
    width = max(len(step.name) for step in results) + 2
    for i, step in enumerate(results, 1):
        print(f"[{i}/{len(results)}] {step.name:<{width}} {step.status:<4} "
              f"期望 {step.expected:<22} 实测 {step.measured}")
        if step.status != STATUS_PASS and step.note:
            print(f"       └─ {step.note}")
        elif step.limit != "-" and step.status == STATUS_PASS:
            print(f"       └─ 合格线 {step.limit}")


def _write_csv(results: list[Step]) -> pathlib.Path:
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = ROOT / "reports" / f"bench_real_{stamp}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["step", "name", "status", "expected", "measured", "limit", "note"])
        for step in results:
            writer.writerow([step.key, step.name, step.status, step.expected,
                             step.measured, step.limit, step.note])
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="真机首跑自检（底盘 + 传感器）")
    parser.add_argument("--backend", choices=("loopback", "socketcan"), default="loopback")
    parser.add_argument("--can", default="can0", help="SocketCAN 接口名")
    parser.add_argument("--measured", type=float, default=None,
                        help="直行测试的实际距离（米，卷尺量）；不给就用命令速度×时间")
    parser.add_argument("--ack-timeout", type=float, default=5.0, help="启动握手应答超时")
    parser.add_argument("--yes", action="store_true", help="跳过真机前的安全确认")
    args = parser.parse_args()

    real = args.backend == "socketcan"
    if real:
        print("=== 真机模式：先确认三件事 ===")
        print("  1) 四个轮子都已经架空（离地）；")
        print("  2) 车周围一米内没有人；")
        print("  3) 急停开关在手边。")
        if not args.yes:
            if input("确认无误请输入 yes 继续：").strip().lower() != "yes":
                print("已取消。")
                return 2
        backend = mm_backend.SocketCanBackend(args.can)
    else:
        print("=== 干跑模式：用回环后端验脚本自身逻辑，不需要车 ===")
        backend = mm_backend.LoopbackBackend()

    rig = Rig(backend, real)
    try:
        backend.__enter__()
        results = _steps(rig, args)
        # 收尾：先零速再失能，别把车留在带速度状态
        try:
            _send_all(backend, mm_proto.speed_value_frame(0.0))
            _send_all(backend, mm_proto.disable_frame())
        except mm_backend.CanError:
            pass
    finally:
        backend.close()

    print()
    _print_table(results)
    out = _write_csv(results)
    passed = sum(1 for r in results if r.status == STATUS_PASS)
    failed = sum(1 for r in results if r.status == STATUS_FAIL)
    skipped = sum(1 for r in results if r.status == STATUS_SKIP)
    print(f"\n结论：{passed} PASS / {failed} FAIL / {skipped} SKIP   → {out}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
