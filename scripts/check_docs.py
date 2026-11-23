"""文档一致性断言：检查文档里最容易过时的三条事实是否与代码一致。

用法：
    python scripts\\check_docs.py

退出码：0 = 全部一致；1 = 有偏差（逐条打印「文档写的 / 代码实际的」）。

设计原则（重要）：
    **断言里不存数字**。每条检查都从 `mission_core` 的常量或 `ros2_ws/src` 的真实目录
    现取真值，再去文档里核对。所以改数据时只需要改「代码」和「文档」两处，本脚本
    永远不用动——它不是第三份副本，只是一次一致性比对。

只查三条，因为这三类历史上真的漂过（换夹爪后夹取高度、降速后速度档、加 mm_base 后包数）。
倾角范围、桶内余量这类**实测结果**不查：它们每次跑都可能不同，断言只会制造假警报。
"""
from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).absolute().parent.parent
sys.path.insert(0, str(ROOT))

from mission_core import (  # noqa: E402
    APPROACH_SPEED,
    CRUISE_SPEED,
    PINCH_ABOVE_FLOOR,
    REVERSE_SPEED,
)

CN_DIGITS = "零一二三四五六七八九十"


def _read(name: str) -> str:
    return (ROOT / name).read_text(encoding="utf-8")


def check_grasp_height() -> tuple[bool, str]:
    """README 里的离地夹持点高度必须等于 PINCH_ABOVE_FLOOR。"""
    text = _read("README.md")
    want = f"{PINCH_ABOVE_FLOOR * 1000:.1f} mm"
    if want not in text:
        return False, f"README 里找不到「{want}」（代码 PINCH_ABOVE_FLOOR={PINCH_ABOVE_FLOOR}）"
    if "18.5 mm" in text:
        return False, "README 里还留着旧值「18.5 mm」（那是 2F-85 时代的）"
    return True, f"README = {want}，与代码一致"


def check_speeds() -> tuple[bool, str]:
    """README 的三档速度必须等于 CRUISE/APPROACH/REVERSE_SPEED。"""
    text = _read("README.md")
    pairs = (
        ("巡航", CRUISE_SPEED),
        ("持杆/接近", APPROACH_SPEED),
        ("倒车", REVERSE_SPEED),
    )
    for label, value in pairs:
        want = f"{label} `{value:.2f}"
        if want not in text:
            return False, f"README 里找不到「{want}」（代码 {value:.2f} m/s）"
    return True, f"{CRUISE_SPEED:.2f} / {APPROACH_SPEED:.2f} / {REVERSE_SPEED:.2f} m/s，与代码一致"


def check_ros_packages() -> tuple[bool, str]:
    """README 与 ARCHITECTURE 里写的 ROS 包数必须等于 ros2_ws/src 下的真实目录数。"""
    src = ROOT / "ros2_ws" / "src"
    packages = sorted(p.name for p in src.iterdir() if p.is_dir())
    count = len(packages)
    chinese = CN_DIGITS[count] if 0 <= count <= 10 else str(count)
    want = f"ROS 2 {chinese}包"
    for doc in ("README.md", "ARCHITECTURE.md"):
        if want not in _read(doc):
            return False, f"{doc} 里找不到「{want}」（实际 {count} 个：{'、'.join(packages)}）"
    return True, f"{count} 个包（{'、'.join(packages)}），与文档一致"


CHECKS = (
    ("夹取高度", check_grasp_height),
    ("速度三档", check_speeds),
    ("ROS 包数", check_ros_packages),
)


def main() -> int:
    failures = 0
    for index, (name, fn) in enumerate(CHECKS, 1):
        try:
            ok, detail = fn()
        except (OSError, KeyError) as exc:      # 文件缺失 / 目录结构变了
            ok, detail = False, f"检查执行失败：{exc}"
        mark = "PASS" if ok else "FAIL"
        print(f"[{index}/{len(CHECKS)}] {name:<8} {mark}  {detail}")
        failures += 0 if ok else 1
    print()
    if failures:
        print(f"结论：{failures} 条文档已落后于代码——改文档，不要改这个脚本。")
        return 1
    print("结论：3 条一致。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
