"""关节与夹爪的平滑插值（纯数学，不依赖 ROS 与 MuJoCo）。

与单进程实现里的 ``ArmMotion.update`` / ``FingerMotion.update`` 同构：
缓入缓出用 smoothstep，一阶导在两端为零，避免阶跃指令造成抖动。
阶段 2 会把 ``mobile_pick_place`` 里的同名逻辑替换成本模块，保证只有一份实现。
"""

from __future__ import annotations

import numpy as np


def smoothstep(t: float) -> float:
    """三次平滑插值系数，输入越界会被截断到 [0, 1]。"""
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


class JointTrajectory:
    """六轴关节空间的定时插值。"""

    def __init__(self, q_start: np.ndarray, q_target: np.ndarray, duration: float) -> None:
        self.q_start = np.asarray(q_start, dtype=float).copy()
        self.q_target = np.asarray(q_target, dtype=float).copy()
        self.duration = max(1e-6, float(duration))
        self.elapsed = 0.0
        self.q = self.q_start.copy()

    @property
    def done(self) -> bool:
        return self.elapsed >= self.duration

    def update(self, dt: float) -> np.ndarray:
        """推进 dt 秒并返回当前关节角。"""
        self.elapsed = min(self.duration, self.elapsed + float(dt))
        s = smoothstep(self.elapsed / self.duration)
        self.q = self.q_start + (self.q_target - self.q_start) * s
        return self.q


class ScalarTrajectory:
    """夹爪开合的单标量定时插值（量纲 0…255）。"""

    def __init__(self, start: float, target: float, duration: float) -> None:
        self.start = float(start)
        self.target = float(target)
        self.duration = max(1e-6, float(duration))
        self.elapsed = 0.0
        self.value = self.start

    @property
    def done(self) -> bool:
        return self.elapsed >= self.duration

    def update(self, dt: float) -> float:
        self.elapsed = min(self.duration, self.elapsed + float(dt))
        s = smoothstep(self.elapsed / self.duration)
        self.value = self.start + (self.target - self.start) * s
        return self.value
