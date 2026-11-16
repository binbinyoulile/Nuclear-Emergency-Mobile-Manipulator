"""端到端冒烟序列（阶段 1 临时，阶段 2 由正式七阶段状态机取代）。

目的
----
用真实适配器走一遍"底盘 → 感知 → 机械臂 → 夹爪"，证明整条链路可用：

1. ``/clock`` 帧同步确实在驱动任务循环（``Clock.step()`` 每次都等到新的一帧）；
2. 两车轮速指令经 DDS 到仿真侧，帧边界生效；
3. 车确实能开到目标观测位，**感知节点在真实距离上给出粗端中心**（误差在门限内）；
4. 机械臂世界坐标目标经逆解服务变成关节流，夹爪开合有状态回读。

它不是任务逻辑，也不做验收判据之外的判断；正式流程由 ``mission_core`` 提供，
``mode:=mission``（默认）跑那套，本模块只在 ``mode:=smoke`` 时使用。

关于 ``VISION_STANDOFF_Y``
--------------------------
单进程版分三站：观测位 2.1 m（粗定位）→ 测量位 1.41 m（精定位）→ 0.85 m（抓取）。
冒烟只验"感知能不能在真实距离上给出坐标"，所以直接开去测量位。目标位置这里读的是
``/sim/target_truth``——这是**测试夹具**用法（冒烟测试要知道把车停哪儿），任务层本身
仍然只认感知输出。
"""

from __future__ import annotations

import math

import numpy as np

from mission_core import (
    MEASURE_STANDOFF_Y,
    STEER_MAX,
    STEER_WHEELBASE,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)

# 测量位：车停在目标正南 1.41 m，此时腕部相机离杆约 1.0 m、落在可见带内。
# 再近（0.85 m）杆会掉出画面下沿，这是相机固定下俯 28.5° 决定的。
VISION_STANDOFF_Y = MEASURE_STANDOFF_Y

VISION_TOLERANCE_M = 0.03

# 机械臂冒烟动作：沿车头方向 ARM_FORWARD_M 处、抬升 ARM_LIFT_M。
# 必须在车头前方而不是正上方——正上方落在臂座轴线上，固定下压姿态不可解。
ARM_FORWARD_M = 0.50
ARM_LIFT_M = 0.55


def _wrap(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _hold(io, car1, car2) -> None:
    """两车停车 + 姿态保持 + 推进一帧。"""
    car1.set_wheels(0.0, 0.0, 0.0)
    car2.set_wheels(0.0, 0.0, 0.0)
    io.arm.hold()
    io.gripper.tick()
    io.clock.step()


def drive_straight(io, car1, car2, distance: float, speed: float) -> float:
    """直行 distance 米并保持初始航向，返回实际位移。"""
    pos0, yaw0 = car1.pose()
    start = np.array(pos0, dtype=float)
    heading = np.array([math.cos(yaw0), math.sin(yaw0)])
    deadline = io.clock.now + max(10.0, distance / max(speed, 0.05) * 4.0)
    while True:
        pos, yaw = car1.pose()
        travelled = float((pos[:2] - start[:2]) @ heading)
        if travelled >= distance:
            break
        if io.clock.now > deadline:
            raise TimeoutError(f"直行超时：已走 {travelled:.2f} m / 目标 {distance:.2f} m")
        steer = float(np.clip(1.0 * _wrap(yaw0 - yaw), -STEER_MAX, STEER_MAX))
        yaw_rate = speed * math.tan(steer) / STEER_WHEELBASE
        left = (speed - yaw_rate * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        right = (speed + yaw_rate * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        car1.set_wheels(left, right, steer)
        car2.set_wheels(0.0, 0.0, 0.0)
        io.arm.hold()
        io.gripper.tick()
        io.clock.step()
    for _ in range(10):
        _hold(io, car1, car2)
    pos, _ = car1.pose()
    return float(np.linalg.norm(pos[:2] - start[:2]))


def drive_to(
    io, car1, car2, goal, speed: float, tolerance: float = 0.12
) -> float:
    """按比例转向开到世界 XY 目标点，返回停车时到目标的距离。

    参数：
        goal: (x, y)，单位 m。
        speed: 巡航速度，m/s；接近目标时自动降速。
        tolerance: 到点判据，m。
    异常：
        超时抛 TimeoutError（距离 20 m 级，正常 20 s 内到位）。
    """
    goal = np.asarray(goal, dtype=float)[:2]
    pos, _ = car1.pose()
    span = float(np.linalg.norm(goal - pos[:2]))
    deadline = io.clock.now + max(20.0, span / max(speed, 0.2) * 8.0)
    while True:
        pos, yaw = car1.pose()
        delta = goal - pos[:2]
        distance = float(np.linalg.norm(delta))
        if distance <= tolerance:
            break
        if io.clock.now > deadline:
            raise TimeoutError(f"驶向目标点超时：还差 {distance:.2f} m")
        heading = math.atan2(float(delta[1]), float(delta[0]))
        steer = float(np.clip(1.4 * _wrap(heading - yaw), -STEER_MAX, STEER_MAX))
        speed_now = float(min(speed, max(0.35, 0.9 * distance)))
        yaw_rate = speed_now * math.tan(steer) / STEER_WHEELBASE
        left = (speed_now - yaw_rate * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        right = (speed_now + yaw_rate * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        car1.set_wheels(left, right, steer)
        car2.set_wheels(0.0, 0.0, 0.0)
        io.arm.hold()
        io.gripper.tick()
        io.clock.step()
    for _ in range(15):
        _hold(io, car1, car2)
    pos, _ = car1.pose()
    return float(np.linalg.norm(goal - pos[:2]))


def run(io, node) -> dict:
    """执行冒烟序列，返回指标字典（写进 RESULT_JSON）。"""
    car1, car2 = io.chassis
    metrics: dict = {}

    node.set_stage("smoke/稳定")
    for _ in range(25):
        _hold(io, car1, car2)

    node.set_stage("smoke/底盘直行")
    metrics["car1_travel_m"] = round(drive_straight(io, car1, car2, 1.2, 0.40), 3)
    if metrics["car1_travel_m"] < 1.0:
        raise RuntimeError(f"直行不足：{metrics['car1_travel_m']:.2f} m")

    node.set_stage("smoke/驶向观测位")
    truth, _ = io.perception.object_pose()
    if not np.all(np.isfinite(truth)):
        raise RuntimeError("未收到 /sim/object_pose，无法确定观测位")
    stance = np.array([truth[0], truth[1] - VISION_STANDOFF_Y], dtype=float)
    metrics["stance_error_m"] = round(drive_to(io, car1, car2, stance, 1.20), 3)

    node.set_stage("smoke/视觉")
    estimate = io.perception.estimate_target()
    error = float(np.linalg.norm(estimate - truth))
    metrics["vision_error_mm"] = round(error * 1000.0, 2)
    metrics["target_xyz"] = [round(float(v), 3) for v in estimate]
    if error > VISION_TOLERANCE_M:
        raise RuntimeError(f"视觉误差超限：{error * 1000.0:.1f} mm")

    node.set_stage("smoke/机械臂")
    pos, yaw = car1.pose()
    # 目标点不能落在臂座正上方：UR10e 在固定下压姿态下解不了"垂直向上"的位置
    # （实测残差 163 mm），沿车头方向偏 0.5 m 后残差降到 1.4 mm。
    target_world = np.array(
        [
            pos[0] + ARM_FORWARD_M * math.cos(yaw),
            pos[1] + ARM_FORWARD_M * math.sin(yaw),
            pos[2] + ARM_LIFT_M,
        ],
        dtype=float,
    )
    io.arm.start_world(target_world, duration=2.5)
    deadline = io.clock.now + 10.0
    while io.arm.active:
        if io.clock.now > deadline:
            raise TimeoutError("机械臂动作超时")
        io.arm.tick()
        io.gripper.tick()
        car1.set_wheels(0.0, 0.0, 0.0)
        car2.set_wheels(0.0, 0.0, 0.0)
        io.clock.step()
    metrics["arm_target_z"] = round(float(target_world[2]), 3)

    node.set_stage("smoke/夹爪")
    io.gripper.start(255.0, 1.2)
    while io.gripper.active:
        _hold(io, car1, car2)
    metrics["gripper_close_effort_n"] = round(io.gripper.force(), 3)
    io.gripper.start(0.0, 1.2)
    while io.gripper.active:
        _hold(io, car1, car2)

    for _ in range(10):
        _hold(io, car1, car2)

    node.set_stage("smoke/完成")
    metrics["contacts"] = int(io.chassis[0].contacts_with(io.chassis[1]))
    return metrics
