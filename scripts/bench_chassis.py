"""架空台架：量真车底盘的轮径、轮距、死区与左右一致性。

文件目的：
    真车一到位，第一件要做的事不是跑任务，而是把"指令 → 实际运动"这条链量清楚：
    轮速指令是 rad/s，里程计报的是 m/s，两者之间隔着轮径、减速比与编码器换算；
    转向靠左右轮速差，中间隔着"有效轮距"（含轮胎滑移）。这些数一旦错，任务层的
    定位、停靠、抓取全都会按错的尺子算。本脚本把这一串测量固化成一条命令。

测什么（每一项都同时给出指令值、里程计值、真实位移值，三者的差别就是答案）：
    · 死区：指令多小开始真的动（轮胎静摩擦、驱动器死区、爬行速度）；
    · 轮径：`r = v / ω`，分别用里程计与真实位移算一次，差值即打滑；
    · 轮距：原地反向转时的 `track = 2·ω·r / 偏航角速度`，这个值是"有效轮距"，
      含轮胎横向刮擦，**不能**直接当几何轮距去改模型；
    · 左右一致性：同速直行 2 s 的偏航漂移，反映左右轮或编码器的系统偏差。

两个使用阶段（脚本不区分，靠数据判断，报告里写明）：
    · 架空：车不动而里程计有读数 → 量的是编码器换算与轮速一致性，`slip≈100%` 属正常；
    · 落地：里程计与真实位移都要对上 → 量的是真实轮径，偏差大说明打滑或换算错。

运行方式：
    python3 scripts/bench_chassis.py --backend sim --out reports/bench_chassis_sim.json
    （真车后端接好后加 --backend real，参数取自 driver 节点）

关键假设：
    · 指令单位是轮角速度 rad/s，左/右分别给，前轮转角恒给 0（真车没有转向节，
      该字段在真机驱动里应被显式拒绝或恒置 0，不能静默吃掉）；
    · 每个指令段先走 0.5 s 过渡再采样 1.0 s，避免把加速段算进稳态值；
    · 仿真与真车共用同一套 `ChassisInterface`，所以量法与判据完全一致。

已知限制：
    · 有效轮距含滑移，只用于把横摆角速度换算成角度，不回写模型参数；
    · 仿真轮胎是刚性滚动，`slip` 接近 0，真车会是几个百分点；
    · 本脚本不动任何项目文件，只写自己的 JSON 报告。

重要性等级：A"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
sys.path.insert(0, str(PROJECT))

FRAME_DT = 0.02
RAMP_FRAMES = 25  # 0.5 s 过渡
SAMPLE_FRAMES = 50  # 1.0 s 采样

# 判据：轮径偏差 1% 内算可用——但**真车要求 0.5%**（航点跟踪只靠里程计，19 m 行程下
# 0.5% 就是 9.5 cm，航点到点容差只有 0.12 m），1% 是这台工具用来报警的线，不是合格线。
# 有效轮距只做量级检查，因为滑移会把它放大。
RADIUS_TOL = 0.01
TRACK_TOL = 0.20
DEADBAND_LIMIT = 0.30  # rad/s，约 0.046 m/s；超过说明起转困难，需要现场看驱动增益
# 原地转的验收下限：0.3 rad/s ≈ 17°/s，摆正 5° 要 0.3 s；低于它现场基本只能靠倒车调姿。
PIVOT_MIN_YAW_RATE = 0.30
# 圆弧法测轮距要求的最小横摆：低于它说明小差速被静摩擦吃掉，只记录、不判定。
TRACK_MIN_YAW_RATE = 0.02


class SimChassis:
    """把单进程仿真入口包成"只需 pose/speed/step"的最小后端。"""

    def __init__(self, seed: int) -> None:
        import dual_car_pick_place as legacy

        self._legacy = legacy
        arm, finger, car1, car2, target = legacy.initialize_mission(seed)
        self._sim = legacy.base.Simulation(realtime=False)
        self.car = car1
        self._time = 0.0

    def set_wheels(self, left: float, right: float, steer: float) -> None:
        self.car.set_wheels(left, right, steer)

    def step(self) -> None:
        self._sim.step()
        self._time += FRAME_DT

    def pose(self) -> tuple[np.ndarray, float]:
        return self.car.pose()

    def speed(self) -> float:
        return self.car.speed()

    def reset(self, x: float = -13.0, y: float = -13.0, yaw: float = math.pi / 2) -> None:
        """把车摆到空旷角落、速度清零，保证每段测量都从静止开始。

        真车后端没有这一项（架空或落地都由现场决定起点），留空实现即可。
        注意只改本车 free joint 的 7 个 qpos 与 6 个 qvel，别碰另一台车。
        """
        import mujoco

        data = self._legacy.base.DATA
        qpos = data.qpos
        qpos[self.car.qadr + 0] = x
        qpos[self.car.qadr + 1] = y
        qpos[self.car.qadr + 3] = math.cos(yaw / 2.0)
        qpos[self.car.qadr + 4] = 0.0
        qpos[self.car.qadr + 5] = 0.0
        qpos[self.car.qadr + 6] = math.sin(yaw / 2.0)
        data.qvel[self.car.dadr : self.car.dadr + 6] = 0.0
        mujoco.mj_forward(self._legacy.base.MODEL, data)


def drive(
    chassis,
    left: float,
    right: float,
    frames: int,
    steer: float = 0.0,
) -> dict:
    """给一段固定轮速，跑若干帧，返回这段时间的运动统计。

    参数：
        chassis: 后端（需提供 set_wheels/step/pose/speed）。
        left, right: 左右侧轮角速度指令，rad/s。
        frames: 帧数（每帧 20 ms）。
        steer: 前轮转角，真车恒 0。
    返回：
        位移、真实平均速度、里程计平均速度、偏航角变化、起止位姿。
    重要性等级：A"""
    start_pos, start_yaw = chassis.pose()
    odom_sum = 0.0
    for _ in range(frames):
        chassis.set_wheels(left, right, steer)
        chassis.step()
        odom_sum += chassis.speed()
    end_pos, end_yaw = chassis.pose()
    dt = frames * FRAME_DT
    distance = float(np.linalg.norm(end_pos[:2] - start_pos[:2]))
    return {
        "distance": distance,
        "v_true": distance / dt,
        "v_odom": odom_sum / frames,
        "dyaw": float(_wrap(end_yaw - start_yaw)),
        "yaw_rate": float(_wrap(end_yaw - start_yaw)) / dt,
        "dt": dt,
    }


def _wrap(angle: float) -> float:
    """把角度折回 (-π, π]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def measure_deadband(chassis, commands: list[float]) -> dict:
    """逐档给同一左右轮速，找"开始能动"的最小指令。

    参数：
        chassis: 后端。
        commands: 待测轮速指令序列，rad/s。
    返回：
        每档的位移与速度，以及首个超过 5 mm 位移的指令值。
    重要性等级：A"""
    rows = []
    threshold = None
    for cmd in commands:
        chassis.reset()
        stats = drive(chassis, cmd, cmd, RAMP_FRAMES + SAMPLE_FRAMES)
        rows.append({"cmd": cmd, **stats})
        if threshold is None and stats["distance"] > 0.005:
            threshold = cmd
    return {"rows": rows, "threshold": threshold}


def measure_radius(chassis, speeds: list[float], nominal_radius: float) -> dict:
    """匀速直行，用里程计与真实位移分别反算轮径。

    参数：
        chassis: 后端。
        speeds: 目标线速度列表，m/s；换算成轮速指令。
        nominal_radius: 模型里的轮半径，m（用于指令换算与对照）。
    返回：
        每档的两种轮径估计、打滑比例与偏航漂移。
    重要性等级：A"""
    rows = []
    for target in speeds:
        omega = target / nominal_radius
        chassis.reset()
        drive(chassis, omega, omega, RAMP_FRAMES)  # 先过渡，采样段从稳态开始
        stats = drive(chassis, omega, omega, SAMPLE_FRAMES)
        rows.append(
            {
                "v_target": target,
                "omega_cmd": omega,
                "v_true": stats["v_true"],
                "v_odom": stats["v_odom"],
                "radius_true": stats["v_true"] / omega,
                "radius_odom": stats["v_odom"] / omega,
                "slip": 1.0 - (stats["v_true"] / stats["v_odom"] if stats["v_odom"] else 0.0),
                "yaw_drift_deg": math.degrees(stats["dyaw"]),
            }
        )
    return {"rows": rows}


def measure_track(chassis, omega_base: float, omega_delta: float, radius: float) -> dict:
    """走一段小曲率圆弧，用左右轮速差反算轮距。

    为什么不原地反向转：原地转靠轮胎横向刮擦换角度，滑移把它放大成"有效轮距"，
    仿真里 ±2 rad/s 甚至几乎不转（静摩擦门限）；而两侧都向前、只有速度差时，
    轮胎是滚动的，`track = (v_right - v_left) / 横摆角速度` 才是几何轮距。
    原地转的效率另由 `measure_pivot` 单独记录，不用来反算轮距。

    参数：
        chassis: 后端。
        omega_base: 基准轮速，rad/s（两侧平均）。
        omega_delta: 轮速差的一半，rad/s；左=base-delta，右=base+delta。
        radius: 已测得的轮半径，m。
    返回：
        有效轮距、横摆角速度与两侧理论线速度。
    重要性等级：A"""
    left = omega_base - omega_delta
    right = omega_base + omega_delta
    chassis.reset()
    drive(chassis, left, right, RAMP_FRAMES)
    stats = drive(chassis, left, right, SAMPLE_FRAMES)
    v_left = left * radius
    v_right = right * radius
    yaw_rate = stats["yaw_rate"]
    if abs(yaw_rate) < TRACK_MIN_YAW_RATE:
        # 小差速转不动：轮胎静摩擦把横摆力矩吃掉了。仿真底盘就是这样（转向靠前轮
        # 转向节的等效机构、差速只在 ±8 rad/s 量级才起作用），真车不应如此。
        return {
            "omega_left": left,
            "omega_right": right,
            "v_left": v_left,
            "v_right": v_right,
            "yaw_rate": yaw_rate,
            "track_eff": None,
            "note": f"横摆只有 {yaw_rate:.4f} rad/s，小差速翻不过静摩擦，轮距量不出来",
        }
    track = (v_right - v_left) / yaw_rate
    return {
        "omega_left": left,
        "omega_right": right,
        "v_left": v_left,
        "v_right": v_right,
        "yaw_rate": yaw_rate,
        "track_eff": track,
    }


def measure_pivot(chassis, omega: float) -> dict:
    """原地反向转：只记录"能否转起来、转多快"，不用来反算轮距。

    参数：
        chassis: 后端。
        omega: 单侧轮速幅值，rad/s（左负右正）。
    返回：
        横摆角速度与等效原地转效率。
    重要性等级：A"""
    chassis.reset()
    drive(chassis, -omega, omega, RAMP_FRAMES)
    stats = drive(chassis, -omega, omega, SAMPLE_FRAMES)
    return {"omega": omega, "yaw_rate": stats["yaw_rate"]}


def evaluate(result: dict, nominal_radius: float, nominal_track: float) -> list[str]:
    """按判据检查测量结果，返回问题清单（空表示通过）。

    参数：
        result: 全部测量结果。
        nominal_radius: 模型轮半径，m。
        nominal_track: 模型轮距，m。
    返回：
        问题描述列表。
    重要性等级：B"""
    problems = []
    radius = float(np.mean([row["radius_odom"] for row in result["radius"]["rows"]]))
    if abs(radius / nominal_radius - 1.0) > RADIUS_TOL:
        problems.append(
            f"里程计轮径 {radius:.4f} m 与模型 {nominal_radius:.4f} 差 "
            f"{abs(radius / nominal_radius - 1.0) * 100:.1f}%（>{RADIUS_TOL * 100:.0f}%）"
        )
    track = result["track"]["track_eff"]
    if track is None:
        pass  # 量不出来已在报告里给了原因；原地转那项会兜住"根本转不动"
    elif abs(track / nominal_track - 1.0) > TRACK_TOL:
        problems.append(
            f"有效轮距 {track:.3f} m 与模型 {nominal_track:.3f} 差 "
            f"{abs(track / nominal_track - 1.0) * 100:.0f}%（>{TRACK_TOL * 100:.0f}%），"
            "先确认是打滑还是编码器换算错"
        )
    if abs(result["pivot"]["yaw_rate"]) < PIVOT_MIN_YAW_RATE:
        problems.append(
            f"原地转只有 {result['pivot']['yaw_rate']:.3f} rad/s"
            f"（<{PIVOT_MIN_YAW_RATE}），摆正会很难；真车该项取决于轮胎与地面"
        )
    threshold = result["deadband"]["threshold"]
    if threshold is None:
        problems.append("所有死区档位都没有产生位移，底盘没动")
    elif threshold > DEADBAND_LIMIT:
        problems.append(f"死区 {threshold:.2f} rad/s 偏大（>{DEADBAND_LIMIT}），起转困难")
    drift = max(abs(row["yaw_drift_deg"]) for row in result["radius"]["rows"])
    if drift > 5.0:
        problems.append(f"直行偏航漂移 {drift:.1f}°/s，左右轮或编码器有系统偏差")
    return problems


def main() -> int:
    """跑完整台架流程、写 JSON 并打印结论。"""
    parser = argparse.ArgumentParser(description="架空台架：底盘指令-运动标定")
    parser.add_argument("--backend", default="sim", choices=["sim", "real"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default="", help="JSON 输出路径，默认 reports/bench_chassis.json")
    args = parser.parse_args()

    if args.backend != "sim":
        print("真车后端还没接上：等驱动节点实现 ChassisInterface 后在这里加一个分支")
        return 2

    from mission_core import WHEEL_RADIUS, WHEEL_TRACK

    chassis = SimChassis(args.seed)
    print(f"后端 sim，seed {args.seed}；模型轮径 {WHEEL_RADIUS} m、轮距 {WHEEL_TRACK} m")

    result = {
        "backend": args.backend,
        "seed": args.seed,
        "nominal": {"radius": WHEEL_RADIUS, "track": WHEEL_TRACK},
        "deadband": measure_deadband(chassis, [0.05, 0.1, 0.2, 0.5, 1.0]),
        "radius": measure_radius(chassis, [0.2, 0.4, 0.6], WHEEL_RADIUS),
    }
    radius_mean = float(np.mean([row["radius_odom"] for row in result["radius"]["rows"]]))
    result["track"] = measure_track(chassis, 2.0, 0.6, radius_mean)
    result["pivot"] = measure_pivot(chassis, 8.0)
    problems = evaluate(result, WHEEL_RADIUS, WHEEL_TRACK)
    result["problems"] = problems

    print("\n死区（指令 rad/s → 1 s 位移 m）：")
    for row in result["deadband"]["rows"]:
        print(f"  ω={row['cmd']:<5} 位移 {row['distance']:.4f} m  速度 {row['v_true']:.4f} m/s")

    print("\n轮径（里程计 / 真实 / 打滑）：")
    for row in result["radius"]["rows"]:
        print(
            f"  目标 {row['v_target']:.2f} m/s  ω={row['omega_cmd']:.3f}  "
            f"r_odom={row['radius_odom']:.4f}  r_true={row['radius_true']:.4f}  "
            f"slip={row['slip'] * 100:5.1f}%  漂移 {row['yaw_drift_deg']:+.2f}°/s"
        )

    track = result["track"]
    if track["track_eff"] is None:
        print(f"\n轮距：量不出来 —— {track['note']}")
    else:
        print(f"\n轮距（圆弧，ω={track['omega_left']:.2f}/{track['omega_right']:.2f} rad/s）："
              f"{track['track_eff']:.3f} m，横摆 {track['yaw_rate']:.3f} rad/s")
    pivot = result["pivot"]
    print(f"原地转（ω=±{pivot['omega']:.1f} rad/s）：横摆 {pivot['yaw_rate']:.3f} rad/s")

    out_path = Path(args.out) if args.out else PROJECT / "reports" / "bench_chassis.json"
    out_path = out_path if out_path.is_absolute() else PROJECT / out_path
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    if problems:
        print("\n不通过：")
        for item in problems:
            print(f"  - {item}")
    elif result["track"]["track_eff"] is None:
        print("\n通过：轮径、死区、左右一致性与原地转都在判据内（轮距未测，原因见上）")
    else:
        print("\n通过：轮径、轮距、死区、左右一致性与原地转都在判据内")
    print(f"json written: {out_path}")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
