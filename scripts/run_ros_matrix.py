"""在 Ubuntu 侧用 ROS 2 并行跑轨道 B（多 seed），产出与轨道 A 同构的指标 CSV。

文件目的：
    轨道 B 验证的是"换成分布式后端后行为是否仍然收敛"。此前每个种子都靠手敲 shell
    启动、再从日志里抠数字——列容易抄错、条件不可复现、失败原因只留在终端。本脚本把
    "启动-等待-解析-判定"固化成一条命令：每个种子独占一个 ROS_DOMAIN_ID，M 路并行，
    全量日志落盘，指标写成与 run_matrix.py 同名的列，可直接交给 compare_metrics.py。
整体输入输出：
    输入：种子范围、并行度、单种子墙钟超时、CSV 与日志路径。
    输出：CSV（每 seed 一行）、seed_<n>.log（launch 全量输出）、终端摘要（含判据带）。
依赖与版本：
    Python 3.10+，仅标准库；运行在装了 ROS 2 Humble 的 Ubuntu 上，要求 ros2_ws 已
    colcon build --symlink-install，mm_bringup 能反推出仓库根。
运行方式：
    python3 scripts/run_ros_matrix.py --seeds 0-20 --parallel 4
    python3 scripts/run_ros_matrix.py --seeds 7 --parallel 1 --timeout 1800
    python3 scripts/run_ros_matrix.py --seeds 0-2 --dry-run
关键假设：
    ①每个种子必须独占 ROS_DOMAIN_ID（默认 100+seed）并设 ROS_LOCALHOST_ONLY=1，
      否则并行时 DDS 串台，/clock 与轮速指令互相污染；
    ②指标只从日志里的两行取：任务节点 RESULT_JSON（与轨道 A 同源）与桥的收尾行
      「帧循环结束：frames=… 指令超时=… 空转帧=… sim_time=…」；
    ③并行度固定后同一批数据才可比——「指令超时」是 50 ms 墙钟预算与每帧开销的赛跑，
      不同并行度之间不能互比。
已知限制：
    本脚本只负责"跑-测-判"，不动代码、不还原工作区；跑完自己看 git status。
    单种子墙钟约 5.5 min（0.8 档），21 个种子 4 路并行约 33 min。
    判定带写死在 CRITERIA 里，越界只记录不重跑。

重要性等级：A"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
# 直接运行本文件时 sys.path[0] 就是 scripts/，可与轨道 A 共用种子解析和摘要。
from run_matrix import COLUMNS, parse_seeds, summarize  # noqa: E402

RESULT_TAG = "RESULT_JSON: "
# 桥收尾行：labels 是中文且可能被改，只认 "=数字" 的骨架（与历史提取口径一致）。
FRAME_RE = re.compile(r"frames=(\d+) [^=]+=(\d+) [^=]+=(\d+) sim_time=([\d.]+)s")
LAUNCH_PKG = "mm_bringup"
LAUNCH_FILE = "sim_mission.launch.py"

# 轨道 B 比 A 多记四项：落杆倾角只在这里有意义，其余三项是桥的帧预算指标。
B_COLUMNS = COLUMNS + ["drop_tilt_deg", "frames", "cmd_timeouts", "idle_frames"]

# 物理判据带（规则见项目准则"两轨验证口径"）：只看真实目标是否达成。
MAX_VISION_MM = 30.0
MAX_HOME_ERROR_M = 0.20


def build_command(seed: int) -> str:
    """拼出单个种子的 bash 命令串（source 环境后 exec，保证信号直达 launch）。

    参数：
        seed: 随机种子，直接作为 launch 参数传下去。
    返回：
        交给 bash -c 的完整命令串。
    重要性等级：B"""
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {PROJECT}/ros2_ws/install/setup.bash && "
        f"exec ros2 launch {LAUNCH_PKG} {LAUNCH_FILE} seed:={seed}"
    )


def seed_env(seed: int, domain_base: int) -> dict[str, str]:
    """给单个种子准备独立 DDS 域的环境变量。

    参数：
        seed: 随机种子。
        domain_base: DOMAIN_ID 基数，实际用 domain_base + seed。
    返回：
        合并后的环境变量字典（保留父进程环境，便于继承代理等设置）。
    重要性等级：A"""
    return dict(
        os.environ,
        ROS_DOMAIN_ID=str(domain_base + seed),
        ROS_LOCALHOST_ONLY="1",
        # 无 X 也出深度图；退出时的 EGL 异常无害。
        MUJOCO_GL="egl",
        PYTHONUNBUFFERED="1",
    )


def parse_log(text: str) -> tuple[dict | None, tuple[int, int, int, float] | None]:
    """从一份 launch 日志里取出 RESULT_JSON 和桥收尾行的四个数字。

    参数：
        text: 日志全文。
    返回：
        (指标字典或 None, (frames, 指令超时, 空转帧, sim_time) 或 None)。
    重要性等级：A"""
    metrics = None
    for line in reversed(text.splitlines()):
        if RESULT_TAG in line:
            payload = line.split(RESULT_TAG, 1)[1]
            try:
                metrics = json.loads(payload)
            except ValueError:
                continue
            break
    match = FRAME_RE.search(text)
    frames = None
    if match:
        frames = (
            int(match.group(1)),
            int(match.group(2)),
            int(match.group(3)),
            float(match.group(4)),
        )
    return metrics, frames


def run_one(seed: int, timeout: float, log_dir: Path, domain_base: int) -> dict:
    """跑单个种子并返回扁平行；失败也返回一行，原因写进 stage/error。

    参数：
        seed: 随机种子。
        timeout: 墙钟超时秒（超时先 SIGINT 让 launch 收尾，再 SIGKILL）。
        log_dir: 日志目录。
        domain_base: DDS 域基数。
    返回：
        B_COLUMNS 的字典。
    副作用：
        写 log_dir/seed_<n>.log。
    重要性等级：A"""
    log_path = log_dir / f"seed_{seed}.log"
    started = time.perf_counter()
    exit_code: int | str = 0
    try:
        with log_path.open("w", encoding="utf-8", errors="replace") as handle:
            proc = subprocess.Popen(
                ["bash", "-c", build_command(seed)],
                cwd=str(PROJECT),
                env=seed_env(seed, domain_base),
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                exit_code = proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                # 整组发信号：ros2 launch 收到 INT 会带关所有节点，留不下孤儿时钟。
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                exit_code = "timeout"
    except OSError as exc:  # 日志都写不了，说明环境不对，直接报出来
        exit_code = f"launch-error:{exc}"
    wall_s = round(time.perf_counter() - started, 3)

    text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.exists() else ""
    metrics, frames = parse_log(text)
    error: str | None = None
    if exit_code == "timeout":
        error = f"TimeoutExpired after {timeout:.0f}s"
    if metrics is None:
        metrics = {
            "ok": False,
            "stage": "no_result",
            "error": error or f"no RESULT_JSON line, exit={exit_code}",
            "sim_time_s": "",
        }
    elif error:
        metrics["ok"] = False
        metrics["error"] = error

    row = {column: "" for column in B_COLUMNS}
    row.update(
        seed=seed,
        ok=metrics.get("ok", False),
        stage=metrics.get("stage", ""),
        error=metrics.get("error") or "",
        exit_code=exit_code,
        wall_s=metrics.get("wall_time_s", wall_s),
        sim_time_s=metrics.get("sim_time_s", ""),
        vision_error_mm=metrics.get("vision_error_mm", ""),
        target_x=(metrics.get("truth_target") or ["", ""])[0],
        target_y=(metrics.get("truth_target") or ["", ""])[1],
        dock_offset=metrics.get("dock_offset", ""),
        car2_dock_x=(metrics.get("car2_dock") or ["", ""])[0],
        car2_top_edge=metrics.get("car2_top_edge", ""),
        car2_outside_frame=metrics.get("car2_outside_frame", ""),
        separation=metrics.get("separation", ""),
        min_separation=metrics.get("min_separation", ""),
        drop_inside=metrics.get("drop_inside", ""),
        drop_xy_error=metrics.get("drop_xy_error", ""),
        final_inside=metrics.get("final_inside", ""),
        final_xy_error=metrics.get("final_xy_error", ""),
        contacts=metrics.get("contacts", ""),
        car1_home_error=metrics.get("car1_home_error", ""),
        car2_home_error=metrics.get("car2_home_error", ""),
        both_returned=metrics.get("both_returned", ""),
        reverse_min_gap=metrics.get("reverse_min_gap", ""),
        drop_tilt_deg=metrics.get("drop_tilt_deg", ""),
        frames=(frames[0] if frames else ""),
        cmd_timeouts=(frames[1] if frames else ""),
        idle_frames=(frames[2] if frames else ""),
        log_path=str(log_path.relative_to(PROJECT)),
    )
    return row


def band_reasons(row: dict) -> list[str]:
    """按物理判据带检查一行，返回违反项清单（空表示通过）。

    参数：
        row: run_one 产出的行。
    返回：
        违反项的简短描述列表。
    重要性等级：A"""
    reasons: list[str] = []
    if str(row["ok"]) != "True":
        reasons.append(f"ok={row['ok']} stage={row['stage']} {row['error']}".strip())
    for key in ("drop_inside", "final_inside"):
        if str(row[key]) != "True":
            reasons.append(f"{key}={row[key]}")
    if str(row["contacts"]) not in ("0", "0.0", ""):
        reasons.append(f"contacts={row['contacts']}")

    def number(key: str) -> float | None:
        try:
            return float(row[key])
        except (TypeError, ValueError):
            return None

    for key, limit in (("vision_error_mm", MAX_VISION_MM), ("car1_home_error", MAX_HOME_ERROR_M),
                       ("car2_home_error", MAX_HOME_ERROR_M)):
        value = number(key)
        if value is None:
            reasons.append(f"{key} 缺失")
        elif value > limit:
            reasons.append(f"{key}={value:.3f} > {limit:g}")
    return reasons


def preflight() -> str | None:
    """确认 ROS 与工作区安装存在，返回错误说明或 None。

    重要性等级：B"""
    setup = PROJECT / "ros2_ws" / "install" / "setup.bash"
    if shutil.which("bash") is None:
        return "找不到 bash，本脚本只能在 Ubuntu 上运行（轨道 B 需要 ROS 2）"
    if not setup.exists():
        return f"缺少 {setup}，先在仓库根跑 colcon build --symlink-install"
    probe = (
        "source /opt/ros/humble/setup.bash && "
        f"source {setup} && "
        f"ros2 pkg prefix {LAUNCH_PKG} >/dev/null"
    )
    done = subprocess.run(["bash", "-c", probe], cwd=str(PROJECT),
                          stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if done.returncode != 0:
        return f"mm_bringup 不可用：{done.stderr.decode('utf-8', 'replace').strip()[:300]}"
    return None


def main() -> int:
    """解析参数、并行跑完全部种子、写 CSV 并按判据带汇总。

    返回：
        0 = 全部落在判据带内；1 = 有种子越界；2 = 环境/参数问题。
    重要性等级：A"""
    parser = argparse.ArgumentParser(description="轨道 B（ROS 多 seed）批量回归")
    parser.add_argument("--seeds", default="0-20", help="如 0-20 或 1,3,7")
    parser.add_argument("--parallel", type=int, default=4, help="并行进程数，默认 4")
    parser.add_argument("--timeout", type=float, default=1800.0, help="单种子墙钟超时秒")
    parser.add_argument("--domain-base", type=int, default=100, help="DDS 域基数，默认 100")
    parser.add_argument("--out", default="", help="CSV 路径，默认 reports/ros_matrix.csv")
    parser.add_argument("--logs", default="", help="日志目录，默认 reports/ros_logs")
    parser.add_argument("--dry-run", action="store_true", help="只打印命令与环境，不启动")
    args = parser.parse_args()

    try:
        seeds = parse_seeds(args.seeds)
    except ValueError as exc:
        print(f"种子写法无法解析：{exc}")
        return 2
    if args.parallel < 1:
        print("--parallel 必须 ≥ 1")
        return 2
    if args.domain_base + seeds[-1] > 232:
        print(f"DDS 域越界：{args.domain_base}+{seeds[-1]} > 232，调小 --domain-base")
        return 2

    out_path = PROJECT / (args.out or Path("reports") / "ros_matrix.csv")
    log_dir = PROJECT / (args.logs or Path("reports") / "ros_logs")

    print(f"entry:   ros2 launch {LAUNCH_PKG} {LAUNCH_FILE}")
    print(f"seeds:   {seeds[0]}..{seeds[-1]} ({len(seeds)} 个)  parallel={args.parallel}")
    print(f"domains: {args.domain_base + seeds[0]}..{args.domain_base + seeds[-1]}")
    print(f"csv:     {out_path}")
    print(f"logs:    {log_dir}")
    if args.dry_run:
        for seed in seeds:
            print(f"  seed {seed:>3}  ROS_DOMAIN_ID={args.domain_base + seed}  {build_command(seed)}")
        return 0

    problem = preflight()
    if problem:
        print(f"预检失败：{problem}")
        return 2
    out_path.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    started = time.perf_counter()
    with out_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=B_COLUMNS)
        writer.writeheader()
        handle.flush()
        with ThreadPoolExecutor(max_workers=args.parallel) as pool:
            futures = {
                pool.submit(run_one, seed, args.timeout, log_dir, args.domain_base): seed
                for seed in seeds
            }
            for index, future in enumerate(futures, start=1):
                seed = futures[future]
                row = future.result()
                rows.append(row)
                # 完成一个写一个：中途被打断也留下已完成的证据。
                writer.writerow(row)
                handle.flush()
                flag = "" if not band_reasons(row) else "  << 越界"
                print(
                    f"[{index}/{len(seeds)}] seed {seed:>3} ok={str(row['ok']):<5} "
                    f"sim={row['sim_time_s']} wall={row['wall_s']}s{flag}",
                    flush=True,
                )
    total_wall = time.perf_counter() - started
    rows.sort(key=lambda r: r["seed"])

    summarize(rows)
    failures = [(row["seed"], band_reasons(row)) for row in rows if band_reasons(row)]
    print(f"判据带：视觉 ≤{MAX_VISION_MM:g} mm、回起点 <{MAX_HOME_ERROR_M:g} m、"
          f"两处 drop/final 在桶内、contacts=0")
    print(f"墙钟合计 {total_wall / 60:.1f} min（{len(seeds)} 个种子，并行 {args.parallel}）")
    if failures:
        print("\n越界清单：")
        for seed, reasons in failures:
            print(f"  seed {seed:>3}: {'; '.join(reasons)}")
        print(f"\n结论：{len(failures)}/{len(rows)} 个种子越界")
    else:
        print(f"\n结论：{len(rows)}/{len(rows)} 个种子落在判据带内")
    print(f"csv written: {out_path}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
