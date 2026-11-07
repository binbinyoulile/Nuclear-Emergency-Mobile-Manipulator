"""批量运行双车协同任务并汇总可靠性矩阵。

文件目的：
    对一组随机种子逐个启动 dual_car_pick_place.py --headless 子进程，读取其
    单行 RESULT_JSON 指标，写出 CSV 矩阵、每个种子的完整日志和终端摘要。
    这是判断“当前版本到底有多可靠”的唯一依据，用于在任何参数或结构改动前后做对比。
整体输入输出：
    输入为种子范围、单种子超时和输出路径；输出为 CSV 指标表、日志目录和摘要。
依赖与版本：
    Python 3.10+，仅标准库；子进程依赖项目自身依赖（mujoco、mink、opencv-python 等）。
运行方式：
    D:/Python313/python.exe scripts/run_matrix.py --seeds 0-20
    D:/Python313/python.exe scripts/run_matrix.py --seeds 7 --timeout 1800
关键假设：
    任务脚本以 --headless 运行，不会覆写 docs/screenshots/；
    RESULT_JSON 是唯一指标来源，解析失败即视为该 seed 失败而不是沿用旧数据。
已知限制：
    串行执行，一个 seed 崩溃不影响后续 seed，但总耗时是各 seed 之和；
    墙钟时间受机器负载影响，只能同机同条件比较。
重要性等级：A"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
ENTRY = PROJECT / "dual_car_pick_place.py"
RESULT_TAG = "RESULT_JSON: "

# CSV 列：成功判定、失败定位、以及验收需要对比的全部指标。
COLUMNS = [
    "seed",
    "ok",
    "stage",
    "error",
    "exit_code",
    "wall_s",
    "sim_time_s",
    "vision_error_mm",
    "target_x",
    "target_y",
    "dock_offset",
    "car2_dock_x",
    "car2_top_edge",
    "car2_outside_frame",
    "separation",
    "min_separation",
    "drop_inside",
    "drop_xy_error",
    "final_inside",
    "final_xy_error",
    "contacts",
    "car1_home_error",
    "car2_home_error",
    "both_returned",
    "reverse_min_gap",
    "log_path",
]


def parse_seeds(text: str) -> list[int]:
    """把 '0-20'、'1,3,7' 或混合写法解析成去重升序的种子列表。

参数：
    text: 命令行 --seeds 的原始文本。
返回：
    种子整数列表。
异常：
    写法无法解析或区间上下界颠倒时抛 ValueError。
重要性等级：B"""
    seeds: set[int] = set()
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            low, high = chunk.split("-", 1)
            start, end = int(low), int(high)
            if end < start:
                raise ValueError(f"bad seed range: {chunk}")
            seeds.update(range(start, end + 1))
        else:
            seeds.add(int(chunk))
    if not seeds:
        raise ValueError("no seed given")
    return sorted(seeds)


def parse_result(output: str) -> dict | None:
    """从子进程输出中取出最后一行 RESULT_JSON。

参数：
    output: 子进程 stdout 全文。
返回：
    解析后的指标字典；没有该行时返回 None。
重要性等级：A"""
    for line in reversed(output.splitlines()):
        if line.startswith(RESULT_TAG):
            return json.loads(line[len(RESULT_TAG):])
    return None


def run_one(seed: int, timeout: float, log_dir: Path) -> dict:
    """以子进程运行单个种子并返回扁平化的指标行。

参数：
    seed: 随机种子。
    timeout: 墙钟超时秒数，超时即杀进程并记为失败。
    log_dir: 完整日志目录。
返回：
    符合 COLUMNS 的字典；失败时 ok=False 且 stage/error 说明失败位置。
副作用：
    写 log_dir/seed_<n>.log。
重要性等级：A"""
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
    command = [sys.executable, str(ENTRY), "--headless", "--seed", str(seed)]
    started = time.perf_counter()
    exit_code: int | str = 0
    output = ""
    error: str | None = None
    try:
        proc = subprocess.run(
            command,
            cwd=str(PROJECT),
            env=env,
            capture_output=True,
            timeout=timeout,
        )
        exit_code = proc.returncode
        output = proc.stdout.decode("utf-8", "replace")
        stderr = proc.stderr.decode("utf-8", "replace")
    except subprocess.TimeoutExpired as exc:
        exit_code = "timeout"
        output = (exc.stdout or b"").decode("utf-8", "replace")
        stderr = (exc.stderr or b"").decode("utf-8", "replace")
        error = f"TimeoutExpired after {timeout:.0f}s"
    wall_s = round(time.perf_counter() - started, 3)

    log_path = log_dir / f"seed_{seed}.log"
    log_path.write_text(output + "\n--- stderr ---\n" + stderr, encoding="utf-8")

    metrics = parse_result(output)
    if metrics is None:
        metrics = {
            "ok": False,
            "stage": "no_result",
            "error": error or f"no RESULT_JSON line, exit={exit_code}",
            "sim_time_s": 0.0,
            "wall_time_s": wall_s,
        }
    elif error:
        metrics["ok"] = False
        metrics["error"] = error

    row = {column: "" for column in COLUMNS}
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
        log_path=str(log_path.relative_to(PROJECT)),
    )
    return row


def summarize(rows: list[dict]) -> None:
    """打印通过率、失败清单和关键指标的均值/极值。

参数：
    rows: 已完成的指标行。
重要性等级：B"""
    total = len(rows)
    passed = [row for row in rows if row["ok"]]
    failed = [row for row in rows if not row["ok"]]
    print("\n" + "=" * 70)
    print(f"通过率: {len(passed)}/{total} = {len(passed) / total * 100:.1f}%")
    if failed:
        print("\n失败清单：")
        for row in failed:
            print(f"  seed {row['seed']:>3}: stage={row['stage']} {row['error']}")

    def stat(key: str, unit: str = "", scale: float = 1.0) -> None:
        values = [
            float(row[key]) * scale
            for row in passed
            if row[key] not in ("", None)
        ]
        if not values:
            return
        print(
            f"  {key:<18} mean={sum(values) / len(values):8.3f}{unit}"
            f"  min={min(values):8.3f}{unit}  max={max(values):8.3f}{unit}"
        )

    if passed:
        print("\n成功样本指标：")
        stat("vision_error_mm", " mm")
        stat("drop_xy_error", " m")
        stat("final_xy_error", " m")
        stat("car1_home_error", " m")
        stat("car2_home_error", " m")
        stat("min_separation", " m")
        stat("sim_time_s", " s")
        stat("wall_s", " s")
    print("=" * 70)


def main() -> int:
    """解析参数、逐种子运行、写 CSV 并打印摘要。

返回：
    进程退出码：全部通过为 0，存在失败为 1。
重要性等级：A"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", default="0-20", help="如 0-20 或 1,3,7")
    parser.add_argument("--timeout", type=float, default=1800.0, help="单种子墙钟超时秒")
    parser.add_argument("--out", default="", help="CSV 路径，默认 reports/regression.csv")
    parser.add_argument("--logs", default="", help="日志目录，默认 reports/logs")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    out_path = Path(args.out) if args.out else PROJECT / "reports" / "regression.csv"
    log_dir = Path(args.logs) if args.logs else PROJECT / "reports" / "logs"
    # 允许传相对路径：统一按项目根解析，后面的 relative_to 才不会失败。
    out_path = out_path if out_path.is_absolute() else PROJECT / out_path
    log_dir = log_dir if log_dir.is_absolute() else PROJECT / log_dir
    out_path.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    print(f"entry: {ENTRY}")
    print(f"seeds: {seeds[0]}..{seeds[-1]} ({len(seeds)} 个), timeout={args.timeout:.0f}s")
    print(f"csv:   {out_path}")

    rows: list[dict] = []
    with out_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for index, seed in enumerate(seeds, start=1):
            row = run_one(seed, args.timeout, log_dir)
            rows.append(row)
            writer.writerow(row)
            handle.flush()
            print(
                f"[{index}/{len(seeds)}] seed {seed:>3} "
                f"ok={str(row['ok']):<5} stage={row['stage']:<14} "
                f"sim={row['sim_time_s']} wall={row['wall_s']}s",
                flush=True,
            )
            if not row["ok"] and row["error"]:
                print(f"           {row['error']}", flush=True)

    summarize(rows)
    print(f"\ncsv written: {out_path}")
    return 0 if all(row["ok"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
