"""对比两次回归 CSV，判断重构前后行为是否一致。

用途：
    `run_matrix.py` 每次输出一份 `reports/regression.csv`。纯重构（不改变行为）应当
    得到逐位一致的指标；有意的行为改动（例如修掉不确定性）则应把差异讲清楚。
    本脚本把两份 CSV 逐 seed、逐指标对比，超出容差就标红并给出退出码 1。

用法：
    python scripts/compare_metrics.py reports/baseline.csv reports/regression.csv
    python scripts/compare_metrics.py a.csv b.csv --tol 1e-6

重要性等级：B"""

import argparse
import csv
import sys

# 参与对比的数值列（ok/stage 这类字符串列单独处理）。
METRICS = [
    "vision_error_mm",
    "drop_xy_error",
    "car2_top_edge",
    "separation",
    "min_separation",
    "car1_home_error",
    "car2_home_error",
    "reverse_min_gap",
    "sim_time_s",
]


def load(path: str) -> dict[str, dict[str, str]]:
    """按 seed 索引读取回归 CSV。"""
    with open(path, encoding="utf-8-sig", newline="") as handle:
        return {row["seed"]: row for row in csv.DictReader(handle)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("--tol", type=float, default=1e-9)
    args = parser.parse_args()

    base = load(args.baseline)
    cand = load(args.candidate)
    missing = sorted(set(base) - set(cand), key=int)
    if missing:
        print(f"candidate 缺少 seed: {missing}")
        return 2

    failures = 0
    for seed in sorted(base, key=int):
        b, c = base[seed], cand[seed]
        diffs = []
        for key in METRICS:
            if not b.get(key) or not c.get(key):
                continue
            delta = float(c[key]) - float(b[key])
            if abs(delta) > args.tol:
                diffs.append((key, float(b[key]), float(c[key]), delta))
        status = "OK " if not diffs and b["ok"] == c["ok"] else "DIFF"
        if status == "DIFF":
            failures += 1
        print(
            f"seed {seed:>2} {status} ok={c['ok']:<5} "
            f"stage={c['stage']:<12} sim={float(c['sim_time_s']):6.2f}s"
        )
        for key, old, new, delta in diffs:
            print(f"        {key:<18} {old:>12.6f} -> {new:>12.6f}  ({delta:+.6f})")

    print()
    if failures:
        print(f"结论：{failures} 个 seed 存在超容差差异（tol={args.tol:g}）")
        return 1
    print(f"结论：{len(base)} 个 seed 指标一致（tol={args.tol:g}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
