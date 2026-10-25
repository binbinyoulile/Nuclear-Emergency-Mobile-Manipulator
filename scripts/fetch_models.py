"""下载仓库所需 MuJoCo Menagerie 二进制网格资产。

文件目的：
    UR10e 与 Robotiq 2F-85 的大体积 OBJ/STL 资产不进入 Git，首次克隆后由本脚本恢复。
整体输入输出：
    输入为可选 --force；输出为 models/universal_robots_ur10e/assets 下的二进制文件。
依赖与版本：
    Python 3.10+ 标准库 urllib；需要可访问 raw.githubusercontent.com。
运行方式：
    D:/Python313/python.exe scripts\fetch_models.py
    D:/Python313/python.exe scripts\fetch_models.py --force
关键假设：
    MuJoCo Menagerie main 分支的资源路径和文件名保持兼容。
已知限制：
    网络不可达、代理阻断或上游改名时下载会失败；脚本不维护本地离线缓存。
重要性等级：B"""

from __future__ import annotations

import argparse
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = ROOT / "models" / "universal_robots_ur10e" / "assets"
BASE_URL = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main"

UR10E_ASSETS = [
    "base_0.obj",
    "base_1.obj",
    "shoulder_0.obj",
    "shoulder_1.obj",
    "shoulder_2.obj",
    "upperarm_0.obj",
    "upperarm_1.obj",
    "upperarm_2.obj",
    "upperarm_3.obj",
    "forearm_0.obj",
    "forearm_1.obj",
    "forearm_2.obj",
    "forearm_3.obj",
    "wrist1_0.obj",
    "wrist1_1.obj",
    "wrist1_2.obj",
    "wrist2_0.obj",
    "wrist2_1.obj",
    "wrist2_2.obj",
    "wrist3.obj",
]
ROBOTIQ_2F85_ASSETS = [
    "base_mount.stl",
    "base.stl",
    "driver.stl",
    "coupler.stl",
    "follower.stl",
    "pad.stl",
    "silicone_pad.stl",
    "spring_link.stl",
]


def source_url(filename: str) -> str:
    """根据文件名选择 Menagerie 上游下载地址。

参数：
    filename: UR10e 或 Robotiq 2F-85 的资产文件名。
返回：
    完整 HTTPS 原始文件 URL。
关键假设：
    文件名必须属于 UR10E_ASSETS 或 ROBOTIQ_2F85_ASSETS；未知名称会按 Robotiq 路径拼接并在下载时报 404。
重要性等级：C"""
    if filename in UR10E_ASSETS:
        return f"{BASE_URL}/universal_robots_ur10e/assets/{filename}"
    return f"{BASE_URL}/robotiq_2f85/assets/{filename}"


def fetch(filename: str, force: bool) -> None:
    """下载单个资产并在成功后原子替换目标文件。

参数：
    filename: 目标资产文件名。
    force: True 时覆盖已存在文件，False 时跳过。
异常：
    网络超时、HTTP 错误、磁盘写入失败或原子替换失败时向调用方抛出。
副作用：
    可能创建或覆盖 ASSET_DIR 下文件；临时路径统一使用 .part 后缀，finally 中清理残留。
关键参数：
    timeout=120 s 和 1 MiB 分块避免大文件长期占用内存。
重要性等级：B"""
    destination = ASSET_DIR / filename
    if destination.exists() and not force:
        print(f"skip {filename}")
        return

    partial = destination.with_name(destination.name + ".part")
    print(f"download {filename}")
    try:
        with urlopen(source_url(filename), timeout=120) as response:
            with partial.open("wb") as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
        partial.replace(destination)
    finally:
        if partial.exists():
            partial.unlink()


def main() -> None:
    """解析 --force 并下载全部 UR10e 和 Robotiq 资产。

副作用：
    创建资产目录、逐文件打印 download/skip 状态并输出最终路径。
异常：
    任一 fetch 失败会中止后续下载并将异常传播给 Python 进程。
重要性等级：B"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true", help="overwrite existing files")
    args = parser.parse_args()

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    for filename in UR10E_ASSETS + ROBOTIQ_2F85_ASSETS:
        fetch(filename, force=args.force)
    print(f"assets ready: {ASSET_DIR}")


if __name__ == "__main__":
    main()
