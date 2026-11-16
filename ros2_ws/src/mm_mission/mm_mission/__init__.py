"""mm_mission：硬件无关的任务决策层。

模块划分
--------
- ``trajectory`` 关节/夹爪插值（纯数学，无 ROS、无 MuJoCo）
- ``adapters``   把 ROS 话题封装成仓库根 ``robot_interface.py`` 的端口实现
- ``node``       MissionNode：rclpy 节点 + 阻塞式任务线程 + /clock 帧同步；
                  ``mode:=mission`` 时直接调用仓库根的 ``mission_core`` 七阶段状态机
- ``smoke``      端到端冒烟序列（阶段 1 用于验证链路，保留作 ``mode:=smoke``）

仓库根注入
----------
``mission_core.py`` 与 ``robot_interface.py`` 放在仓库根（``ros2_ws`` 的上一级），
单进程入口与 ROS 任务节点共用同一份，所以这里把它们加进 ``sys.path``。
``colcon build --symlink-install`` 时 ``realpath`` 会指回源码树，因此能反推出仓库根；
脱离源码树安装时用环境变量 ``MM_REPO_ROOT`` 显式指定。
"""

import os
import sys
from pathlib import Path


def _inject_repo_root() -> None:
    """把仓库根加入 sys.path，使 ``import mission_core`` 可用。"""
    candidates = []
    env_root = os.environ.get("MM_REPO_ROOT")
    if env_root:
        candidates.append(Path(env_root))
    candidates.append(Path(os.path.realpath(__file__)).parents[4])
    for root in candidates:
        if (root / "mission_core.py").is_file():
            root_str = str(root)
            if root_str not in sys.path:
                sys.path.insert(0, root_str)
            return
    raise ImportError(
        "找不到仓库根的 mission_core.py：请用 `colcon build --symlink-install` 构建，"
        "或设置环境变量 MM_REPO_ROOT=<仓库根目录>"
    )


_inject_repo_root()
