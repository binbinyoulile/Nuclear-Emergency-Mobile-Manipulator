"""一键拉起：仿真桥 + 目标定位 + 任务节点。

用法::

    ros2 launch mm_bringup sim_mission.launch.py seed:=6
    ros2 launch mm_bringup sim_mission.launch.py use_rviz:=true

说明
----
``repo_root`` 由本文件位置反推（``ros2_ws/src/mm_bringup/launch`` 上溯四级），
所以必须在源码树里用 ``colcon build --symlink-install`` 构建；
脱离源码树安装时请改为显式传参 ``repo_root:=<仓库根>``。
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _repo_root() -> str:
    """从本文件真实路径反推仓库根（--symlink-install 时 realpath 会指回源码树）。"""
    here = Path(os.path.realpath(__file__)).resolve()
    return str(here.parents[4])


def generate_launch_description() -> LaunchDescription:
    repo_root = _repo_root()
    seed = LaunchConfiguration("seed")
    command_timeout = LaunchConfiguration("command_timeout")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "seed", default_value="-1", description="随机种子；-1 表示每次随机"
            ),
            DeclareLaunchArgument(
                "command_timeout",
                default_value="0.05",
                description="帧同步等待两车指令的墙钟超时，单位 s",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="false", description="是否同时启动 rviz2"
            ),
            Node(
                package="mm_sim",
                executable="sim_bridge",
                name="mm_sim_bridge",
                output="screen",
                parameters=[
                    {
                        "repo_root": repo_root,
                        "seed": ParameterValue(seed, value_type=int),
                        "command_timeout": ParameterValue(
                            command_timeout, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="mm_perception",
                executable="target_localizer",
                name="mm_target_localizer",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="mm_mission",
                executable="mission_node",
                name="mm_mission",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "seed": ParameterValue(seed, value_type=int),
                        "frame_dt": 0.02,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
