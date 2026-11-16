"""只拉起仿真侧（不含任务层），用于验证话题频率、TF 与 /clock。

用法::

    ros2 launch mm_bringup sim_only.launch.py seed:=6
    ros2 launch mm_bringup sim_only.launch.py max_frames:=500 use_rviz:=true

``max_frames`` 大于 0 时帧循环跑满即退出，便于在无人值守的命令行里验收。
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
    here = Path(os.path.realpath(__file__)).resolve()
    return str(here.parents[4])


def generate_launch_description() -> LaunchDescription:
    repo_root = _repo_root()
    seed = LaunchConfiguration("seed")
    max_frames = LaunchConfiguration("max_frames")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("seed", default_value="-1"),
            DeclareLaunchArgument(
                "max_frames",
                default_value="0",
                description="跑满多少帧后退出；0 表示一直跑",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                package="mm_sim",
                executable="sim_bridge",
                name="mm_sim_bridge",
                output="screen",
                parameters=[
                    {
                        "repo_root": repo_root,
                        "seed": ParameterValue(seed, value_type=int),
                        "max_frames": ParameterValue(max_frames, value_type=int),
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
