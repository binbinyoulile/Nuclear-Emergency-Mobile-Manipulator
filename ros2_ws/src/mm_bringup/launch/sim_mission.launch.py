"""一键拉起：仿真桥 + 目标定位 + 任务节点。

用法::

    ros2 launch mm_bringup sim_mission.launch.py seed:=6
    ros2 launch mm_bringup sim_mission.launch.py seed:=6 mode:=smoke
    ros2 launch mm_bringup sim_mission.launch.py use_rviz:=true

说明
----
``repo_root`` 由本文件位置反推（``ros2_ws/src/mm_bringup/launch`` 上溯四级），
所以必须在源码树里用 ``colcon build --symlink-install`` 构建；
脱离源码树安装时请改为显式传参 ``repo_root:=<仓库根>``。

任务节点退出时必须连带关掉整组（``on_exit=Shutdown``）：否则仿真桥会留在那里
继续发 ``/clock``，下一次启动就会同时有两条时钟流（TF_OLD_DATA 刷屏、
两车里程计互相覆盖）。
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition
from launch.events import Shutdown
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
    mode = LaunchConfiguration("mode")
    command_timeout = LaunchConfiguration("command_timeout")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_summary = LaunchConfiguration("lidar_summary")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "seed", default_value="-1", description="随机种子；-1 表示每次随机"
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="mission",
                description="mission = 七阶段正式任务；smoke = 阶段 1 端到端冒烟",
            ),
            DeclareLaunchArgument(
                "command_timeout",
                default_value="0.05",
                description="帧同步等待两车指令的墙钟超时，单位 s",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="false", description="是否同时启动 rviz2"
            ),
            DeclareLaunchArgument(
                "lidar_summary",
                default_value="false",
                description="同时跑雷达摘要节点，与桥的摘要逐帧对照（check 模式）",
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
                        # 点云只在对照时发，正常回归不增加 DDS 负担。
                        "publish_cloud": ParameterValue(
                            lidar_summary, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="mm_perception",
                executable="lidar_summary",
                name="mm_lidar_summary",
                output="screen",
                condition=IfCondition(lidar_summary),
                additional_env={"MM_REPO_ROOT": repo_root},
                parameters=[
                    {
                        "use_sim_time": True,
                        # 仿真里只用 check 模式：不接管任务话题，只做等效性对照。
                        "mode": "check",
                        "car": "car1",
                        # 桥的点云已经按离地高度过滤过，这里关掉地面门限。
                        "filter_ground": False,
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
                on_exit=[EmitEvent(event=Shutdown())],
                additional_env={"MM_REPO_ROOT": repo_root},
                parameters=[
                    {
                        "use_sim_time": True,
                        "seed": ParameterValue(seed, value_type=int),
                        "mode": ParameterValue(mode, value_type=str),
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
