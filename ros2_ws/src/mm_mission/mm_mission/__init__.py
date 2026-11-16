"""mm_mission：硬件无关的任务决策层。

模块划分
--------
- ``trajectory`` 关节/夹爪插值（纯数学，无 ROS、无 MuJoCo）
- ``adapters``   把 ROS 话题封装成与 ``robot_interface.py`` 同构的硬件端口
- ``node``       MissionNode：rclpy 节点 + 阻塞式任务线程 + /clock 帧同步
- ``smoke``      端到端冒烟序列（阶段 1 用于验证链路，阶段 2 由正式状态机取代）
"""
