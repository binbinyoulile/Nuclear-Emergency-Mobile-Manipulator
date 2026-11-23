"""mm_base：真机底盘 CAN 驱动包。

模块
----
- ``protocol``   纯协议层：命令帧、反馈解析、单位换算、启动握手（不 import rclpy/socket）
- ``backend``    传输层：SocketCAN、回环假驱动器、录制包装
- ``kinematics`` 纯数学：/cmd_vel 分解、四轮里程计、轮关节换算
- ``base_node``  ROS2 节点：/cmd_vel → CAN，CAN → /odom、/joint_states
- ``selftest``   不接 ROS2/CAN 的自检（Windows 可跑）

``import mm_base`` 本身不导入任何子模块，也不依赖 rclpy；各模块按需单独导入。
"""

__all__ = ["protocol", "backend", "kinematics", "base_node", "selftest"]
