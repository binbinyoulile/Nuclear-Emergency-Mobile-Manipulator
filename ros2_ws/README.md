# ros2_ws：ROS 2 分布式骨架与任务层接入

把单进程仿真程序按 **设备 I/O / 任务决策 / 目标识别** 拆成三个节点，用 `/clock` 与帧同步保证可复现。
任务本体（七阶段状态机 + 三个路径控制器 + 抓取/投放编排）只有一份，在仓库根的
`mission_core.py`，单进程入口与这里的 `mm_mission` 共用。
设计依据见 [`../docs/ros2_stage1_design.md`](../docs/ros2_stage1_design.md)。

```
mm_sim_bridge ──(状态话题 + /clock + /tf)──▶ mm_mission ──(调仓库根 mission_core)──▶ 指令
      ▲                                          │
      └──────────(轮速 / 关节 / 夹爪指令)─────────┘
                    mm_target_localizer ──(目标位姿)──▶ mm_mission
```

## 包

| 包 | 类型 | 内容 |
|---|---|---|
| `mm_msgs` | ament_cmake | 6 个 msg + 2 个 srv（轮速、雷达摘要、夹爪、目标、状态、逆解、截图） |
| `mm_sim` | ament_python | `sim_bridge`：独占 MuJoCo，物理 / 传感器 / `/clock` / 帧同步 |
| `mm_mission` | ament_python | `mission_node`：任务线程 + 硬件端口适配器；`mode:=mission` 跑仓库根的 `mission_core`，`mode:=smoke` 跑冒烟 |
| `mm_perception` | ament_python | `target_localizer`：深度图 + TF → **粗端中心**世界坐标 |
| `mm_bringup` | ament_cmake | launch 文件 |

依赖方向：`mm_sim` 与 `mm_mission` **互不 import**，只共享 `mm_msgs` 的消息契约。
`mm_mission` 不依赖 MuJoCo，也不依赖机械臂模型；它依赖仓库根的 `mission_core.py` /
`robot_interface.py`（`mm_mission/__init__.py` 把仓库根注入 `sys.path`，见下文）。

## 依赖

```bash
sudo apt install ros-humble-desktop ros-humble-tf2-ros \
                 ros-humble-vision-opencv python3-opencv \
                 python3-colcon-common-extensions
```

仓库本身的 Python 依赖（mujoco / mink / opencv）见仓库根的 `requirements.txt`。

## 构建

```bash
cd <仓库>/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 是必须的：launch 文件靠自身真实路径反推仓库根，用来定位
`dual_car_scene.xml` 与 `models/`；`mm_mission` 也靠同样方式找到仓库根的
`mission_core.py`。脱离源码树安装时改为显式传参 `repo_root:=<仓库根>`，
并给任务节点设环境变量 `MM_REPO_ROOT=<仓库根>`（launch 文件已自动传）。
`mm_mission` 会 import 仓库根的 `mission_core`，因此系统 python3 里需要 `numpy` 与
`scipy`（`python3 -m pip install --user scipy`），不需要 `mujoco` / `mink`。

## 运行

```bash
# 全链路七阶段任务（仿真 + 感知 + 任务）
ros2 launch mm_bringup sim_mission.launch.py seed:=6

# 阶段 1 的端到端冒烟（同一套节点，跑的是 smoke 序列）
ros2 launch mm_bringup sim_mission.launch.py seed:=6 mode:=smoke

# 看画面
ros2 launch mm_bringup sim_mission.launch.py seed:=6 use_rviz:=true

# 只起仿真侧，验证 /clock、话题频率与 TF（跑满 500 帧自动退出）
ros2 launch mm_bringup sim_only.launch.py seed:=6 max_frames:=500
```

任务节点退出时 launch 会连带关掉整组（`on_exit=Shutdown()`）。这不是贴心而是必需：
桥留在后台会继续发 `/clock`，下一次启动就有两条时钟流（TF_OLD_DATA 刷屏、两车里程计
互相覆盖）。

注意两件事：

- **`sim_only` 不含感知节点**，所以 `/perception/target` 在那条 launch 下根本不会出现（那是设计如此，不是故障）。要看感知必须用 `sim_mission`。
- **`/clock` 的 50 Hz 指仿真时间**（1 帧 = 20 ms），不是墙钟频率：桥用 `realtime=False`，任务节点在跑时会被 CPU 速度推进（墙钟可能几百 Hz）；`sim_only` 下没有任务层发轮速，每帧都在等指令 50 ms，墙钟反而只有 ~20 Hz。**唯一可靠的判据是收尾日志 `frames=N sim_time=T`**，即 `T == N × 0.02`。
- **收尾日志的超时计数分两段**：`帧循环结束：frames=N 指令超时=T 空转帧=I sim_time=…`。
  首帧轮速指令到达前桥是空转（任务节点在等状态话题就绪），这段只记 `空转帧`（`sim_only` 下
  `T=0`、`I=N` 属正常）；两车都下发过指令之后仍等不到新指令才算 `指令超时`（`T`），
  `T` 是并行跑是否污染时序的唯一判据，要求为 0。警告本身是节流的（第 1~3 次 + 每 200 次），
  **不要用警告行数当超时次数**。

检查链路：

```bash
ros2 topic hz /clock                 # 50 Hz（跑满即停，属于正常）
ros2 topic hz /car1/odom             # 50 Hz
ros2 topic hz /car1/obstacle_summary # 10 Hz
ros2 topic hz /sim/object_pose       # 50 Hz（被操作对象的实时位姿）
ros2 run tf2_tools view_frames       # 5 个帧全是 map 的子帧：carN/base_link、car1/arm_base_link、car1/camera_link、car1/pinch

# 单独确认这两个帧的世界位姿（父帧是 map，值即世界坐标，应落在车体附近而不是 ~20 m 外）
ros2 run tf2_ros tf2_echo map car1/arm_base_link
ros2 run tf2_ros tf2_echo map car1/pinch
```

任务结束会打印一行 `RESULT_JSON:`，同时发布到 `/mission/result`。

## 当前状态（阶段 2：任务层已接入）

- **任务本体只有一份**：`DualCarMission`、`PathFollower` / `SkidTurnFollower` /
  `ReverseStraightFollower`、`run_*_concurrent`、抓取与协同投放编排全在仓库根的
  `mission_core.py`（零 MuJoCo）。`mode:=mission` 时 `MissionNode` 直接用它，单进程入口
  `dual_car_pick_place.py` 也用同一份——不再有两份状态机。
- **端口**：`robot_interface.py` 定义 `Clock / LidarInterface / ChassisInterface /
  ArmInterface / GripperInterface / PerceptionInterface / RobotIO`；`mm_mission/adapters.py`
  的 `Ros*` 类直接继承这些 ABC，少实现一个方法在构造时就报 `TypeError`
  （Windows 上没有 rclpy，用 `../scripts/check_ports.py` 做这一层检查）。
- **阶段 2 新增的契约**：`/sim/object_pose`（杆的实时位姿，投放对中用）、
  `car1/pinch` TF（夹持点坐标）、`/car1/lidar/ignore_peer`（末段直线入位时临时关掉
  "绕行同伴车"）。详见设计文档 5.5。
- **帧同步的两个补丁**：`RosClock.step()` 每帧补发两车轮速指令，`request_estimate()`
  在等感知时继续推进帧——否则机械臂/视觉阶段桥会每帧超时，仿真也会脱离任务层自由跑。
- **TF 只有一层**：桥发布的 5 个帧（`carN/base_link`、`car1/arm_base_link`、
  `car1/camera_link`、`car1/pinch`）父帧**全是 `map`**，因为值都是 MuJoCo 世界量。
  `car1/arm_base_link` 不等于"固定基座坐标系"：`arm_ik_scene.xml` 的 base 体相对底盘臂座
  转了 180°（`mission_core.FIXED_BASE_ROT`），`ArmInterface.start_fixed` 的入参是后者。
  这条链路错了不会报错——残差照样几毫米，机械臂只是走到镜像位置。
  `scripts/check_ports.py` 的第 3 项就是钉这个往返，第 4 项钉"工具姿态真的送到逆解服务"。
  **两项都做过 FAIL→OK 的反向验证**（第 3 项去掉 `.T`、第 4 项把姿态换成默认值），
  不是写完就摆在那里的死代码。
- **工具姿态必须随请求走**：`SolveIK.srv` 的 `orientation` 是固定基座系四元数，由
  `RosArm.start_world`/`solve_ik` 原样透传。抓取要求工具绕自身 Z 轴转 ±90° 才能横跨
  杆身，退回 0° 时夹爪照常闭合、力也有读数（0.95~1.62 N，比空夹的 0.2~0.6 N 还高），
  但杆一根都不会被拿起来，任务卡在 `4_grasp`。**最早的判据是
  `finger force after close`：真正夹住约 5 N，夹空是 1 N 量级。**
- **超时参数是环境宽容度，不是判据**：`frame_timeout` 5→15 s、`ready_timeout` 30→60 s。
  实测同一台虚拟机出现过 2~6 倍降速（场景导入 3 s→31 s、控制帧 30 ms→88 ms），5 s 的帧超时
  会把**正常运行**判成失败；放宽后真故障照样超时（桥真死了 15 s 必然没有 `/clock`），
  而任务成功率、物理判据带一个字都没动。调这个参数不影响任何验收结论。
- **验证状态**：任务层拆分后单进程轨道 A 复跑 `seed 0-20`，与重构前基线逐位一致；
  ROS 轨道 B（21 种子，`e09370d`）Ubuntu 上 **21/21**（并行度 4、每种子独立 `ROS_DOMAIN_ID`，
  墙钟 23.5 min），口内余量 19~61 mm、杆顶 ≤0.628 m、视觉 0.93~6.34 mm、车车接触 0。
  首次运行暴露的 TF 父帧、`start_fixed` 坐标系、工具姿态丢失三处错误已修。

`mode:=smoke` 保留为链路冒烟，判据（离线用单进程后端跑过 seed 2/4/6/7/14，Ubuntu 端
seed 6 实测一次）：

| 指标 | 期望 |
|---|---|
| `car1_travel_m` | ≥ 1.0 m |
| `stance_error_m` | ≤ 0.15 m（实测 0.103~0.110） |
| `vision_error_mm` | ≤ 30 mm（实测 1.0~2.4 mm） |
| `gripper_close_effort_n` | 只需 > 0；**空夹约 0.2~0.6 N 是正常值**（12 N 是夹爪怼到自身臂杆才会出现的饱和值） |
| `contacts` | 0 |

机械臂目标必须在**车头前方**：正上方正好落在臂座轴线上，UR10e 在固定下压姿态下
解不出来（实测残差 163 mm），偏 0.5 m 后降到 2 mm 量级。

## 话题与参数

契约表见设计文档第 5 节。本工作空间新增的节点参数：

| 节点 | 参数 | 默认 | 说明 |
|---|---|---|---|
| `mm_sim_bridge` | `repo_root` | 由 launch 反推 | 仓库根目录 |
| | `seed` | `-1` | `-1` 表示每次随机 |
| | `command_timeout` | `0.05` | 帧同步等待两车指令的墙钟超时，s |
| | `max_frames` | `0` | 跑满多少帧后退出，0 表示一直跑 |
| `mm_mission` | `frame_dt` | `0.02` | 控制帧时长，与仿真保持一致 |
| | `mode` | `mission` | `mission` = 七阶段正式任务；`smoke` = 阶段 1 端到端冒烟 |
| | `frame_timeout` | `15.0` | 等 `/clock` 的墙钟超时 |
| | `ready_timeout` | `60.0` | 等首帧状态的超时（场景导入在慢机器上要 30 s 级） |
| | `seed` | `-1` | 只用于指标，场景随机化在仿真侧 |
| `mm_target_localizer` | `roi_x_min` / `roi_x_max` | `-0.30` / `0.30` | 搜索窗 X 范围（map 系），默认对齐绿框内缩 5 cm |
| | `roi_y_min` / `roi_y_max` | `20.74` / `21.26` | 搜索窗 Y 范围，默认对齐绿框内缩 4 cm |

## 感知

`mm_perception` 已同步到最新被操作对象：不是 HSV 颜色分割，而是与单进程版同一套深度算法
（反投影 → 地平面拟合 → 离地 0.8~30 mm → 连通域 + 搜索窗 → PCA 定轴 → 两端顶面 1 mm
高差判粗端），输出**粗端中心**；同一帧喂两边结果逐位一致（实测 0.000 mm）。
物理侧的杆与 Ø240 屏蔽桶由 `mm_sim` 动态 import 仓库根的 `dual_car_pick_place` 自动带到，
不需要在 ROS 侧复制任何几何参数。

