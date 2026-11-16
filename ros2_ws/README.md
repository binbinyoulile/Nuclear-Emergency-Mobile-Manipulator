# ros2_ws：阶段 1 ROS 2 分布式骨架

把单进程仿真程序按 **设备 I/O / 任务决策 / 目标识别** 拆成三个节点，用 `/clock` 与帧同步保证可复现。
设计依据见 [`../docs/ros2_stage1_design.md`](../docs/ros2_stage1_design.md)。

```
mm_sim_bridge ──(状态话题 + /clock + /tf)──▶ mm_mission
      ▲                                          │
      └──────────(轮速 / 关节 / 夹爪指令)─────────┘
                    mm_target_localizer ──(目标位姿)──▶ mm_mission
```

## 包

| 包 | 类型 | 内容 |
|---|---|---|
| `mm_msgs` | ament_cmake | 6 个 msg + 2 个 srv（轮速、雷达摘要、夹爪、目标、状态、逆解、截图） |
| `mm_sim` | ament_python | `sim_bridge`：独占 MuJoCo，物理 / 传感器 / `/clock` / 帧同步 |
| `mm_mission` | ament_python | `mission_node`：任务线程 + 硬件端口适配器 + 冒烟序列 |
| `mm_perception` | ament_python | `target_localizer`：深度图 + TF → **粗端中心**世界坐标 |
| `mm_bringup` | ament_cmake | launch 文件 |

依赖方向：`mm_sim` 与 `mm_mission` **互不 import**，只共享 `mm_msgs` 的消息契约。
`mm_mission` 不依赖 MuJoCo，也不依赖机械臂模型。

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
`dual_car_scene.xml` 与 `models/`。脱离源码树安装时改为显式传参 `repo_root:=<仓库根>`。

## 运行

```bash
# 全链路（仿真 + 感知 + 任务）
ros2 launch mm_bringup sim_mission.launch.py seed:=6

# 看画面
ros2 launch mm_bringup sim_mission.launch.py seed:=6 use_rviz:=true

# 只起仿真侧，验证 /clock、话题频率与 TF（跑满 500 帧自动退出）
ros2 launch mm_bringup sim_only.launch.py seed:=6 max_frames:=500
```

注意两件事：

- **`sim_only` 不含感知节点**，所以 `/perception/target` 在那条 launch 下根本不会出现（那是设计如此，不是故障）。要看感知必须用 `sim_mission`。
- **`/clock` 的 50 Hz 指仿真时间**（1 帧 = 20 ms），不是墙钟频率：桥用 `realtime=False`，任务节点在跑时会被 CPU 速度推进（墙钟可能几百 Hz）；`sim_only` 下没有任务层发轮速，每帧都在等指令超时 50 ms，墙钟反而只有 ~20 Hz 并刷 `等待两车指令超时` 警告。**唯一可靠的判据是收尾日志 `frames=N sim_time=T`**，即 `T == N × 0.02`。

检查链路：

```bash
ros2 topic hz /clock                # 50 Hz（跑满即停，属于正常）
ros2 topic hz /car1/odom            # 50 Hz
ros2 topic hz /car1/obstacle_summary # 10 Hz
ros2 run tf2_tools view_frames      # map → carN/base_link、car1/arm_base_link、car1/camera_link
```

任务结束会打印一行 `RESULT_JSON:`，同时发布到 `/mission/result`。

## 话题与参数

契约表见设计文档第 5 节。本工作空间新增的节点参数：

| 节点 | 参数 | 默认 | 说明 |
|---|---|---|---|
| `mm_sim_bridge` | `repo_root` | 由 launch 反推 | 仓库根目录 |
| | `seed` | `-1` | `-1` 表示每次随机 |
| | `command_timeout` | `0.05` | 帧同步等待两车指令的墙钟超时，s |
| | `max_frames` | `0` | 跑满多少帧后退出，0 表示一直跑 |
| `mm_mission` | `frame_dt` | `0.02` | 控制帧时长，与仿真保持一致 |
| | `frame_timeout` | `5.0` | 等 `/clock` 的墙钟超时 |
| | `ready_timeout` | `30.0` | 等首帧状态的超时（模型加载要几秒） |
| | `seed` | `-1` | 只用于指标，场景随机化在仿真侧 |
| `mm_target_localizer` | `roi_x_min` / `roi_x_max` | `-0.30` / `0.30` | 搜索窗 X 范围（map 系），默认对齐绿框内缩 5 cm |
| | `roi_y_min` / `roi_y_max` | `20.74` / `21.26` | 搜索窗 Y 范围，默认对齐绿框内缩 4 cm |

## 当前状态（阶段 1 骨架）

- 已实现：三节点的完整链路、`/clock` 帧同步、帧边界指令生效、TF、触发式取图、
  逆解服务、指标输出；
- **感知已同步到最新被操作对象**：`mm_perception` 不再是 HSV 黄色分割，而是与单进程版
  同一套深度算法（反投影 → 地平面拟合 → 离地 0.8~30 mm → 连通域 + 搜索窗 → PCA 定轴 →
  两端顶面 1 mm 高差判粗端），输出**粗端中心**；同一帧喂两边，结果逐位一致（实测 0.000 mm）。
  物理侧的杆与 Ø240 屏蔽桶由 `mm_sim` 动态 import 仓库根的 `dual_car_pick_place` 自动带到，
  不需要在 ROS 侧复制任何几何参数；
- `mm_mission` 里的任务序列目前是 `smoke.py`：稳定 → 直行 1.2 m → **开到目标正南 1.41 m
  的观测位** → 感知出粗端中心并与真值比对 → 机械臂沿车头前 0.5 m、上方 0.55 m 移动 →
  夹爪开合。它不是七阶段任务流程，但已经能在真实距离上验证整条链路（观测位用
  `/sim/target_truth` 当测试夹具，任务层本身仍然只认感知输出）。

  冒烟判据（离线用单进程后端跑过 seed 2/4/6/7/14，Ubuntu 端 seed 6 实测一次）：

  | 指标 | 期望 |
  |---|---|
  | `car1_travel_m` | ≥ 1.0 m |
  | `stance_error_m` | ≤ 0.15 m（实测 0.103~0.110） |
  | `vision_error_mm` | ≤ 30 mm（实测 1.0~2.4 mm） |
  | `gripper_close_effort_n` | 只需 > 0；**空夹约 0.2~0.6 N 是正常值**（12 N 是夹爪怼到自身臂杆才会出现的饱和值） |
  | `contacts` | 0 |

  机械臂目标必须在**车头前方**：正上方正好落在臂座轴线上，UR10e 在固定下压姿态下
  解不出来（实测残差 163 mm），偏 0.5 m 后降到 2 mm 量级。

- 七阶段状态机、三个路径控制器、21 个种子的 ROS 回归矩阵属于阶段 2 的工作，
  计划见设计文档第 11 节。
