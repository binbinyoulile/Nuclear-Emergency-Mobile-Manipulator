# 阶段 1 设计草案：ROS 2 分布式骨架

> 状态：**已实施**（阶段 1 骨架与阶段 2 任务层接入均已落地；阶段 2 的增补项见 5.5）。
> 本文只讲"节点怎么切、接口长什么样、为什么这么切"，不讲实现细节。
> 阶段 0 的接口层（`robot_interface.py`）是本阶段直接复用的地基。

---

## 0. 一句话

把现在"一个进程 + 一个阻塞循环"的仿真程序，按 **设备 I/O / 任务决策 / 目标识别** 切成 3 个 ROS 2 节点，
用 `/clock` 与帧同步保证可复现，**状态机与三个控制器的控制律一行不改**。

---

## 1. 这一阶段要证明什么

| # | 要证明的事 | 验收方式 |
|---|---|---|
| 1 | 任务层在**进程外**只靠话题/服务就能拿到全部信息并完成任务 | 21 个种子全过 |
| 2 | 换掉仿真节点 = 换成真机节点，任务层不用改 | 接口契约冻结；阶段 2 只换驱动 |
| 3 | 分布式之后行为仍然可复现、可回归 | 逐指标对比 + 单次仿真时长在带内 |

**明确不做**（避免范围失控）：

- 不做真机驱动、不做手眼标定、不做 Nav2 接入；
- 不做多机（两台车跑在同一台机器上，用命名空间区分，不引入 WiFi/DDS 组网）；
- 不重写状态机为"事件驱动行为树"——当前阻塞式流程在阶段 1 保留（见 3.5）。

---

## 2. 现状约束（设计必须满足的硬条件）

1. **控制帧 = 20 ms**。物理 1 ms × `STEPS_PER_FRAME=20`，任务层每帧推进一次，即 50 Hz。这个节拍不能变，
   变了下游所有标定（纯追踪增益、轮速斜坡、停靠余量）都要重标。
2. **并发是隐式的**。现在没有线程，靠 8 个 `run_*_concurrent` 循环人工交错推进两辆车。
   这类循环一旦被拆散，两车轨迹都会变（历史教训：给轮速加斜坡把 seed 3 的停靠航向改了 71°）。
3. **行为已被 21 个种子钉死**。重构必须能证明"没变"，否则一次改动引入的偏差会被误判成新 bug。
4. 代码要同时服务两个运行时：Windows 单进程（日常调试、带 viewer）与 Ubuntu ROS 2（集成与迁移）。

---

## 3. 关键架构决策

### 3.1 进程边界画在哪：3 个节点，不是 5 个

最初设想的 `chassis_sim / arm_sim / gripper_sim` 三节点，在仿真里**拆不动**：它们共享同一份 `mjData`，
同一次 `mj_step` 同时推进两车与机械臂。把物理拆进三个进程，只能造出三个互相 RPC 的假节点。

正确的切法是"**按职责，不按设备**"：

| 节点 | 为什么是它 | 阶段 2 之后的对应物 |
|---|---|---|
| `mm_sim_bridge` | 唯一拥有 MuJoCo 的进程；对外一律通过话题暴露设备 | 拆成 2×底盘驱动 + 机械臂驱动 + 夹爪驱动 + 相机/雷达驱动 |
| `mm_mission` | 决策 + 三个控制器 + 指标，不碰任何仿真内部量 | **不变** |
| `mm_target_localizer` | 深度图 → **粗端中心**（map 系），真机上就是感知栈 | **不变**（阶段 3 换手眼标定） |

设备级的**接口**在阶段 1 就切开（话题已按 `/carN/...`、`/car1/arm/...` 分好），
设备级的**进程**等真机到位再切——那时每个驱动天然属于一块硬件。

### 3.2 底盘指令用"轮速 + 前轮转角"，不用 `cmd_vel`

`geometry_msgs/Twist` 是差速底盘的抽象。本项目底盘带转向节（非纯差速），厂家的 CAN 协议收的是
**左右轮目标转速 + 前轮目标转角**。用 `Twist` 意味着在驱动里再造一个自行车模型反解，
把已经验证过的混控代码（`PathFollower.update` 里的转向/轮速合成）搬到一个没被验证的地方。

结论：自定义 `WheelCommand{left, right, steer}`，语义与现在的 `set_wheels()` 完全一致。
将来若接 Nav2，在它和底盘之间加一个 `cmd_vel → WheelCommand` 的转换节点即可，不影响本层。

### 3.3 机械臂逆解放在驱动侧

任务层现在调 `arm.solve_ik(...)`，背后是 mink + `arm_ik_scene.xml` + UR10e 网格。
如果逆解留在任务侧，`mm_mission` 就要背一份机器人模型与网格资源——两个包各存一份模型必然漂移。

改成服务调用：`/car1/arm/solve_ik`（基座坐标系位姿 → 六个关节角 + 残差）。
真机上这正好对应 `ur_robot_driver` 的 `get_inverse_kin` 服务。副作用是任务包变成**纯 Python + numpy + rclpy**，
不依赖 MuJoCo、不依赖模型文件。

### 3.4 时间：`/clock` 由仿真节点发布，帧同步

- `mm_sim_bridge` 每帧发布 `/clock`（+0.02 s），所有节点 `use_sim_time:=true`；
- 帧内顺序：应用上一帧指令 → `mj_step × 20` → 发布状态与 `/clock` → 等本帧指令（**50 ms 墙钟超时**）→ 下一帧。

超时机制保证两件事：仿真不会跑在决策前面（状态永远是新的），任务层卡住也不会把仿真冻死（超时计数会打印）。

指令延迟恒定为 **1 帧（20 ms）**——与现在的行为一致（现在也是"算完 → 下一帧生效"）。
仿真时间与墙钟解耦：机器慢只是整体慢，不影响判定，回归不看墙钟。

### 3.5 任务层不做"事件驱动重写"

现在 `run_turn_with_secondary` 这类循环是"边推进边等条件"。ROS 化有两种写法：

- **A（推荐）**：任务逻辑跑在独立线程，`Clock.step()` 变成"阻塞等下一帧 `/clock`"；
  订阅回调在另一线程把状态写进缓存。任务层代码**形状不变**，并发交错关系不变，因此轨迹不变。
- B：把状态机改成定时器 + 回调，阻塞循环全部拆成状态。正确但工作量大一个数量级，且会改变两车交错的时序。

阶段 1 选 A。代价是任务节点里有一处"线程 + 条件变量"的样板代码（约 60 行，全部在适配器里），
换来的是**控制律与并发语义零改动**。B 留到真机上多进程、多机器时再评估。

---

## 4. 节点清单与数据流

```
                    ┌──────────────────────┐
                    │   mm_sim_bridge      │  MuJoCo（物理 + 传感器 + TF + /clock）
                    │  car1 / car2 / 臂 / 爪│
                    └───────┬──────▲───────┘
        /clock, /tf, 状态话题  │      │  wheel_command, arm/joint_command, gripper/command
                    ┌───────▼──────┴───────┐        ┌──────────────────────┐
                    │    mm_mission        │◄───────┤ mm_target_localizer  │
                    │ 状态机 + 3 个控制器   │ 目标位姿 │ 图像 → 目标三维坐标   │
                    └──────────────────────┘        └──────────▲───────────┘
                                                               │ 图像 / 深度 / 内参
                                                          （由 sim_bridge 提供）
```

---

## 5. 接口契约

### 5.1 话题

| 话题 | 类型 | 发布 → 订阅 | 频率 | QoS |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim → all | 50 Hz | — |
| `/tf` | `tf2_msgs/TFMessage` | sim → localizer, rviz | 50 Hz | reliable, depth 10 |
| `/car{1,2}/wheel_command` | `mm_msgs/WheelCommand` | mission → sim | 50 Hz | reliable, depth 1 |
| `/car{1,2}/odom` | `nav_msgs/Odometry` | sim → mission | 50 Hz | reliable, depth 1 |
| `/car{1,2}/joint_states` | `sensor_msgs/JointState` | sim → mission | 50 Hz | reliable, depth 1 |
| `/car{1,2}/obstacle_summary` | `mm_msgs/ObstacleSummary` | sim → mission | 10 Hz | reliable, depth 1 |
| `/car1/arm/joint_command` | `std_msgs/Float64MultiArray` | mission → sim | 50 Hz | reliable, depth 1 |
| `/car1/arm/joint_states` | `sensor_msgs/JointState` | sim → mission | 50 Hz | reliable, depth 1 |
| `/car1/gripper/command` | `mm_msgs/GripperCommand` | mission → sim | 50 Hz | reliable, depth 1 |
| `/car1/gripper/state` | `mm_msgs/GripperState` | sim → mission | 50 Hz | reliable, depth 1 |
| `/car1/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | sim → localizer | 触发式 | reliable, depth 1 |
| `/car1/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (32FC1, m) | sim → localizer | 触发式 | reliable, depth 1 |
| `/car1/camera/color/camera_info` | `sensor_msgs/CameraInfo` | sim → localizer | 启动 1 次 | reliable, transient_local |
| `/perception/target` | `mm_msgs/TargetEstimate` | localizer → mission | ≤10 Hz | reliable, depth 1 |
| `/sim/target_truth` | `geometry_msgs/PoseStamped` | sim → mission | 10 Hz | reliable, depth 1 |
| `/sim/object_pose` | `geometry_msgs/PoseStamped` | sim → mission | 50 Hz | reliable, depth 1 |
| `/car1/lidar/ignore_peer` | `std_msgs/Bool` | mission → sim | 按需 | reliable, transient_local |
| `/sim/contacts` | `std_msgs/Int32` | sim → mission | 50 Hz | reliable, depth 1 |
| `/mission/status` | `mm_msgs/MissionStatus` | mission → rviz/log | 1 Hz | reliable, depth 1 |
| `/mission/result` | `std_msgs/String`（单行 JSON） | mission → 回归脚本 | 结束 1 次 | reliable, transient_local |

`/sim/*` 前缀是约定：**只有仿真才有的量**。真机上这些话题不存在，任务层必须能降级（接触数视为 0、真值视为 NaN）。

### 5.2 服务

| 服务 | 类型 | 调用 → 提供 | 用途 |
|---|---|---|---|
| `/car1/camera/capture` | `std_srvs/Trigger` | mission → sim | 触发一帧离屏渲染并发布图像（真机换成相机触发/取流） |
| `/car1/arm/solve_ik` | `mm_msgs/SolveIK` | mission → sim | 世界坐标 + 工具姿态（固定基座系）→ 关节角 + 残差 |
| `/sim/snapshot` | `mm_msgs/Snapshot` | mission → sim | 保存关键帧截图（仅 `--snapshot`） |

**不引入 action**：阶段 1 的运动执行都在任务层或驱动内部完成，没有"长时间目标 + 反馈 + 结果"的语义需求。
真机上底盘导航、机械臂轨迹会需要 action，届时按 `start/tick/active` 三段式映射，任务层不必重写。

### 5.3 消息定义（`mm_msgs`，6 个 msg + 2 个 srv）

```text
WheelCommand.msg     float64 left      # 左侧轮角速度 rad/s
                     float64 right     # 右侧轮角速度 rad/s
                     float64 steer     # 前轮转角 rad（正 = 左转）

ObstacleSummary.msg  std_msgs/Header header
                     bool   front_blocked
                     float64 front_min, front_lateral, front_lateral_signed
                     float64 left_min, right_min, rear_min, rear_clear

GripperCommand.msg   float64 position     # 0 全开 … 255 全闭（与 2F-85 寄存器同量纲）
                     float64 max_effort   # N

GripperState.msg     std_msgs/Header header
                     float64 position, effort
                     bool    moving

TargetEstimate.msg   std_msgs/Header header
                     geometry_msgs/Point position   # map 系，m；粗端中心（抓取点）
                     float64 confidence
                     bool    valid

MissionStatus.msg    string stage
                     bool   done, ok
                     float64 sim_time        # s

SolveIK.srv          geometry_msgs/Point target            # map 坐标系
                     geometry_msgs/Quaternion orientation  # 固定基座坐标系，wxyz
                     ---
                     float64[6] joint_positions
                     float64 position_error     # m

Snapshot.srv         string name
                     ---
                     bool ok
```

`ObstacleSummary` 的字段与 `LidarScanner` 的 7 个距离量 + 1 个阻塞标志一一对应，**数值定义不变**，
所以避障判据不需要重新标定。雷达原始点云在阶段 1 不入话题（一帧是 16 线 × 120 方位 = 1920 个回波，
只为得到 8 个数不值得走 DDS）；真机上由一个 `lidar_filter` 节点产生同样的 summary。

`TargetEstimate` 没有用 `vision_msgs/Detection3DArray`：当前是单类别单目标，数组 + bbox + hypothesis
三层结构在这里是空转包装。接入真实检测器（多类别、多目标）时再换。

### 5.4 坐标系

| frame | 内容 | 发布者 |
|---|---|---|
| `map` | 世界系（场地原点） | — |
| `car{1,2}/base_link` | 底盘中心 | sim_bridge（50 Hz，父系 `map`） |
| `car1/camera_link` | D435i 光心 | sim_bridge（50 Hz，父系 `map`） |
| `car1/arm_base_link` | 机械臂基座（任务层用它把臂座系目标换算到世界系） | sim_bridge（50 Hz，父系 `map`） |
| `car1/pinch` | 夹持点（pinch 站点），投放对中要量"夹持点 → 质心" | sim_bridge（50 Hz，父系 `map`） |

**父系一律是 `map`**，因为桥发布的值全是 MuJoCo 世界量（`xpos`/`xmat`/`site_xpos`）。
曾经把 `car1/arm_base_link` 挂在 `car1/base_link` 下、`car1/pinch` 挂在 `car1/arm_base_link`
下，但给的值仍是世界坐标，TF 合成时把车体/臂座的位置又加了一遍：世界目标整体偏 ~20 m，
任务卡在抓取阶段（IK 残差 18.7 m）。所以 `_send_world_tf()` 干脆不开放 parent 参数——
值是世界量，父帧只能是 `map`，写错都写不出来。真机上这几帧由各自驱动发布真正的相对变换。

**`car1/arm_base_link` 不是"固定基座坐标系"**：`arm_ik_scene.xml` 的 base 体相对底盘臂座
转了 180°（`FIXED_BASE_ROT = diag(-1,-1,1)`），而 `ArmInterface.start_fixed` 的入参是固定
基座系。任务层只在这一个方法上需要翻回来，顺序见 `RosArm.start_fixed`；漏掉不会报错，
残差仍然只有几毫米，机械臂只是安静地走到关于臂座镜像的位置。

相机是**眼在手上**（装在 UR10e 末端 `wrist_3_link` 上），仿真里直接取世界位姿发布，不在阶段 1 拼六轴链；
真机阶段由 `ur_robot_driver` + `robot_state_publisher` 给出完整链。

### 5.5 阶段 2 增补（任务层拆分带出来的三个契约）

任务层搬到仓库根的 `mission_core.py` 之后，有四样东西单进程里是"直接读全局量"、分布式的必须走契约：

| 需要的信息 | 单进程来源 | ROS 契约 |
|---|---|---|
| 手中物体的**实时**位姿（投放对中、抓取成败、验收） | `DATA.qpos[OBJ_QADR:]` | `/sim/object_pose` |
| 夹持点世界坐标（量"夹持点 → 质心"偏移） | `DATA.site_xpos[pinch]` | `car1/pinch` TF |
| 雷达"是否把同伴车当动态障碍"（末段直线入位要临时关掉） | `lidar.dynamic_roots` | `/car1/lidar/ignore_peer` 的 `Bool` |
| 每帧渲染回调（雷达点云叠加） | `sim.viewer` | `RobotIO.frame_hook`，ROS 侧为 `None` |

`/sim/target_truth` 与 `/sim/object_pose` **不能混用**：前者是生成时的落点（姿态恒为单位四元数），
只用于视觉误差验收；后者才是那根正在被搬运的杆。历史上把两者当同一个用过，结果抓取判据永远为假。

---

## 6. 一帧的生命周期

```
帧 k（sim 时间 t_k = k × 0.02 s）
 1. sim_bridge 应用缓冲指令 → mj_step × 20（1 ms × 20）→ 更新派生量（雷达每 10 Hz 才扫一次）
 2. sim_bridge 发布 /carN/odom、joint_states、obstacle_summary、/tf、/sim/target_truth，最后发布 /clock = t_k
 3. mission 的 Clock.step() 等到 t_k → 读缓存 → 任务层计算 → 发布两车 wheel_command、臂、爪
 4. sim_bridge 收到两车 wheel_command（超时 50 ms）→ 写入缓冲 → 进入帧 k+1
```

要点：

- 第 4 步的等待是**只等底盘指令**。机械臂、夹爪按各自 50 Hz 流式更新，最多滞后一帧（臂运动慢，无感）；
  视觉触发是带外请求，不参与帧同步。
- 阶段 2 补了两条：①机械臂动作、视觉等待等阶段任务层不刷新底盘指令，`RosClock.step()`
  每帧把两车最近一次的轮速指令补发一遍（仿真里 ctrl 会保持，话题里不会），否则桥每帧都超时；
  ②视觉等待期间（`request_estimate()`）也在推进帧并补发指令——图像与 TF 都在帧边界产生，
  只有推进才会出新帧，这样仿真时间始终由任务层驱动而不是凭墙钟自由跑。
- 超时会在日志里累加计数，出现非零即说明任务层某处卡了，是排查线索而不是故障。
- 帧同步让仿真"跑得比墙钟快"是安全的（不会出现状态堆积），所以回归不必等实时。

---

## 7. 代码组织

### 7.1 仓库布局

```
mobile_manipulator/
├─ dual_car_pick_place.py          # Windows 单进程入口（保留，行为不变）
├─ mobile_pick_place.py            # MuJoCo 底层（保留，行为不变）
├─ dual_car_scene.xml / models/    # 场景与模型（不变）
├─ scripts/                        # fetch_models.py / run_matrix.py / compare_metrics.py
│                                  # + run_ros_matrix.py（新增）
└─ ros2_ws/src/
   ├─ mm_msgs/                     # ament_cmake：6 msg + 2 srv
   ├─ mm_mission/                  # ament_python：状态机、3 个 follower、插值、指标、Ros* 适配器、mission 节点
   ├─ mm_sim/                      # ament_python：CarController、LidarScanner、场景装配、Sim* 适配器、sim_bridge 节点
   ├─ mm_perception/               # ament_python：target_localizer 节点
   └─ mm_bringup/                  # ament_cmake：launch、config、rviz
```

### 7.2 两个运行时怎么共用代码

Windows 侧没有 colcon，Ubuntu 侧才是 ament 包。**代码只有一份**，放在 `ros2_ws/src/` 里：

- Windows：`dual_car_pick_place.py` 顶部把 `ros2_ws/src` 加入 `sys.path`，然后 `from mm_mission import ...`。
  入口脚本里两行引导，使用者看不到差别，仍是 `python .\dual_car_pick_place.py --seed 6`。
- Ubuntu：`colcon build --symlink-install` 正常安装，节点代码干净无路径魔法。

这是"双轨期"的既定残留：等 Windows 不再是主运行环境（阶段 2），把 `mobile_pick_place.py` 与
`models/` 收进 `mm_sim`，路径引导随之删除。

### 7.3 包依赖

```
mm_msgs  ◄── mm_sim ──► mm_mission ◄── mm_perception（仅 msgs）
   ▲            ▲            ▲
   └────────────┴────────────┘        mm_bringup 依赖全部（只放 launch/config）
```

`mm_sim` 与 `mm_mission` 之间**没有代码依赖**，只有消息契约。这是本阶段最重要的一条：
谁都不许 import 对方。

---

## 8. 任务层改造映射

| 现有写法 | ROS 2 中对应 |
|---|---|
| `io.clock.step()` 推进 `mj_step` | 等下一帧 `/clock`（物理在 sim_bridge 里推进） |
| `car.set_motion(l, r, s)` 写 ctrl | 发 `/carN/wheel_command` |
| `car.pose()` 读 `DATA.qpos` | 读缓存的 `/carN/odom` |
| `car.wheel_speeds()` | 读缓存的 `joint_states.velocity` |
| `car.lidar.front_min` 等 7 个距离量 + 阻塞标志 | 读缓存的 `obstacle_summary` |
| `car.contacts_with(other)` | 读 `/sim/contacts` |
| `arm.set_joint_targets(q)` / `tick()` | 插值在任务侧计算，发 `/car1/arm/joint_command` |
| `arm.solve_ik(...)` | 调 `/car1/arm/solve_ik` |
| `gripper.start/tick/hold()` | 插值在任务侧计算，发 `/car1/gripper/command` |
| `gripper.force()` | 读 `/car1/gripper/state` |
| `perception.estimate_target()` | 触发 `/car1/camera/capture`，等一帧新的 `/perception/target` |
| `perception.object_pose()` | 订阅 `/sim/object_pose`（真机由腕部相机给出） |
| `arm.tool_position()` | 查 `car1/pinch` TF |
| `lidar.peer_avoidance` | 发 `/car1/lidar/ignore_peer` |
| `capture_snapshot()` | 调 `/sim/snapshot` |
| 控制器内部的 `base.DATA.time`（约 20 处） | 改成注入的 `io.clock.now`（阶段 2 已还清） |

**保持不变**：`DualCarMission.run()` 的七阶段流程、`PathFollower` / `SkidTurnFollower` /
`ReverseStraightFollower` 的控制律、8 个 `run_*_concurrent` 的交错方式、`metrics` 的全部字段与判据。

需要小幅改动的接口（均为收敛而不是扩张，阶段 2 已全部落地）：

1. `Clock` 注入给三个 follower（还上面的时钟债）；`PathFollower` 的 `frame_dt` 也取自时钟；
2. `PerceptionInterface` 由 `capture()+detect_target()+object_truth()` 收敛为
   `estimate_target()+object_pose()`（感知拆出去后，任务层不该关心像素，也不该有"真值"这个概念）；
3. `RobotIO.viewer` 换成每帧渲染回调 `frame_hook`（任务层不认识 viewer 这个 MuJoCo 对象）；
4. `LidarInterface` 独立成端口（底盘持有它），`ChassisInterface` 补 `set_motion` 默认实现、
   `left/right/steer_cmd` 输出缓存与 `min_other_separation`。

---

## 9. 回归与验收

### 9.1 两轨验证

| 轨道 | 命令 | 判据 | 目的 |
|---|---|---|---|
| A 单进程（Windows） | `run_matrix.py --seeds 0-20` + `compare_metrics.py` | 21/21 且与 `reports/baselines/stage0_refactor.csv` 逐位一致（1e-9） | 证明本阶段的代码搬动没有改变行为 |
| B 分布式（Ubuntu） | `ros2 launch mm_bringup sim_mission.launch.py seed:=N` × 21 | 21/21 + 指标在物理带内 | 证明分布式架构可用 |

轨道 A 是每次重构的护栏，轨道 B 是本阶段的交付物。两条都跑，才能区分"拆分错了"和"时序变了"。

### 9.2 轨道 B 的指标带（物理判据，不是基线范围）

| 指标 | 判据 |
|---|---|
| 视觉定位误差 | ≤ 30 mm（与现有硬门限一致） |
| 落桶判据 | 整根杆在桶口投影内（端点离内壁 ≥ 3 mm）、低于桶沿、高于桶底 |
| 停靠时刻车身投影上沿 | < 20.700 m |
| 全程最小中心距 | ≥ 0.82 m |
| 车车接触数 | 0 |
| 回起点误差 | < 0.20 m |
| 帧推进时的指令超时（收尾日志 `指令超时=`） | 0（只统计两车都下发过指令之后；启动空转记 `空转帧`，不计入） |
| 单次仿真时长 | 仅记录、不判定（真机速度口径后单进程 69.4~70.9 s，ROS 侧更长） |

不做逐位比对：跨进程必然有调度抖动，要求 1e-9 一致是不现实的，也不该成为目标。

### 9.3 记录与复现

- `scripts/run_ros_matrix.py`：逐种子起 launch → 订阅 `/mission/result` → 落 `reports/ros_regression.csv`；
- 并行跑：topic 名写死，同一 DDS domain 不能并跑，每个种子给独立 domain
  （`ROS_DOMAIN_ID=$((100+seed))` + `ROS_LOCALHOST_ONLY=1`）配 `xargs -P N`，`N = min(4, nproc/2)`。
  合并结果前先看每份收尾日志的 `指令超时=` 是否为 0：桥等两车轮速用的是墙钟 `command_timeout`（50 ms），
  CPU 争用会超时并用旧指令推进，等于偷偷给某辆车加了指令延迟；警告行本身是节流的（第 1~3 次 +
  每 200 次一行），不能当超时次数用；
- 录一条完整 `ros2 bag`（`/clock` + 全部话题），用于离线回放与"任务层脱离仿真"的验证（阶段 1.5 可选）。

---

## 10. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| VM 性能不够（MuJoCo 现在是 ~1.0× 实时） | 单次仿真变慢，21 个种子跑很久 | 帧同步自定速（不等墙钟）；回归用 `--headless`；必要时降到 30 Hz 控制帧并重跑 A 轨对照 |
| DDS 抖动导致指令晚到 | 轨迹轻微漂移 | 单机 `ROS_LOCALHOST_ONLY=1` + CycloneDDS；状态话题 depth=1（要新不要全）；帧同步吸收抖动 |
| 图像带宽（640×480×3 + 深度） | 拖慢整条链 | 触发式采集（全程约 10 帧），不持续推流 |
| 双端代码漂移 | 两边行为不一致 | 只认 git；禁止用共享文件夹传源码；每次改动先跑轨道 A |
| 拆分引入隐蔽行为变化 | 回归失败难归因 | 按第 11 节顺序推进，每步都有独立验收；ROS 工作放在 `feature/ros2-stage1` 分支，master 始终可跑 |

---

## 11. 实施顺序（每步都有独立验收）

| 步 | 内容 | 产出 / 验收 | 状态 |
|---|---|---|---|
| 0 | VM 装 ROS 2 Humble，验证 `ros2 doctor`、`colcon build`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；仓库克隆到 ASCII 路径 | `ros2 run demo_nodes_cpp talker/listener` 通 | ✅ |
| 1 | `mm_msgs` 全量定义 | `colcon build` + `ros2 interface show` 逐条核对；不含任何运行时逻辑 | ✅ |
| 2 | `mm_sim` + `sim_bridge`：物理、传感器、TF、`/clock`、指令接收 | **不改任务层**，单跑 sim_bridge，`ros2 topic hz` 全部达标，rviz 里能看到两车与 TF | ✅ |
| 3 | 还时钟债 + `mm_mission` 的 `Ros*` 适配器 + mission 节点 | 轨道 A 仍逐位一致（证明搬动无误）；seed 6 端到端跑通 | ✅ 阶段 1 完成（冒烟）|
| 4 | 感知拆分：`mm_perception` 替换任务层内的检测 | 视觉误差复测 ≤ 30 mm | ✅ 同帧逐位一致（0.000 mm）|
| 5 | 任务层拆分：`mission_core.py` + 端口扩展 + 桥补 `/sim/object_pose` / `car1/pinch` | 轨道 A 21/21 且与基线逐位一致；轨道 B 21 种子（Ubuntu） | 轨道 A ✅ / 轨道 B ✅ 21/21（`e09370d`）|
| 6 | `run_ros_matrix.py` + bag 录制 + 指标带对比 | `reports/ros_regression.csv`，21/21 | 待做 |
| 7 | 文档与版本：ARCHITECTURE 增 ROS 章节、README 增运行命令、打 `v1.1.0` | 仓库自洽，新人按 README 能跑起来 | 待做 |

步 3 是分水岭：过了这一步，"任务层脱离仿真"就已经被证明；步 5 把**同一份**任务代码
搬到 `mission_core.py` 后，两个后端不再各有一份状态机。

---

## 12. 需要你拍板的四件事

1. **感知是否现在就拆成第 3 个节点？**
   推荐拆（架构完整、TF 与图像链路一次做对，真机阶段不用再动）。
   替代：阶段 1 先留在任务节点内，阶段 3 随手眼标定一起拆——少约 150 行新代码与一次视觉回归风险。
2. **底盘指令用"轮速 + 转角"（推荐，理由见 3.2）还是标准 `cmd_vel`？**
   选 `cmd_vel` 就得在驱动里引入自行车模型，且要重新验证混控。
3. **仓库结构：就地引导（`ros2_ws/` + 入口脚本两行路径）还是这一阶段整体重排成标准 ROS 工作区？**
   推荐就地引导——本阶段的风险预算应该花在分布式设计上，而不是 3000 行的搬迁。
4. **阶段 1 之后，Windows 单进程入口是否保留？**
   推荐保留（本地迭代不用起 ROS，21 个种子的快速回归仍然可用）。
