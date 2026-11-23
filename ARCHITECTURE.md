# 架构说明

> 本文档面向"要改这个项目的人"和"要把它迁到真机/ROS2 的人"，只讲结构、契约与数据流。
> 项目能力、快速开始、回归基线见 [README.md](README.md)。

## 1. 文档维护规则

- 改动了模块边界、接口签名、状态机阶段或回归指标字段时，必须同步本文档对应章节。
- 本文档不写"将来可能会做"的设想，只写已经实现的结构；路线图放在对话或 README 的里程碑里。

## 2. 项目目标与当前边界

**最终目标**：基于多模态感知（雷达 + 深度相机）的核电应急机器人运动控制 Sim2Real 闭环验证。

**当前边界（`master`；冻结基线是 tag `v1.0.0`，之后被操作对象换成不锈钢监测棒、屏蔽桶换成 Ø400 圆筒）**：

- 已实现：双车协同全流程仿真（雷达避障 → 视觉识别 → 侧面抓取 → 动态停靠 → 协同投放 → 倒车脱离 → 精准返航），75 kg 真实参数重载底盘，21 个随机种子的可回归验收；阶段 1 的三节点 ROS 2 骨架（`ros2_ws`）已在 Ubuntu 22.04 + 原生 Humble 上 `colcon build` 通过；阶段 2 把任务层拆到零 MuJoCo 的 `mission_core.py`，单进程与 ROS 2 共用同一份实现，端口扩到 `Lidar / object_pose / tool_position`，轨道 B（ROS 侧 21 个种子）同样 21/21，并由 `scripts/run_ros_matrix.py` 脚本化（并行 4 路、墙钟 30.6 min）；真车避障链缺的"点云 → 雷达摘要"节点已在 `mm_perception/lidar_summary` 补齐，并与桥逐帧对照（950 帧、7 项距离量最大偏差 `9.1e-8 m`）。
- 未实现：真机 CAN 驱动（**报文格式已从厂家源码查实，见 `docs/vehicle_workspace_refs.md`，不再是卡点**，缺的只是上车）、手眼标定、实车参数辨识。软件侧能做的已做完：`scripts/bench_chassis.py`（架空/落地台架标定）、`docs/real_machine_calibration.md`（公差表：轮径 0.5%、相机 2 mm/0.1°、雷达 5 cm）、`mm_perception/lidar_summary`（点云→雷达摘要，与桥对照 950 帧、9.1e-8 m）。明确不做：bag 回放（只覆盖感知决策段——不录服务、不含物理结局）。待上车实测：摘要节点的安装高度与自身盲区、轮径真值、相机手眼、转矩指令的单位（源码只给了 ±1910 的钳位）。

## 3. 跨端协作环境

| 角色 | 位置 | 职责 |
|---|---|---|
| Windows 宿主机 | `D:\桌面\机械臂\mobile_manipulator` | 主代码库、MuJoCo 仿真、回归矩阵、Git 推送 |
| Ubuntu 虚拟机 | `192.168.214.128` | ROS 2 构建与真机驱动（阶段 1 起启用），带 GUI |
| 跨端共享区 | `D:\桌面\share` ↔ `/mnt/hgfs/share` | 唯一协作区：交接状态块与制品交换，**不是备份盘** |

协作约定（详见共享区 `GLOBAL_RULES.md`）：

- 开工前读 `GLOBAL_RULES.md` 与项目状态块；写入前拿锁（`.win_lock` / `.ubuntu_lock`），完工删锁。
- 交接只走"项目 README 顶部状态块（Owner / State / Updated / Next）"，不另建 PROGRESS/TODO 文件。
- 文本一律 UTF-8 无 BOM + LF；共享区路径写相对路径，两端各自换算绝对路径。
- 本机为规避 MuJoCo 对 Windows 中文路径的兼容问题，另建 ASCII junction `D:\robot_mobile` 指向本项目；运行期不要删除或重建它。

## 4. 目录与职责

```
mobile_manipulator/
├─ mission_core.py          # 任务层：七阶段状态机、三个路径控制器、抓取/投放编排、任务常量（零 MuJoCo）
├─ dual_car_pick_place.py   # 单进程入口：Sim* 适配器、场景初始化、深度视觉、雷达点云叠加、截图与指标
├─ mobile_pick_place.py     # MuJoCo 底层：MJCF 加载、mink IK、重力补偿、ArmMotion/FingerMotion、Simulation
├─ robot_interface.py       # 抽象层：Clock / Lidar / Chassis / Arm / Gripper / Perception / RobotIO（不含 MuJoCo）
├─ dual_car_scene.xml       # 主场景（30×30 m、红线、绿框、两车装配）
├─ arm_ik_scene.xml         # 固定基座 IK 场景
├─ models/                  # UR10e 复合模型 + 监测棒网格（chopstick.obj 入库，Menagerie 的 assets 由脚本下载）
├─ scripts/                 # fetch_models.py / run_matrix.py / compare_metrics.py / check_ports.py
├─ ros2_ws/src/             # ROS 2 五包：mm_msgs / mm_sim / mm_mission / mm_perception / mm_bringup（Humble 构建通过）
├─ docs/screenshots/        # README 配图
└─ reports/                 # regression.csv + baselines/（入库）、logs/（本地忽略）
```

**分层方向是单向的**：`mission_core` → `robot_interface`（只认端口）；两个后端
（`dual_car_pick_place` / `ros2_ws/.../mm_mission`）→ `mission_core`。
`mission_core` 不 import `mujoco`、不读任何仿真全局量，`scripts/check_ports.py` 会在
提交前验证两个后端都实现了全部端口。

## 5. 核心模块职责

| 模块 | 关键符号 | 职责 | 依赖 |
|---|---|---|---|
| 抽象层 | `robot_interface.RobotIO` | 定义硬件端口：时钟、两车底盘（含雷达视图）、机械臂、夹爪、感知；`frame_hook` 是每帧渲染回调 | 仅 numpy |
| 任务层 | `mission_core.DualCarMission` | 七阶段状态机、并发调度（`run_*_concurrent`）、指标采集与验收判据 | RobotIO |
| 路径控制 | `mission_core.PathFollower` | 纯追踪 + 雷达局部避障 + 终点直行 + 低速掉头；横向容差与超时保护 | Chassis / Lidar / Clock |
| 转向控制 | `mission_core.SkidTurnFollower` | 差速原地转（方向锁存、越界即停、可选轮速斜坡） | Chassis / Clock |
| 倒车控制 | `mission_core.ReverseStraightFollower` | 锁车道 + 前轮转角保持的直线倒车，后向净空限速 | Chassis / Lidar / Clock |
| 抓取/投放编排 | `mission_core.execute_grasp_concurrent / execute_cooperative_place` | 接近-下探-闭合-提升-收回；按实测杆轴把质心对到桶心后在桶口上方松爪 | Arm / Gripper / Perception |
| 仿真适配 | `SimClock / SimArm / SimGripper / SimPerception` | 把端口翻译成 MuJoCo 的 ctrl/qpos 读写；`build_sim_robot_io()` 是唯一装配点 | mobile_pick_place |
| 底盘 | `CarController`（实现 `ChassisInterface`） | 四轮轮速 + 前轮转角下发、世界位姿/速度回读、接触统计、雷达扫描 | base |
| 感知 | `get_camera_data / depth_to_world / detect_steel_target / process_camera_frame` | D435i 深度视觉四层：取图 → 反投影点云 → 地平面拟合与离地分割 → PCA 定轴、凭两端高差判粗端 | mujoco.Renderer |
| 机械臂 | `ArmMotion`（经 `SimArm` 暴露） | 关节目标下发 + 平滑插值 + 重力补偿；IK 用 mink 且以当前关节角为种子 | mink |
| 夹爪 | `FingerMotion`（经 `SimGripper` 暴露） | 单执行器开合插值（0 开 / 255 闭） | base |
| 入口 | `main()` / `initialize_mission()` | 参数解析（`--seed/--headless/--snapshot/--backend`）、场景与目标随机化、viewer | 全部 |
| ROS 适配 | `mm_mission.adapters.Ros*` | 把端口翻译成话题/服务/TF；`RosClock.step()` 等 `/clock` 并补发两车轮速指令 | robot_interface |

## 6. 数据流

### 6.1 任务阶段

```
阶段1 起点
  └─阶段2 CAR1 雷达避障 → 观察点(Y≈18.4) 粗定位
      └─阶段3 D435i 精定位：先开到 1.41 m 测量位再采一帧 → 通知 CAR2 出发 → CAR1 逼近目标正南 0.85 m
          └─阶段4 CAR1 侧向抓取（接近/下探/闭合/提升/收回），CAR2 并行动态停靠
              └─阶段5 停靠检查（车身投影在绿框外、中心距 ≥ 0.82 m）
                  └─阶段6 协同投放（质心对桶心 + 桶口上方 6 cm 自由落体）→ 两车错时倒车退出 Y=17.5
                      └─阶段7 正向返航 → 验收（入桶、回位、零接触）
```

### 6.2 单帧控制闭环

```
任务状态机 ──读取、写指令──> RobotIO
                              ├── chassis[i].pose()/speed()/wheel_speeds()   ← 位姿与轮速回读
                              ├── chassis[i].set_wheels(l, r, steer)          → 轮速/转角下发
                              ├── chassis[i].lidar.*                          ← 四个扇区距离 + scan_if_due()
                              ├── arm.start_world()/start_fixed()/tick()/hold() → 六个关节指令
                              ├── arm.tool_position()                         ← 夹持点世界坐标
                              ├── arm.solve_ik()                              ← 关节角 + 残差
                              ├── gripper.start()/tick()/hold()/force()       → 夹爪指令
                              ├── perception.estimate_target()                ← 目标粗端中心
                              ├── perception.object_pose()                    ← 手中物体的实时位姿
                              ├── frame_hook()                                ← 雷达点云叠加到 viewer
                              clock.step()
                                  ├─ 仿真：MuJoCo mj_step × STEPS_PER_FRAME → 物理演化 → 传感器
                                  └─ ROS ：等 /clock 递增 + 补发两车轮速指令
```

同一段任务代码在两个后端上的差别只在这张表的右半部分：`SimClock.step()` 直接推进物理，
`RosClock.step()` 阻塞等下一条 `/clock`；`SimPerception` 自己渲染并分割，
`RosPerception` 触发采集服务再等 `mm_target_localizer` 出结果。

### 6.3 指标链路

```
DualCarMission.metrics（逐阶段 dict）
  └─ run_mission(): 追加 seed / wall_time / sim_time / ok → 单行 "RESULT_JSON: {...}" → exit 0/1
      └─ scripts/run_matrix.py: 逐 seed 子进程 + 超时 → reports/regression.csv + reports/logs/seed_N.log
          └─ scripts/compare_metrics.py: 两次 CSV 逐指标对比（重构等价性验证）
```

## 7. 并发调度模型

仿真里没有独立的执行线程：**一切由任务层的一个阻塞循环驱动**，每轮"推进各控制器一帧 → 推进时钟一帧"。

- `run_wait_concurrent / run_arm_motion_concurrent / run_finger_motion_concurrent / finish_follower / run_two_followers / run_followers_until_primary_done / run_turn_with_secondary / run_two_straight_reverses` 都是这种循环。
- 副作用耦合：A 车的循环顺带推进 B 车的控制器，所以**改动任一处的时长都会改变另一辆车的轨迹**（见教训：加转向斜坡把 seed 3 的停靠航向改了 71°）。改这类共享时序后必须跑全矩阵。
- ROS 2 侧保留同一套阻塞循环，只是把 `Clock.step()` 换成"等 `/clock`"：任务层仍然自己驱动节奏，桥不会抢跑。机械臂动作、视觉等待这些阶段任务层不刷新底盘指令，所以 `RosClock.step()` 每帧把两车最近一次的轮速指令补发一遍，桥的帧同步才不会每帧超时。
- 视觉等待期间（`request_estimate()`）也在推进帧并补发指令：图像与 TF 都在帧边界产生，只有推进才会出新帧，这样仿真时间始终由任务层驱动而不是凭墙钟自由跑。

## 8. 关键参数与设计约束

| 项 | 值 | 原因 |
|---|---|---|
| 轮径 / 轮距 / 轴距 | 0.305 / 0.555 / 0.500 m | 厂家参数表 |
| 原地转轮速 | ±8 rad/s | ±2.5 低于轮胎静摩擦门限，只打滑不转（偏航 0.026 rad/s） |
| 横摆角速度上限 | 1.5 rad/s | 超过会把重载车甩飞 |
| 二号车停靠中心 Y | 20.06 m（绿框下边界 -0.64） | 车身任意朝向投影 ≤0.578 m，留 0.06 m 余量 |
| 抓取前摆正阈值 | 12° | 补 3° 的原地转要滑 14 mm，代价大于收益 |
| 目标区 / 放射源散布 | 0.70×0.60 m；杆原点 X −0.269~+0.100、Y 20.78–21.22 | UR10e 臂展 1.3 m，抓取点 0.94 m（72%）；区间按 202 mm 杆的两端半长从搜索窗内缩，保证整根杆不出界 |
| 监测棒 | Ø6×33 + Ø5×169，总长 0.202 m，33.5 g，质心距 body 原点 79 mm | 质心偏移由模型惯性参数读出，投放按质心对准桶心 |
| 屏蔽桶 | 内径 Ø400、内深 240（外高 260）、底厚 20 mm | 桶口必须远大于杆长与真机定位误差：Ø240 只容忍 18 mm，Ø400 约 98 mm |
| 投放松爪高度 | 桶口上方 0.06 m | 杆整根自由落体到底，不碰桶沿 |
| 落桶判据 | 整根杆在桶口投影内（端点离内壁 ≥ 3 mm）、低于桶沿、高于桶底 | 杆横躺时质心只能偏十几毫米、立着时能偏 100 mm，同一质心容差对两种姿态自相矛盾；质心偏移仍记录，作横向精度参考 |

## 9. 回归与验收

- 指标：视觉误差、桶内 XY 误差、停靠时刻中心距、全程最小中心距、车身投影上沿、回起点误差、倒车最小间距、车车接触数、仿真时长。
- 判据：`stage=done`、入桶、接触 0、车身在绿框外、回位 <0.20 m。
- 命令：`python scripts/run_matrix.py --seeds 0-20`；对比：`python scripts/compare_metrics.py a.csv b.csv`。

## 10. 已知限制与迁移映射

| 限制 | 现状 | 迁移动作 |
|---|---|---|
| 前轮转向节是等效机构 | 控制器用"转角 + 轮速差"，非纯差速 | 重写横摆控制并重新标定 |
| 雷达检测 2.8 m < 制动 3.5 m | 已按 2.0 m/s 巡航把制动距离压到约 1.5 m | 真机换轮胎/载荷后复核 |
| 摩擦/静摩擦门限为仿真整定值 | 未做真机辨识 | 直线/原地转/坡道三组试验回填 |
| 视觉是渲染深度图 + 地平面拟合 | 用反投影深度做离地分割，无真机标定与去噪 | 换相机驱动 + 手眼标定 + 深度尺度标定 |
| ROS 侧姿态参数未接通 | 逆解服务固定竖直下压姿态，工具朝向由 `TARGET_QUAT` 决定 | 真机接 servoj 时把姿态并入 SolveIK 请求 |

## 11. 变更记录

| 时间 | 内容 |
|---|---|
| 2026-11-21 | 轨道 B 回归脚本化：`scripts/run_ros_matrix.py` 取代手敲 shell（每种子独占 `ROS_DOMAIN_ID`、N 路并行、超时先 SIGINT 再 SIGKILL，避免孤儿桥继续发 `/clock`，指标写成与轨道 A 同构的 CSV 直接喂 `compare_metrics.py`）。21 种子并行 4 路实测 **21/21**、墙钟 30.6 min；解析逻辑用冻结的 21 份日志对基线复核，21/21 逐项一致 |
| 2026-11-21 | 补上真车避障链缺的"点云 → 雷达摘要"节点（`mm_perception/lidar_summary` + 纯 numpy 的 `lidar_geometry.summarize`）。仿真桥新增 `/{car}/lidar/points`（10 Hz，`publish_cloud` 默认关），节点 check 模式按时间戳把两边算的 8 个数逐帧配对。踩到两个坑：①点云是 float32，扇区边界上（仿真方位角是 3° 整数倍，30/90/150 度必有射线）归属会翻转，按仿真闭合规则留 `1e-6 rad` 余量；②仿真侧 +150° 那条射线比门限低 `4.4e-16` 被排除、−150° 正好相等被包含，归属由浮点末位决定，这类帧单独计数不进统计。最终 seed 0 全流程：统计 950 帧、最大偏差 `9.1e-8 m`、`front_blocked` 0 帧不一致、边界打平 219 帧 |
| 2026-11-17 | 轨道 B 21 个种子在 `ceee430` 上复跑 **21/21**（并行 N=4、墙钟 22.8 min）：口内余量 28~62 mm、杆顶 0.618~0.628 m、视觉 0.89~3.83 mm、回起点 ≤0.10 m、接触 0，原始指标入库 `reports/baselines/ros_track_b_ceee430.csv`。同时定位并修掉「指令超时」的来源：任务线程等 `SolveIK`（每次 1~2 s）时不推进帧，桥收不到轮速只能按旧指令推进——实测 234~601 帧（占全流程 8%），同种子串行反而更高（seed 0 并行 431 / 串行 577），因此与并行无关；`_call_service` 改为等待期间 `wait_next_frame()` + `flush_wheel_commands()`，与等感知结果同一做法，这个计数才恢复判别力 | 
| 2026-11-17 | 轨道 B 21 个种子在 `e09370d` 上 **21/21**（Ubuntu，并行 N=4、每种子独立 `ROS_DOMAIN_ID`，墙钟 23.5 min）：口内余量 19~61 mm、杆顶 ≤0.628 m（桶沿 0.664）、视觉 0.93~6.34 mm、回起点 ≤0.100 m、车车接触 0。同时修桥的超时计数口径：原来启动空转和任务中的指令超时混在一个计数里，且只在第 1~3 次与每 200 次打印一行，**日志行数根本不等于超时次数**（那些超时主要来自任务线程等逆解服务时没泵帧，见下一条）。改为两段分开——首帧指令到达前记 `空转帧`，两车都下发过指令之后仍等不到新指令才算 `指令超时`，收尾行同时给出两个数；并行跑的数据可信性判据改为 `指令超时=0` | 
| 2026-11-17 | 速度按真机口径定档并把绕行方向改成带记忆：巡航 `6.5 → 2.0 m/s`、持杆/接近 `1.2 → 1.0`、入位 `0.8/0.55 → 0.5`、加减速 `4.5/6.0 → 1.5/2.0 m/s²`。依据是雷达只看到 2.8 m 而 6.5 m/s 要 3.5 m 制动、且横摆限幅 1.5 rad/s 跟不上该速度下的转向需求（实测 5 m/s 时航向误差 0.45 rad 需要 3.1 rad/s）。同时把绕行方向加 1.5 s 记忆：ROS 侧二号车卡死的日志里"发现 0.27 m 障碍 → 左绕 → 已清"刷 65 次，实际是左右交替翻转导致净位移为零。新增"原地磨"告警（距航点远且无前进速度持续 1 s）。单进程 21/21，单次仿真 59.6 → 69.9 s，新基线 `reports/baselines/real_speed.csv` | 
| 2026-11-17 | 落桶判据从"质心离桶心 ≤0.05 m"改为几何包含（`rod_inside_bucket`：杆两端都在桶口内接圆内、低于桶沿、高于桶底）。质心容差是照"杆横躺"一种姿态定的代理量，而 ROS 侧松手瞬间杆会被甩成竖直落入（倾角 1.4~21°），同一条容差对两种姿态自相矛盾——出现"杆明明在桶里却判负"。对横躺反而更严：质心允许偏移 50 mm → 约 18 mm。单进程 21/21 且与旧基线逐位一致（1e-9），说明只改判定、没动物理 | 
| 2026-11-16 | 修轨道 B 第二个运行期错误：`RosArm` 把工具姿态丢了（`del target_quat` + 逆解服务写死 `TARGET_QUAT`），而抓取要求工具绕自身 Z 轴转 ±90° 才能横跨杆身，全部种子卡在 `4_grasp`（`car1 failed to grasp the target rod`，力 0.95~1.62 N、杆原地不动）。单进程强制 0° 复现了同一现象（力 12.00 N、杆不动），正常 ±90° 是 5.10 N 且杆被抬到 0.84 m；`choose_grasp_spin` 也因此退化成"永远取第一个候选"。修法：`SolveIK.srv` 增 `geometry_msgs/Quaternion orientation`（固定基座系），`RosArm.start_world`/`solve_ik` 原样透传，桥按请求姿态逆解；`scripts/check_ports.py` 增加第 4 项"工具姿态透传"（已验证丢掉姿态时会 FAIL）。顺带把 `frame_timeout` 5→15 s、`ready_timeout` 30→60 s，避免虚拟机降速时把正常运行判成失败 |
| 2026-11-16 | 修轨道 B 首次运行暴露的两个坐标系错误：①桥把 `car1/arm_base_link`、`car1/pinch` 的世界坐标挂在车体/臂座父帧下，TF 合成后世界目标偏 ~20 m，任务卡在 `4_grasp`（IK 残差 18.7 m）——改为 `_send_world_tf()`，父帧固定 `map`，不再开放 parent 参数；②`RosArm.start_fixed` 把固定基座系目标直接当臂座系用，少了 `FIXED_BASE_ROT = diag(-1,-1,1)` 的 180°（`arm_ik_scene.xml` 的 base 体旋转），机械臂会安静地走到镜像位置（残差 1.65 mm 看不出来）——常数移入 `mission_core` 并由单进程入口用模型复核，`scripts/check_ports.py` 增加"ROS 与单进程 `start_fixed` 落到同一 IK 目标"的往返检查 |
| 2026-11-16 | 阶段 2 任务层拆分：`DualCarMission` + 三个路径控制器 + `run_*_concurrent` + 抓取/投放编排 + 任务常量移到零 MuJoCo 的 `mission_core.py`，单进程与 ROS 共用一份；端口扩 `LidarInterface`、`ArmInterface.start_world/tool_position`、`PerceptionInterface.estimate_target/object_pose`，`RobotIO.viewer` 换成每帧渲染回调 `frame_hook`；桥补 `/sim/object_pose`、`car1/pinch` TF、`/car1/lidar/ignore_peer`；launch 给任务节点加 `on_exit=Shutdown()`（否则桥常驻、下一次启动出现两条 `/clock`）；顺手删掉从未被调用的相机参数搜索与死参数 |
| 2026-11-16 | 阶段 1 构建验证：Ubuntu 22.04 + Humble 上 5 包 `colcon build --symlink-install` 一次通过；冒烟 A 500 帧 = 10.00 s 仿真时间，冒烟 B `vision_error_mm=2.36`；回填 3 个运行期修复（节点收尾顺序、`rosgraph_msgs/Clock` 导入、服务调用改 `call_async` 轮询、机械臂冒烟目标移到车头前 0.5 m）|
| 2026-11-16 | 盘点清理：删除 3 张过期截图（旧圆柱/旧桶）、6 个从未被引用的模块级常量与若干过期注释；质心偏移改为从模型惯性参数读取 |
| 2026-11-09 | 被操作对象改为不锈钢监测棒（Ø6×33 + Ø5×169、202 mm、33.5 g）、屏蔽桶改 32 边圆筒（内径 Ø240）、投放改纯重力落桶；采样区间按杆的两端半长从视觉搜索窗内缩；末段直行增加"车头延长线压到终点"判据 |
| 2026-11-08 | 阶段 1 骨架：`ros2_ws/src` 下三节点（sim_bridge / mission / target_localizer）+ `/clock` 帧同步 + launch，设计见 `docs/ros2_stage1_design.md`（尚未入库） |
| 2026-11-08 | 阶段 0：抽出 `robot_interface.py` 与 `Sim*` 适配器，任务层只依赖 `RobotIO`；视觉等待改同步（回归可复现）；删除 247 行死代码；新增 `compare_metrics.py` |
| 2026-11-08 | v1.0.0 基线冻结（tag `v1.0.0`），补 MIT 协议与 README 徽章 |
