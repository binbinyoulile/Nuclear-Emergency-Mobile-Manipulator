# 架构说明

> 本文档面向"要改这个项目的人"和"要把它迁到真机/ROS2 的人"，只讲结构、契约与数据流。
> 项目能力、快速开始、回归基线见 [README.md](README.md)。

## 1. 文档维护规则

- 改动了模块边界、接口签名、状态机阶段或回归指标字段时，必须同步本文档对应章节。
- 本文档不写"将来可能会做"的设想，只写已经实现的结构；路线图放在对话或 README 的里程碑里。

## 2. 项目目标与当前边界

**最终目标**：基于多模态感知（雷达 + 深度相机）的核电应急机器人运动控制 Sim2Real 闭环验证。

**当前边界（v1.0.0 仿真基线，tag `v1.0.0`）**：

- 已实现：双车协同全流程仿真（雷达避障 → 视觉识别 → 侧面抓取 → 动态停靠 → 协同投放 → 倒车脱离 → 精准返航），75 kg 真实参数重载底盘，21 个随机种子的可回归验收。
- 未实现：真机驱动、ROS 2 节点、手眼标定、实车参数辨识。

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
├─ dual_car_pick_place.py   # 仿真侧：控制器、感知、任务状态机、Sim* 适配器、指标
├─ mobile_pick_place.py     # 共享底层：MJCF 加载、mink IK、重力补偿、ArmMotion/FingerMotion、Simulation
├─ robot_interface.py       # 抽象层：Clock / Chassis / Arm / Gripper / Perception / RobotIO（不含 MuJoCo）
├─ dual_car_scene.xml       # 主场景（30×30 m、红线、绿框、两车装配）
├─ arm_ik_scene.xml         # 固定基座 IK 场景
├─ models/                  # UR10e 复合模型（assets 由 scripts/fetch_models.py 下载，不入库）
├─ scripts/                 # fetch_models.py / run_matrix.py / compare_metrics.py
├─ docs/screenshots/        # README 配图
└─ reports/                 # regression.csv（入库）+ logs/（本地忽略）
```

## 5. 核心模块职责

| 模块 | 关键符号 | 职责 | 依赖 |
|---|---|---|---|
| 抽象层 | `robot_interface.RobotIO` | 定义硬件端口：时钟、两车底盘、机械臂、夹爪、感知 | 仅 numpy |
| 仿真适配 | `SimClock / SimArm / SimGripper / SimPerception` | 把接口翻译成 MuJoCo 的 ctrl/qpos 读写；`build_robot_io()` 是唯一装配点 | mobile_pick_place |
| 底盘 | `CarController`（实现 `ChassisInterface`） | 四轮轮速 + 前轮转角下发、世界位姿/速度回读、接触统计、雷达扫描、停稳 | base |
| 路径控制 | `PathFollower` | 纯追踪 + 雷达局部避障 + 终点直行 + 低速掉头；横向容差与超时保护 | CarController |
| 转向控制 | `SkidTurnFollower` | 差速原地转（方向锁存、越界即停、可选轮速斜坡） | CarController |
| 倒车控制 | `ReverseStraightFollower` | 锁车道 + 前轮转角保持的直线倒车，后向净空限速 | CarController |
| 感知 | `get_camera_data / detect_yellow_target / pixel_to_world / process_camera_frame` | D435i 四层视觉：取图 → HSV 分割 → 针孔反投影 → 外参变换 | mujoco.Renderer |
| 机械臂 | `ArmMotion`（经 `SimArm` 暴露） | 关节目标下发 + 平滑插值 + 重力补偿；IK 用 mink 且以当前关节角为种子 | mink |
| 夹爪 | `FingerMotion`（经 `SimGripper` 暴露） | 单执行器开合插值（0 开 / 255 闭） | base |
| 任务 | `DualCarMission` | 七阶段状态机、并发调度（`run_*_concurrent`）、指标采集与验收判据 | RobotIO |
| 入口 | `main()` / `initialize_mission()` | 参数解析（`--seed/--headless/--snapshot/--backend`）、场景与目标随机化、viewer | 全部 |

## 6. 数据流

### 6.1 任务阶段

```
阶段1 起点
  └─阶段2 CAR1 雷达避障 → 观察点(Y≈18.4)
      └─阶段3 D435i 取图识别放射源 → 通知 CAR2 出发 → CAR1 逼近目标正南 0.85 m
          └─阶段4 CAR1 侧向抓取（接近/下探/闭合/提升/收回），CAR2 并行动态停靠
              └─阶段5 停靠检查（车身投影在绿框外、中心距 ≥ 0.82 m）
                  └─阶段6 协同投放 → 两车错时倒车退出 Y=17.5
                      └─阶段7 正向返航 → 验收（入桶、回位、零接触）
```

### 6.2 单帧控制闭环

```
任务状态机 ──读取、写指令──> RobotIO
                              ├── chassis[i].pose()/speed()/wheel_speeds()   ← 位姿与轮速回读
                              ├── chassis[i].set_wheels(l, r, steer)          → ctrl 写入
                              ├── chassis[i].lidar.scan()                    ← 点云
                              ├── arm.start_fixed()/tick()/hold()            → 六个关节 ctrl
                              ├── gripper.start()/tick()/hold()              → 夹爪 ctrl
                              └── perception.capture()/detect_target()       ← RGB/深度
                              clock.step()
                                  └── MuJoCo mj_step × STEPS_PER_FRAME → 物理演化 → 传感器
```

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
- 迁移 ROS 2 时这套循环由节点定时器替代，任务层只保留"当前阶段 + 目标"的状态，不再自己 `step`。

## 8. 关键参数与设计约束

| 项 | 值 | 原因 |
|---|---|---|
| 轮径 / 轮距 / 轴距 | 0.305 / 0.555 / 0.500 m | 厂家参数表 |
| 原地转轮速 | ±8 rad/s | ±2.5 低于轮胎静摩擦门限，只打滑不转（偏航 0.026 rad/s） |
| 横摆角速度上限 | 1.5 rad/s | 超过会把重载车甩飞 |
| 二号车停靠中心 Y | 20.06 m（绿框下边界 -0.64） | 车身任意朝向投影 ≤0.578 m，留 0.06 m 余量 |
| 抓取前摆正阈值 | 12° | 补 3° 的原地转要滑 14 mm，代价大于收益 |
| 目标区 / 放射源散布 | 0.70×0.60 m / X±0.28、Y20.78–21.22 | UR10e 臂展 1.3 m，抓取点 0.94 m（72%） |
| 落桶容差 | 0.05 m | 桶内半径 0.05 m |

## 9. 回归与验收

- 指标：视觉误差、桶内 XY 误差、停靠时刻中心距、全程最小中心距、车身投影上沿、回起点误差、倒车最小间距、车车接触数、仿真时长。
- 判据：`stage=done`、入桶、接触 0、车身在绿框外、回位 <0.20 m。
- 命令：`python scripts/run_matrix.py --seeds 0-20`；对比：`python scripts/compare_metrics.py a.csv b.csv`。

## 10. 已知限制与迁移映射

| 限制 | 现状 | 迁移动作 |
|---|---|---|
| 前轮转向节是等效机构 | 控制器用"转角 + 轮速差"，非纯差速 | 重写横摆控制并重新标定 |
| 雷达检测 2.8 m < 制动 3.5 m | 未联合标定 | 真机前做速度-距离标定 |
| 摩擦/静摩擦门限为仿真整定值 | 未做真机辨识 | 直线/原地转/坡道三组试验回填 |
| 视觉是渲染图 + HSV | 无真机标定与去噪 | 换相机驱动 + 手眼标定 |
| 控制器仍直接读仿真时钟 | `PathFollower` 等内部用 `base.DATA.time` | 注入 `Clock`（下一阶段） |

## 11. 变更记录

| 时间 | 内容 |
|---|---|
| 2026-11-08 | 阶段 0：抽出 `robot_interface.py` 与 `Sim*` 适配器，任务层只依赖 `RobotIO`；视觉等待改同步（回归可复现）；删除 247 行死代码；新增 `compare_metrics.py` |
| 2026-11-08 | v1.0.0 基线冻结（tag `v1.0.0`），补 MIT 协议与 README 徽章 |
