# 底盘 CAN 驱动（mm_base）：协议、节点与真机首跑

结论先行
--------
- 新包 `ros2_ws/src/mm_base`（ament_python）实现了厂家 `zeus_s2_driver` 的 CAN 协议：
  命令帧 `0x101..0x104`、反馈帧 `0x601..0x604`、模式应答 `0x700`。
- 节点默认 `backend:=socketcan`、`can_interface:=can0`，按 **125 Hz** 发四轮转速帧；
  订阅 `/cmd_vel`，发布 `/odom`（`odom`→`base_link`）与 `/joint_states`。
- 指令分解用 **0.61 m** 轮距，里程计用 **1.35 m** 轮距；两者不是同一个量，理由见第五节。
- 启动顺序固定为 **清报警 → 使能 ×4 → 转速模式 ×4 → 零速**；使能/模式应答超时会报错退出，
  不会带着未知驱动器状态开始发速度。
- **ROS 2 侧已在 Ubuntu 22.04 + Humble 实测**（构建、`ros2 run mm_base selftest` 112 条断言、
  loopback 节点启动与 `/cmd_vel → /odom` 端到端，见第八节）；**真机侧未验证**——没上过车，
  SocketCAN、驱动器应答、速度给定单位、编码器符号都要上车确认。
- **任务书 D 第 5 条与 C 的 1.35 m 口径在数学上冲突**：按 0.61 分解、1.35 折航向，
  原地转 1.0 rad/s 的里程计偏航是 `0.61/1.35 ≈ 0.452 rad`，不是 1.0 rad。
  自检按 1.35 断言 0.452，并另测 0.61 口径得到 1.0，详见第七节。

---

## 一、包结构

```
ros2_ws/src/mm_base/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/mm_base
└── mm_base/
    ├── __init__.py
    ├── protocol.py     纯协议：帧构造/解析、单位换算、启动握手（不 import rclpy/socket）
    ├── backend.py      传输层：SocketCanBackend / LoopbackBackend / RecordingBackend
    ├── kinematics.py   纯数学：/cmd_vel 分解、四轮里程计、轮关节换算
    ├── base_node.py    rclpy 节点
    └── selftest.py     无 ROS2、无 CAN 的自检（Windows 可跑）
```

没有修改 `mm_msgs`、`mm_sim`、`mm_perception`、`mm_mission`、`mm_bringup` 任何一个文件；
也没有新增 `mm_bringup/launch` 文件——`ros2 run mm_base base_node` 加参数已经够用，
少一个入口就少一处要维护的配置。

---

## 二、话题与参数

### 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 只用 `linear.x`（m/s）与 `angular.z`（rad/s） |
| 发布 | `/odom` | `nav_msgs/Odometry` | `header.frame_id=odom`，`child_frame_id=base_link` |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | `wheel_fl_joint`、`wheel_fr_joint`、`wheel_rr_joint`、`wheel_rl_joint`，位置 rad、速度 rad/s |

节点不发布 `odom → base_link` 的 TF。需要 TF 的导航层自行加，或者下一步在节点里补一个
`TransformBroadcaster`；现在刻意不写，避免和 `mm_sim` 的 TF 语义混在一起。

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `can_interface` | `can0` | SocketCAN 接口名 |
| `wheel_track` | `0.61` | 指令分解轮距 m（厂家 `wheel_track`） |
| `odom_wheel_track` | `1.35` | 里程计轮距 m（厂家 `wheel_track_4wd`，滑移补偿） |
| `max_speed` | `1.0` | 单轮线速度限幅 m/s |
| `max_yaw_rate` | `1.0` | 横摆限幅 rad/s；厂家导航最大角速度也是 1.0 |
| `publish_rate` | `125.0` | 主循环/发帧频率 Hz |
| `backend` | `socketcan` | `socketcan` 或 `loopback` |
| `invert_side` | `false` | 左右两侧指令同时取反；默认按厂家源码符号 |
| `cmd_timeout` | `0.5` | `/cmd_vel` 看门狗，超时清零 |
| `startup_timeout` | `5.0` | 每步等待四个驱动器应答的总超时 s |

参数在构造函数里校验：`publish_rate`、轮距、限速、超时必须为正，`max_yaw_rate` 不能为负；
任何一项非法直接抛 `ValueError`，不会带着除零或无限速运行。

---

## 三、协议实现（附出处）

出处均为 `D:\桌面\机械臂\vehicle_workspace\slam20220903\src\zeus_s2\zeus_s2_driver\`，
查证记录见 `docs/vehicle_workspace_refs.md` 第一、二节。标准帧、DLC=8、多字节大端。

### 命令帧：ID = `0x101 + (n-1)`，n = 1..4

| 动作 | data[0..7] | 源码函数 |
|---|---|---|
| 使能 | `00 24 00 10 00 01 00 00` | `enable_motor_driver()` |
| 失能 | `00 1E 00 10 00 00 00 00` | `disable_motor_driver()` |
| 清报警（须先失能） | `00 21 00 10 00 00 00 01` | `clear_ALM_motor_driver()` |
| 设为转矩模式 | `00 1E 00 03 00 01 00 00` | `Set_motor_torque_mode()` |
| 设为转速模式 | `00 1E 00 03 00 02 00 00` | `Set_motor_speed_mode()` |
| 给转矩值 | `00 28 00 01 hi lo 00 00` | `Set_torque_value_to_motor()` |
| 给转速值 | `00 28 00 00 hi lo 00 00` | `Set_speed_value_to_motor()` |

轮序：`0x101` 左前、`0x102` 右前、`0x103` 右后、`0x104` 左后（源码注释 + 手册
“车头顺时针 1→4”一致）。

### 反馈帧：ID = `0x601..0x604`

| data[1] | 含义 | 本包字段 |
|---|---|---|
| `0xCA` | `data[2..5]` int32 大端编码器计数；`data[6..7]` 电流（厂家注释“千分单位”） | `encoder_counts`、`current_raw` |
| `0xCB` | `data[2..3]` 温度；`data[4..5]` int16 实际转速；`data[6..7]` 故障码 | `temperature_c`、`speed_rpm`、`fault_code` |

关键细节：厂家在 `updateParam()` 里写的是 `motorActSpeed = -((data[4]<<8)|data[5])`，
**反馈转速取负号**；编码器计数不取负。`parse_feedback()` 原样照做：`speed_rpm` 是取负后的值，
`encoder_counts` 是原始 int32。未知 `data[1]` 返回 `None`，帧长不是 8 抛 `ValueError`。

### 模式/使能应答：ID = `0x700`

| 应答 | 判据 | 源码 |
|---|---|---|
| 使能成功 | `data[1]=0x25`、`data[3]=0x10`、`data[5]=1` | `Calculate_LeftAndRight_Meter()` |
| 模式生效 | `data[1]=0x1F` | `mode_switch_judge()` |

`data[0]` 是电机号 1..4。启动握手要求四个驱动器都收到对应应答，缺一个就报
`StartupError` 并列出缺失的驱动器号。

---

## 四、速度模式 vs 转矩模式

厂家的做法（`zeus_s2_driver.cpp` + `zeus_params.yaml`）：

- 驱动器设为**转矩模式**；
- 上位机 125 Hz 跑速度 PI：`Kp=1500`、`Ki=25`、`Kd=0`；
- 输出钳位 `±1910`，再通过 `0x28 / 0x0001` 发给驱动器；
- 好处是力/电流可控、能按轮做抗饱和；代价是标定和调参都在上位机，且**转矩值的单位
  源码没写**（只与额定 1.91 N·m 数值吻合，见 `vehicle_workspace_refs.md` 第五节）。

本包第一版选择**转速模式**：

| 维度 | 转速模式（本包） | 转矩模式 + 上位 PI（厂家） |
|---|---|---|
| 控制闭环 | 驱动器内部速度环 | 上位机 125 Hz PI |
| 代码量 | 只发目标转速 | 要读转速、算 PI、抗饱和、限幅 |
| 转矩单位 | 不涉及 | 未验证，是上真机的硬卡点 |
| 轮端力控 | 没有 | 有 |
| 低速/打滑行为 | 依赖驱动器内部环，未验证 | 厂家已在这台车上跑过 |
| 调参风险 | 低 | 高，必须先架空台架 |

**下一步**（按顺序）：

1. 转速模式跑通直线/原地转，记录驱动器内部速度环的稳态误差和响应；
2. 台架上给一个小转矩，用电流/转速响应反推 `0x28 / 0x0001` 的单位；
3. 再移植厂家 PI（先照抄 Kp/Ki/Kd，再按轮标定），对比直线保持与原地转精度；
4. 加加减速规划（厂家导航加速度 1.0 m/s²、0.6 rad/s²），现在节点是阶跃给目标。

没有验证过的：驱动器内部速度环带宽、低速死区、堵转保护、转速指令的 16 位单位。
这些都写进第八节的“未验证”表，不当作已完成。

---

## 五、单位与符号（最容易错的地方）

### 转速 ↔ 线速度

```
rpm_to_mps(rpm) = rpm / 40 * π * 0.305 / 60
mps_to_rpm(mps) = mps * 60 * 40 / (π * 0.305)
```

校验：3000 rpm → 1.199 m/s，与额定 1.20 m/s 吻合（`vehicle_workspace_refs.md` 第二节）。
编码器在电机侧，所以积分计数时**不乘减速比**；`counts_to_m()` 已经用
`417455 count/m = 10000×40/(π×0.305)` 把减速比包含进去了。

### 速度给定帧的 16 位定点

`Set_speed_value_to_motor(uint16_t value)` 只把 value 原样拆成 `data[4..5]`；**厂家唯一
调用点**在 `mode_switching_state_machine()` 的 `DRIVER_PARKING_MODE` 分支，传的是 `0`。
所以**源码不能证明**这个 16 位值是 rpm 整数还是 0.1 rpm 定点。

本包取 `1 LSB = 1 rpm`，理由是同一条反馈链的 0xCB 转速按 rpm 解算
（`motorActSpeed/40*πD/60`），命令与反馈用同一单位最自然。该常量集中在
`protocol.SPEED_VALUE_RPM_PER_LSB`，**真机首次上电必须用小转速核对**：如果实际转速只有
预期的 1/10，把它改成 `0.1` 即可，其余代码不用动。状态：**未验证**。

### 左右/前后符号

厂家指令里右轮取负（电机镜像安装）：

```
v_L = v - ω·0.61/2
v_R = -(v + ω·0.61/2)
```

本包的约定：

- `v_L` / `v_R` 都是“车轮前进为正”的物理量；
- 发帧时右轮用 `-v_R`（`twist_to_wheel_speeds()` 的返回值已经是这个符号，不要再取负）；
- 0xCA 编码器计数：左轮前进增加、右轮前进减少；里程计按 `WHEEL_SIDE_SIGN`
  （左 +1、右 −1）还原成前进距离；
- 0xCB 反馈转速已被厂家取负：右轮前进时 `speed_rpm` 为负，`/joint_states` 再乘一次
  `WHEEL_SIDE_SIGN` 还原成“四轮前进都为正”的关节速度。

`invert_side:=true` 会把左右两侧指令同时取反，只用于“电机镜像方向与厂家相反”的整车；
它不是单轮方向修正。如果只有某一个轮反了，要改接线/驱动器参数，不要在软件里拿整车符号凑。

### 两个轮距

- `wheel_track=0.61`：只用于 `/cmd_vel` → 左右轮速。出处 `zeus_params.yaml`。
- `odom_wheel_track=1.35`：只用于编码器里程 → 航向。出处 `zeus_params.yaml` 的
  `wheel_track_4wd`，是厂家用角度校准 launch 标出来的滑移补偿，约为 URDF 几何轮距 0.594
  的 2.27 倍。

含义：同样的轮速差，实际横摆约为几何模型的 44%。所以**不能**拿 0.61 去算里程计，
也不能拿 1.35 去分解 `/cmd_vel`；这两个错误都会让原地转差 2 倍以上。

---

## 六、真机首跑

前置条件（来自 `docs/real_machine_calibration.md` 与 `vehicle_workspace_refs.md`）：

- 车轮**架空**，两侧急停在手，至少两人；
- 平地（电机无制动器，坡道不能停）；
- 电池电压高于 45 V 才允许行驶；
- CAN 总线 1000 kbps、两端 120 Ω 终端电阻（CAN 常规要求，厂家文档未逐条写）；
- 先跑 Windows 自检，再跑 loopback，最后才接真 CAN。

> **一条命令的简化版**：`scripts/bench_real.py` 把下面第 1~4 步（拉 can0 → 启动握手 →
> 四轮方向 → 里程计）串成一次自检，每步打印期望/实测/合格线并存 CSV：
> `sudo python3 scripts/bench_real.py --backend socketcan --can can0`。
> 想在没车时先验脚本自身逻辑：`python scripts/bench_real.py --backend loopback`。
> 本节下面保留的是手工步骤与排查细节，出问题时对照用。

### 1. 拉起 can0

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

期望输出里能看到 `bitrate 1000000`。如果接口不存在，先确认 USB-CAN / 板载 CAN 驱动
（`ip link` 里应有 `can0`）。用 `candump can0` 看总线是否有帧；驱动器上电后通常会有
0x601..0x604 的反馈，没有就先查接线、终端电阻和 ID。

### 2. 机器人上先跑 loopback（不接 CAN）

```bash
ros2 run mm_base base_node --ros-args -p backend:=loopback
```

另开终端：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic echo /odom
ros2 topic echo /joint_states
```

确认节点能启动、`/odom` 会动、Ctrl-C 后日志里有“已发送四轮失能”。loopback 不模拟打滑、
惯量和电流，只验证 ROS 链路与符号。

### 3. 接真 CAN，轮子架空

```bash
ros2 run mm_base base_node --ros-args \
  -p backend:=socketcan -p can_interface:=can0 \
  -p max_speed:=0.5 -p publish_rate:=125.0
```

启动日志应依次出现“开始启动握手”“启动握手完成”。如果超时，日志会列出没应答的驱动器号，
节点返回非零退出；先查那一路的供电/CAN 线/ID，不要绕过握手直接发速度。

启动顺序按任务书是“清报警 → 使能 → 转速模式”。厂家 `initRobot()` 自己的顺序是
“失能 → 转矩模式 → 转矩 0 → 清报警 → 等 1 s → 使能”，比我们多一步失能；厂家源码**没有**
说明上电默认是失能还是使能。本节点的启动顺序按任务书实现，因此假设驱动器上电处于失能
状态。如果上一次异常退出后驱动器仍使能，请先断电重启，否则清报警可能不生效。这一点必须
在真机上专门核对；核对结果如果要求先失能，就在 `run_startup_sequence()` 最前面补四帧失能
（自检第 8 项的帧数断言也要同步改）。

然后用**很小**的目标试方向：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

- 四轮应同时向前；`/joint_states` 的四个位置都应向正方向增长；
- 如果整体反向，先查 `invert_side` 与电机相序，不要直接改协议符号；
- 如果单轮反向，查该驱动器参数/接线；
- 如果转速只有预期的 1/10，回到第五节改 `SPEED_VALUE_RPM_PER_LSB`。

### 4. 落地

- 首次落地 **0.5 m/s 以内**（厂家导航自己只跑 0.5，0.8 是它的平滑上限）；
- 先直线 5 m、再原地转 90°，用卷尺/角度对比 `/odom`；
- 1.35 m 是厂家的标定值，不是我们的实测值；先照抄，跑顺后再按 `zeus_calibrate_angular`
  的流程重标，**不要**用几何轮距 0.594 替换它；
- 运行中任何驱动器故障码非零，节点会把目标清零并锁存；重启节点会重新清报警/使能。

### 5. 关机

Ctrl-C → 节点发四轮失能 → 关闭 CAN → 进程退出。确认日志里有失能记录，再断总电。
不要直接拔 CAN 线；驱动器可能保持上一次目标。

---

## 七、自检

Windows、无 ROS2、无 CAN 硬件，一条命令：

```powershell
D:\Python313\python.exe D:\robot_mobile\ros2_ws\src\mm_base\mm_base\selftest.py
```

也可以在 `ros2_ws/src/mm_base` 目录下用 `D:\Python313\python.exe -m mm_base.selftest`。
`selftest.py` 不 import rclpy；`protocol.py` 不 import rclpy/socket；`backend.py` 在 Windows
上 import 安全，只有实例化 `SocketCanBackend` 时才抛 `BackendUnavailable`。

实际输出（本机 2026-11-23，Python 3.13.12）：

```
mm_base selftest: Windows / no ROS2 / no CAN hardware
Python 3.13.12
[PASS] 1 command frame bytes
[PASS] 2 CAN ID 0x101..0x104
[PASS] 3 feedback decode and edge cases
[PASS] 4 unit conversion and zero-division guard
       straight: x=2.0002 m, y=0.000000 m, yaw=0.000000000 rad
[PASS] 5 straight 1.0 m/s x 2.0 s
       rotate @ odom_track=1.35: x=0.000000 m, y=0.000000 m, yaw=0.451885 rad
       rotate @ odom_track=0.61: yaw=1.000072 rad
[PASS] 6 rotate-in-place 1.0 rad/s x 1.0 s
       not enabled: x=0.000000 m, yaw=0.000000 rad
[PASS] 7 not enabled -> no motion (safety)
[PASS] 8 startup order and timeout reporting
[PASS] 9 Windows SocketCAN guard and import rules
----------------------------------------------------------------
ALL 112 ASSERTIONS PASSED
```

覆盖：

1. 七种命令帧逐字节对照厂家源码（期望值硬编码，不是“跑一遍填结果”）；
2. 命令/反馈 CAN ID；
3. 正/负编码器计数、负转速取负、故障码、未知命令字、7 字节错误帧；
4. rpm↔m/s、417455 count/m、16 位定点往返、NaN/越界/轮距为 0；
5. 直线 1.0 m/s × 2.0 s → 2.0002 m（±2%），偏航 0，左右编码器符号相反；
6. 原地转 1.0 rad/s × 1.0 s → 1.35 m 口径 0.4519 rad（±5%），0.61 m 口径 1.0001 rad（±5%）；
7. 未使能不运动、转矩模式收到速度帧不运动、运行中失能计数冻结；
8. 启动顺序 16 帧（清报警 ×4 → 使能 ×4 → 转速模式 ×4 → 零速 ×4）、ID 顺序、超时抛
   `StartupError`，以及回环 100 Hz 反馈（1 s 800 帧）；
9. Windows 上 SocketCAN 抛 `BackendUnavailable`、`protocol.py` 不 import rclpy/socket、
   `selftest.py` 不 import rclpy、`base_node.py` 能通过 `ast.parse`。

### 关于任务书 D 第 5 条与 1.35 m 口径的冲突

任务书 C 要求“里程计用 1.35，0.61 只用于指令分解”，任务书 D 第 5 条又要求
“原地转角速度 1.0 rad/s 跑 1.0 s，偏航 ≈1.0 rad（±5%）”。按 C 的口径：

```
轮速差 = 1.0 rad/s × 0.61 m = 0.61 m/s
里程计偏航率 = 0.61 / 1.35 = 0.452 rad/s
1.0 s 后偏航 = 0.452 rad
```

两者不能同时成立。本包按 C（1.35 m 是厂家滑移补偿口径，不能拿 0.61 算里程计）实现，
自检断言 0.452 rad（±5%）；同时用 `odom_wheel_track=0.61` 另跑一次，得到 1.0001 rad
（±5%），证明积分公式本身正确，不是算错。换句话说：按几何关系车辆实际转了约 1.0 rad，
但厂家 1.35 的里程计口径只报 0.452 rad——这正是滑移补偿的副作用，1.35 应该在真车上重标，
而不是拿 0.61 替换它。**如果验收必须按 D5 的 1.0 rad，唯一做法是把里程计轮距改成 0.61，
但那会违背 C 和厂家标定口径，不建议。**

---

## 八、验证状态

### 已在本地验证（Windows 10/11，Python 3.13.12，无 ROS2、无 CAN）

| 项 | 证据 |
|---|---|
| 七种命令帧字节与厂家源码一致 | `selftest` 第 1 项，期望值硬编码 |
| 命令/反馈 CAN ID | `selftest` 第 2 项 |
| 0xCA/0xCB 解码、负转速、故障码、未知命令、错误帧长 | `selftest` 第 3 项 |
| rpm↔m/s、count/m、16 位定点、除零/越界 | `selftest` 第 4 项 |
| 回环后端语义（使能/模式/目标/100 Hz 反馈/未使能不运动） | `selftest` 第 5–7 项 |
| 四轮里程计积分、1.35/0.61 口径 | `selftest` 第 5–6 项 |
| 启动顺序、应答超时上报 | `selftest` 第 8 项 |
| Windows 上 SocketCAN 只不可用、不自爆 | `selftest` 第 9 项 |
| `protocol.py` 不 import rclpy/socket、`selftest.py` 不 import rclpy | `selftest` 第 9 项 |
| 所有 `.py` 语法（含 `base_node.py` 的 rclpy import 不执行） | `python -m py_compile` 通过 |

### 已在 Ubuntu 22.04 + ROS2 Humble 上验证（2026-11-23）

| 项 | 结果 |
|---|---|
| `colcon build --packages-select mm_base --symlink-install` | 通过（1.7 s，无警告） |
| `ros2 run mm_base selftest`（安装后的入口） | `ALL 112 ASSERTIONS PASSED` |
| 节点启动（`backend:=loopback`） | 启动握手日志完整：清报警 → 使能 → 转速模式 → 「启动握手完成，进入运行」→「里程计已对齐四轮编码器」 |
| 话题 | `/cmd_vel`、`/odom`、`/joint_states` 三个都在 |
| `/cmd_vel` → `/odom` 端到端 | 发一次 `linear.x=0.5`，3 s 后 `/odom` 的 `position.x = 0.2480 m`。0.5 m/s × 0.5 s（`cmd_timeout` 到点清零）= 0.25 m，与预期一致 |
| 指令超时保护 | 日志出现「/cmd_vel 超过 0.50 s 未更新，目标速度清零」 |

**构建位置有坑**：VMware 共享目录 `/mnt/hgfs/share` 不支持符号链接，
`--symlink-install` 会在 `os.symlink` 上抛 `OSError: 95 Operation not supported`。
要么把工作区复制到原生路径（本次用 `/home/qq/mm_ws`）再构建，要么去掉 `--symlink-install`
（但那样改代码后要重新 build）。

### 需要真机才能验证（当前均未验证）

| 项 | 为什么没验证 | 首跑怎么确认 |
|---|---|---|
| SocketCAN 打开 can0、收 0x601..0x604 | 没有 CAN 硬件 | `ip link` + `candump` + 节点启动日志 |
| 驱动器使能/模式应答 0x700 的时序 | 需要真驱动器 | 启动握手日志；超时会列出缺失驱动器 |
| 速度给定 1 LSB = 1 rpm | 源码只发过 0 | 小转速核对；不对就改 `SPEED_VALUE_RPM_PER_LSB` |
| 编码器/转速/右轮取负的实车符号 | 只能上车看 | 0.1 m/s 前进，四轮 `/joint_states` 位置应同向增长 |
| 1.35 m 里程计轮距 | 厂家标定值，未在本车复测 | 直线 5 m + 原地转 90°，对比卷尺/角度 |
| 驱动器内部速度环带宽/稳态误差/低速死区 | 需要真驱动器 | 台架阶跃 + 编码器记录 |
| 转矩模式 + 上位 PI | 本版未实现 | 先反推转矩单位，再移植厂家 PI |
| 故障码清报警/恢复流程 | 需要真驱动器 | 人为触发故障，确认节点锁存、重启后清报警 |

---

## 九、已知限制与下一步

1. **没有加减速规划**：`/cmd_vel` 阶跃直接变轮速阶跃。真机首跑必须小步给；下一步加
   `max_accel/max_yaw_accel`（厂家导航 1.0 m/s²、0.6 rad/s²）。
2. **没有 TF**：只发 `/odom`，不发 `odom→base_link`。导航层需要时再加。
3. **没有单轮方向修正**：`invert_side` 只翻整车两侧；单轮反了要改接线/驱动器参数。
4. **没有温度/电流保护**：0xCA 电流与 0xCB 温度已解析，但节点只锁存故障码，不做降额。
5. **没有转矩模式**：见第四节，列为下一步。
6. **回环不是物理仿真**：不模拟打滑、惯量、电流环、CAN 延迟；它只证明协议、符号和积分
   公式自洽，不能替代台架。
7. **1.35 m 是厂家值**：本车轮胎/地面不同，必须按 `zeus_calibrate_angular` 流程重标；
   重标前所有“原地转角度准确”的结论都不成立。
8. **四轮缺一则里程计停更**：`DifferentialOdometry` 要求四个轮子都有新计数才积分，
   一路反馈断了就只保持旧位姿；这是“宁可停更，不拿三轮猜”的选择，不是自动容错。
9. **没有反馈看门狗**：`/cmd_vel` 超时会清零，但如果 0x601..0x604 全部收不到而
   `/cmd_vel` 仍在发，节点会继续按最后目标发速度。下一步加 `feedback_timeout`，
   反馈断了也清零；现在真机上必须同时监控 `/odom` 是否在更新。
