# 真车厂家工作区参考（2022-09-03 版）

老师给的 `slam20220903` 是厂家真车上的 ROS1 工作区原件，放在
`D:\桌面\机械臂\vehicle_workspace\slam20220903`（109.7 MB、823 个文件、5 个嵌套 git 仓库），
本仓库不复制其内容。本文只记**查实的事实**与**对我们有用的数字**，每条都指到具体文件。

用途：写 ROS2 底盘驱动、填真车 `base_link` 偏移、定真机首跑速度时照这个查，不要凭猜。

---

## 一、CAN 报文（原"唯一硬卡点"，现已完整）

厂家实现见 `src/zeus_s2/zeus_s2_driver/src/zeus_s2_driver.cpp`。
标准帧，DLC=8，多字节数值**大端**。

### 命令帧（上位机 → 驱动器）：ID = `0x101 + (n-1)`，n = 1..4

| 动作 | data[0..7] | 源码 |
| --- | --- | --- |
| 使能 | `00 24 00 10 00 01 00 00` | `enable_motor_driver` |
| 失能 | `00 1E 00 10 00 00 00 00` | `disable_motor_driver` |
| 清报警（须先失能） | `00 21 00 10 00 00 00 01` | `clear_ALM_motor_driver` |
| 设为转矩模式 | `00 1E 00 03 00 01 00 00` | `Set_motor_torque_mode` |
| 设为速度模式 | `00 1E 00 03 00 02 00 00` | `Set_motor_speed_mode` |
| 给转矩值 | `00 28 00 01 hi lo 00 00` | `Set_torque_value_to_motor` |
| 给速度值 | `00 28 00 00 hi lo 00 00` | `Set_speed_value_to_motor` |

规律：`data[1]` 是命令码（`0x1E` 写参数、`0x24` 使能、`0x21` 清报警、`0x28` 设运行值），
`data[2..3]` 是参数索引（`0x0010` 使能、`0x0003` 运行模式、`0x0001` 转矩、`0x0000` 速度），
`data[4..5]` 是 16 位数值。

### 反馈帧（驱动器 → 上位机）：ID = `0x601..0x604`

| data[1] | 含义 |
| --- | --- |
| `0xCA` | `data[2..5]` int32 大端 = 编码器计数；`data[6..7]` = 电流（千分单位） |
| `0xCB` | `data[2..3]` 温度；`data[4..5]` int16 = 实际转速（源码里取负号 `motorActSpeed = -(...)`）；`data[6..7]` = 故障码 |

另有 ID `0x700` 模式应答：`data[0]` = 电机号，`data[1]=0x1F` 表示模式已生效；
使能应答为 `data[1]=0x25`、`data[3]=0x10`、`data[5]=1`。

轮序（源码注释 + 手册"车头顺时针 1→4"一致）：
`0x101` 左前、`0x102` 右前、`0x103` 右后、`0x104` 左后。

### 波特率

**1000 kbps**。源码 `Timing0=0x00 / Timing1=0x14`，注释写明 `1000kbps [0x00 0x14]`、
`500 Kbps [0x00 0x1C]`。→ 与产品说明书的 1000 一致，**课程实验指导书写的 500 kbps 是错的**。

### 传输层

- 高配（Jetson）：Linux **SocketCAN `can0`**，`can-ctrl.c` 用 `AF_CAN` 裸套接字，无需厂商库；
- 低配（Intel）：USB-CAN-II 分析仪，走 `VCI_*` 接口 + 随包 `libcontrolcan.so`。

→ 我们的 ROS2 驱动用 SocketCAN 即可；低配时换个适配器，协议层不变。

### 不要照抄的部分

`zeus_s2` 包本身是 Apache-2.0，但 `can-ctrl.c/h` 另带 HopeMotion 的限制性声明（限定只能跑在它的
硬件上），`lib*.so` 是厂商二进制、`LSLIDAR` / `rplidar_ros` / `laser_filters` 是各自的上游包。
协议可以照用，代码不走这条路——我们自己写 SocketCAN 收发，也就几十行。

---

## 二、单位换算（驱动里在跑的常量）

| 量 | 值 / 公式 | 出处 |
| --- | --- | --- |
| 编码器每电机转 | 10000 count（2500 线 × 4 倍频） | `zeus_params.yaml` `encoder_resolution_4wd` |
| 减速比 | 40 | `gear_reduction_4wd` |
| 轮径 | 0.305 m | `wheel_diameter_4wd` |
| 里程标定 | 417 455 count/m，1 count ≈ 2.4 µm | `ticks_per_meter_` 计算式 |
| 转速→线速度 | `rpm / 40 × π×0.305 / 60` | `zeus_s2_driver.cpp` 各轮 `velocity` 行 |
| 校验 | 3000 rpm → 1.199 m/s（与额定 1.20 m/s 吻合） | 反算 |
| 转矩输出钳位 | ±1910 | `zeus_s2_driver.h` `MAX/MIN` |
| 转速环 | Kp 1500 / Ki 25 / Kd 0，125 Hz（8 ms） | `zeus_params.yaml`、`ros::Rate loop_rate(125)` |

**指令分解**（`send_speed_callback`，`wheel_track = 0.61`）：

```
v_L =  v − ω·0.61/2
v_R = −(v + ω·0.61/2)      // 右轮取负：电机镜像安装
原地转（v 恰为 0）：v_L = v_R = −ω·0.61/2
```

**`start_rotation_limit_w = 0.4 rad/s` 的真实作用域**：只在"低速前进（0 < v < 0.1 m/s）
同时要求小幅转向"时把 ω 抬到 0.4，**纯原地转不抬**。所以它不是"原地转最小角速度"，
不能拿它当死区判据；原地转到底能不能动，要在架空台架上用 `bench_chassis.py` 量。

**里程计轮距 `wheel_track_4wd = 1.35 m`**：把左右轮里程折成航向时用的值，是 URDF 几何轮距
0.594 的 2.27 倍，对应说明书"轮距约为真实轮距的 2 倍多"，是厂家用角度校准 launch 标出来的
滑移补偿。含义：同样轮速差，实际横摆约为几何模型的 1/2.27 ≈ 44%。
（交叉参考：我们仿真里 ±8 rad/s 实测横摆/几何 = 0.30，量级一致。但**只能说明量级对，
不能当数值用**——转速、地面不同，标定值就不同。）

---

## 三、安装尺寸（底盘部分 URDF 已给，机械臂缺）

来源 `src/zeus_s2/zeus_s2_description/urdf/zeus_s2-gps_c16_d435.urdf`，
坐标 `base_footprint`（地面，x=车头），单位 m。

| 量 | 值 | 关节 |
| --- | --- | --- |
| 车体参考面（离地） | 0.132 | `base_footprint → car_base_link` |
| 四个轮心 | x = ±0.250，y = ±0.297，z = 0.152 | `wheel_*_joint` |
| 2D 雷达 `laser_link` | (0.354, 0, 0.425) | `laser_joint` |
| D435 机身 `d435_link` | (0.256, 0, 0.396) | `d435_joint` |
| 超声波 ×4 | x = ±0.447，y = ±0.156，z = 0.187 | `radar_*_joint` |
| 塔架 `car_tower_link` | z = 0.409 | `car_tower_joint` |
| Velodyne 座（历史配置） | z = 0.893 | `velodyne_base_mount_joint` |
| 导航用车身包络 | 0.94 × 0.76 | `zeus_s2_navi/config/zeus/costmap_common_params.yaml` |

注意：

- 轮心 x = ±0.250 严格对称，所以 URDF 的 x 原点就是**轴距中点**，与我们的
  `base_link` 定义（地面 + 轴距中点）一致，偏移可直接用；
- 轮心高 0.152 m、标称轮半径 0.1525 m，两者只差 0.5 mm → 这组数自洽，可信；
- 轮距 **0.594**（产品说明书写 555 mm，差 39 mm；驱动指令里用 0.61，更接近 URDF，
  取 0.594 或现场量，见标定表）；
- 机械臂**不在**这份 URDF 里（2022 版装的是 ROCR6），臂座偏移仍待确认，见第五节。

---

## 四、厂家在这台车上实际用的参数（比自创保守值可靠）

| 项 | 值 | 出处 |
| --- | --- | --- |
| 导航最大线速度 / 最小 | 0.5 / 0.08 m/s | `navi/config/zeus/base_local_planner_params.yaml` |
| 最大角速度 | ±1.0 rad/s | 同上 |
| 原地转最小角速度 | 1.0 rad/s | 同上 `min_in_place_vel_theta` |
| 线/角加速度 | 1.0 m/s² / 0.6 rad/s² | 同上 |
| 到点容差 | 0.10 m / 0.05 rad | 同上 |
| 速度平滑上限 | 0.8 m/s / 1.0 rad/s | `zeus_s2_driver/config/standalone.yaml` |
| 速度平滑加速度 | 0.3 m/s² / 3.5 rad/s² | 同上 |
| 手柄遥控倍率 | 0.5 m/s、1.6 rad/s | `vision_rocr6/launch/robot_vision_wrapper_with_zeus.launch` |
| 雷达自遮挡框 | x ∈ [−0.10, 0.20]，y ∈ [±0.25] | `zeus_s2_bringup/config/my_rplidar_filter.yaml` |

对照我们的仿真档位（巡航 0.80 / 接近 0.50 / 倒车 0.30，`MAX_YAW_RATE=1.5`）：
**厂家导航自己只跑 0.5 m/s，0.8 是它的平滑上限**。我们的 0.8 正好压在那条上限上，
建议真机首次落地用 0.5（与厂家导航一致），跑顺了再放到 0.8。

雷达自遮挡框是厂家在 `laser_link` 系里写的**不对称矩形**（前 0.20、后 0.10、左右各 0.25），
而我们的 `lidar_summary` 只有**圆形盲区** `self_radius`（`rho >= self_radius` 才保留），
套不上这个形状。真车上先别填数，把一帧原始扫描打出来看车体回波落在哪一圈，再定半径；
必要时给节点补一个矩形裁剪。仿真里无自遮挡，保持 0。

---

## 五、仍然不能确定的（不要猜，不要写进代码）

1. **转矩单位**。`MAX = 1910` 与额定转矩 1.91 N·m 数值吻合，像 0.001 N·m/count，
   但源码没写单位。**课程实验指导书里也没有**——它明写"报文的解读篇幅过于庞大"并甩了一个
   CSDN 链接（PDF 第 42 页），力矩只出现 `Set_torque_value_to_motor(..., 0)` 这种置零调用。
   所以两条路：查 `LS伺服驱动器通讯使用手册-20200602.pdf`（该 PDF 字体未内嵌，文本提取失败，
   需渲染页面看），或架空上电给一个小转矩，用电流与转速响应反推。
2. ~~**实车当前装的是哪套臂**~~（2026-11-23 已确认：**UR10 + OnRobot RG2**）。2022 工作区是
   ROCR6（`/dev/ROCR6` 串口）+ G3P 三指夹爪（`/dev/g3p`），与本项目无关；本项目已按 UR10 + RG2
   建模并作为默认配置。臂的**安装位置**（车头一侧）仍待厂家尺寸。
3. **里程计包 `zeus_s2_odom` 不在这份拷贝里**。有 launch 引用它、目录不存在。
   但积分公式在课程实验指导书里有（PDF 第 70–71 页）：
   `delta_th_ = ((dR+rR)/2 − (dL+rL)/2) / wheel_track_4wd`、`v = 平均里程差 / dt`，
   所以不必逆向，照这个写即可，`wheel_track_4wd` 作为待标定值。
4. **雷达型号**。URDF 用 `laser_a2.STL` 网格与 `laser_a2.xacro`，与说明书 A2 一致；
   同目录还有 `ls_c16.xacro`（内容是 A2 的拷贝，无独立安装位）和 0.89 m 的 Velodyne 座，
   说明历史上装过三维雷达。当前按 A2（`input:=scan`）为准，上车看一眼确认。

## 六、附带的其它可用资料

| 资料 | 位置 | 能不能用 |
| --- | --- | --- |
| 真车建图地图（2021-11-10） | `zeus_s2_bringup/maps/` | 参考场地尺寸用，分辨率 0.05 m，原点 −7.51/−7.38 |
| 手眼 + 内参标定结果 | `vision_rocr6/config/eyeToHand_calibration.yaml`、`intrinsic_calibration.yaml` | **格式可参考，数值不能用**（ROCR6 + 车头固定 D435i，我们是 UR10 + 腕部相机）。内参 fx≈fy≈1362、cx 974、cy 536 与 1920×1080 的 D435i 相符，可当"标定量级"参照 |
| 建图/导航启动脚本 | 根目录 `buildmap.sh`、`navi.sh`、`savemap.sh` | 记录厂家用 cartographer 2D + move_base + amcl 的老方案，我们的技术路线不同，只作背景 |
| 伺服驱动器手册 ×2、60LCB060C 样本 | `zeus_s2_driver/` 下同名 PDF | 转矩单位与故障码要查这里，见第五节第 1 条 |
| 课程实验指导书（109 页，2022-08） | `hardware_docs/室外重载作业驱控一体底盘-实验指导书.pdf` | 内容是 ROS 课程：小乌龟→话题→RVIZ→URDF→串口→CAN→伺服→传感器→差速→雷达→建图→导航。第七章的驱动源码与我们手上的 `zeus_s2_driver.cpp` 同源。**缺陷：全文插图与公式图片缺失**（"图 x.x"处渲染空白，公式照不出来），文字层完整；CAN 报文格式它没写，别指望它 |
