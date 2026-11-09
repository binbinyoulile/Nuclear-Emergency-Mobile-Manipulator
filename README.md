<div align="center">

<img alt="version" src="https://img.shields.io/badge/version-v1.0.0-2f6feb?style=for-the-badge" />
<img alt="license" src="https://img.shields.io/badge/license-MIT-yellow?style=for-the-badge" />
<img alt="python" src="https://img.shields.io/badge/python-3.10%2B%20%7C%203.13.12-3776AB?style=for-the-badge&logo=python&logoColor=white" />
<img alt="mujoco" src="https://img.shields.io/badge/MuJoCo-3.16.0-orange?style=for-the-badge" />
<img alt="mink" src="https://img.shields.io/badge/IK-mink%201.3.0-7b61ff?style=for-the-badge" />
<img alt="opencv" src="https://img.shields.io/badge/OpenCV-5.0.0-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" />
<img alt="regression" src="https://img.shields.io/badge/seed_0--20-21%2F21%20passed-brightgreen?style=for-the-badge" />
<img alt="platform" src="https://img.shields.io/badge/platform-Windows%20%7C%20Linux-lightgrey?style=for-the-badge&logo=windows&logoColor=white" />

</div>

# 双车协同核应急移动操作臂仿真

[GitHub 仓库](https://github.com/binbinyoulile/Nuclear-Emergency-Mobile-Manipulator) ·
[架构说明](ARCHITECTURE.md)

基于 MuJoCo 的双车协同核废料回收项目：UR10e 移动操作臂负责雷达避障、D435i 深度相机识别、抓取与投放；运输车负责动态停靠和屏蔽桶转运。项目以纯 Python 状态机实现，并保留清晰的 ROS2 / 真机迁移接口。

## 效果展示

| 长距离场景总览 | 一号车雷达避障 |
| --- | --- |
| ![长距离场景总览](docs/screenshots/dual_overview.png) | ![一号车雷达避障](docs/screenshots/dual_car1_avoid.png) |

| 双车协同抓取 | 投放：监测棒落进屏蔽桶 | 两车精准返航 |
| --- | --- | --- |
| ![双车协同抓取](docs/screenshots/dual_parallel_grasp.png) | ![投放落桶](docs/screenshots/dual_bucket_release.png) | ![两车精准返航](docs/screenshots/dual_final_returned.png) |

## 核心能力

- **双车协同**：共享 `CarController` 底盘控制，实现差速运动、平滑加减速、雷达局部避障和动态停靠。
- **D435i 视觉**：离屏渲染深度图，先最小二乘拟合地面平面，再取"离地 0.8~30 mm"的点云做连通域与 PCA 轴线拟合，最后按两端上表面高差判定粗端并反投影出世界坐标；不依赖颜色、不读真值。
- **机器人操作**：UR10e + Robotiq 2F-85，使用 mink IK 抓住不锈钢监测棒的粗端，抬起搬运后把质心对准屏蔽桶中心、在桶口上方松爪，靠自由落体落进桶底。
- **任务闭环**：起点出发、长距离避障、并行调度、抓取投放、倒车脱离辐射区、正向精准返航。
- **迁移友好**：视觉的数据获取与坐标变换层、底盘和机械臂控制层均有明确接口，便于替换为 ROS2 话题和 TF2。

## 底盘模型

两辆车按企业真实重载底盘参数建模（`models/universal_robots_ur10e/dual_mobile_ur10e.xml`）：

| 参数 | 数值 |
|---|---:|
| 整车质量（底盘 + 四轮） | `75 kg`（车体 `65 kg` + 四轮 `10 kg`） |
| 外形尺寸 长×宽×高 | `0.912 × 0.710 × 0.410 m` |
| 离地间隙 | `0.11 m` |
| 轴距 | `0.50 m`（前轮 X=`+0.25`，后轮 X=`-0.25`） |
| 轮距 | `0.555 m`（左轮 Y=`+0.2775`，右轮 Y=`-0.2775`） |
| 轮径 | `0.305 m`（轮心 Z=`0.1525`） |
| 顶面（机械臂/雷达安装板） | `z = 0.40356` |

- 物理碰撞体是一个按真实包络定位的 Box（车体 `65 kg`），四轮按真实轴距/轮距布置，各 `2.5 kg`。
- 纯外观件（车身外壳、前后防撞管、顶部安装板、雷达托盘）全部 `contype="0" conaffinity="0"`、`mass="0"`，不参与物理。
- 已知偏差：参数表标注"四轮差速转向"，但控制器运动学使用前轮转向角；去掉转向节后实测原地转仅 `0.13 rad/s` 且车体滑移 `0.3 m`，任务定位精度不足，因此暂保留前轮转向节作为等效机构，待控制器改为纯差速转向后再移除。
- 机械臂模型（Menagerie UR10e）自身约 `33.8 kg`，因此一号车整机仿真质量约 `108.8 kg`（真实 75 kg 底盘 + UR10e + 夹爪）。

## 被操作对象与屏蔽桶

放射源按真实不锈钢监测棒建模（`dual_car_scene.xml` 的 `grasp_object`）：

| 参数 | 数值 |
|---|---:|
| 粗端 / 细端 | `Ø6 mm × 33 mm` / `Ø5 mm × 169 mm` |
| 总长 | `202 mm` |
| 材料 | 不锈钢 `7930 kg/m³`，整件约 `33.5 g`，质心距粗端 `92 mm` |
| 外观 | 端面 0.25 mm 倒角网格（`models/chopstick/chopstick.obj`），银灰高光 |
| 碰撞体 | 两段标准圆柱（长度各缩 0.3 mm 代表倒角），`condim=6`，滚动摩擦 `0.03` |

- 物体原点放在**粗端圆柱中心**：真值点、视觉输出的粗端点、抓取点三者天然重合。
- 二号车屏蔽桶是 32 边圆筒（内径 `Ø240 mm`、深 `260 mm`、底厚 `20 mm`）。桶口比监测棒长，
  夹住粗端的机械臂把质心对准桶心后松爪，监测棒整根自由落进桶底。
- 为什么桶口必须大于杆长：刚性杆只能从"开口 ≥ 杆长"或"倾斜着"进入更窄的洞。实测 `Ø100` / `Ø130`
  桶口时，水平杆必然横架在桶沿上；把夹持力从 `5.1 N` 提到 `48.4 N` 也压不进去（杆变成刚性探针、
  机械臂直接压不动），夹持力小时杆反而被桶沿顶出夹爪。另：2F-85 夹住杆后无法施加俯仰力矩
  （实测工具下俯 58°、杆纹丝不动），所以杆只能水平搬运。

## 环境要求

- Python 3.10+
- Windows、Linux 或 macOS
- 支持 OpenGL 的运行环境（实时预览需要）
- 当前基准环境：Python 3.13.12、`mujoco 3.16.0`、`mink 1.3.0`、`opencv-python 5.0.0.93`、`numpy 2.5.4`、`scipy 1.18.1`

## 快速开始

### Windows PowerShell

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1
.\.venv\Scripts\python.exe scripts\fetch_models.py
.\.venv\Scripts\python.exe .\dual_car_pick_place.py --seed 7
```

### Linux / macOS

```bash
bash setup.sh
source .venv/bin/activate
python scripts/fetch_models.py
python dual_car_pick_place.py --seed 7
```

无窗口验收并更新核心截图：

```powershell
.\.venv\Scripts\python.exe .\dual_car_pick_place.py --snapshot --seed 7
```

`setup.sh` / `setup.ps1` 只创建项目内 `.venv`，不会使用 `--system-site-packages`。脚本运行结束后会提示模型资产下载命令。

## 模型资产

MuJoCo 网格文件体积较大，因此 `models/**/assets/`、`*.obj` 和 `*.stl` 已加入 `.gitignore`。仓库保留必要的 MJCF、许可证和说明文件；新环境执行：

```powershell
python scripts\fetch_models.py
```

脚本从 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) 获取：

- [Universal Robots UR10e](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur10e)
- [Robotiq 2F-85](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85)

自建的 `dual_mobile_ur10e.xml` 将 UR10e、2F-85、移动底盘和 D435i 安装关系组合在同一 MuJoCo 层级中。

## 工程结构

```text
mobile_manipulator/
├── dual_car_pick_place.py   # 双车任务状态机、雷达避障、视觉与协同投放
├── mobile_pick_place.py     # 共享底盘、IK、夹爪与仿真运动底层
├── dual_car_scene.xml       # 双车主场景
├── arm_ik_scene.xml         # 机械臂 IK 辅助场景
├── models/                  # 自建 MJCF；二进制 mesh 由脚本下载
├── scripts/fetch_models.py  # Menagerie 资产下载脚本
├── scripts/run_matrix.py    # 多 seed 回归矩阵（CSV + 每 seed 日志）
├── docs/screenshots/        # 5 张核心展示截图
├── requirements.txt
├── setup.ps1
└── setup.sh
```

## 任务流程

1. 两车从红线后方的固定起点出发。
2. 一号车通过雷达局部避障前往目标区，二号车在收到一号车停稳坐标后并行出发。
3. 一号车用 D435i 深度图识别不锈钢放射源：观测位先粗定位，再开到 1.41 m 测量位精定位，
   打印像素、深度、视觉坐标和真值误差。
4. UR10e 自上而下抓住监测棒的**粗端**（离地夹持点高度按夹爪实测几何取 18.5 mm），二号车动态停靠到绿框外：全程纯追踪平滑转弯入位，
   到点**完全不原地转**——差速原地转在地面上是轮胎打滑，只为补正几度就会把车整体
   滑移 2 cm，而且正好发生在机械臂准备投放、车身必须绝对静止的时刻。
   停靠点 Y 预留 0.06 m 车身投影余量，任何朝向下都越不过绿框边界。
   一号车抓取前同理：只有残余偏航超过 12° 才原地摆正，实测 21 个种子残差都在
   0.1°~9.2°，全部直接留着不动车。
5. 一号车把监测棒送到二号车屏蔽桶口：先按实测杆轴把质心对到桶心，再在桶口上方 6 cm
   松爪，监测棒整根自由落体落进桶底；随后两车倒车脱离辐射区并正向精准返航。
6. 终态检查：物体在桶内、两车回到起点、车车物理接触数为 0。

目标区按 UR10e 工作空间收紧为 `0.70 × 0.60 m`（X `±0.35`，Y `20.70 ~ 21.30`），
`dual_car_pick_place.py` 的 `TARGET_ZONE_*` 常量与 `dual_car_scene.xml` 的四个绿色边框
geom 一一对应，改一处必须同步另一处。

监测棒长 202 mm、body 原点在粗端截面中心、生成时沿世界 `+X` 平放，所以采样不是在框内
均匀取原点，而是**按整根杆的占地范围收缩**：搜索窗（`±0.30 m`）减去两端半长（粗端
`16.2 mm`、细端 `185.2 mm`）再留 `15 mm` 余量，得到 `OBJECT_X_RANGE = -0.269 ~ +0.100`、
`OBJECT_Y_RANGE = 20.78 ~ 21.22`。区间由杆的几何自动推出，杆一改长就直接失效报错，
不会静默生成"两头戳出绿框"的非法用例——这类用例会让相机拍不全、臂展够不到，
反映出的是测试用例不合理，而不是控制或视觉算法的缺陷。

## 视觉四层架构

1. `get_camera_data()`：MuJoCo `Renderer` 采集 RGB/depth；ROS2 中替换为 D435i 图像话题订阅。
2. `depth_to_world()`：整幅深度图反投影成世界点云（`fx=fy=(H/2)/tan(fovy/2)`，相机 +Z 与视线反向，深度取负）。
3. `detect_steel_target()`：点云先最小二乘拟合地面平面，取离地 0.8~30 mm 的点，连通域只保留与搜索区相交者，
   二维 PCA 求轴线、1%/99% 分位定两端，再比较两端外侧 20% 的上表面高度（Ø6 粗端比 Ø5 细端高 0.5 mm）判定粗端。
4. 任务状态机：仅使用视觉返回坐标执行抓取，真实物体坐标只用于误差验证。

为什么不用颜色：不锈钢本色在中性灰地面上没有色差。为什么不能拿单帧深度当绝对高度：渲染器深度带约 0.15% 的系统性尺度偏差，
2 m 外的地面会"高出" 1.5 mm，必须先用地平面拟合把它吸收掉。

## 多 seed 回归与验证基线

单次成功不能说明可靠性，因此每次改动后跑同一组种子：

```powershell
D:\Python313\python.exe .\scripts\run_matrix.py --seeds 0-20
```

脚本逐个起 `--headless` 子进程（不覆写截图），单种子超时保护，把指标写入
`reports/regression.csv`、每个种子的完整日志写入 `reports/logs/seed_<n>.log`。
任务本身在成功/失败时都会打印一行 `RESULT_JSON:` 并以退出码 `0`/`1` 结束，
失败时 `stage` 字段直接指出卡在哪个阶段。

`seed 0–20`（21 个）当前结果（不锈钢监测棒 + Ø240 屏蔽桶）：

- 通过率：`21/21`
- D435i 视觉坐标误差：`0.83 ~ 2.66 mm`
- 桶内质心 XY 误差：`0.001 ~ 0.009 m`（容差 `0.05 m`）
- 投放后监测棒倾角：`63.5 ~ 67.0°`（整根横躺进桶，不立起、不架在桶沿）
- 返航后物体仍在桶内：`21/21`（终态水平残差 `0.003 ~ 0.011 m`）
- 停靠时刻两车中心距：安全阈值 `0.82 m`，全程最小中心距 `0.977 ~ 1.091 m`，物理接触 `0`
- 回起点误差：CAR1 `0.084 ~ 0.086 m`，CAR2 `0.090 ~ 0.096 m`（阈值 `0.20 m`）
- 单次仿真时长：`59.7 ~ 61.4 s`

回归矩阵覆盖的是"随机落点 + 固定流程"，随机区间本身就是用例的一部分：202 mm 的杆一旦
戳出视觉搜索窗，观测位就拍不全、粗端判错约一个杆长；更极端的落点还会让车停在相机下视场
之外、机械臂够不到。这类失败反映的是用例不合理而非算法缺陷，因此采样区间按杆的两端半长
从搜索窗内缩（见上文"被操作对象与屏蔽桶"），21 个种子生成的杆全部完整落在窗内。

历史基线快照保存在 `reports/baselines/`（`v1.0.0.csv` / `stage0_sync.csv` / `stage0_refactor.csv`），
可用 `python scripts/compare_metrics.py a.csv b.csv` 做逐指标对比，验证重构前后是否等价。

## 版本与里程碑

### v1.0.0 — 仿真基线闭环（2026-11-07，tag `v1.0.0`）

第一版"物理可信、逻辑闭环、可回归"的仿真基线，作为后续 Isaac Sim / 真机迁移的对照基准。

已闭环的内容：

- **底盘物理保真**：按企业参数表重构 75 kg 真实重载车（车体 65 kg + 四轮 10 kg、
  912×710×410 mm、轮径 305 mm、轴距 500 mm、轮距 555 mm、四轮独立驱动、前后防撞管），
  视觉件 `mass=0` 且不参与碰撞。
- **两车协同全流程**：雷达避障 → 视觉识别（误差 8.5~10.0 mm）→ 侧面抓取 →
  动态停靠（不原地转、车身投影留在绿框外）→ 协同投放（桶内误差 1~5 mm）→
  倒车脱离 → 正向精准返航（回起点 0.08~0.10 m）。
- **物理真实性**：全程无穿模、无翻车、无"锁死物理/瞬移"作弊；车车物理接触 0；
  抓取与投放期间两车底盘逐帧静止（只有机械臂反作用造成的毫米级微动）。
- **可回归**：`seed 0–20` 21/21 通过，指标落 `reports/regression.csv`，
  单次仿真 54~63 s，本机约 20 分钟跑完一轮。

复现这一版：

```powershell
git checkout v1.0.0
D:\Python313\python.exe .\scripts\run_matrix.py --seeds 0-20
```

### 已知限制（迁移前需要解决）

- 前轮转向节是"等效机构"：控制器运动学用转向角 + 轮速差，与厂家"四轮差速转向"
  不完全一致；改纯差速需重写横摆控制并重新标定（见 `底盘模型` 一节）。
- 雷达检测距离 2.8 m 小于 3.5 m 制动距离，限速与避障阈值还没有做联合标定。
- 摩擦、静摩擦门限、接触参数仍是仿真整定值，未经真实轮胎辨识。
- 视觉是渲染深度图 + 地平面拟合，真机需要标定（内外参、深度尺度）与去噪流程替换。

## 跨平台路径

脚本统一以 `Path(__file__).resolve().parent` 定位项目文件。MuJoCo 加载前会临时切换到场景目录并使用相对文件名，规避 Windows 中文绝对路径兼容问题，同时保持 Linux 路径可用。

## License

本项目自建代码与场景采用 [MIT License](LICENSE)，Copyright (c) 2026 Haobin Zhu。

第三方资产沿用其上游许可证：UR10e 模型文件遵循 MuJoCo Menagerie 附带的 BSD-3-Clause License；Robotiq 2F-85 资产按其上游许可证使用。
