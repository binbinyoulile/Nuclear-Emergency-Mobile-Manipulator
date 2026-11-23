"""双车协同核废料回收任务层（与后端无关，零 MuJoCo 依赖）。

文件目的：
    把「七阶段任务状态机 + 三个路径控制器 + 抓取/投放编排」收敛成**唯一一份**实现，
    单进程仿真入口（`dual_car_pick_place.py`）与 ROS 2 任务节点
    （`ros2_ws/src/mm_mission/mm_mission/node.py`）共用，避免两份任务逻辑漂移。

依赖与版本：
    只用 numpy、scipy.spatial.transform 和 `robot_interface`；不 import mujoco，
    也不读任何后端全局量——所有硬件访问都经由 `RobotIO` 的端口。

分层约定：
    robot_interface.py      抽象端口（Clock / Chassis / Lidar / Arm / Gripper / Perception）
    mission_core.py         本文件：任务与控制器逻辑，只认端口
    两个后端               Sim* 适配器（MuJoCo）/ Ros* 适配器（ROS 2 话题）

整体输入输出：
    输入为已装配好的 `RobotIO` 与目标真值；输出为 `DualCarMission.metrics` 结构化指标
    和任务成功布尔值。

关键假设：
    两辆车由同一台底盘控制器驱动，雷达只做局部避障、不做全局重规划；
    目标区内的物体位置由近距视觉给出（0.83~2.66 mm 量级）。

已知限制：
    规则控制、理想传感器；雷达 2.8 m 警戒距离是按 0.8 m/s 巡航定的（制动距离约
    0.21 m），换更大负载或更高车速档时必须重新做"速度—检测距离"联合标定。

重要性等级：A"""

from __future__ import annotations

import os
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from robot_interface import (
    ChassisInterface,
    Clock,
    RobotIO,
)

# ---------------------------------------------------------------------------
# 底盘几何与控制参数。
# 由 task 层统一持有：仿真场景（`dual_car_scene_rg2.xml`）、MuJoCo 单进程入口与 ROS 2
# 任务节点必须用同一套数值，否则控制器对车体尺寸的理解和物理模型不一致。
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.1525
WHEEL_TRACK = 0.555
STEER_WHEELBASE = 0.50
STEER_MAX = 0.70
CHASSIS_SETTLE_TIME = 1.2

# 路径跟随参数。V_MAX 是限幅上限，实际速度由各段 max_speed 决定。
V_MAX = 20.0
K_POS = 3.50
# 纵向加减速度上限（m/s²）。真车参数表给出空载最高车速 ≤1.0 m/s（减速比 40、轮径 305 mm
# → 1.0 m/s 对应电机 2505 rpm，2.0 m/s 要 5010 rpm，这套传动做不到），所以各段定档为
# 巡航 0.80 / 持杆·入位 0.50 / 倒车 0.30 m/s；0.8 m/s 时制动距离约 0.21 m，雷达 2.8 m
# 仍有十倍以上余量。见 README「已知限制」里的速度—检测距离标定说明。
ACCEL_UP = 1.5
ACCEL_DOWN = 2.0
COMMAND_RAMP = 0.35

# 三档速度（m/s）。改档位只改这里：各段 max_speed 与长直线段的航点超时都跟着走。
CRUISE_SPEED = 0.80
APPROACH_SPEED = 0.50
REVERSE_SPEED = 0.30

# 长直线段的航点超时按"这段要走多远"给，不按秒写死：
#   [s] = PATH_TIMEOUT_BASE + PATH_TIMEOUT_SLACK × 距离 ÷ 速度
# 系数 3.0 由实测标定：绕行、转弯降速与两端停靠都要吃时间——19 m 直线在 0.8 m/s 下
# 理论 24 s，实际用掉约 80 s。速度档改小后超时自动放大，不会再出现"改了速度、旧超时
# 把正常跑的车判成超时"（2026-11-21 踩过一次：0.8 档沿用 2.0 档的 22 s，seed 7 停在半路）。
PATH_TIMEOUT_SLACK = 3.0
PATH_TIMEOUT_BASE = 10.0
# 短距离挪车段（0.3~1.5 m）的超时：由刹车、小步调整与等同伴主导，与速度无关，单独给常数。
SHORT_MANEUVER_TIMEOUT = 36.0


def leg_timeout(distance: float, speed: float) -> float:
    """按距离预算给长直线段的航点超时，单位 s。"""
    return PATH_TIMEOUT_BASE + PATH_TIMEOUT_SLACK * distance / max(speed, 1e-3)

# 路径跟随允许的最大横摆角速度，单位 rad/s。
# 纯追踪本身不限幅：5.3 m/s 时它会要求 7.6 rad/s，左右轮速差约 ±29 rad/s，实测直接
# 把车甩飞（车体升到 0.59 m、落地悬空卡死）。所以横摆角速度在这里单独限到 1.5。
MAX_YAW_RATE = 1.5

# 原地转向的轮速命令，单位 rad/s。
# 108 kg 时代实测：±2.5 rad/s 低于轮胎静摩擦门限（四轮空转、偏航只有 0.026 rad/s），
# ±8.0 rad/s 时约 1.32 rad/s，再高会明显平移离开对准点。
# 按实车布置整车加到 141.6 kg 后，8 rad/s 只能转过去一部分（连续三次摆正残差
# 35.7° → 6.9° → 6.4° 能收敛，但也有种子停在 18~21° 不动）；试过 12 rad/s
# （按正压力比例补偿静摩擦门限），反而过冲到 yaw −1.7 rad、横向滑出 0.3 m，
# 所以留 8 rad/s，转不齐的情形由"测量位失败退回观测位粗定位"兜住。
# 真机上限仍是空载 1.0 m/s，8 rad/s × 0.1525 = 1.22 m/s 只存在于仿真；真机摆正必须
# 按实车实测重标，且只用于补几度偏航。
SKID_TURN_WHEEL_SPEED = 8.0
# 原地转向起转与收尾的轮速斜坡时间，单位 s。
# 直接写 0 时车身仍带约 1.5 rad/s 横摆角速度，轮胎被瞬间拖停：实测定位点被
# 拖走 25 mm，画面上是一次突然抽动；起转也只给一帧满速，同样看着像抽搐。
# 起转 0.20 s 内升到满速、到位后 0.35 s 内降到 0，都靠轮胎摩擦自然过渡。
SKID_TURN_RAMP_TIME = 0.20
SKID_TURN_SPINDOWN_TIME = 0.35

# ---------------------------------------------------------------------------
# 机械臂与夹爪：姿态、站点高度与动作时长。
# ---------------------------------------------------------------------------
GROUND_Z = 0.0
# 夹爪几何（OnRobot RG2，以 pinch 站点为原点）：两指垫各半高 14.75 mm、中心与 pinch
# 同高（`scripts/check_rg2_model.py` 实测 dz=0），所以夹持面覆盖 pinch 上下各 14.75 mm。
# 由此推出"自上而下夹一根平躺圆棒"的可行带：pinch 离地 14.75~17.75 mm——低了指垫底面
# 压进地面，高了夹持面盖不到棒身最宽处（Ø6 杆中心离地 3 mm）。2F-85 旧模型的指垫长
# 3 mm，那条带是 17.7~20.7 mm。
# 取带内靠下沿的 0.0155 而不是中值 0.01625：两指仍能把整根棒身（中心离地 3 mm、最高点
# 6 mm）完整裹住，允许 ±5 mm 定位误差；取中值时指垫底缘只比杆顶低 0.75 mm，ROS 侧实测
# 只要高 3 mm 就变成"指垫坐在杆顶上"，夹爪闭合了却抬不起来。
PINCH_ABOVE_FLOOR = 0.0155
GRASP_WORLD_Z = GROUND_Z + PINCH_ABOVE_FLOOR
APPROACH_WORLD_Z = GRASP_WORLD_Z + 0.17
LIFT_WORLD_Z = GRASP_WORLD_Z + 0.30
# 工具长度：RG2 的 pinch 在腕部原点外 0.2900 m，2F-85 旧模型是 0.2558 m（短 34.2 mm）。
# 抓取点高度保持 0.45 m 不做补偿：换长工具后腕部自然高出 34 mm，离底盘甲板更远。真正
# 要处理的是 IK 分支——不给种子时，更长的工具会把解推到"腕部转 180°"的另一支上（实测
# joint1 从 −4.07 变成 +0.95），腕部相机跟着转走，观测位连杆都拍不到。
# 搬运姿态，位于底盘轮廓内；**固定基座坐标系**（arm_ik_scene.xml 的 base 体）。
CARRY_FIXED = np.array([0.20, 0.0, 0.45])
# 2F-85 时代的搬运姿态解，作为 RG2 的 IK 种子，保证落在同一支上（见上面的说明）。
CARRY_SEED_Q = np.array(
    [-4.070902, -2.609742, 2.224456, -1.185309, -1.574098, -0.928937]
)
# 工具坐标系 +Y 轴朝下，wxyz。
TARGET_QUAT = np.array([0.0, 1.0, 0.0, 0.0])
# 底盘臂座系 → 固定基座（IK 模型）系的旋转固定值。arm_ik_scene.xml 里 base 体相对底盘
# 转了 180°（建模时把 URDF 的 +Y 朝前翻成 +X 朝前），所以同一个物理点在这两个坐标系里
# 的 x/y 都反号：基座系 (0.20, 0, 0.45) 对应 chassis 臂座系 (-0.20, 0, 0.45)。
# 任何"把固定基座系目标换算到世界系"的后端都必须乘它（顺序见 RosArm.start_fixed），
# 漏掉不会报错，只会让机械臂安静地走到镜像位置上——历史上真的这样错过一次。
# 单进程后端不需要它：solve_ik_fixed 直接吃固定基座系，没有中间坐标系。
FIXED_BASE_ROT = np.diag([-1.0, -1.0, 1.0])
# RG2 的夹爪执行器 ctrl 直接就是半开度 g（米）：0.055 全开（指面间隙 110 mm）、
# 0.0005 全闭（间隙 1 mm，远小于 Ø6 杆，所以能夹住）。
FINGER_OPEN = 0.055
FINGER_CLOSE = 0.0005

# 动作时长，单位 s。
APPROACH_TIME = 2.0
DESCEND_TIME = 1.5
# 抓取位伺服静差的收敛小步：只走几毫米，给 0.5 s 足够（比 DESCEND_TIME 短得多，
# 不改变整体节拍，只补掉 3~4 mm 的跟踪误差）。
GRASP_SETTLE_TIME = 0.5
CLOSE_TIME = 1.5
LIFT_TIME = 1.8
CARRY_TIME = 2.8

# ---------------------------------------------------------------------------
# 任务几何布局
# ---------------------------------------------------------------------------
# 目标区按 UR10 工作空间收紧：车停在 (x, y-0.85)，抓取点只有 0.94 m 远
# （水平 0.85 + 垂直 0.39，臂展 1.3 m，约 72%），横向摆动也从 ±0.70 收到 ±0.28。
TARGET_ZONE_X_MIN = -0.35
TARGET_ZONE_X_MAX = 0.35
TARGET_ZONE_Y_MIN = 20.70
TARGET_ZONE_Y_MAX = 21.30
# 视觉搜索窗＝目标区内缩 4~5 cm，把贴地的绿色边框（高 0.8 mm，也落在离地带内）
# 挡在搜索范围之外；边框内沿在 X=±0.332、Y=20.718/21.282。
STEEL_ROI_X = (TARGET_ZONE_X_MIN + 0.05, TARGET_ZONE_X_MAX - 0.05)
STEEL_ROI_Y = (TARGET_ZONE_Y_MIN + 0.04, TARGET_ZONE_Y_MAX - 0.04)
# 被操作对象是 202 mm 长杆，body 原点在粗端截面中心、生成时沿世界 +X 平放，
# 因此从原点量起：到粗端端面 16.2 mm、到细端末端 185.2 mm。采样时必须让整根杆
# 都留在搜索窗内——杆一戳出窗，观测位就会拍不全（粗端判错、误差约一个杆长），
# 测量位甚至分割不到。那是测试用例本身不合理，不是控制或视觉算法的问题。
ROD_BACK_HALF = 0.0162
ROD_FRONT_HALF = 0.1852
OBJECT_SPAWN_MARGIN = 0.015
OBJECT_X_RANGE = (
    STEEL_ROI_X[0] + OBJECT_SPAWN_MARGIN + ROD_BACK_HALF,   # -0.2688
    STEEL_ROI_X[1] - OBJECT_SPAWN_MARGIN - ROD_FRONT_HALF,   # +0.0998
)
OBJECT_Y_RANGE = (20.78, 21.22)
OBJECT_Z = 0.003   # 平躺在 1 号车前的粗端轴心高度（粗端 Ø6 mm）
if not OBJECT_X_RANGE[0] < OBJECT_X_RANGE[1]:
    raise ValueError("rod no longer fits inside the vision search window")
# 监测棒质心到 body 原点（粗端截面中心）的轴向距离，单位 m。
# 数值由 models/chopstick 的惯性参数导出（body_ipos=[0.07908824, 0, 0]），
# 单进程入口启动时会用 MODEL.body_ipos 复核一次；ROS 侧没有模型，直接用本常数。
ROD_COM_OFFSET_M = 0.0790882374823302

CAR1_START = np.array([-0.70, -1.00])
CAR2_START = np.array([0.90, -1.20])
CAR_START_YAW = np.pi / 2.0
CAR1_SEARCH_STOP = np.array([0.0, 18.50])
CAR1_SEARCH_PATH = [
    (float(CAR1_SEARCH_STOP[0]), float(CAR1_SEARCH_STOP[1])),
]
RETURN_LINE_Y = -0.25
RADIATION_EXIT_Y = 17.50
# 二号车停靠中心 Y：绿框下边界再往南 0.64 m。
# 车体在任意朝向下投影最大 0.578 m（0.912×0.710 的车斜着放最宽），留 0.06 m 余量，
# 这样即使纯追踪到点留下十几度残余偏航，车身上沿也越不过 TARGET_ZONE_Y_MIN；
# 不再需要靠停靠后原地转摆正来"压窄"车身投影。
CAR2_DOCK_CENTER_Y = TARGET_ZONE_Y_MIN - 0.64
DOCK_OFFSET_MIN = 0.75
# 停靠点侧向余量，单位 m。
# 视觉阶段只能用"预测的一号车位姿"，实测偏差可达 0.2 m；不留余量会让
# 实际中心距跌破 CAR2_MIN_SEPARATION，二号车被自身安全间距规则卡在原地。
DOCK_POSE_MARGIN = 0.20
# 二号车先在停靠点正南这么远集结，再直线北上停靠，避免贴近一号车横向挪车。
CAR2_STAGING_DY = 1.20
CAR2_MIN_SEPARATION = 0.82

# ---------------------------------------------------------------------------
# 屏蔽桶与投放
# ---------------------------------------------------------------------------
BUCKET_BASE_Z = 0.40356                 # 二号车顶面
BUCKET_HEIGHT = 0.26                    # 屏蔽桶总高（底 20 mm + 壁 240 mm）
BUCKET_FLOOR_Z = BUCKET_BASE_Z + 0.02   # 桶底上表面
BUCKET_OPENING_Z = BUCKET_BASE_Z + BUCKET_HEIGHT
# 投放是纯重力自由落体：桶内径 0.40 m 比 0.202 m 的杆长得多，只要把**质心**对到
# 桶心、在桶口上方 6 cm 松爪，杆就整根落进桶底。更窄的桶口（Ø100 / Ø130 / 原版 Ø240）
# 都试过：桶口越小，可容忍的桶心偏差越小（Ø240 只有 18 mm，真机定位误差就吃光了余量），
# 所以按企业确认加宽到 Ø400（约 98 mm 余量）。加大夹持力没用——杆会变成刚性探针。
PLACE_APPROACH_TIME = 2.4
PLACE_DESCEND_TIME = 4.0
PLACE_INSERT_Z = BUCKET_OPENING_Z + 0.06    # 松爪时夹持点的高度：杆悬在桶口正上方
PLACE_SETTLE_TIME = 2.0
# 落桶验收：直接量"整根杆在不在桶里"，不再用"质心离桶心多远"。
# 质心容差是照着"杆横躺"这一种姿态定的代理量：横躺时杆两端离质心 95 / 106 mm，
# 质心偏十几毫米就有一头探出桶口；而杆立着时只占 Ø6 一小块，质心偏 100 mm 也还在桶内。
# 同一条容差对两种姿态自相矛盾（实测 ROS 侧杆会被甩成竖直落入），因此改成几何判据。
BUCKET_MOUTH_RADIUS = 0.20              # 桶口内接圆半径（32 边形内切圆，Ø400）
ROD_THIN_TIP = 0.1852                   # body 原点 → 细端尖端（碰撞体实测）
ROD_THICK_TIP = 0.0162                  # body 原点 → 粗端尖端
ROD_RADIUS = 0.003                      # 粗端半径
BUCKET_EDGE_MARGIN = 0.003              # 端点离桶口内壁至少留这么多
BUCKET_FLOOR_MARGIN = 0.02              # 端点允许低于桶底平面的量

# 一号车停车距离：抓取位按机械臂可达性取 0.85 m；近距测量位取 1.85 m，腕部相机
# 离目标约 1.26 m（视场中心线附近），详见 DualCarMission.run 中的注释。
# 为什么不是早期的 1.41 m：机械臂座前移到平板中心（+0.188 m）并抬高 56 mm 后，相机跟着
# 前移、抬高，杆在画面里的位置整体下移——1.55 m 时实测视场最近只到车前方 1.53 m
# （再近就被夹爪自身挡住），Ø6 细杆正好卡在画面最底边，深度图上只剩 1 px 宽、连通域断开，
# 报"离地带为空 / 没有落在搜索区里的连通域"。1.85 m 把杆拉回视场中部，与观测位（2.1 m）
# 相比仍是更近的一档。
GRASP_STANDOFF_Y = 0.85
MEASURE_STANDOFF_Y = 1.85

# ---------------------------------------------------------------------------
# 激光雷达：任务层自己解读的四个扇区定义 + 扫描周期。
# 射线数量、俯仰分辨率等纯建模参数留在后端（本层不需要）。
# ---------------------------------------------------------------------------
LIDAR_PERIOD = 0.10
# 2.8 m 是按 0.8 m/s 巡航定的：制动距离约 0.21 m，留十倍余量。早期 6.5 m/s 时代这个
# 距离比制动距离还短，才需要"速度—检测距离"联合标定；现在换更大负载或更高车速档时仍要
# 重新算一遍。注意调大检测距离会让长距离返航提前偏航（实测有 seed 绕到 y=28），动之前
# 先看返航段。
LIDAR_DETECT_RANGE = 2.8
LIDAR_FRONT_HALF = np.deg2rad(30.0)
# 绕行方向记忆：同一障碍的反复重入不再重新选边。ROS 侧二号车卡死时日志里
# "发现 0.27 m 障碍 → 左绕 → 已清" 刷了 65 次，而实际左右在交替翻转
# （1.6 s 内 LEFT→RIGHT→LEFT），车头左右摆、净位移为零。1.5 s 足够覆盖
# 帧级闪烁的周期，又短到换一个新障碍时会重新判断。
AVOID_SIDE_HOLD = 1.5
# 卡死告警：距航点还很远却没有前进速度，打一行日志，不用等 waypoint 超时。
STALL_WARN_TIME = 1.0
STALL_WARN_ERR = 0.5
STALL_WARN_SPEED = 0.02
LIDAR_SIDE_MAX = np.deg2rad(90.0)
LIDAR_REAR_HALF = np.deg2rad(30.0)

# 工具自旋候选角：抓取时在可达性等价解中挑最优。
# 角度以 pinch 站点自身坐标系为准。候选必须让**两指开合方向垂直于杆轴**（杆沿世界 x，
# 所以开合方向要落在世界 ±y）：2F-85 的 pinch 站点把开合方向定在站点 +y 上，±90° 正好；
# OnRobot RG2 的指沿站点 **+x** 开合（见 models/onrobot_rg2/rg2_subtree.xml），必须改用
# 0°/180°。用错时两指沿杆轴张合，下降时指垫直接压在杆身上，pinch 停在离地 20 mm
# （实测），夹爪空合。切回 2F-85 时这一行也要一起改回。
GRASP_SPIN_AXIS = "z"
GRASP_SPIN_CANDIDATES_DEG = (0.0, 180.0)

# 调试开关：DUAL_TRACE=1 时按 0.25 s 输出路径跟随器内部状态。
_TRACE = os.environ.get("DUAL_TRACE") == "1"

# 快照回调签名：(名称, lookat, distance, azimuth, elevation)。
SnapshotHook = Callable[[str, list[float], float, float, float], None]


def wrap_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi)。

参数：
    angle: 弧度角，可为任意实数。
返回：
    等价的最小表示角，单位 rad。
用途：
    计算车头与目标方向的短路径角误差，避免跨 pi 时反向急转。
重要性等级：B"""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _peer(
    cars: tuple[ChassisInterface, ...] | list[ChassisInterface],
    car: ChassisInterface,
) -> ChassisInterface:
    """从车辆集合里取出另一辆车（同伴）。

参数：
    cars: `RobotIO.chassis`，必须同时持有两辆车。
    car: 本车。
返回：
    同伴车的端口。
异常：
    集合里没有第二辆车时抛 RuntimeError。
重要性等级：B"""
    for other in cars:
        if other is not car:
            return other
    raise RuntimeError("RobotIO.chassis 必须同时持有两辆车，才能计算同伴距离")


def _sync_viewer(io: RobotIO) -> None:
    """触发后端注册的每帧渲染回调（无渲染后端时是空操作）。"""
    if io.frame_hook is not None:
        io.frame_hook()


# ---------------------------------------------------------------------------
# 底盘路径控制器（非阻塞：调用方每帧 update 一次）
# ---------------------------------------------------------------------------
class PathFollower:
    """非阻塞的全局目标跟踪与雷达局部避障控制器。

控制思路：
    正常阶段始终朝当前航点前进；只有前向警戒区出现障碍时才产生局部转角，障碍消失后立即恢复直线目标。
平滑策略：
    使用加速度限制、转向降速和接近目标的平方根制动，避免急起急停。
重要性等级：A"""

    def __init__(
        self,
        car: ChassisInterface,
        path: list[tuple[float, float]],
        clock: Clock,
        waypoint_timeout: float,
        final_tolerance: float,
        max_speed: float = V_MAX,
    ) -> None:
        """初始化路径、时序、雷达避障和速度状态。

参数：
    car: 被控制车辆。
    path: 非空航点列表，单位 m。
    clock: 帧时钟，提供 now 与 dt。
    waypoint_timeout/final_tolerance/max_speed: 超时 s、终点容差 m、最大速度 m/s。
异常：
    path 为空时抛 ValueError。
重要性等级：A"""
        if not path:
            raise ValueError("path must not be empty")
        self.car = car
        self.path = [(float(x), float(y)) for x, y in path]
        self.clock = clock
        self.waypoint_timeout = waypoint_timeout
        self.final_tolerance = final_tolerance
        self.max_speed = float(max_speed)
        self.index = 0
        self.waypoint_start = clock.now
        self.done = False
        self.avoid_active = False
        self.avoid_side = 0
        self.avoid_start = 0.0
        self.avoid_stop_until = 0.0
        self._side_memo = 0.0
        self._side_memo_until = 0.0
        self._stall_since: float | None = None
        self._stall_warned = False
        self._pivot_dir = 0.0
        self.speed_cmd = 0.0
        self.frame_dt = float(clock.dt)
        print(f"\n[{car.name}] lidar path: {len(path)} waypoint(s)")
        self._print_waypoint()

    def _print_waypoint(self) -> None:
        """打印当前航点序号和坐标。

副作用：
    仅输出终端日志，不改变控制状态。
重要性等级：C"""
        wx, wy = self.path[self.index]
        print(
            f"  waypoint {self.index + 1}/{len(self.path)}: "
            f"({wx:.2f}, {wy:.2f})"
        )

    def append_path(self, extra: list[tuple[float, float]]) -> bool:
        """在尚未完成时追加航点。

参数：
    extra: 新航点列表，单位 m。
返回：
    True 表示已追加；控制器已完成或列表为空时返回 False。
重要性等级：B"""
        if self.done or not extra:
            return False
        self.path.extend((float(x), float(y)) for x, y in extra)
        print(
            f"  [{self.car.name}] path extended by {len(extra)} waypoint(s)"
        )
        return True

    def replace_final_waypoint(
        self, waypoint: tuple[float, float]
    ) -> bool:
        """不中断行驶地替换最后一个航点。

参数：
    waypoint: 新终点 (x, y)，单位 m。
返回：
    True 表示已更新；控制器已完成时返回 False。
用途：
    二号车根据视觉结果动态修正停靠点，避免半路停车再启动。
重要性等级：A"""
        if self.done:
            return False
        self.path[-1] = (float(waypoint[0]), float(waypoint[1]))
        self.waypoint_start = self.clock.now
        print(
            f"  [{self.car.name}] final waypoint updated to "
            f"({waypoint[0]:.3f}, {waypoint[1]:.3f}) without stopping"
        )
        return True

    def update(self, cars: tuple[ChassisInterface, ...] | list[ChassisInterface]) -> bool:
        """推进一个控制帧的纯追踪、雷达避障和速度分配。

参数：
    cars: 全部车辆，用于取同伴车位姿。
返回：
    True 表示路径完成，否则 False。
异常：
    单航点超过 waypoint_timeout 时抛 RuntimeError。
关键参数：
    LIDAR_DETECT_RANGE 决定提前避障距离；CAR2_MIN_SEPARATION 用于避免两车过近。
重要性等级：A"""
        if self.done:
            return True
        car = self.car
        car.lidar.scan_if_due()
        pos, yaw = car.pose()
        wx, wy = self.path[self.index]
        dx = float(wx - pos[0])
        dy = float(wy - pos[1])
        distance = float(np.hypot(dx, dy))
        tolerance = (
            self.final_tolerance
            if self.index == len(self.path) - 1
            else 0.30
        )
        if distance < tolerance:
            if self.index == len(self.path) - 1:
                self.done = True
                self.speed_cmd = 0.0
                car.set_wheels(0.0, 0.0, 0.0)
                print(
                    f"  [{car.name}] path complete at "
                    f"({pos[0]:.3f}, {pos[1]:.3f})"
                )
                return True
            self.index += 1
            self.waypoint_start = self.clock.now
            self._print_waypoint()
            wx, wy = self.path[self.index]
            dx = float(wx - pos[0])
            dy = float(wy - pos[1])
            distance = float(np.hypot(dx, dy))

        if self.clock.now - self.waypoint_start > self.waypoint_timeout:
            raise RuntimeError(
                f"{car.name} waypoint {self.index + 1} timeout at "
                f"({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f}, "
                f"z={pos[2]:.3f}, speed={car.speed():.2f} m/s, "
                f"target=({wx:.2f}, {wy:.2f})"
            )

        other_pos, _ = _peer(cars, car).pose()
        separation = float(np.linalg.norm(pos[:2] - other_pos[:2]))
        car.min_other_separation = min(car.min_other_separation, separation)
        # 只有“开向目标会继续靠近另一辆车”时才倒车；否则即使当前距离偏近也要
        # 允许继续前进，否则路过同伴时会被这条规则永久钉住。
        to_goal_dir = np.array([dx, dy], dtype=float) / max(distance, 1e-6)
        away_dir = (pos[:2] - other_pos[:2]) / max(separation, 1e-6)
        closing_in = separation < CAR2_MIN_SEPARATION and float(
            np.dot(to_goal_dir, away_dir)
        ) < 0.2
        if closing_in:
            alpha = wrap_angle(
                float(np.arctan2(pos[1] - other_pos[1], pos[0] - other_pos[0]))
                - yaw
            )
            forward = -0.18
        else:
            alpha = wrap_angle(float(np.arctan2(dy, dx)) - yaw)
            forward = float(np.clip(K_POS * distance, 0.0, self.max_speed))
            forward *= max(0.0, float(np.cos(alpha)))
            if abs(alpha) < np.deg2rad(95.0):
                forward = max(forward, 0.15)

        # 绕行偏置前的真实航向误差，用于判断要不要原地掉头。
        heading_error = alpha

        if car.lidar.front_blocked:
            # 仅做局部避障：宏观目标方向始终指向终点。
            if car.lidar.front_lateral <= 0.85:
                if not self.avoid_active:
                    self.avoid_active = True
                    # 同一个障碍的反复重入沿用上次的绕行方向：左右交替会让车头
                    # 来回摆、净位移为零（ROS 侧实测）。
                    if self.clock.now >= self._side_memo_until:
                        if abs(car.lidar.front_lateral_signed) < 0.05:
                            self._side_memo = (
                                -1.0
                                if car.lidar.right_min >= car.lidar.left_min
                                else 1.0
                            )
                        else:
                            self._side_memo = (
                                -1.0 if car.lidar.front_lateral_signed > 0.0 else 1.0
                            )
                        self._side_memo_until = self.clock.now + AVOID_SIDE_HOLD
                    self.avoid_side = self._side_memo
                    self.avoid_start = self.clock.now
                    side_name = "RIGHT" if self.avoid_side < 0 else "LEFT"
                    print(
                        f"  [{car.name}] {LIDAR_DETECT_RANGE:.1f} m radar warning: "
                        f"obstacle {car.lidar.front_min:.2f} m, "
                        f"local detour {side_name}"
                    )
                bias = np.deg2rad(55.0 if car.lidar.front_min < 1.0 else 35.0)
                alpha = wrap_angle(alpha + self.avoid_side * bias)
        elif self.avoid_active:
            print(f"  [{car.name}] obstacle cleared: resume direct goal")
            self.avoid_active = False
            self.avoid_side = 0

        # 平滑纵向速度曲线：长直线全速，转弯降速，接近终点按平方根曲线制动。
        if forward >= 0.0:
            turn_factor = 1.0 - 0.55 * min(
                abs(alpha) / (np.pi / 2.0), 1.0
            )
            goal_speed = forward * turn_factor
            braking_speed = np.sqrt(
                max(0.0, 2.0 * 1.10 * max(distance - 0.05, 0.0))
            )
            goal_speed = min(goal_speed, braking_speed)
        else:
            goal_speed = forward
        accel = ACCEL_UP if goal_speed > self.speed_cmd else ACCEL_DOWN
        max_delta = accel * self.frame_dt
        self.speed_cmd += float(
            np.clip(goal_speed - self.speed_cmd, -max_delta, max_delta)
        )
        forward = self.speed_cmd

        # 目标在侧后方时线速度会归零，纯转向又没有角速度权限，会永久卡死：
        # 这里直接用高于静摩擦门限的差速原地转。
        # >95° 时即使正在绕行也必须先掉头，否则 cos<0 让绕行也没有前进量。
        pivoting = (
            abs(heading_error) > np.deg2rad(95.0)
            or (not self.avoid_active and abs(heading_error) > np.deg2rad(80.0))
            # 车几乎停下时纯追踪给不出角速度，40° 以上偏差就会永远磨不过去。
            or (forward < 0.30 and abs(heading_error) > np.deg2rad(45.0))
        )
        terminal_straight = (
            self.index == len(self.path) - 1
            and distance < 0.80
            and abs(heading_error) < np.deg2rad(50.0)
            # 末段直行锁定前轮、不再修正横向偏差，所以必须确认现有车头方向真的
            # 指向终点：航向偏 20° 时直线开 0.8 m 就偏出 0.27 m，车会从终点旁边
            # 擦过去，越过终点后 heading_error 翻号、pivot 反复触发，绕圈十几秒
            # 也回不到位（seed 4 实测）。用"车头延长线离终点的横向距离"当判据，
            # 物理含义就是这条直线能不能压到终点上。
            and distance * abs(float(np.sin(heading_error))) < 0.06
        )
        # 掉头优先级高于终点直行：两者可能同时成立（终点直行看的是绕行锁存航向），
        # 若先走直行分支会把左右轮速同时写成 0，彻底卡死。
        if not pivoting:
            self._pivot_dir = 0.0
        if pivoting:
            # 目标正好在正后方时航向误差会在 ±π 之间跳变，若每帧重取符号会原地抖动，
            # 因此进入掉头瞬间锁存方向。
            if self._pivot_dir == 0.0:
                self._pivot_dir = 1.0 if heading_error > 0.0 else -1.0
            # 左右反向驱动原地转；轮速必须高于静摩擦门限（实测 ±2.5 rad/s 只打滑）。
            left_target = -SKID_TURN_WHEEL_SPEED * self._pivot_dir
            right_target = SKID_TURN_WHEEL_SPEED * self._pivot_dir
            steer_target = 0.0
        elif terminal_straight:
            steer_target = 0.0
            left_target = forward / WHEEL_RADIUS
            right_target = forward / WHEEL_RADIUS
        else:
            lookahead = float(np.clip(distance, 0.65, 1.80))
            steer_target = float(
                np.arctan2(
                    2.0 * STEER_WHEELBASE * np.sin(alpha),
                    lookahead,
                )
            )
            steer_target = float(
                np.clip(steer_target, -STEER_MAX, STEER_MAX)
            )
            # 最后 1.1 m 内把方向盘平滑回正：否则低速误差会让前轮来回打方向，
            # 车体被惯性带着侧滑抖动（用户观察到的"揉库"）。
            if self.index == len(self.path) - 1:
                steer_target *= float(
                    np.clip((distance - 0.30) / 0.80, 0.0, 1.0)
                )
            yaw_rate = (
                forward * float(np.tan(steer_target)) / STEER_WHEELBASE
                if forward > 0.02
                else 0.0
            )
            yaw_rate = float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE))
            left_target = (
                forward - yaw_rate * WHEEL_TRACK / 2.0
            ) / WHEEL_RADIUS
            right_target = (
                forward + yaw_rate * WHEEL_TRACK / 2.0
            ) / WHEEL_RADIUS

        car.left_cmd += COMMAND_RAMP * (left_target - car.left_cmd)
        car.right_cmd += COMMAND_RAMP * (right_target - car.right_cmd)
        car.steer_cmd += COMMAND_RAMP * (steer_target - car.steer_cmd)
        car.set_wheels(car.left_cmd, car.right_cmd, car.steer_cmd)
        # 卡死告警：距航点还很远却没有前进速度，超过 STALL_WARN_TIME 打一行，
        # 不用等 waypoint 超时才在错误信息里看到。
        stuck = (
            not pivoting
            and distance > STALL_WARN_ERR
            and abs(forward) < STALL_WARN_SPEED
        )
        if not stuck:
            self._stall_since = None
            self._stall_warned = False
        elif self._stall_since is None:
            self._stall_since = self.clock.now
        elif (
            not self._stall_warned
            and self.clock.now - self._stall_since >= STALL_WARN_TIME
        ):
            self._stall_warned = True
            print(
                f"  [{car.name}] WARNING 原地磨 "
                f"{self.clock.now - self._stall_since:.1f} s："
                f"speed={forward:.2f}, 距航点 {distance:.2f} m, "
                f"heading_err={np.rad2deg(heading_error):.0f}°, "
                f"blocked={int(car.lidar.front_blocked)}, "
                f"detour={int(self.avoid_active)}, fmin={car.lidar.front_min:.2f}"
            )
        if _TRACE and self.clock.now - getattr(self, "_next_trace", 0.0) >= 0.25:
            self._next_trace = self.clock.now
            print(
                f"TRACE {car.name} t={self.clock.now:6.2f} "
                f"pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:.2f}) "
                f"yaw={yaw:+.2f} err={heading_error:+.2f} fwd={forward:+.2f} "
                f"blocked={int(car.lidar.front_blocked)} "
                f"fmin={car.lidar.front_min:.2f} "
                f"avoid={int(self.avoid_active)} side={self.avoid_side:+.0f} "
                f"pivot={int(pivoting)} term={int(terminal_straight)} "
                f"cmd=({car.left_cmd:+.1f},{car.right_cmd:+.1f}) "
                f"st={car.steer_cmd:+.2f} "
                f"wq=["
                + " ".join(f"{v:+.1f}" for v in car.wheel_speeds())
                + "]"
            )
        return False


class SkidTurnFollower:
    """用较小轮速差和前轮辅助完成重载四轮车的短距离转向。

重要性等级：B"""

    def __init__(
        self,
        car: ChassisInterface,
        target_heading: float,
        clock: Clock,
        timeout: float = 8.0,
        ramp_time: float = 0.0,
    ) -> None:
        """初始化目标朝向和转向超时。

参数：
    car: 被控制车辆。
    target_heading: 目标偏航角，单位 rad。
    clock: 帧时钟。
    timeout: 超时时间，单位 s。
    ramp_time: 收尾轮速斜坡时间，单位 s；0 表示瞬时起停（旧行为）。
        传正值时同时启用起转斜坡 SKID_TURN_RAMP_TIME。
重要性等级：B"""
        self.car = car
        self.target_heading = float(target_heading)
        self.clock = clock
        self.start_time = clock.now
        self.timeout = float(timeout)
        self.done = False
        self._next_log = 0.5
        self._cmd_mag = 0.0
        self._spin_dir = 0.0
        self._spindown_t0: float | None = None
        self.ramp_time = float(ramp_time)
        print(
            f"  [{car.name}] steering-assist turn to heading "
            f"{self.target_heading:.3f} rad"
        )

    def update(self, cars) -> bool:
        """推进一帧转向对齐。

参数：
    cars: 场景车辆列表，接口兼容用（本控制器只动自己）。
返回：
    True 表示已对齐，否则 False。
异常：
    超过 timeout 仍未对齐时抛 RuntimeError。
重要性等级：B"""
        del cars
        if self.done:
            return True
        car = self.car
        pos, yaw = car.pose()
        error = wrap_angle(self.target_heading - yaw)
        # 航向容差 6°：抓取与投放都用世界坐标解 IK，几度偏差不影响可达性，
        # 却能让车少做一次原地"揉库"。
        tolerance = np.deg2rad(6.0)
        # 转向方向在开始转的同一帧锁定，且绝不反向找补：
        # 航向差接近 ±180° 时 wrap_angle 的符号会逐帧跳变，逐帧取符号会让车先反着
        # 转半圈再转回来（画面上就是"原地绕了一大圈"）；越过目标就收尾，也不会转满一圈。
        if self._spin_dir == 0.0:
            self._spin_dir = 1.0 if error < 0.0 else -1.0
        overshot = (self._spin_dir < 0.0) != (error > 0.0)
        if self._spindown_t0 is None and (abs(error) <= tolerance or overshot):
            if self._cmd_mag <= 0.0 or self.ramp_time <= 0.0:
                # 从来没转过（到点已在容差内）或不开斜坡：直接结束，不动车。
                self.done = True
                car.set_wheels(0.0, 0.0, 0.0)
                print(
                    f"  [{car.name}] heading aligned at yaw={yaw:.3f} rad, "
                    f"pose=({pos[0]:.3f}, {pos[1]:.3f})"
                )
                return True
            # 到位后不再瞬间写 0：按 ramp_time 把轮速线性降下来。
            self._spindown_t0 = self.clock.now
            print(
                f"  [{car.name}] heading aligned at yaw={yaw:.3f} rad, "
                f"pose=({pos[0]:.3f}, {pos[1]:.3f}), spinning down"
            )
        if self._spindown_t0 is not None:
            frac = (self.clock.now - self._spindown_t0) / self.ramp_time
            if frac >= 1.0:
                self.done = True
                car.set_wheels(0.0, 0.0, 0.0)
                return True
            speed = self._cmd_mag * max(0.0, 1.0 - frac)
            car.set_wheels(
                speed * self._spin_dir, -speed * self._spin_dir, 0.0
            )
            return False
        if self.clock.now - self.start_time > self.timeout:
            raise RuntimeError(f"{car.name} final heading alignment timeout")

        direction = self._spin_dir
        if self.ramp_time > 0.0:
            ramp = min(
                1.0,
                (self.clock.now - self.start_time) / SKID_TURN_RAMP_TIME,
            )
            self._cmd_mag = SKID_TURN_WHEEL_SPEED * max(ramp, 0.0)
        else:
            self._cmd_mag = SKID_TURN_WHEEL_SPEED
        if self.clock.now - self.start_time >= self._next_log:
            self._next_log += 0.5
            print(
                f"    [align] t={self.clock.now - self.start_time:.2f}s "
                f"yaw={yaw:.3f} err={error:.3f} "
                f"pos=({pos[0]:.3f}, {pos[1]:.3f}) z={pos[2]:.3f} "
                f"dir={direction:+.0f} "
                f"wq=["
                + " ".join(f"{v:+.1f}" for v in car.wheel_speeds())
                + "]"
            )
        # 纯差速原地转：前轮不再转向。实测给前轮 -0.65 rad 反而削弱偏航（1.32 → 0.82 rad/s）。
        car.set_wheels(
            self._cmd_mag * direction,
            -self._cmd_mag * direction,
            0.0,
        )
        return False


class ReverseStraightFollower:
    """沿当前车头方向直线倒车，并以后向雷达限制速度。

重要性等级：B"""

    def __init__(
        self,
        car: ChassisInterface,
        target_y: float,
        clock: Clock,
        max_speed: float = 0.55,
    ) -> None:
        """初始化倒车目标 Y、最大速度和状态。

参数：
    car: 被控制车辆。
    target_y: 世界 Y 目标，单位 m。
    clock: 帧时钟。
    max_speed: 倒车最大速度，单位 m/s，内部按负速度指令使用。
重要性等级：B"""
        self.car = car
        self.target_y = float(target_y)
        self.clock = clock
        self.max_speed = float(max_speed)
        self.done = False
        start_pos, start_yaw = car.pose()
        # 锁定起步时的车道与朝向：倒车若只“沿车头方向”走，3~5° 的航向残差
        # 在 2.4 m 行程上会横向漂 0.2 m，两车会互相逼近。
        self.lane_x = float(start_pos[0])
        self.base_heading = float(start_yaw)
        print(
            f"  [{car.name}] straight reverse to Y={self.target_y:.2f}, "
            f"heading={start_yaw:.3f} rad, lane_x={self.lane_x:.3f}"
        )

    def update(self, cars) -> bool:
        """推进一帧直线倒车。

参数：
    cars: 车辆列表，供雷达周期扫描与同伴距离判断。
返回：
    True 表示到达目标 Y，否则 False。
关键参数：
    后向 1.20 m 开始降速、0.50 m 停止，避免倒车碰撞。
重要性等级：B"""
        if self.done:
            return True
        car = self.car
        if self.clock.now - car.lidar.last_scan_time >= 0.50:
            car.lidar.scan()
        pos, yaw = car.pose()
        if pos[1] <= self.target_y + 0.08:
            self.done = True
            car.set_wheels(0.0, 0.0, 0.0)
            print(
                f"  [{car.name}] reverse complete at "
                f"({pos[0]:.3f}, {pos[1]:.3f})"
            )
            return True

        # 车尾净空：用后半平面回波（含斜后方），并额外确认与另一辆车的距离，
        # 避免只盯 ±30° 车尾扇区而漏掉并排倒车的同伴。
        clearance = min(car.lidar.rear_clear, car.lidar.rear_min)
        other_pos, _ = _peer(cars, car).pose()
        gap = float(np.linalg.norm(pos[:2] - other_pos[:2]))
        previous_gap = getattr(car, "_reverse_gap", None)
        car._reverse_gap = gap
        # 只有“距离在缩小”才停，避免两车都停住互相等待导致超时。
        closing = previous_gap is not None and gap < previous_gap - 1e-4
        speed = -self.max_speed
        if clearance < 1.20:
            speed = max(speed, -0.28)
        if clearance < 0.60:
            speed = 0.0
        if gap < 0.85 and closing:
            speed = 0.0
        elif gap < 1.05:
            speed = max(speed, -0.35)
        wheel = speed / WHEEL_RADIUS
        # 车道保持：航向残差 + 横向偏差折算成期望横摆角速度，再用前轮转角实现。
        # 低速小差速低于轮胎静摩擦门限（实测差速 ±0.7 rad/s 只有 1.6% 的转向效果），
        # 所以这里走转向角而不是轮速差。
        heading_err = wrap_angle(self.base_heading - yaw)
        lateral_err = self.lane_x - pos[0]
        yaw_cmd = float(
            np.clip(1.5 * heading_err + 1.2 * lateral_err, -0.30, 0.30)
        )
        tan_steer = (
            yaw_cmd * STEER_WHEELBASE / speed
            if abs(speed) > 0.05
            else 0.0
        )
        steer_target = float(np.clip(np.arctan(tan_steer), -0.45, 0.45))
        car.left_cmd += COMMAND_RAMP * (wheel - car.left_cmd)
        car.right_cmd += COMMAND_RAMP * (wheel - car.right_cmd)
        car.steer_cmd += COMMAND_RAMP * (steer_target - car.steer_cmd)
        car.set_wheels(car.left_cmd, car.right_cmd, car.steer_cmd)
        if _TRACE and self.clock.now - getattr(car, "_rev_trace", 0.0) >= 0.5:
            car._rev_trace = self.clock.now
            print(
                f"REV {car.name} t={self.clock.now:6.2f} "
                f"pos=({pos[0]:+.3f},{pos[1]:+.2f}) yaw={yaw:+.3f} "
                f"lat={lateral_err:+.3f} yawcmd={yaw_cmd:+.3f} "
                f"v={speed:+.2f} gap={gap:.2f} clear={clearance:.2f}"
            )
        return False


# ---------------------------------------------------------------------------
# 底盘动作编排：停车稳定、走一条路径
# ---------------------------------------------------------------------------
def stop_and_settle(io: RobotIO, car: ChassisInterface) -> None:
    """刹停底盘并推进若干帧等待悬挂和车身稳定。

参数：
    io: 任务层硬件端口。
    car: 要停稳的车辆。
副作用：
    清零运动指令、保持机械臂与夹爪，并持续刷新画面和物理状态。
重要性等级：B"""
    car.left_cmd = 0.0
    car.right_cmd = 0.0
    car.steer_cmd = 0.0
    car.set_wheels(0.0, 0.0, 0.0)
    steps = max(1, int(round(CHASSIS_SETTLE_TIME / io.clock.dt)))
    for _ in range(steps):
        io.arm.hold()
        io.gripper.hold()
        _sync_viewer(io)
        io.clock.step()


def drive_path(
    io: RobotIO,
    car: ChassisInterface,
    path: list[tuple[float, float]],
    waypoint_timeout: float = 55.0,
    final_tolerance: float = 0.16,
    max_speed: float = V_MAX,
) -> None:
    """阻塞执行一条路径，直到 PathFollower 完成或超时。

参数：
    io: 任务层硬件端口。
    car: 要行驶的车辆。
    path: [(x, y), ...] 世界坐标航点，单位 m。
    waypoint_timeout: 单航点超时，单位 s。
    final_tolerance: 终点容差，单位 m。
    max_speed: 本段最大线速度，单位 m/s。
异常：
    超时由 PathFollower.update 抛 RuntimeError。
副作用：
    推进物理与画面，结束后调用 stop_and_settle。
重要性等级：A"""
    cars = list(io.chassis)
    follower = PathFollower(
        car,
        path,
        io.clock,
        waypoint_timeout=waypoint_timeout,
        final_tolerance=final_tolerance,
        max_speed=max_speed,
    )
    while not follower.done:
        io.arm.hold()
        io.gripper.hold()
        follower.update(cars)
        _sync_viewer(io)
        io.clock.step()
    stop_and_settle(io, car)
    pos, yaw = car.pose()
    print(f"  [{car.name}] stopped at ({pos[0]:.3f}, {pos[1]:.3f})")


def run_turn_with_secondary(
    io: RobotIO,
    turner: SkidTurnFollower,
    secondary: PathFollower,
) -> None:
    """阻塞推进主车转向，同时让二号车继续按路径行驶。

参数：
    io: 任务层硬件端口。
    turner: 主车的转向控制器。
    secondary: 可并行的二号车 PathFollower。
副作用：
    每帧保持机械臂与夹爪，更新两车控制并推进时钟。
重要性等级：B"""
    cars = list(io.chassis)
    while not turner.done:
        io.arm.hold()
        io.gripper.hold()
        turner.update(cars)
        if not secondary.done:
            secondary.update(cars)
        _sync_viewer(io)
        io.clock.step()


def run_two_straight_reverses(
    io: RobotIO,
    followers: list[ReverseStraightFollower],
) -> float:
    """并行推进两辆车的直线倒车控制器。

参数：
    io: 任务层硬件端口。
    followers: ReverseStraightFollower 列表。
返回：
    倒车过程中两车中心距的最小值，单位 m。
时序：
    第一辆车先倒车 2 s，第二辆再起步，使两车在纵向上错开，
    避免并排同时倒退时车尾互相逼近（车头雷达正后方是盲区）。
副作用：
    每帧刷新画面、推进物理；全部完成后返回。
重要性等级：B"""
    cars = list(io.chassis)
    lead_until = io.clock.now + 2.00
    # 两条倒车车道互相背离 0.12 m：两车并排只有约 1.0 m 中心距，
    # 各自几度航向残差就会在 2.4 m 行程上收敛到危险距离。
    for follower in followers:
        other_pos, _ = _peer(cars, follower.car).pose()
        own_pos, _ = follower.car.pose()
        away = 1.0 if own_pos[0] >= other_pos[0] else -1.0
        follower.lane_x += away * 0.12
    min_gap = float("inf")
    while any(not follower.done for follower in followers):
        io.arm.hold()
        io.gripper.hold()
        for index, follower in enumerate(followers):
            if index > 0 and io.clock.now < lead_until:
                # 明确保持静止，不让残留指令把车往前带。
                follower.car.set_wheels(0.0, 0.0, 0.0)
                continue
            if not follower.done:
                follower.update(cars)
        _sync_viewer(io)
        io.clock.step()
        if len(followers) >= 2:
            first_pos, _ = followers[0].car.pose()
            second_pos, _ = followers[1].car.pose()
            min_gap = min(
                min_gap, float(np.linalg.norm(first_pos[:2] - second_pos[:2]))
            )
    return min_gap


def run_two_followers(
    io: RobotIO,
    followers: list[PathFollower],
) -> None:
    """并行推进两辆车的 PathFollower 直到全部结束。

重要性等级：B"""
    cars = list(io.chassis)
    while any(not follower.done for follower in followers):
        io.arm.hold()
        io.gripper.hold()
        for follower in followers:
            if not follower.done:
                follower.update(cars)
        _sync_viewer(io)
        io.clock.step()


def run_followers_until_primary_done(
    io: RobotIO,
    primary: PathFollower,
    secondary: PathFollower,
) -> None:
    """推进主车直到完成，二号车在未完成前继续并行跟踪。

重要性等级：B"""
    cars = list(io.chassis)
    while not primary.done:
        io.arm.hold()
        io.gripper.hold()
        primary.update(cars)
        if not secondary.done:
            secondary.update(cars)
        _sync_viewer(io)
        io.clock.step()


def finish_follower(
    io: RobotIO,
    follower: PathFollower,
) -> None:
    """阻塞推进二号车路径直到完成。

异常：
    PathFollower.update 的超时异常原样传播。
重要性等级：B"""
    cars = list(io.chassis)
    while not follower.done:
        io.arm.hold()
        io.gripper.hold()
        follower.update(cars)
        _sync_viewer(io)
        io.clock.step()


# ---------------------------------------------------------------------------
# 机械臂与夹爪动作编排（每帧同时推进二号车）
# ---------------------------------------------------------------------------
def _update_follower(
    follower: PathFollower | None,
    cars: tuple[ChassisInterface, ...] | list[ChassisInterface],
) -> None:
    """若二号车路径控制器仍存在且未完成，则推进一帧。

参数：
    follower: 可空的 PathFollower。
    cars: 车辆接口集合。
重要性等级：C"""
    if follower is not None and not follower.done:
        follower.update(list(cars))


def run_arm_motion_concurrent(
    io: RobotIO,
    follower: PathFollower | None = None,
) -> None:
    """推进机械臂运动，同时让二号车继续路径跟踪。

关键时序：
    arm.active 期间每帧先更新机械臂，再更新 follower，最后推进时钟。
重要性等级：A"""
    while io.arm.active:
        io.arm.tick()
        io.gripper.hold()
        _update_follower(follower, io.chassis)
        _sync_viewer(io)
        io.clock.step()
    io.arm.hold()


def run_finger_motion_concurrent(
    io: RobotIO,
    follower: PathFollower | None = None,
) -> None:
    """推进夹爪开合，同时让二号车继续路径跟踪。

重要性等级：A"""
    while io.gripper.active:
        io.arm.hold()
        io.gripper.tick()
        _update_follower(follower, io.chassis)
        _sync_viewer(io)
        io.clock.step()
    io.gripper.hold()


def run_wait_concurrent(
    io: RobotIO,
    duration: float,
    follower: PathFollower | None = None,
) -> None:
    """保持机械臂与夹爪指定时长，同时允许二号车继续行驶。

参数：
    io: 任务层硬件端口。
    duration: 等待时间，单位 s。
    follower: 可空的二号车路径控制器。
关键参数：
    至少执行一步，避免零时长导致控制循环空转。
重要性等级：B"""
    steps = max(1, int(round(duration / io.clock.dt)))
    for _ in range(steps):
        io.arm.hold()
        io.gripper.hold()
        _update_follower(follower, io.chassis)
        _sync_viewer(io)
        io.clock.step()


# ---------------------------------------------------------------------------
# 抓取与投放
# ---------------------------------------------------------------------------
def start_arm_world_flexible_quat(spin_deg: float) -> np.ndarray:
    """构造"工具竖直 + 抓取自旋"的工具四元数（固定基座坐标系，wxyz）。

    参数：
        spin_deg: 抓取时选定的工具自旋角，单位 deg。
    重要性等级：A"""
    q_base = Rotation.from_quat(TARGET_QUAT, scalar_first=True)
    return (
        q_base * Rotation.from_euler(GRASP_SPIN_AXIS, np.deg2rad(spin_deg))
    ).as_quat(scalar_first=True)


def held_rod_axis(io: RobotIO) -> np.ndarray:
    """读出被夹住的不锈钢监测棒的实际轴向（世界系）。

    参数：
        io: 任务层硬件端口，对象位姿来自 PerceptionInterface.object_pose。
    返回：
        单位向量 shape=(3,)。
    说明：
        夹爪只能传递绕竖直轴的转动，杆在夹爪里可能还会滚动/俯仰，所以投放对准用
        "实测朝向"而不是"按抓取姿态推算"。真机上这一项由腕部相机给出。
    重要性等级：A"""
    _, quat = io.perception.object_pose()
    axis = Rotation.from_quat(
        np.asarray(quat, dtype=float), scalar_first=True
    ).as_matrix()[:, 0]
    return axis / max(float(np.linalg.norm(axis)), 1e-9)


def rod_inside_bucket(
    rod_pos: np.ndarray,
    axis: np.ndarray,
    bucket_xy: np.ndarray,
) -> tuple[bool, float, float]:
    """整根杆是否落在桶口投影内、并夹在桶底与桶沿之间。

    参数：
        rod_pos: 杆 body 原点（粗端中心）的世界坐标，shape=(3,)。
        axis: 杆轴单位向量（从粗端指向细端），shape=(3,)。
        bucket_xy: 桶心世界 XY，shape=(2,)。
    返回：
        (是否在桶内, 端点离桶口内壁的最近余量 m, 杆最高点 z)。
    说明：
        杆是一条线段，到某点的最大距离必在端点取得，竖直方向同理，所以只查两个端点。
        这样"横躺"与"立着"用同一把尺子，不依赖任何姿态假设；早期用"质心偏移 ≤50 mm"
        在这两种姿态下等价于两条完全不同的要求。
    重要性等级：A"""
    a = np.asarray(axis, dtype=float)
    a = a / max(float(np.linalg.norm(a)), 1e-9)
    pos = np.asarray(rod_pos, dtype=float)
    ends = (pos + ROD_THIN_TIP * a, pos - ROD_THICK_TIP * a)
    center = np.asarray(bucket_xy, dtype=float)[:2]
    limit = float(BUCKET_MOUTH_RADIUS - ROD_RADIUS - BUCKET_EDGE_MARGIN)
    margin = min(
        limit - float(np.linalg.norm(end[:2] - center)) for end in ends
    )
    top = float(max(end[2] for end in ends)) + ROD_RADIUS
    bottom = float(min(end[2] for end in ends))
    inside = bool(
        margin > 0.0
        and bottom > BUCKET_FLOOR_Z - BUCKET_FLOOR_MARGIN
        and top < BUCKET_OPENING_Z
    )
    return inside, float(margin), top


def choose_grasp_spin(
    io: RobotIO,
    target_world: np.ndarray,
    candidates: tuple[float, ...] = GRASP_SPIN_CANDIDATES_DEG,
) -> float:
    """在候选工具自旋角中寻找 IK 残差和关节舒适度最优值。

参数：
    io: 任务层硬件端口，逆解走 ArmInterface.solve_ik。
    target_world: 世界坐标目标，shape=(3,)，单位 m。
    candidates: 候选绕工具 Z 轴旋转角，单位 deg。
返回：
    最优角度，单位 deg。
异常：
    最优位置残差超过 5 mm 时抛 RuntimeError。
评分：
    score=误差(mm)+10*舒适度+3*关节位移，舒适度用关节归一化角的均方根近似；
    关节位移项让同样可达时优先选离当前姿态最近的解。
重要性等级：A"""
    q_base = Rotation.from_quat(TARGET_QUAT, scalar_first=True)
    q_now = np.asarray(io.arm.joint_positions(), dtype=float)
    best_angle = 0.0
    best_error = float("inf")
    best_comfort = float("inf")
    best_score = float("inf")
    for angle_deg in candidates:
        q_candidate = (
            q_base * Rotation.from_euler(GRASP_SPIN_AXIS, np.deg2rad(angle_deg))
        ).as_quat(scalar_first=True)
        q_solution, error = io.arm.solve_ik(target_world, q_candidate)
        comfort = float(np.sqrt(np.mean((q_solution / np.pi) ** 2)))
        # 关节位移惩罚：同样可达时优先选离当前姿态最近的解，减少甩臂。
        travel = float(np.mean(np.abs(q_solution - q_now)))
        score = error * 1000.0 + 10.0 * comfort + 3.0 * travel
        if score < best_score:
            best_score = score
            best_error = error
            best_comfort = comfort
            best_angle = angle_deg
    print(
        f"  tool-spin selected: {best_angle:+.1f} deg, "
        f"IK residual={best_error * 1000:.2f} mm, "
        f"comfort={best_comfort:.3f}"
    )
    if best_error > 0.005:
        raise RuntimeError(
            f"no reachable equivalent grasp orientation, best error={best_error * 1000:.2f} mm"
        )
    return float(best_angle)


def start_arm_world_flexible(
    io: RobotIO,
    target_world: np.ndarray,
    duration: float,
    spin_deg: float,
) -> None:
    """用指定工具自旋角启动世界坐标机械臂运动。

参数：
    io: 任务层硬件端口。
    target_world: 世界目标位置，shape=(3,)，单位 m。
    duration: 运动时长，单位 s。
    spin_deg: 绕工具轴旋转角，单位 deg。
重要性等级：A"""
    quat = start_arm_world_flexible_quat(spin_deg)
    print(f"  arm target world={np.round(target_world, 3)}")
    io.arm.start_world(target_world, duration, quat)


def start_arm_world_quat(
    io: RobotIO,
    target_world: np.ndarray,
    duration: float,
    quat: np.ndarray,
) -> None:
    """用给定世界姿态启动机械臂运动（不做工具自旋展开）。

    参数：
        io: 任务层硬件端口。
        target_world: 世界目标位置，shape=(3,)，单位 m。
        duration: 运动时长，单位 s。
        quat: 工具姿态四元数 wxyz（固定基座坐标系）。
    重要性等级：A"""
    print(f"  arm target world={np.round(target_world, 3)}")
    io.arm.start_world(target_world, duration, np.asarray(quat, dtype=float))


def execute_grasp_concurrent(
    io: RobotIO,
    target_pos: np.ndarray,
    follower: PathFollower | None,
) -> tuple[bool, float]:
    """执行 CAR1 接近、抓取、提升和收回，同时让 CAR2 继续行驶。

参数：
    io: 任务层硬件端口。
    target_pos: 视觉估计的物体世界位置，shape=(3,)，单位 m。
    follower: 可空的 CAR2 路径控制器。
返回：
    (是否抓取成功, 本次使用的工具自旋角)；物体高度比抓取目标高出 0.04 m 视为成功。
异常：
    IK 不可达或姿态搜索失败时由底层抛 RuntimeError。
重要性等级：A"""
    print("\n[GRASP] CAR1 arm starts while CAR2 continues driving")
    # 抓取前显式张开夹爪。不能依赖模块初始化时的默认值：ROS 侧的夹爪指令缓存初值曾是 0，
    # MuJoCo 把 0 钳到行程下限（全闭），下降时两指只隔 1 mm，指垫坐在杆顶上，pinch 卡在
    # 19.8 mm 下不来——夹爪照样"夹住"（力 38~40 N），杆却一动不动。这一行两套后端都无害。
    io.gripper.start(FINGER_OPEN, 0.6)
    run_finger_motion_concurrent(io, follower)
    # 夹爪几何（pinch 站点为原点）：RG2 两指垫各半高 14.75 mm、中心与 pinch 同高，
    # 因此 pinch 必须停在离地 14.75~17.75 mm 这条带里，才能让夹持面盖住一根平躺的
    # Ø6 圆棒（棒心离地 3 mm）又不让指垫底面压进地面（见 PINCH_ABOVE_FLOOR）。
    approach = np.array([target_pos[0], target_pos[1], APPROACH_WORLD_Z])
    grasp = np.array([target_pos[0], target_pos[1], GRASP_WORLD_Z])
    lift = np.array([target_pos[0], target_pos[1], LIFT_WORLD_Z])

    spin_deg = choose_grasp_spin(io, approach)
    start_arm_world_flexible(io, approach, APPROACH_TIME, spin_deg)
    run_arm_motion_concurrent(io, follower)
    start_arm_world_flexible(io, grasp, DESCEND_TIME, spin_deg)
    run_arm_motion_concurrent(io, follower)
    # 轨迹结束不等于到位：末段插值很慢，再等 0.35 s 让伺服稳下来再测量，否则会把
    # "还没停稳"误判成静差，一补就补过头。
    run_wait_concurrent(io, 0.35, follower)

    # 伺服静差闭环：位置伺服带重力负载会停在命令值上方几毫米（单进程约 0.4 mm，
    # ROS 侧约 5 mm），而指垫的夹持窗口只有 3 mm（pinch 离地 14.75~17.75 mm）。
    # 差 4 mm 就变成"指垫全部悬在杆上方"，夹爪空合（力 12~15 N、杆不动）。
    # 按实测偏差修正目标再走一小段，最多两轮。
    # 这里**不设**高度下限：偏差是"命令 → 实际"的常数偏移，只能靠把命令往下压来抵消；
    # 真有地面挡着时，指垫压到地面就自然停住，比用一个固定下限更可靠。
    for attempt in range(2):
        err = np.asarray(io.arm.tool_position(), dtype=float) - grasp
        if abs(err[2]) < 0.0015 and float(np.linalg.norm(err[:2])) < 0.0015:
            break
        print(f"  grasp settle #{attempt + 1}: err={np.round(err * 1000, 1)} mm")
        grasp = grasp - err
        start_arm_world_flexible(io, grasp, GRASP_SETTLE_TIME, spin_deg)
        run_arm_motion_concurrent(io, follower)
        run_wait_concurrent(io, 0.25, follower)

    # 命令值与实际夹持点的对照。抓取空合时这一行能立刻分开两种情况：
    # 机械臂没到位（实际值偏离命令值）还是夹爪没夹住（值对得上、杆却没抬起来）。
    print(
        f"  pinch at grasp: {np.round(io.arm.tool_position(), 4)} "
        f"(cmd {np.round(grasp, 4)})"
    )
    print(f"  arm q at grasp: {np.round(io.arm.joint_positions(), 3)}")
    io.gripper.start(FINGER_CLOSE, CLOSE_TIME)
    run_finger_motion_concurrent(io, follower)
    print(f"  finger force after close: {io.gripper.force():.2f} N")

    start_arm_world_flexible(io, lift, LIFT_TIME, spin_deg)
    run_arm_motion_concurrent(io, follower)
    io.arm.start_fixed(CARRY_FIXED, CARRY_TIME)
    run_arm_motion_concurrent(io, follower)

    obj, _ = io.perception.object_pose()
    print(f"  rod after grasp: {np.round(obj, 4)}")
    return bool(obj[2] > target_pos[2] + 0.04), spin_deg


def execute_cooperative_place(
    io: RobotIO,
    bucket_xy: np.ndarray,
    spin_deg: float,
) -> None:
    """夹着粗端把整根杆下探进屏蔽桶，松爪后靠重力落进桶底。

    参数：
        io: 任务层硬件端口；等待中的二号车也要保持控制刷新。
        bucket_xy: CAR2 屏蔽桶中心世界 XY，shape=(2,)，单位 m。
        spin_deg: 与抓取时相同的工具自旋角——搬运途中不调整姿态。
    副作用：
        推进机械臂/夹爪、打印入桶站点与物体位姿。
    说明：
        桶内径 0.40 m、内深 0.24 m（外高 0.26 m）。杆长 0.202 m，只要**质心**大致对着
        桶心（夹持点贴着粗端，离质心 79 mm），整根杆就落在桶口投影内、不会横搭在桶沿上；
        对准后在桶口正上方 6 cm 松爪，杆靠重力落进桶底。Ø400 的可容忍桶心偏差约 98 mm，
        是 Ø240（约 18 mm）的五倍多，这样真机的定位误差才吃得下。
        对准用实测杆轴：夹爪只能传递绕竖直轴的转动，杆在夹爪里可能已经相对工具转过
        一个角度。（"夹着杆压到桶口以下再松"和"加大夹持力硬压"都试过：前者杆会被桶沿
        顶住、从夹爪里向上滑脱；后者杆变成刚性探针、机械臂直接压不动。）
    重要性等级：A"""
    bucket = np.asarray(bucket_xy, dtype=float)
    quat = start_arm_world_flexible_quat(spin_deg)
    stage = np.array([bucket[0], bucket[1], BUCKET_OPENING_Z + 0.30])
    start_arm_world_quat(io, stage, PLACE_APPROACH_TIME, quat)
    run_arm_motion_concurrent(io, None)

    axis = held_rod_axis(io)
    # 只把夹持点对到桶心不够：搬运途中杆可能在夹爪里沿轴向滑过一段（实测偏移可达
    # 45 mm）。Ø400 桶口的余量约 98 mm，已经吃得下这个量，但仍按实测再对一次，留给真机。
    # 这里直接量"夹持点 → 质心"的实际偏移，再把它反推到桶心正上方。
    rod_pos, _ = io.perception.object_pose()
    rod_com = rod_pos + ROD_COM_OFFSET_M * axis
    offset = rod_com - io.arm.tool_position()
    insert = np.array([bucket[0], bucket[1], PLACE_INSERT_Z]) - np.array(
        [offset[0], offset[1], 0.0]
    )
    print(
        f"  place: spin={spin_deg:+.0f} deg, rod axis={np.round(axis, 3)}, "
        f"com_offset={np.round(offset * 1000, 1)} mm, insert={np.round(insert, 3)}"
    )

    start_arm_world_quat(io, insert, PLACE_DESCEND_TIME, quat)
    run_arm_motion_concurrent(io, None)

    # 松爪前按实测质心再修一次：下探途中杆还可能沿轴向再滑一点（Ø400 余量宽，这一步
    # 在仿真里多数是十几毫米以内，留着是为了真机上不靠运气）。
    for _ in range(2):
        com_now = io.perception.object_pose()[0] + (
            ROD_COM_OFFSET_M * held_rod_axis(io)
        )
        err = com_now[:2] - np.array([bucket[0], bucket[1]])
        print(f"  drop centring residual = {np.round(err * 1000, 1)} mm")
        if float(np.linalg.norm(err)) < 0.008:
            break
        insert = insert - np.array([err[0], err[1], 0.0])
        start_arm_world_quat(io, insert, 0.8, quat)
        run_arm_motion_concurrent(io, None)

    pos_at_release, quat_at_release = io.perception.object_pose()
    print(f"  bucket insert site: {np.round(io.arm.tool_position(), 4)}")
    print(
        "  object at release: "
        f"{np.round(np.concatenate([pos_at_release, quat_at_release]), 4)}"
    )

    io.gripper.start(FINGER_OPEN, 1.0)
    run_finger_motion_concurrent(io, None)
    run_wait_concurrent(io, PLACE_SETTLE_TIME, None)

    # 原姿态竖直抬起再收臂：直接转回 CARRY 会横扫过桶口，把刚放下的杆打飞。
    start_arm_world_quat(
        io, insert + np.array([0.0, 0.0, 0.32]), PLACE_APPROACH_TIME, quat
    )
    run_arm_motion_concurrent(io, None)
    io.arm.start_fixed(CARRY_FIXED, 2.0)
    run_arm_motion_concurrent(io, None)
    run_wait_concurrent(io, 1.5, None)


# ---------------------------------------------------------------------------
# 七阶段任务状态机
# ---------------------------------------------------------------------------
class DualCarMission:
    """保存双车任务共享对象并执行完整协同状态机。

数据流：
    CAR1 雷达导航 -> 停车采图 -> 通知 CAR2 并行出发 -> 抓取 -> CAR2 动态停靠 -> 协同投放 -> 倒车脱离 -> 正向精准返航。
重要性等级：A"""

    def __init__(
        self,
        io: RobotIO,
        target_truth: np.ndarray,
        snapshot: SnapshotHook | None = None,
    ) -> None:
        """保存任务所需的硬件端口与目标真值。

参数：
    io: 任务层唯一允许使用的硬件端口集合（时钟/底盘/机械臂/夹爪/感知）。
        `io.chassis` 必须恰好两辆车，下标 0 = CAR1。
    target_truth: 仅用于视觉结果误差验证的物体真值，shape=(3,)，单位 m。
    snapshot: 可选截图回调；None 表示不截图（真机/ROS 头端通常没有离屏渲染）。
重要性等级：A"""
        self.io = io
        self.car1, self.car2 = io.chassis
        self.target_truth = target_truth
        self._snapshot = snapshot
        # 回归矩阵使用的结构化指标；run() 逐阶段填充，失败时保留已完成部分。
        self.metrics: dict[str, object] = {"stage": "init"}

    def _capture(self, name: str, lookat: list[float], distance: float,
                 azimuth: float, elevation: float) -> None:
        """按需调用后端截图回调（未配置时静默跳过）。"""
        if self._snapshot is not None:
            self._snapshot(name, lookat, distance, azimuth, elevation)

    def inter_car_contacts(self) -> int:
        """统计两车根刚体之间的当前物理接触数。

返回：
    接触对数量；0 表示当前没有车车接触。
用途：
    最终验收中检测是否发生物理穿模或碰撞。
重要性等级：A"""
        return self.io.chassis[0].contacts_with(self.io.chassis[1])

    def run(self) -> bool:
        """执行双车协同回收完整状态机。

返回：
    True 表示物体最终在桶内、两车回到目标初始位置且车车接触数为 0。
副作用：
    推进长时间仿真、打印阶段日志，并按配置保存关键截图。
异常：
    路径超时、视觉丢失、IK 不可达或物理状态发散时由子调用抛异常。
重要性等级：A"""
        self.metrics["stage"] = "1_start"
        print("\n[阶段 1] 两车位于起点红线外侧，任务开始。")
        print(
            f"           CAR1={np.round(self.car1.pose()[0][:2], 2)}, "
            f"CAR2={np.round(self.car2.pose()[0][:2], 2)}, "
            f"red_line_Y={RETURN_LINE_Y:.2f}"
        )

        self.metrics["stage"] = "2_car1_navigate"
        print("\n[阶段 2] 一号车前往目标区，正在雷达避障...")
        print("[模块解耦] 移动控制阶段：仅使用雷达和底盘控制，不调用相机/Renderer。")
        drive_path(
            self.io,
            self.car1,
            CAR1_SEARCH_PATH,
            final_tolerance=0.12,
            max_speed=CRUISE_SPEED,
        )
        observation_pos, observation_yaw = self.car1.pose()
        # 抵达观测位的总览截图：此时雷达点云、两车与目标都在画面里，
        # 与 main() 的开场总览用同一组相机参数，便于 README 前后对照。
        self._capture("dual_car1_avoid.png", [2.0, 11.0, 0.20], 38.0, 135.0, -52.0)

        self.metrics["stage"] = "3_vision_dock"
        print(
            f"\n[阶段 3] 一号车抵达停靠点，坐标 "
            f"({observation_pos[0]:.3f}, {observation_pos[1]:.3f})，通知二号车出发。"
        )

        provisional_goal = np.array(
            [0.84, CAR2_DOCK_CENTER_Y - CAR2_STAGING_DY]
        )
        car2_follower = PathFollower(
            self.car2,
            [(float(provisional_goal[0]), float(provisional_goal[1]))],
            self.io.clock,
            waypoint_timeout=leg_timeout(19.0, CRUISE_SPEED),
            final_tolerance=0.05,
            max_speed=CRUISE_SPEED,
        )
        print("[并发调度] 二号车已收到出发指令，开始连续前往目标区。")
        run_wait_concurrent(self.io, 0.15, car2_follower)

        print("[模块解耦] 一号车已经停车，先采一帧做粗定位。")
        print("\n[视觉识别] 观测位约 2.1 m：只做粗定位（棒在图上约 60 像素长）。")
        # 刻意同步调用：若在后台线程里处理、主循环按 future.done() 边步进边等，
        # 推进帧数会取决于线程墙钟，同一 seed 两次可能走出不同轨迹（回归不可复现）。
        try:
            coarse = self.io.perception.estimate_target()
        except (RuntimeError, TimeoutError) as exc:
            # 观测位看不清时退回"目标区中心"这一任务级先验：目标点一定在该区内，
            # 先验只用来决定把车开到哪里补拍，最终抓取点仍由近距测量给出。
            coarse = np.array(
                [
                    0.5 * (TARGET_ZONE_X_MIN + TARGET_ZONE_X_MAX),
                    0.5 * (TARGET_ZONE_Y_MIN + TARGET_ZONE_Y_MAX),
                    OBJECT_Z,
                ],
                dtype=float,
            )
            print(f"[视觉识别] 观测位粗定位失败（{exc}），改用目标区中心先验")
        print(
            f"[视觉识别] 粗定位 ({coarse[0]:.3f}, {coarse[1]:.3f})"
        )
        self.metrics.update(coarse_target=coarse.tolist())

        predicted_car1_pos = np.array(
            [coarse[0], coarse[1] - GRASP_STANDOFF_Y], dtype=float
        )
        dock_dy = CAR2_DOCK_CENTER_Y - predicted_car1_pos[1]
        dock_offset = max(
            DOCK_OFFSET_MIN,
            float(np.sqrt(max(CAR2_MIN_SEPARATION**2 - dock_dy**2, 0.0)))
            + DOCK_POSE_MARGIN,
        )
        car2_goal = np.array(
            [predicted_car1_pos[0] + dock_offset, CAR2_DOCK_CENTER_Y]
        )
        # 保留原来的集结航点（已验证的安全走廊），只在它之后追加：
        # 先在同一纬度横移到停靠点正南，再直线北上，避免贴着另一辆车挪车。
        car2_follower.append_path(
            [
                (
                    float(car2_goal[0]),
                    float(CAR2_DOCK_CENTER_Y - CAR2_STAGING_DY),
                ),
                (float(car2_goal[0]), float(car2_goal[1])),
            ]
        )
        self.metrics.update(
            car1_predicted_y=float(predicted_car1_pos[1]),
            car2_goal=car2_goal.tolist(),
            dock_offset=float(dock_offset),
        )

        # 近距测量位：停车距离 1.41 m，腕部相机离目标约 1.0 m、俯角约 45°。
        # 不能直接开去 0.85 m 的抓取位再拍——D435i 相对工具固定下俯 28.5°，视场
        # 下沿只到 49.8°，而 0.85 m 停车位下相机到目标的俯角是 66°，实测投影
        # v=715（图像高 480），目标落在画面外。1.41 m 处约 88 像素长、2.6 像素宽。
        measure_xy = np.array(
            [coarse[0], coarse[1] - MEASURE_STANDOFF_Y], dtype=float
        )
        car1_measure = PathFollower(
            self.car1,
            [(float(measure_xy[0]), float(measure_xy[1]))],
            self.io.clock,
            waypoint_timeout=SHORT_MANEUVER_TIMEOUT,
            final_tolerance=0.10,
            max_speed=APPROACH_SPEED,
        )
        run_followers_until_primary_done(self.io, car1_measure, car2_follower)

        # 测量位拍照前先把车头摆正：臂座前移到平板中心后，腕部相机跟着前移，
        # 相机视野对车头朝向更敏感——长距离绕行 + 整车增重后残余偏航能到 60° 以上，
        # 腕部相机变成"斜着看"，实测视场里只剩空地，杆完全不在画面里（深度图上什么都没有）。
        # 在观测位（y≈18.5）摆正的代价是滑移几厘米，但测量位的杆世界坐标由视觉给出、
        # 不依赖车位置精度，所以这里值得转。整车 141 kg 后差速原地转的效率比 108 kg 时更低
        # （同样轮速下横摆只有原来的六成），单次转常常转不到位，所以最多迭代三次。
        for attempt in range(3):
            _, car1_yaw_here = self.car1.pose()
            measure_align_error = abs(wrap_angle(CAR_START_YAW - car1_yaw_here))
            if measure_align_error <= np.deg2rad(5.0):
                break
            print(
                f"  [CAR1] 测量位前残余偏航 {np.rad2deg(measure_align_error):.1f}°"
                f"（第 {attempt + 1} 次摆正）"
            )
            aligner = SkidTurnFollower(
                self.car1,
                CAR_START_YAW,
                self.io.clock,
                timeout=8.0,
                ramp_time=SKID_TURN_SPINDOWN_TIME,
            )
            run_turn_with_secondary(self.io, aligner, car2_follower)

        # 无论转没转，都等车停稳再拍帧（差速原地转收尾时车体仍有残余速度）。
        run_wait_concurrent(self.io, 0.2, car2_follower)

        print("\n[视觉识别] 近距复测：由粗定位把车开到测量位，再采一帧精定位。")
        target = None
        for attempt in range(2):
            try:
                target = self.io.perception.estimate_target()
                break
            except (RuntimeError, TimeoutError) as exc:
                if attempt == 1:
                    # 测量位两帧都没拍到杆：退回观测位的粗定位结果。实车布置下
                    # 141.6 kg 的车差速原地转不可靠，车头可能仍偏十几度，腕部相机
                    # 就斜着看、杆落到画面外。粗定位在 2.1 m 处的实测误差 1.1~1.7 mm，
                    # 抓取的 xy 容差是厘米级，够用——比让整条任务失败合理。
                    print(
                        f"[视觉识别] 测量位重试仍失败（{exc}），退回观测位粗定位 "
                        f"({coarse[0]:.3f}, {coarse[1]:.3f})"
                    )
                    target = coarse.copy()
                    break
                # 粗定位偏 0.3 m 以上时，测量位会停得离目标过近，杆落到相机下视场
                # 之外（画面下缘只剩细端、又落在搜索区外）。后退 0.35 m 再采一帧。
                print(f"[视觉识别] 测量位识别失败（{exc}），后退 0.35 m 重试")
                retry_xy = measure_xy - np.array([0.0, 0.35])
                retry = PathFollower(
                    self.car1,
                    [(float(retry_xy[0]), float(retry_xy[1]))],
                    self.io.clock,
                    waypoint_timeout=SHORT_MANEUVER_TIMEOUT,
                    final_tolerance=0.10,
                    max_speed=REVERSE_SPEED,
                )
                run_followers_until_primary_done(
                    self.io, retry, car2_follower
                )
        vision_error = float(np.linalg.norm(target - self.target_truth))
        self.metrics.update(
            vision_error_mm=vision_error * 1000.0,
            vision_target=target.tolist(),
            truth_target=self.target_truth.tolist(),
        )
        print("[模块解耦] 视觉完成；界面与感知已通过 PerceptionInterface 隔离。")

        approach_xy = np.array(
            [target[0], target[1] - GRASP_STANDOFF_Y], dtype=float
        )
        # 用实测抓取位重算二号车停靠点：上面那次是按**粗定位**算的，粗定位偏 0.2 m 时
        # 最终两车中心距只剩 0.80 m（低于 0.82 m 安全阈值），二号车会被自己的雷达挡在
        # 停靠点外、到点超时（实测 seed 0）。这里只换最后一个航点，集结航点不动。
        dock_dy = CAR2_DOCK_CENTER_Y - approach_xy[1]
        dock_offset = max(
            DOCK_OFFSET_MIN,
            float(np.sqrt(max(CAR2_MIN_SEPARATION**2 - dock_dy**2, 0.0)))
            + DOCK_POSE_MARGIN,
        )
        car2_goal = np.array(
            [approach_xy[0] + dock_offset, CAR2_DOCK_CENTER_Y]
        )
        car2_follower.replace_final_waypoint(
            (float(car2_goal[0]), float(car2_goal[1]))
        )
        self.metrics.update(
            car1_predicted_y=float(approach_xy[1]),
            car2_goal=car2_goal.tolist(),
            dock_offset=float(dock_offset),
        )
        car1_align = PathFollower(
            self.car1,
            [(float(approach_xy[0]), float(approach_xy[1]))],
            self.io.clock,
            waypoint_timeout=SHORT_MANEUVER_TIMEOUT,
            final_tolerance=0.12,
            max_speed=APPROACH_SPEED,
        )
        # 最后这段 0.56 m 直线入位期间关掉"绕行同伴车"：两车的最小间距由停靠规划
        # 保证（中心距 ≥ 0.82 m），而雷达一旦恰好在这时触发绕行，车会被带偏到抓取位
        # 之外——实测 seed 0 绕行后停在 1.3 m 外、残余偏航 160°，机械臂直接够不到。
        # 只关一号车这一段；二号车的雷达与之前各段都不受影响。
        peer_avoidance = self.car1.lidar.peer_avoidance
        self.car1.lidar.peer_avoidance = False
        try:
            run_followers_until_primary_done(
                self.io,
                car1_align,
                car2_follower,
            )
        finally:
            self.car1.lidar.peer_avoidance = peer_avoidance
        # 抓取前把车头摆正：世界坐标解 IK 时基座航向直接决定抓取方向，残差几度可以，
        # 差太多会让夹爪斜着咬圆柱。但原地转是"轮胎打滑换旋转"——实测只为补 3° 就滑
        # 移 14 mm 且起停瞬间一次抖动，而这正好是"停在放射物前"该完全静止的时刻。
        # 因此只在残差超过 12° 时才转，正常 6~10° 的残差直接留着不动车。
        _, car1_yaw_now = self.car1.pose()
        car1_align_error = abs(wrap_angle(CAR_START_YAW - car1_yaw_now))
        print(f"  [CAR1] 抓取前残余偏航 {np.rad2deg(car1_align_error):.1f}°")
        if car1_align_error > np.deg2rad(12.0):
            turner = SkidTurnFollower(
                self.car1,
                CAR_START_YAW,
                self.io.clock,
                timeout=8.0,
                ramp_time=SKID_TURN_SPINDOWN_TIME,
            )
            run_turn_with_secondary(self.io, turner, car2_follower)
        else:
            run_wait_concurrent(self.io, 0.15, car2_follower)
        run_wait_concurrent(self.io, 0.5, car2_follower)

        grasp_start = self.io.clock.now
        self.metrics["stage"] = "4_grasp"
        grasp_success, grasp_spin = execute_grasp_concurrent(
            self.io,
            target,
            car2_follower,
        )
        if not grasp_success:
            raise RuntimeError("car1 failed to grasp the target rod")
        self.metrics.update(grasp_ok=True, grasp_sim_start=float(grasp_start))
        print(
            f"\n[阶段 4] 一号车抓取成功，夹爪闭合。"
            f" 仿真时间 {self.io.clock.now:.2f} s"
        )

        finish_follower(self.io, car2_follower)
        # 停靠后不再原地转摆正：纯追踪只在行进中转向，到点可能留几度残余偏航，
        # 但差速原地转在地面上是"轮胎打滑"——实测只为补正 3° 就把车整体滑移了 2 cm，
        # 而且正好发生在机械臂准备投放、车身必须绝对静止的时刻（用户看到的"动一下"）。
        # 车身投影在最坏朝向下也只有 0.578 m，配停靠点 Y≈20.05 仍在绿框 20.70 外侧，
        # 所以这点残余偏航直接留着，不做原地转。
        _, car2_yaw_now = self.car2.pose()
        align_error = abs(wrap_angle(CAR_START_YAW - car2_yaw_now))
        print(
            f"  [CAR2] 停靠残余偏航 {np.rad2deg(align_error):.1f}°（保持，不原地转）"
        )
        stop_and_settle(self.io, self.car2)
        self.metrics["stage"] = "5_dock_check"
        car2_pos, car2_yaw = self.car2.pose()
        car1_pose, _ = self.car1.pose()
        car2_y_extent = (
            abs(np.sin(car2_yaw)) * 0.456
            + abs(np.cos(car2_yaw)) * 0.355
        )
        car2_top_edge = float(car2_pos[1] + car2_y_extent)
        car2_outside_frame = car2_top_edge < TARGET_ZONE_Y_MIN
        separation = float(np.linalg.norm(car2_pos[:2] - car1_pose[:2]))
        self.metrics.update(
            car2_dock=[float(car2_pos[0]), float(car2_pos[1])],
            car2_top_edge=car2_top_edge,
            car2_outside_frame=bool(car2_outside_frame),
            separation=separation,
            min_separation=float(self.car2.min_other_separation),
        )
        print(
            f"\n[阶段 5] 二号车已抵达动态停靠点，坐标 "
            f"({car2_pos[0]:.3f}, {car2_pos[1]:.3f})。"
        )
        print(
            f"           两车中心距={separation:.3f} m, "
            f"最小过程距离={self.car2.min_other_separation:.3f} m"
        )
        print(
            f"           CAR2 最上沿 Y={car2_top_edge:.3f} m, "
            f"绿框下边界 Y={TARGET_ZONE_Y_MIN:.3f} m, "
            f"完全在框外={car2_outside_frame}"
        )
        print("           停靠点按最小安全侧向偏移计算，投放姿态由实际 IK 自动选择。")
        if not car2_outside_frame:
            raise RuntimeError("car2 body intersects the radiation frame")
        if separation < CAR2_MIN_SEPARATION:
            raise RuntimeError("unsafe inter-car separation")

        self._capture(
            "dual_parallel_grasp.png",
            [float((car1_pose[0] + car2_pos[0]) / 2.0),
             float((car1_pose[1] + car2_pos[1]) / 2.0), 0.40],
            5.6,
            135.0,
            -38.0,
        )

        execute_cooperative_place(self.io, car2_pos[:2], grasp_spin)
        self._capture(
            "dual_bucket_release.png",
            [float(car2_pos[0]), float(car2_pos[1]), 0.52],
            1.5,
            150.0,
            -62.0,
        )
        obj_after_drop, _ = self.io.perception.object_pose()
        rod_axis = held_rod_axis(self.io)
        # 质心偏移只是记录量（横向精度参考），判定用"整根杆在不在桶里"
        rod_com = obj_after_drop + ROD_COM_OFFSET_M * rod_axis
        drop_xy_error = float(np.linalg.norm(rod_com[:2] - car2_pos[:2]))
        drop_tilt_deg = float(
            np.rad2deg(np.arccos(min(1.0, abs(float(rod_axis[2])))))
        )
        dropped_inside, mouth_margin, rod_top_z = rod_inside_bucket(
            obj_after_drop, rod_axis, car2_pos[:2]
        )
        print(
            f"           投放后杆质心=({rod_com[0]:.3f}, {rod_com[1]:.3f}, "
            f"{rod_com[2]:.3f})，与桶心水平偏差={drop_xy_error:.3f} m，"
            f"杆与竖直方向夹角={drop_tilt_deg:.1f}°，"
            f"口内余量={mouth_margin * 1000:.0f} mm，杆顶={rod_top_z:.3f} m"
        )
        if not dropped_inside:
            raise RuntimeError(
                f"cooperative drop failed: mouth margin={mouth_margin * 1000:.1f} mm, "
                f"rod top={rod_top_z:.3f} m (opening {BUCKET_OPENING_Z:.3f} m), "
                f"xy error={drop_xy_error:.4f} m, tilt={drop_tilt_deg:.1f} deg"
            )
        self.metrics.update(
            drop_xy_error=drop_xy_error,
            drop_tilt_deg=drop_tilt_deg,
            drop_inside=True,
            obj_after_drop=obj_after_drop.tolist(),
            rod_com_after_drop=rod_com.tolist(),
        )

        self.metrics["stage"] = "6_return"
        print("\n[阶段 6] 投放完成，两车开始倒车/返航。")
        reverse_exit = [
            ReverseStraightFollower(
                self.car1, RADIATION_EXIT_Y, self.io.clock, max_speed=REVERSE_SPEED
            ),
            ReverseStraightFollower(
                self.car2, RADIATION_EXIT_Y, self.io.clock, max_speed=REVERSE_SPEED
            ),
        ]
        reverse_min_gap = run_two_straight_reverses(self.io, reverse_exit)
        self.metrics["reverse_min_gap"] = reverse_min_gap

        car1_return_path = [
            (-2.80, 19.50),
            (-4.80, 18.00),
            (-4.80, 13.00),
            (-4.80, 8.00),
            (-3.50, 3.00),
            (-1.50, 0.00),
            (float(CAR1_START[0]), float(CAR1_START[1])),
        ]
        car2_return_path = [
            (5.00, 20.00),
            (6.20, 17.00),
            (6.20, 13.00),
            (5.00, 8.00),
            (4.50, 3.00),
            (3.00, 0.00),
            (float(CAR2_START[0]), float(CAR2_START[1])),
        ]
        car1_return = PathFollower(
            self.car1,
            car1_return_path,
            self.io.clock,
            waypoint_timeout=leg_timeout(16.0, CRUISE_SPEED),
            final_tolerance=0.10,
            max_speed=CRUISE_SPEED,
        )
        car2_return = PathFollower(
            self.car2,
            car2_return_path,
            self.io.clock,
            waypoint_timeout=leg_timeout(16.0, CRUISE_SPEED),
            final_tolerance=0.10,
            max_speed=CRUISE_SPEED,
        )
        run_two_followers(
            self.io,
            [car1_return, car2_return],
        )

        self.metrics["stage"] = "7_acceptance"
        car1_final, _ = self.car1.pose()
        car2_final, _ = self.car2.pose()
        obj_final, _ = self.io.perception.object_pose()
        final_axis = held_rod_axis(self.io)
        final_com = obj_final + ROD_COM_OFFSET_M * final_axis
        final_xy_error = float(
            np.linalg.norm(final_com[:2] - car2_final[:2])
        )
        final_inside, final_margin, _ = rod_inside_bucket(
            obj_final, final_axis, car2_final[:2]
        )
        car1_home_error = float(np.linalg.norm(car1_final[:2] - CAR1_START))
        car2_home_error = float(np.linalg.norm(car2_final[:2] - CAR2_START))
        both_home = bool(car1_home_error < 0.20 and car2_home_error < 0.20)
        both_returned = bool(
            car1_final[1] < RETURN_LINE_Y - 0.20
            and car2_final[1] < RETURN_LINE_Y - 0.20
            and both_home
        )
        contacts = self.inter_car_contacts()
        self.metrics.update(
            final_xy_error=final_xy_error,
            final_inside=bool(final_inside),
            contacts=int(contacts),
            car1_final=[float(car1_final[0]), float(car1_final[1])],
            car2_final=[float(car2_final[0]), float(car2_final[1])],
            car1_home_error=car1_home_error,
            car2_home_error=car2_home_error,
            both_home=bool(both_home),
            both_returned=bool(both_returned),
            sim_time_s=float(self.io.clock.now),
        )

        if not both_returned:
            raise RuntimeError("one or both cars did not return behind the red line")
        if contacts:
            raise RuntimeError("physical contact detected between CAR1 and CAR2")
        print(
            f"\n[阶段 7] 两车均已安全退回红线外侧，任务结束。\n"
            f"           CAR1=({car1_final[0]:.3f}, {car1_final[1]:.3f}), "
            f"CAR2=({car2_final[0]:.3f}, {car2_final[1]:.3f})\n"
            f"           视觉误差={vision_error * 1000:.2f} mm, "
            f"桶内余量={final_margin * 1000:.0f} mm（质心偏移 {final_xy_error:.4f} m）, "
            f"车车接触={contacts}\n"
            f"           回起点误差: CAR1={car1_home_error:.3f} m, "
            f"CAR2={car2_home_error:.3f} m"
        )

        self._capture("dual_final_returned.png", [0.25, -3.5, 0.35], 9.0, 135.0, -38.0)
        self.metrics["stage"] = "done"
        return bool(final_inside and both_returned and contacts == 0)
