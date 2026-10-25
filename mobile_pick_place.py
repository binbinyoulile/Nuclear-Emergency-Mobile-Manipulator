"""UR10e 单臂移动操作臂规则控制仿真。

文件目的：
    在 MuJoCo 中演示四轮底盘移动到目标、UR10e 抓取平躺圆柱并将物体放入屏蔽桶。
整体输入输出：
    输入为 MJCF 场景、任务常量与可选 headless 参数；输出为仿真状态、终端日志和布尔成功结果。
依赖与版本：
    Python 3.10+、mujoco、mink、numpy、scipy；quadprog 可选，缺失时自动回退 daqp。
运行方式：
    D:/Python313/python.exe mobile_pick_place.py
    D:/Python313/python.exe mobile_pick_place.py --headless
关键假设：
    模型名、关节名和夹爪执行器名与仓库内 XML 一致；场景资产已由 scripts/fetch_models.py 下载。
已知限制：
    使用简化环境与规则控制，不包含真实通信延迟、传感器噪声或在线碰撞规划。
重要性等级：A"""

from __future__ import annotations

import argparse
import os
import time

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
IK_SCENE = os.path.join(HERE, "arm_ik_scene.xml")
MOBILE_SCENE = os.environ.get(
    "MOBILE_SCENE_PATH", os.path.join(HERE, "dual_car_scene.xml")
)
if not os.path.isabs(MOBILE_SCENE):
    MOBILE_SCENE = os.path.join(HERE, MOBILE_SCENE)
MOBILE_SCENE = os.path.abspath(MOBILE_SCENE)


def load_mjcf(scene_path: str) -> mujoco.MjModel:
    """从文件加载 MJCF 模型。

功能：先切换到场景目录，再用 ASCII 相对文件名调用 MuJoCo，规避 Windows 中文绝对路径兼容问题。
参数：
    scene_path: MJCF 场景路径，可为相对或绝对路径。
返回：
    已解析的 mujoco.MjModel。
异常：
    文件不存在时抛 FileNotFoundError；XML 或模型资产错误时由 MuJoCo 抛出。
副作用：
    调用期间临时改变进程工作目录，finally 中必定恢复。
重要性等级：A"""
    scene_path = os.path.abspath(scene_path)
    if not os.path.isfile(scene_path):
        raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")
    previous_cwd = os.getcwd()
    try:
        os.chdir(os.path.dirname(scene_path))
        return mujoco.MjModel.from_xml_path(os.path.basename(scene_path))
    finally:
        os.chdir(previous_cwd)

# ---------------------------------------------------------------------------
# 任务几何与控制参数
# ---------------------------------------------------------------------------
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
TARGET_QUAT = np.array([0.0, 1.0, 0.0, 0.0])  # 工具坐标系 +Y 轴朝下

GROUND_Z = 0.0
OBJECT_RADIUS = 0.01
OBJ_POS = np.array([6.00, 0.00, GROUND_Z + OBJECT_RADIUS])
BUCKET_POS = np.array([6.00, 4.00, 0.00])
OBJECT_STOP = np.array([5.00, 0.00])       # 位于圆柱后方 1.00 m
BUCKET_STOP = np.array([5.00, 4.00])       # 位于屏蔽桶后方 1.00 m
BUCKET_DROP_WORLD = np.array([6.00, 4.00, 0.28])

APPROACH_WORLD_Z = OBJ_POS[2] + 0.17
GRASP_WORLD_Z = OBJ_POS[2] + 0.007
LIFT_WORLD_Z = OBJ_POS[2] + 0.30
PRE_DROP_WORLD_Z = 0.42
DROP_WORLD_Z = 0.28

CARRY_FIXED = np.array([0.20, 0.0, 0.45])   # 机械臂坐标系中的搬运姿态，位于底盘轮廓内
FINGER_OPEN = 0.0
FINGER_CLOSE = 255.0

# 时间参数，单位 s
APPROACH_TIME = 2.0
DESCEND_TIME = 1.5
CLOSE_TIME = 1.5
LIFT_TIME = 1.8
CARRY_TIME = 2.8
PRE_DROP_TIME = 2.0
DROP_TIME = 1.5
RELEASE_TIME = 1.0
RELEASE_SETTLE_TIME = 0.8
RETRACT_TIME = 2.0
FINAL_SETTLE_TIME = 2.0
CHASSIS_SETTLE_TIME = 1.2

# 底盘 P 控制器
WHEEL_RADIUS = 0.1525
WHEEL_TRACK = 0.59
K_POS = 1.20
V_MAX = 0.80
W_MAX = 1.00
POS_TOL = 0.03
STEER_WHEELBASE = 0.585
STEER_MAX = 0.70
COMMAND_RAMP = 0.12
YAW_TOL = 0.03
MAX_DRIVE_TIME = 40.0

# 仿真与渲染
STEPS_PER_FRAME = 20
IK_TOL = 0.003
LIFT_TOL = 0.05
CONTACT_FORCE_TOL = 0.5
BUCKET_XY_TOL = 0.05
BUCKET_Z_MAX = 0.15


def smoothstep(t: float) -> float:
    """计算三次平滑插值系数。

参数：
    t: 归一化时间，通常位于 [0, 1]；越界值会被截断。
返回：
    t*t*(3-2*t)，范围 [0, 1]，一阶导在两端为零。
用途：
    对机械臂和夹爪控制量做缓入缓出，避免阶跃指令造成抖动。
重要性等级：C"""
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


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


def pick_solver() -> str:
    """选择可用的 mink QP 求解器。

返回：
    优先返回 quadprog；该包不可用时返回 daqp。
异常：
    本函数只探测导入，不主动抛异常；实际求解器不可用时会在 IK 调用处报错。
重要性等级：C"""
    try:
        import quadprog  # noqa: F401
        return "quadprog"
    except ImportError:
        return "daqp"


# ---------------------------------------------------------------------------
# 模型与命名索引
# ---------------------------------------------------------------------------
IK_MODEL = load_mjcf(IK_SCENE)
MODEL = load_mjcf(MOBILE_SCENE)
DATA = mujoco.MjData(MODEL)
SOLVER = pick_solver()

IK_GRASP_SITE = mujoco.mj_name2id(IK_MODEL, mujoco.mjtObj.mjOBJ_SITE, "pinch")
IK_BASE_BODY = mujoco.mj_name2id(IK_MODEL, mujoco.mjtObj.mjOBJ_BODY, "base")

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
WHEEL_JOINT_NAMES = [
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
]
ARM_ACT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def joint_qadr(name: str) -> int:
    """查询命名关节在 qpos 数组中的起始下标。

参数：
    name: MJCF 中唯一的关节名称。
返回：
    整数 qpos 下标。
异常：
    名称不存在时底层 MuJoCo 查询会产生无效索引，调用方应保证模型一致。
重要性等级：B"""
    return int(MODEL.jnt_qposadr[mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, name)])


def joint_dadr(name: str) -> int:
    """查询命名关节在 qvel/qfrc 数组中的起始下标。

参数：
    name: MJCF 中唯一的关节名称。
返回：
    整数自由度下标。
异常：
    名称不存在时底层 MuJoCo 查询会产生无效索引。
重要性等级：B"""
    return int(MODEL.jnt_dofadr[mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, name)])


def actuator_id(name: str) -> int:
    """查询命名执行器下标。

参数：
    name: MJCF 中唯一的执行器名称。
返回：
    整数 actuator 下标，用于访问 ctrl 和 actuator_force。
异常：
    名称不存在时底层 MuJoCo 查询会产生无效索引。
重要性等级：B"""
    return int(mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_ACTUATOR, name))


CHASSIS_JOINT = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
CHASSIS_QADR = int(MODEL.jnt_qposadr[CHASSIS_JOINT])
CHASSIS_BODY = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_BODY, "mobile_chassis")
ARM_BASE_BODY = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_BODY, "base")

ARM_QADR = np.array([joint_qadr(n) for n in ARM_JOINT_NAMES], dtype=int)
ARM_DADR = np.array([joint_dadr(n) for n in ARM_JOINT_NAMES], dtype=int)
ARM_ACT = np.array([actuator_id(n) for n in ARM_ACT_NAMES], dtype=int)
FINGER_ACT = actuator_id("fingers_actuator")
WHEEL_ACT = {n: actuator_id(f"{n.replace('_joint', '')}_act") for n in WHEEL_JOINT_NAMES}
WHEEL_STEER_ACT = np.array(
    [actuator_id("wheel_fl_steer_act"), actuator_id("wheel_fr_steer_act")], dtype=int
)

OBJ_JOINT = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, "obj_free")
OBJ_QADR = int(MODEL.jnt_qposadr[OBJ_JOINT])

# ---------------------------------------------------------------------------
# 固定基座 IK 求解器，复用于移动底盘上的机械臂
# ---------------------------------------------------------------------------
IK_POSTURE_COST = np.zeros(IK_MODEL.nv)
IK_POSTURE_COST[:6] = 0.005


_ik_probe = mujoco.MjData(IK_MODEL)
_ik_probe.qpos[:6] = HOME_QPOS
mujoco.mj_forward(IK_MODEL, _ik_probe)
FIXED_BASE_ROT = _ik_probe.xmat[IK_BASE_BODY].reshape(3, 3).copy()
TARGET_ROT = Rotation.from_quat(TARGET_QUAT, scalar_first=True).as_matrix()


def solve_ik_fixed(
    target_pos: np.ndarray,
    target_quat: np.ndarray = TARGET_QUAT,
) -> tuple[np.ndarray, float]:
    """在固定基座 IK 模型中求解 6 轴机械臂关节角。

功能：
    使用 mink FrameTask 对齐 pinch 站点位置和姿态，并用 PostureTask 保持关节处于自然姿态附近。
参数：
    target_pos: 固定基座模型坐标系下的目标位置，shape=(3,)，单位 m。
    target_quat: 目标姿态四元数，顺序为 wxyz，默认工具 +Y 朝下。
返回：
    (前 6 个关节角, 末端位置残差)；关节角 shape=(6,)，残差单位 m。
异常：
    QP 求解失败时由 mink/numpy 向后抛出。
关键参数：
    600 次迭代和 dt=0.01 控制收敛速度；PostureTask 权重过大会牺牲末端精度，过小则可能出现肘部翻转。
排查：
    残差异常大时先确认目标是否在可达空间，再检查 frame_name、四元数顺序和姿态约束。
重要性等级：A"""
    tmp = mujoco.MjData(IK_MODEL)
    tmp.qpos[:6] = HOME_QPOS
    tmp.qpos[6:8] = 0.0
    mujoco.mj_forward(IK_MODEL, tmp)

    cfg = mink.Configuration(IK_MODEL)
    cfg.update(tmp.qpos)
    ee = mink.FrameTask(
        frame_name="pinch",
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )
    posture = mink.PostureTask(IK_MODEL, cost=IK_POSTURE_COST)
    posture_target = tmp.qpos.copy()
    posture_target[:6] = HOME_QPOS
    posture.set_target(posture_target)

    quat = np.asarray(target_quat, dtype=float)
    quat = quat / np.linalg.norm(quat)
    ee.set_target(
        mink.SE3.from_rotation_and_translation(
            mink.SO3(quat), np.asarray(target_pos, dtype=float)
        )
    )

    error = float("inf")
    for _ in range(600):
        velocity = mink.solve_ik(cfg, [ee, posture], 0.01, solver=SOLVER)
        cfg.integrate_inplace(velocity, 0.01)
        error = float(np.linalg.norm(cfg.data.site_xpos[IK_GRASP_SITE] - target_pos))
        if error < 1e-4:
            break
    return cfg.q[:6].copy(), error


CARRY_Q, CARRY_IK_ERROR = solve_ik_fixed(CARRY_FIXED, TARGET_QUAT)


def world_target_to_fixed(target_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把移动底盘世界系中的目标转换到固定基座 IK 模型坐标系。

参数：
    target_world: 世界坐标位置，shape=(3,)，单位 m。
返回：
    (target_fixed, quat_fixed)，分别为固定基座坐标系位置和目标四元数 wxyz。
副作用：
    调用 mj_forward 刷新当前移动底盘的 xpos/xmat。
重要性等级：A"""
    mujoco.mj_forward(MODEL, DATA)
    arm_pos = DATA.xpos[ARM_BASE_BODY].copy()
    arm_rot = DATA.xmat[ARM_BASE_BODY].reshape(3, 3).copy()
    target_world = np.asarray(target_world, dtype=float)

    target_fixed = FIXED_BASE_ROT @ arm_rot.T @ (target_world - arm_pos)
    rot_fixed = FIXED_BASE_ROT @ arm_rot.T @ TARGET_ROT
    quat_fixed = Rotation.from_matrix(rot_fixed).as_quat(scalar_first=True)
    return target_fixed, quat_fixed


def mobile_gravity_ctrl(q_arm: np.ndarray) -> np.ndarray:
    """计算机械臂位置执行器的重力补偿控制量。

功能：
    在独立 MjData 中复制底盘位姿和指定关节角，求 qfrc_bias，再按 ctrl=q+tau/kp 生成位置指令。
参数：
    q_arm: 6 个机械臂关节目标角，shape=(6,)，单位 rad。
返回：
    6 个执行器控制量，shape=(6,)；数值单位取决于位置执行器增益。
副作用：
    不修改主 DATA，避免污染仿真中的夹爪和物体状态。
关键参数：
    必须使用独立 tmp_data；直接对主 DATA 调用 mj_inverse 会混入接触和外力并产生错误力矩。
重要性等级：A"""
    tmp = mujoco.MjData(MODEL)
    tmp.qpos[:] = DATA.qpos
    tmp.qpos[ARM_QADR] = q_arm
    tmp.qvel[:] = 0.0
    mujoco.mj_forward(MODEL, tmp)
    tau = tmp.qfrc_bias[ARM_DADR].copy()
    kp = MODEL.actuator_gainprm[ARM_ACT, 0]
    return q_arm + tau / kp


# ---------------------------------------------------------------------------
# 运动辅助函数
# ---------------------------------------------------------------------------
class ArmMotion:
    """管理机械臂从当前关节角到目标关节角的平滑运动。

状态：
    active 表示是否正在插值；q_start/q_target/q_hold 均为 6 维关节角。
副作用：
    运行期间持续写入主 DATA.ctrl 中对应的 6 个机械臂执行器。
重要性等级：A"""
    def __init__(self) -> None:
        """初始化机械臂运动状态并停在预计算搬运姿态。

返回：
    无显式返回值。
副作用：
    只初始化对象字段，不写 MuJoCo 数据。
重要性等级：B"""
        self.active = False
        self.t = 0.0
        self.duration = 0.0
        self.q_start = CARRY_Q.copy()
        self.q_target = CARRY_Q.copy()
        self.q_hold = CARRY_Q.copy()
        self.active = False

    def set_ctrl(self, q: np.ndarray) -> None:
        """写入带重力补偿的机械臂位置控制指令。

参数：
    q: 6 维关节目标角，shape=(6,)，单位 rad。
副作用：
    修改 DATA.ctrl 的 ARM_ACT 项。
重要性等级：B"""
        DATA.ctrl[ARM_ACT] = mobile_gravity_ctrl(q)

    def hold(self) -> None:
        """保持当前记录的机械臂关节姿态。

副作用：
    每个仿真帧调用一次以持续提供重力补偿控制。
重要性等级：B"""
        self.set_ctrl(self.q_hold)

    def start_fixed(
        self,
        target_pos_fixed: np.ndarray,
        duration: float,
        target_quat: np.ndarray = TARGET_QUAT,
    ) -> None:
        """启动到固定基座坐标系目标位姿的平滑运动。

参数：
    target_pos_fixed: 固定基座坐标系目标位置，shape=(3,)，单位 m。
    duration: 插值时长，单位 s；小于 1e-6 时按 1e-6 处理。
    target_quat: 目标姿态四元数 wxyz。
副作用：
    求解 IK 并更新 q_start、q_target、q_hold、active 等状态，同时打印 IK 残差。
异常：
    不可达目标通常表现为较大残差，而不是直接抛异常。
重要性等级：A"""
        q_target, error = solve_ik_fixed(target_pos_fixed, target_quat)
        self.q_start = DATA.qpos[ARM_QADR].copy()
        self.q_target = q_target
        self.q_hold = q_target.copy()
        self.t = 0.0
        self.duration = max(float(duration), 1e-6)
        self.active = True
        print(
            f"  arm IK fixed={np.round(target_pos_fixed, 3)} "
            f"err={error * 1000:.2f} mm"
        )

    def start_world(self, target_world: np.ndarray, duration: float) -> None:
        """把世界坐标目标转换后启动机械臂运动。

参数：
    target_world: 世界坐标位置，shape=(3,)，单位 m。
    duration: 插值时长，单位 s。
副作用：
    打印目标位置并更新机械臂运动状态。
重要性等级：A"""
        target_fixed, quat_fixed = world_target_to_fixed(target_world)
        print(f"  arm target world={np.round(target_world, 3)}")
        self.start_fixed(target_fixed, duration, quat_fixed)

    def update(self, dt: float) -> None:
        """推进一帧机械臂平滑插值。

参数：
    dt: 本帧仿真时间增量，单位 s。
副作用：
    写入控制量；到达终点后自动切回 hold 状态。
重要性等级：B"""
        if not self.active:
            self.hold()
            return
        self.t = min(self.duration, self.t + dt)
        alpha = smoothstep(self.t / self.duration)
        q_cmd = (1.0 - alpha) * self.q_start + alpha * self.q_target
        self.set_ctrl(q_cmd)
        if self.t >= self.duration:
            self.active = False
            self.q_hold = self.q_target.copy()
            self.set_ctrl(self.q_target)


class FingerMotion:
    """管理 Robotiq 2F-85 夹爪单执行器的开合运动。

控制约定：
    FINGER_OPEN=0.0，FINGER_CLOSE=255.0；数值范围由 XML 执行器 ctrlrange 决定。
重要性等级：A"""
    def __init__(self) -> None:
        """初始化夹爪为张开且非运动状态。

副作用：
    只初始化字段，不写 MuJoCo 数据。
重要性等级：C"""
        self.active = False
        self.t = 0.0
        self.duration = 0.0
        self.start_ctrl = float(FINGER_OPEN)
        self.target = float(FINGER_OPEN)

    def hold(self) -> None:
        """持续写入当前夹爪目标控制量。

副作用：
    修改 DATA.ctrl 中的夹爪执行器项。
重要性等级：B"""
        DATA.ctrl[FINGER_ACT] = float(self.target)

    def start(self, target: float, duration: float) -> None:
        """启动夹爪开合插值。

参数：
    target: 目标执行器控制量，典型值为 0.0 或 255.0。
    duration: 插值时长，单位 s。
副作用：
    从当前 ctrl 值开始过渡，避免阶跃冲击。
重要性等级：B"""
        self.start_ctrl = float(DATA.ctrl[FINGER_ACT])
        self.target = float(target)
        self.t = 0.0
        self.duration = max(float(duration), 1e-6)
        self.active = True

    def update(self, dt: float) -> None:
        """推进一帧夹爪开合插值。

参数：
    dt: 本帧仿真时间增量，单位 s。
副作用：
    修改夹爪控制量；到达目标后保持。
重要性等级：B"""
        if not self.active:
            self.hold()
            return
        self.t = min(self.duration, self.t + dt)
        alpha = smoothstep(self.t / self.duration)
        DATA.ctrl[FINGER_ACT] = (
            (1.0 - alpha) * self.start_ctrl + alpha * self.target
        )
        if self.t >= self.duration:
            self.active = False
            self.hold()


class Simulation:
    """封装固定步数的 MuJoCo 推进、viewer 同步和实时节流。

用途：
    所有任务函数通过同一入口步进物理，保证相机处理或路径控制不会绕过 mj_step。
重要性等级：A"""
    def __init__(self, viewer=None, realtime: bool = True) -> None:
        """配置仿真步进与可选 viewer。

参数：
    viewer: mujoco.viewer 句柄；headless 时为 None。
    realtime: True 时按仿真时间与真实耗时的差值 sleep。
重要性等级：B"""
        self.viewer = viewer
        self.realtime = realtime
        self.steps_per_frame = STEPS_PER_FRAME
        self.dt = float(MODEL.opt.timestep * self.steps_per_frame)

    def step(self) -> None:
        """推进一帧物理仿真并同步 viewer。

算法：
    每次调用执行 STEPS_PER_FRAME 个固定 timestep；sim.dt 是整帧时长。
异常：
    viewer 被关闭时抛 KeyboardInterrupt，用于终止任务循环。
副作用：
    修改 DATA 全量状态，并在有 viewer 时同步画面。
重要性等级：A"""
        frame_start = time.perf_counter()
        for _ in range(self.steps_per_frame):
            mujoco.mj_step(MODEL, DATA)
        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()
            if self.realtime:
                remaining = self.dt - (time.perf_counter() - frame_start)
                if remaining > 0.0:
                    time.sleep(remaining)


def set_wheel_velocity(left: float, right: float, steer: float = 0.0) -> None:
    # 四轮同时滚动；前轴同时负责转向。若给前轴速度执行器下发 0，
    # 它会制动车轮并导致底盘侧滑。
    """设置四轮速度与前轮转向角。

参数：
    left: 左侧两轮角速度指令，单位 rad/s。
    right: 右侧两轮角速度指令，单位 rad/s。
    steer: 前轮转向角指令，单位 rad。
关键参数：
    前轮也必须保持滚动速度；只转向而把速度设零会触发执行器制动，导致打滑。
副作用：
    写入五个底盘相关执行器。
重要性等级：A"""
    DATA.ctrl[WHEEL_ACT["wheel_fl_joint"]] = left
    DATA.ctrl[WHEEL_ACT["wheel_fr_joint"]] = right
    DATA.ctrl[WHEEL_ACT["wheel_rl_joint"]] = left
    DATA.ctrl[WHEEL_ACT["wheel_rr_joint"]] = right
    DATA.ctrl[WHEEL_STEER_ACT] = steer


def chassis_pose() -> tuple[np.ndarray, float]:
    """读取底盘世界位置和偏航角。

返回：
    (pos, yaw)，pos shape=(3,) 单位 m，yaw 单位 rad。
副作用：
    调用 mj_forward 刷新派生状态。
重要性等级：B"""
    mujoco.mj_forward(MODEL, DATA)
    pos = DATA.qpos[CHASSIS_QADR : CHASSIS_QADR + 3].copy()
    rot = DATA.xmat[CHASSIS_BODY].reshape(3, 3)
    yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    return pos, yaw


def run_arm_motion(sim: Simulation, arm: ArmMotion, finger: FingerMotion) -> None:
    """阻塞推进机械臂运动直到当前插值完成。

参数：
    sim、arm、finger: 共享的仿真、机械臂和夹爪控制器。
副作用：
    每帧更新机械臂控制、保持夹爪并调用 sim.step。
重要性等级：B"""
    while arm.active:
        arm.update(sim.dt)
        finger.hold()
        sim.step()
    arm.hold()


def run_finger_motion(sim: Simulation, arm: ArmMotion, finger: FingerMotion) -> None:
    """阻塞推进夹爪运动直到当前插值完成。

副作用：
    每帧保持机械臂、更新夹爪并调用 sim.step。
重要性等级：B"""
    while finger.active:
        arm.hold()
        finger.update(sim.dt)
        sim.step()
    finger.hold()


def wait_with_pose(
    sim: Simulation,
    arm: ArmMotion,
    finger: FingerMotion,
    duration: float,
) -> None:
    """在保持当前机械臂和夹爪姿态的情况下等待指定时长。

参数：
    duration: 等待时间，单位 s。
关键参数：
    等待步数由 round(duration/sim.dt) 得到，至少一步。
副作用：
    将轮速清零并推进仿真，使接触和物体沉降稳定。
重要性等级：B"""
    set_wheel_velocity(0.0, 0.0, 0.0)
    steps = max(1, int(round(duration / sim.dt)))
    for _ in range(steps):
        arm.hold()
        finger.hold()
        sim.step()


# ---------------------------------------------------------------------------
# 底盘控制器
# ---------------------------------------------------------------------------
def drive_chassis(
    sim: Simulation,
    arm: ArmMotion,
    finger: FingerMotion,
    target_x: float,
    target_y: float,
) -> None:
    """使用前轮转向和差速轮速控制底盘驶向目标点。

参数：
    target_x, target_y: 世界平面目标坐标，单位 m。
关键参数：
    K_POS 控制距离比例；STEER_MAX 限制前轮角；COMMAND_RAMP 限制指令变化率。
返回：
    无返回值，结束后底盘已刹停并稳定。
异常：
    超过 MAX_DRIVE_TIME 仍未满足 POS_TOL 时函数退出，不主动报错。
算法：
    alpha 为目标方位角误差；纯追踪前轮角控制转向，再用
    v_left=v-w*track/2、v_right=v+w*track/2 分配左右轮速。
重要性等级：A"""
    target = np.array([target_x, target_y], dtype=float)
    start_time = DATA.time
    left_cmd = 0.0
    right_cmd = 0.0
    steer_cmd = 0.0
    print(f"\n[DRIVE CHASSIS] target=({target_x:.3f}, {target_y:.3f})")

    while DATA.time - start_time < MAX_DRIVE_TIME:
        pos, yaw = chassis_pose()
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        distance = float(np.hypot(dx, dy))
        if distance < POS_TOL:
            break

        alpha = wrap_angle(float(np.arctan2(dy, dx)) - yaw)
        v = float(np.clip(K_POS * distance, 0.0, V_MAX)) * max(0.0, float(np.cos(alpha)))

        # 纯追踪前轮转角与自行车模型偏航角速度。
        steer_target = float(
            np.arctan2(
                2.0 * STEER_WHEELBASE * np.sin(alpha),
                max(distance, 0.20),
            )
        )
        steer_target = float(np.clip(steer_target, -STEER_MAX, STEER_MAX))
        if v > 0.02:
            w = v * float(np.tan(steer_target)) / STEER_WHEELBASE
        else:
            w = 0.0
        w = float(np.clip(w, -W_MAX, W_MAX))

        v_left = v - w * WHEEL_TRACK / 2.0
        v_right = v + w * WHEEL_TRACK / 2.0
        left_target = v_left / WHEEL_RADIUS
        right_target = v_right / WHEEL_RADIUS

        left_cmd += COMMAND_RAMP * (left_target - left_cmd)
        right_cmd += COMMAND_RAMP * (right_target - right_cmd)
        steer_cmd += COMMAND_RAMP * (steer_target - steer_cmd)
        set_wheel_velocity(left_cmd, right_cmd, steer_cmd)
        arm.hold()
        finger.hold()
        sim.step()

    # 先制动并等待底盘稳定，再让机械臂开始运动。
    set_wheel_velocity(0.0, 0.0, 0.0)
    settle_steps = max(1, int(round(CHASSIS_SETTLE_TIME / sim.dt)))
    for _ in range(settle_steps):
        arm.hold()
        finger.hold()
        sim.step()

    pos, yaw = chassis_pose()
    print(
        f"  stopped at ({pos[0]:.3f}, {pos[1]:.3f}), "
        f"yaw={yaw:.3f} rad"
    )


# ---------------------------------------------------------------------------
# 任务函数
# ---------------------------------------------------------------------------
def execute_grasp(
    sim: Simulation,
    arm: ArmMotion,
    finger: FingerMotion,
    target_pos: np.ndarray,
) -> None:
    """执行接近、下探、闭合、提升和收回的抓取序列。

参数：
    target_pos: 圆柱世界坐标，shape=(3,)，单位 m。
关键参数：
    grasp/approach/lift 的 Z 高度决定是否压入地面或漏抓；夹爪闭合值由 FINGER_CLOSE 控制。
返回：
    无显式返回值；成功与否由后续物体高度和任务结果判断。
副作用：
    推进仿真、打印夹爪作用力，并可能抓起物体。
重要性等级：A"""
    print("\n[GRASP]")
    target_pos = np.asarray(target_pos, dtype=float)
    approach = np.array([target_pos[0], target_pos[1], APPROACH_WORLD_Z])
    grasp = np.array([target_pos[0], target_pos[1], GRASP_WORLD_Z])
    lift = np.array([target_pos[0], target_pos[1], LIFT_WORLD_Z])

    arm.start_world(approach, APPROACH_TIME)
    run_arm_motion(sim, arm, finger)

    arm.start_world(grasp, DESCEND_TIME)
    run_arm_motion(sim, arm, finger)

    finger.start(FINGER_CLOSE, CLOSE_TIME)
    run_finger_motion(sim, arm, finger)
    force = float(abs(DATA.actuator_force[FINGER_ACT]))
    print(f"  finger force after close: {force:.2f} N")

    arm.start_world(lift, LIFT_TIME)
    run_arm_motion(sim, arm, finger)

    arm.start_fixed(CARRY_FIXED, CARRY_TIME)
    run_arm_motion(sim, arm, finger)

    obj = DATA.qpos[OBJ_QADR : OBJ_QADR + 3]
    print(f"  cylinder after grasp: {np.round(obj, 4)}")


def execute_place(
    sim: Simulation,
    arm: ArmMotion,
    finger: FingerMotion,
    target_pos: np.ndarray,
) -> None:
    """执行移动到桶口、下探、释放、等待沉降和收回的投放序列。

参数：
    target_pos: 桶口投放目标，shape=(3,)，单位 m。
副作用：
    推进仿真并打印释放前后的物体位置及桶内 XY 误差。
异常：
    本函数不主动抛异常；执行器或模型名称错误会在访问 DATA 时暴露。
重要性等级：A"""
    print("\n[PLACE]")
    target_pos = np.asarray(target_pos, dtype=float)
    pre_drop = np.array([target_pos[0], target_pos[1], PRE_DROP_WORLD_Z])
    drop = np.array([target_pos[0], target_pos[1], DROP_WORLD_Z])

    arm.start_world(pre_drop, PRE_DROP_TIME)
    run_arm_motion(sim, arm, finger)

    arm.start_world(drop, DROP_TIME)
    run_arm_motion(sim, arm, finger)
    wait_with_pose(sim, arm, finger, 0.8)

    gs = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_SITE, "pinch")
    obj_before = DATA.qpos[OBJ_QADR : OBJ_QADR + 3]
    print(f"  site before release: {np.round(DATA.site_xpos[gs], 4)}")
    print(f"  object before release: {np.round(obj_before, 4)}")

    finger.start(FINGER_OPEN, RELEASE_TIME)
    run_finger_motion(sim, arm, finger)
    wait_with_pose(sim, arm, finger, RELEASE_SETTLE_TIME)

    arm.start_fixed(CARRY_FIXED, RETRACT_TIME)
    run_arm_motion(sim, arm, finger)
    wait_with_pose(sim, arm, finger, FINAL_SETTLE_TIME)

    obj = DATA.qpos[OBJ_QADR : OBJ_QADR + 3]
    xy_error = float(np.linalg.norm(obj[:2] - BUCKET_POS[:2]))
    print(f"  cylinder final: {np.round(obj, 4)}")
    print(f"  bucket xy error: {xy_error:.4f} m")


# ---------------------------------------------------------------------------
# 初始状态与主流程
# ---------------------------------------------------------------------------
def setup_initial_state() -> tuple[ArmMotion, FingerMotion]:
    """重置底盘、机械臂、夹爪和圆柱的初始仿真状态。

返回：
    (ArmMotion, FingerMotion) 控制器对象。
副作用：
    覆盖 DATA.qpos、qvel、ctrl 和 time，属于全局状态重置。
重要性等级：A"""
    DATA.qpos[:] = 0.0
    DATA.qpos[CHASSIS_QADR : CHASSIS_QADR + 3] = [0.0, 0.0, 0.0]
    DATA.qpos[CHASSIS_QADR + 3 : CHASSIS_QADR + 7] = [1.0, 0.0, 0.0, 0.0]
    DATA.qpos[ARM_QADR] = CARRY_Q
    DATA.qpos[OBJ_QADR : OBJ_QADR + 3] = OBJ_POS
    DATA.qpos[OBJ_QADR + 3 : OBJ_QADR + 7] = [1.0, 0.0, 0.0, 0.0]
    DATA.qvel[:] = 0.0
    DATA.ctrl[:] = 0.0
    mujoco.mj_forward(MODEL, DATA)

    arm = ArmMotion()
    finger = FingerMotion()
    arm.q_hold = CARRY_Q.copy()
    arm.hold()
    DATA.ctrl[FINGER_ACT] = FINGER_OPEN
    finger.hold()
    DATA.time = 0.0
    return arm, finger


def run_sequence(sim: Simulation, arm: ArmMotion, finger: FingerMotion) -> bool:
    """按“行进-抓取-行进-投放”顺序运行完整单臂任务。

返回：
    True 表示圆柱最终位于桶口 XY 容差与高度范围内。
副作用：
    推进仿真并打印最终底盘、圆柱、抓取和放置指标。
重要性等级：A"""
    drive_chassis(sim, arm, finger, OBJECT_STOP[0], OBJECT_STOP[1])
    execute_grasp(sim, arm, finger, OBJ_POS)

    drive_chassis(sim, arm, finger, BUCKET_STOP[0], BUCKET_STOP[1])
    execute_place(sim, arm, finger, BUCKET_DROP_WORLD)

    obj = DATA.qpos[OBJ_QADR : OBJ_QADR + 3].copy()
    xy_error = float(np.linalg.norm(obj[:2] - BUCKET_POS[:2]))
    inside = bool(
        xy_error < BUCKET_XY_TOL and 0.0 < float(obj[2]) < BUCKET_Z_MAX
    )
    pos, yaw = chassis_pose()
    print("\n" + "=" * 68)
    print(f"chassis final: ({pos[0]:.4f}, {pos[1]:.4f}), yaw={yaw:.4f} rad")
    print(f"cylinder final: {np.round(obj, 4)}")
    print(f"bucket xy error: {xy_error:.4f} m")
    print(f"cylinder inside bucket: {inside}")
    print(f"grasp success: {float(obj[2]) > 0.0}")
    print(f"place success: {inside}")
    print("=" * 68)
    return inside


def main() -> None:
    """解析命令行参数并启动单臂仿真。

参数：
    命令行可选 --headless；是否离屏由 argparse 决定。
返回：
    无显式返回值。
异常：
    用户关闭 viewer 或按 Ctrl+C 时由上层捕获 KeyboardInterrupt。
副作用：
    创建 viewer、初始化全局仿真状态并执行任务。
重要性等级：A"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    print(f"mobile scene: {MOBILE_SCENE}")
    print(f"IK scene: {IK_SCENE}")
    print(f"model: nq={MODEL.nq} nv={MODEL.nv} nu={MODEL.nu}")
    print(f"IK solver: {SOLVER}")
    print(f"carry IK error: {CARRY_IK_ERROR * 1000:.2f} mm")

    arm, finger = setup_initial_state()

    if args.headless:
        sim = Simulation(realtime=False)
        run_sequence(sim, arm, finger)
        return

    import mujoco.viewer

    with mujoco.viewer.launch_passive(MODEL, DATA) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [1.25, 0.90, 0.25]
        viewer.cam.distance = 3.60
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -24.0

        sim = Simulation(viewer, realtime=True)
        run_sequence(sim, arm, finger)

        print("Motion finished. Close the viewer window to exit.")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.")