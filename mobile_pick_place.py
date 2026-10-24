"""UR10e mobile manipulator phase-1.5 rule-based pick-and-place.

Sequence:
  chassis drives to 0.5 m in front of the cylinder
  -> arm grasps and retracts
  -> chassis drives beside the bucket
  -> arm places the cylinder into the bucket

Run with 3D viewer:
    python mobile_pick_place.py

Headless:
    python mobile_pick_place.py --headless
"""

from __future__ import annotations

import argparse
import os
import time

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
ASCII_ROOT = "D:/robot_mobile"
IK_ROOT = ASCII_ROOT if os.path.exists(os.path.join(ASCII_ROOT, "arm_ik_scene.xml")) else HERE
IK_SCENE = os.path.join(IK_ROOT, "arm_ik_scene.xml")
MOBILE_SCENE = os.environ.get(
    "MOBILE_SCENE_PATH", os.path.join(IK_ROOT, "dual_car_scene.xml")
)

# ---------------------------------------------------------------------------
# Task geometry and control parameters
# ---------------------------------------------------------------------------
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
TARGET_QUAT = np.array([0.0, 1.0, 0.0, 0.0])  # tool +Y down

GROUND_Z = 0.0
OBJECT_RADIUS = 0.01
OBJ_POS = np.array([6.00, 0.00, GROUND_Z + OBJECT_RADIUS])
BUCKET_POS = np.array([6.00, 4.00, 0.00])
OBJECT_STOP = np.array([5.00, 0.00])       # 1.00 m behind the cylinder
BUCKET_STOP = np.array([5.00, 4.00])       # 1.00 m behind the bucket
BUCKET_DROP_WORLD = np.array([6.00, 4.00, 0.28])

APPROACH_WORLD_Z = OBJ_POS[2] + 0.17
GRASP_WORLD_Z = OBJ_POS[2] + 0.007
LIFT_WORLD_Z = OBJ_POS[2] + 0.30
PRE_DROP_WORLD_Z = 0.42
DROP_WORLD_Z = 0.28

CARRY_FIXED = np.array([0.20, 0.0, 0.45])   # arm-frame carry pose, inside chassis footprint
FINGER_OPEN = 0.0
FINGER_CLOSE = 255.0

# Timings, seconds
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

# Chassis P controller
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

# Simulation / rendering
STEPS_PER_FRAME = 20
IK_TOL = 0.003
LIFT_TOL = 0.05
CONTACT_FORCE_TOL = 0.5
BUCKET_XY_TOL = 0.05
BUCKET_Z_MAX = 0.15


def smoothstep(t: float) -> float:
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def pick_solver() -> str:
    try:
        import quadprog  # noqa: F401
        return "quadprog"
    except ImportError:
        return "daqp"


# ---------------------------------------------------------------------------
# Models and named indices
# ---------------------------------------------------------------------------
IK_MODEL = mujoco.MjModel.from_xml_path(IK_SCENE)
MODEL = mujoco.MjModel.from_xml_path(MOBILE_SCENE)
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
    return int(MODEL.jnt_qposadr[mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, name)])


def joint_dadr(name: str) -> int:
    return int(MODEL.jnt_dofadr[mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, name)])


def actuator_id(name: str) -> int:
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
# Fixed-base IK solver, reused for the arm on the moving chassis
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
    """Map a world target into the fixed-base IK model's world frame."""
    mujoco.mj_forward(MODEL, DATA)
    arm_pos = DATA.xpos[ARM_BASE_BODY].copy()
    arm_rot = DATA.xmat[ARM_BASE_BODY].reshape(3, 3).copy()
    target_world = np.asarray(target_world, dtype=float)

    target_fixed = FIXED_BASE_ROT @ arm_rot.T @ (target_world - arm_pos)
    rot_fixed = FIXED_BASE_ROT @ arm_rot.T @ TARGET_ROT
    quat_fixed = Rotation.from_matrix(rot_fixed).as_quat(scalar_first=True)
    return target_fixed, quat_fixed


def mobile_gravity_ctrl(q_arm: np.ndarray) -> np.ndarray:
    tmp = mujoco.MjData(MODEL)
    tmp.qpos[:] = DATA.qpos
    tmp.qpos[ARM_QADR] = q_arm
    tmp.qvel[:] = 0.0
    mujoco.mj_forward(MODEL, tmp)
    tau = tmp.qfrc_bias[ARM_DADR].copy()
    kp = MODEL.actuator_gainprm[ARM_ACT, 0]
    return q_arm + tau / kp


# ---------------------------------------------------------------------------
# Motion helpers
# ---------------------------------------------------------------------------
class ArmMotion:
    def __init__(self) -> None:
        self.active = False
        self.t = 0.0
        self.duration = 0.0
        self.q_start = CARRY_Q.copy()
        self.q_target = CARRY_Q.copy()
        self.q_hold = CARRY_Q.copy()
        self.active = False

    def set_ctrl(self, q: np.ndarray) -> None:
        DATA.ctrl[ARM_ACT] = mobile_gravity_ctrl(q)

    def hold(self) -> None:
        self.set_ctrl(self.q_hold)

    def start_fixed(
        self,
        target_pos_fixed: np.ndarray,
        duration: float,
        target_quat: np.ndarray = TARGET_QUAT,
    ) -> None:
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
        target_fixed, quat_fixed = world_target_to_fixed(target_world)
        print(f"  arm target world={np.round(target_world, 3)}")
        self.start_fixed(target_fixed, duration, quat_fixed)

    def update(self, dt: float) -> None:
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
    def __init__(self) -> None:
        self.active = False
        self.t = 0.0
        self.duration = 0.0
        self.start_ctrl = float(FINGER_OPEN)
        self.target = float(FINGER_OPEN)

    def hold(self) -> None:
        DATA.ctrl[FINGER_ACT] = float(self.target)

    def start(self, target: float, duration: float) -> None:
        self.start_ctrl = float(DATA.ctrl[FINGER_ACT])
        self.target = float(target)
        self.t = 0.0
        self.duration = max(float(duration), 1e-6)
        self.active = True

    def update(self, dt: float) -> None:
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
    def __init__(self, viewer=None, realtime: bool = True) -> None:
        self.viewer = viewer
        self.realtime = realtime
        self.steps_per_frame = STEPS_PER_FRAME
        self.dt = float(MODEL.opt.timestep * self.steps_per_frame)

    def step(self) -> None:
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
    # Four-wheel rolling. The front axle also steers; applying zero speed to it
    # would make the velocity actuators brake the wheels and skid the chassis.
    DATA.ctrl[WHEEL_ACT["wheel_fl_joint"]] = left
    DATA.ctrl[WHEEL_ACT["wheel_fr_joint"]] = right
    DATA.ctrl[WHEEL_ACT["wheel_rl_joint"]] = left
    DATA.ctrl[WHEEL_ACT["wheel_rr_joint"]] = right
    DATA.ctrl[WHEEL_STEER_ACT] = steer


def chassis_pose() -> tuple[np.ndarray, float]:
    mujoco.mj_forward(MODEL, DATA)
    pos = DATA.qpos[CHASSIS_QADR : CHASSIS_QADR + 3].copy()
    rot = DATA.xmat[CHASSIS_BODY].reshape(3, 3)
    yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    return pos, yaw


def run_arm_motion(sim: Simulation, arm: ArmMotion, finger: FingerMotion) -> None:
    while arm.active:
        arm.update(sim.dt)
        finger.hold()
        sim.step()
    arm.hold()


def run_finger_motion(sim: Simulation, arm: ArmMotion, finger: FingerMotion) -> None:
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
    set_wheel_velocity(0.0, 0.0, 0.0)
    steps = max(1, int(round(duration / sim.dt)))
    for _ in range(steps):
        arm.hold()
        finger.hold()
        sim.step()


# ---------------------------------------------------------------------------
# Chassis controller
# ---------------------------------------------------------------------------
def drive_chassis(
    sim: Simulation,
    arm: ArmMotion,
    finger: FingerMotion,
    target_x: float,
    target_y: float,
) -> None:
    """Differential-drive controller with front steering.

    The commanded linear/angular velocity is converted with
        v_left  = v - w * track / 2
        v_right = v + w * track / 2
    and the rear/front wheel speed actuators receive those left/right speeds.
    """
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

        # Pure-pursuit front-wheel angle and bicycle-model yaw rate.
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

    # Brake and let the chassis settle before the arm moves.
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
# Task functions
# ---------------------------------------------------------------------------
def execute_grasp(
    sim: Simulation,
    arm: ArmMotion,
    finger: FingerMotion,
    target_pos: np.ndarray,
) -> None:
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
# Initial state and main sequence
# ---------------------------------------------------------------------------
def setup_initial_state() -> tuple[ArmMotion, FingerMotion]:
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