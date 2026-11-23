"""临时诊断：抓取阶段到底停在哪、夹爪有没有拢到杆。

做法：不改仓库文件，只把 `mission_core.run_arm_motion_concurrent` 换成带打印的包装，
每段机械臂动作结束后打印 pinch 实际位置、指爪关节角、杆位姿和与夹爪相关的接触。

运行：D:\\Python313\\python.exe scripts\\diag_grasp.py --headless --seed 0
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).absolute().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

import dual_car_pick_place as dcp  # noqa: E402
import mission_core as mc  # noqa: E402
from mobile_pick_place import (  # noqa: E402
    DATA,
    MODEL,
    OBJ_QADR,
    ARM_QADR,
)

BASE = dcp.base
RG2_BODIES = {
    mujoco_id
    for mujoco_id in range(MODEL.nbody)
    if "rg2" in (MODEL.body(mujoco_id).name or "")
}
JAW_JOINTS = [
    i for i in range(MODEL.njnt) if "rg2_jaw" in (MODEL.joint(i).name or "")
]
JAW_QADR = [int(MODEL.jnt_qposadr[j]) for j in JAW_JOINTS]


def contacts_touching(bodies: set[int]) -> list[str]:
    """列出与给定 body 集合相关的接触对（geom 名）。"""
    out = []
    for i in range(DATA.ncon):
        c = DATA.contact[i]
        b1 = int(MODEL.geom_bodyid[c.geom1])
        b2 = int(MODEL.geom_bodyid[c.geom2])
        if b1 in bodies or b2 in bodies:
            g1 = MODEL.geom(c.geom1).name or f"geom{c.geom1}"
            g2 = MODEL.geom(c.geom2).name or f"geom{c.geom2}"
            out.append(f"{g1}|{g2}")
    return out


_orig = mc.run_arm_motion_concurrent
_phase = {"n": 0}


def traced(io, follower):
    _orig(io, follower)
    _phase["n"] += 1
    pinch = io.arm.tool_position()
    xmat = DATA.site_xmat[dcp.PINCH_SITE].reshape(3, 3)
    jaw = [round(float(DATA.qpos[a]), 4) for a in JAW_QADR]
    rod = DATA.qpos[OBJ_QADR : OBJ_QADR + 3]
    rod_q = DATA.qpos[OBJ_QADR + 3 : OBJ_QADR + 7]
    print(
        f"TRACE#{_phase['n']} t={DATA.time:6.2f} pinch={np.round(pinch, 4)} "
        f"jaw_q={jaw} force={abs(float(DATA.actuator_force[BASE.FINGER_ACT])):.2f} "
        f"rod={np.round(rod, 4)} rod_q={np.round(rod_q, 3)}"
    )
    print(f"         jaw_axis_world={np.round(xmat[:, 0], 3)} "
          f"tool_z_world={np.round(xmat[:, 2], 3)}")
    pairs = contacts_touching(RG2_BODIES)
    if pairs:
        uniq = sorted(set(pairs))
        print(f"         rg2 contacts n={len(pairs)}: {uniq[:8]}")


mc.run_arm_motion_concurrent = traced


# --- 逐控制帧跟踪：抓住之后杆有没有滑脱、什么时候滑脱 ---
OBJ_BODY = int(MODEL.body("grasp_object").id)
_orig_step = dcp.SimClock.step
_next_log = {"t": 0.0}


def logged_step(self):
    """推进一帧后按 0.2 s 打印杆与夹持点的关系。"""
    _orig_step(self)
    if DATA.time < 40.0 or DATA.time - _next_log["t"] < 0.2:
        return
    _next_log["t"] = DATA.time
    rod = DATA.qpos[OBJ_QADR : OBJ_QADR + 3]
    pinch = DATA.site_xpos[dcp.PINCH_SITE]
    n_obj = sum(
        1
        for i in range(DATA.ncon)
        if int(MODEL.geom_bodyid[DATA.contact[i].geom1]) == OBJ_BODY
        or int(MODEL.geom_bodyid[DATA.contact[i].geom2]) == OBJ_BODY
    )
    n_pad = sum(
        1
        for i in range(DATA.ncon)
        if "rg2_jaw" in (MODEL.geom(DATA.contact[i].geom1).name or "")
        or "rg2_jaw" in (MODEL.geom(DATA.contact[i].geom2).name or "")
    )
    jaw = [round(float(DATA.qpos[a]), 4) for a in JAW_QADR]
    print(
        f"STEP t={DATA.time:6.2f} pinch={np.round(pinch, 3)} rod={np.round(rod, 3)} "
        f"d={np.linalg.norm(rod - pinch) * 1000:7.1f}mm n_obj={n_obj} n_pad={n_pad} "
        f"jaw={jaw}"
    )


dcp.SimClock.step = logged_step

sys.argv = ["dual_car_pick_place.py", "--headless", "--seed", "0"]
try:
    dcp.main()
except SystemExit as exc:
    print(f"EXIT={exc.code}")
