"""把夹爪单独渲染一张特写，用于目检视觉模型（不参与任何物理与基线）。

用法：
    D:\\Python313\\python.exe scripts\\render_gripper.py
输出：
    reports/diag/gripper_visual.png
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).absolute().parent.parent
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import mobile_pick_place as mpp  # noqa: E402

MODEL, DATA = mpp.MODEL, mpp.DATA
PINCH = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_SITE, "pinch")

DATA.qpos[mpp.ARM_QADR] = mpp.CARRY_Q
DATA.qpos[19:21] = [0.045, 0.045]
mujoco.mj_forward(MODEL, DATA)

lookat = DATA.site_xpos[PINCH].copy() + np.array([0.0, 0.0, 0.09])
renderer = mujoco.Renderer(MODEL, height=480, width=480)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = lookat
cam.distance = 0.55
cam.azimuth = 40.0
cam.elevation = -12.0
renderer.update_scene(DATA, cam)
img = renderer.render()

out = ROOT / "reports" / "diag" / "gripper_visual.png"
out.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
print(f"written {out}")

# 顺便把夹爪各几何体的世界包围盒打出来，判断视觉与碰撞是否对得上。
def geom_world_z(gid: int) -> tuple[float, float]:
    """返回该几何体在竖直方向的世界范围（近似：用几何中心 + 半径/尺寸）。"""
    pos = DATA.geom_xpos[gid]
    if MODEL.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
        mid = int(MODEL.geom_dataid[gid])
        nvert = int(MODEL.mesh_vertnum[mid])
        start = int(MODEL.mesh_vertadr[mid])
        verts = MODEL.mesh_vert[start : start + nvert]
        world = (DATA.geom_xmat[gid].reshape(3, 3) @ verts.T).T + pos
        return float(world[:, 2].min()), float(world[:, 2].max())
    size = MODEL.geom_size[gid]
    half = float(size[2] if MODEL.geom_type[gid] != mujoco.mjtGeom.mjGEOM_SPHERE else size[0])
    return float(pos[2] - half), float(pos[2] + half)


print("\nRG2 各几何体竖直范围（世界 z）：")
for gid in range(MODEL.ngeom):
    name = MODEL.geom(gid).name or ""
    if "rg2" not in name:
        continue
    lo, hi = geom_world_z(gid)
    print(f"  {name:24s} type={int(MODEL.geom_type[gid])} "
          f"z=[{lo:.4f}, {hi:.4f}]  visual_group={MODEL.geom_group[gid]}")
