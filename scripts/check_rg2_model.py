"""生成 RG2 场景副本并做加载/几何自检。

检查项：
  1. 模型能编译（网格、equality、actuator 名字都对）。
  2. 两条整车总质量、机械臂总质量是否落在预期。
  3. RG2 在全闭 / 全开时的几何：指垫间隙、指垫相对 pinch 的位置。
     → 这组数是 `PINCH_ABOVE_FLOOR` 的唯一依据，不靠猜。
"""
from __future__ import annotations

import pathlib

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).absolute().parent.parent  # 不要用 resolve()：会把 ASCII junction 还原成中文真实路径
SRC_SCENE = ROOT / "dual_car_scene.xml"
DST_SCENE = ROOT / "dual_car_scene_rg2.xml"


def make_scene() -> None:
    text = SRC_SCENE.read_text(encoding="utf-8")
    text = text.replace(
        '<include file="models/universal_robots_ur10e/dual_mobile_ur10e.xml" />',
        '<include file="models/universal_robots_ur10e/dual_mobile_ur10_rg2.xml" />',
    )
    # 2F-85 的 tendon 执行器随子树一起没了；注意 `</actuator>` 原本就写在这一行末尾，要留下
    lines = [
        "</actuator>" if 'name="fingers_actuator" tendon=' in ln else ln
        for ln in text.splitlines()
    ]
    DST_SCENE.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"已生成 {DST_SCENE.name}")


def selftest() -> None:
    m = mujoco.MjModel.from_xml_path(str(DST_SCENE))
    d = mujoco.MjData(m)
    print(f"\n编译通过：nq={m.nq} nv={m.nv} njnt={m.njnt} nu={m.nu} nbody={m.nbody} ngeom={m.ngeom} nmesh={m.nmesh}")

    total = float(m.body_mass.sum())
    print(f"整车总质量（两车+臂+夹爪+桶）: {total:.3f} kg")

    arm_bodies = [
        "base", "shoulder_link", "upper_arm_link", "forearm_link",
        "wrist_1_link", "wrist_2_link", "wrist_3_link",
    ]
    arm_mass = 0.0
    for name in arm_bodies:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        assert bid >= 0, name
        arm_mass += float(m.body_mass[bid])
    print(f"UR10 七段合计: {arm_mass:.3f} kg（UR 官方 32.4 kg）")

    grip = ["rg2_bracket", "rg2_body", "rg2_jaw_pos", "rg2_jaw_neg"]
    gm = sum(float(m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)]) for b in grip)
    print(f"RG2 合计: {gm:.3f} kg（datasheet 产品重量 0.65 + 支架 0.09 + 两指 0.04）")

    qid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rg2_jaw_pos_joint")
    nid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rg2_jaw_neg_joint")
    assert qid >= 0 and nid >= 0, "找不到 RG2 指关节"
    qadr = m.jnt_qposadr[qid]
    nadr = m.jnt_qposadr[nid]
    pinch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pinch")
    p1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "rg2_jaw_pos_pad")
    p2 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "rg2_jaw_neg_pad")

    print("\n  半开度g(mm)  指面间隙(mm)   垫心相对pinch: 横向(mm)  dz(mm)")
    for g in (0.0005, 0.003, 0.01, 0.055):
        d.qpos[qadr] = g
        d.qpos[nadr] = g   # 运行时由 equality 保证；测试里手动对称赋值
        mujoco.mj_forward(m, d)
        sep = d.geom_xpos[p1] - d.geom_xpos[p2]
        gap = np.linalg.norm(sep) * 1000 - 11.5  # 减去两块垫的厚度
        rel = (d.geom_xpos[p1] - d.site_xpos[pinch]) * 1000
        lateral = float(np.linalg.norm(rel[:2]))
        print(f"  {g * 1000:9.1f}   {gap:11.1f}    {lateral:11.1f} {rel[2]:8.1f}")
    print("\n注：指垫厚 11.5 mm、半高 14.75 mm 取自 flex_finger.stl 包围盒；"
          "\n    垫心与 pinch 同高（dz≈0）→ 抓取高度必须让 pinch 离地至少 14.75 mm。")


def grip_check() -> None:
    """夹住 Ø6 圆棒的力与保持性自检（生成后必须过，否则搬运中杆会滑掉）。

    真实 RG2 是恒力夹持，位置伺服只能靠穿透量产生力：刚度不够时稳态力只有 9 N，
    杆会以约 10 mm/s 匀速滑出（历史上就是这么丢的）。这里断言两条：
      1. 稳态夹持力接近 40 N 上限（说明伺服确实饱和，模拟的是恒力）；
      2. 后 1 s 内杆相对夹持点的位移 < 3 mm（说明没有持续滑移）。
    """
    import os
    import sys

    sys.path.insert(0, str(ROOT))
    os.environ.setdefault("MOBILE_SCENE_PATH", str(DST_SCENE))
    import mobile_pick_place as mpp

    m, d = mpp.MODEL, mujoco.MjData(mpp.MODEL)
    pinch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pinch")
    jaws = [
        int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
        for n in ("rg2_jaw_pos_joint", "rg2_jaw_neg_joint")
    ]
    d.qpos[mpp.ARM_QADR] = mpp.CARRY_Q
    d.qpos[jaws] = [0.055, 0.055]
    mujoco.mj_forward(m, d)
    pinch_pos = d.site_xpos[pinch].copy()
    xmat = d.site_xmat[pinch].reshape(3, 3)
    rod_axis = xmat[:, 1]                       # 指沿站点 x 开合，杆轴取站点 y
    rot = np.column_stack([rod_axis, np.cross(xmat[:, 2], rod_axis), xmat[:, 2]])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    d.qpos[mpp.OBJ_QADR : mpp.OBJ_QADR + 3] = pinch_pos + 0.0162 * rod_axis
    d.qpos[mpp.OBJ_QADR + 3 : mpp.OBJ_QADR + 7] = quat
    mujoco.mj_forward(m, d)

    scratch = mujoco.MjData(m)
    arm_kp = float(m.actuator_gainprm[mpp.ARM_ACT[0], 0])
    settle = int(3.0 / m.opt.timestep)
    for _ in range(settle):
        scratch.qpos[:] = d.qpos
        scratch.qpos[mpp.ARM_QADR] = mpp.CARRY_Q
        scratch.qvel[:] = 0.0
        mujoco.mj_forward(m, scratch)
        d.ctrl[mpp.ARM_ACT] = mpp.CARRY_Q + scratch.qfrc_bias[mpp.ARM_DADR] / arm_kp
        d.ctrl[mpp.FINGER_ACT] = float(mpp.FINGER_CLOSE)
        mujoco.mj_step(m, d)
    force = abs(float(d.actuator_force[mpp.FINGER_ACT]))
    mark = d.qpos[mpp.OBJ_QADR : mpp.OBJ_QADR + 3].copy()
    for _ in range(int(1.0 / m.opt.timestep)):
        scratch.qpos[:] = d.qpos
        scratch.qpos[mpp.ARM_QADR] = mpp.CARRY_Q
        scratch.qvel[:] = 0.0
        mujoco.mj_forward(m, scratch)
        d.ctrl[mpp.ARM_ACT] = mpp.CARRY_Q + scratch.qfrc_bias[mpp.ARM_DADR] / arm_kp
        d.ctrl[mpp.FINGER_ACT] = float(mpp.FINGER_CLOSE)
        mujoco.mj_step(m, d)
    creep = float(
        np.linalg.norm(d.qpos[mpp.OBJ_QADR : mpp.OBJ_QADR + 3] - mark)
    )
    pads = sum(
        1
        for i in range(d.ncon)
        if "rg2_jaw" in (m.geom(d.contact[i].geom1).name or "")
        or "rg2_jaw" in (m.geom(d.contact[i].geom2).name or "")
    )
    jaw = [round(float(d.qpos[a]), 4) for a in jaws]
    print(
        f"\n夹持自检：稳态力 {force:.1f} N，后 1 s 位移 {creep * 1000:.1f} mm，"
        f"指爪 qpos={jaw}，指垫接触 {pads} 个"
    )
    assert pads >= 2, "指垫没夹到杆：检查 pinch 站点开合方向与指杆几何"
    assert force > 30.0, f"稳态夹持力只有 {force:.1f} N，伺服没饱和，搬运中杆会滑出"
    assert creep < 0.003, f"杆在 1 s 内滑了 {creep * 1000:.1f} mm，夹不住"
    print("  通过：夹持力饱和、无持续滑移")


if __name__ == "__main__":
    make_scene()
    selftest()
    grip_check()
