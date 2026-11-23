"""把 UR10e + Robotiq 2F-85 的模型转换成 UR10 + OnRobot RG2 版本。

母本两份，各生成一份新文件，旧文件一个字节都不动（用户要求"留两份"）：
  · models/universal_robots_ur10e/dual_mobile_ur10e.xml  → dual_mobile_ur10_rg2.xml（整车）
  · models/universal_robots_ur10e/ur10e.xml              → ur10_rg2.xml（单臂，给 IK 用）

两处必须同时换：IK 用的单臂模型如果还留在 2F-85，夹爪会比真模型短 34 mm，
机械臂会把腕部抬高 34 mm，RG2 的指垫正好从杆下面穿过去——实测就是这么失败的。

用法：
    D:\\Python313\\python.exe scripts\\build_rg2_model.py
"""
from __future__ import annotations

import pathlib
import re

ROOT = pathlib.Path(__file__).absolute().parent.parent  # 不要 resolve()：junction 会还原成中文路径
MODEL_DIR = ROOT / "models" / "universal_robots_ur10e"
RG2_DIR = ROOT / "models" / "onrobot_rg2"

JOBS = [
    ("dual_mobile_ur10e.xml", "dual_mobile_ur10_rg2.xml"),
    ("ur10e.xml", "ur10_rg2.xml"),
]

# UR 官方 UR10 各连杆质量（ros-industrial/universal_robot, config/ur10/physical_parameters.yaml）
MASS_UR10 = {
    "base": (4.0, 4.0),
    "shoulder_link": (7.778, 7.1),
    "upper_arm_link": (12.93, 12.7),
    "forearm_link": (3.87, 4.27),
    "wrist_1_link": (1.96, 2.0),
    "wrist_2_link": (1.96, 2.0),
    "wrist_3_link": (0.202, 0.365),
}

FINGER_ACT = (
    '    <general name="fingers_actuator" joint="rg2_jaw_pos_joint" ctrlrange="0.0005 0.055"\n'
    '             forcerange="-40 40" gainprm="24000 0 0" biasprm="0 -24000 -20"\n'
    '             biastype="affine" />\n'
)

RG2_EQ = """<equality>
    <joint joint1="rg2_jaw_pos_joint" joint2="rg2_jaw_neg_joint" polycoef="0 1 0 0 0" solimp="0.99 0.999 0.0001" solref="0.001 1" />
  </equality>"""


def cut_block(text: str, start_pat: str, tag: str) -> tuple[int, int]:
    """从起始标签开始，返回整块（含结束标签）的 [start, end) 下标。"""
    m = re.search(start_pat, text)
    if not m:
        raise SystemExit(f"找不到起始标记：{start_pat}")
    start = m.start()
    depth = 0
    i = start
    open_re = re.compile(rf"<{tag}\b")
    close_re = re.compile(rf"</{tag}>")
    while i < len(text):
        o = open_re.search(text, i)
        c = close_re.search(text, i)
        if c and (not o or c.start() < o.start()):
            depth -= 1
            i = c.end()
            if depth == 0:
                return start, i
        elif o:
            depth += 1
            i = o.end()
        else:
            break
    raise SystemExit(f"{tag} 块没有闭合")


def convert(src: str, subtree: str) -> tuple[str, list[str]]:
    log: list[str] = []

    # 1) 夹爪子树整段替换
    s, e = cut_block(src, r'<body name="robotiq_2f85_mount"', "body")
    src = src[:s] + subtree.strip() + src[e:]

    # 2) 2F-85 的 split tendon 随子树消失
    if "<tendon>" in src:
        s, e = cut_block(src, r"<tendon>", "tendon")
        src = src[:s] + src[e:]

    # 3) equality：换成两指镜像联动
    s, e = cut_block(src, r"<equality>", "equality")
    src = src[:s] + RG2_EQ + src[e:]

    # 4) 夹爪执行器
    if "<actuator>" not in src:
        raise SystemExit("母本里没有 <actuator>")
    src = src.replace("<actuator>\n", "<actuator>\n" + FINGER_ACT, 1)

    # 5) contact/exclude：剔除引用了已删除 body 的条目
    bodies = set(re.findall(r'<body\s+name="([^"]+)"', src))
    removed: list[str] = []

    def _drop(m: re.Match[str]) -> str:
        b1 = re.search(r'body1="([^"]+)"', m.group(1))
        b2 = re.search(r'body2="([^"]+)"', m.group(1))
        if b1 and b2 and (b1.group(1) not in bodies or b2.group(1) not in bodies):
            removed.append(f"{b1.group(1)}<->{b2.group(1)}")
            return ""
        return m.group(0)

    src = re.sub(r"<exclude\s+([^>]*?)/>", _drop, src)
    log += [f"剔除失效自碰撞排除: {r}" for r in removed]

    # 6) UR10 质量（惯量按质量比等比例缩放，见 docs）
    for body, (m_e, m_10) in MASS_UR10.items():
        pat = re.compile(rf'(<body name="{body}"[^>]*>\s*)<inertial ([^/]*?)/>')
        m = pat.search(src)
        if not m:
            log.append(f"{body}: 未找到 inertial，跳过")
            continue
        attrs = re.sub(r'mass="[^"]*"', f'mass="{m_10}"', m.group(2))
        ratio = m_10 / m_e
        diag = re.search(r'diaginertia="([^"]*)"', attrs)
        if diag:
            scaled = " ".join(f"{float(v) * ratio:.9g}" for v in diag.group(1).split())
            attrs = attrs.replace(diag.group(1), scaled)
        src = src[: m.start(2)] + attrs + src[m.end(2):]
        log.append(f"{body}: {m_e} -> {m_10} kg（惯量 ×{ratio:.3f}）")

    # 7) 资源：RG2 本体视觉网格。与 UR10e 的网格同目录、同 meshdir，由 fetch_models.py 下载。
    #    （子树的 <body> 片段里不能写 <asset>，所以在这里注入。）
    src, n = re.subn(
        r"(<asset>\s*\n)",
        r'\1    <mesh name="rg2_body_mesh" file="rg2_body.stl" />\n',
        src,
        count=1,
    )
    if n != 1:
        raise SystemExit("母本里没有 <asset>，无法注入 RG2 网格")
    # 8) 实车安装布置（2026-11-23 老师给的两批尺寸，单位 m；x 前、y 左、z 上，
    #    原点在车体几何中心地面）。**含前后防撞梁全长 912**，前后一条链闭合：
    #      防撞梁前端 0.456 → 导轨前端 0.350（前 106）→ 平板前端 0.306（再 44）
    #      → 平板长 180，中心 **0.216** → 平板后端 0.126 → 控制室前端 0.008（再 118）
    #      → 控制室长 420 → 后端 −0.412 → 车尾防撞梁 −0.456（再 44）。
    #    导轨长 700 → x ∈ [−0.350, +0.350]，中心间距 260、宽 40、高 36；后段被控制室盖住，
    #    模型里只画露出的 [0.008, 0.350]（视觉件，质量仍按全长计）。
    #    平板 180(前后)×300(左右)×20，顶面 z=0.45956；控制室 420×460×272、25 kg。
    #    平板与导轨质量按铝 2700 kg/m³ 估算（导轨 2.72 kg ×2、平板 2.92 kg）；控制室用 25 kg。
    pedestal = (
        '      <geom name="car1_rail_l" type="box" pos="0.179 0.13 0.42156"\n'
        '            size="0.171 0.02 0.018" mass="2.72" contype="0" conaffinity="0"\n'
        '            group="2" rgba="0.55 0.57 0.60 1" />\n'
        '      <geom name="car1_rail_r" type="box" pos="0.179 -0.13 0.42156"\n'
        '            size="0.171 0.02 0.018" mass="2.72" contype="0" conaffinity="0"\n'
        '            group="2" rgba="0.55 0.57 0.60 1" />\n'
        '      <geom name="car1_arm_plate" type="box" pos="0.216 0 0.44956"\n'
        '            size="0.09 0.15 0.01" mass="2.92" contype="0" conaffinity="0"\n'
        '            group="2" rgba="0.72 0.74 0.78 1" />\n'
        '      <geom name="car1_control_box" type="box" pos="-0.202 0 0.53956"\n'
        '            size="0.21 0.23 0.136" mass="25" contype="0" conaffinity="0"\n'
        '            group="2" rgba="0.82 0.82 0.80 1" />\n'
    )
    src, n = re.subn(
        r'\s*<geom name="arm_pedestal"[^>]*?/>\s*', "\n" + pedestal, src, count=1
    )
    log.append(
        "安装件：两导轨 + 平板 + 控制室（25 kg）" if n else "安装件：母本无 arm_pedestal，跳过"
    )

    # 臂座移到平板上表面中心 x=0.216（平板顶面 z = 0.40356 + 0.036 + 0.020 = 0.45956）
    old_base = '<body name="base" pos="0 0 0.40356" quat="0 0 0 -1" childclass="ur10e">'
    if old_base in src:
        src = src.replace(
            old_base,
            '<body name="base" pos="0.216 0 0.45956" quat="0 0 0 -1" childclass="ur10e">',
        )
        log.append("臂座移到 (0.216, 0, 0.45956)（平板中心）")

    # 9) 防撞梁长度：实车"含前后防撞梁全长 912"，而母本的圆柱中心在 ±0.452、半径 0.032
    #    → 外沿 ±0.484 = 968 mm，比实车长 56 mm。收进 ±0.456 即中心 ±0.424。
    for old, new in (
        ('pos="0.452 0 0.215"', 'pos="0.424 0 0.215"'),
        ('pos="-0.452 0 0.215"', 'pos="-0.424 0 0.215"'),
    ):
        if old in src:
            src = src.replace(old, new)
    log.append("防撞梁收到 ±0.456（含梁全长 912）")
    return src, log


def main() -> None:
    subtree = (RG2_DIR / "rg2_subtree.xml").read_text(encoding="utf-8")

    for src_name, dst_name in JOBS:
        src, log = convert((MODEL_DIR / src_name).read_text(encoding="utf-8"), subtree)
        # newline="\n"：项目约定 LF，Windows 上 write_text 默认会写 CRLF
        (MODEL_DIR / dst_name).write_text(src, encoding="utf-8", newline="\n")
        print(f"已生成 {dst_name}")
        for line in log:
            print("  " + line)


if __name__ == "__main__":
    main()
