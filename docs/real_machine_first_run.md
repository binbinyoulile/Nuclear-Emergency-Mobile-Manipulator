# 真机首跑操作卡（一页，照着做）

> 这份只写「人要做什么、看到什么算对、出问题看哪」。协议与排查细节在
> `docs/can_driver.md`，各项公差在 `docs/real_machine_calibration.md`。
> 一条命令的自动化版本是 `scripts/bench_real.py`。

## 0. 车还没到（现在就能做）

```bash
# 在 Ubuntu 虚拟机上跑一次，确认脚本本身没问题（不需要车、不需要 CAN）
python3 scripts/bench_real.py --backend loopback
```

应该看到 `4 PASS / 0 FAIL / 4 SKIP`，SKIP 的是"CAN 接口"与两个传感器话题——正常。

**这一步要一次性打通的三件事**（需要 sudo 密码，跑一次就好）：

```bash
echo "qq ALL=(ALL) NOPASSWD: /usr/bin/python3, /usr/sbin/ip, /usr/bin/apt-get, /usr/sbin/modprobe" \
  | sudo tee /etc/sudoers.d/dsh-can >/dev/null \
  && sudo chmod 440 /etc/sudoers.d/dsh-can \
  && sudo apt-get install -y can-utils \
  && sudo ip link show >/dev/null && echo PREP_OK
```

看到 `PREP_OK` 就说明：免密 sudo 配好了、`candump`/`cansend` 装好了、`ip` 能用。
**这一步做完之后，跑自检、看反馈帧都可以远程代做，不用你每次输密码。**

## 1. 到场前，先确认四件事

| 项 | 怎么确认 | 不对会怎样 |
|---|---|---|
| 四个轮子**全部架空** | 垫砖或吊起，手推轮子能自由转 | 没架空给速度会直接窜出去 |
| 两条**急停**在手边 | 按一下确认能断电，再复位 | 唯一能立刻停住车的东西 |
| 电池 **> 45 V** | 看面板或万用表 | 低于 45 V 厂家禁止行驶 |
| **两人在场**、平地 | 一人操作一人盯车 | 电机无制动器，坡道会溜车 |

## 2. 接线

1. USB-CAN 适配器插到**跑虚拟机的那台电脑**的 USB 口；
2. VMware 菜单 →「虚拟机」→「可移除设备」→ 找到适配器 →「连接」；
3. CAN 线两端各 120 Ω 终端电阻（厂家文档没逐条写，这是 CAN 常规要求）；
4. 在虚拟机里确认设备出现：

```bash
ip -br link | grep can        # 应该出现 can0
```

**看到 `can0` 就说明虚拟机拿到了设备**；没有就是 USB 直通那一步没做对。

## 3. 拉起 CAN 总线

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0 | head -3
```

应该看到 `state UP` 和 `bitrate 1000000`。

## 4. 跑自检（这一步之后可以交给我远程做）

```bash
cd ~/mm_repo
sudo python3 scripts/bench_real.py --backend socketcan --can can0
```

第一次跑要**手动输入一次 `yes`**（脚本会问：轮子架空了吗、周围没人吗、急停在手吗）。
这个确认必须由现场的人按——确认完就可以把控制权交给远程。

八项结果的含义：

| 项 | PASS 意味着 | FAIL 先查什么 |
|---|---|---|
| 1 CAN 接口 | 接口 UP、1000 kbps | `ip link`；适配器是否直通进 VM |
| 2 启动握手 | 四个驱动器都应答了使能与转速模式 | CAN 接线、驱动器供电、ID 是否 `0x101~0x104` |
| 3 反馈频率 | 每轮 ≥ 20 Hz | 掉太多说明 CPU 忙，不影响安全 |
| 4 四轮方向 | 四个编码器同向且都在转 | 反向 → 电机相序；不动 → 机械卡死/编码器 |
| 5 起转门限 | （记录项，看几路起转） | 某路总不起转 → 该轮机械阻力过大 |
| 6 里程计 | 直行距离吻合 ≤ 0.5% | 偏差大 → 第 5 步做轮径校准 |
| 7 雷达 `/scan` | 话题在发数据 | 驱动没启动时 SKIP 属正常 |
| 8 机械臂 `/joint_states` | 同上 | UR 驱动没接时 SKIP 属正常 |

结果同时写进 `reports/bench_real_<时间>.csv`，**留着**，下次改完东西再跑一遍对比。

## 5. 如果里程计那项 FAIL（约 15 分钟）

1. 轮子**落地**，找一条 2 米以上的直线；
2. 慢速直行一段，用卷尺量车实际走过的距离（比如 1.982 m）；
3. 带上量到的数重跑：

```bash
sudo python3 scripts/bench_real.py --backend socketcan --can can0 --measured 1.982
```

偏差仍在 0.5% 以上，就按厂家说明书调 `linear_correction_factor`（步骤见
`docs/can_driver.md` 第六节）；角度偏差调 `wheel_track_4wd`。

## 6. 收尾（顺序别反）

```bash
sudo ip link set can0 down
# 然后：关驱动器电源 → 关总开关（先软关机再断电，见说明书）
```

脚本结束时已经自动发过零速与失能；但**断电前确认四个轮子都静止**。

## 红线（任何一条都不要试）

- 没架空就给速度（哪怕 0.1 m/s）；
- 一个人在坡道上操作（电机无制动器，会溜）；
- 电池低于 45 V 还行驶；
- 把急停放在够不到的地方；
- 上电顺序颠倒（正确：总开关 → 急停复位 → 长按启动 3 s）。

## 分工（谁做什么）

| 事 | 谁 |
|---|---|
| 架空、接线、上电、按那一次 `yes`、物理调整（换线序等） | **现场的人** |
| 跑自检、读结果、判断 FAIL 原因、换参数重跑、给结论 | **可以远程代做**（虚拟机 SSH 可达即可） |
| 拆车、动电柜、改厂家固件 | **不要做**（超出范围，问老师） |
