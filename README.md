# 双车协同核废料回收仿真 v2.0

## 文件职责

- `dual_car_pick_place.py`：主入口；包含双车任务状态机、雷达避障、D435i 视觉、抓取投放和返航。
- `mobile_pick_place.py`：共享底层库；提供底盘/机械臂 IK、夹爪、仿真步进和基础运动控制。主入口会导入它，不能删除。
- `dual_car_scene.xml`：当前 MuJoCo 主场景，包含双车、UR10e、2F-85、D435i、桶、障碍物和场地标线。
- `arm_ik_scene.xml`：机械臂 IK 辅助场景；共享底层库加载，不能删除。
- `models/`：本地 MuJoCo 模型与 OBJ/STL 资产；体积较大，已由 `.gitignore` 排除。
- `docs/screenshots/`：当前版本的总览、避障、抓取、投放、视觉 RGB/掩码/深度截图。

## 运行

```powershell
D:\Python313\python.exe D:\robot_mobile\dual_car_pick_place.py --seed 7
```

无窗口完整验收并生成截图：

```powershell
D:\Python313\python.exe D:\robot_mobile\dual_car_pick_place.py --snapshot --seed 7
```

项目直接使用全局 Python，不创建或激活虚拟环境。
