# 单台底盘的轮速与转向指令。
# 与 dual_car_pick_place.py 中 ChassisInterface.set_wheels() 语义完全一致：
# 上层算好左右侧轮速与前轮转角，驱动侧只负责写执行器 / 关速度环。
float64 left    # 左侧轮目标角速度，rad/s，正数前进
float64 right   # 右侧轮目标角速度，rad/s，正数前进
float64 steer   # 前轮目标转角，rad，正数左转
