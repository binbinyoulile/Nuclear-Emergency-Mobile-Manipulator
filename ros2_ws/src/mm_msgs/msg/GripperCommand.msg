# 二指夹爪指令。开合值是半开度（米，越大越开），与 mission_core.FINGER_OPEN/FINGER_CLOSE 同量纲；
# 真机驱动负责换算成具体夹爪的指令（OnRobot RG2 是宽度 + 力）。
float64 position     # 半开度，m：当前 RG2 为 0.0005（全闭）… 0.055（全开）
float64 max_effort   # 期望夹持力，N
