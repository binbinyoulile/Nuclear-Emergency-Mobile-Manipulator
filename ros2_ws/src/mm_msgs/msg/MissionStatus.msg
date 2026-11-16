# 任务状态心跳（1 Hz），供 rviz / 日志 / 回归脚本观察。
string  stage      # 当前阶段名
bool    done       # 任务是否结束
bool    ok         # 结束时是否判定成功
float64 sim_time   # 对应仿真时刻，s
