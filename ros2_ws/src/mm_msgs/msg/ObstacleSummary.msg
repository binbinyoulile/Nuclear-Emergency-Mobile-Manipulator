# 雷达派生量摘要（10 Hz）。
# 字段与 LidarScanner 的 7 个距离量 + 1 个阻塞标志一一对应，数值定义不变，
# 因此下游避障判据不需要重新标定。原始点云不走这个话题。
std_msgs/Header header

bool    front_blocked          # 前向警戒区内是否有障碍
float64 front_min              # 前向最近回波距离，m
float64 front_lateral          # 前向通道的横向净空，m
float64 front_lateral_signed   # 横向净空带符号值（左正右负），m
float64 left_min               # 左侧最近回波距离，m
float64 right_min              # 右侧最近回波距离，m
float64 rear_min               # 正后方最近回波距离，m
float64 rear_clear             # 整个后半平面最近回波距离，m
