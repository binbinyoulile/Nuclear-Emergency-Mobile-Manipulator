# 感知节点输出的目标位姿估计（世界坐标系 map）。
# 替代仿真里的 TruthOnly 字段：真机侧 valid=false 时下游必须降级。
std_msgs/Header header

geometry_msgs/Point position   # 目标中心，map 系，m
float64 confidence             # 置信度，0…1
bool    valid                  # 本次估计是否可用
