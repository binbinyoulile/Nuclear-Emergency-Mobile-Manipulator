# 二指夹爪状态反馈。
std_msgs/Header header

float64 position   # 实测半开度，m（与 GripperCommand.position 同量纲）
float64 effort     # 实测夹持力，N
bool    moving     # 是否仍在动作中
