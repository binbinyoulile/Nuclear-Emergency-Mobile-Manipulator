# 二指夹爪状态反馈。
std_msgs/Header header

float64 position   # 实测开合值，0…255
float64 effort     # 实测夹持力，N
bool    moving     # 是否仍在动作中
