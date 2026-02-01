# 单关节状态信息 Joint.msg

# 通用消息头,包含时间戳和frame_id
std_msgs/Header header

# 基础关节信息
string name
float64 position
float64 velocity
float64 effort

# 控制模式
string mode