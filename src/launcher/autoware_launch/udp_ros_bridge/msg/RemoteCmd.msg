# 遥控指令消息
std_msgs/Header header
uint32 msg_id
uint32 data_len
uint64 timestamp
uint8 mode          # 0-无效,1-遥控,2-循迹
uint8 throttle      # 0~100
uint8 brake         # 0~100
int8 steer          # -100~100
uint8 gear          # 0-无效,1-前进,2-倒档,3-中心转向
uint8 parking       # 0-驻车,1-解除
uint8 estop         # 0-不使能,1-使能
uint8 speed_limit   # 0~50km/h
uint8 heartbeat     # 0~255

