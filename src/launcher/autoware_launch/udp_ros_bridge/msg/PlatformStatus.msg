# 平台状态消息
std_msgs/Header header
uint32 msg_id
uint64 timestamp
uint8 msg_from

uint8 ctrl_mode
uint8 run_mode
uint8 estop_state
uint8 warn_level
uint8 platform_state
uint8 parking_state
uint16 power_max
uint8 speed_max
uint16 speed_current
int32 current_lon
int32 current_lat
int32 current_high
int16 steer_angle
uint8 steer_mode
uint8 steer_faultcode
uint8 steer_current
uint8 brake_pressure
uint8 brake_status
uint8 brake_fault_level

uint8 ucmfault_lf
uint8 ucmfault_lb
uint8 ucmfault_rf
uint8 ucmfault_rb

uint8 ucmfaultcode_lf
uint8 ucmfaultcode_lb
uint8 ucmfaultcode_rf
uint8 ucmfaultcode_rb

uint8 ucmctemp_lf
uint8 ucmctemp_lb
uint8 ucmctemp_rf
uint8 ucmctemp_rb

#uint16 check_sum

