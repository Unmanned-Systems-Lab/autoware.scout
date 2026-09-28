#ifndef UDP_ROS_BRIDGE__UDP_BRIDGE_NODE_HPP_
#define UDP_ROS_BRIDGE__UDP_BRIDGE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "protocol_defines.hpp"
#include "udp_ros_bridge/msg/traj_cmd.hpp"
#include "udp_ros_bridge/msg/remote_cmd.hpp"
#include "udp_ros_bridge/msg/auto_cmd.hpp"
#include "udp_ros_bridge/msg/platform_status.hpp"
#include "std_msgs/msg/string.hpp" // 添加std_msgs头文件

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <cstdint>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

namespace udp_ros_bridge
{

// 电机位置枚举
enum MotorPosition {
  MOTOR_POSITION_LF = 0,  // 左前
  MOTOR_POSITION_LB = 1,  // 左后
  MOTOR_POSITION_RF = 2,  // 右前
  MOTOR_POSITION_RB = 3   // 右后
};

class UdpRosBridgeNode : public rclcpp::Node
{
public:
  explicit UdpRosBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UdpRosBridgeNode() override;

private:
  bool initUdpSockets();
  bool initCanSocket();  // 新增CAN套接字初始化
  void udpRecvThread();
  void canRecvThread();  // 新增CAN接收线程
  void networkHealthMonitor(); // 网络健康监控线程
  void parseCanFrame(const struct can_frame & frame);  // 解析CAN帧
  void parseMotorControllerStatus(const struct can_frame & frame, MotorPosition position);  // 解析电机控制器状态
  void parseMotorControllerFault(const struct can_frame & frame, MotorPosition position);  // 解析电机控制器故障
  void parseVehicleStatus(const struct can_frame & frame);  // 解析整车状态
  void parseBrakeSystemStatus(const struct can_frame & frame);  // 解析制动系统状态
  void parseSteeringAngleSensor(const struct can_frame & frame);  // 解析转向角度传感器
  void sendRemoteCmdToCan(const udp_ros_bridge::msg::RemoteCmd & msg);  // 新增发送到CAN的功能
  bool parseTrajCmd(const uint8_t * buf, size_t len, udp_ros_bridge::msg::TrajCmd & msg);
  bool parseRemoteCmd(const uint8_t * buf, size_t len, udp_ros_bridge::msg::RemoteCmd & msg);
  bool parseAutoCmd(const uint8_t * buf, size_t len, udp_ros_bridge::msg::AutoCmd & msg);
  bool packPlatformStatus(const udp_ros_bridge::msg::PlatformStatus & msg, uint8_t * buf, size_t & len);
  uint16_t calculateCheckSum(const uint8_t * buf, size_t len);
  uint64_t parseTimestamp(const uint8_t * ts_buf);
  void packTimestamp(uint64_t timestamp, uint8_t * ts_buf);
  void platformStatusCb(const udp_ros_bridge::msg::PlatformStatus::SharedPtr msg);

  rclcpp::Publisher<udp_ros_bridge::msg::TrajCmd>::SharedPtr traj_cmd_pub_;
  rclcpp::Publisher<udp_ros_bridge::msg::RemoteCmd>::SharedPtr remote_cmd_pub_;
  rclcpp::Publisher<udp_ros_bridge::msg::AutoCmd>::SharedPtr auto_cmd_pub_;
  rclcpp::Subscription<udp_ros_bridge::msg::PlatformStatus>::SharedPtr platform_status_sub_;
  
  // Publisher for network health status
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr network_status_pub_;

  int sock_recv_;
  int sock_send_;
  int can_sock_;  // 新增CAN套接字
  sockaddr_in remote_addr_;
  std::thread recv_thread_;
  std::thread can_recv_thread_;  // 新增CAN接收线程
  std::thread health_monitor_thread_; // 网络健康监控线程
  std::atomic<bool> exit_thread_;

  // Network health tracking
  std::atomic<int> udp_send_failures_{0};
  std::atomic<int> udp_recv_count_{0};
  std::atomic<int> ros_publish_failures_{0};
  std::chrono::steady_clock::time_point last_health_check_time_;
  
  // CAN总线接收的状态变量
  uint8_t can_motor_fault_lf_{0};      // 左前电机故障状态
  uint8_t can_motor_fault_lb_{0};      // 左后电机故障状态
  uint8_t can_motor_fault_rf_{0};      // 右前电机故障状态
  uint8_t can_motor_fault_rb_{0};      // 右后电机故障状态

  uint8_t can_motor_fault_code_lf_{0}; // 左前电机故障码
  uint8_t can_motor_fault_code_lb_{0}; // 左后电机故障码
  uint8_t can_motor_fault_code_rf_{0}; // 右前电机故障码
  uint8_t can_motor_fault_code_rb_{0}; // 右后电机故障码

  uint8_t can_motor_temp_lf_{0};       // 左前电机温度
  uint8_t can_motor_temp_lb_{0};       // 左后电机温度
  uint8_t can_motor_temp_rf_{0};       // 右前电机温度
  uint8_t can_motor_temp_rb_{0};       // 右后电机温度

  uint8_t can_motor_current_lf_{0};    // 左前电机电流
  uint8_t can_motor_current_lb_{0};    // 左后电机电流
  uint8_t can_motor_current_rf_{0};    // 右前电机电流
  uint8_t can_motor_current_rb_{0};    // 右后电机电流

  uint16_t can_motor_speed_lf_{0};     // 左前电机转速
  uint16_t can_motor_speed_lb_{0};     // 左后电机转速
  uint16_t can_motor_speed_rf_{0};     // 右前电机转速
  uint16_t can_motor_speed_rb_{0};     // 右后电机转速

  uint16_t can_vehicle_speed_{0};      // 车辆速度
  uint8_t can_vehicle_brake_pressure_{0}; // 制动压力
  uint8_t can_vehicle_brake_status_{0};   // 制动状态
  uint8_t can_vehicle_gear_{0};           // 档位
  uint8_t can_vehicle_park_state_{0};     // 驻车状态
  uint8_t can_vehicle_estop_state_{0};    // 急停状态
  uint8_t can_vehicle_ctrl_mode_{0};      // 控制模式
  uint8_t can_vehicle_run_mode_{0};       // 运行模式
  uint8_t can_vehicle_warn_level_{0};     // 报警等级
  uint8_t can_vehicle_platform_state_{0}; // 平台状态
  uint16_t can_vehicle_power_max_{0};     // 最大功率

  uint8_t can_brake_pressure_{0};      // 制动压力（制动系统状态报文）
  uint8_t can_brake_status_{0};        // 制动状态
  uint8_t can_brake_fault_level_{0};   // 制动故障等级

  int16_t can_steering_angle_{0};      // 转向角度
  uint8_t can_steering_mode_{0};       // 转向模式
  uint8_t can_steering_fault_code_{0}; // 转向故障码
  uint8_t can_steering_current_{0};    // 转向电流

  uint64_t parseTimestampFromUint32(uint32_t timestamp_raw);
  void printHexData(const uint8_t *buf, size_t len);
  
  // 待修订20260326
  void remoteCmdCallback(const udp_ros_bridge::msg::RemoteCmd::SharedPtr msg);

};

} // namespace udp_ros_bridge

#endif  // UDP_ROS_BRIDGE__UDP_BRIDGE_NODE_HPP_