#ifndef UDP_ROS_BRIDGE__CONFIG_MANAGER_HPP_
#define UDP_ROS_BRIDGE__CONFIG_MANAGER_HPP_

#include <string>

namespace udp_ros_bridge {

struct NetworkConfig {
  std::string local_ip;
  std::string remote_ip;
  int udp_port_cmd;
  int udp_port_status;
  int udp_buf_size;
};

struct MessageIds {
  uint32_t msg_id_auto;
  uint32_t msg_id_traj;
  uint32_t msg_id_remote;
  uint32_t msg_id_status;
};

struct ProtocolParams {
  int max_traj_points;
  int time_stamp_len;
  int check_sum_len;
};

struct CanConfig {
  int can_id_base;
};

class ConfigManager {
public:
  static ConfigManager& getInstance();

  bool loadConfig(const std::string & config_file_path = "");

  // Getter methods
  const NetworkConfig & getNetworkConfig() const { return network_config_; }
  const MessageIds & getMessageIds() const { return message_ids_; }
  const ProtocolParams & getProtocolParams() const { return protocol_params_; }
  const CanConfig & getCanConfig() const { return can_config_; }

private:
  ConfigManager();  // Private constructor
  NetworkConfig network_config_;
  MessageIds message_ids_;
  ProtocolParams protocol_params_;
  CanConfig can_config_;
};

}  // namespace udp_ros_bridge

#endif  // UDP_ROS_BRIDGE__CONFIG_MANAGER_HPP_