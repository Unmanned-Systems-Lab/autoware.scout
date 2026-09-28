#include "udp_ros_bridge/config_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace udp_ros_bridge {

ConfigManager::ConfigManager()
{
  // 设置默认值
  network_config_.local_ip = "192.168.1.102";
  network_config_.remote_ip = "192.168.1.115";
  network_config_.udp_port_cmd = 60001;
  network_config_.udp_port_status = 60001;
  network_config_.udp_buf_size = 1024;

  message_ids_.msg_id_auto = 0x00010001;
  message_ids_.msg_id_traj = 0x00010002;
  message_ids_.msg_id_remote = 0x00010003;
  message_ids_.msg_id_status = 0x00020001;

  protocol_params_.max_traj_points = 200;
  protocol_params_.time_stamp_len = 8;
  protocol_params_.check_sum_len = 2;

  can_config_.can_id_base = 0x105;
}

ConfigManager & ConfigManager::getInstance()
{
  static ConfigManager instance;
  return instance;
}

bool ConfigManager::loadConfig(const std::string & config_file_path)
{
  std::string config_path = config_file_path;
  
  // 如果没有指定配置文件路径，则尝试多个可能的位置
  if (config_path.empty()) {
    // 尝试当前工作目录
    config_path = "./config.yaml";
    
    // 检查是否存在于当前目录
    if (!std::filesystem::exists(config_path)) {
      // 尝试在包的标准位置查找
      const char* ros_install_prefix = std::getenv("AMENT_PREFIX_PATH");
      if (ros_install_prefix) {
        std::string install_config = std::string(ros_install_prefix) + "/share/udp_ros_bridge/config/config.yaml";
        if (std::filesystem::exists(install_config)) {
          config_path = install_config;
        } else {
          // 回退到包源码目录
          config_path = "/home/xmt/udp_ros_bridge/src/udp_ros_bridge/config.yaml";
        }
      } else {
        // 如果没有环境变量，回退到包源码目录
        config_path = "/home/xmt/udp_ros_bridge/src/udp_ros_bridge/config.yaml";
      }
    }
  }

  try {
    YAML::Node config = YAML::LoadFile(config_path);

    // Load network config
    if (config["network"]) {
      if (config["network"]["local_ip"]) {
        network_config_.local_ip = config["network"]["local_ip"].as<std::string>();
      }
      if (config["network"]["remote_ip"]) {
        network_config_.remote_ip = config["network"]["remote_ip"].as<std::string>();
      }
      if (config["network"]["udp_port_cmd"]) {
        network_config_.udp_port_cmd = config["network"]["udp_port_cmd"].as<int>();
      }
      if (config["network"]["udp_port_status"]) {
        network_config_.udp_port_status = config["network"]["udp_port_status"].as<int>();
      }
      if (config["network"]["udp_buf_size"]) {
        network_config_.udp_buf_size = config["network"]["udp_buf_size"].as<int>();
      }
    }

    // Load message IDs
    if (config["message_ids"]) {
      if (config["message_ids"]["msg_id_auto"]) {
        message_ids_.msg_id_auto = config["message_ids"]["msg_id_auto"].as<uint32_t>();
      }
      if (config["message_ids"]["msg_id_traj"]) {
        message_ids_.msg_id_traj = config["message_ids"]["msg_id_traj"].as<uint32_t>();
      }
      if (config["message_ids"]["msg_id_remote"]) {
        message_ids_.msg_id_remote = config["message_ids"]["msg_id_remote"].as<uint32_t>();
      }
      if (config["message_ids"]["msg_id_status"]) {
        message_ids_.msg_id_status = config["message_ids"]["msg_id_status"].as<uint32_t>();
      }
    }

    // Load protocol params
    if (config["protocol_params"]) {
      if (config["protocol_params"]["max_traj_points"]) {
        protocol_params_.max_traj_points = config["protocol_params"]["max_traj_points"].as<int>();
      }
      if (config["protocol_params"]["time_stamp_len"]) {
        protocol_params_.time_stamp_len = config["protocol_params"]["time_stamp_len"].as<int>();
      }
      if (config["protocol_params"]["check_sum_len"]) {
        protocol_params_.check_sum_len = config["protocol_params"]["check_sum_len"].as<int>();
      }
    }

    // Load CAN config
    if (config["can_config"]) {
      if (config["can_config"]["can_id_base"]) {
        can_config_.can_id_base = config["can_config"]["can_id_base"].as<int>();
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("ConfigManager"), "Configuration loaded successfully from %s", config_path.c_str());
    return true;
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ConfigManager"), "Failed to load config file: %s", e.what());
    return false;
  }
}

}  // namespace udp_ros_bridge