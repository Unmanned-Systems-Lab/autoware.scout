#include "udp_ros_bridge/udp_bridge_node.hpp"
#include "udp_ros_bridge/config_manager.hpp" // 添加配置管理器头文件
#include <cmath>
#include <stdexcept>
#include <chrono>   // 添加chrono库支持高精度时间
#include <vector>   // 用于替代变长数组
#include <sstream>  // 添加sstream for stringstream
#include <iostream> //

namespace udp_ros_bridge
{

  UdpRosBridgeNode::UdpRosBridgeNode(const rclcpp::NodeOptions &options)
      : Node("udp_ros_bridge_node", options), exit_thread_(false)
  {
    // 加载配置
    ConfigManager &config_manager = ConfigManager::getInstance();
    config_manager.loadConfig(); // 使用默认配置文件路径

    // 显式忽略未使用的变量以避免警告
    (void)(config_manager);
    // const NetworkConfig & net_config = config_manager.getNetworkConfig();
    // const MessageIds & msg_ids = config_manager.getMessageIds();

    traj_cmd_pub_ = this->create_publisher<udp_ros_bridge::msg::TrajCmd>("/udp/traj_cmd", 10);
    remote_cmd_pub_ = this->create_publisher<udp_ros_bridge::msg::RemoteCmd>("/udp/remote_cmd", 10);
    auto_cmd_pub_ = this->create_publisher<udp_ros_bridge::msg::AutoCmd>("/udp/auto_cmd", 10);
    platform_status_sub_ = this->create_subscription<udp_ros_bridge::msg::PlatformStatus>(
        "/udp/platform_status", 10,
        std::bind(&UdpRosBridgeNode::platformStatusCb, this, std::placeholders::_1));

    // Initialize network status publisher
    network_status_pub_ = this->create_publisher<std_msgs::msg::String>("/udp/network_status", 10);

    if (!initUdpSockets())
    {
      RCLCPP_FATAL(this->get_logger(), "UDP sockets init failed!");
      rclcpp::shutdown();
      return;
    }

    // if (!initCanSocket())
    // {
    //   RCLCPP_ERROR(this->get_logger(), "CAN socket init failed, continuing without CAN support");
    // }

    // 启动CAN接收线程
    // can_recv_thread_ = std::thread(&UdpRosBridgeNode::canRecvThread, this);

    recv_thread_ = std::thread(&UdpRosBridgeNode::udpRecvThread, this);
    health_monitor_thread_ = std::thread(&UdpRosBridgeNode::networkHealthMonitor, this);

    last_health_check_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "UDP ROS Bridge Node initialized!");
  }

  UdpRosBridgeNode::~UdpRosBridgeNode()
  {
    exit_thread_ = true;
    if (recv_thread_.joinable())
    {
      recv_thread_.join();
    }
    if (health_monitor_thread_.joinable())
    {
      health_monitor_thread_.join();
    }
    // if (can_recv_thread_.joinable())
    // {
    //   can_recv_thread_.join();
    // }
    close(sock_recv_);
    close(sock_send_);
    // if (can_sock_ >= 0)
    // {
    //   close(can_sock_);
    // }
  }

  bool UdpRosBridgeNode::initUdpSockets()
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const NetworkConfig &net_config = config_manager.getNetworkConfig();

    sock_recv_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_recv_ < 0)
      return false;

    sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr(net_config.local_ip.c_str());
    local_addr.sin_port = htons(net_config.udp_port_cmd);

    int opt = 1;
    setsockopt(sock_recv_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(sock_recv_, (sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
      close(sock_recv_);
      return false;
    }

    sock_send_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_send_ < 0)
    {
      close(sock_recv_);
      return false;
    }

    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_addr.s_addr = inet_addr(net_config.remote_ip.c_str());
    remote_addr_.sin_port = htons(net_config.udp_port_status);

    return true;
  }

  bool UdpRosBridgeNode::initCanSocket()
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const CanConfig &can_config = config_manager.getCanConfig();

    // 创建CAN套接字
    can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error creating CAN socket: %s", strerror(errno));
      return false;
    }

    // 设置CAN接口
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can1"); // 使用CAN总线1
    ioctl(can_sock_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定套接字
    if (bind(can_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error binding CAN socket: %s", strerror(errno));
      close(can_sock_);
      return false;
    }

    // 设置非阻塞模式
    int flags = fcntl(can_sock_, F_GETFL, 0);
    fcntl(can_sock_, F_SETFL, flags | O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "CAN socket initialized with base ID: 0x%x", can_config.can_id_base);
    return true;
  }

  void UdpRosBridgeNode::canRecvThread()
  {
    // ConfigManager & config_manager = ConfigManager::getInstance();
    // const CanConfig & can_config = config_manager.getCanConfig();  // 未使用的变量

    while (!exit_thread_ && rclcpp::ok())
    {
      struct can_frame frame;
      ssize_t bytes_read = read(can_sock_, &frame, sizeof(struct can_frame));

      if (bytes_read < 0)
      {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
          RCLCPP_WARN(this->get_logger(), "Error reading CAN frame: %s", strerror(errno));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 避免过度占用CPU
        continue;
      }

      if (bytes_read == sizeof(struct can_frame))
      {
        // 解析CAN帧并更新平台状态
        parseCanFrame(frame);
      }
    }
  }

  void UdpRosBridgeNode::parseCanFrame(const struct can_frame &frame)
  {
    // 根据CAN ID解析不同的消息类型
    switch (frame.can_id)
    {
    case 0x185: // 电机控制器状态报文 - 左前
      parseMotorControllerStatus(frame, MOTOR_POSITION_LF);
      break;
    case 0x186: // 电机控制器状态报文 - 左后
      parseMotorControllerStatus(frame, MOTOR_POSITION_LB);
      break;
    case 0x187: // 电机控制器状态报文 - 右前
      parseMotorControllerStatus(frame, MOTOR_POSITION_RF);
      break;
    case 0x188: // 电机控制器状态报文 - 右后
      parseMotorControllerStatus(frame, MOTOR_POSITION_RB);
      break;
    case 0x285: // 电机控制器故障报文 - 左前
      parseMotorControllerFault(frame, MOTOR_POSITION_LF);
      break;
    case 0x286: // 电机控制器故障报文 - 左后
      parseMotorControllerFault(frame, MOTOR_POSITION_LB);
      break;
    case 0x287: // 电机控制器故障报文 - 右前
      parseMotorControllerFault(frame, MOTOR_POSITION_RF);
      break;
    case 0x288: // 电机控制器故障报文 - 右后
      parseMotorControllerFault(frame, MOTOR_POSITION_RB);
      break;
    case 0x385: // 整车状态报文
      parseVehicleStatus(frame);
      break;
    case 0x485: // 制动系统状态报文
      parseBrakeSystemStatus(frame);
      break;
    case 0x585: // 方向盘转角传感器报文
      parseSteeringAngleSensor(frame);
      break;
    default:
      RCLCPP_DEBUG(this->get_logger(), "Unknown CAN frame ID: 0x%x", frame.can_id);
      break;
    }
  }

  void UdpRosBridgeNode::parseMotorControllerStatus(const struct can_frame &frame, MotorPosition position)
  {
    // 解析电机控制器状态报文
    // 示例解析（实际协议可能不同）
    uint8_t temp = frame.data[0];                            // 温度
    uint16_t current = (frame.data[2] << 8) | frame.data[1]; // 电流
    uint16_t speed = (frame.data[4] << 8) | frame.data[3];   // 转速

    // 根据位置更新相应的状态变量
    switch (position)
    {
    case MOTOR_POSITION_LF:
      can_motor_temp_lf_ = temp;
      can_motor_current_lf_ = current & 0xFF;
      can_motor_speed_lf_ = speed;
      break;
    case MOTOR_POSITION_LB:
      can_motor_temp_lb_ = temp;
      can_motor_current_lb_ = current & 0xFF;
      can_motor_speed_lb_ = speed;
      break;
    case MOTOR_POSITION_RF:
      can_motor_temp_rf_ = temp;
      can_motor_current_rf_ = current & 0xFF;
      can_motor_speed_rf_ = speed;
      break;
    case MOTOR_POSITION_RB:
      can_motor_temp_rb_ = temp;
      can_motor_current_rb_ = current & 0xFF;
      can_motor_speed_rb_ = speed;
      break;
    }
  }

  void UdpRosBridgeNode::parseMotorControllerFault(const struct can_frame &frame, MotorPosition position)
  {
    // 解析电机控制器故障报文
    uint8_t fault_state = frame.data[0];
    uint8_t fault_code = frame.data[1];

    switch (position)
    {
    case MOTOR_POSITION_LF:
      can_motor_fault_lf_ = fault_state;
      can_motor_fault_code_lf_ = fault_code;
      break;
    case MOTOR_POSITION_LB:
      can_motor_fault_lb_ = fault_state;
      can_motor_fault_code_lb_ = fault_code;
      break;
    case MOTOR_POSITION_RF:
      can_motor_fault_rf_ = fault_state;
      can_motor_fault_code_rf_ = fault_code;
      break;
    case MOTOR_POSITION_RB:
      can_motor_fault_rb_ = fault_state;
      can_motor_fault_code_rb_ = fault_code;
      break;
    }
  }

  void UdpRosBridgeNode::parseVehicleStatus(const struct can_frame &frame)
  {
    // 解析整车状态报文
    can_vehicle_speed_ = (frame.data[1] << 8) | frame.data[0];     // 车辆速度
    can_vehicle_brake_pressure_ = frame.data[2];                   // 制动压力
    can_vehicle_brake_status_ = frame.data[3];                     // 制动状态
    can_vehicle_gear_ = frame.data[4] & 0x0F;                      // 档位
    can_vehicle_park_state_ = (frame.data[4] >> 4) & 0x0F;         // 驻车状态
    can_vehicle_estop_state_ = frame.data[5] & 0x01;               // 急停状态
    can_vehicle_ctrl_mode_ = (frame.data[5] >> 1) & 0x03;          // 控制模式
    can_vehicle_run_mode_ = (frame.data[5] >> 3) & 0x07;           // 运行模式
    can_vehicle_warn_level_ = frame.data[6];                       // 报警等级
    can_vehicle_power_max_ = (frame.data[8] << 8) | frame.data[7]; // 最大功率
  }

  void UdpRosBridgeNode::parseBrakeSystemStatus(const struct can_frame &frame)
  {
    // 解析制动系统状态报文
    can_brake_pressure_ = frame.data[0];
    can_brake_status_ = frame.data[1];
    can_brake_fault_level_ = frame.data[2];
  }

  void UdpRosBridgeNode::parseSteeringAngleSensor(const struct can_frame &frame)
  {
    // 解析方向盘转角传感器报文
    int16_t raw_angle = (frame.data[1] << 8) | frame.data[0];
    can_steering_angle_ = raw_angle;          // 转向角度
    can_steering_mode_ = frame.data[2];       // 转向模式
    can_steering_fault_code_ = frame.data[3]; // 转向故障码
    can_steering_current_ = frame.data[4];    // 转向电流
  }

  void UdpRosBridgeNode::udpRecvThread()
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const NetworkConfig &net_config = config_manager.getNetworkConfig();
    const MessageIds &msg_ids = config_manager.getMessageIds();

    // 使用vector替代变长数组
    std::vector<uint8_t> recv_buf(net_config.udp_buf_size);
    sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (!exit_thread_ && rclcpp::ok())
    {
      ssize_t recv_len = recvfrom(
          sock_recv_, recv_buf.data(), net_config.udp_buf_size, 0,
          (sockaddr *)&sender_addr, &sender_len);

      if (recv_len < 0)
        continue;

      std::string sender_ip = inet_ntoa(sender_addr.sin_addr);
      if (sender_ip != net_config.remote_ip)
        continue;

      if (recv_len < 4)
        continue;
      // uint32_t msg_id = ntohl(*(uint32_t *)recv_buf.data());
      uint32_t msg_id = *(uint32_t *)recv_buf.data();

      // 输出原始数据的十六进制表示
      std::cout << "recv_len: " << std::dec << std::uppercase << recv_len << std::endl;
      printHexData(recv_buf.data(), recv_len);

      if (msg_id == msg_ids.msg_id_traj)
      {
        std::cout << "Processing TrajCmd message..." << std::endl;
        udp_ros_bridge::msg::TrajCmd traj_msg;
        if (parseTrajCmd(recv_buf.data(), recv_len, traj_msg))
        {
          traj_msg.header.stamp = this->get_clock()->now();
          traj_cmd_pub_->publish(traj_msg);
        }
      }
      else if (msg_id == msg_ids.msg_id_remote)
      {
        std::cout << "Processing RemoteCmd message..." << std::endl;
        udp_ros_bridge::msg::RemoteCmd remote_msg;
        if (parseRemoteCmd(recv_buf.data(), recv_len, remote_msg))
        {
          remote_msg.header.stamp = this->get_clock()->now();
          remote_cmd_pub_->publish(remote_msg);

          // 同时转发到CAN总线
          // sendRemoteCmdToCan(remote_msg);
        }
      }
      else if (msg_id == msg_ids.msg_id_auto)
      {
        std::cout << "Processing AutoCmd message..." << std::endl;
        udp_ros_bridge::msg::AutoCmd auto_msg;
        if (parseAutoCmd(recv_buf.data(), recv_len, auto_msg))
        {
          auto_msg.header.stamp = this->get_clock()->now();
          std::cout << "AutoCmd id..." << auto_msg.msg_id << std::endl;
          auto_cmd_pub_->publish(auto_msg);
        }
      }
    }
  }

  void UdpRosBridgeNode::networkHealthMonitor()
  {
    rclcpp::Rate rate(1); // 每秒检查一次

    while (!exit_thread_ && rclcpp::ok())
    {
      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                          now - last_health_check_time_)
                          .count();

      // 检查网络状态
      std_msgs::msg::String status_msg;
      std::stringstream status_str;

      status_str << "Time: " << duration << "s | ";
      status_str << "UDP Recv: " << udp_recv_count_.load() << " | ";
      status_str << "UDP Send Failures: " << udp_send_failures_.load() << " | ";
      status_str << "ROS Publish Failures: " << ros_publish_failures_.load();

      status_msg.data = status_str.str();
      network_status_pub_->publish(status_msg);

      // 如果失败次数过多，输出警告
      if (udp_send_failures_.load() > 10)
      {
        RCLCPP_WARN(this->get_logger(),
                    "High number of UDP send failures detected: %d",
                    udp_send_failures_.load());

        // 尝试重置统计信息
        udp_send_failures_ = 0;
      }

      if (ros_publish_failures_.load() > 5)
      {
        RCLCPP_WARN(this->get_logger(),
                    "High number of ROS publish failures detected: %d",
                    ros_publish_failures_.load());

        ros_publish_failures_ = 0;
      }

      rate.sleep();
    }
  }

  void UdpRosBridgeNode::sendRemoteCmdToCan(const udp_ros_bridge::msg::RemoteCmd &msg)
  {
    if (can_sock_ < 0)
    {
      return; // 如果CAN套接字未初始化，则跳过
    }

    ConfigManager &config_manager = ConfigManager::getInstance();
    const CanConfig &can_config = config_manager.getCanConfig();

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    // 设置CAN ID，这里假设远程控制命令的ID是特定值
    frame.can_id = can_config.can_id_base; // 根据配置的CAN ID发送
    frame.can_dlc = 8;                     // CAN帧长度为8字节

    // 将RemoteCmd消息转换为CAN数据
    // 按照协议将各字段打包到CAN帧的数据部分
    if (msg.mode == 2)
    {
      frame.data[0] = (0x1 << 6U) | frame.data[0];
    }
    frame.data[0] = (msg.estop << 6U) | frame.data[0];
    if (msg.gear == 1)
    {
      frame.data[0] = (0x1 << 1U) | frame.data[0];
    }
    else if (msg.gear == 2)
    {
      frame.data[0] = (0x1 << 2U) | frame.data[0];
    }
    else if (msg.gear == 3)
    {
      frame.data[0] = (0x1 << 3U) | frame.data[0];
    }

    frame.data[1] = msg.parking;

    frame.data[2] = msg.throttle;
    frame.data[3] = msg.brake;
    // 左负右正
    if (msg.steer < 0)
    {
      frame.data[4] = -msg.steer;
    }
    else if (msg.steer > 0)
    {
      frame.data[5] = msg.steer;
    }
    frame.data[6] = msg.speed_limit;
    frame.data[7] = msg.heartbeat;

    // 发送CAN帧
    ssize_t bytes_sent = write(can_sock_, &frame, sizeof(frame));
    if (bytes_sent < 0)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Successfully sent CAN frame with %ld bytes", bytes_sent);
    }
  }

  uint16_t UdpRosBridgeNode::calculateCheckSum(const uint8_t *buf, size_t len)
  {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++)
      sum += buf[i];
    return (uint16_t)(sum & 0xFFFF);
  }

  uint64_t UdpRosBridgeNode::parseTimestamp(const uint8_t *ts_buf)
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const ProtocolParams &proto_params = config_manager.getProtocolParams();

    uint64_t ts = 0;
    memcpy(&ts, ts_buf, proto_params.time_stamp_len);
    return ts;
  }

  uint64_t UdpRosBridgeNode::parseTimestampFromUint32(uint32_t timestamp_raw)
  {
    // 将4字节的时间戳转换为uint64_t类型，乘以10^9转换为纳秒
    return static_cast<uint64_t>(timestamp_raw) * 1000000000ULL;
  }

  void UdpRosBridgeNode::packTimestamp(uint64_t timestamp, uint8_t *ts_buf)
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const ProtocolParams &proto_params = config_manager.getProtocolParams();

    memcpy(ts_buf, &timestamp, proto_params.time_stamp_len);
  }

  void UdpRosBridgeNode::printHexData(const uint8_t *buf, size_t len)
  {
    std::cout << "HEX DUMP [" << len << "]: ";
    for (size_t i = 0; i < len; ++i)
    {
      printf("%02X ", buf[i]);
    }
    std::cout << std::endl;
  }

  bool UdpRosBridgeNode::parseTrajCmd(const uint8_t *buf, size_t len, udp_ros_bridge::msg::TrajCmd &msg)
  {
    // 验证最小长度要求
    size_t min_expected_len = 4 + 4 + 8 + 1 + 1 + 2; // msg_id + data_len + timestamp + source + point_num + check_sum
    if (len < min_expected_len)
    {
      RCLCPP_WARN(this->get_logger(), "Received TrajCmd packet too short: %zu bytes, expected at least %zu", len, min_expected_len);
      return false;
    }

    size_t offset = 0;
    // msg.msg_id = ntohl(*(uint32_t *)(buf + offset));
    msg.msg_id = (*(uint32_t *)(buf + offset));
    offset += 4;
    // msg.data_len = ntohl(*(uint32_t *)(buf + offset));
    msg.data_len = (*(uint32_t *)(buf + offset));
    offset += 4;

    msg.timestamp = parseTimestamp(buf + offset);
    offset += ConfigManager::getInstance().getProtocolParams().time_stamp_len;

    msg.source = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.point_num = *(uint8_t *)(buf + offset);
    std::cout << "msg.point_num: " << (uint8_t)msg.point_num << std::endl;

    offset += 1;

    ConfigManager &config_manager = ConfigManager::getInstance();
    const ProtocolParams &proto_params = config_manager.getProtocolParams();
    if (msg.point_num > proto_params.max_traj_points)
    {
      RCLCPP_WARN(this->get_logger(), "TrajCmd point_num exceeds maximum: %u > %u",
                  msg.point_num, proto_params.max_traj_points);
      return false;
    }

    // 计算轨迹点所需的额外字节数
    size_t expected_traj_bytes = static_cast<size_t>(msg.point_num) * 12; // 每个 TrajPoint 是 3 个 int32，共 12 字节
    size_t expected_total_len = offset + expected_traj_bytes + 2;         // offset + 轨迹点字节 + checksum(2 字节)

    if (expected_total_len > len)
    {
      RCLCPP_WARN(this->get_logger(), "TrajCmd length mismatch:  at least %zu bytes but got %zu (point_num=%u)",
                  expected_total_len, len, msg.point_num);
      return false;
    }

    msg.points.clear();
    for (uint8_t i = 0; i < msg.point_num; i++)
    {
      udp_ros_bridge::msg::TrajPoint p;
      // p.lon = ntohl(*(int32_t *)(buf + offset));
      p.lon = (*(int32_t *)(buf + offset));
      offset += 4;
      // p.lat = ntohl(*(int32_t *)(buf + offset));
      p.lat = (*(int32_t *)(buf + offset));
      offset += 4;
      // p.alt = ntohl(*(int32_t *)(buf + offset));
      p.alt = (*(int32_t *)(buf + offset));
      offset += 4;
      msg.points.push_back(p);
    }

    ConfigManager &config_manager_ref = config_manager;
    // msg.check_sum = ntohs(*(uint16_t *)(buf + offset));
    msg.check_sum = (*(uint16_t *)(buf + offset));
    uint16_t calc_sum = calculateCheckSum(buf, len - config_manager_ref.getProtocolParams().check_sum_len);
    // return (msg.check_sum == calc_sum);
    return true;
  }

  bool UdpRosBridgeNode::parseRemoteCmd(const uint8_t *buf, size_t len, udp_ros_bridge::msg::RemoteCmd &msg)
  {
    // 根据最新输出，RemoteCmd数据包实际是25字节
    // 按照RemoteCmd消息定义，结构应为：
    // msg_id (4) + data_len (4) + timestamp (8) + mode (1) + throttle (1) + brake (1) + steer (1) + gear (1) + parking (1) + estop (1) + speed_limit (1) + heartbeat (1) = 26
    // 实际是25字节，可能没有data_len或timestamp是4字节，或者少了一个字段
    // 尝试：msg_id (4) + timestamp (8) + mode (1) + ... + heartbeat (1) = 22，仍然不够
    // 尝试：msg_id (4) + data_len (4) + timestamp (4) + mode (1) + ... + heartbeat (1) = 25，符合！

    if (len != 25)
    {
      RCLCPP_WARN(this->get_logger(), "Received RemoteCmd packet wrong size: %zu bytes, expected 25", len);
      return false;
    }

    size_t offset = 0;
    msg.msg_id = (*(uint32_t *)(buf + offset));
    offset += 4;

    // 跳过data_len字段（4字节）
    uint32_t reported_data_len = (*(uint32_t *)(buf + offset));
    offset += 4;

    // 使用8字节timestamp
    uint64_t timestamp_raw = (*(uint32_t *)(buf + offset));
    msg.timestamp = static_cast<uint64_t>(timestamp_raw) * 1000000000ULL; // 转换为纳秒
    // offset += 8;
    offset += ConfigManager::getInstance().getProtocolParams().time_stamp_len;

    msg.mode = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.throttle = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.brake = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.steer = *(int8_t *)(buf + offset);
    offset += 1;
    msg.gear = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.parking = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.estop = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.speed_limit = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.heartbeat = *(uint8_t *)(buf + offset);

    // 检查值的有效性
    if (msg.throttle > 100 || msg.brake > 100 || std::abs(msg.steer) > 100 || msg.speed_limit > 50)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid RemoteCmd values - throttle:%d, brake:%d, steer:%d, speed_limit:%d",
                  msg.throttle, msg.brake, msg.steer, msg.speed_limit);
      return false;
    }

    // std::cout << "data len: " << len << "; reported data_len: " << reported_data_len << "; msg_id: " << std::hex << std::uppercase << msg.msg_id
    // << std::dec << "; mode: " << (int)msg.mode << "; throttle: " << (int)msg.throttle
    // << "; brake: " << (int)msg.brake << "; steer: " << (int)msg.steer << std::endl;

    return true;
  }

  // 待修订20260326
  bool UdpRosBridgeNode::parseAutoCmd(const uint8_t *buf, size_t len, udp_ros_bridge::msg::AutoCmd &msg)
  {
    // 验证最小长度要求
    size_t min_expected_len = 4 + 8 + 1 + 1 + 2 + 2 + 2 + 1 + 1 + 2; // 所有字段的最小长度24

    if (len < min_expected_len)
    {
      RCLCPP_WARN(this->get_logger(), "Received AutoCmd packet too short: %zu bytes, expected at least %zu", len, min_expected_len);
      std::cout << "autocmd len < min_expected_len :" << len << " < " << min_expected_len << std::endl;
      return false;
    }
    std::cout << "autocmd len: " << len << std::endl;
    size_t offset = 0;
    // msg.msg_id = ntohl(*(uint32_t *)(buf + offset));
    msg.msg_id = (*(uint32_t *)(buf + offset));
    offset += 4;
    //  msg.data_len = ntohl(*(uint32_t *)(buf + offset)); offset +=4;
    msg.timestamp = parseTimestamp(buf + offset);
    offset += ConfigManager::getInstance().getProtocolParams().time_stamp_len;
    msg.source = *(uint8_t *)(buf + offset);
    offset += 1;

    msg.uc_auto_sys_state = *(uint8_t *)(buf + offset);
    offset += 1;
    // msg.us_auto_speed_limit = ntohs(*(uint16_t *)(buf + offset));
    msg.us_auto_speed_limit = (*(uint16_t *)(buf + offset));
    offset += 2;
    // msg.us_expect_speed = ntohs(*(int16_t *)(buf + offset));
    msg.us_expect_speed = (*(int16_t *)(buf + offset));
    offset += 2;
    // msg.us_expect_acc = ntohs(*(uint16_t *)(buf + offset));
    msg.us_expect_acc = (*(uint16_t *)(buf + offset));
    offset += 2;
    msg.uc_emerg_stop = *(uint8_t *)(buf + offset);
    offset += 1;
    msg.us_auto_sys_heart = *(uint8_t *)(buf + offset);
    offset += 1;
    // msg.us_checksum = ntohs(*(uint16_t *)(buf + offset));
    msg.us_checksum = (*(uint16_t *)(buf + offset));

    uint16_t calc_sum = calculateCheckSum(buf, len - ConfigManager::getInstance().getProtocolParams().check_sum_len);
    // std::cout << "data len: " << len << "; msg_id: " << std::hex << std::uppercase << msg.msg_id << "; recv_sum: " << msg.us_checksum << "; calc_sum: " << calc_sum << std::endl;

    // return (msg.us_checksum == calc_sum);
    return true;
  }

  bool UdpRosBridgeNode::packPlatformStatus(const udp_ros_bridge::msg::PlatformStatus &msg, uint8_t *buf, size_t &len)
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const ProtocolParams &proto_params = config_manager.getProtocolParams();
    const MessageIds &msg_ids = config_manager.getMessageIds();

    // Calculate actual required size instead of using fixed size
    size_t calculated_size = 0;
    calculated_size += 4;                           // msg_id (uint32_t)
    calculated_size += proto_params.time_stamp_len; // timestamp
    calculated_size += 1;                           // msg_from
    calculated_size += 13;                          // fixed fields: ctrl_mode, run_mode, estop_state, warn_level, platform_state, parking_state, + 1 byte fields
    calculated_size += 2;                           // power_max (uint16_t)
    calculated_size += 1;                           // speed_max
    calculated_size += 2;                           // speed_current (uint16_t)
    calculated_size += 12;                          // lon, lat, high (3 * int32)
    calculated_size += 2;                           // steer_angle (int16_t)
    calculated_size += 3;                           // steer_mode, faultcode, current
    calculated_size += 3;                           // brake pressure, status, fault level
    calculated_size += 4;                           // ucmfault fields
    calculated_size += 4;                           // ucmfaultcode fields
    calculated_size += 4;                           // ucmtemp fields

    if (len < calculated_size)
      return false;

    memset(buf, 0, calculated_size); // Only zero out the space we'll use
    size_t offset = 0;
    // *(uint32_t *)(buf + offset) = htonl(msg_ids.msg_id_status);
    *(uint32_t *)(buf + offset) = (msg_ids.msg_id_status);
    offset += 4;

    // 使用当前系统时间，精确到毫秒
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now().time_since_epoch())
                   .count();
    packTimestamp(now, buf + offset);
    offset += proto_params.time_stamp_len;

    *(uint8_t *)(buf + offset) = msg.msg_from;
    offset += 1;

    // 从CAN总线获取的实时状态覆盖传入的消息状态
    *(uint8_t *)(buf + offset) = can_vehicle_ctrl_mode_;
    offset += 1; // 控制模式
    *(uint8_t *)(buf + offset) = can_vehicle_run_mode_;
    offset += 1; // 运行模式
    *(uint8_t *)(buf + offset) = can_vehicle_estop_state_;
    offset += 1; // 急停状态
    *(uint8_t *)(buf + offset) = can_vehicle_warn_level_;
    offset += 1; // 报警等级
    *(uint8_t *)(buf + offset) = can_vehicle_platform_state_;
    offset += 1; // 平台状态
    *(uint8_t *)(buf + offset) = can_vehicle_park_state_;
    offset += 1; // 驻车状态
    // *(uint16_t *)(buf + offset) = htons(can_vehicle_power_max_);
    *(uint16_t *)(buf + offset) = (can_vehicle_power_max_);
    offset += 2; // 最大功率
    *(uint8_t *)(buf + offset) = msg.speed_max;
    offset += 1; // 速度上限（保持原值）
    // *(uint16_t *)(buf + offset) = htons(can_vehicle_speed_);
    *(uint16_t *)(buf + offset) = (can_vehicle_speed_);
    offset += 2; // 当前速度（来自CAN）
    // *(int32_t *)(buf + offset) = htonl(msg.current_lon);
    *(int32_t *)(buf + offset) = (msg.current_lon);
    offset += 4; // 经度（保持原值）
    // *(int32_t *)(buf + offset) = htonl(msg.current_lat);
    *(int32_t *)(buf + offset) = (msg.current_lat);
    offset += 4; // 纬度（保持原值）
    // *(int32_t *)(buf + offset) = htonl(msg.current_high);
    *(int32_t *)(buf + offset) = (msg.current_high);
    offset += 4; // 高度（保持原值）
    // *(int16_t *)(buf + offset) = htons(can_steering_angle_);
    *(int16_t *)(buf + offset) = (can_steering_angle_);
    offset += 2; // 转向角度（来自CAN）
    *(uint8_t *)(buf + offset) = can_steering_mode_;
    offset += 1; // 转向模式（来自CAN）
    *(uint8_t *)(buf + offset) = can_steering_fault_code_;
    offset += 1; // 转向故障码（来自CAN）
    *(uint8_t *)(buf + offset) = can_steering_current_;
    offset += 1; // 转向电流（来自CAN）
    *(uint8_t *)(buf + offset) = can_brake_pressure_;
    offset += 1; // 制动压力（来自CAN）
    *(uint8_t *)(buf + offset) = can_brake_status_;
    offset += 1; // 制动状态（来自CAN）
    *(uint8_t *)(buf + offset) = can_brake_fault_level_;
    offset += 1; // 制动故障等级（来自CAN）

    // UCM相关字段（来自CAN总线）
    *(uint8_t *)(buf + offset) = can_motor_fault_lf_;
    offset += 1; // 左前电机故障状态
    *(uint8_t *)(buf + offset) = can_motor_fault_lb_;
    offset += 1; // 左后电机故障状态
    *(uint8_t *)(buf + offset) = can_motor_fault_rf_;
    offset += 1; // 右前电机故障状态
    *(uint8_t *)(buf + offset) = can_motor_fault_rb_;
    offset += 1; // 右后电机故障状态

    *(uint8_t *)(buf + offset) = can_motor_fault_code_lf_;
    offset += 1; // 左前电机故障码
    *(uint8_t *)(buf + offset) = can_motor_fault_code_lb_;
    offset += 1; // 左后电机故障码
    *(uint8_t *)(buf + offset) = can_motor_fault_code_rf_;
    offset += 1; // 右前电机故障码
    *(uint8_t *)(buf + offset) = can_motor_fault_code_rb_;
    offset += 1; // 右后电机故障码

    *(uint8_t *)(buf + offset) = can_motor_temp_lf_;
    offset += 1; // 左前电机温度
    *(uint8_t *)(buf + offset) = can_motor_temp_lb_;
    offset += 1; // 左后电机温度
    *(uint8_t *)(buf + offset) = can_motor_temp_rf_;
    offset += 1; // 右前电机温度
    *(uint8_t *)(buf + offset) = can_motor_temp_rb_;
    offset += 1; // 右后电机温度

    // 现在我们知道了实际数据长度，可以计算校验和
    uint16_t check_sum = calculateCheckSum(buf, offset);
    *(uint16_t *)(buf + offset) = check_sum;
    offset += 2; // 校验和长度

    len = offset; // 返回实际使用的长度

    return true;
  }

  void UdpRosBridgeNode::platformStatusCb(const udp_ros_bridge::msg::PlatformStatus::SharedPtr msg)
  {
    ConfigManager &config_manager = ConfigManager::getInstance();
    const NetworkConfig &net_config = config_manager.getNetworkConfig();

    // 使用vector替代变长数组，大小刚好够用
    std::vector<uint8_t> send_buf(net_config.udp_buf_size);
    size_t send_len = net_config.udp_buf_size;
    if (packPlatformStatus(*msg, send_buf.data(), send_len))
    {
      // send_len现在包含实际的数据长度
      ssize_t result = sendto(sock_send_, send_buf.data(), send_len, 0, (sockaddr *)&remote_addr_, sizeof(remote_addr_));
      if (result < 0)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to send platform status via UDP: %s", strerror(errno));
        udp_send_failures_++;
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "Successfully sent %ld bytes via UDP", send_len);
      }
    }
  }

} // namespace udp_ros_bridge

int main(int argc, char *argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // 创建节点实例
  auto node = std::make_shared<udp_ros_bridge::UdpRosBridgeNode>();

  // 运行节点（阻塞，直到Ctrl+C退出）
  rclcpp::spin(node);

  // 退出ROS2
  rclcpp::shutdown();
  return 0;
}