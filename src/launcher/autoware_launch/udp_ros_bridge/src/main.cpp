#include "udp_ros_bridge/udp_bridge_node.hpp"

int main(int argc, char * argv[])
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