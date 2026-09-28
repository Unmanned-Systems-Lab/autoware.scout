#include "ez21_vehicle_launch/ez21_vehicle_interface_node.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ez21_vehicle_launch::Ez21VehicleInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
