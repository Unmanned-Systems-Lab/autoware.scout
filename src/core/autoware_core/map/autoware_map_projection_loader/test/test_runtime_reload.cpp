#include "autoware/map_projection_loader/map_projection_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using namespace std::chrono_literals;

TEST(MapProjectionLoader, ReloadsProjectorWhenPathChanges)
{
  rclcpp::init(0, nullptr);
  const auto map_dir =
    std::filesystem::temp_directory_path() / "autoware_map_projection_reload_test";
  std::filesystem::create_directories(map_dir);
  const auto original = map_dir / "original.yaml";
  const auto updated = map_dir / "updated.yaml";
  {
    std::ofstream file(original);
    file << "projector_type: Local\nmap_origin:\n  latitude: 29.6\n"
            "  longitude: 106.28\n  altitude: 255.0\n";
  }
  {
    std::ofstream file(updated);
    file << "projector_type: LocalCartesian\nvertical_datum: WGS84\n"
            "map_origin:\n  latitude: 29.6001\n"
            "  longitude: 106.28\n  altitude: 255.0\n";
  }

  const auto options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("map_projector_info_path", original.string()),
     rclcpp::Parameter("lanelet2_map_path", "")});
  auto loader = std::make_shared<autoware::map_projection_loader::MapProjectionLoader>(options);
  EXPECT_TRUE(loader->get_parameter("supports_runtime_reload").as_bool());
  const auto changed =
    loader->set_parameter(rclcpp::Parameter("map_projector_info_path", updated.string()));
  ASSERT_TRUE(changed.successful) << changed.reason;
  EXPECT_EQ(loader->get_parameter("map_projector_info_path").as_string(), updated.string());

  auto observer = std::make_shared<rclcpp::Node>("map_projection_reload_test");
  autoware_map_msgs::msg::MapProjectorInfo::SharedPtr received;
  auto subscription = observer->create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
    "/map/map_projector_info", rclcpp::QoS(1).reliable().transient_local(),
    [&received](autoware_map_msgs::msg::MapProjectorInfo::SharedPtr message) {
      received = message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }
  ASSERT_NE(received, nullptr);
  EXPECT_DOUBLE_EQ(received->map_origin.latitude, 29.6001);

  const auto rejected = loader->set_parameter(
    rclcpp::Parameter("map_projector_info_path", (map_dir / "missing.yaml").string()));
  EXPECT_FALSE(rejected.successful);
  EXPECT_EQ(loader->get_parameter("map_projector_info_path").as_string(), updated.string());
  executor.remove_node(observer);
  std::filesystem::remove_all(map_dir);
  rclcpp::shutdown();
}
