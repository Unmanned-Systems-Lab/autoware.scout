#include "scout_ht_planning/velocity_module.hpp"
#include "test_grid.hpp"

#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
namespace mvp = autoware::motion_velocity_planner;
class VelocityTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<mvp::PlannerData> data;
  scout_ht_planning::VelocityModule module;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    const std::string root = SCOUT_WS_ROOT;
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file",
       root + "/src/vehicle/scout_vehicle_description/config/vehicle_info.param.yaml",
       "--params-file",
       root +
         "/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/"
         "lane_driving/motion_planning/motion_velocity_planner/motion_velocity_planner.param.yaml",
       "--params-file",
       root + "/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/"
              "common/nearest_search.param.yaml"});
    node = std::make_shared<rclcpp::Node>("ht_velocity_test", options);
    data = std::make_shared<mvp::PlannerData>(*node);
    data->ego_nearest_dist_threshold = 3;
    data->ego_nearest_yaw_threshold = 1.57;
    data->vehicle_info_.max_longitudinal_offset_m = .5;
    data->vehicle_info_.min_longitudinal_offset_m = -.5;
    data->vehicle_info_.max_lateral_offset_m = .3;
    data->vehicle_info_.min_lateral_offset_m = -.3;
    module.init(*node, "scout_ht_planning::VelocityModule");
    publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/scout/traversability/fused_costmap", 1);
    for (int x = -2; x <= 8; ++x) {
      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = x;
      p.pose.orientation.w = 1;
      p.longitudinal_velocity_mps = 1;
      points.push_back(p);
    }
    data->current_odometry.header.frame_id = "map";
    data->current_odometry.child_frame_id = "base_link";
    data->current_odometry.pose.pose.orientation.w = 1;
    data->current_odometry.twist.twist.linear.x = 1;
  }
  void send(nav_msgs::msg::OccupancyGrid grid)
  {
    for (int i = 0; i < 8; ++i) {
      grid.header.stamp = node->now();
      publisher->publish(grid);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(20ms);
    }
  }
  mvp::VelocityPlanningResult plan()
  {
    data->current_odometry.header.stamp = node->now();
    return module.plan(points, points, data);
  }
};
TEST_F(VelocityTest, MissingMapActuallyZerosThePlannerTrajectory)
{
  auto output = points;
  const auto result = plan();
  ASSERT_EQ(result.stop_points.size(), 1u);
  mvp::utils::apply_planning_result(output, result, node->get_logger());
  for (const auto & p : output)
    if (p.pose.position.x >= 0) {
      EXPECT_EQ(p.longitudinal_velocity_mps, 0);
    }
}
TEST_F(VelocityTest, WhiteMapLeavesExistingSpeedsUntouched)
{
  send(free_grid());
  EXPECT_TRUE(plan().stop_points.empty());
}
TEST_F(VelocityTest, BlackCellCreatesStopBeforeBodyContactAndCapsApproachSpeed)
{
  auto grid = free_grid();
  obstacle(grid, 4, 0);
  send(grid);
  auto result = plan();
  ASSERT_EQ(result.stop_points.size(), 1u);
  EXPECT_GT(result.stop_points[0].x, 2);
  EXPECT_LT(result.stop_points[0].x, 3.1);
  EXPECT_FALSE(result.slowdown_intervals.empty());
  auto output = points;
  mvp::utils::apply_planning_result(output, result, node->get_logger());
  for (const auto & p : output) {
    EXPECT_LE(p.longitudinal_velocity_mps, 1);
    if (p.pose.position.x >= 3.2) {
      EXPECT_EQ(p.longitudinal_velocity_mps, 0);
    }
  }
}
TEST_F(VelocityTest, MapExpiryCannotFallBackToTheOldWhiteMap)
{
  send(free_grid());
  EXPECT_TRUE(plan().stop_points.empty());
  std::this_thread::sleep_for(550ms);
  EXPECT_EQ(plan().stop_points.size(), 1u);
}
TEST_F(VelocityTest, FreshMapCanRecoverWithoutRestart)
{
  auto grid = free_grid();
  obstacle(grid, 0, 0);
  send(grid);
  EXPECT_EQ(plan().stop_points.size(), 1u);
  send(free_grid());
  EXPECT_TRUE(plan().stop_points.empty());
}
TEST_F(VelocityTest, WrongFrameAndInvalidNewMapRevokePriorWhiteResult)
{
  auto grid = free_grid();
  grid.header.frame_id = "odom";
  send(grid);
  EXPECT_EQ(plan().stop_points.size(), 1u);
  grid = free_grid();
  grid.data.clear();
  send(grid);
  EXPECT_EQ(plan().stop_points.size(), 1u);
}
TEST_F(VelocityTest, ReverseIsHeldRatherThanSilentlySkippingTheGuard)
{
  send(free_grid());
  data->is_driving_forward = false;
  EXPECT_EQ(plan().stop_points.size(), 1u);
}

TEST_F(VelocityTest, NativePlannerCanDiscoverAndInstantiateThePlugin)
{
  pluginlib::ClassLoader<mvp::PluginModuleInterface> loader(
    "autoware_motion_velocity_planner", "autoware::motion_velocity_planner::PluginModuleInterface");
  ASSERT_TRUE(loader.isClassAvailable("scout_ht_planning::VelocityModule"));
  EXPECT_NE(loader.createSharedInstance("scout_ht_planning::VelocityModule"), nullptr);
}

TEST_F(VelocityTest, FutureStampedMapCannotAuthorizeExecution)
{
  auto grid = free_grid();
  for (int i = 0; i < 8; ++i) {
    grid.header.stamp = node->now() + rclcpp::Duration::from_seconds(3);
    publisher->publish(grid);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(plan().stop_points.size(), 1u);
}

TEST_F(VelocityTest, AnExistingStopAndLowerVelocityAreNeverRaised)
{
  send(free_grid());
  points[3].longitudinal_velocity_mps = 0;
  points[4].longitudinal_velocity_mps = .1;
  auto output = points;
  mvp::utils::apply_planning_result(output, plan(), node->get_logger());
  for (size_t i = 0; i < points.size(); ++i)
    EXPECT_LE(output[i].longitudinal_velocity_mps, points[i].longitudinal_velocity_mps);
}
