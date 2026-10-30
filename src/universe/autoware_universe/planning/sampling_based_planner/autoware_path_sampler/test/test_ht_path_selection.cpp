#include "autoware_path_sampler/node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
namespace ps = autoware::path_sampler;
namespace sc = autoware::sampler_common;
class TestSampler : public ps::PathSampler
{
public:
  using PathSampler::generatePath;
  using PathSampler::ht_check_distance_;
  using PathSampler::ht_snapshot_;
  using PathSampler::htPathClear;
  using PathSampler::PathSampler;
  using PathSampler::prev_path_;
};
class HtPathSelection : public ::testing::Test
{
protected:
  std::shared_ptr<TestSampler> node;
  ps::PlannerData data;
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid;
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    const std::string root = SCOUT_WS_ROOT;
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file",
       root + "/src/vehicle/scout_vehicle_description/config/vehicle_info.param.yaml",
       "--params-file",
       root + "/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/"
              "lane_driving/motion_planning/path_sampler/path_sampler.param.yaml",
       "--params-file",
       root + "/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/"
              "common/nearest_search.param.yaml"});
    options.parameter_overrides({rclcpp::Parameter("scout_ht.enabled", true)});
    node = std::make_shared<TestSampler>(options);
    grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = "map";
    grid->info.resolution = .1;
    grid->info.width = 400;
    grid->info.height = 100;
    grid->info.origin.position.x = -5;
    grid->info.origin.position.y = -5;
    grid->info.origin.orientation.w = 1;
    grid->data.assign(40000, 0);
    node->ht_snapshot_ = {grid, "READY"};
    node->ht_check_distance_ = 12;
    data.header.frame_id = "map";
    data.ego_pose.orientation.w = 1;
    data.ego_vel = 1;
    for (int i = -2; i <= 25; ++i) {
      ps::TrajectoryPoint p;
      p.pose.position.x = i;
      p.pose.orientation.w = 1;
      p.longitudinal_velocity_mps = 1;
      data.traj_points.push_back(p);
      geometry_msgs::msg::Point bound;
      bound.x = i;
      bound.y = 2.5;
      data.left_bound.push_back(bound);
      bound.y = -2.5;
      data.right_bound.push_back(bound);
    }
  }
  void black(double x, double y)
  {
    const int ix = static_cast<int>((x + 5) / grid->info.resolution);
    const int iy = static_cast<int>((y + 5) / grid->info.resolution);
    grid->data.at(iy * grid->info.width + ix) = 100;
  }
  sc::Path straight()
  {
    sc::Path p;
    for (int i = 0; i <= 10; ++i) {
      p.points.emplace_back(i, 0);
      p.yaws.push_back(0);
      p.curvatures.push_back(0);
      p.lengths.push_back(i);
    }
    return p;
  }
  sc::Path solve()
  {
    auto pub = node->create_publisher<ps::PredictedObjects>("/path_sampler/input/objects", 1);
    for (int i = 0; i < 8; ++i) {
      ps::PredictedObjects objects;
      objects.header.frame_id = "map";
      objects.header.stamp = node->now();
      pub->publish(objects);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(20ms);
    }
    return node->generatePath(data);
  }
};
TEST_F(HtPathSelection, RejectsBlackCenterPathButAllowsWhiteLateralCandidate)
{
  black(3, 0);
  EXPECT_FALSE(node->htPathClear(straight(), data));
  auto detour = straight();
  for (size_t i = 1; i < detour.points.size(); ++i) detour.points[i].y() = 1.5;
  EXPECT_TRUE(node->htPathClear(detour, data));
}
TEST_F(HtPathSelection, CandidateSolveFindsAWhitePathInsteadOfTheBlockedReference)
{
  black(8, 0);
  ASSERT_FALSE(node->htPathClear(straight(), data));
  const auto path = solve();
  ASSERT_FALSE(path.points.empty());
  EXPECT_TRUE(node->htPathClear(path, data));
  double max_y = 0;
  for (const auto & p : path.points) max_y = std::max(max_y, std::abs(p.y()));
  EXPECT_GT(max_y, .4);
}
TEST_F(HtPathSelection, ANewBlackMapCannotReuseTheOldWhitePath)
{
  ASSERT_FALSE(solve().points.empty());
  std::fill(grid->data.begin(), grid->data.end(), 100);
  EXPECT_TRUE(solve().points.empty());
}
TEST_F(HtPathSelection, MissingMapCannotReturnAnOldFallbackPath)
{
  node->prev_path_ = straight();
  node->ht_snapshot_ = {nullptr, "MISSING"};
  EXPECT_TRUE(solve().points.empty());
}
TEST_F(HtPathSelection, SnapshotIsFixedWhileANewGridIsPrepared)
{
  EXPECT_TRUE(node->htPathClear(straight(), data));
  auto next = std::make_shared<nav_msgs::msg::OccupancyGrid>(*grid);
  std::fill(next->data.begin(), next->data.end(), 100);
  EXPECT_TRUE(node->htPathClear(straight(), data));
  node->ht_snapshot_ = {next, "READY"};
  EXPECT_FALSE(node->htPathClear(straight(), data));
}

TEST_F(HtPathSelection, NewLaneBoundsCannotReuseAnOldPathOutsideTheLane)
{
  ASSERT_FALSE(solve().points.empty());
  for (auto & p : data.left_bound) p.y = .1;
  for (auto & p : data.right_bound) p.y = -.1;
  EXPECT_TRUE(solve().points.empty());
}
