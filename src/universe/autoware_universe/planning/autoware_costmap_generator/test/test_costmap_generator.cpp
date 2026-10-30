// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/costmap_generator/costmap_generator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::costmap_generator::CostmapGenerator;
using autoware_internal_planning_msgs::msg::Scenario;

class TestCostmapGenerator : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    set_up_node();
    setup_lanelet_map();
  }

  void set_up_node()
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto costmap_generator_dir =
      ament_index_cpp::get_package_share_directory("autoware_costmap_generator");
    node_options.arguments(
      {"--ros-args", "--params-file",
       costmap_generator_dir + "/config/costmap_generator.param.yaml"});
    costmap_generator_ = std::make_shared<CostmapGenerator>(node_options);
  }

  void setup_lanelet_map()
  {
    const auto lanelet2_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "overlap_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet2_path, 5.0);
    lanelet_map_ = lanelet_map_ = autoware::experimental::lanelet2_utils::remove_const(
      autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin_msg));
  }

  [[nodiscard]] double get_grid_length_x() { return costmap_generator_->costmap_.getLength()[0]; }

  [[nodiscard]] double get_grid_length_y() { return costmap_generator_->costmap_.getLength()[1]; }

  [[nodiscard]] double get_grid_resolution()
  {
    return costmap_generator_->costmap_.getResolution();
  }

  [[nodiscard]] std::vector<std::string> get_grid_layers()
  {
    return costmap_generator_->costmap_.getLayers();
  }

  size_t test_load_road_areas()
  {
    std::vector<geometry_msgs::msg::Polygon> road_areas;
    costmap_generator_->loadRoadAreasFromLaneletMap(lanelet_map_, road_areas);
    return road_areas.size();
  }

  size_t test_load_parking_areas()
  {
    std::vector<geometry_msgs::msg::Polygon> parking_areas;
    costmap_generator_->loadParkingAreasFromLaneletMap(lanelet_map_, parking_areas);
    return parking_areas.size();
  }

  bool test_is_active_by_pos(
    const bool is_within_parking_lot = true, const bool no_lanelet_map = false)
  {
    costmap_generator_->lanelet_map_.reset();
    if (no_lanelet_map) {
      return costmap_generator_->isActive();
    }

    costmap_generator_->lanelet_map_ = lanelet_map_;
    costmap_generator_->param_->activate_by_scenario = false;

    geometry_msgs::msg::PoseStamped::SharedPtr p(new geometry_msgs::msg::PoseStamped());
    p->pose.position.x = 3697.07;
    p->pose.position.y = 73735.49;
    if (!is_within_parking_lot) {
      p->pose.position.x += 10.0;
    }

    costmap_generator_->current_pose_ = p;
    return costmap_generator_->isActive();
  }

  bool test_is_active_by_scenario(
    const bool is_parking_scenario = true, const bool no_lanelet_map = false)
  {
    costmap_generator_->lanelet_map_.reset();
    if (no_lanelet_map) {
      return costmap_generator_->isActive();
    }

    costmap_generator_->lanelet_map_ = lanelet_map_;
    costmap_generator_->param_->activate_by_scenario = true;

    Scenario scenario;
    scenario.current_scenario = is_parking_scenario ? Scenario::PARKING : Scenario::LANEDRIVING;
    if (is_parking_scenario) {
      scenario.activating_scenarios.push_back(Scenario::PARKING);
    }

    costmap_generator_->scenario_ = std::make_shared<Scenario>(scenario);
    return costmap_generator_->isActive();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    costmap_generator_ = nullptr;
    lanelet_map_ = nullptr;
  }

  bool lane_driving_active(bool enabled, bool in_activation_list)
  {
    costmap_generator_->lanelet_map_ = lanelet_map_;
    costmap_generator_->param_->activate_by_scenario = true;
    costmap_generator_->param_->activate_in_lane_driving = enabled;
    auto scenario = std::make_shared<Scenario>();
    if (in_activation_list)
      scenario->activating_scenarios.push_back(Scenario::LANEDRIVING);
    else
      scenario->current_scenario = Scenario::LANEDRIVING;
    costmap_generator_->scenario_ = scenario;
    return costmap_generator_->isActive();
  }

  void check_input_freshness()
  {
    auto & node = *costmap_generator_;
    node.param_->use_points = true;
    node.param_->use_objects = false;
    node.param_->input_timeout = .5;
    EXPECT_FALSE(node.inputsFresh());
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    points->header.frame_id = "map";
    points->header.stamp = node.now();
    node.points_ = points;
    EXPECT_TRUE(node.inputsFresh());
    points->header.stamp = node.now() - rclcpp::Duration::from_seconds(1);
    EXPECT_FALSE(node.inputsFresh());
    points->header.stamp = node.now() + rclcpp::Duration::from_seconds(1);
    EXPECT_FALSE(node.inputsFresh());
    points->header.stamp.sec = -1;
    EXPECT_FALSE(node.inputsFresh());
  }

  void check_missing_tf()
  {
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    points->header.frame_id = "missing_sensor";
    points->header.stamp = costmap_generator_->now();
    costmap_generator_->param_->activate_in_lane_driving = true;
    EXPECT_THROW(costmap_generator_->generatePointsCostmap(points, 0), tf2::TransformException);
  }

  void check_published_source_stamp()
  {
    auto & node = *costmap_generator_;
    node.param_->activate_by_scenario = true;
    node.param_->activate_in_lane_driving = true;
    node.param_->use_points = true;
    node.param_->use_objects = false;
    auto peer = std::make_shared<rclcpp::Node>("costmap_stamp_peer");
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr received;
    auto sub = peer->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap_generator/output/occupancy_grid", 1,
      [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { received = msg; });
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    points->header.frame_id = "map";
    points->header.stamp = node.now() - rclcpp::Duration::from_seconds(.2);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node.now();
    tf.transform.rotation.w = 1;
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(peer);
    exec.add_node(costmap_generator_);
    for (int i = 0; i < 12 && !received; ++i) {
      node.points_ = points;
      node.publishCostmap(node.costmap_, tf);
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->header.stamp, points->header.stamp);
  }

  bool continuous_active(bool enabled, bool has_map)
  {
    auto & node = *costmap_generator_;
    node.param_->publish_continuously = enabled;
    node.param_->activate_by_scenario = true;
    node.scenario_.reset();
    node.lanelet_map_ = has_map ? lanelet_map_ : nullptr;
    return node.isActive();
  }

  void check_timer_early_return(bool missing_tf, bool without_scenario = false)
  {
    auto & node = *costmap_generator_;
    node.timer_->cancel();
    node.lanelet_map_ = lanelet_map_;
    node.param_->activate_by_scenario = true;
    node.param_->activate_in_lane_driving = true;
    node.param_->use_points = true;
    node.param_->use_objects = false;
    node.param_->publish_continuously = without_scenario;
    if (!without_scenario) {
      auto publisher = node.create_publisher<Scenario>("~/input/scenario", 1);
      Scenario scenario;
      scenario.current_scenario = Scenario::LANEDRIVING;
      for (int i = 0; i < 50 && !node.scenario_; ++i) {
        publisher->publish(scenario);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        node.update_data();
      }
      ASSERT_NE(node.scenario_, nullptr);
    } else {
      ASSERT_FALSE(node.scenario_);
      ASSERT_TRUE(node.strictInputTime());
    }
    ASSERT_TRUE(node.isActive());
    if (!missing_tf) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = node.param_->costmap_frame;
      tf.child_frame_id = node.param_->vehicle_frame;
      tf.header.stamp = node.now();
      tf.transform.rotation.w = 1;
      ASSERT_TRUE(node.tf_buffer_.setTransform(tf, "test", true));
    }
    // No sensor points: the successful TF path must hit the freshness hold.
    // Both branches previously left lookupTransform open and terminated in the
    // outer ScopedTimeTrack destructor. Repeat to check later timer invocations too.
    ASSERT_FALSE(node.points_);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NO_THROW(node.onTimer());
    }
  }

  std::shared_ptr<CostmapGenerator> costmap_generator_;
  lanelet::LaneletMapPtr lanelet_map_;
};

TEST_F(TestCostmapGenerator, testInitializeGridmap)
{
  const double grid_resolution = get_grid_resolution();
  const double grid_length_x = get_grid_length_x();
  const double grid_length_y = get_grid_length_y();
  EXPECT_FLOAT_EQ(grid_resolution, 0.3);
  EXPECT_TRUE(70.0 - grid_length_x < grid_resolution);
  EXPECT_TRUE(70.0 - grid_length_y < grid_resolution);

  const auto grid_layers = get_grid_layers();

  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "points") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "objects") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "primitives") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "combined") != grid_layers.end());
}

TEST_F(TestCostmapGenerator, testLoadAreasFromLaneletMap)
{
  EXPECT_EQ(test_load_road_areas(), 12ul);
  EXPECT_EQ(test_load_parking_areas(), 10ul);
}

TEST_F(TestCostmapGenerator, testIsActive)
{
  EXPECT_TRUE(test_is_active_by_pos());
  EXPECT_FALSE(test_is_active_by_pos(false));
  EXPECT_FALSE(test_is_active_by_pos(true, true));

  EXPECT_TRUE(test_is_active_by_scenario());
  EXPECT_FALSE(test_is_active_by_scenario(false));
  EXPECT_FALSE(test_is_active_by_scenario(true, true));
}

TEST_F(TestCostmapGenerator, LaneDrivingPublicationIsExplicitAndKeepsLegacyDefaults)
{
  EXPECT_FALSE(lane_driving_active(false, false));
  EXPECT_FALSE(lane_driving_active(false, true));
  EXPECT_TRUE(lane_driving_active(true, false));
  EXPECT_TRUE(lane_driving_active(true, true));
}

TEST_F(TestCostmapGenerator, FreshInputsRequiredInLaneDrivingMode)
{
  check_input_freshness();
}
TEST_F(TestCostmapGenerator, MissingSensorTfCannotPublishAWhiteMap)
{
  check_missing_tf();
}
TEST_F(TestCostmapGenerator, OutputTimestampIsNotNewerThanSensorData)
{
  check_published_source_stamp();
}

TEST_F(TestCostmapGenerator, MissingTfDoesNotTerminateTimerOrLaterUpdates)
{
  check_timer_early_return(true);
}

TEST_F(TestCostmapGenerator, MissingSensorInputDoesNotTerminateTimerOrLaterUpdates)
{
  check_timer_early_return(false);
}

TEST_F(TestCostmapGenerator, ContinuousPublicationNeedsMapButNotScenarioAndIsOptIn)
{
  EXPECT_FALSE(continuous_active(false, true));
  EXPECT_FALSE(continuous_active(true, false));
  EXPECT_TRUE(continuous_active(true, true));
}

TEST_F(TestCostmapGenerator, ContinuousPublicationStillRejectsMissingRealSensorInputs)
{
  check_timer_early_return(false, true);
}
