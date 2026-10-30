#ifndef SCOUT_HT_PLANNING__COSTMAP_HPP_
#define SCOUT_HT_PLANNING__COSTMAP_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <limits>
#include <string>
#include <vector>

namespace scout_ht_planning
{
struct Pose2
{
  double x{}, y{}, yaw{};
};
struct Footprint
{
  double front{}, rear{}, left{}, right{};  // Signed base_link offsets.
};
struct Collision
{
  bool blocked{true};
  double distance{};  // Conservative free arc length from the first pose.
  std::string reason{"INVALID_INPUT"};
};
Pose2 planar_pose(const geometry_msgs::msg::Pose & pose);
bool valid_grid(const nav_msgs::msg::OccupancyGrid & grid);
Collision check_path(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<Pose2> & path,
  const Footprint & footprint, double max_distance = std::numeric_limits<double>::infinity());
double safe_speed(double remaining_distance, double deceleration, double latency);
}  // namespace scout_ht_planning
#endif
