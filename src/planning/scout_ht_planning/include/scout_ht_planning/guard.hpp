#ifndef SCOUT_HT_PLANNING__GUARD_HPP_
#define SCOUT_HT_PLANNING__GUARD_HPP_

#include "scout_ht_planning/costmap.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

namespace scout_ht_planning
{
struct Snapshot
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  std::string reason;
  std::chrono::steady_clock::time_point receipt{};
};
class CostmapGuard
{
public:
  explicit CostmapGuard(rclcpp::Node & node);
  Snapshot snapshot(const std::string & frame);
  bool current(const Snapshot & snapshot);
  double stop_margin{}, deceleration{}, latency{}, minimum_lookahead{};

private:
  rclcpp::Node & node_;
  std::mutex mutex_;
  double timeout_{};
  double epoch_{};
  double last_now_{};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  std::optional<autoware_map_msgs::msg::MapProjectorInfo> projector_;
  std::chrono::steady_clock::time_point receipt_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr projector_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace scout_ht_planning
#endif
