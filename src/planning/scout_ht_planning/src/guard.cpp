#include "scout_ht_planning/guard.hpp"

#include <cmath>
#include <stdexcept>

namespace scout_ht_planning
{
namespace
{
template <typename T>
T param(rclcpp::Node & node, const std::string & name, const T & value)
{
  return node.has_parameter(name) ? node.get_parameter(name).get_value<T>()
                                  : node.declare_parameter<T>(name, value);
}
}  // namespace
CostmapGuard::CostmapGuard(rclcpp::Node & node) : node_(node)
{
  const auto topic =
    param<std::string>(node, "scout_ht.costmap_topic", "/scout/traversability/fused_costmap");
  timeout_ = param<double>(node, "scout_ht.map_timeout", 0.5);
  stop_margin = param<double>(node, "scout_ht.stop_margin", 0.5);
  deceleration = param<double>(node, "scout_ht.deceleration", 0.5);
  latency = param<double>(node, "scout_ht.latency", 0.3);
  minimum_lookahead = param<double>(node, "scout_ht.minimum_lookahead", 2.0);
  if (
    topic.empty() || topic.front() != '/' || !std::isfinite(timeout_) || timeout_ <= 0 ||
    !std::isfinite(stop_margin) || stop_margin < 0 || !std::isfinite(deceleration) ||
    deceleration <= 0 || !std::isfinite(latency) || latency < 0 ||
    !std::isfinite(minimum_lookahead) || minimum_lookahead <= 0)
    throw std::invalid_argument("Invalid Scout HT planning parameters");
  sub_ = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      grid_ = valid_grid(*msg) ? msg : nullptr;
      receipt_ = std::chrono::steady_clock::now();
    });
  projector_sub_ = node.create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
    "/map/map_projector_info", rclcpp::QoS(1).transient_local(),
    [this](autoware_map_msgs::msg::MapProjectorInfo::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (projector_ && *projector_ != *msg) {
        grid_.reset();
        epoch_ = node_.now().seconds();
      }
      projector_ = *msg;
    });
  parameter_callback_ =
    node.add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        if (p.get_name().rfind("scout_ht.", 0) == 0) {
          result.successful = false;
          result.reason = "Scout HT planning parameters are startup-only";
        }
      }
      return result;
    });
}
Snapshot CostmapGuard::snapshot(const std::string & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = node_.now().seconds();
  if (now < last_now_) {
    grid_.reset();
    epoch_ = now;
  }
  last_now_ = now;
  if (!grid_) return {nullptr, "MISSING_OR_INVALID_MAP"};
  const double stamp = rclcpp::Time(grid_->header.stamp).seconds();
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - receipt_).count();
  if (
    stamp <= 0 || stamp < epoch_ || stamp > now + 0.05 || now - stamp > timeout_ || age > timeout_)
    return {nullptr, "STALE_MAP"};
  if (frame != "map" || grid_->header.frame_id != frame) return {nullptr, "FRAME_MISMATCH"};
  return {grid_, "READY", receipt_};
}
bool CostmapGuard::current(const Snapshot & snapshot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!snapshot.grid || !grid_) return false;
  const double now = node_.now().seconds();
  const double stamp = rclcpp::Time(snapshot.grid->header.stamp).seconds();
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - snapshot.receipt).count();
  return now >= last_now_ && stamp >= epoch_ && stamp <= now + 0.05 && now - stamp <= timeout_ &&
         age <= timeout_;
}
}  // namespace scout_ht_planning
